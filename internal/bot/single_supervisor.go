package bot

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"math/rand"
	"strings"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/config"
	ct "github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/health"
	"github.com/hectorgimenez/koolo/internal/run"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// Define a constant for the timeout on menu operations
const menuActionTimeout = 30 * time.Second

// Define constants for the in-game activity monitor
const (
	activityCheckInterval = 15 * time.Second
	maxStuckDuration      = 3 * time.Minute
)

type SinglePlayerSupervisor struct {
	*baseSupervisor
}

func (s *SinglePlayerSupervisor) GetData() *game.Data {
	return s.bot.ctx.Data
}

func (s *SinglePlayerSupervisor) GetContext() *ct.Context {
	return s.bot.ctx
}

func NewSinglePlayerSupervisor(name string, bot *Bot, statsHandler *StatsHandler) (*SinglePlayerSupervisor, error) {
	bs, err := newBaseSupervisor(bot, name, statsHandler)
	if err != nil {
		return nil, err
	}

	return &SinglePlayerSupervisor{
		baseSupervisor: bs,
	}, nil
}

var ErrUnrecoverableClientState = errors.New("unrecoverable client state, forcing restart")

func (s *SinglePlayerSupervisor) orderRuns(runs []string) []string {

	if s.bot.ctx.CharacterCfg.Game.Difficulty == "Nightmare" {

		s.bot.ctx.Logger.Info("Changing difficulty to Nightmare")

		s.changeDifficulty(difficulty.Nightmare)

	}

	if s.bot.ctx.CharacterCfg.Game.Difficulty == "Hell" {

		s.bot.ctx.Logger.Info("Changing difficulty to Hell")

		s.changeDifficulty(difficulty.Hell)

	}

	lvl, _ := s.bot.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)

	if s.bot.ctx.CharacterCfg.Game.StopLevelingAt > 0 && lvl.Value >= s.bot.ctx.CharacterCfg.Game.StopLevelingAt {

		s.bot.ctx.Logger.Info("Character level is already high enough, stopping.")

		s.Stop()

		return nil

	}

	return runs

}

func (s *SinglePlayerSupervisor) changeDifficulty(d difficulty.Difficulty) {

	s.bot.ctx.GameReader.GetSelectedCharacterName()

	s.bot.ctx.HID.Click(game.LeftButton, 6, 6)

	utils.Sleep(1000)

	switch d {

	case difficulty.Normal:

		s.bot.ctx.HID.Click(game.LeftButton, 400, 350)

	case difficulty.Nightmare:

		s.bot.ctx.HID.Click(game.LeftButton, 400, 400)

	case difficulty.Hell:

		s.bot.ctx.HID.Click(game.LeftButton, 400, 450)

	}

	utils.Sleep(1000)

	s.bot.ctx.HID.Click(game.LeftButton, 6, 6)

	utils.Sleep(1000)

}

// Start will return error if it can be started, otherwise will always return nil
func (s *SinglePlayerSupervisor) Start() error {
	ctx, cancel := context.WithCancel(context.Background())
	s.cancelFn = cancel

	err := s.ensureProcessIsRunningAndPrepare()
	if err != nil {
		return fmt.Errorf("error preparing game: %w", err)
	}

	firstRun := true
	var timeSpentNotInGameStart = time.Now()
	const maxTimeNotInGame = 3 * time.Minute

	for {
		// Check if the main context has been cancelled
		select {
		case <-ctx.Done():
			return nil
		default:
		}

		if firstRun {
			if err = s.waitUntilCharacterSelectionScreen(); err != nil {
				return fmt.Errorf("error waiting for character selection screen: %w", err)
			}
		}

		// LOGIC OUTSIDE OF GAME (MENUS)
		if !s.bot.ctx.Manager.InGame() {
			// This outer timer is the ultimate watchdog. If the bot is out of game for too long,
			// for any reason (including a frozen state read), this will trigger.
			if time.Since(timeSpentNotInGameStart) > maxTimeNotInGame {
				s.bot.ctx.Logger.Error(fmt.Sprintf("Bot has been outside of a game for more than %s. Forcing client restart.", maxTimeNotInGame))
				if killErr := s.KillClient(); killErr != nil {
					s.bot.ctx.Logger.Error(fmt.Sprintf("Error killing client after timeout: %s", killErr.Error()))
				}
				return ErrUnrecoverableClientState
			}

			// We execute the menu handling in a goroutine so we can timeout the whole process
			// if it gets stuck reading game state.
			errChan := make(chan error, 1)
			go func() {
				errChan <- s.HandleMenuFlow()
			}()

			select {
			case err := <-errChan:
				// Menu flow finished (or returned an error) before the timeout.
				if err != nil {
					if errors.Is(err, ErrUnrecoverableClientState) {
						s.bot.ctx.Logger.Error(fmt.Sprintf("Unrecoverable client state detected: %s. Forcing client restart.", err.Error()))
						return err
					}
					if err.Error() == "loading screen" || err.Error() == "" || err.Error() == "idle" {
						utils.Sleep(100)
						continue
					}
					s.bot.ctx.Logger.Error(fmt.Sprintf("Error during menu flow: %s", err.Error()))
					utils.Sleep(1000)
					continue
				}
			case <-time.After(maxTimeNotInGame):
				// The entire HandleMenuFlow function took too long. This means a game state read is likely frozen.
				s.bot.ctx.Logger.Error(fmt.Sprintf("Menu flow frozen for more than %s. Forcing client restart.", maxTimeNotInGame))
				if killErr := s.KillClient(); killErr != nil {
					s.bot.ctx.Logger.Error(fmt.Sprintf("Error killing client after menu flow timeout: %s", killErr.Error()))
				}
				return ErrUnrecoverableClientState
			}
		}

		// In-game logic
		timeSpentNotInGameStart = time.Now()

		stringRuns := make([]string, len(s.bot.ctx.CharacterCfg.Game.Runs))
		for i, r := range s.bot.ctx.CharacterCfg.Game.Runs {
			stringRuns[i] = string(r)
		}
		orderedRuns := s.orderRuns(stringRuns)
		if orderedRuns == nil {
			return nil
		}

		runs := run.BuildRuns(s.bot.ctx.CharacterCfg, orderedRuns)
		gameStart := time.Now()
		cfg, _ := config.GetCharacter(s.name)

		if cfg.Game.RandomizeRuns {
			rand.Shuffle(len(runs), func(i, j int) { runs[i], runs[j] = runs[j], runs[i] })
		}

		event.Send(event.GameCreated(event.Text(s.name, "New game created"), s.bot.ctx.GameReader.LastGameName(), s.bot.ctx.GameReader.LastGamePass()))
		s.bot.ctx.CurrentGame.FailedToCreateGameAttempts = 0
		s.bot.ctx.LastBuffAt = time.Time{}
		s.logGameStart(runs)
		s.bot.ctx.RefreshGameData()

		if s.bot.ctx.CharacterCfg.Companion.Enabled && s.bot.ctx.CharacterCfg.Companion.Leader {
			event.Send(event.RequestCompanionJoinGame(event.Text(s.name, "New Game Started "+s.bot.ctx.Data.Game.LastGameName), s.bot.ctx.CharacterCfg.CharacterName, s.bot.ctx.Data.Game.LastGameName, s.bot.ctx.Data.Game.LastGamePassword))
		}

		if firstRun {
			missingKeybindings := s.bot.ctx.Char.CheckKeyBindings()
			if len(missingKeybindings) > 0 {
				var missingKeybindingsText = "Missing key binding for skill(s):"
				for _, v := range missingKeybindings {
					missingKeybindingsText += fmt.Sprintf("\n%s", skill.SkillNames[v])
				}
				missingKeybindingsText += "\nPlease bind the skills. Pausing bot..."

				utils.ShowDialog("Missing keybindings for "+s.name, missingKeybindingsText)
				s.TogglePause()
			}
		}

		// Context with a timeout for the game itself
		runCtx := ctx
		var runCancel context.CancelFunc
		if s.bot.ctx.CharacterCfg.MaxGameLength > 0 {
			runCtx, runCancel = context.WithTimeout(ctx, time.Duration(s.bot.ctx.CharacterCfg.MaxGameLength)*time.Second)
		} else {
			runCtx, runCancel = context.WithCancel(ctx)
		}
		defer runCancel()

		// In-Game Activity Monitor
		go func() {
			ticker := time.NewTicker(activityCheckInterval)
			defer ticker.Stop()
			var lastPosition data.Position
			var stuckSince time.Time
			var droppedMouseItem bool // Track if we've already tried dropping mouse item

			// Initial position check
			if s.bot.ctx.GameReader.InGame() && s.bot.ctx.Data.PlayerUnit.ID > 0 {
				lastPosition = s.bot.ctx.Data.PlayerUnit.Position
			}

			for {
				select {
				case <-runCtx.Done(): // Exit when the run is over (either completed, errored, or timed out)
					return
				case <-ticker.C:
					if s.bot.ctx.ExecutionPriority == ct.PriorityPause {
						continue
					}

					if !s.bot.ctx.GameReader.InGame() || s.bot.ctx.Data.PlayerUnit.ID == 0 {
						continue
					}
					currentPos := s.bot.ctx.Data.PlayerUnit.Position
					if currentPos.X == lastPosition.X && currentPos.Y == lastPosition.Y {
						if stuckSince.IsZero() {
							stuckSince = time.Now()
							droppedMouseItem = false // Reset flag when first detecting stuck
						}

						stuckDuration := time.Since(stuckSince)

						// After 90 seconds stuck, try dropping mouse item
						if stuckDuration > 90*time.Second && !droppedMouseItem {
							s.bot.ctx.Logger.Warn("Player stuck for 90 seconds. Attempting to drop any item on cursor...")
							// Click to drop any item that might be stuck on cursor
							s.bot.ctx.HID.Click(game.LeftButton, 500, 500)
							droppedMouseItem = true
							s.bot.ctx.Logger.Info("Clicked to drop mouse item (if any). Continuing to monitor for movement...")
						}

						// After 3 minutes stuck, force restart
						if stuckDuration > maxStuckDuration {
							s.bot.ctx.Logger.Error(fmt.Sprintf("In-game activity monitor: Player has been stuck for over %s. Forcing client restart.", maxStuckDuration))
							if err := s.KillClient(); err != nil {
								s.bot.ctx.Logger.Error(fmt.Sprintf("Activity monitor failed to kill client: %v", err))
							}
							runCancel() // Also cancel the context to stop bot.Run gracefully
							return
						}
					} else {
						stuckSince = time.Time{} // Reset timer if the player has moved
						droppedMouseItem = false // Reset flag if player moved
					}
					lastPosition = currentPos
				}
			}
		}()

		err = s.bot.Run(runCtx, firstRun, runs)
		firstRun = false

		if err != nil {
			if errors.Is(err, context.DeadlineExceeded) {
				// We don't log the generic "Bot run finished with error" message if it was a planned timeout
			} else {
				s.bot.ctx.Logger.Info(fmt.Sprintf("Bot run finished with error: %s. Initiating game exit and cooldown.", err.Error()))
			}

			if exitErr := s.bot.ctx.Manager.ExitGame(); exitErr != nil {
				s.bot.ctx.Logger.Error(fmt.Sprintf("Error trying to exit game: %s", exitErr.Error()))
				return ErrUnrecoverableClientState
			}

			s.bot.ctx.Logger.Info("Waiting 5 seconds for game client to close completely...")
			utils.Sleep(int(5 * time.Second / time.Millisecond))

			timeout := time.After(15 * time.Second)
			for s.bot.ctx.Manager.InGame() {
				select {
				case <-ctx.Done():
					return nil
				case <-timeout:
					s.bot.ctx.Logger.Error("Timeout waiting for game to report 'not in game' after exit attempt. Forcing client kill.")
					if killErr := s.KillClient(); killErr != nil {
						s.bot.ctx.Logger.Error(fmt.Sprintf("Failed to kill client after timeout and InGame() check: %s", killErr.Error()))
					}
					return ErrUnrecoverableClientState
				default:
					s.bot.ctx.Logger.Debug("Still detected as in game, waiting for RefreshGameData to update...")
					utils.Sleep(int(500 * time.Millisecond / time.Millisecond))
					s.bot.ctx.RefreshGameData()
				}
			}
			s.bot.ctx.Logger.Info("Game client successfully detected as 'not in game'.")
			timeSpentNotInGameStart = time.Now()

			var gameFinishReason event.FinishReason
			switch {
			case errors.Is(err, health.ErrChicken):
				gameFinishReason = event.FinishedChicken
			case errors.Is(err, health.ErrMercChicken):
				gameFinishReason = event.FinishedMercChicken
			case errors.Is(err, health.ErrDied):
				gameFinishReason = event.FinishedDied
			default:
				gameFinishReason = event.FinishedError
			}
			event.Send(event.GameFinished(event.WithScreenshot(s.name, err.Error(), s.bot.ctx.GameReader.Screenshot()), gameFinishReason))

			s.bot.ctx.Logger.Warn(
				fmt.Sprintf("Game finished with errors, reason: %s. Game total time: %0.2fs", err.Error(), time.Since(gameStart).Seconds()),
				slog.String("supervisor", s.name),
				slog.Uint64("mapSeed", uint64(s.bot.ctx.GameReader.MapSeed())),
			)
			continue
		}

		gameFinishReason := event.FinishedOK
		event.Send(event.GameFinished(event.Text(s.name, "Game finished successfully"), gameFinishReason))
		s.bot.ctx.Logger.Info(
			fmt.Sprintf("Game finished successfully. Game total time: %0.2fs", time.Since(gameStart).Seconds()),
			slog.String("supervisor", s.name),
			slog.Uint64("mapSeed", uint64(s.bot.ctx.GameReader.MapSeed())),
		)
		if s.bot.ctx.CharacterCfg.Companion.Enabled && s.bot.ctx.CharacterCfg.Companion.Leader {
			event.Send(event.ResetCompanionGameInfo(event.Text(s.name, "Game "+s.bot.ctx.Data.Game.LastGameName+" finished"), s.bot.ctx.CharacterCfg.CharacterName))
		}
		if exitErr := s.bot.ctx.Manager.ExitGame(); exitErr != nil {
			errMsg := fmt.Sprintf("Error exiting game %s", exitErr.Error())
			event.Send(event.GameFinished(event.WithScreenshot(s.name, errMsg, s.bot.ctx.GameReader.Screenshot()), event.FinishedError))
			return errors.New(errMsg)
		}
		s.bot.ctx.Logger.Info("Game finished successfully. Waiting 3 seconds for client to close.")
		utils.Sleep(int(3 * time.Second / time.Millisecond))
		timeSpentNotInGameStart = time.Now()
	}
}

// NEW HELPER FUNCTION that wraps a blocking operation with a timeout
func (s *SinglePlayerSupervisor) callManagerWithTimeout(fn func() error) error {
	errChan := make(chan error, 1)
	go func() {
		errChan <- fn()
	}()

	select {
	case err := <-errChan:
		return err
	case <-time.After(menuActionTimeout):
		return fmt.Errorf("menu action timed out after %s", menuActionTimeout)
	}
}

func (s *SinglePlayerSupervisor) HandleMenuFlow() error {
	s.bot.ctx.RefreshGameData()

	if s.bot.ctx.Data.OpenMenus.LoadingScreen {
		utils.Sleep(500)
		return fmt.Errorf("loading screen")
	}

	s.bot.ctx.Logger.Debug("[Menu Flow]: Starting menu flow ...")

	if s.bot.ctx.GameReader.IsInCharacterCreationScreen() {
		s.bot.ctx.Logger.Debug("[Menu Flow]: We're in character creation screen, exiting ...")
		s.bot.ctx.HID.PressKey(0x1B)
		time.Sleep(2000)
		if s.bot.ctx.GameReader.IsInCharacterCreationScreen() {
			return errors.New("[Menu Flow]: Failed to exit character creation screen")
		}
	}

	if s.bot.ctx.Manager.InGame() {
		s.bot.ctx.Logger.Debug("[Menu Flow]: We're still ingame, exiting ...")
		return s.bot.ctx.Manager.ExitGame()
	}

	isDismissableModalPresent, text := s.bot.ctx.GameReader.IsDismissableModalPresent()
	if isDismissableModalPresent {
		s.bot.ctx.Logger.Debug("[Menu Flow]: Detected dismissable modal with text: " + text)
		s.bot.ctx.HID.PressKey(0x1B)
		time.Sleep(1000)

		isDismissableModalStillPresent, _ := s.bot.ctx.GameReader.IsDismissableModalPresent()
		if isDismissableModalStillPresent {
			s.bot.ctx.Logger.Warn(fmt.Sprintf("[Menu Flow]: Dismissable modal still present after attempt to dismiss: %s", text))
			s.bot.ctx.CurrentGame.FailedToCreateGameAttempts++
			const MAX_MODAL_DISMISS_ATTEMPTS = 3
			if s.bot.ctx.CurrentGame.FailedToCreateGameAttempts >= MAX_MODAL_DISMISS_ATTEMPTS {
				s.bot.ctx.Logger.Error(fmt.Sprintf("[Menu Flow]: Failed to dismiss modal '%s' %d times. Assuming unrecoverable state.", text, MAX_MODAL_DISMISS_ATTEMPTS))
				s.bot.ctx.CurrentGame.FailedToCreateGameAttempts = 0
				return ErrUnrecoverableClientState
			}
			return errors.New("[Menu Flow]: Failed to dismiss popup (still present)")
		}
	} else {
		// If no dismissable modal is present, reset the counter for failed attempts if it's related to modals
		s.bot.ctx.CurrentGame.FailedToCreateGameAttempts = 0
	}

	if s.bot.ctx.CharacterCfg.Companion.Enabled && !s.bot.ctx.CharacterCfg.Companion.Leader {
		return s.HandleCompanionMenuFlow()
	}

	return s.HandleStandardMenuFlow()
}

func (s *SinglePlayerSupervisor) HandleStandardMenuFlow() error {
	atCharacterSelectionScreen := s.bot.ctx.GameReader.IsInCharacterSelectionScreen()

	if atCharacterSelectionScreen && s.bot.ctx.CharacterCfg.AuthMethod != "None" && !s.bot.ctx.CharacterCfg.Game.CreateLobbyGames {
		s.bot.ctx.Logger.Debug("[Menu Flow]: We're at the character selection screen, ensuring we're online ...")

		err := s.ensureOnline()
		if err != nil {
			return err
		}

		s.bot.ctx.Logger.Debug("[Menu Flow]: We're online, creating new game ...")

		// USE THE NEW TIMEOUT FUNCTION
		return s.callManagerWithTimeout(s.bot.ctx.Manager.NewGame)

	} else if atCharacterSelectionScreen && s.bot.ctx.CharacterCfg.AuthMethod == "None" {

		s.bot.ctx.Logger.Debug("[Menu Flow]: Creating new game ...")
		return s.callManagerWithTimeout(s.bot.ctx.Manager.NewGame)
	}

	atLobbyScreen := s.bot.ctx.GameReader.IsInLobby()

	if atLobbyScreen && s.bot.ctx.CharacterCfg.Game.CreateLobbyGames {
		s.bot.ctx.Logger.Debug("[Menu Flow]: We're at the lobby screen and we should create a lobby game ...")

		if s.bot.ctx.CharacterCfg.Game.PublicGameCounter == 0 {
			s.bot.ctx.CharacterCfg.Game.PublicGameCounter = 1
		}

		return s.createLobbyGame()
	} else if !atLobbyScreen && s.bot.ctx.CharacterCfg.Game.CreateLobbyGames {
		s.bot.ctx.Logger.Debug("[Menu Flow]: We're not at the lobby screen, trying to enter lobby ...")
		err := s.tryEnterLobby()
		if err != nil {
			return err
		}

		return s.createLobbyGame()
	} else if atLobbyScreen && !s.bot.ctx.CharacterCfg.Game.CreateLobbyGames {
		s.bot.ctx.Logger.Debug("[Menu Flow]: We're at the lobby screen, but we shouldn't be, going back to character selection screen ...")

		s.bot.ctx.HID.PressKey(0x1B)
		time.Sleep(2000)

		if s.bot.ctx.GameReader.IsInLobby() {
			return fmt.Errorf("[Menu Flow]: Failed to exit lobby")
		}

		if s.bot.ctx.GameReader.IsInCharacterSelectionScreen() {
			return s.callManagerWithTimeout(s.bot.ctx.Manager.NewGame)
		}
	}

	return fmt.Errorf("[Menu Flow]: Unhandled menu scenario")
}

func (s *SinglePlayerSupervisor) HandleCompanionMenuFlow() error {
	s.bot.ctx.Logger.Debug("[Menu Flow]: Trying to enter lobby ...")

	gameName := s.bot.ctx.CharacterCfg.Companion.CompanionGameName
	gamePassword := s.bot.ctx.CharacterCfg.Companion.CompanionGamePassword

	if gameName == "" {
		utils.Sleep(2000)
		return fmt.Errorf("idle")
	}

	if s.bot.ctx.GameReader.IsInCharacterSelectionScreen() {
		err := s.ensureOnline()
		if err != nil {
			return err
		}

		err = s.tryEnterLobby()
		if err != nil {
			return err
		}

		joinGameFunc := func() error {
			return s.bot.ctx.Manager.JoinOnlineGame(gameName, gamePassword)
		}
		return s.callManagerWithTimeout(joinGameFunc)
	}

	if s.bot.ctx.GameReader.IsInLobby() {
		s.bot.ctx.Logger.Debug("[Menu Flow]: We're in lobby, joining game ...")
		joinGameFunc := func() error {
			return s.bot.ctx.Manager.JoinOnlineGame(gameName, gamePassword)
		}
		return s.callManagerWithTimeout(joinGameFunc)
	}

	return fmt.Errorf("[Menu Flow]: Unhandled Companion menu scenario")
}

func (s *SinglePlayerSupervisor) tryEnterLobby() error {
	if s.bot.ctx.GameReader.IsInLobby() {
		s.bot.ctx.Logger.Debug("[Menu Flow]: We're already in lobby, exiting ...")
		return nil
	}

	retryCount := 0
	for !s.bot.ctx.GameReader.IsInLobby() {
		s.bot.ctx.Logger.Info("Entering lobby", slog.String("supervisor", s.name))
		if retryCount >= 5 {
			return fmt.Errorf("[Menu Flow]: Failed to enter bnet lobby after 5 retries")
		}

		s.bot.ctx.HID.Click(game.LeftButton, 744, 650)
		utils.Sleep(1000)
		retryCount++
	}

	return nil
}

func (s *SinglePlayerSupervisor) createLobbyGame() error {
	s.bot.ctx.Logger.Debug("[Menu Flow]: Trying to create lobby game ...")

	// USE THE NEW TIMEOUT FUNCTION
	createGameFunc := func() error {
		_, err := s.bot.ctx.Manager.CreateLobbyGame(s.bot.ctx.CharacterCfg.Game.PublicGameCounter)
		return err
	}
	err := s.callManagerWithTimeout(createGameFunc)

	if err != nil {
		s.bot.ctx.CharacterCfg.Game.PublicGameCounter++
		s.bot.ctx.CurrentGame.FailedToCreateGameAttempts++
		const MAX_GAME_CREATE_ATTEMPTS = 5
		if s.bot.ctx.CurrentGame.FailedToCreateGameAttempts >= MAX_GAME_CREATE_ATTEMPTS {
			s.bot.ctx.Logger.Error(fmt.Sprintf("[Menu Flow]: Failed to create lobby game %d times. Forcing client restart.", MAX_GAME_CREATE_ATTEMPTS))
			s.bot.ctx.CurrentGame.FailedToCreateGameAttempts = 0
			return ErrUnrecoverableClientState
		}
		return fmt.Errorf("[Menu Flow]: Failed to create lobby game: %w", err)
	}

	isDismissableModalPresent, text := s.bot.ctx.GameReader.IsDismissableModalPresent()
	if isDismissableModalPresent {
		s.bot.ctx.CharacterCfg.Game.PublicGameCounter++
		s.bot.ctx.Logger.Warn(fmt.Sprintf("[Menu Flow]: Dismissable modal present after game creation attempt: %s", text))

		if strings.Contains(strings.ToLower(text), "failed to create game") || strings.Contains(strings.ToLower(text), "unable to join") {
			s.bot.ctx.CurrentGame.FailedToCreateGameAttempts++
			const MAX_GAME_CREATE_ATTEMPTS_MODAL = 3
			if s.bot.ctx.CurrentGame.FailedToCreateGameAttempts >= MAX_GAME_CREATE_ATTEMPTS_MODAL {
				s.bot.ctx.Logger.Error(fmt.Sprintf("[Menu Flow]: 'Failed to create game' modal detected %d times. Forcing client restart.", MAX_GAME_CREATE_ATTEMPTS_MODAL))
				s.bot.ctx.CurrentGame.FailedToCreateGameAttempts = 0
				return ErrUnrecoverableClientState
			}
		}
		return fmt.Errorf("[Menu Flow]: Failed to create lobby game: %s", text)
	}

	s.bot.ctx.Logger.Debug("[Menu Flow]: Lobby game created successfully")
	s.bot.ctx.CharacterCfg.Game.PublicGameCounter++
	s.bot.ctx.CurrentGame.FailedToCreateGameAttempts = 0
	return nil
}
