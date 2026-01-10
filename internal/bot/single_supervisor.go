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
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
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

// Define constants for leecher/follower mode
const (
	leecherMaxGameDuration   = 4 * time.Hour    // Maximum time to stay in one game
	leecherLeaderTimeout     = 60 * time.Second // How long to wait for leader to appear
	leecherExitTimeout       = 10 * time.Second // Timeout when waiting for game exit
	leecherRosterPollDelay   = 2 * time.Second  // Delay between roster poll attempts
	leecherDefaultPollMs     = 2000             // Default poll interval in milliseconds
	leecherBackoffBaseMs     = 2000             // Base delay for exponential backoff
	leecherBackoffMaxMs      = 30000            // Maximum backoff delay
	followerDefaultPollMs    = 30000            // Default poll interval for followers
	leecherLeaderAreaRetries = 10               // Retries to get leader's area
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
	// Attach context to this goroutine so context.Get() works
	s.bot.ctx.AttachRoutine(ct.PriorityNormal)
	defer s.bot.ctx.Detach()

	ctx, cancel := context.WithCancel(context.Background())
	s.cancelFn = cancel

	err := s.ensureProcessIsRunningAndPrepare()
	if err != nil {
		return fmt.Errorf("error preparing game: %w", err)
	}

	// MANUAL MODE: Early exit - handle before normal game loop
	if s.bot.ctx.ManualModeActive {
		s.bot.ctx.Logger.Info("Manual mode: reaching character selection...")
		if err = s.waitUntilCharacterSelectionScreen(); err != nil {
			return fmt.Errorf("manual mode: error waiting for character selection: %w", err)
		}

		s.bot.ctx.Logger.Info("Manual mode: waiting for window repositioning...")
		time.Sleep(5 * time.Second)

		// Pause/resume cycle to free resources
		s.bot.ctx.Logger.Info("Manual mode: pausing...")
		s.bot.ctx.SwitchPriority(ct.PriorityPause)
		s.bot.ctx.MemoryInjector.RestoreMemory()
		event.Send(event.GamePaused(event.Text(s.name, "Manual mode active"), true))

		time.Sleep(500 * time.Millisecond)

		s.bot.ctx.Logger.Info("Manual mode: resuming...")
		s.bot.ctx.MemoryInjector.Load()
		s.bot.ctx.SwitchPriority(ct.PriorityNormal)
		event.Send(event.GamePaused(event.Text(s.name, "Manual mode ready"), false))

		s.bot.ctx.Logger.Info("Manual mode: initialization complete")

		// Keep process alive until stopped
		for {
			select {
			case <-ctx.Done():
				return nil
			default:
				utils.Sleep(1000)
			}
		}
	}

	// Check if this supervisor is a follower (started by a leader coordinator)
	followerInfo := GetFollowerInfo(s.name)
	if followerInfo != nil {
		s.bot.ctx.Logger.Info("Running in follower mode",
			slog.String("leader", followerInfo.LeaderName),
			slog.String("gamePattern", followerInfo.GamePattern))
		return s.runFollowerMode(ctx, followerInfo)
	}

	// Check if this supervisor is a leecher (started by a leader-leecher coordinator)
	leecherInfo := GetLeecherInfo(s.name)
	if leecherInfo != nil {
		s.bot.ctx.Logger.Info("Running in leecher mode",
			slog.String("leader", leecherInfo.LeaderName),
			slog.String("gamePattern", leecherInfo.GamePattern))
		return s.runLeecherMode(ctx, leecherInfo)
	}

	// NORMAL MODE: Original code unchanged from here
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

		// Check for pending Drop via Drop manager
		if s.bot.ctx.Drop != nil && s.bot.ctx.Drop.Pending() != nil {
			// Skip if Drop is already in progress
			if s.bot.ctx.Drop.Active() != nil {
				s.bot.ctx.Logger.Debug("Drop already in progress, skipping check")
				continue
			}

			// Immediately run the pending Drop before entering the normal menu flow
			s.bot.ctx.Logger.Info("Pending Drop detected, launching Drop before menu flow")
			s.bot.ctx.SwitchPriority(ct.PriorityNormal)
			action.SwitchToLegacyMode()
			action.SwitchToLegacyMode()
			DropRun := run.NewDrop()
			if err := DropRun.Run(nil); err != nil {
				s.bot.ctx.Logger.Error("Drop run failed", "error", err)
			}
			continue
		}

		if firstRun {
			if err = s.waitUntilCharacterSelectionScreen(); err != nil {
				return fmt.Errorf("error waiting for character selection screen: %w", err)
			}
			// warning for barb to set keybinds
			if s.bot.ctx.CharacterCfg.Character.Class == "barb_leveling" {
				lvl, _ := s.bot.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
				if lvl.Value == 1 {
					warningText := "IMPORTANT: The Leveling Barbarian requires F9-F12 keybindings for Skills 9-12!\n\n"
					warningText += "Please set in the Controller Settings (Diablo 2):\n"
					warningText += "- F9 for Skill 9\n"
					warningText += "- F10 for Skill 10\n"
					warningText += "- F11 for Skill 11\n"
					warningText += "- F12 for Skill 12\n\n"
					warningText += "Bot will be paused..."

					utils.ShowDialog("F9-F12 Keybindings Required", warningText)
					s.TogglePause()
				}
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

		// Initialize ping monitor for this game session
		// Configuration from koolo.yaml (default: quit after 30s of ping > 500ms)
		pingThreshold := 500
		sustainedDuration := 30 * time.Second
		pingEnabled := false

		if config.Koolo.PingMonitor.Enabled {
			pingEnabled = true
			if config.Koolo.PingMonitor.HighPingThreshold > 0 {
				pingThreshold = config.Koolo.PingMonitor.HighPingThreshold
			}
			if config.Koolo.PingMonitor.SustainedDuration > 0 {
				sustainedDuration = time.Duration(config.Koolo.PingMonitor.SustainedDuration) * time.Second
			}
		}

		pingMonitor := health.NewPingMonitor(
			s.bot.ctx.Logger,
			pingThreshold,
			sustainedDuration,
		)
		pingMonitor.Enabled = pingEnabled
		pingMonitor.SetCallback(func() {
			s.bot.ctx.Logger.Error("Sustained high ping detected. Forcing game exit.",
				slog.Int("threshold", pingThreshold),
				slog.Duration("duration", sustainedDuration))
			runCancel()
		})

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

					// Check for sustained high ping
					if pingMonitor.CheckPing(s.bot.ctx.Data.Game.Ping) {
						s.bot.ctx.Logger.Error("Ping monitor triggered game exit.")
						return
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
		time.Sleep(2000 * time.Millisecond)
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
		time.Sleep(1000 * time.Millisecond)

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
		time.Sleep(2000 * time.Millisecond)

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

// runFollowerMode runs the supervisor in follower mode, following a human leader
func (s *SinglePlayerSupervisor) runFollowerMode(ctx context.Context, followerInfo *FollowerInfo) error {
	s.bot.ctx.Logger.Info("Follower mode: waiting for character selection screen...")

	if err := s.waitUntilCharacterSelectionScreen(); err != nil {
		return fmt.Errorf("follower mode: error waiting for character selection screen: %w", err)
	}

	s.bot.ctx.Logger.Info("Follower mode: ready to follow leader",
		slog.String("leader", followerInfo.LeaderName))

	// Main follower loop
	for {
		select {
		case <-ctx.Done():
			return nil
		default:
		}

		// Refresh followerInfo periodically to get any updates
		currentInfo := GetFollowerInfo(s.name)
		if currentInfo != nil {
			followerInfo = currentInfo
		}

		// If not in game, try to join the leader's game
		if !s.bot.ctx.Manager.InGame() {
			// Ensure we're online first
			if s.bot.ctx.GameReader.IsInCharacterSelectionScreen() {
				s.bot.ctx.Logger.Info("Follower mode: ensuring online connection...")
				if err := s.ensureOnline(); err != nil {
					s.bot.ctx.Logger.Warn("Follower mode: failed to ensure online",
						slog.String("error", err.Error()))
					if !s.sleepWithContextCheck(ctx, 5000) {
						return nil
					}
					continue
				}
			}

			// Enter the lobby if not already there
			if !s.bot.ctx.GameReader.IsInLobby() {
				s.bot.ctx.Logger.Info("Follower mode: entering lobby...")
				if err := s.tryEnterLobby(); err != nil {
					s.bot.ctx.Logger.Warn("Follower mode: failed to enter lobby",
						slog.String("error", err.Error()))
					if !s.sleepWithContextCheck(ctx, 5000) {
						return nil
					}
					continue
				}
			}

			// Double-check we're not in a game before trying to join
			if s.bot.ctx.Manager.InGame() {
				s.bot.ctx.Logger.Warn("Follower mode: still in game when trying to join, exiting first",
					slog.String("supervisor", s.name))
				if err := s.bot.ctx.Manager.ExitGame(); err != nil {
					s.bot.ctx.Logger.Warn("Follower mode: error exiting residual game",
						slog.String("supervisor", s.name),
						slog.String("error", err.Error()))
				}
				if !s.sleepWithContextCheck(ctx, 3000) {
					return nil
				}
				continue
			}

			// Get the current game name (pattern + counter)
			gameName := GetCurrentGameName(s.name)
			if gameName == "" {
				gameName = followerInfo.GamePattern + "1" // Fallback
			}

			s.bot.ctx.Logger.Info("Follower mode: attempting to join leader's game",
				slog.String("supervisor", s.name),
				slog.String("gameName", gameName),
				slog.Int("retryCount", GetJoinRetries(s.name)))

			// Random delay before joining using utility function
			delay := CalculateRandomDelay(followerInfo.JoinDelayMin, followerInfo.JoinDelayMax)
			s.bot.ctx.Logger.Info("Follower mode: waiting before joining",
				slog.String("supervisor", s.name),
				slog.Int("delayMs", delay))
			if !s.sleepWithContextCheck(ctx, delay) {
				return nil
			}

			// Final check before join attempt
			if s.bot.ctx.Manager.InGame() {
				s.bot.ctx.Logger.Warn("Follower mode: entered game during delay, skipping join",
					slog.String("supervisor", s.name))
				continue
			}

			// Try to join the game
			joinGameFunc := func() error {
				return s.bot.ctx.Manager.JoinOnlineGame(gameName, followerInfo.GamePassword)
			}
			err := s.callManagerWithTimeout(joinGameFunc)
			if err != nil {
				s.bot.ctx.Logger.Warn("Follower mode: failed to join game",
					slog.String("supervisor", s.name),
					slog.String("error", err.Error()),
					slog.String("gameName", gameName),
					slog.Int("retryCount", GetJoinRetries(s.name)+1))

				// Dismiss any modal that might be blocking (e.g., "Game does not exist")
				s.dismissFollowerModal()

				// Check if we've hit max retries
				retryCount := GetJoinRetries(s.name)
				if IncrementJoinRetries(s.name) {
					s.bot.ctx.Logger.Warn("Follower mode: max join retries reached, skipping to next game",
						slog.String("supervisor", s.name),
						slog.String("currentGame", gameName))
					IncrementGameCounter(s.name)
					nextGame := GetCurrentGameName(s.name)
					s.bot.ctx.Logger.Info("Follower mode: will try next game",
						slog.String("supervisor", s.name),
						slog.String("nextGame", nextGame))
					// Reset retry count after skipping to next game
					retryCount = 0
				}

				// Use exponential backoff for rate limit protection
				backoffDelay := CalculateExponentialBackoff(leecherBackoffBaseMs, leecherBackoffMaxMs, retryCount)
				s.bot.ctx.Logger.Debug("Follower mode: waiting before retry",
					slog.String("supervisor", s.name),
					slog.Int("backoffMs", backoffDelay),
					slog.Int("retryCount", retryCount))
				if !s.sleepWithContextCheck(ctx, backoffDelay) {
					return nil
				}
				continue
			}

			// Reset retry count on successful join
			ResetJoinRetries(s.name)
			s.bot.ctx.Logger.Info("Follower mode: joined game successfully",
				slog.String("supervisor", s.name),
				slog.String("gameName", gameName))
		}

		// In-game follower behavior
		if s.bot.ctx.Manager.InGame() {
			err := s.runFollowerInGame(ctx, followerInfo)
			if err != nil {
				s.bot.ctx.Logger.Warn("Follower mode: error in game, leaving...",
					slog.String("supervisor", s.name),
					slog.String("error", err.Error()))
				// Only try to exit if we're still in game
				if s.bot.ctx.Manager.InGame() {
					s.bot.ctx.Manager.ExitGame()
					// Wait for exit to complete
					exitDeadline := time.Now().Add(leecherExitTimeout)
				exitWaitLoop:
					for s.bot.ctx.Manager.InGame() {
						select {
						case <-ctx.Done():
							return nil
						default:
							if time.Now().After(exitDeadline) {
								s.bot.ctx.Logger.Error("Follower mode: timeout waiting for error recovery exit",
									slog.String("supervisor", s.name))
								break exitWaitLoop
							}
							time.Sleep(500 * time.Millisecond)
						}
					}
				}
				if !s.sleepWithContextCheck(ctx, 3000) {
					return nil
				}
			}
		}

		if !s.sleepWithContextCheck(ctx, 1000) {
			return nil
		}
	}
}

// sleepWithContextCheck sleeps for the specified duration in milliseconds
// Returns false if context was cancelled during sleep
func (s *SinglePlayerSupervisor) sleepWithContextCheck(ctx context.Context, ms int) bool {
	select {
	case <-ctx.Done():
		return false
	case <-time.After(time.Duration(ms) * time.Millisecond):
		return true
	}
}

// dismissFollowerModal dismisses any modal dialogs that might be blocking follower operations
func (s *SinglePlayerSupervisor) dismissFollowerModal() {
	isDismissable, text := s.bot.ctx.GameReader.IsDismissableModalPresent()
	if isDismissable {
		s.bot.ctx.Logger.Debug("Follower mode: dismissing modal",
			slog.String("text", text))
		s.bot.ctx.HID.PressKey(0x1B) // Escape
		utils.Sleep(500)

		// Check if still present and try clicking OK button area
		stillPresent, _ := s.bot.ctx.GameReader.IsDismissableModalPresent()
		if stillPresent {
			s.bot.ctx.Logger.Debug("Follower mode: modal still present, trying click dismiss")
			// Click center-ish area where OK button typically is
			s.bot.ctx.HID.Click(game.LeftButton, 640, 400)
			utils.Sleep(500)
		}
	}
}

// runFollowerInGame handles follower behavior while in a game
func (s *SinglePlayerSupervisor) runFollowerInGame(ctx context.Context, followerInfo *FollowerInfo) error {
	s.bot.ctx.Logger.Info("Follower mode: in game, setting up...",
		slog.String("supervisor", s.name))

	// Wait for game to fully load
	s.bot.ctx.WaitForGameToLoad()

	// Fetch map data for consistency (even though followers don't navigate much)
	// This ensures area data and collision grid are available if needed
	if err := s.bot.ctx.GameReader.FetchMapData(); err != nil {
		s.bot.ctx.Logger.Warn("Follower mode: failed to fetch map data, continuing anyway",
			slog.String("supervisor", s.name),
			slog.String("error", err.Error()))
	}
	s.bot.ctx.RefreshGameData()

	// Enable legacy graphics if configured
	if followerInfo.UseLegacyGraphics {
		s.bot.ctx.Logger.Info("Follower mode: enabling legacy graphics",
			slog.String("supervisor", s.name))
		action.ForceSwitchToLegacyMode()
	}

	// Try to join the leader's party
	s.bot.ctx.Logger.Info("Follower mode: looking for leader",
		slog.String("supervisor", s.name),
		slog.String("leader", followerInfo.LeaderName))

	// Wait for leader to appear in roster (with timeout)
	start := time.Now()
	leaderFound := false
	for time.Since(start) < leecherLeaderTimeout {
		select {
		case <-ctx.Done():
			return nil
		default:
		}

		// Check if we're still in game
		if !s.bot.ctx.Manager.InGame() {
			return fmt.Errorf("disconnected from game while searching for leader")
		}

		s.bot.ctx.RefreshGameData()
		if action.FindPlayerInRoster(followerInfo.LeaderName) {
			leaderFound = true
			break
		}
		if !s.sleepWithContextCheck(ctx, int(leecherRosterPollDelay.Milliseconds())) {
			return nil
		}
	}

	if !leaderFound {
		return fmt.Errorf("leader %s not found in game after %v", followerInfo.LeaderName, leecherLeaderTimeout)
	}

	s.bot.ctx.Logger.Info("Follower mode: leader found, joining party",
		slog.String("supervisor", s.name))

	// Try to join party
	if !action.IsInPartyWith(followerInfo.LeaderName) {
		if err := action.JoinPlayerParty(followerInfo.LeaderName, 3); err != nil {
			s.bot.ctx.Logger.Warn("Follower mode: failed to join party",
				slog.String("supervisor", s.name),
				slog.String("error", err.Error()))
		}
	}

	// Go to town and wait
	s.bot.ctx.Logger.Info("Follower mode: idling in town, monitoring leader",
		slog.String("supervisor", s.name))

	// Poll for leader presence using followerInfo.PollInterval
	pollInterval := time.Duration(followerDefaultPollMs) * time.Millisecond
	if followerInfo.PollInterval > 0 {
		pollInterval = time.Duration(followerInfo.PollInterval) * time.Millisecond
	}

	s.bot.ctx.Logger.Debug("Follower mode: poll interval set",
		slog.String("supervisor", s.name),
		slog.Duration("interval", pollInterval))

	ticker := time.NewTicker(pollInterval)
	defer ticker.Stop()

	gameStartTime := time.Now()

	for {
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
			// Check maximum game duration (safeguard against infinite loops)
			if time.Since(gameStartTime) > leecherMaxGameDuration {
				s.bot.ctx.Logger.Warn("Follower mode: max game duration reached, leaving game",
					slog.Duration("duration", time.Since(gameStartTime)))
				return fmt.Errorf("max game duration exceeded")
			}

			// Check if we're still in game
			if !s.bot.ctx.Manager.InGame() {
				s.bot.ctx.Logger.Warn("Follower mode: disconnected from game during monitoring")
				return fmt.Errorf("disconnected from game")
			}

			// Refresh followerInfo to get any updates
			currentInfo := GetFollowerInfo(s.name)
			if currentInfo != nil {
				followerInfo = currentInfo
			}

			s.bot.ctx.RefreshGameData()

			// Check if leader is still in game
			if !action.FindPlayerInRoster(followerInfo.LeaderName) {
				s.bot.ctx.Logger.Info("Follower mode: leader has left the game, leaving and preparing for next game",
					slog.String("supervisor", s.name))

				// Leave the current game first (only if still in game)
				if s.bot.ctx.Manager.InGame() {
					s.bot.ctx.Logger.Info("Follower mode: exiting current game...",
						slog.String("supervisor", s.name))
					if err := s.bot.ctx.Manager.ExitGame(); err != nil {
						s.bot.ctx.Logger.Warn("Follower mode: error exiting game",
							slog.String("supervisor", s.name),
							slog.String("error", err.Error()))
					}
				}

				// Wait for game exit to actually complete
				exitDeadline := time.Now().Add(leecherExitTimeout)
				for s.bot.ctx.Manager.InGame() {
					select {
					case <-ctx.Done():
						return nil
					default:
						if time.Now().After(exitDeadline) {
							s.bot.ctx.Logger.Error("Follower mode: timeout waiting for game exit, forcing restart",
								slog.String("supervisor", s.name))
							// If still in game after timeout, something is very wrong - return error to trigger recovery
							return fmt.Errorf("timeout waiting for game exit")
						}
						s.bot.ctx.RefreshGameData()
						time.Sleep(500 * time.Millisecond)
					}
				}

				s.bot.ctx.Logger.Info("Follower mode: successfully exited game",
					slog.String("supervisor", s.name))

				// Extra wait after confirmed exit for stability
				if !s.sleepWithContextCheck(ctx, 2000) {
					return nil
				}

				// Increment the game counter for the next game
				IncrementGameCounter(s.name)
				nextGame := GetCurrentGameName(s.name)
				s.bot.ctx.Logger.Info("Follower mode: next game will be",
					slog.String("supervisor", s.name),
					slog.String("gameName", nextGame))

				// Random delay before rejoining (staggered) using utility function
				delay := CalculateRandomDelay(followerInfo.JoinDelayMin, followerInfo.JoinDelayMax)
				s.bot.ctx.Logger.Info("Follower mode: waiting before rejoining",
					slog.String("supervisor", s.name),
					slog.Int("delayMs", delay))
				if !s.sleepWithContextCheck(ctx, delay) {
					return nil
				}

				return nil // Exit in-game loop to rejoin from lobby
			}
		}
	}
}

// runLeecherMode runs the supervisor in leecher mode, following a human leader through portals
func (s *SinglePlayerSupervisor) runLeecherMode(ctx context.Context, leecherInfo *LeecherInfo) error {
	s.bot.ctx.Logger.Info("Leecher mode: waiting for character selection screen...")

	if err := s.waitUntilCharacterSelectionScreen(); err != nil {
		return fmt.Errorf("leecher mode: error waiting for character selection screen: %w", err)
	}

	s.bot.ctx.Logger.Info("Leecher mode: ready to follow leader",
		slog.String("leader", leecherInfo.LeaderName))

	// Main leecher loop
	for {
		select {
		case <-ctx.Done():
			return nil
		default:
		}

		// Refresh leecherInfo periodically to get any updates
		currentInfo := GetLeecherInfo(s.name)
		if currentInfo != nil {
			leecherInfo = currentInfo
		}

		// If not in game, try to join the leader's game
		if !s.bot.ctx.Manager.InGame() {
			// Ensure we're online first
			if s.bot.ctx.GameReader.IsInCharacterSelectionScreen() {
				s.bot.ctx.Logger.Info("Leecher mode: ensuring online connection...")
				if err := s.ensureOnline(); err != nil {
					s.bot.ctx.Logger.Warn("Leecher mode: failed to ensure online",
						slog.String("error", err.Error()))
					if !s.sleepWithContextCheck(ctx, 5000) {
						return nil
					}
					continue
				}
			}

			// Enter the lobby if not already there
			if !s.bot.ctx.GameReader.IsInLobby() {
				s.bot.ctx.Logger.Info("Leecher mode: entering lobby...")
				if err := s.tryEnterLobby(); err != nil {
					s.bot.ctx.Logger.Warn("Leecher mode: failed to enter lobby",
						slog.String("error", err.Error()))
					if !s.sleepWithContextCheck(ctx, 5000) {
						return nil
					}
					continue
				}
			}

			// Double-check we're not in a game before trying to join
			if s.bot.ctx.Manager.InGame() {
				s.bot.ctx.Logger.Warn("Leecher mode: still in game when trying to join, exiting first",
					slog.String("supervisor", s.name))
				if err := s.bot.ctx.Manager.ExitGame(); err != nil {
					s.bot.ctx.Logger.Warn("Leecher mode: error exiting residual game",
						slog.String("supervisor", s.name),
						slog.String("error", err.Error()))
				}
				if !s.sleepWithContextCheck(ctx, 3000) {
					return nil
				}
				continue
			}

			// Get the current game name (pattern + counter)
			gameName := GetLeecherGameName(s.name)
			if gameName == "" {
				gameName = leecherInfo.GamePattern + "1" // Fallback
			}

			s.bot.ctx.Logger.Info("Leecher mode: attempting to join leader's game",
				slog.String("supervisor", s.name),
				slog.String("gameName", gameName))

			// Random delay before joining
			delay := CalculateLeecherJoinDelay(leecherInfo.JoinDelayMin, leecherInfo.JoinDelayMax)
			s.bot.ctx.Logger.Info("Leecher mode: waiting before joining",
				slog.String("supervisor", s.name),
				slog.Int("delayMs", delay))
			if !s.sleepWithContextCheck(ctx, delay) {
				return nil
			}

			// Final check before join attempt
			if s.bot.ctx.Manager.InGame() {
				s.bot.ctx.Logger.Warn("Leecher mode: entered game during delay, skipping join",
					slog.String("supervisor", s.name))
				continue
			}

			// Try to join the game
			joinGameFunc := func() error {
				return s.bot.ctx.Manager.JoinOnlineGame(gameName, leecherInfo.GamePassword)
			}
			err := s.callManagerWithTimeout(joinGameFunc)
			if err != nil {
				s.bot.ctx.Logger.Warn("Leecher mode: failed to join game",
					slog.String("supervisor", s.name),
					slog.String("error", err.Error()),
					slog.String("gameName", gameName),
					slog.Int("retryCount", GetLeecherJoinRetries(s.name)+1))

				// Dismiss any modal that might be blocking
				s.dismissFollowerModal()

				// Check if we've hit max retries
				retryCount := GetLeecherJoinRetries(s.name)
				if IncrementLeecherJoinRetries(s.name) {
					s.bot.ctx.Logger.Warn("Leecher mode: max join retries reached, skipping to next game",
						slog.String("supervisor", s.name),
						slog.String("currentGame", gameName))
					IncrementLeecherGameCounter(s.name)
					nextGame := GetLeecherGameName(s.name)
					s.bot.ctx.Logger.Info("Leecher mode: will try next game",
						slog.String("supervisor", s.name),
						slog.String("nextGame", nextGame))
					// Reset retry count after skipping to next game
					retryCount = 0
				}

				// Use exponential backoff for rate limit protection
				backoffDelay := CalculateExponentialBackoff(leecherBackoffBaseMs, leecherBackoffMaxMs, retryCount)
				s.bot.ctx.Logger.Debug("Leecher mode: waiting before retry",
					slog.String("supervisor", s.name),
					slog.Int("backoffMs", backoffDelay),
					slog.Int("retryCount", retryCount))
				if !s.sleepWithContextCheck(ctx, backoffDelay) {
					return nil
				}
				continue
			}

			// Reset retry count on successful join
			ResetLeecherJoinRetries(s.name)
			s.bot.ctx.Logger.Info("Leecher mode: joined game successfully",
				slog.String("supervisor", s.name),
				slog.String("gameName", gameName))
		}

		// In-game leecher behavior
		if s.bot.ctx.Manager.InGame() {
			err := s.runLeecherInGame(ctx, leecherInfo)
			if err != nil {
				s.bot.ctx.Logger.Warn("Leecher mode: error in game, leaving...",
					slog.String("supervisor", s.name),
					slog.String("error", err.Error()))
				// Only try to exit if we're still in game
				if s.bot.ctx.Manager.InGame() {
					s.bot.ctx.Manager.ExitGame()
					// Wait for exit to complete
					exitDeadline := time.Now().Add(leecherExitTimeout)
				exitWaitLoop:
					for s.bot.ctx.Manager.InGame() {
						select {
						case <-ctx.Done():
							return nil
						default:
							if time.Now().After(exitDeadline) {
								s.bot.ctx.Logger.Error("Leecher mode: timeout waiting for error recovery exit",
									slog.String("supervisor", s.name))
								break exitWaitLoop
							}
							time.Sleep(500 * time.Millisecond)
						}
					}
				}
				if !s.sleepWithContextCheck(ctx, 3000) {
					return nil
				}
			}
		}

		if !s.sleepWithContextCheck(ctx, 1000) {
			return nil
		}
	}
}

// runLeecherInGame handles leecher behavior while in a game
func (s *SinglePlayerSupervisor) runLeecherInGame(ctx context.Context, leecherInfo *LeecherInfo) error {
	s.bot.ctx.Logger.Info("Leecher mode: in game, setting up...",
		slog.String("supervisor", s.name))

	// Wait for game to fully load
	s.bot.ctx.WaitForGameToLoad()

	// Brief wait for roster data to populate
	if !s.sleepWithContextCheck(ctx, 500) {
		return nil
	}

	// CRITICAL: Fetch map data for collision grid - this is required for pathfinding
	s.bot.ctx.Logger.Info("Leecher mode: fetching map data for pathfinding",
		slog.String("supervisor", s.name))
	if err := s.bot.ctx.GameReader.FetchMapData(); err != nil {
		s.bot.ctx.Logger.Error("Leecher mode: failed to fetch map data",
			slog.String("supervisor", s.name),
			slog.String("error", err.Error()))
		return fmt.Errorf("failed to fetch map data: %w", err)
	}
	s.bot.ctx.RefreshGameData()
	s.bot.ctx.Logger.Info("Leecher mode: map data loaded successfully",
		slog.String("supervisor", s.name))

	// Enable legacy graphics if configured
	if leecherInfo.UseLegacyGraphics {
		s.bot.ctx.Logger.Info("Leecher mode: enabling legacy graphics",
			slog.String("supervisor", s.name))
		action.ForceSwitchToLegacyMode()
		// Brief wait for graphics mode change to settle
		if !s.sleepWithContextCheck(ctx, 500) {
			return nil
		}
		// Quick refresh to ensure data is current
		s.bot.ctx.RefreshGameData()
		s.bot.ctx.Logger.Debug("Game data refreshed after legacy graphics switch")
	}

	// Update leecher state
	SetLeecherState(s.name, LeecherStateWaiting)

	// Check if we critically need TP scrolls (0-1 in tome)
	// Don't risk dying trying to buy if we have a few already
	s.bot.ctx.RefreshGameData()
	portalTome, hasTome := s.bot.ctx.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	tpCount := 0
	if hasTome {
		if qty, found := portalTome.FindStat(stat.Quantity, 0); found {
			tpCount = qty.Value
		}
	}
	
	// Only buy if we have 0-1 TPs or no tome at all
	criticallyLow := !hasTome || tpCount <= 1
	if s.bot.ctx.Data.PlayerUnit.Area.IsTown() && criticallyLow {
		s.bot.ctx.Logger.Info("Leecher mode: critically low on TP scrolls, buying",
			slog.String("supervisor", s.name),
			slog.Int("currentTPs", tpCount),
			slog.Bool("hasTome", hasTome))
		if err := action.VendorRefill(true, false); err != nil {
			s.bot.ctx.Logger.Warn("Leecher mode: failed to buy TPs, continuing anyway",
				slog.String("supervisor", s.name),
				slog.String("error", err.Error()))
			// Don't let vendor failures block us - we can try later or use portals
		}
	} else {
		s.bot.ctx.Logger.Debug("Leecher mode: skipping TP purchase",
			slog.String("supervisor", s.name),
			slog.Int("currentTPs", tpCount),
			slog.Bool("hasTome", hasTome))
	}

	// Try to join the leader's party
	s.bot.ctx.Logger.Info("Leecher mode: looking for leader",
		slog.String("supervisor", s.name),
		slog.String("leader", leecherInfo.LeaderName))

	// Wait for leader to appear in roster (with timeout)
	start := time.Now()
	leaderFound := false
	for time.Since(start) < leecherLeaderTimeout {
		select {
		case <-ctx.Done():
			return nil
		default:
		}

		// Check if we're still in game
		if !s.bot.ctx.Manager.InGame() {
			return fmt.Errorf("disconnected from game while searching for leader")
		}

		s.bot.ctx.RefreshGameData()
		if action.FindPlayerInRoster(leecherInfo.LeaderName) {
			leaderFound = true
			break
		}
		if !s.sleepWithContextCheck(ctx, int(leecherRosterPollDelay.Milliseconds())) {
			return nil
		}
	}

	if !leaderFound {
		return fmt.Errorf("leader %s not found in game after %v", leecherInfo.LeaderName, leecherLeaderTimeout)
	}

	// Get the leader's area - retry a few times if area is 0 (data not fully loaded)
	var leaderArea area.ID
	var leaderAct int
	for i := 0; i < leecherLeaderAreaRetries; i++ {
		s.bot.ctx.RefreshGameData()
		_, leaderArea, _ = action.GetLeaderPosition(leecherInfo.LeaderName)
		leaderAct = action.GetActFromArea(leaderArea)

		// Debug: Log roster contents
		if i == 0 || leaderAct == 0 {
			for _, member := range s.bot.ctx.Data.Roster {
				s.bot.ctx.Logger.Debug("Roster member",
					slog.String("name", member.Name),
					slog.Int("area", int(member.Area)),
					slog.Int("x", member.Position.X),
					slog.Int("y", member.Position.Y))
			}
		}

		if leaderAct > 0 {
			break
		}
		if !s.sleepWithContextCheck(ctx, 1000) {
			return nil
		}
	}

	// If we still can't determine leader's act, use our current act
	if leaderAct == 0 {
		leaderAct = action.GetActFromArea(s.bot.ctx.Data.PlayerUnit.Area)
		s.bot.ctx.Logger.Warn("Could not determine leader's act, using current act",
			slog.Int("currentAct", leaderAct))
	}

	s.bot.ctx.Logger.Info("Leecher mode: leader found, joining party",
		slog.String("supervisor", s.name),
		slog.Int("leaderAct", leaderAct),
		slog.Int("leaderArea", int(leaderArea)))

	// Try to join party
	if !action.IsInPartyWith(leecherInfo.LeaderName) {
		if err := action.JoinPlayerParty(leecherInfo.LeaderName, 3); err != nil {
			s.bot.ctx.Logger.Warn("Leecher mode: failed to join party",
				slog.String("supervisor", s.name),
				slog.String("error", err.Error()))
		}
	}

	// Navigate to leader's act if possible (using the area we already captured)
	if leaderAct > 0 {
		if err := action.GoToAct(leaderAct); err != nil {
			s.bot.ctx.Logger.Warn("Leecher mode: cannot go to leader's act, waiting in current town",
				slog.String("supervisor", s.name),
				slog.Int("targetAct", leaderAct),
				slog.String("error", err.Error()))
		}
	}

	// Go to portal waiting area
	s.bot.ctx.Logger.Info("Leecher mode: going to portal waiting area",
		slog.String("supervisor", s.name))
	if err := action.WaitAtPortalArea(); err != nil {
		s.bot.ctx.Logger.Warn("Leecher mode: failed to go to portal area",
			slog.String("supervisor", s.name),
			slog.String("error", err.Error()))
	}

	SetLeecherState(s.name, LeecherStateInPortalArea)

	// Main leecher loop - wait for portal and "come" command
	pollInterval := time.Duration(leecherInfo.PollInterval) * time.Millisecond
	if pollInterval == 0 {
		pollInterval = time.Duration(leecherDefaultPollMs) * time.Millisecond
	}

	s.bot.ctx.Logger.Info("Leecher mode: entering command polling loop",
		slog.String("supervisor", s.name),
		slog.Duration("pollInterval", pollInterval))

	ticker := time.NewTicker(pollInterval)
	defer ticker.Stop()

	gameStartTime := time.Now()

	inDungeon := false
	var entryPortalPos data.Position // Track the portal we entered through
	lastActMismatchAct := 0           // Track last act we couldn't reach to avoid log spam

	for {
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
			// Check maximum game duration
			if time.Since(gameStartTime) > leecherMaxGameDuration {
				s.bot.ctx.Logger.Warn("Leecher mode: max game duration reached, leaving game",
					slog.String("supervisor", s.name))
				return fmt.Errorf("max game duration reached")
			}

			// Check if we're still in game
			if !s.bot.ctx.Manager.InGame() {
				return fmt.Errorf("disconnected from game")
			}

			// Check for EXIT command
			if GetLeecherExitReceived(s.name) {
				s.bot.ctx.Logger.Info("Leecher mode: EXIT command received, leaving game",
					slog.String("supervisor", s.name))
				SetLeecherExitReceived(s.name, false) // Reset flag

				// Exit the game
				if err := s.bot.ctx.Manager.ExitGame(); err != nil {
					s.bot.ctx.Logger.Warn("Leecher mode: error exiting game",
						slog.String("supervisor", s.name),
						slog.String("error", err.Error()))
				}

				// Wait for exit and return
				exitDeadline := time.Now().Add(leecherExitTimeout)
				for s.bot.ctx.Manager.InGame() {
					if time.Now().After(exitDeadline) {
						break
					}
					time.Sleep(500 * time.Millisecond)
				}

				// Increment game counter for next game
				IncrementLeecherGameCounter(s.name)
				return nil
			}

			s.bot.ctx.RefreshGameData()

			// Check if leader is still in game and get their current location
			_, leaderArea, leaderFound := action.GetLeaderPosition(leecherInfo.LeaderName)
			if !leaderFound {
				s.bot.ctx.Logger.Info("Leecher mode: leader left game, preparing to exit",
					slog.String("supervisor", s.name))

				// Exit the game
				if err := s.bot.ctx.Manager.ExitGame(); err != nil {
					s.bot.ctx.Logger.Warn("Leecher mode: error exiting game",
						slog.String("supervisor", s.name),
						slog.String("error", err.Error()))
				}

				// Wait for exit to complete
				exitDeadline := time.Now().Add(leecherExitTimeout)
			leaderLeftExitLoop:
				for s.bot.ctx.Manager.InGame() {
					select {
					case <-ctx.Done():
						return nil
					default:
						if time.Now().After(exitDeadline) {
							s.bot.ctx.Logger.Error("Leecher mode: timeout waiting for exit after leader left",
								slog.String("supervisor", s.name))
							break leaderLeftExitLoop
						}
						s.bot.ctx.RefreshGameData()
						time.Sleep(500 * time.Millisecond)
					}
				}

				s.bot.ctx.Logger.Info("Leecher mode: successfully exited game",
					slog.String("supervisor", s.name))

				// Extra wait after confirmed exit for stability
				if !s.sleepWithContextCheck(ctx, 2000) {
					return nil
				}

				// Increment the game counter for the next game
				IncrementLeecherGameCounter(s.name)
				nextGame := GetLeecherGameName(s.name)
				s.bot.ctx.Logger.Info("Leecher mode: next game will be",
					slog.String("supervisor", s.name),
					slog.String("gameName", nextGame))

				// Random delay before rejoining
				delay := CalculateLeecherJoinDelay(leecherInfo.JoinDelayMin, leecherInfo.JoinDelayMax)
				s.bot.ctx.Logger.Info("Leecher mode: waiting before rejoining",
					slog.String("supervisor", s.name),
					slog.Int("delayMs", delay))
				if !s.sleepWithContextCheck(ctx, delay) {
					return nil
				}

				return nil // Exit in-game loop to rejoin from lobby
			}

			// Get current player location and check if we need to follow leader to different act
			myArea := s.bot.ctx.Data.PlayerUnit.Area
			myAct := action.GetActFromArea(myArea)
			leaderAct := action.GetActFromArea(leaderArea)

			// Debug logging for act tracking
			if myAct != leaderAct {
				s.bot.ctx.Logger.Debug("Leecher mode: act mismatch detected",
					slog.String("supervisor", s.name),
					slog.Int("myAct", myAct),
					slog.Int("leaderAct", leaderAct),
					slog.String("myArea", myArea.Area().Name),
					slog.String("leaderArea", leaderArea.Area().Name),
					slog.Bool("inDungeon", inDungeon),
					slog.Bool("stayReceived", GetLeecherStayReceived(s.name)))
			}

			// If leader is in a different act and we're not in a dungeon, follow them
			// But NOT if STAY is active
			if !inDungeon && myAct != leaderAct && !GetLeecherStayReceived(s.name) {
				// Only log if this is a new act mismatch (different from last time)
				if lastActMismatchAct != leaderAct {
					s.bot.ctx.Logger.Info("Leecher mode: leader in different act",
						slog.String("supervisor", s.name),
						slog.Int("myAct", myAct),
						slog.Int("leaderAct", leaderAct),
						slog.String("leaderArea", leaderArea.Area().Name))
					lastActMismatchAct = leaderAct
				}

				// Try to navigate to leader's act
				if err := action.GoToAct(leaderAct); err != nil {
					// If we can't reach the act (no waypoint), just wait - don't spam retries
					s.bot.ctx.Logger.Debug("Leecher mode: cannot reach leader's act, waiting",
						slog.String("supervisor", s.name),
						slog.String("error", err.Error()))
					// Don't try to go to portal area or anything - just wait for leader to come back or for us to get waypoint
					continue
				}
				
				// Successfully changed acts - reset mismatch tracker and go to portal waiting area
				lastActMismatchAct = 0
				if err := action.WaitAtPortalArea(); err != nil {
					s.bot.ctx.Logger.Warn("Leecher mode: failed to go to portal area after act change",
						slog.String("supervisor", s.name),
						slog.String("error", err.Error()))
				}
				continue
			} else if myAct == leaderAct {
				// We're in the same act now - reset the tracker
				lastActMismatchAct = 0
			}

			// If in dungeon, follow the leader
			if inDungeon {
				SetLeecherState(s.name, LeecherStateInDungeon)

				myArea := s.bot.ctx.Data.PlayerUnit.Area
				myPos := s.bot.ctx.Data.PlayerUnit.Position
				
				// Verify we're actually in a dungeon (not town)
				if myArea.IsTown() {
					s.bot.ctx.Logger.Info("Leecher mode: we're in town but inDungeon flag is set, resetting",
						slog.String("supervisor", s.name))
					inDungeon = false
					SetLeecherState(s.name, LeecherStateInPortalArea)
					
					// Go back to portal waiting area
					if err := action.WaitAtPortalArea(); err != nil {
						s.bot.ctx.Logger.Warn("Leecher mode: failed to go to portal area",
							slog.String("supervisor", s.name),
							slog.String("error", err.Error()))
					}
					continue
				}
				
				s.bot.ctx.Logger.Debug("Leecher mode: in dungeon loop",
					slog.String("supervisor", s.name),
					slog.String("myArea", myArea.Area().Name),
					slog.Int("myX", myPos.X),
					slog.Int("myY", myPos.Y),
					slog.String("leaderArea", leaderArea.Area().Name),
					slog.Bool("inDungeon", inDungeon))

				// Check if we should return to town:
				// 1. TP button was pressed (TPReceived flag) - always obey
				// 2. A new town portal appeared near us (leader opened one)
				// NOTE: We do NOT return just because leader went to town - only explicit TP command or portal
				shouldReturnToTown := false
				returnReason := ""

				// Check TP command - always obey regardless of STAY
				if GetLeecherTPReceived(s.name) {
					shouldReturnToTown = true
					returnReason = "TP command received"
					SetLeecherTPReceived(s.name, false) // Reset the flag
				}

				// Check if a NEW portal appeared near us (leader opened one for us)
				// Only check this if we haven't already decided to return
				// A "new" portal is one that's NOT at the same position as our entry portal
				if !shouldReturnToTown {
					portal, found := action.FindLeaderPortal(leecherInfo.LeaderName)
					if found {
						// Check if this is a DIFFERENT portal from our entry portal
						dx := portal.Position.X - entryPortalPos.X
						dy := portal.Position.Y - entryPortalPos.Y
						portalDistSq := dx*dx + dy*dy
						if portalDistSq > 100 { // More than 10 units away = different portal
							// Check if new portal is within reasonable distance (100 units) from us
							myPos := s.bot.ctx.Data.PlayerUnit.Position
							dx := myPos.X - portal.Position.X
							dy := myPos.Y - portal.Position.Y
							distSq := dx*dx + dy*dy
							if distSq < 10000 { // 100^2 = 10000
								shouldReturnToTown = true
								returnReason = "new portal detected nearby"
								s.bot.ctx.Logger.Info("Leecher mode: detected new portal from leader",
									slog.Int("portalX", portal.Position.X),
									slog.Int("portalY", portal.Position.Y))
							}
						}
					}
				}

				if shouldReturnToTown {
					s.bot.ctx.Logger.Info("Leecher mode: returning to town",
						slog.String("supervisor", s.name),
						slog.String("reason", returnReason))

					// If TP command was received, use own TP scroll
					// If portal detected, try to use that portal
					if returnReason == "TP command received" {
						// Use own TP scroll
						if err := action.ReturnTown(); err != nil {
							s.bot.ctx.Logger.Warn("Leecher mode: failed to use TP scroll",
								slog.String("supervisor", s.name),
								slog.String("error", err.Error()))
						}
					} else {
						// Try to use portal to return to town
						if err := action.ReturnThroughPortal(); err != nil {
							s.bot.ctx.Logger.Warn("Leecher mode: failed to return through portal, using TP",
								slog.String("supervisor", s.name),
								slog.String("error", err.Error()))
							// Fallback: use own TP
							if err := action.ReturnTown(); err != nil {
								s.bot.ctx.Logger.Warn("Leecher mode: failed to return to town",
									slog.String("supervisor", s.name),
									slog.String("error", err.Error()))
							}
						}
					}

					inDungeon = false
					SetLeecherState(s.name, LeecherStateInPortalArea)

					// Reset come received for next portal cycle
					SetLeecherComeReceived(s.name, false)

					// Go back to portal waiting area
					if err := action.WaitAtPortalArea(); err != nil {
						s.bot.ctx.Logger.Warn("Leecher mode: failed to return to portal area",
							slog.String("supervisor", s.name),
							slog.String("error", err.Error()))
					}
					continue
				}

				// Check if STAY command was received (pause following)
				if GetLeecherStayReceived(s.name) {
					// But check if COME was sent to resume following
					if GetLeecherComeReceived(s.name) {
						s.bot.ctx.Logger.Info("Leecher mode: COME received, resuming following",
							slog.String("supervisor", s.name))
						SetLeecherStayReceived(s.name, false)
						SetLeecherComeReceived(s.name, false)
						// Continue to following logic below
					} else {
						s.bot.ctx.Logger.Debug("Leecher mode: STAY active, not following leader",
							slog.String("supervisor", s.name))
						// Don't follow - just wait in place
						continue
					}
				}

				// Follow the leader at max distance for XP
				s.bot.ctx.Logger.Debug("Leecher mode: attempting to follow leader",
					slog.String("supervisor", s.name),
					slog.String("leader", leecherInfo.LeaderName),
					slog.Int("maxDistance", leecherInfo.MaxLeaderDistance))

				if err := action.FollowPlayer(leecherInfo.LeaderName, leecherInfo.MaxLeaderDistance); err != nil {
					// Check if leader moved to a different area
					s.bot.ctx.RefreshGameData()
					myArea := s.bot.ctx.Data.PlayerUnit.Area
					myPos := s.bot.ctx.Data.PlayerUnit.Position
					
					// Get fresh leader position
					leaderPos, currentLeaderArea, leaderFound := action.GetLeaderPosition(leecherInfo.LeaderName)
					if !leaderFound {
						s.bot.ctx.Logger.Debug("Leecher mode: leader not found after follow error",
							slog.String("supervisor", s.name))
						continue
					}
					leaderArea = currentLeaderArea
					
				// Calculate distance to leader
				dx := float64(myPos.X - leaderPos.X)
				dy := float64(myPos.Y - leaderPos.Y)
				distanceToLeader := int(dx*dx + dy*dy)
				
				// Check if leader is in town while we're in dungeon
				// This happens when leader TPs back - we should wait for explicit command
				if leaderArea.IsTown() {
					s.bot.ctx.Logger.Debug("Leecher mode: leader in town while we're in dungeon, waiting for command",
						slog.String("supervisor", s.name),
						slog.String("leaderArea", leaderArea.Area().Name))
					// Don't try to follow or navigate - just wait
					continue
				}
				
				// If leader is very close (within ~100 units) even if different area ID, it might be same physical area
				// This handles cases like Tal Rasha's Tombs where each tomb has a different area ID
				const closeProximityThresholdSq = 10000 // 100^2
				if distanceToLeader < closeProximityThresholdSq {
					s.bot.ctx.Logger.Debug("Leecher mode: leader close by despite different area ID, continuing to follow",
						slog.String("supervisor", s.name),
						slog.Int("distanceSq", distanceToLeader))
					continue
				}
					
					if leaderArea != myArea {
						s.bot.ctx.Logger.Info("Leecher mode: leader in different area",
							slog.String("supervisor", s.name),
							slog.String("myArea", myArea.Area().Name),
							slog.String("leaderArea", leaderArea.Area().Name),
							slog.Int("distanceSq", distanceToLeader))

						// Keep following through adjacent areas until we reach the leader or they're no longer adjacent
						const maxAdjacentHops = 10 // Safety limit to prevent infinite loops
						adjacentHops := 0
						
						for adjacentHops < maxAdjacentHops {
							// Refresh game data to get current state
							s.bot.ctx.RefreshGameData()
							myArea = s.bot.ctx.Data.PlayerUnit.Area
							
							// Check if we've reached the leader's area
							if myArea == leaderArea {
								s.bot.ctx.Logger.Info("Leecher mode: reached leader's area",
									slog.String("supervisor", s.name),
									slog.String("area", myArea.Area().Name))
								break
							}
							
							// Get updated leader position
							_, newLeaderArea, leaderFound := action.GetLeaderPosition(leecherInfo.LeaderName)
							if !leaderFound {
								s.bot.ctx.Logger.Debug("Leecher mode: leader not found in roster during area navigation",
									slog.String("supervisor", s.name))
								break
							}
							leaderArea = newLeaderArea
							
							// Check if leader's current area is adjacent to us
							isAdjacent := false
							for _, adj := range s.bot.ctx.Data.AdjacentLevels {
								if adj.Area == leaderArea {
									isAdjacent = true
									break
								}
							}

							if isAdjacent {
								s.bot.ctx.Logger.Info("Leecher mode: moving to adjacent area",
									slog.String("supervisor", s.name),
									slog.String("targetArea", leaderArea.Area().Name),
									slog.Int("hop", adjacentHops+1))

								// Move to adjacent area
								if err := action.MoveToArea(leaderArea); err != nil {
									s.bot.ctx.Logger.Debug("Leecher mode: error moving to area",
										slog.String("supervisor", s.name),
										slog.String("error", err.Error()))
									break
								}
								
								adjacentHops++
								utils.Sleep(500) // Brief pause after area transition
							} else {
								// Leader is not in an adjacent area - they used waypoint/portal
								// Return to town and wait for commands
								s.bot.ctx.Logger.Info("Leecher mode: leader not in adjacent area, returning to town",
									slog.String("supervisor", s.name),
									slog.String("myArea", myArea.Area().Name),
									slog.String("leaderArea", leaderArea.Area().Name))

								if err := action.ReturnTown(); err != nil {
									s.bot.ctx.Logger.Warn("Leecher mode: failed to return to town with TP",
										slog.String("supervisor", s.name),
										slog.String("error", err.Error()))
									
									// Recovery: Try to find and use any nearby portal (leader's or our old one)
									s.bot.ctx.Logger.Info("Leecher mode: attempting portal recovery",
										slog.String("supervisor", s.name))
									if portalErr := action.ReturnThroughPortal(); portalErr != nil {
										s.bot.ctx.Logger.Warn("Leecher mode: portal recovery also failed, staying in dungeon",
											slog.String("supervisor", s.name),
											slog.String("error", portalErr.Error()))
										// Stay in dungeon and keep trying to follow - leader might come back
										continue
									}
									
									// Successfully used a portal
									s.bot.ctx.Logger.Info("Leecher mode: portal recovery successful",
										slog.String("supervisor", s.name))
									inDungeon = false
									SetLeecherState(s.name, LeecherStateInPortalArea)
								} else {
									inDungeon = false
									SetLeecherState(s.name, LeecherStateInPortalArea)
									SetLeecherComeReceived(s.name, false)

									// Go to portal waiting area
									if err := action.WaitAtPortalArea(); err != nil {
										s.bot.ctx.Logger.Warn("Leecher mode: failed to go to portal area",
											slog.String("supervisor", s.name),
											slog.String("error", err.Error()))
									}
								}
								break
							}
						}
						
						if adjacentHops >= maxAdjacentHops {
							s.bot.ctx.Logger.Warn("Leecher mode: reached max adjacent hops, stopping navigation",
								slog.String("supervisor", s.name))
						}
					} else {
						// Leader is in the same area but FollowPlayer failed for another reason
						// Check if it's a TP scroll issue and if leader is close
						if strings.Contains(err.Error(), "town portal") || strings.Contains(err.Error(), "TP") {
							s.bot.ctx.Logger.Info("Leecher mode: out of TP scrolls, checking for nearby portal or close leader",
								slog.String("supervisor", s.name))
							
							// Check if leader is very close - if so, keep following without TP
							if distanceToLeader < closeProximityThresholdSq {
								s.bot.ctx.Logger.Info("Leecher mode: leader still close, continuing to follow despite TP issue",
									slog.String("supervisor", s.name),
									slog.Int("distanceSq", distanceToLeader))
								continue
							}
							
							// Try to find and use any nearby portal
							s.bot.ctx.Logger.Info("Leecher mode: attempting to use nearby portal",
								slog.String("supervisor", s.name))
							if portalErr := action.ReturnThroughPortal(); portalErr != nil {
								s.bot.ctx.Logger.Warn("Leecher mode: no portal found, will wait here",
									slog.String("supervisor", s.name))
								// Just wait here - leader might come back or open a portal
								continue
							}
							
							// Successfully used a portal - return to town
							s.bot.ctx.Logger.Info("Leecher mode: used portal to return to town",
								slog.String("supervisor", s.name))
							inDungeon = false
							SetLeecherState(s.name, LeecherStateInPortalArea)
							
							// Go to portal waiting area
							if err := action.WaitAtPortalArea(); err != nil {
								s.bot.ctx.Logger.Warn("Leecher mode: failed to go to portal area",
									slog.String("supervisor", s.name),
									slog.String("error", err.Error()))
							}
							continue
						}
						
						// Log the error and try again next tick
						s.bot.ctx.Logger.Warn("Leecher mode: failed to follow leader in same area",
							slog.String("supervisor", s.name),
							slog.String("error", err.Error()),
							slog.String("area", myArea.Area().Name))
					}
				} else {
					// Successfully following leader (or already in range)
					s.bot.ctx.Logger.Debug("Leecher mode: following leader successfully",
						slog.String("supervisor", s.name))
				}
				continue
			}

			// Not in dungeon - waiting at portal area
			SetLeecherState(s.name, LeecherStateInPortalArea)

			// Check if "come" command was received
			comeReceived := GetLeecherComeReceived(s.name)
			s.bot.ctx.Logger.Debug("Leecher mode: polling for commands",
				slog.String("supervisor", s.name),
				slog.Bool("comeReceived", comeReceived))

			if comeReceived {
				s.bot.ctx.Logger.Info("Leecher mode: COME command received",
					slog.String("supervisor", s.name))

				// Reset STAY flag when COME is received
				SetLeecherStayReceived(s.name, false)

				// Wait portal entry delay
				if leecherInfo.PortalEntryDelay > 0 {
					s.bot.ctx.Logger.Info("Leecher mode: waiting before entering portal",
						slog.String("supervisor", s.name),
						slog.Int("delaySec", leecherInfo.PortalEntryDelay))
					if !s.sleepWithContextCheck(ctx, leecherInfo.PortalEntryDelay*1000) {
						return nil
					}
				}

				// Find and enter leader's portal
				_, found := action.FindLeaderPortal(leecherInfo.LeaderName)
				if !found {
					s.bot.ctx.Logger.Warn("Leecher mode: no portal found to enter",
						slog.String("supervisor", s.name))
					SetLeecherComeReceived(s.name, false) // Reset for retry
					continue
				}

				if err := action.EnterPlayerPortal(leecherInfo.LeaderName); err != nil {
					s.bot.ctx.Logger.Warn("Leecher mode: failed to enter portal",
						slog.String("supervisor", s.name),
						slog.String("error", err.Error()))
					SetLeecherComeReceived(s.name, false) // Reset for retry
					continue
				}

				s.bot.ctx.Logger.Info("Leecher mode: entered portal, now following leader",
					slog.String("supervisor", s.name))
				inDungeon = true
				SetLeecherState(s.name, LeecherStateFollowing)

				// Refresh game data after portal entry to ensure area is synced
				s.bot.ctx.RefreshGameData()

				// Record the entry portal position so we can detect NEW portals later
				if dungeonPortal, found := action.FindLeaderPortal(leecherInfo.LeaderName); found {
					entryPortalPos = dungeonPortal.Position
					s.bot.ctx.Logger.Debug("Leecher mode: recorded entry portal position",
						slog.Int("x", entryPortalPos.X),
						slog.Int("y", entryPortalPos.Y))
				}
				
				// Wait a moment for area transition to stabilize
				utils.Sleep(1000)
			}
		}
	}
}
