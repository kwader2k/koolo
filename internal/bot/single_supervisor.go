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
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	ct "github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/drop"
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

func (s *SinglePlayerSupervisor) shouldSkipKeybindingsForRespec() bool {
	ctx := s.bot.ctx
	if ctx == nil || ctx.CharacterCfg == nil {
		return false
	}
	if _, isLevelingChar := ctx.Char.(ct.LevelingCharacter); isLevelingChar {
		return false
	}

	autoCfg := ctx.CharacterCfg.Character.AutoStatSkill
	if !autoCfg.Enabled || !autoCfg.Respec.Enabled || autoCfg.Respec.Applied {
		return false
	}
	if autoCfg.Respec.TargetLevel == 0 {
		return true
	}

	level, ok := ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
	return ok && level.Value == autoCfg.Respec.TargetLevel
}

// Start will return error if it can be started, otherwise will always return nil
func (s *SinglePlayerSupervisor) Start() error {
	ctx, cancel := context.WithCancel(context.Background())
	s.cancelFn = cancel

	err := s.ensureProcessIsRunningAndPrepare()
	if err != nil {
		return fmt.Errorf("error preparing game: %w", err)
	}

	if err := s.ensureSkillKeyBindingsReady(); err != nil {
		return err
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
			timeSpentNotInGameStart = time.Now()
			continue
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
		s.bot.ctx.LastCastAt = time.Time{}
		s.logGameStart(runs)
		s.bot.ctx.RefreshGameData()

		// Dump armory data on game start
		if err := s.dumpArmory(); err != nil {
			s.bot.ctx.Logger.Warn("Failed to dump armory data", slog.Any("error", err))
		}

		if config.Koolo.Debug.OpenOverlayMapOnGameStart {
			automapKB := s.bot.ctx.Data.KeyBindings.Automap
			if automapKB.Key1[0] != 0 || automapKB.Key2[0] != 0 {
				s.bot.ctx.HID.PressKeyBinding(automapKB)
				utils.PingSleep(utils.Light, 50)
			} else {
				s.bot.ctx.Logger.Debug("Open overlay map on game start is enabled, but no automap key binding is set")
			}
		}

		if s.bot.ctx.Data.IsLevelingCharacter && s.bot.ctx.Data.ActiveWeaponSlot != 0 {
			for attempt := 0; attempt < 3 && s.bot.ctx.Data.ActiveWeaponSlot != 0; attempt++ {
				s.bot.ctx.HID.PressKeyBinding(s.bot.ctx.Data.KeyBindings.SwapWeapons)
				utils.PingSleep(utils.Light, 150)
				s.bot.ctx.RefreshGameData()
			}
			if s.bot.ctx.Data.ActiveWeaponSlot != 0 {
				s.bot.ctx.Logger.Warn("Failed to return to main weapon slot after game start", "slot", s.bot.ctx.Data.ActiveWeaponSlot)
			}
		}

		if s.bot.ctx.CharacterCfg.Companion.Enabled && s.bot.ctx.CharacterCfg.Companion.Leader {
			event.Send(event.RequestCompanionJoinGame(event.Text(s.name, "New Game Started "+s.bot.ctx.Data.Game.LastGameName), s.bot.ctx.CharacterCfg.CharacterName, s.bot.ctx.Data.Game.LastGameName, s.bot.ctx.Data.Game.LastGamePassword))
		}

		if firstRun {
			if s.shouldSkipKeybindingsForRespec() {
				s.bot.ctx.Logger.Info("Auto respec pending; skipping keybinding check for this run")
			} else {
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
		}

		action.EnsureRunMode()

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
					lastAction := s.bot.ctx.ContextDebug[s.bot.ctx.ExecutionPriority].LastAction

					// Check for stat/skill allocation activities
					isAllocating := lastAction == "AutoRespecIfNeeded" ||
						lastAction == "EnsureStatPoints" ||
						lastAction == "EnsureSkillPoints" ||
						lastAction == "EnsureSkillBindings" ||
						lastAction == "AllocateStatPointPacket" ||
						lastAction == "LearnSkillPacket"
					if isAllocating && (s.bot.ctx.Data.OpenMenus.Character || s.bot.ctx.Data.OpenMenus.SkillTree || s.bot.ctx.Data.OpenMenus.Inventory) {
						stuckSince = time.Time{}
						droppedMouseItem = false
						lastPosition = currentPos
						continue
					}

					// Check for cube transmutation activities (player is stationary but actively working)
					// Cube activities involve opening cube, stash, moving items between them
					isCubing := lastAction == "CubeRecipes" || lastAction == "CubeAddItems" ||
						lastAction == "SocketAddItems" || strings.Contains(lastAction, "Cube")
					cubeMenuOpen := s.bot.ctx.Data.OpenMenus.Cube || s.bot.ctx.Data.OpenMenus.Stash
					if isCubing && cubeMenuOpen {
						stuckSince = time.Time{}
						droppedMouseItem = false
						lastPosition = currentPos
						continue
					}

					// Also reset stuck timer if cube or stash is open (player might be actively managing items)
					if s.bot.ctx.Data.OpenMenus.Cube {
						stuckSince = time.Time{}
						droppedMouseItem = false
						lastPosition = currentPos
						continue
					}

					// Check for gambling activities (player is stationary at vendor)
					isGambling := lastAction == "Gamble" || lastAction == "GambleSingleItem" || lastAction == "gambleItems"
					if isGambling && s.bot.ctx.Data.OpenMenus.NPCShop {
						stuckSince = time.Time{}
						droppedMouseItem = false
						lastPosition = currentPos
						continue
					}
					if currentPos.X == lastPosition.X && currentPos.Y == lastPosition.Y {
						if stuckSince.IsZero() {
							stuckSince = time.Now()
							droppedMouseItem = false // Reset flag when first detecting stuck
						}

						stuckDuration := time.Since(stuckSince)

						// After 90 seconds stuck, try dropping mouse item
						if stuckDuration > 90*time.Second {
							if len(s.bot.ctx.Data.Inventory.ByLocation(item.LocationCursor)) > 0 && !droppedMouseItem {
								s.bot.ctx.Logger.Warn("Player stuck for 90 seconds - Clicking to drop mouse item - Continuing to monitor for movement...")
								s.bot.ctx.HID.Click(game.LeftButton, 500, 500)
								droppedMouseItem = true
							} else if s.bot.ctx.IsAllocatingStatsOrSkills.Load() {
								// We don't want a false positive on being stuck when the character is respeccing
								s.bot.ctx.Logger.Debug("Player stuck for 90 seconds - Currently respeccing - letting it continue.")
								stuckSince = time.Now()
							} else if droppedMouseItem {
								s.bot.ctx.Logger.Warn("Player still stuck after dropping the item - Forcing client restart.")
								if err := s.KillClient(); err != nil {
									s.bot.ctx.Logger.Error(fmt.Sprintf("Activity monitor failed to kill client: %v", err))
								}
								runCancel()
								return
							}
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
			if errors.Is(err, drop.ErrInterrupt) {
				s.bot.ctx.Logger.Info("Drop interrupt received. Exiting game and restarting loop.")
				s.bot.ctx.Manager.ExitGame()
				utils.Sleep(2000)
				timeSpentNotInGameStart = time.Now()
				continue
			}
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
			s.bot.ctx.GameReader.ClearMapData() // Free map data memory while not in game
			s.bot.ctx.Data.Areas = nil          // Clear context's map reference to allow GC
			s.bot.ctx.Data.AreaData = game.AreaData{}
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
		s.bot.ctx.GameReader.ClearMapData() // Free map data memory while not in game
		s.bot.ctx.Data.Areas = nil          // Clear context's map reference to allow GC
		s.bot.ctx.Data.AreaData = game.AreaData{}
		timeSpentNotInGameStart = time.Now()
	}
}

func (s *SinglePlayerSupervisor) ensureSkillKeyBindingsReady() error {
	cfg := s.bot.ctx.CharacterCfg
	if cfg == nil {
		s.bot.ctx.Logger.Debug("Skipping key binding check: character config is nil")
		return nil
	}
	characterName := strings.TrimSpace(cfg.CharacterName)
	if characterName == "" {
		s.bot.ctx.Logger.Debug("Skipping key binding check: character name is empty")
		return nil
	}
	if s.bot.ctx.ManualModeActive {
		s.bot.ctx.Logger.Debug("Skipping key binding check: manual mode active")
		return nil
	}
	if s.bot.ctx.Manager.InGame() {
		s.bot.ctx.Logger.Debug("Skipping key binding check: already in game")
		return nil
	}

	kbResult, kbErr := config.EnsureSkillKeyBindings(cfg, config.Koolo.UseCustomSettings)
	if kbErr != nil {
		s.bot.ctx.Logger.Warn("Failed to ensure skill key bindings", slog.Any("error", kbErr))
	}

	if kbResult.Updated {
		s.bot.ctx.Logger.Info("Skill key bindings updated on disk; restarting client to apply")
		if killErr := s.KillClient(); killErr != nil {
			return killErr
		}
		return ErrUnrecoverableClientState
	}

	if !kbResult.Missing {
		return nil
	}

	s.bot.ctx.Logger.Info("Key binding file missing; entering a game to bootstrap", slog.String("character", characterName))

	if err := s.waitUntilCharacterSelectionScreen(); err != nil {
		return err
	}
	enterDeadline := time.Now().Add(2 * time.Minute)
	for !s.bot.ctx.Manager.InGame() {
		if time.Now().After(enterDeadline) {
			return fmt.Errorf("timed out entering game for key binding bootstrap")
		}
		if err := s.HandleMenuFlow(); err != nil {
			if errors.Is(err, ErrUnrecoverableClientState) {
				return err
			}
			if err.Error() == "loading screen" || err.Error() == "" || err.Error() == "idle" {
				utils.Sleep(100)
				continue
			}
			return err
		}
		utils.Sleep(100)
	}

	if waitErr := config.WaitForKeyBindings(kbResult.SaveDir, characterName, cfg.AuthMethod, 45*time.Second); waitErr != nil {
		s.bot.ctx.Logger.Warn("Timed out waiting for key binding file", slog.Any("error", waitErr))
	}

	kbResult, kbErr = config.EnsureSkillKeyBindings(cfg, config.Koolo.UseCustomSettings)
	if kbErr != nil {
		s.bot.ctx.Logger.Warn("Failed to ensure skill key bindings after bootstrap", slog.Any("error", kbErr))
	}

	if s.bot.ctx.Manager.InGame() {
		if exitErr := s.bot.ctx.Manager.ExitGame(); exitErr != nil {
			s.bot.ctx.Logger.Warn("Failed to exit game after key binding bootstrap", slog.Any("error", exitErr))
		} else {
			exitDeadline := time.Now().Add(15 * time.Second)
			for s.bot.ctx.Manager.InGame() && time.Now().Before(exitDeadline) {
				utils.Sleep(250)
				s.bot.ctx.RefreshGameData()
			}
		}
	}

	if kbResult.Updated {
		s.bot.ctx.Logger.Info("Skill key bindings updated on disk; restarting client to apply")
		if killErr := s.KillClient(); killErr != nil {
			return killErr
		}
		return ErrUnrecoverableClientState
	}

	if kbResult.Missing {
		s.bot.ctx.Logger.Warn("Key binding file still missing after bootstrap", slog.String("character", characterName))
	}

	return nil
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

// dumpArmory saves the current character inventory state to a JSON file
func (s *SinglePlayerSupervisor) dumpArmory() error {
	if s.bot.ctx.Data == nil {
		return fmt.Errorf("game data not available")
	}

	gameName := s.bot.ctx.GameReader.LastGameName()
	return dumpArmoryData(s.name, s.bot.ctx.Data, gameName)
}
