package bot

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"sync"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	botCtx "github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/drop"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/health"
	"github.com/hectorgimenez/koolo/internal/run"
	"github.com/hectorgimenez/koolo/internal/utils"
	"golang.org/x/sync/errgroup"
)

type Bot struct {
	ctx                   *botCtx.Context
	lastActivityTimeMux   sync.Mutex
	lastActivityTime      time.Time
	lastKnownPosition     data.Position
	lastPositionCheckTime time.Time
	MuleManager
}

func (b *Bot) NeedsTPsToContinue() bool {
	return !action.HasTPsAvailable()
}

func (b *Bot) shouldReturnToTown(lvl int, needHealingPotionsRefill, needManaPotionsRefill, townChicken bool) bool {
	if (b.ctx.Data.PlayerUnit.TotalPlayerGold() > 500 && lvl <= 5) ||
		(b.ctx.Data.PlayerUnit.TotalPlayerGold() > 1000 && lvl < 20) ||
		(b.ctx.Data.PlayerUnit.TotalPlayerGold() > 5000 && lvl >= 20) {
		if (b.ctx.CharacterCfg.BackToTown.NoHpPotions && needHealingPotionsRefill ||
			b.ctx.CharacterCfg.BackToTown.EquipmentBroken && action.IsEquipmentBroken() ||
			b.ctx.CharacterCfg.BackToTown.NoMpPotions && needManaPotionsRefill ||
			townChicken ||
			b.ctx.CharacterCfg.BackToTown.MercDied &&
				b.ctx.Data.MercHPPercent() <= 0 &&
				b.ctx.CharacterCfg.Character.UseMerc &&
				b.ctx.Data.PlayerUnit.TotalPlayerGold() > 100000) &&
			!b.ctx.Data.PlayerUnit.Area.IsTown() &&
			b.ctx.Data.PlayerUnit.Area != area.UberTristram {
			return true
		}
	}

	return false
}

func NewBot(ctx *botCtx.Context, mm MuleManager) *Bot {
	return &Bot{
		ctx:                   ctx,
		lastActivityTime:      time.Now(),      // Initialize
		lastKnownPosition:     data.Position{}, // Will be updated on first game data refresh
		lastPositionCheckTime: time.Now(),      // Initialize
		MuleManager:           mm,
	}
}

func (b *Bot) updateActivityAndPosition() {
	b.lastActivityTimeMux.Lock()
	defer b.lastActivityTimeMux.Unlock()
	b.lastActivityTime = time.Now()
	// Update lastKnownPosition and lastPositionCheckTime only if current game data is valid
	if b.ctx.Data.PlayerUnit.Position != (data.Position{}) {
		b.lastKnownPosition = b.ctx.Data.PlayerUnit.Position
		b.lastPositionCheckTime = time.Now()
	}
}

// getActivityData returns the activity-related data in a thread-safe manner.
func (b *Bot) getActivityData() (time.Time, data.Position, time.Time) {
	b.lastActivityTimeMux.Lock()
	defer b.lastActivityTimeMux.Unlock()
	return b.lastActivityTime, b.lastKnownPosition, b.lastPositionCheckTime
}

func (b *Bot) Run(ctx context.Context, firstRun bool, runs []run.Run) error {
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()
	g, ctx := errgroup.WithContext(ctx)

	gameStartedAt := time.Now()
	b.ctx.SwitchPriority(botCtx.PriorityNormal) // Restore priority to normal, in case it was stopped in previous game
	b.ctx.CurrentGame = botCtx.NewGameHelper()  // Reset current game helper structure
	// Drop: Initialize Drop manager and start watch context
	if b.ctx.Drop == nil {
		b.ctx.Drop = drop.NewManager(b.ctx.Name, b.ctx.Logger)
	}

	err := b.ctx.GameReader.FetchMapData()
	if err != nil {
		return err
	}

	// Let's make sure we have updated game data also fully loaded before performing anything
	b.ctx.WaitForGameToLoad()

	// Cleanup the current game helper structure
	b.ctx.Cleanup()

	// Switch to legacy mode if configured
	action.SwitchToLegacyMode()
	b.ctx.RefreshGameData()

	b.updateActivityAndPosition() // Initial update for activity and position

	// This routine is in charge of refreshing the game data and handling cancellation, will work in parallel with any other execution
	g.Go(func() error {
		b.ctx.AttachRoutine(botCtx.PriorityBackground)
		ticker := time.NewTicker(100 * time.Millisecond)
		for {
			select {
			case <-ctx.Done():
				cancel()
				b.Stop()
				return nil
			case <-ticker.C:
				if b.ctx.ExecutionPriority == botCtx.PriorityPause {
					continue
				}
				b.ctx.RefreshGameData()
				// Update activity here because the bot is actively refreshing game data.
				b.updateActivityAndPosition()
			}
		}
	})

	// This routine is in charge of handling the health/chicken of the bot, will work in parallel with any other execution
	g.Go(func() error {
		b.ctx.AttachRoutine(botCtx.PriorityBackground)
		ticker := time.NewTicker(100 * time.Millisecond)

		const globalLongTermIdleThreshold = 2 * time.Minute // From move.go example
		const minMovementThreshold = 30                     // From move.go example

		for {
			select {
			case <-ctx.Done():
				b.Stop()
				return nil
			case <-ticker.C:
				if b.ctx.ExecutionPriority == botCtx.PriorityPause {
					continue
				}
				if b.ctx.Drop != nil && (b.ctx.Drop.Pending() != nil || b.ctx.Drop.Active() != nil) {
					// Skip health handling while Drop run takes over (character may be out of game)
					continue
				}

				err = b.ctx.HealthManager.HandleHealthAndMana()
				if err != nil {
					b.ctx.Logger.Info("HealthManager: Detected critical error (chicken/death), stopping bot.", "error", err.Error())
					cancel()
					b.Stop()
					return err
				}

				// Always update activity when HealthManager runs, as it signifies process activity
				b.updateActivityAndPosition()

				// Retrieve current activity data in a thread-safe manner
				_, lastKnownPos, lastPosCheckTime := b.getActivityData()
				currentPosition := b.ctx.Data.PlayerUnit.Position

				// Check for position-based long-term idle
				if currentPosition != (data.Position{}) && lastKnownPos != (data.Position{}) { // Ensure valid positions
					distanceFromLastKnown := utils.CalculateDistance(lastKnownPos, currentPosition)

					if distanceFromLastKnown > float64(minMovementThreshold) {
						// Player has moved significantly, reset position-based idle timer
						b.updateActivityAndPosition() // This will update lastKnownPosition and lastPositionCheckTime
						b.ctx.Logger.Debug(fmt.Sprintf("Bot: Player moved significantly (%.2f units), resetting global idle timer.", distanceFromLastKnown))
					} else if time.Since(lastPosCheckTime) > globalLongTermIdleThreshold {
						// Player hasn't moved much for the long-term threshold, quit the game
						b.ctx.Logger.Error(fmt.Sprintf("Bot: Player has been globally idle (no significant movement) for more than %v, quitting game.", globalLongTermIdleThreshold))
						b.Stop()
						return errors.New("bot globally idle for too long (no movement), quitting game")
					}
				} else {
					// If for some reason positions are invalid, just update activity to prevent immediate idle.
					// This handles initial states or temporary data glitches.
					b.updateActivityAndPosition()
				}

				// Check for max game length (this is a separate check from idle)
				if time.Since(gameStartedAt).Seconds() > float64(b.ctx.CharacterCfg.MaxGameLength) {
					b.ctx.Logger.Info("Max game length reached, try to exit game", slog.Float64("duration", time.Since(gameStartedAt).Seconds()))
					b.Stop() // This will set PriorityStop and detach the context
					return fmt.Errorf(
						"max game length reached, try to exit game: %0.2f",
						time.Since(gameStartedAt).Seconds(),
					)
				}
			}
		}
	})

	// High priority loop, this will interrupt (pause) low priority loop
	g.Go(func() error {
		defer func() {
			cancel()
			b.Stop()
			recover()
		}()

		b.ctx.AttachRoutine(botCtx.PriorityHigh)
		ticker := time.NewTicker(time.Millisecond * 100)

		for {
			select {
			case <-ctx.Done():
				return nil
			case <-ticker.C:
				if b.ctx.ExecutionPriority == botCtx.PriorityPause {
					continue
				}

				if b.ctx.Drop != nil && (b.ctx.Drop.Pending() != nil || b.ctx.Drop.Active() != nil) {
					// Drop is in progress, skip high-priority actions until handled
					continue
				}

				// Update activity for high-priority actions as they indicate bot is processing.
				b.updateActivityAndPosition()

				// Merc check (Fast)
				if b.ctx.CharacterCfg.BackToTown.MercDied && b.ctx.Data.MercHPPercent() <= 0 && b.ctx.CharacterCfg.Character.UseMerc {
					time.Sleep(200 * time.Millisecond)
				}

				// Legacy/Portrait/Chat checks (Fast, Read-only/Input-gated)
				if b.ctx.CharacterCfg.ClassicMode && !b.ctx.Data.LegacyGraphics {
					action.SwitchToLegacyMode()
					time.Sleep(150 * time.Millisecond)
				}
				// Hide merc/other players portraits if enabled
				if b.ctx.CharacterCfg.HidePortraits && b.ctx.Data.OpenMenus.PortraitsShown {
					action.HidePortraits()
					time.Sleep(150 * time.Millisecond)
				}
				// Close chat if somehow was opened (prevention)
				if b.ctx.Data.OpenMenus.ChatOpen {
					b.ctx.HID.PressKey(b.ctx.Data.KeyBindings.Chat.Key1[0])
					time.Sleep(150 * time.Millisecond)
				}

				// Max Level Check (Fast, Read-only)
				lvl, _ := b.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
				MaxLevel := b.ctx.CharacterCfg.Game.StopLevelingAt
				if lvl.Value >= MaxLevel && MaxLevel > 0 {
					b.ctx.Logger.Info(fmt.Sprintf("Player reached level %d (>= MaxLevel %d). Triggering supervisor stop via context.", lvl.Value, MaxLevel), "run", "Leveling")
					b.ctx.StopSupervisor()
					return nil
				}

				// Check-Then-Lock Pattern
				// We pre-calculate if we need to switch priority to High.
				// This prevents locking the main thread (Low Priority Loop) when there is nothing to do.

				shouldPickup := false
				if b.ctx.CurrentGame.PickupItems {
					// Peek if there are items without locking
					if len(action.GetItemsToPickup(30)) > 0 {
						shouldPickup = true
					}
				}

				shouldBuff := action.IsRebuffRequired()

				_, healingPotionsFoundInBelt := b.ctx.Data.Inventory.Belt.GetFirstPotion(data.HealingPotion)
				_, manaPotionsFoundInBelt := b.ctx.Data.Inventory.Belt.GetFirstPotion(data.ManaPotion)
				_, rejuvPotionsFoundInBelt := b.ctx.Data.Inventory.Belt.GetFirstPotion(data.RejuvenationPotion)

				// Check potions in inventory
				hasHealingPotionsInInventory := b.ctx.Data.HasPotionInInventory(data.HealingPotion)
				hasManaPotionsInInventory := b.ctx.Data.HasPotionInInventory(data.ManaPotion)
				hasRejuvPotionsInInventory := b.ctx.Data.HasPotionInInventory(data.RejuvenationPotion)

				needHealingPotionsRefill := !healingPotionsFoundInBelt && b.ctx.CharacterCfg.Inventory.BeltColumns.Total(data.HealingPotion) > 0
				needManaPotionsRefill := !manaPotionsFoundInBelt && b.ctx.CharacterCfg.Inventory.BeltColumns.Total(data.ManaPotion) > 0
				needRejuvPotionsRefill := !rejuvPotionsFoundInBelt && b.ctx.CharacterCfg.Inventory.BeltColumns.Total(data.RejuvenationPotion) > 0

				// Determine if we should refill for each type based on availability in inventory
				shouldRefillHealingPotions := needHealingPotionsRefill && hasHealingPotionsInInventory
				shouldRefillManaPotions := needManaPotionsRefill && hasManaPotionsInInventory
				shouldRefillRejuvPotions := needRejuvPotionsRefill && hasRejuvPotionsInInventory

				shouldRefillBelt := ((shouldRefillHealingPotions || healingPotionsFoundInBelt) &&
					(shouldRefillManaPotions || manaPotionsFoundInBelt) &&
					(needHealingPotionsRefill || needManaPotionsRefill)) || shouldRefillRejuvPotions

				isInTown := b.ctx.Data.PlayerUnit.Area.IsTown()
				if isInTown {
					shouldRefillBelt = false
				}

				shouldReturnTown := false
				townChicken := b.ctx.CharacterCfg.Health.TownChickenAt > 0 && b.ctx.Data.PlayerUnit.HPPercent() <= b.ctx.CharacterCfg.Health.TownChickenAt

				if _, found := b.ctx.Data.KeyBindings.KeyBindingForSkill(skill.TomeOfTownPortal); found {
					if !b.NeedsTPsToContinue() {
						shouldReturnTown = b.shouldReturnToTown(lvl.Value, needHealingPotionsRefill, needManaPotionsRefill, townChicken)
					}
				}

				shouldCorrectArea := b.ctx.CurrentGame.AreaCorrection.Enabled

				// Action Execution
				// Only switch to High Priority if we actually have work to do.
				if shouldPickup || shouldBuff || shouldRefillBelt || shouldReturnTown || shouldCorrectArea {
					b.ctx.SwitchPriority(botCtx.PriorityHigh)

					// Execute Area Correction
					if shouldCorrectArea {
						if err = action.AreaCorrection(); err != nil {
							b.ctx.Logger.Warn("Area correction failed", "error", err)
						}
					}

					// Execute Pickup
					if shouldPickup {
						action.ItemPickup(30)
					}

					// Execute Buff
					if shouldBuff {
						action.BuffIfRequired()
					}

					// Execute Belt Refill
					if shouldRefillBelt && !isInTown {
						// Double check condition inside lock if needed, but usually safe to run
						action.ManageBelt()
						action.RefillBeltFromInventory()

						if shouldReturnTown {
							b.ctx.RefreshGameData()
							_, healingPotionsFoundInBelt = b.ctx.Data.Inventory.Belt.GetFirstPotion(data.HealingPotion)
							_, manaPotionsFoundInBelt = b.ctx.Data.Inventory.Belt.GetFirstPotion(data.ManaPotion)
							needHealingPotionsRefill = !healingPotionsFoundInBelt && b.ctx.CharacterCfg.Inventory.BeltColumns.Total(data.HealingPotion) > 0
							needManaPotionsRefill = !manaPotionsFoundInBelt && b.ctx.CharacterCfg.Inventory.BeltColumns.Total(data.ManaPotion) > 0
							shouldReturnTown = b.shouldReturnToTown(lvl.Value, needHealingPotionsRefill, needManaPotionsRefill, townChicken)
						}
					}

					// Execute Town Return
					if shouldReturnTown {
						// Log the exact reason for going back to town
						var reason string
						if b.ctx.CharacterCfg.BackToTown.NoHpPotions && needHealingPotionsRefill {
							reason = "No healing potions found"
						} else if b.ctx.CharacterCfg.BackToTown.EquipmentBroken && action.RepairRequired() {
							reason = "Equipment broken"
						} else if b.ctx.CharacterCfg.BackToTown.NoMpPotions && needManaPotionsRefill {
							reason = "No mana potions found"
						} else if b.ctx.CharacterCfg.BackToTown.MercDied && b.ctx.Data.MercHPPercent() <= 0 && b.ctx.CharacterCfg.Character.UseMerc {
							reason = "Mercenary is dead"
						} else if townChicken {
							reason = "Town chicken"
						}

						b.ctx.Logger.Info("Going back to town", "reason", reason)

						if err = action.InRunReturnTownRoutine(); err != nil {
							b.ctx.Logger.Warn("Failed returning town. Returning error to stop game.", "error", err)
							return err
						}
					}

					b.ctx.SwitchPriority(botCtx.PriorityNormal)
				}
			}
		}
	})

	// Low priority loop, this will keep executing main run scripts
	g.Go(func() (returnErr error) {
		defer func() {
			cancel()
			b.Stop()
			if r := recover(); r != nil {
				if e, ok := r.(error); ok && errors.Is(e, health.ErrChicken) {
					returnErr = e
				}
			}
		}()

		b.ctx.AttachRoutine(botCtx.PriorityNormal)
		for _, r := range runs {
			select {
			case <-ctx.Done():
				return nil
			default:
				skipTownRoutines := false
				if skipper, ok := r.(run.TownRoutineSkipper); ok && skipper.SkipTownRoutines() {
					skipTownRoutines = true
				}

				event.Send(event.RunStarted(event.Text(b.ctx.Name, fmt.Sprintf("Starting run: %s", r.Name())), r.Name()))

				// Update activity here because a new run sequence is starting.
				b.updateActivityAndPosition()

				if !skipTownRoutines {
					err = action.PreRun(firstRun)
					if err != nil {
						return err
					}
					firstRun = false
				}

				// Update activity before the main run logic is executed.
				b.updateActivityAndPosition()
				err = r.Run(nil)

				// Drop: Handle Drop interrupt from step functions
				if errors.Is(err, drop.ErrInterrupt) {
					b.ctx.Logger.Info("Drop request acknowledged, switching to Drop routine")
					step.CleanupForDrop()
					_ = b.ctx.Manager.ExitGame()

					d := run.NewDrop()
					if derr := d.Run(nil); derr != nil {
						b.ctx.Logger.Error("Drop run failed", "error", derr)
					} else {
						b.ctx.Logger.Info("Drop run completed successfully")
					}

					// Note: ResetDropContext() is called in Drop.go's defer
					// to handle both success and failure cases consistently

					return nil
				}

				var runFinishReason event.FinishReason
				if err != nil {
					switch {
					case errors.Is(err, health.ErrChicken):
						runFinishReason = event.FinishedChicken
					case errors.Is(err, health.ErrMercChicken):
						runFinishReason = event.FinishedMercChicken
					case errors.Is(err, health.ErrDied):
						runFinishReason = event.FinishedDied
					case errors.Is(err, errors.New("player idle for too long, quitting game")): // Match the specific error
						runFinishReason = event.FinishedError
					case errors.Is(err, errors.New("bot globally idle for too long (no movement), quitting game")): // Match the specific error for movement-based idle
						runFinishReason = event.FinishedError
					case errors.Is(err, errors.New("player stuck in an unrecoverable movement loop, quitting")): // Match the specific error for movement-based idle
						runFinishReason = event.FinishedError
					case errors.Is(err, action.ErrFailedToEquip): // This is the new line
						runFinishReason = event.FinishedError
					default:
						runFinishReason = event.FinishedError
					}
				} else {
					runFinishReason = event.FinishedOK
				}

				event.Send(event.RunFinished(event.Text(b.ctx.Name, fmt.Sprintf("Finished run: %s", r.Name())), r.Name(), runFinishReason))

				if err != nil {
					return err
				}

				if !skipTownRoutines {
					err = action.PostRun(r == runs[len(runs)-1])
					if err != nil {
						return err
					}
				}
			}
		}
		return nil
	})

	return g.Wait()
}

func (b *Bot) Stop() {
	b.ctx.SwitchPriority(botCtx.PriorityStop)
	b.ctx.Detach()
}

type MuleManager interface {
	ShouldMule(stashFull bool, characterName string) (bool, string)
}

type StatsReporter interface {
	ReportStats()
}
