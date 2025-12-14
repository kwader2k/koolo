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
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	botCtx "github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/drop"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/health"
	"github.com/hectorgimenez/koolo/internal/run"
	"github.com/hectorgimenez/koolo/internal/utils"

	"github.com/hectorgimenez/d2go/pkg/data/skill"
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

				// Sometimes when we switch areas, monsters are not loaded yet, and we don't properly detect the Merc
				// let's add some small delay (just few ms) when this happens, and recheck the merc status
				if b.ctx.CharacterCfg.BackToTown.MercDied && b.ctx.Data.MercHPPercent() <= 0 && b.ctx.CharacterCfg.Character.UseMerc {
					time.Sleep(200 * time.Millisecond)
				}

				// extra RefreshGameData not needed for Legacygraphics/Portraits since Background loop will automatically refresh after 100ms
				if b.ctx.CharacterCfg.ClassicMode && !b.ctx.Data.LegacyGraphics {
					// Toggle Legacy if enabled
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
				b.ctx.SwitchPriority(botCtx.PriorityHigh)

				// Area correction (only check if enabled)
				if b.ctx.CurrentGame.AreaCorrection.Enabled {
					if err = action.AreaCorrection(); err != nil {
						b.ctx.Logger.Warn("Area correction failed", "error", err)
					}
				}

				// Perform item pickup if enabled
				if b.ctx.CurrentGame.PickupItems {
					action.ItemPickup(30)
				}
				action.BuffIfRequired()

				lvl, _ := b.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)

				MaxLevel := b.ctx.CharacterCfg.Game.StopLevelingAt

				if lvl.Value >= MaxLevel && MaxLevel > 0 {
					b.ctx.Logger.Info(fmt.Sprintf("Player reached level %d (>= MaxLevelAct1 %d). Triggering supervisor stop via context.", lvl.Value, MaxLevel), "run", "Leveling")
					b.ctx.StopSupervisor()
					return nil // Return nil to gracefully end the current run loop
				}

				isInTown := b.ctx.Data.PlayerUnit.Area.IsTown()

				// Check potions in belt
				_, healingPotionsFoundInBelt := b.ctx.Data.Inventory.Belt.GetFirstPotion(data.HealingPotion)
				_, manaPotionsFoundInBelt := b.ctx.Data.Inventory.Belt.GetFirstPotion(data.ManaPotion)
				_, rejuvPotionsFoundInBelt := b.ctx.Data.Inventory.Belt.GetFirstPotion(data.RejuvenationPotion)

				// Check potions in inventory
				hasHealingPotionsInInventory := b.ctx.Data.HasPotionInInventory(data.HealingPotion)
				hasManaPotionsInInventory := b.ctx.Data.HasPotionInInventory(data.ManaPotion)
				hasRejuvPotionsInInventory := b.ctx.Data.HasPotionInInventory(data.RejuvenationPotion)

				// Check if we actually need each type of potion
				needHealingPotionsRefill := !healingPotionsFoundInBelt && b.ctx.CharacterCfg.Inventory.BeltColumns.Total(data.HealingPotion) > 0
				needManaPotionsRefill := !manaPotionsFoundInBelt && b.ctx.CharacterCfg.Inventory.BeltColumns.Total(data.ManaPotion) > 0
				needRejuvPotionsRefill := !rejuvPotionsFoundInBelt && b.ctx.CharacterCfg.Inventory.BeltColumns.Total(data.RejuvenationPotion) > 0

				// Determine if we should refill for each type based on availability in inventory
				shouldRefillHealingPotions := needHealingPotionsRefill && hasHealingPotionsInInventory
				shouldRefillManaPotions := needManaPotionsRefill && hasManaPotionsInInventory
				shouldRefillRejuvPotions := needRejuvPotionsRefill && hasRejuvPotionsInInventory

				// Refill belt if:
				// 1. Each potion type (healing/mana) is either already in belt or needed and available in inventory
				// 2. And at least one potion type actually needs refilling
				// Note: If one type (healing/mana) can be refilled but the other cannot, we skip refill and go to town instead
				// 3. BUT will refill in any case if rejuvenation potions are needed and available in inventory
				shouldRefillBelt := ((shouldRefillHealingPotions || healingPotionsFoundInBelt) &&
					(shouldRefillManaPotions || manaPotionsFoundInBelt) &&
					(needHealingPotionsRefill || needManaPotionsRefill)) || shouldRefillRejuvPotions

				if shouldRefillBelt && !isInTown {
					action.ManageBelt()
					action.RefillBeltFromInventory()
					b.ctx.RefreshGameData()

					// Recheck potions in belt after refill
					_, healingPotionsFoundInBelt = b.ctx.Data.Inventory.Belt.GetFirstPotion(data.HealingPotion)
					_, manaPotionsFoundInBelt = b.ctx.Data.Inventory.Belt.GetFirstPotion(data.ManaPotion)
					needHealingPotionsRefill = !healingPotionsFoundInBelt && b.ctx.CharacterCfg.Inventory.BeltColumns.Total(data.HealingPotion) > 0
					needManaPotionsRefill = !manaPotionsFoundInBelt && b.ctx.CharacterCfg.Inventory.BeltColumns.Total(data.ManaPotion) > 0
				}

				townChicken := b.ctx.CharacterCfg.Health.TownChickenAt > 0 && b.ctx.Data.PlayerUnit.HPPercent() <= b.ctx.CharacterCfg.Health.TownChickenAt

				// Check if we need to go back to town (level, gold, and TP quantity are met, AND then other conditions)
				if _, found := b.ctx.Data.KeyBindings.KeyBindingForSkill(skill.TomeOfTownPortal); found {

					lvl, _ := b.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)

					if !b.NeedsTPsToContinue() { // Now calls b.NeedsTPsToContinue()
						// The curly brace was misplaced here. It should enclose the entire inner 'if' block.
						// This outer 'if' now acts as the gatekeeper for the inner 'if'.
						if (b.ctx.Data.PlayerUnit.TotalPlayerGold() > 500 && lvl.Value <= 5) ||
							(b.ctx.Data.PlayerUnit.TotalPlayerGold() > 1000 && lvl.Value < 20) ||
							(b.ctx.Data.PlayerUnit.TotalPlayerGold() > 5000 && lvl.Value >= 20) {

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
									// THIS IS THE KEY CHANGE: If InRunReturnTownRoutine() returns an error, we propagate it.
									// This will cause the entire errgroup to cancel, and the bot.Run to return this error.
									return err // <--- THIS IS THE ONLY CHANGE IN THIS FILE
								}
							}
						}
					}
				} // This closing brace was misplaced. It should be here, closing the outer 'if'.
				b.ctx.SwitchPriority(botCtx.PriorityNormal)
			}
		}
	})

	// Low priority loop, this will keep executing main run scripts
	g.Go(func() error {
		defer func() {
			cancel()
			b.Stop()
			recover()
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
