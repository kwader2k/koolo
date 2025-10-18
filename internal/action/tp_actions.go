// internal/action/tp_actions.go
package action

import (
	"errors"
	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/health"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/utils"
)

func checkPlayerDeathForTP(ctx *context.Status) error {
	if ctx.Data.PlayerUnit.HPPercent() <= 0 {
		return health.ErrDied
	}
	return nil
}

func ReturnTown() error {
	ctx := context.Get()
	ctx.SetLastAction("ReturnTown")
	ctx.PauseIfNotPriority()

	// Proactive death check at the start of the action
	if err := checkPlayerDeathForTP(ctx); err != nil {
		return err
	}

	if ctx.Data.PlayerUnit.Area.IsTown() {
		return nil
	}

	err := step.OpenPortal()
	if err != nil {
		// If opening portal fails, check if we died
		if errCheck := checkPlayerDeathForTP(ctx); errCheck != nil {
			return errCheck
		}
		return err
	}
	portal, found := ctx.Data.Objects.FindOne(object.TownPortal)
	if !found {
		// If portal not found, check if we died
		if errCheck := checkPlayerDeathForTP(ctx); errCheck != nil {
			return errCheck
		}
		return errors.New("portal not found")
	}

	initialInteractionErr := InteractObject(portal, func() bool {
		// Check for death during interaction callback
		if errCheck := checkPlayerDeathForTP(ctx); errCheck != nil {
			return false // Returning false will stop the interaction loop, and the error will be caught outside
		}
		return ctx.Data.PlayerUnit.Area.IsTown()
	})

	if initialInteractionErr != nil {
		ctx.Logger.Debug("Initial portal interaction failed, attempting to clear area.", "error", initialInteractionErr)
		// If initial interaction fails, THEN clear the area
		if err = ClearAreaAroundPosition(portal.Position, 8, data.MonsterAnyFilter()); err != nil {
			ctx.Logger.Warn("Error clearing area around portal", "error", err)
			// Even if clearing area fails, check if we died during the process
			if errCheck := checkPlayerDeathForTP(ctx); errCheck != nil {
				return errCheck
			}
		}

		// After (attempting to) clear, try to interact with the portal again
		err = InteractObject(portal, func() bool {
			// Check for death during interaction callback
			if errCheck := checkPlayerDeathForTP(ctx); errCheck != nil {
				return false // Returning false will stop the interaction loop, and the error will be caught outside
			}
			return ctx.Data.PlayerUnit.Area.IsTown()
		})
		if err != nil {
			// If even after clearing, interaction fails, check for death and return error
			if errCheck := checkPlayerDeathForTP(ctx); errCheck != nil {
				return errCheck
			}
			return err
		}
	}

	// Wait for area transition and data sync
	utils.Sleep(1000)
	ctx.RefreshGameData()

	// Wait for town area data to be fully loaded
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		// Check for death during the wait for town data
		if errCheck := checkPlayerDeathForTP(ctx); errCheck != nil {
			return errCheck
		}

		if ctx.Data.PlayerUnit.Area.IsTown() {
			// Verify area data exists and is loaded
			if townData, ok := ctx.Data.Areas[ctx.Data.PlayerUnit.Area]; ok {
				if townData.IsInside(ctx.Data.PlayerUnit.Position) {
					return nil
				}
			}
		}
		utils.Sleep(100)
		ctx.RefreshGameData()
	}

	return fmt.Errorf("failed to verify town area data after portal transition")
}

func UsePortalInTown() error {
	ctx := context.Get()
	ctx.SetLastAction("UsePortalInTown")

	// Proactive death check at the start of the action
	if err := checkPlayerDeathForTP(ctx); err != nil {
		return err
	}

	tpArea := town.GetTownByArea(ctx.Data.PlayerUnit.Area).TPWaitingArea(*ctx.Data)
	_ = MoveToCoords(tpArea) // MoveToCoords already has death checks

	err := UsePortalFrom(ctx.Data.PlayerUnit.Name)
	if err != nil {
		// If using portal fails, check if we died
		if errCheck := checkPlayerDeathForTP(ctx); errCheck != nil {
			return errCheck
		}
		return err
	}

	// Wait for area sync before attempting any movement
	utils.Sleep(500)
	ctx.RefreshGameData()
	// Check for death after refreshing game data
	if err := checkPlayerDeathForTP(ctx); err != nil {
		return err
	}

	if err := ensureAreaSync(ctx, ctx.Data.PlayerUnit.Area); err != nil {
		return err
	}

	// Ensure we're not in town
	if ctx.Data.PlayerUnit.Area.IsTown() {
		return fmt.Errorf("failed to leave town area")
	}

	// Perform item pickup after re-entering the portal
	err = ItemPickup(40)
	if err != nil {
		ctx.Logger.Warn("Error during item pickup after portal use", "error", err)
		// If item pickup fails, check if we died
		if errCheck := checkPlayerDeathForTP(ctx); errCheck != nil {
			return errCheck
		}
	}

	return nil
}

func UsePortalFrom(owner string) error {
	ctx := context.Get()
	ctx.SetLastAction("UsePortalFrom")

	if err := checkPlayerDeathForTP(ctx); err != nil {
		return err
	}

	if !ctx.Data.PlayerUnit.Area.IsTown() {
		return nil
	}

	var portalObj *data.Object

	for i := range ctx.Data.Objects {
		obj := &ctx.Data.Objects[i]

		if obj.IsPortal() && obj.Owner == owner {
			portalObj = obj
			break
		}
	}

	if portalObj == nil {
		return errors.New("portal not found")
	}

	const maxAttempts = 8
	for interactionAttempts := 1; interactionAttempts <= maxAttempts; interactionAttempts++ {
		err := InteractObjectByID(portalObj.ID, func() bool {
			if errCheck := checkPlayerDeathForTP(ctx); errCheck != nil {
				return false
			}

			utils.Sleep(100)

			if !ctx.Data.PlayerUnit.Area.IsTown() {
				utils.Sleep(500)
				ctx.RefreshGameData()

				if errCheck := checkPlayerDeathForTP(ctx); errCheck != nil {
					return false
				}

				if err := ensureAreaSync(ctx, ctx.Data.PlayerUnit.Area); err != nil {
					ctx.Logger.Error("Area sync failed after portal use: %v", err)
					return false
				}

				return true
			}

			return false
		})

		if err == nil && !ctx.Data.PlayerUnit.Area.IsTown() {
			return nil
		}

		if err != nil {
			ctx.Logger.Debug("Failed to interact with portal (attempt %d): %v", interactionAttempts, err)
		}

		if interactionAttempts < maxAttempts {
			ctx.Logger.Debug("Performing random movement to reset position for portal retry #%d.", interactionAttempts)

			ctx.PathFinder.RandomMovement()

			utils.Sleep(1500)
		}
	}

	return errors.New("failed to use portal after multiple attempts")
}
