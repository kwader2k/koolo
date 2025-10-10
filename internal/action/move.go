// internal/action/move.go
package action

import (
	"errors"
	"fmt"
	"log/slog"
	"sort"
	"time"

	"github.com/hectorgimenez/koolo/internal/utils"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/health"
)

const (
	maxAreaSyncAttempts   = 10
	areaSyncDelay         = 100 * time.Millisecond
	monsterHandleCooldown = 500 * time.Millisecond // Reduced cooldown for more immediate re-engagement
	lootAfterCombatRadius = 25                     // Define a radius for looting after combat
)

var actionLastMonsterHandlingTime = time.Time{}

// checkPlayerDeath checks if the player is dead and returns ErrDied if so.
func checkPlayerDeath(ctx *context.Status) error {
	if ctx.Data.PlayerUnit.HPPercent() <= 0 {
		return health.ErrDied
	}
	return nil
}

func ensureAreaSync(ctx *context.Status, expectedArea area.ID) error {
	// Skip sync check if we're already in the expected area and have valid area data
	if ctx.Data.PlayerUnit.Area == expectedArea {
		return nil
	}

	// Wait for area data to sync
	for attempts := 0; attempts < maxAreaSyncAttempts; attempts++ {
		ctx.RefreshGameData()

		// Check for death during area sync
		if err := checkPlayerDeath(ctx); err != nil {
			return err
		}

		if ctx.Data.PlayerUnit.Area == expectedArea {
			return nil
		}

		time.Sleep(areaSyncDelay)
	}

	return fmt.Errorf("area sync timeout - expected: %v, current: %v", expectedArea, ctx.Data.PlayerUnit.Area)
}

func MoveToArea(dst area.ID) error {
	ctx := context.Get()
	ctx.SetLastAction("MoveToArea")

	// Proactive death check at the start of the action
	if err := checkPlayerDeath(ctx); err != nil {
		return err
	}

	if err := ensureAreaSync(ctx, ctx.Data.PlayerUnit.Area); err != nil {
		return err
	}

	// Exceptions for:
	// Arcane Sanctuary
	if dst == area.ArcaneSanctuary && ctx.Data.PlayerUnit.Area == area.PalaceCellarLevel3 {
		ctx.Logger.Debug("Arcane Sanctuary detected, finding the Portal")
		portal, _ := ctx.Data.Objects.FindOne(object.ArcaneSanctuaryPortal)
		MoveToCoords(portal.Position)

		return step.InteractObject(portal, func() bool {
			return ctx.Data.PlayerUnit.Area == area.ArcaneSanctuary
		})
	}
	// Canyon of the Magi
	if dst == area.CanyonOfTheMagi && ctx.Data.PlayerUnit.Area == area.ArcaneSanctuary {
		ctx.Logger.Debug("Canyon of the Magi detected, finding the Portal")
		tome, _ := ctx.Data.Objects.FindOne(object.YetAnotherTome)
		MoveToCoords(tome.Position)
		InteractObject(tome, func() bool {
			if _, found := ctx.Data.Objects.FindOne(object.PermanentTownPortal); found {
				ctx.Logger.Debug("Opening YetAnotherTome!")
				return true
			}
			return false
		})
		ctx.Logger.Debug("Using Canyon of the Magi Portal")
		portal, _ := ctx.Data.Objects.FindOne(object.PermanentTownPortal)
		MoveToCoords(portal.Position)
		return step.InteractObject(portal, func() bool {
			return ctx.Data.PlayerUnit.Area == area.CanyonOfTheMagi
		})
	}

	lvl := data.Level{}
	for _, a := range ctx.Data.AdjacentLevels {
		if a.Area == dst {
			lvl = a
			break
		}
	}

	if lvl.Position.X == 0 && lvl.Position.Y == 0 {
		return fmt.Errorf("destination area not found: %s", dst.Area().Name)
	}

	toFun := func() (data.Position, bool) {
		// Check for death during movement target evaluation
		if err := checkPlayerDeath(ctx); err != nil {
			return data.Position{}, false // Signal to stop moving if dead
		}

		if ctx.Data.PlayerUnit.Area == dst {
			ctx.Logger.Debug("Reached area", slog.String("area", dst.Area().Name))
			return data.Position{}, false
		}

		if ctx.Data.PlayerUnit.Area == area.TamoeHighland && dst == area.MonasteryGate {
			ctx.Logger.Debug("Monastery Gate detected, moving to static coords")
			return data.Position{X: 15139, Y: 5056}, true
		}

		if ctx.Data.PlayerUnit.Area == area.MonasteryGate && dst == area.TamoeHighland {
			ctx.Logger.Debug("Monastery Gate detected, moving to static coords")
			return data.Position{X: 15142, Y: 5118}, true
		}

		// To correctly detect the two possible exits from Lut Gholein
		if dst == area.RockyWaste && ctx.Data.PlayerUnit.Area == area.LutGholein {
			if _, _, found := ctx.PathFinder.GetPath(data.Position{X: 5004, Y: 5065}); found {
				return data.Position{X: 4989, Y: 5063}, true
			} else {
				return data.Position{X: 5096, Y: 4997}, true
			}
		}

		// This means it's a cave, we don't want to load the map, just find the entrance and interact
		if lvl.IsEntrance {
			return lvl.Position, true
		}

		objects := ctx.Data.Areas[lvl.Area].Objects
		// Sort objects by the distance from me
		sort.Slice(objects, func(i, j int) bool {
			distanceI := ctx.PathFinder.DistanceFromMe(objects[i].Position)
			distanceJ := ctx.PathFinder.DistanceFromMe(objects[j].Position)

			return distanceI < distanceJ
		})

		// Let's try to find any random object to use as a destination point, once we enter the level we will exit this flow
		for _, obj := range objects {
			_, _, found := ctx.PathFinder.GetPath(obj.Position)
			if found {
				return obj.Position, true
			}
		}

		return lvl.Position, true
	}

	var err error

	// Areas that require a distance override for proper entrance interaction (Tower, Harem, Sewers)
	if dst == area.HaremLevel1 && ctx.Data.PlayerUnit.Area == area.LutGholein ||
		dst == area.SewersLevel3Act2 && ctx.Data.PlayerUnit.Area == area.SewersLevel2Act2 ||
		dst == area.TowerCellarLevel1 && ctx.Data.PlayerUnit.Area == area.ForgottenTower ||
		dst == area.TowerCellarLevel2 && ctx.Data.PlayerUnit.Area == area.TowerCellarLevel1 ||
		dst == area.TowerCellarLevel3 && ctx.Data.PlayerUnit.Area == area.TowerCellarLevel2 ||
		dst == area.TowerCellarLevel4 && ctx.Data.PlayerUnit.Area == area.TowerCellarLevel3 ||
		dst == area.TowerCellarLevel5 && ctx.Data.PlayerUnit.Area == area.TowerCellarLevel4 {

		// Use a custom loop to integrate the distance override with monster handling.
		entrancePosition, _ := toFun()

		for {
			moveErr := step.MoveTo(entrancePosition, step.WithDistanceToFinish(7))

			if moveErr != nil {
				if errors.Is(moveErr, step.ErrMonstersInPath) {
					// RE-INTRODUCING COMBAT LOGIC FROM MoveTo(toFun)
					clearPathDist := ctx.CharacterCfg.Character.ClearPathDist
					ctx.Logger.Debug("Monster detected while using distance override. Engaging.")

					if time.Since(actionLastMonsterHandlingTime) > monsterHandleCooldown {
						actionLastMonsterHandlingTime = time.Now()
						_ = ClearAreaAroundPosition(ctx.Data.PlayerUnit.Position, clearPathDist, data.MonsterAnyFilter())

						lootErr := ItemPickup(lootAfterCombatRadius)
						if lootErr != nil {
							ctx.Logger.Warn("Error picking up items after combat (Tower/Harem/Sewers)", slog.String("error", lootErr.Error()))
						}
					}
					continue
				}
				// Handle other errors (like pathfinding failure or death)
				err = moveErr
				break
			}
			err = nil
			break
		}
	} else {
		err = MoveTo(toFun)
	}

	if err != nil {
		if errors.Is(err, health.ErrDied) { // Propagate death error
			return err
		}
		ctx.Logger.Warn("error moving to area, will try to continue", slog.String("error", err.Error()))
	}

	if lvl.IsEntrance {
		maxAttempts := 3
		for attempt := 0; attempt < maxAttempts; attempt++ {
			// Check current distance
			currentDistance := ctx.PathFinder.DistanceFromMe(lvl.Position)

			if currentDistance > 7 {
				// For distances > 7, recursively call MoveToArea as it includes the entrance interaction
				return MoveToArea(dst)
			} else if currentDistance > 3 && currentDistance <= 7 {
				// For distances between 4 and 7, use direct click
				screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(
					lvl.Position.X-2,
					lvl.Position.Y-2,
				)
				ctx.HID.Click(game.LeftButton, screenX, screenY)
				utils.Sleep(800)
			}

			// Proactive death check before interacting with entrance
			if err := checkPlayerDeath(ctx); err != nil {
				return err
			}

			// Try to interact with the entrance
			err = step.InteractEntrance(dst)
			if err == nil {
				break
			}

			if attempt < maxAttempts-1 {
				ctx.Logger.Debug("Entrance interaction failed, retrying",
					slog.Int("attempt", attempt+1),
					slog.String("error", err.Error()))
				utils.Sleep(1000)
			}
		}

		if err != nil {
			return fmt.Errorf("failed to interact with area %s after %d attempts: %v", dst.Area().Name, maxAttempts, err)
		}

		// Wait for area transition to complete
		if err := ensureAreaSync(ctx, dst); err != nil {
			return err
		}
	}

	event.Send(event.InteractedTo(event.Text(ctx.Name, ""), int(dst), event.InteractionTypeEntrance))
	return nil
}

func MoveToCoords(to data.Position) error {
	ctx := context.Get()

	// Proactive death check at the start of the action
	if err := checkPlayerDeath(ctx); err != nil {
		return err
	}

	if err := ensureAreaSync(ctx, ctx.Data.PlayerUnit.Area); err != nil {
		return err
	}

	return MoveTo(func() (data.Position, bool) {
		return to, true
	})
}

func MoveTo(toFunc func() (data.Position, bool)) error {
	ctx := context.Get()
	ctx.SetLastAction("MoveTo")

	// Proactive death check at the start of the action
	if err := checkPlayerDeath(ctx); err != nil {
		return err
	}

	// Ensure no menus are open that might block movement
	for ctx.Data.OpenMenus.IsMenuOpen() {
		ctx.Logger.Debug("Found open menus while moving, closing them...")
		if err := step.CloseAllMenus(); err != nil {
			return err
		}

		utils.Sleep(500)
	}

	lastMovement := false
	clearPathDist := ctx.CharacterCfg.Character.ClearPathDist // Get this once

	// Initial sync check
	if err := ensureAreaSync(ctx, ctx.Data.PlayerUnit.Area); err != nil {
		return err
	}

	for {
		ctx.RefreshGameData()
		// Check for death after refreshing game data in the loop
		if err := checkPlayerDeath(ctx); err != nil {
			return err
		}

		to, found := toFunc()
		if !found {
			// This covers the case where toFunc itself might return false due to death
			return nil
		}

		// If we can teleport.
		if ctx.Data.CanTeleport() {
			moveErr := step.MoveTo(to)
			if moveErr != nil {
				if errors.Is(moveErr, step.ErrMonstersInPath) {
					ctx.Logger.Debug("Teleporting character encountered monsters in path. Engaging.")
					if time.Since(actionLastMonsterHandlingTime) > monsterHandleCooldown {
						actionLastMonsterHandlingTime = time.Now()
						_ = ClearAreaAroundPosition(ctx.Data.PlayerUnit.Position, clearPathDist, data.MonsterAnyFilter())
						// After clearing, immediately try to pick up items
						lootErr := ItemPickup(lootAfterCombatRadius)
						if lootErr != nil {
							ctx.Logger.Warn("Error picking up items after combat (teleporter)", slog.String("error", lootErr.Error()))
						}
					}
					continue
				}
				return moveErr
			}
			return nil // Teleport move successful
		}

		if lastMovement {
			return nil
		}

		// Check if we are very close to the destination before trying to move
		if _, distance, _ := ctx.PathFinder.GetPathFrom(ctx.Data.PlayerUnit.Position, to); distance <= step.DistanceToFinishMoving {
			lastMovement = true
		}

		moveErr := step.MoveTo(to)
		if moveErr != nil {
			// This part is now more of a fallback/additional check,
			// as the proactive check above should catch most cases for non-teleporters.
			if errors.Is(moveErr, step.ErrMonstersInPath) && clearPathDist > 0 {
				ctx.Logger.Debug("Monsters still detected by pathfinding after safe zone check. Re-engaging for non-teleporter.")
				if time.Since(actionLastMonsterHandlingTime) > monsterHandleCooldown {
					actionLastMonsterHandlingTime = time.Now()
					_ = ClearAreaAroundPosition(ctx.Data.PlayerUnit.Position, clearPathDist, data.MonsterAnyFilter())
					// After fallback engagement, pick up items
					lootErr := ItemPickup(lootAfterCombatRadius)
					if lootErr != nil {
						ctx.Logger.Warn("Error picking up items after fallback combat", slog.String("error", lootErr.Error()))
					}
				}
				continue // Re-evaluate after combat
			}
			return moveErr
		}

		if lastMovement {
			return nil
		}
	}
}

func MoveToCoordIgnoreClearPath(position data.Position) {
	ctx := context.Get()
	clearPath := ctx.CharacterCfg.Character.ClearPathDist
	ctx.CharacterCfg.Character.ClearPathDist = 0
	MoveToCoords(position)
	ctx.CharacterCfg.Character.ClearPathDist = clearPath
}
