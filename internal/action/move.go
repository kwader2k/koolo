// internal/action/move.go
package action

import (
	"errors"
	"fmt"
	"log/slog"
	"math"
	"slices"
	"sort"
	"time"

	"github.com/hectorgimenez/koolo/internal/chicken"
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/utils"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/drop"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/health"
)

const (
	maxAreaSyncAttempts   = 10
	areaSyncDelay         = 200 * time.Millisecond
	monsterHandleCooldown = 500 * time.Millisecond // Reduced cooldown for more immediate re-engagement
	lootAfterCombatRadius = 25                     // Define a radius for looting after combat
)

var alwaysTakeShrines = []object.ShrineType{
	object.RefillShrine,
	object.HealthShrine,
	object.ManaShrine,
}

var prioritizedShrines = []struct {
	shrineType object.ShrineType
	state      state.State
}{
	{shrineType: object.ExperienceShrine, state: state.ShrineExperience},
	{shrineType: object.ManaRegenShrine, state: state.ShrineManaRegen},
	{shrineType: object.StaminaShrine, state: state.ShrineStamina},
	{shrineType: object.SkillShrine, state: state.ShrineSkill},
}

var curseBreakingShrines = []object.ShrineType{
	object.ExperienceShrine,
	object.ManaRegenShrine,
	object.StaminaShrine,
	object.SkillShrine,
	object.ArmorShrine,
	object.CombatShrine,
	object.ResistLightningShrine,
	object.ResistFireShrine,
	object.ResistColdShrine,
	object.ResistPoisonShrine,
}

var (
	ErrArcaneDeadEnd = errors.New("arcane sanctuary dead end")
)

// checkPlayerDeath checks if the player is dead and returns ErrDied if so.
func checkPlayerDeath(ctx *context.Status) error {
	if ctx.Manager == nil || !ctx.Manager.InGame() || ctx.Data.PlayerUnit.ID == 0 {
		// Avoid false death checks while out of game or data is not yet valid.
		return nil
	}

	if ctx.Data.PlayerUnit.Area.IsTown() {
		return nil
	}

	if ctx.Data.PlayerUnit.IsDead() {
		return health.ErrDied
	}
	return nil
}

func ensureAreaSync(ctx *context.Status, expectedArea area.ID) error {
	if ctx.Context != nil && ctx.Context.Drop != nil && ctx.Context.Drop.Pending() != nil && ctx.Context.Drop.Active() == nil {
		return drop.ErrInterrupt
	}

	// Wait for area data to sync
	for attempts := range maxAreaSyncAttempts {
		ctx.RefreshGameData()

		// Check for death during area sync
		if err := checkPlayerDeath(ctx); err != nil {
			return err
		}

		if ctx.Data.PlayerUnit.Area == expectedArea {
			// Area ID matches, now verify collision data is loaded
			if ctx.Data.AreaData.Grid != nil &&
				ctx.Data.AreaData.Grid.CollisionGrid != nil &&
				len(ctx.Data.AreaData.Grid.CollisionGrid) > 0 {
				// Additional check: ensure we have adjacent level data if this is a cross-area operation
				// Give it one more refresh cycle to ensure all data is populated
				if attempts > 0 {
					time.Sleep(areaSyncDelay + 50*time.Millisecond)
					ctx.RefreshGameData()
				}

				return nil
			}
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

	cachedPos := data.Position{}
	if !lvl.IsEntrance && ctx.Data.PlayerUnit.Area != dst {
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
				cachedPos = obj.Position
				break
			}
		}

		if cachedPos == (data.Position{}) {
			cachedPos = lvl.Position
		}
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

		return cachedPos, true
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
		err = MoveTo(toFun, step.WithDistanceToFinish(7))
	} else {
		err = MoveTo(toFun)
	}

	if err != nil {
		if errors.Is(err, drop.ErrInterrupt) {
			return err
		}
		if errors.Is(err, health.ErrDied) { // Propagate death error
			return err
		}
		if !lvl.IsEntrance {
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

	// apply buffs after entering a new area if configured
	if ctx.CharacterCfg.Character.BuffOnNewArea {
		Buff()
	}

	event.Send(event.InteractedTo(event.Text(ctx.Name, ""), int(dst), event.InteractionTypeEntrance))
	return nil
}

func MoveToCoords(to data.Position, options ...step.MoveOption) error {
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
	}, options...)
}

func onSafeNavigation() {
	ctx := context.Get()

	if _, isLevelingChar := ctx.Char.(context.LevelingCharacter); isLevelingChar {
		statPoints, hasUnusedPoints := ctx.Data.PlayerUnit.FindStat(stat.StatPoints, 0)
		if hasUnusedPoints && statPoints.Value > 0 {
			ctx.PauseIfNotPriority()
			ctx.DisableItemPickup()
			EnsureSkillPoints()
			EnsureStatPoints()
			EnsureSkillBindings()
			ctx.EnableItemPickup()
		}
		if ctx.HealthManager.IsLowStamina() {
			TryConsumeStaminaPot()
		}
	}

}

func getPathOffsets(to data.Position) (int, int) {
	ctx := context.Get()

	minOffsetX := ctx.Data.AreaData.OffsetX
	minOffsetY := ctx.Data.AreaData.OffsetY

	if !ctx.Data.AreaData.IsInside(to) {
		for _, otherArea := range ctx.Data.AreaData.AdjacentLevels {
			destination, exists := ctx.Data.Areas[otherArea.Area]
			if !exists {
				continue
			}
			if destination.IsInside(to) {
				minOffsetX = min(minOffsetX, destination.OffsetX)
				minOffsetY = min(minOffsetY, destination.OffsetY)
			}
		}
	}

	return minOffsetX, minOffsetY
}

func MoveTo(toFunc func() (data.Position, bool), options ...step.MoveOption) error {
	ctx := context.Get()
	ctx.SetLastAction("MoveTo")

	// Initialize options
	opts := &step.MoveOpts{}

	// Apply any provided options
	for _, o := range options {
		o(opts)
	}

	minDistanceToFinishMoving := step.DistanceToFinishMoving
	if opts.DistanceToFinish() != nil {
		minDistanceToFinishMoving = *opts.DistanceToFinish()
	}

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

	clearPathDist := ctx.CharacterCfg.Character.ClearPathDist // Get this once
	overrideClearPathDist := false
	if opts.ClearPathOverride() != nil {
		clearPathDist = *opts.ClearPathOverride()
		overrideClearPathDist = true
	}
	ignoreShrines := !ctx.CharacterCfg.Game.InteractWithShrines
	initialMovementArea := ctx.Data.PlayerUnit.Area
	actionLastMonsterHandlingTime := time.Time{}
	var targetPosition, previousTargetPosition, previousPosition data.Position
	var shrine, chest data.Object
	var pathOffsetX, pathOffsetY int
	var path pather.Path
	var distanceToTarget int
	var pathFound bool
	var pathErrors int
	var stuck bool
	blacklistedInteractions := map[data.UnitID]bool{}
	adjustMinDist := false

	//Arcane sanctuary portal navigation
	var tpPad data.Object
	var blacklistedPads []data.Object

	// Initial sync check
	if err := ensureAreaSync(ctx, ctx.Data.PlayerUnit.Area); err != nil {
		return err
	}

	for {
		ctx.PauseIfNotPriority()
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

		targetPosition = to

		//We're not trying to get to town, yet we are. Let bot do his stuff in town and wait to be back on the field
		if !initialMovementArea.IsTown() && ctx.Data.AreaData.Area.IsTown() && !town.IsPositionInTown(targetPosition) {
			utils.Sleep(100)
			continue
		}

		isSafe := true
		if !ctx.Data.AreaData.Area.IsTown() {
			if !ctx.Data.CanTeleport() {
				chicken.CheckForScaryAuraAndCurse()
			}

			//Safety first, handle enemies
			if !opts.IgnoreMonsters() && (!ctx.Data.CanTeleport() || overrideClearPathDist) && time.Since(actionLastMonsterHandlingTime) > monsterHandleCooldown {
				actionLastMonsterHandlingTime = time.Now()
				filters := opts.MonsterFilters()
				filters = append(filters, func(monsters data.Monsters) (filteredMonsters []data.Monster) {
					for _, m := range monsters {
						if stuck || !ctx.Char.ShouldIgnoreMonster(m) {
							filteredMonsters = append(filteredMonsters, m)
						}
					}
					return filteredMonsters
				})
				_ = ClearAreaAroundPosition(ctx.Data.PlayerUnit.Position, clearPathDist, filters...)
				if !opts.IgnoreItems() {
					// After clearing, immediately try to pick up items
					lootErr := ItemPickup(lootAfterCombatRadius)
					if lootErr != nil {
						ctx.Logger.Warn("Error picking up items after combat", slog.String("error", lootErr.Error()))
					}
				}
			}

			//Check shrine nearby
			if !ignoreShrines && shrine.ID == 0 {
				if closestShrine := findClosestShrine(50.0); closestShrine != nil {
					blacklisted, exists := blacklistedInteractions[closestShrine.ID]
					if !exists || !blacklisted {
						shrine = *closestShrine
						//ctx.Logger.Debug(fmt.Sprintf("MoveTo: Found shrine at %v, redirecting destination from %v", closestShrine.Position, targetPosition))

						//Reset target chest
						chest = (data.Object{})
					}
				}
			}

			// Check chests nearby
			if shrine.ID == 0 && chest.ID == 0 {
				// "Super chests only" has priority over the generic "all chests" mode.
				if ctx.CharacterCfg.Game.InteractWithSuperChests && !ctx.CharacterCfg.Game.InteractWithChests {
					if closestChest, chestFound := ctx.PathFinder.GetClosestSuperChest(ctx.Data.PlayerUnit.Position, true); chestFound {
						blacklisted, exists := blacklistedInteractions[closestChest.ID]
						if !exists || !blacklisted {
							chest = *closestChest
						}
					}
				} else if ctx.CharacterCfg.Game.InteractWithChests {
					if closestChest, chestFound := ctx.PathFinder.GetClosestChest(ctx.Data.PlayerUnit.Position, true); chestFound {
						blacklisted, exists := blacklistedInteractions[closestChest.ID]
						if !exists || !blacklisted {
							chest = *closestChest
							//ctx.Logger.Debug(fmt.Sprintf("MoveTo: Found chest at %v, redirecting destination from %v", chest.Position, targetPosition))
						}
					}
				}
			}

			//Check if we're safe to do some stuff on the field
			if enemyFound, _ := IsAnyEnemyAroundPlayer(max(clearPathDist*2, 30)); !enemyFound {
				onSafeNavigation()
			} else {
				isSafe = false
			}
		}

		//If we have something to interact with, temporarly change target position
		if shrine.ID != 0 {
			targetPosition = shrine.Position
		} else if chest.ID != 0 {
			targetPosition = chest.Position
		} else if !utils.IsZeroPosition(tpPad.Position) {
			targetPosition = tpPad.Position
		}

		//Only recompute path if needed, it can be heavy
		if !utils.IsSamePosition(previousTargetPosition, targetPosition) || !pathFound {
			previousTargetPosition = targetPosition
			path, _, pathFound = ctx.PathFinder.GetPath(targetPosition)
			pathOffsetX, pathOffsetY = getPathOffsets(targetPosition)
		}

		distanceToTarget = ctx.PathFinder.DistanceFromMe(targetPosition)
		//We didn't find a path, try to handle the case
		if !pathFound {
			//We're in town for some reason, use tp
			if ctx.Data.PlayerUnit.Area.IsTown() && !ctx.Data.AreaData.IsInside(targetPosition) {
				if err := UsePortalInTown(); err != nil {
					return errors.New("path failed during moveto. player in town, target position outside of town and no tp")
				}
			} else if ctx.Data.PlayerUnit.Area == area.ArcaneSanctuary {
				//try to go to the end of the tp lane to find target position
				tpPad, err := getArcaneNextTeleportPadPosition(blacklistedPads)
				if err != nil {
					return err
				}
				blacklistedPads = append(blacklistedPads, tpPad)
				continue
			} else {
				pathErrors++
				//Try some randome movements to help pathfinding (not sure that it helps)
				if pathErrors < 5 {
					ctx.Logger.Warn("No path found, trying random movement to fix")
					ctx.PathFinder.RandomMovement()
					utils.Sleep(200)
					continue
				} else {
					return errors.New("path could not be calculated. Current area: [" + ctx.Data.PlayerUnit.Area.Area().Name + "]. Trying to path to Destination: [" + fmt.Sprintf("%d,%d", to.X, to.Y) + "]")
				}
			}
		} else {
			pathErrors = 0
		}

		//Handle Distance to finish movement
		finishMoveDist := step.DistanceToFinishMoving
		var moveOptions = make([]step.MoveOption, len(options))
		copy(moveOptions, options)
		if minDistanceToFinishMoving != step.DistanceToFinishMoving {
			if utils.IsSamePosition(to, targetPosition) {
				//We don't have any intermediate interactions with objects to do, keep provided parameter
				finishMoveDist = minDistanceToFinishMoving
			} else {
				//Override the parameter with the default value for interactions
				moveOptions = append(moveOptions, step.WithDistanceToFinish(step.DistanceToFinishMoving))
			}
		}

		//We've reached our target destination !
		if distanceToTarget <= finishMoveDist || (adjustMinDist && distanceToTarget <= finishMoveDist*2) {
			if shrine.ID != 0 && targetPosition == shrine.Position {
				//Handle shrine if any
				if err := InteractObject(shrine, func() bool {
					obj, found := ctx.Data.Objects.FindByID(shrine.ID)
					return found && !obj.Selectable
				}); err != nil {
					ctx.Logger.Warn("Failed to interact with shrine", slog.Any("error", err))
				}
				blacklistedInteractions[shrine.ID] = true
				shrine = data.Object{}
				continue
			} else if chest.ID != 0 && targetPosition == chest.Position {
				//Handle chest if any
				if err := InteractObject(chest, func() bool {
					obj, found := ctx.Data.Objects.FindByID(chest.ID)
					return found && !obj.Selectable
				}); err != nil {
					ctx.Logger.Warn("Failed to interact with chest", slog.Any("error", err))
					blacklistedInteractions[chest.ID] = true
				}
				if !opts.IgnoreItems() {
					lootErr := ItemPickup(lootAfterCombatRadius)
					if lootErr != nil {
						ctx.Logger.Warn("Error picking up items after chest opening", slog.String("error", lootErr.Error()))
					}
				}
				chest = data.Object{}
				continue
			} else if !utils.IsZeroPosition(tpPad.Position) && targetPosition == tpPad.Position {
				//Handle arcane sanctuary tp pad if any
				if err := InteractObject(tpPad, func() bool {
					return ctx.PathFinder.DistanceFromMe(tpPad.Position) > 5
				}); err != nil {
					return err
				}
				tpPad = data.Object{}
				exitPad := getClosestTeleportPad(blacklistedPads)
				blacklistedPads = append(blacklistedPads, exitPad)
				continue
			}

			//We've reach the final destination
			return nil
		} else {
			adjustMinDist = false
		}

		//We're not done yet, split the path into smaller segments
		nextPosition := targetPosition
		pathStep := 0
		if !ctx.Data.AreaData.Area.IsTown() {
			//Default path step when teleporting
			maxPathStep := 10

			//Restrict path step when walking
			if !ctx.Data.CanTeleport() {
				if isSafe {
					maxPathStep = 8
				} else {
					//baby steps for safety
					maxPathStep = 3
				}
			} else {
				maxPathStep = ctx.PathFinder.GetLastPathIndexOnScreen(path)
			}

			//Pick target position on path and convert path position to global coordinates
			pathStep = min(maxPathStep, len(path)-1)
			nextPathPos := path[pathStep]
			nextPosition = utils.PositionAddCoords(nextPathPos, pathOffsetX, pathOffsetY)
			if pather.DistanceFromPoint(nextPosition, targetPosition) <= minDistanceToFinishMoving {
				nextPosition = targetPosition
			}
		} else {
			// In town: use path segmentation to avoid getting stuck on corners/objects
			// Larger steps than combat but still segmented for obstacle avoidance
			maxPathStep := 12

			pathStep = min(maxPathStep, len(path)-1)
			if pathStep > 0 {
				nextPathPos := path[pathStep]
				nextPosition = utils.PositionAddCoords(nextPathPos, pathOffsetX, pathOffsetY)
				if pather.DistanceFromPoint(nextPosition, targetPosition) <= minDistanceToFinishMoving {
					nextPosition = targetPosition
				}
			}
		}

		//Do the actual movement...
		moveErr := step.MoveTo(nextPosition, moveOptions...)
		if moveErr != nil {
			//... Reset previous target position to recompute path...
			previousTargetPosition = (data.Position{})

			//... and handle errors if possible
			if errors.Is(moveErr, step.ErrMonstersInPath) {
				continue
			} else if errors.Is(moveErr, step.ErrPlayerStuck) || errors.Is(moveErr, step.ErrPlayerRoundTrip) {
				if (!ctx.Data.CanTeleport() || stuck) || ctx.Data.PlayerUnit.Area.IsTown() {
					ctx.PathFinder.RandomMovement()
					time.Sleep(time.Millisecond * 200)
				}
				stuck = true
				continue
			} else if errors.Is(moveErr, step.ErrNoPath) && pathStep > 0 {
				ctx.PathFinder.RandomMovement()
				time.Sleep(time.Millisecond * 200)
				continue
			}

			//Cannot recover, abort and report error
			return moveErr
		} else if utils.IsSamePosition(previousPosition, ctx.Data.PlayerUnit.Position) {
			adjustMinDist = true
		}

		stuck = false
		previousPosition = ctx.Data.PlayerUnit.Position
		//Move forward in the path after successful movement
		if pathStep > 0 {
			path = path[pathStep:]
		}
	}
}

func findClosestShrine(maxScanDistance float64) *data.Object {
	ctx := context.Get()

	// Check if the bot is dead or chickened before proceeding.
	if ctx.Data.PlayerUnit.IsDead() || ctx.Data.PlayerUnit.HPPercent() <= ctx.Data.CharacterCfg.Health.ChickenAt || ctx.Data.AreaData.Area.IsTown() {
		ctx.Logger.Debug("Bot is dead or chickened, skipping shrine search.")
		return nil
	}

	if ctx.Data.AreaData.Area == area.TowerCellarLevel5 {
		return nil
	}

	if ctx.Data.PlayerUnit.States.HasState(state.Amplifydamage) ||
		ctx.Data.PlayerUnit.States.HasState(state.Lowerresist) ||
		ctx.Data.PlayerUnit.States.HasState(state.Decrepify) {

		ctx.Logger.Debug("Curse detected on player. Prioritizing finding any shrine to break it.")

		var closestCurseBreakingShrine *data.Object
		minDistance := maxScanDistance

		for _, o := range ctx.Data.Objects {
			if o.IsShrine() && o.Selectable {
				for _, sType := range curseBreakingShrines {
					if o.Shrine.ShrineType == sType {
						distance := float64(ctx.PathFinder.DistanceFromMe(o.Position))
						if distance < minDistance {
							minDistance = distance
							closestCurseBreakingShrine = &o
						}
					}
				}
			}
		}
		if closestCurseBreakingShrine != nil {
			return closestCurseBreakingShrine
		}
	}

	var closestAlwaysTakeShrine *data.Object
	minDistance := maxScanDistance
	for _, o := range ctx.Data.Objects {
		if o.IsShrine() && o.Selectable {
			for _, sType := range alwaysTakeShrines {
				if o.Shrine.ShrineType == sType {
					if sType == object.HealthShrine && ctx.Data.PlayerUnit.HPPercent() > 95 {
						continue
					}
					if sType == object.ManaShrine && ctx.Data.PlayerUnit.MPPercent() > 95 {
						continue
					}
					if sType == object.RefillShrine && ctx.Data.PlayerUnit.HPPercent() > 95 && ctx.Data.PlayerUnit.MPPercent() > 95 {
						continue
					}

					distance := float64(ctx.PathFinder.DistanceFromMe(o.Position))
					if distance < minDistance {
						minDistance = distance
						closestAlwaysTakeShrine = &o
					}
				}
			}
		}
	}

	if closestAlwaysTakeShrine != nil {
		return closestAlwaysTakeShrine
	}

	var currentPriorityIndex int = -1

	for i, p := range prioritizedShrines {
		if ctx.Data.PlayerUnit.States.HasState(p.state) {
			currentPriorityIndex = i
			break
		}
	}

	for _, o := range ctx.Data.Objects {
		if o.IsShrine() && o.Selectable {
			shrinePriorityIndex := -1
			for i, p := range prioritizedShrines {
				if o.Shrine.ShrineType == p.shrineType {
					shrinePriorityIndex = i
					break
				}
			}

			if shrinePriorityIndex != -1 && (currentPriorityIndex == -1 || shrinePriorityIndex <= currentPriorityIndex) {
				distance := float64(ctx.PathFinder.DistanceFromMe(o.Position))
				if distance < minDistance {
					minDistance = distance
					closestShrine := &o
					return closestShrine
				}
			}
		}
	}

	return nil
}

func getArcaneNextTeleportPadPosition(blacklistedPads []data.Object) (data.Object, error) {
	ctx := context.Get()
	teleportPads := getValidTeleportPads(blacklistedPads)
	var bestPad data.Object
	bestPathDistance := math.MaxInt
	padFound := false

	for _, tpPad := range teleportPads {
		_, distance, found := ctx.PathFinder.GetPath(tpPad.Position)
		//Basic rule to go to the end : go to closest reachable portal
		if found && distance < bestPathDistance {
			bestPad = tpPad
			bestPathDistance = distance
			padFound = true
		}
	}

	if !padFound {
		return bestPad, ErrArcaneDeadEnd
	}
	return bestPad, nil
}

func getClosestTeleportPad(blacklistedPads []data.Object) data.Object {
	ctx := context.Get()
	tpPads := getValidTeleportPads(blacklistedPads)
	var bestPad data.Object
	closestDistance := math.MaxInt

	for _, tpPad := range tpPads {
		distance := ctx.PathFinder.DistanceFromMe(tpPad.Position)
		if distance < closestDistance {
			bestPad = tpPad
			closestDistance = distance
		}
	}

	return bestPad
}

func getValidTeleportPads(blacklistedPads []data.Object) []data.Object {
	ctx := context.Get()
	var teleportPads []data.Object
	for _, obj := range ctx.Data.AreaData.Objects {
		if slices.ContainsFunc(blacklistedPads, func(e data.Object) bool {
			return utils.IsSamePosition(e.Position, obj.Position)
		}) {
			continue
		}
		switch obj.Name {
		case object.TeleportationPad1, object.TeleportationPad2, object.TeleportationPad3, object.TeleportationPad4:
			teleportPads = append(teleportPads, obj)
		}
	}
	return teleportPads
}
