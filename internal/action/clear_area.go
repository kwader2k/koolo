package action

import (
	"fmt"
	"slices"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/character/core"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/pather"
)

func ClearAreaAroundPlayer(radius int, filter data.MonsterFilter) error {
	return ClearAreaAroundPosition(context.Get().Data.PlayerUnit.Position, radius, filter)
}

func monsterTypePriority(monsterType data.MonsterType) int {
	switch monsterType {
	case data.MonsterTypeSuperUnique:
		return 0
	case data.MonsterTypeUnique:
		return 1
	case data.MonsterTypeMinion:
		return 2
	case data.MonsterTypeChampion:
		return 3
	case data.MonsterTypeNone:
		return 4
	default:
		return 5
	}
}

func SortEnemiesByPriority(enemies *[]data.Monster) {
	if enemies == nil || len(*enemies) < 2 {
		return
	}

	ctx := context.Get()
	canTeleport := ctx.Data.CanTeleport()
	distanceByID := make(map[data.UnitID]int, len(*enemies))
	typeByID := make(map[data.UnitID]int, len(*enemies))
	raiserByID := make(map[data.UnitID]int, len(*enemies))
	var lineOfSightByID map[data.UnitID]bool
	meleeCount := 0
	if !canTeleport {
		lineOfSightByID = make(map[data.UnitID]bool, len(*enemies))
	}

	for _, m := range *enemies {
		distance := ctx.PathFinder.DistanceFromMe(m.Position)
		distanceByID[m.UnitID] = distance
		typeByID[m.UnitID] = monsterTypePriority(m.Type)
		if m.IsMonsterRaiser() {
			raiserByID[m.UnitID] = 0
		} else {
			raiserByID[m.UnitID] = 1
		}
		if !canTeleport {
			lineOfSightByID[m.UnitID] = ctx.PathFinder.LineOfSight(ctx.Data.PlayerUnit.Position, m.Position)
			if distance <= core.MeleeRange {
				meleeCount++
			}
		}
	}

	compareByDistance := func(monsterI, monsterJ data.Monster) int {
		distanceI := distanceByID[monsterI.UnitID]
		distanceJ := distanceByID[monsterJ.UnitID]
		if distanceI < distanceJ {
			return -1
		}
		if distanceI > distanceJ {
			return 1
		}

		return 0
	}
	compareByTypeThenDistance := func(monsterI, monsterJ data.Monster) int {
		// SuperUnique > Unique > Minion > Champion > None > Unknown
		typeI := typeByID[monsterI.UnitID]
		typeJ := typeByID[monsterJ.UnitID]
		if typeI != typeJ {
			if typeI < typeJ {
				return -1
			}
			return 1
		}

		// Raiser > Normal
		raiserI := raiserByID[monsterI.UnitID]
		raiserJ := raiserByID[monsterJ.UnitID]
		if raiserI != raiserJ {
			if raiserI < raiserJ {
				return -1
			}
			return 1
		}

		return compareByDistance(monsterI, monsterJ)
	}
	compareByLineOfSight := func(monsterI, monsterJ data.Monster) int {
		losI := lineOfSightByID[monsterI.UnitID]
		losJ := lineOfSightByID[monsterJ.UnitID]
		if losI == losJ {
			return 0
		}
		if losI {
			return -1
		}
		return 1
	}

	// Teleport: Pick closest of highest type
	if canTeleport {
		slices.SortStableFunc(*enemies, func(monsterI, monsterJ data.Monster) int {
			return compareByTypeThenDistance(monsterI, monsterJ)
		})
		return
	}

	// Melee or Surrounded: Pick closest in LoS
	mainSkillRange := ctx.Char.MainSkillRange()
	if mainSkillRange <= core.MeleeRange || meleeCount >= 2 {
		slices.SortStableFunc(*enemies, func(monsterI, monsterJ data.Monster) int {
			if cmp := compareByLineOfSight(monsterI, monsterJ); cmp != 0 {
				return cmp
			}
			return compareByDistance(monsterI, monsterJ)
		})

		return
	}

	// Default: Pick closest of highest type in LoS
	slices.SortStableFunc(*enemies, func(monsterI, monsterJ data.Monster) int {
		if cmp := compareByLineOfSight(monsterI, monsterJ); cmp != 0 {
			return cmp
		}
		return compareByTypeThenDistance(monsterI, monsterJ)
	})
}

func ClearAreaAroundPosition(pos data.Position, radius int, filters ...data.MonsterFilter) error {
	ctx := context.Get()
	ctx.SetLastAction("ClearAreaAroundPosition")

	// Disable item pickup at the beginning of the function
	ctx.DisableItemPickup()

	// Defer the re-enabling of item pickup to ensure it happens regardless of how the function exits
	defer ctx.EnableItemPickup()

	return ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		enemies := d.Monsters.Enemies(filters...)

		SortEnemiesByPriority(&enemies)

		isValidEnemy := func(m data.Monster) bool {
			// Special case: Vizier can spawn on weird/off-grid tiles in Chaos Sanctuary.
			isVizier := m.Type == data.MonsterTypeSuperUnique && m.Name == npc.StormCaster

			// Skip monsters that exist in data but are placed on non-walkable tiles (often "underwater/off-grid").
			if !isVizier && !ctx.Data.AreaData.IsWalkable(m.Position) {
				return false
			}

			if !ctx.Data.CanTeleport() {
				// If no path exists, do not target it (prevents chasing "ghost" monsters).
				_, _, pathFound := ctx.PathFinder.GetPath(m.Position)
				if !pathFound {
					return false
				}

				// Keep the door check to avoid targeting monsters behind closed doors.
				if hasDoorBetween, _ := ctx.PathFinder.HasDoorBetween(ctx.Data.PlayerUnit.Position, m.Position); hasDoorBetween {
					return false
				}
			}

			return true
		}

		for _, m := range enemies {
			distanceToTarget := pather.DistanceFromPoint(pos, m.Position)
			if distanceToTarget > radius {
				continue
			}
			if ctx.Char.ShouldIgnoreMonster(m) {
				continue
			}

			if isValidEnemy(m) {
				return m.UnitID, true
			}
		}

		return data.UnitID(0), false
	}, nil)
}

func ClearThroughPath(pos data.Position, radius int, filter data.MonsterFilter) error {
	ctx := context.Get()

	lastMovement := false
	for {
		ctx.PauseIfNotPriority()

		ClearAreaAroundPosition(ctx.Data.PlayerUnit.Position, radius, filter)

		if lastMovement {
			return nil
		}

		path, _, found := ctx.PathFinder.GetPath(pos)
		if !found {
			return fmt.Errorf("path could not be calculated")
		}

		movementDistance := radius
		if radius > len(path) {
			movementDistance = len(path)
		}

		dest := data.Position{
			X: path[movementDistance-1].X + ctx.Data.AreaData.OffsetX,
			Y: path[movementDistance-1].Y + ctx.Data.AreaData.OffsetY,
		}

		// Let's handle the last movement logic to MoveTo function, we will trust the pathfinder because
		// it can finish within a bigger distance than we expect (because blockers), so we will just check how far
		// we should be after the latest movement in a theoretical way
		if len(path)-movementDistance <= step.DistanceToFinishMoving {
			lastMovement = true
		}
		// Increasing DistanceToFinishMoving prevent not being to able to finish movement if our destination is center of a large object like Seal in diablo run.
		// is used only for pathing, attack.go will use default DistanceToFinishMoving
		err := step.MoveTo(dest, step.WithDistanceToFinish(7))
		if err != nil {

			if strings.Contains(err.Error(), "monsters detected in movement path") {
				ctx.Logger.Debug("ClearThroughPath: Movement failed due to monsters, attempting to clear them")
				clearErr := ClearAreaAroundPosition(ctx.Data.PlayerUnit.Position, radius+5, filter)
				if clearErr != nil {
					ctx.Logger.Error(fmt.Sprintf("ClearThroughPath: Failed to clear monsters after movement failure: %v", clearErr))
				} else {
					ctx.Logger.Debug("ClearThroughPath: Successfully cleared monsters, continuing with next iteration")
					continue
				}
			}
			return err
		}
	}
}
