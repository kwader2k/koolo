package action

import (
	"fmt"
	"sort"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/pather"
)

func ClearAreaAroundPlayer(radius int, filter data.MonsterFilter) error {
	return ClearAreaAroundPosition(context.Get().Data.PlayerUnit.Position, radius, filter)
}

func IsPriorityMonster(m data.Monster) bool {
	priorityMonsters := []npc.ID{
		npc.FallenShaman,
		npc.CarverShaman,
		npc.DevilkinShaman,
		npc.DarkShaman,
		npc.WarpedShaman,
		npc.MummyGenerator,
		npc.BaalSubjectMummy,
		npc.FetishShaman,
	}

	for _, priorityMonster := range priorityMonsters {
		if m.Name == priorityMonster {
			return true
		}
	}
	return false
}

func SortEnemiesByPriority(enemies *[]data.Monster) {
	ctx := context.Get()
	sort.Slice(*enemies, func(i, j int) bool {
		monsterI := (*enemies)[i]
		monsterJ := (*enemies)[j]

		isPriorityI := IsPriorityMonster(monsterI)
		isPriorityJ := IsPriorityMonster(monsterJ)

		distanceI := ctx.PathFinder.DistanceFromMe(monsterI.Position)
		distanceJ := ctx.PathFinder.DistanceFromMe(monsterJ.Position)

		if distanceI > 2 && distanceJ > 2 {
			if isPriorityI && !isPriorityJ {
				return true
			} else if !isPriorityI && isPriorityJ {
				return false
			}
		}

		return distanceI < distanceJ
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

		for _, m := range enemies {
			distanceToTarget := pather.DistanceFromPoint(pos, m.Position)
			if distanceToTarget > radius {
				continue
			}

			// Special case: Vizier can spawn on weird/off-grid tiles in Chaos Sanctuary.
			isVizier := m.Type == data.MonsterTypeSuperUnique && m.Name == npc.StormCaster

			// Skip monsters that exist in data but are placed on non-walkable tiles (often "underwater/off-grid").
			if !isVizier && !ctx.Data.AreaData.IsWalkable(m.Position) {
				continue
			}

			validEnemy := true
			if !ctx.Data.CanTeleport() {
				// If no path exists, do not target it (prevents chasing "ghost" monsters).
				_, _, pathFound := ctx.PathFinder.GetPath(m.Position)
				if !pathFound {
					validEnemy = false
				}

				// Keep the door check to avoid targeting monsters behind closed doors.
				if hasDoorBetween, _ := ctx.PathFinder.HasDoorBetween(ctx.Data.PlayerUnit.Position, m.Position); hasDoorBetween {
					validEnemy = false
				}
			}

			if validEnemy {
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
