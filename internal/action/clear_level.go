package action

import (
	"errors"
	"fmt"
	"log/slog"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

var interactableShrines = []object.ShrineType{
	object.ExperienceShrine,
	object.StaminaShrine,
	object.ManaRegenShrine,
	object.SkillShrine,
	object.RefillShrine,
	object.HealthShrine,
	object.ManaShrine,
}

func ClearCurrentLevel(openChests bool, filter data.MonsterFilter) error {
	return ClearCurrentLevelEx(openChests, filter, nil)
}

func ClearCurrentLevelEx(openChests bool, filter data.MonsterFilter, shouldInterrupt func() bool) error {
	ctx := context.Get()
	ctx.SetLastAction("ClearCurrentLevel")

	openAllChests := ctx.CharacterCfg.Game.InteractWithChests
	openSuperOnly := ctx.CharacterCfg.Game.InteractWithSuperChests && !openAllChests

	// We can make this configurable later, but 20 is a good starting radius.
	const pickupRadius = 20

	rooms := ctx.PathFinder.OptimizeRoomsTraverseOrder()
	for _, r := range rooms {
		if errDeath := checkPlayerDeath(ctx); errDeath != nil {
			return errDeath
		}
		if shouldInterrupt != nil && shouldInterrupt() {
			return nil
		}

		// First, clear the room of monsters
		err := clearRoom(r, filter)
		if err != nil {
			ctx.Logger.Warn("Failed to clear room", slog.Any("error", err))
		}

		//ctx.Logger.Debug(fmt.Sprintf("Clearing room complete, attempting to pickup items in a radius of %d", pickupRadius))
		err = ItemPickup(pickupRadius)
		if err != nil {
			ctx.Logger.Warn("Failed to pickup items", slog.Any("error", err))
		}

		// Iterate through objects in the current room
		for _, o := range ctx.Data.Objects {
			if r.IsInside(o.Position) {
				shouldOpen := false
				if o.Selectable {
					// Global settings override per-run openChests.
					switch {
					case openSuperOnly:
						shouldOpen = o.IsSuperChest()
					case openAllChests:
						shouldOpen = o.IsChest() || o.IsSuperChest()
					case openChests:
						shouldOpen = o.IsChest()
					}
				}

				if shouldOpen {
					ctx.Logger.Debug(fmt.Sprintf(
						"Found chest. attempting to interact. Name=%s.\nID=%v UnitID=%v Pos=%v,%v Area='%s' InteractType=%v",
						o.Desc().Name,
						o.Name,
						o.ID,
						o.Position.X,
						o.Position.Y,
						ctx.Data.PlayerUnit.Area.Area().Name,
						o.InteractType,
					))

					err = MoveToCoords(o.Position)
					if err != nil {
						ctx.Logger.Warn("Failed moving to chest", slog.Any("error", err))
						continue
					}

					err = InteractObject(o, func() bool {
						chest, _ := ctx.Data.Objects.FindByID(o.ID)
						return !chest.Selectable
					})
					if err != nil {
						ctx.Logger.Warn("Failed interacting with chest", slog.Any("error", err))
					}

					// Add small delay to allow the game to open the chest and drop the content
					utils.Sleep(500)
				}
			}
		}
	}

	return nil
}

func clearRoom(room data.Room, filter data.MonsterFilter) error {
	ctx := context.Get()
	ctx.SetLastAction("clearRoom")

	path, _, found := ctx.PathFinder.GetClosestWalkablePath(room.GetCenter())
	if !found {
		return errors.New("failed to find a path to the room center")
	}

	to := data.Position{
		X: path.To().X + ctx.Data.AreaOrigin.X,
		Y: path.To().Y + ctx.Data.AreaOrigin.Y,
	}

	err := MoveToCoords(to, step.WithMonsterFilter(filter))
	if err != nil {
		return fmt.Errorf("failed moving to room center: %w", err)
	}

	for {
		ctx.PauseIfNotPriority()
		if err := checkPlayerDeath(ctx); err != nil {
			return err
		}

		monsters := getMonstersInRoom(room, filter)
		if len(monsters) == 0 {
			return nil
		}

		SortEnemiesByPriority(&monsters)

		// Pick the top-priority target after sorting.
		targetMonster := data.Monster{}
		targetPathFound := false

		for _, m := range monsters {
			if ctx.Char.ShouldIgnoreMonster(m) {
				continue
			}
			if _, _, foundPath := ctx.PathFinder.GetPath(m.Position); foundPath {
				targetMonster = m
				targetPathFound = true
				break
			}
		}

		if targetMonster.UnitID == 0 {
			//No valid targets, done
			return nil
		}

		if targetPathFound {
			if !ctx.Data.CanTeleport() {
				hasDoorBetween, door := ctx.PathFinder.HasDoorBetween(ctx.Data.PlayerUnit.Position, targetMonster.Position)
				if hasDoorBetween && door.Selectable {
					ctx.Logger.Debug("Door is blocking the path to the monster, moving closer")
					MoveTo(func() (data.Position, bool) { return door.Position, true })
				}
			}

			ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
				m, found := d.Monsters.FindByID(targetMonster.UnitID)
				if found && m.Stats[stat.Life] > 0 {
					return targetMonster.UnitID, true
				}

				return 0, false
			}, nil)
		}
	}
}

func getMonstersInRoom(room data.Room, filter data.MonsterFilter) []data.Monster {
	ctx := context.Get()
	ctx.SetLastAction("getMonstersInRoom")

	monstersInRoom := make([]data.Monster, 0)
	for _, m := range ctx.Data.Monsters.Enemies(filter) {
		// Fix operator precedence: alive AND (in room OR close to player).
		if m.Stats[stat.Life] <= 0 {
			continue
		}
		if !(room.IsInside(m.Position) || ctx.PathFinder.DistanceFromMe(m.Position) < 30) {
			continue
		}

		// Skip monsters that exist in data but are placed on non-walkable tiles (often "underwater/off-grid").
		// Keep Vizier exception (Chaos Sanctuary).
		isVizier := m.Type == data.MonsterTypeSuperUnique && m.Name == npc.StormCaster
		if !isVizier && !ctx.Data.AreaData.IsWalkable(m.Position) {
			continue
		}

		monstersInRoom = append(monstersInRoom, m)
	}

	return monstersInRoom
}
