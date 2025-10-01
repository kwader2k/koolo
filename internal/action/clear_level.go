package action

import (
	"errors"
	"fmt"
	"log/slog"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// interactableShrines is a list of shrine types that the bot should interact with.
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
	ctx := context.Get()
	ctx.SetLastAction("ClearCurrentLevel")

	// We can make this configurable later, but 20 is a good starting radius.
	const pickupRadius = 20

	rooms := ctx.PathFinder.OptimizeRoomsTraverseOrder()
	for _, r := range rooms {
		// First, clear the room of monsters
		err := clearRoom(r, filter)
		if err != nil {
			ctx.Logger.Warn("Failed to clear room: %v", err)
		}

		ctx.Logger.Debug(fmt.Sprintf("Clearing room complete, attempting to pickup items in a radius of %d", pickupRadius))
		err = ItemPickup(pickupRadius)
		if err != nil {
			ctx.Logger.Warn("Failed to pickup items", slog.Any("error", err))
		}

		// Iterate through objects in the current room
		for _, o := range ctx.Data.Objects {
			if r.IsInside(o.Position) {
				// Interact with chests if openChests is true
				if openChests && o.IsChest() && o.Selectable {
					ctx.Logger.Debug(fmt.Sprintf("Found chest. attempting to interact. Name=%s. ID=%v UnitID=%v Pos=%v,%v Area='%s' InteractType=%v", o.Desc().Name, o.Name, o.ID, o.Position.X, o.Position.Y, ctx.Data.PlayerUnit.Area.Area().Name, o.InteractType))
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
					utils.Sleep(500) // Add small delay to allow the game to open the chest and drop the content
				}

				// Interact with desired shrine types
				if ctx.CharacterCfg.Game.InteractWithShrines && o.IsShrine() && o.Selectable {
					for _, shrineType := range interactableShrines {
						if o.Shrine.ShrineType == shrineType {
							ctx.Logger.Debug(fmt.Sprintf("Found %s shrine. attempting to interact. Name=%s. ID=%v UnitID=%v Pos=%v,%v Area='%s' InteractType=%v", o.Desc().Name, o.Desc().Name, o.ID, o.Position.X, o.Position.Y, ctx.Data.PlayerUnit.Area.Area().Name, o.InteractType))
							err = MoveToCoords(o.Position)
							if err != nil {
								ctx.Logger.Warn("Failed moving to shrine", slog.Any("error", err))
								continue
							}
							err = InteractObject(o, func() bool {
								shrine, _ := ctx.Data.Objects.FindByID(o.ID)
								return !shrine.Selectable
							})
							if err != nil {
								ctx.Logger.Warn("Failed interacting with shrine", slog.Any("error", err))
							}
							utils.Sleep(500)
							break
						}
					}
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
	err := MoveToCoords(to)
	if err != nil {
		return fmt.Errorf("failed moving to room center: %w", err)
	}

	for {
		monsters := getMonstersInRoom(room, filter)
		if len(monsters) == 0 {
			return nil
		}

		// Check if there are monsters that can summon new monsters, and kill them first
		targetMonster := monsters[0]
		for _, m := range monsters {
			if m.IsMonsterRaiser() {
				targetMonster = m
				break
			}
		}

		_, _, mPathFound := ctx.PathFinder.GetPath(targetMonster.Position)
		if mPathFound {
			if !ctx.Data.CanTeleport() {
				hasDoorBetween, door := ctx.PathFinder.HasDoorBetween(ctx.Data.PlayerUnit.Position, targetMonster.Position)

				if hasDoorBetween && door.Selectable {
					ctx.Logger.Debug("Door is blocking the path to the monster, moving closer")
					MoveTo(func() (data.Position, bool) {
						return door.Position, true
					})
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
		if m.Stats[stat.Life] > 0 && room.IsInside(m.Position) || ctx.PathFinder.DistanceFromMe(m.Position) < 30 {
			monstersInRoom = append(monstersInRoom, m)
		}
	}

	return monstersInRoom
}
