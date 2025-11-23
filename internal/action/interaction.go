package action

import (
	"fmt"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/game"
)

func InteractNPC(npc npc.ID) error {
	ctx := context.Get()
	ctx.SetLastAction("InteractNPC")

	pos, found := getNPCPosition(npc, ctx.Data)
	if !found {
		return fmt.Errorf("npc with ID %d not found", npc)
	}

	var err error
	for range 5 {
		err = MoveToCoords(pos)
		if err != nil {
			continue
		}

		err = step.InteractNPC(npc)
		if err != nil {
			continue
		}
		break
	}
	if err != nil {
		return err
	}

	event.Send(event.InteractedTo(event.Text(ctx.Name, ""), int(npc), event.InteractionTypeNPC))

	return nil
}

func InteractObject(o data.Object, isCompletedFn func() bool) error {
	ctx := context.Get()
	ctx.SetLastAction("InteractObject")

	pos := o.Position
	distFinish := step.DistanceToFinishMoving
	if ctx.Data.PlayerUnit.Area == area.RiverOfFlame && o.IsWaypoint() {
		pos = data.Position{X: 7800, Y: 5919}
		o.ID = 0
		// Special case for seals: we cant teleport directly to center. Interaction range is bigger then DistanceToFinishMoving so we modify it
	} else if strings.Contains(o.Desc().Name, "Seal") {
		distFinish = 10
	}

	var err error
	for range 5 {
		// For waypoints, check if we should use telekinesis
		if o.IsWaypoint() {
			// Check if telekinesis is enabled and available
			useTelekinesis := false
			switch ctx.CharacterCfg.Character.Class {
			case "sorceress":
				useTelekinesis = ctx.CharacterCfg.Character.BlizzardSorceress.UseTelekinesis
			case "nova":
				useTelekinesis = ctx.CharacterCfg.Character.NovaSorceress.UseTelekinesis
			case "lightsorc":
				useTelekinesis = ctx.CharacterCfg.Character.LightningSorceress.UseTelekinesis
			case "hydraorb":
				useTelekinesis = ctx.CharacterCfg.Character.HydraOrbSorceress.UseTelekinesis
			case "fireballsorc":
				useTelekinesis = ctx.CharacterCfg.Character.FireballSorceress.UseTelekinesis
			case "sorceress_leveling":
				useTelekinesis = ctx.CharacterCfg.Character.SorceressLeveling.UseTelekinesis
			}

			if useTelekinesis {
				// Only move if distance is greater than 21 (telekinesis max range)
				// Otherwise stay where we are and use telekinesis from current position
				distance := ctx.PathFinder.DistanceFromMe(pos)
				if distance > 21 {
					// Too far, move closer to 15 units (optimal TK range)
					err = step.MoveTo(pos, step.WithDistanceToFinish(15), step.WithIgnoreMonsters())
					if err != nil {
						continue
					}
				}
				// If distance <= 21, don't move at all - use telekinesis from current position
			} else if !ctx.Data.AreaData.Area.IsTown() {
				// Original behavior for non-town waypoints without TK - move directly to waypoint
				err = MoveToCoords(pos)
				if err != nil {
					continue
				}
			} else {
				// Town waypoints without TK - use default distance
				err = step.MoveTo(pos, step.WithDistanceToFinish(distFinish), step.WithIgnoreMonsters())
				if err != nil {
					continue
				}
			}
		} else {
			err = step.MoveTo(pos, step.WithDistanceToFinish(distFinish), step.WithIgnoreMonsters())
			if err != nil {
				continue
			}
		}

		err = step.InteractObject(o, isCompletedFn)
		if err != nil {
			continue
		}
		break
	}

	return err
}

func InteractObjectByID(id data.UnitID, isCompletedFn func() bool) error {
	ctx := context.Get()
	ctx.SetLastAction("InteractObjectByID")

	o, found := ctx.Data.Objects.FindByID(id)
	if !found {
		return fmt.Errorf("object with ID %d not found", id)
	}

	return InteractObject(o, isCompletedFn)
}

func getNPCPosition(npc npc.ID, d *game.Data) (data.Position, bool) {
	monster, found := d.Monsters.FindOne(npc, data.MonsterTypeNone)
	if found {
		return monster.Position, true
	}

	n, found := d.NPCs.FindOne(npc)
	if !found {
		return data.Position{}, false
	}

	return data.Position{X: n.Positions[0].X, Y: n.Positions[0].Y}, true
}
