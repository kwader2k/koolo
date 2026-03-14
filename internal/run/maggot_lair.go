package run

import (
	"errors"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/context"
)

type MaggotLair struct {
	ctx *context.Status
}

func NewMaggotLair() *MaggotLair {
	return &MaggotLair{
		ctx: context.Get(),
	}
}

func (m MaggotLair) Name() string {
	return "maggot_lair"
}

func (m MaggotLair) CheckConditions(parameters *RunParameters) SequencerResult {
	return SequencerOk
}

func (m MaggotLair) Run(parameters *RunParameters) error {
	m.ctx.Logger.Info("Starting Maggot Lair run for the Staff of Kings")

	// 1. Travel to Far Oasis Waypoint
	if err := action.WayPoint(area.FarOasis); err != nil {
		return err
	}

	// 2. Find and enter Maggot Lair Level 1
	if err := action.MoveToArea(area.MaggotLairLevel1); err != nil {
		return err
	}

	// 3. Go down to Level 2
	if err := action.MoveToArea(area.MaggotLairLevel2); err != nil {
		return err
	}

	// 4. Go down to Level 3
	if err := action.MoveToArea(area.MaggotLairLevel3); err != nil {
		return err
	}

	// 5. Secure the Boss Room
	// Turn off looting so she isn't distracted by drops while fighting Coldworm the Burrower!
	m.ctx.DisableItemPickup()
	m.ctx.Logger.Info("Disabling loot to clear Coldworm and the bug swarm...")

	chest, found := m.ctx.Data.Objects.FindOne(object.StaffOfKingsChest)
	if found {
		// Teleport to the chest and blast everything within 30 yards
		action.MoveToCoords(chest.Position)
		action.ClearAreaAroundPlayer(30, data.MonsterAnyFilter())

		// 6. Open the special chest
		m.ctx.Logger.Info("Opening the Staff of Kings chest!")
		action.InteractObject(chest, func() bool {
			obj, stillFound := m.ctx.Data.Objects.FindOne(object.StaffOfKingsChest)
			return !stillFound || !obj.Selectable
		})
	} else {
		return errors.New("staff of kings chest not found")
	}

	// 7. Re-enable loot and pick up the Staff!
	m.ctx.EnableItemPickup()
	m.ctx.Logger.Info("Room secure. Looting the Staff of Kings!")
	action.ItemPickup(30)

	return nil
}