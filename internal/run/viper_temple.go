package run

import (
	"errors"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/context"
)

type ViperTemple struct {
	ctx *context.Status
}

func NewViperTemple() *ViperTemple {
	return &ViperTemple{
		ctx: context.Get(),
	}
}

func (v ViperTemple) Name() string {
	return "viper_temple"
}

func (v ViperTemple) CheckConditions(parameters *RunParameters) SequencerResult {
	return SequencerOk
}

func (v ViperTemple) Run(parameters *RunParameters) error {
	v.ctx.Logger.Info("Starting Claw Viper Temple run")

	// 1. Travel to Lost City Waypoint
	if err := action.WayPoint(area.LostCity); err != nil {
		return err
	}

	// 2. Find the Valley of Snakes
	if err := action.MoveToArea(area.ValleyOfSnakes); err != nil {
		return err
	}

	// 3. Enter Claw Viper Temple Level 1
	if err := action.MoveToArea(area.ClawViperTempleLevel1); err != nil {
		return err
	}

	// 4. Enter Claw Viper Temple Level 2
	if err := action.MoveToArea(area.ClawViperTempleLevel2); err != nil {
		return err
	}

	// 5. Secure the Altar Room
	// Turn off looting so she doesn't try to grab items while Fangskin is charging her!
	v.ctx.DisableItemPickup()
	v.ctx.Logger.Info("Disabling loot to clear Fangskin and minions...")

	altar, found := v.ctx.Data.Objects.FindOne(object.TaintedSunAltar)
	if found {
		// Teleport to the altar and kill everything within 30 yards
		action.MoveToCoords(altar.Position)
		action.ClearAreaAroundPlayer(30, data.MonsterAnyFilter())

		// 6. Break the Altar
		v.ctx.Logger.Info("Breaking the Tainted Sun Altar!")
		action.InteractObject(altar, func() bool {
			obj, stillFound := v.ctx.Data.Objects.FindOne(object.TaintedSunAltar)
			return !stillFound || !obj.Selectable
		})
	} else {
		return errors.New("tainted sun altar not found")
	}

	// 7. Re-enable loot and pick up the Amulet of the Viper!
	v.ctx.EnableItemPickup()
	v.ctx.Logger.Info("Room secure. Looting the Amulet!")
	action.ItemPickup(30)

	return nil
}