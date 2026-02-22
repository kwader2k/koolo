package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type Cube struct {
	ctx *context.Status
}

func NewCube() *Cube {
	return &Cube{
		ctx: context.Get(),
	}
}

func (c Cube) Name() string {
	return string(config.CubeRun)
}

func (c Cube) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}
	if !c.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed() {
		return SequencerStop
	}
	_, found := c.ctx.Data.Inventory.Find("HoradricCube", item.LocationInventory, item.LocationStash)
	if found {
		return SequencerSkip
	}
	return SequencerOk
}

func (c Cube) Run(parameters *RunParameters) error {
	c.ctx.Logger.Info("Starting Retrieve the Cube Quest...")

	err := action.WayPoint(area.LutGholein)
	if err != nil {
		return err
	}

	err = action.WayPoint(area.HallsOfTheDeadLevel2)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.HallsOfTheDeadLevel3)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveTo(func() (data.Position, bool) {
		chest, found := c.ctx.Data.Objects.FindOne(object.HoradricCubeChest)
		if found {
			c.ctx.Logger.Info("Horadric Cube chest found, moving to that room")
			return chest.Position, true
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())

	obj, found := c.ctx.Data.Objects.FindOne(object.HoradricCubeChest)
	if !found {
		return err
	}

	err = action.InteractObject(obj, func() bool {
		updatedObj, found := c.ctx.Data.Objects.FindOne(object.HoradricCubeChest)
		if found {
			if !updatedObj.Selectable {
				c.ctx.Logger.Debug("Interacted with Horadric Cube Chest")
			}
			return !updatedObj.Selectable
		}
		return false
	})
	if err != nil {
		return err
	}

	// Making sure we pick up the cube
	action.ItemPickup(10)

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	return nil
}
