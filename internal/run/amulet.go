package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type Amulet struct {
	ctx *context.Status
}

func NewAmulet() *Amulet {
	return &Amulet{
		ctx: context.Get(),
	}
}

func (a Amulet) Name() string {
	return string(config.AmuletRun)
}

func (a Amulet) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}

	if !a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed() {
		return SequencerStop
	}

	if a.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() || a.ctx.Data.Quests[quest.Act2TheHoradricStaff].Completed() {
		return SequencerSkip
	}

	//check if already have amulet
	if _, horadricStaffFound := a.ctx.Data.Inventory.Find("HoradricStaff", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube); horadricStaffFound {
		return SequencerSkip
	}
	if _, amuletFound := a.ctx.Data.Inventory.Find("AmuletOfTheViper", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube); amuletFound {
		return SequencerSkip
	}
	return SequencerOk
}

func (a Amulet) Run(parameters *RunParameters) error {
	action.InteractNPC(npc.Drognan)

	err := action.WayPoint(area.LostCity)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.ValleyOfSnakes)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.ClawViperTempleLevel1)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.ClawViperTempleLevel2)
	if err != nil {
		return err
	}
	err = action.MoveTo(func() (data.Position, bool) {
		chest, found := a.ctx.Data.Objects.FindOne(object.TaintedSunAltar)
		if found {
			a.ctx.Logger.Info("Tainted Sun Altar found, moving to that room")
			return chest.Position, true
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	//action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())

	obj, found := a.ctx.Data.Objects.FindOne(object.TaintedSunAltar)
	if !found {
		return err
	}

	err = action.InteractObject(obj, func() bool {
		updatedObj, found := a.ctx.Data.Objects.FindOne(object.TaintedSunAltar)
		if found {
			if !updatedObj.Selectable {
				a.ctx.Logger.Debug("Interacted with Tainted Sun Altar")
			}
			return !updatedObj.Selectable
		}
		return false
	})
	if err != nil {
		return err
	}

	action.ReturnTown()

	// This stops us being blocked from getting into Palace
	action.InteractNPC(npc.Drognan)

	return nil
}
