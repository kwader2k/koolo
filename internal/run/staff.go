package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Staff struct {
	ctx *context.Status
}

func NewStaff() *Staff {
	return &Staff{
		ctx: context.Get(),
	}
}

func (s Staff) Name() string {
	return string(config.StaffRun)
}

func (s Staff) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}

	if !s.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed() {
		return SequencerStop
	}

	if s.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() || s.ctx.Data.Quests[quest.Act2TheHoradricStaff].Completed() {
		return SequencerSkip
	}

	//check if already have staff
	if _, horadricStaffFound := s.ctx.Data.Inventory.Find("HoradricStaff", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube); horadricStaffFound {
		return SequencerSkip
	}
	if _, staffFound := s.ctx.Data.Inventory.Find("StaffOfKings", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube); staffFound {
		return SequencerSkip
	}

	return SequencerOk
}

func (s Staff) Run(parameters *RunParameters) error {
	err := action.WayPoint(area.FarOasis)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.MaggotLairLevel1)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.MaggotLairLevel2)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.MaggotLairLevel3)
	if err != nil {
		return err
	}

	err = action.MoveTo(func() (data.Position, bool) {
		chest, found := s.ctx.Data.Objects.FindOne(object.StaffOfKingsChest)
		if found {
			s.ctx.Logger.Info("Staff Of Kings chest found, moving to that room")
			return chest.Position, true
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	if s.ctx.CharacterCfg.Game.Difficulty != difficulty.Hell {
		action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())
	}

	obj, found := s.ctx.Data.Objects.FindOne(object.StaffOfKingsChest)
	if !found {
		return err
	}

	err = action.InteractObject(obj, func() bool {
		updatedObj, found := s.ctx.Data.Objects.FindOne(object.StaffOfKingsChest)
		if found {
			return !updatedObj.Selectable
		}
		return false
	})
	if err != nil {
		return err
	}

	utils.Sleep(200)
	action.ItemPickup(-1)

	return nil
}
