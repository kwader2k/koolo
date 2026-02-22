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

type KhalimsEye struct {
	ctx *context.Status
}

func NewKhalimsEye() *KhalimsEye {
	return &KhalimsEye{
		ctx: context.Get(),
	}
}

func (ke KhalimsEye) Name() string {
	return string(config.KhalimsEyeRun)
}

func (ke KhalimsEye) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}

	if !ke.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerStop
	}

	if ke.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		return SequencerSkip
	}

	if _, found := ke.ctx.Data.Inventory.Find("KhalimsEye", item.LocationInventory, item.LocationStash, item.LocationCube); found {
		return SequencerSkip
	}

	if _, found := ke.ctx.Data.Inventory.Find("KhalimsWill", item.LocationInventory, item.LocationStash, item.LocationEquipped); found {
		return SequencerSkip
	}

	return SequencerOk
}

func (ke KhalimsEye) Run(parameters *RunParameters) error {
	err := action.WayPoint(area.SpiderForest)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.SpiderCavern)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveTo(func() (data.Position, bool) {
		chest, found := ke.ctx.Data.Objects.FindOne(object.KhalimChest3)
		if found {
			ke.ctx.Logger.Info("Khalm Chest found, moving to that room")
			return chest.Position, true
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())

	kalimchest3, found := ke.ctx.Data.Objects.FindOne(object.KhalimChest3)
	if !found {
		ke.ctx.Logger.Debug("Khalim Chest not found")
	}

	err = action.InteractObject(kalimchest3, func() bool {
		chest, _ := ke.ctx.Data.Objects.FindOne(object.KhalimChest3)
		return !chest.Selectable
	})
	if err != nil {
		return err
	}

	action.ItemPickup(15)

	return nil
}
