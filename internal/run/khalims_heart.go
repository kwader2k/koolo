package run

import (
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type KhalimsHeart struct {
	ctx *context.Status
}

func NewKhalimsHeart() *KhalimsHeart {
	return &KhalimsHeart{
		ctx: context.Get(),
	}
}

func (kh KhalimsHeart) Name() string {
	return string(config.KhalimsHeartRun)
}

func (kh KhalimsHeart) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}

	if !kh.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerStop
	}

	if kh.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		return SequencerSkip
	}

	if _, found := kh.ctx.Data.Inventory.Find("KhalimsHeart", item.LocationInventory, item.LocationStash, item.LocationCube); found {
		return SequencerSkip
	}

	if _, found := kh.ctx.Data.Inventory.Find("KhalimsWill", item.LocationInventory, item.LocationStash, item.LocationEquipped); found {
		return SequencerSkip
	}

	return SequencerOk
}

func (kh KhalimsHeart) Run(parameters *RunParameters) error {
	err := action.WayPoint(area.KurastBazaar)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.SewersLevel1Act3)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveTo(func() (data.Position, bool) {
		for _, l := range kh.ctx.Data.AdjacentLevels {
			if l.Area == area.SewersLevel2Act3 {
				return l.Position, true
			}
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())

	stairs, found := kh.ctx.Data.Objects.FindOne(object.Act3SewerStairsToLevel3)
	if !found {
		kh.ctx.Logger.Debug("Khalim Chest not found")
	}

	err = action.InteractObject(stairs, func() bool {
		o, _ := kh.ctx.Data.Objects.FindOne(object.Act3SewerStairsToLevel3)

		return !o.Selectable
	})
	if err != nil {
		return err
	}

	time.Sleep(4000 * time.Millisecond)

	err = action.MoveToArea(area.SewersLevel2Act3)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveTo(func() (data.Position, bool) {
		kh.ctx.Logger.Info("Khalm Chest found, moving to that room")
		chest, found := kh.ctx.Data.Objects.FindOne(object.KhalimChest1)

		return chest.Position, found
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())

	kalimchest1, found := kh.ctx.Data.Objects.FindOne(object.KhalimChest1)
	if !found {
		kh.ctx.Logger.Debug("Khalim Chest not found")
	}

	err = action.InteractObject(kalimchest1, func() bool {
		chest, _ := kh.ctx.Data.Objects.FindOne(object.KhalimChest1)
		return !chest.Selectable
	})
	if err != nil {
		return err
	}

	action.ItemPickup(15)

	return nil
}
