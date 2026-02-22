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
	"github.com/hectorgimenez/koolo/internal/utils"
)

type KhalimsBrain struct {
	ctx *context.Status
}

func NewKhalimsBrain() *KhalimsBrain {
	return &KhalimsBrain{
		ctx: context.Get(),
	}
}

func (kb KhalimsBrain) Name() string {
	return string(config.KhalimsBrainRun)
}

func (kb KhalimsBrain) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}

	if !kb.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerStop
	}

	if kb.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		return SequencerSkip
	}

	if _, found := kb.ctx.Data.Inventory.Find("KhalimsBrain", item.LocationInventory, item.LocationStash, item.LocationCube); found {
		return SequencerSkip
	}

	if _, found := kb.ctx.Data.Inventory.Find("KhalimsWill", item.LocationInventory, item.LocationStash, item.LocationEquipped); found {
		return SequencerSkip
	}

	return SequencerOk
}

func (kb KhalimsBrain) Run(parameters *RunParameters) error {

	// Use waypoint to FlayerJungle
	err := action.WayPoint(area.FlayerJungle)
	if err != nil {
		return err
	}

	// Move to FlayerDungeonLevel1
	if err = action.MoveToArea(area.FlayerDungeonLevel1); err != nil {
		return err
	}

	// Move to FlayerDungeonLevel2
	if err = action.MoveToArea(area.FlayerDungeonLevel2); err != nil {
		return err
	}

	// Move to FlayerDungeonLevel3
	if err = action.MoveToArea(area.FlayerDungeonLevel3); err != nil {
		return err
	}

	var khalimChest2 data.Object

	// Move to KhalimChest
	action.MoveTo(func() (data.Position, bool) {
		for _, o := range kb.ctx.Data.Objects {
			if o.Name == object.KhalimChest2 {
				khalimChest2 = o
				return o.Position, true
			}
		}
		return data.Position{}, false
	})

	// Clear monsters around player
	action.ClearAreaAroundPlayer(15, data.MonsterEliteFilter())

	// Open the chest
	err = action.InteractObject(khalimChest2, func() bool {
		for _, obj := range kb.ctx.Data.Objects {
			if obj.Name == object.KhalimChest2 && !obj.Selectable {
				return true
			}
		}
		return false
	})

	if err != nil {
		return err
	}

	utils.Sleep(500)
	action.ItemPickup(15)
	return nil
}
