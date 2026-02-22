package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type LamEsen struct {
	ctx *context.Status
}

func NewLamEsen() *LamEsen {
	return &LamEsen{
		ctx: context.Get(),
	}
}

func (le LamEsen) Name() string {
	return string(config.LamEsenRun)
}

func (le LamEsen) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}
	if !le.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerStop
	}
	if le.ctx.Data.Quests[quest.Act3LamEsensTome].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (le LamEsen) Run(parameters *RunParameters) error {
	le.ctx.Logger.Info("Starting Retrieve Book Quest...")

	_, tomeFound := le.ctx.Data.Inventory.Find("LamEsensTome", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube)

	err := action.WayPoint(area.KurastDocks)
	if err != nil {
		return err
	}

	if !tomeFound {
		err = action.WayPoint(area.KurastBazaar)
		if err != nil {
			return err
		}
		action.Buff()

		err = action.MoveToArea(area.RuinedTemple)
		if err != nil {
			return err
		}
		action.Buff()

		err = action.MoveTo(func() (data.Position, bool) {
			for _, o := range le.ctx.Data.Objects {
				if o.Name == object.LamEsensTome {
					return o.Position, true
				}
			}

			return data.Position{}, false
		})
		if err != nil {
			return err
		}

		action.ClearAreaAroundPlayer(30, data.MonsterAnyFilter())

		tome, found := le.ctx.Data.Objects.FindOne(object.LamEsensTome)
		if !found {
			return err
		}

		err = action.InteractObject(tome, func() bool {
			_, tomeFound := le.ctx.Data.Inventory.Find("LamEsensTome", item.LocationInventory, item.LocationGround)
			return tomeFound
		})
		if err != nil {
			return err
		}

		// Making sure we pick up the tome
		action.ItemPickup(10)

		err = action.ReturnTown()
		if err != nil {
			return err
		}
	}

	err = action.InteractNPC(npc.Alkor)
	if err != nil {
		return err
	}

	step.CloseAllMenus()

	return nil
}
