package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type Shenk struct {
	ctx *context.Status
}

func NewShenk() *Shenk {
	return &Shenk{
		ctx: context.Get(),
	}
}

func (s Shenk) Name() string {
	return string(config.ShenkRun)
}

func (s Shenk) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		if !s.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() {
			return SequencerSkip
		}
		return SequencerOk
	}

	if !s.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() {
		return SequencerStop
	}

	if s.ctx.Data.Quests[quest.Act5SiegeOnHarrogath].Completed() {
		return SequencerSkip
	}

	return SequencerOk
}

func (s Shenk) Run(parameters *RunParameters) error {
	var shenkPosition = data.Position{
		X: 3895,
		Y: 5120,
	}

	s.ctx.Logger.Info("Starting Kill Shenk Quest...")

	err := action.WayPoint(area.Harrogath)
	if err != nil {
		return err
	}

	err = action.WayPoint(area.FrigidHighlands)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.BloodyFoothills)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToCoords(shenkPosition)
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(25, data.MonsterAnyFilter())
	action.ItemPickup(30)

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.Larzuk)
	if err != nil {
		return err
	}

	step.CloseAllMenus()

	return nil
}
