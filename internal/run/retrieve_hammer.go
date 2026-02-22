package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type RetrieveHammer struct {
	ctx *context.Status
}

func NewRetrieveHammer() *RetrieveHammer {
	return &RetrieveHammer{
		ctx: context.Get(),
	}
}

func (rh RetrieveHammer) Name() string {
	return string(config.RetrieveHammerRun)
}

func (rh RetrieveHammer) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}
	if rh.ctx.Data.Quests[quest.Act1ToolsOfTheTrade].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (rh RetrieveHammer) Run(parameters *RunParameters) error {
	rh.ctx.Logger.Info("Starting Retrieve Hammer Quest...")

	err := action.WayPoint(area.RogueEncampment)
	if err != nil {
		return err
	}

	err = action.WayPoint(area.OuterCloister)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.Barracks)
	if err != nil {
		return err
	}

	err = action.MoveTo(func() (data.Position, bool) {
		for _, o := range rh.ctx.Data.Objects {
			if o.Name == object.Malus {
				return o.Position, true
			}
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(20, data.MonsterAnyFilter())

	malus, found := rh.ctx.Data.Objects.FindOne(object.Malus)
	if !found {
		rh.ctx.Logger.Debug("Malus not found")
	}

	err = action.InteractObject(malus, nil)
	if err != nil {
		return err
	}

	action.ItemPickup(20)

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.Charsi)
	if err != nil {
		return err
	}

	step.CloseAllMenus()

	return nil
}
