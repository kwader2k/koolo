package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

// special barb leveling handling
type bossEquipmentPreparer interface {
	PrepareBossEquipment(bossNPC npc.ID)
}

type Izual struct {
	ctx *context.Status
}

func NewIzual() *Izual {
	return &Izual{
		ctx: context.Get(),
	}
}

func (i Izual) Name() string {
	return string(config.IzualRun)
}

func (i Izual) CheckConditions(parameters *RunParameters) SequencerResult {
	if !i.ctx.Data.Quests[quest.Act3TheGuardian].Completed() {
		if IsFarmingRun(parameters) {
			return SequencerSkip
		}
		return SequencerStop
	}

	if !IsFarmingRun(parameters) && i.ctx.Data.Quests[quest.Act4TheFallenAngel].Completed() {
		return SequencerSkip
	}

	return SequencerOk
}

func (i Izual) Run(parameters *RunParameters) error {
	i.ctx.Logger.Info("Starting Kill Izual Quest...")

	action.WayPoint(area.ThePandemoniumFortress)

	err := action.MoveToArea(area.OuterSteppes)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.PlainsOfDespair)
	if err != nil {
		return err
	}
	action.Buff()

	//special barb leveling handling
	if preparer, ok := i.ctx.Char.(bossEquipmentPreparer); ok {
		preparer.PrepareBossEquipment(npc.Izual)
	}

	// Once Izual is found, move to him
	err = action.MoveTo(func() (data.Position, bool) {
		areaData := i.ctx.Data.Areas[area.PlainsOfDespair]
		izualNPC, found := areaData.NPCs.FindOne(npc.Izual)
		if !found {
			return data.Position{}, false
		}

		return izualNPC.Positions[0], true
	})
	if err != nil {
		return err
	}

	if _, corpseFound := i.ctx.Data.Corpses.FindOne(npc.Izual, data.MonsterTypeNone); !corpseFound {
		// Engage and kill Izual
		err = i.ctx.Char.KillIzual()
		if err != nil {
			return err
		}

		action.ItemPickup(30)
	}

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.Tyrael2)
	if err != nil {
		return err
	}

	if IsQuestRun(parameters) {
		err = action.UsePortalInTown()
		if err != nil {
			return err
		}
		err = action.MoveToArea(area.CityOfTheDamned)
		if err != nil {
			return err
		}
		err = action.DiscoverWaypoint()
		if err != nil {
			return err
		}
	}
	return nil
}
