package run

import (
	"errors"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Gidbinn struct {
	ctx *context.Status
}

func NewGidbinn() *Gidbinn {
	return &Gidbinn{
		ctx: context.Get(),
	}
}

func (g Gidbinn) Name() string {
	return string(config.GidbinnRun)
}

func (g Gidbinn) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}

	if !g.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerStop
	}

	if g.ctx.Data.Quests[quest.Act3BladeOfTheOldReligion].Completed() {
		return SequencerSkip
	}

	return SequencerOk
}

func (g Gidbinn) Run(parameters *RunParameters) error {
	if g.hasGidbinn() || g.ctx.Data.Quests[quest.Act3BladeOfTheOldReligion].HasStatus(quest.StatusInProgress5) {
		err := action.WayPoint(area.KurastDocks)
		if err != nil {
			return err
		}
		return g.finishQuest()
	}

	// Use waypoint to FlayerJungle
	err := action.WayPoint(area.FlayerJungle)
	if err != nil {
		return err
	}

	altar, found := g.ctx.Data.Objects.FindOne(object.GidbinnAltarDecoy)
	if !found {
		return errors.New("couldn't find gidbinn altar")
	}

	err = action.MoveToCoords(altar.Position)
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(50, data.MonsterAnyFilter())

	err = action.MoveToCoords(altar.Position)
	if err != nil {
		return err
	}

	action.InteractObject(altar, func() bool {
		obj, found := g.ctx.Data.Objects.FindOne(object.GidbinnAltarDecoy)
		return !found || !obj.Selectable
	})

	utils.PingSleep(utils.Medium, 5000)

	for range 5 {
		action.ClearAreaAroundPlayer(50, data.MonsterAnyFilter())
		action.ItemPickup(30)
		err = action.MoveToCoords(altar.Position)
		if err != nil {
			return err
		}
		action.ItemPickup(30)
		if g.hasGidbinn() {
			break
		}

		utils.PingSleep(utils.Medium, 2000)
	}

	if !g.hasGidbinn() {
		return errors.New("didn't find gidbinn")
	}

	action.ReturnTown()

	return g.finishQuest()
}

func (g Gidbinn) hasGidbinn() bool {
	_, foundTG := g.ctx.Data.Inventory.Find("TheGidbinn")
	return foundTG
}

func (g Gidbinn) finishQuest() error {
	err := action.InteractNPC(npc.Ormus)
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.Asheara)
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.Ormus)
	if err != nil {
		return err
	}

	return nil
}
