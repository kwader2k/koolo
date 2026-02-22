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
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Radament struct {
	ctx *context.Status
}

func NewRadament() *Radament {
	return &Radament{
		ctx: context.Get(),
	}
}

func (r Radament) Name() string {
	return string(config.RadamentRun)
}

func (r Radament) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		if !r.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed() || !r.ctx.Data.Quests[quest.Act2RadamentsLair].Completed() {
			return SequencerSkip
		}
		return SequencerOk
	}
	if !r.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed() {
		return SequencerStop
	}
	_, found := r.ctx.Data.Inventory.Find("BookofSkill")
	if !found && r.ctx.Data.Quests[quest.Act2RadamentsLair].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (r Radament) Run(parameters *RunParameters) error {
	if _, found := r.ctx.Data.Inventory.Find("BookofSkill"); found {
		return r.finishQuest()
	}

	var startingPositionAtma = data.Position{
		X: 5138,
		Y: 5057,
	}

	r.ctx.Logger.Info("Starting Kill Radament Quest...")

	err := action.WayPoint(area.LutGholein)
	if err != nil {
		return err
	}

	err = action.WayPoint(area.SewersLevel2Act2)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.SewersLevel3Act2)
	if err != nil {
		return err
	}
	action.Buff()

	// cant find npc.Radament for some reason, using the sparkly chest with ID 355 next him to find him
	err = action.MoveTo(func() (data.Position, bool) {
		for _, o := range r.ctx.Data.Objects {
			if o.Name == object.Name(355) {
				return o.Position, true
			}
		}

		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(30, data.MonsterAnyFilter())

	if IsQuestRun(parameters) {
		// Sometimes it moves too far away from the book to pick it up, making sure it moves back to the chest
		err = action.MoveTo(func() (data.Position, bool) {
			for _, o := range r.ctx.Data.Objects {
				if o.Name == object.Name(355) {
					return o.Position, true
				}
			}

			return data.Position{}, false
		})
		if err != nil {
			return err
		}

		// If its still too far away, we're making sure it detects it
		action.ItemPickup(50)

		err = action.ReturnTown()
		if err != nil {
			return err
		}

		utils.PingSleep(utils.Medium, 1000)

		err = action.MoveToCoords(startingPositionAtma)
		if err != nil {
			return err
		}

		err = r.finishQuest()
		if err != nil {
			return err
		}
	}

	return nil
}

func (r Radament) finishQuest() error {
	err := action.InteractNPC(npc.Atma)
	if err != nil {
		return err
	}

	step.CloseAllMenus()
	r.ctx.HID.PressKeyBinding(r.ctx.Data.KeyBindings.Inventory)
	itm, _ := r.ctx.Data.Inventory.Find("BookofSkill")
	screenPos := ui.GetScreenCoordsForItem(itm)
	utils.Sleep(200)
	r.ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
	step.CloseAllMenus()
	return nil
}
