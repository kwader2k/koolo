package run

import (
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type JadeFigurine struct {
	ctx *context.Status
}

func NewJadeFigurine() *JadeFigurine {
	return &JadeFigurine{
		ctx: context.Get(),
	}
}

func (jf JadeFigurine) Name() string {
	return string(config.JadeFigurineRun)
}

func (jf JadeFigurine) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}
	if !jf.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerStop
	}
	if _, potionFound := jf.ctx.Data.Inventory.Find("PotionOfLife", item.LocationInventory); potionFound {
		return SequencerOk
	}
	q := jf.ctx.Data.Quests[quest.Act3TheGoldenBird]
	if q.NotStarted() || q.Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (jf JadeFigurine) Run(parameters *RunParameters) error {
	if jf.ctx.Data.PlayerUnit.Area != area.KurastDocks {
		action.WayPoint(area.KurastDocks)
	}

	_, jadefigureFound := jf.ctx.Data.Inventory.Find("AJadeFigurine", item.LocationInventory)
	if jadefigureFound {
		action.InteractNPC(npc.Meshif2)
	}

	_, goldenbirdFound := jf.ctx.Data.Inventory.Find("TheGoldenBird", item.LocationInventory)
	if goldenbirdFound {
		// Talk to Alkor
		action.InteractNPC(npc.Alkor)
		action.InteractNPC(npc.Ormus)
		action.InteractNPC(npc.Alkor)
		utils.Sleep(500)
	}

	lifepotion, lifepotfound := jf.ctx.Data.Inventory.Find("PotionOfLife", item.LocationInventory)
	if lifepotfound {
		jf.ctx.HID.PressKeyBinding(jf.ctx.Data.KeyBindings.Inventory)
		screenPos := ui.GetScreenCoordsForItem(lifepotion)
		jf.ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
		step.CloseAllMenus()
	}
	return nil
}
