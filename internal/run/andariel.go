package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
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
	"github.com/lxn/win"
)

var andarielClearPos1 = data.Position{
	X: 22575,
	Y: 9634,
}

var andarielClearPos2 = data.Position{
	X: 22562,
	Y: 9636,
}

var andarielClearPos3 = data.Position{
	X: 22553,
	Y: 9636,
}

var andarielClearPos4 = data.Position{
	X: 22541,
	Y: 9636,
}

var andarielClearPos5 = data.Position{
	X: 22535,
	Y: 9630,
}

var andarielClearPos6 = data.Position{
	X: 22546,
	Y: 9618,
}

var andarielClearPos7 = data.Position{
	X: 22545,
	Y: 9604,
}

var andarielClearPos8 = data.Position{
	X: 22560,
	Y: 9590,
}

var andarielClearPos9 = data.Position{
	X: 22578,
	Y: 9588,
}

var andarielClearPos10 = data.Position{
	X: 22545,
	Y: 9590,
}

var andarielClearPos11 = data.Position{
	X: 22536,
	Y: 9589,
}

var andarielAttackPos1 = data.Position{
	X: 22547,
	Y: 9582,
}

var simpleAndarielStartingPosition = data.Position{
	X: 22561,
	Y: 9553,
}

var simpleAndarielClearPos1 = data.Position{
	X: 22570,
	Y: 9591,
}

var simpleAndarielClearPos2 = data.Position{
	X: 22547,
	Y: 9593,
}

var simpleAndarielClearPos3 = data.Position{
	X: 22533,
	Y: 9591,
}

var simpleAndarielClearPos4 = data.Position{
	X: 22535,
	Y: 9579,
}

var simpleAndarielClearPos5 = data.Position{
	X: 22548,
	Y: 9580,
}

var simpleAndarielAttackPos1 = data.Position{
	X: 22548,
	Y: 9570,
}

type Andariel struct {
	ctx *context.Status
}

func NewAndariel() *Andariel {
	return &Andariel{
		ctx: context.Get(),
	}
}

func (a Andariel) Name() string {
	return string(config.AndarielRun)
}

func (a Andariel) CheckConditions(parameters *RunParameters) SequencerResult {
	farmingRun := IsFarmingRun(parameters)
	needLeaveTown := (a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].HasStatus(quest.StatusRewardGranted+quest.StatusLeaveTown+quest.StatusEnterArea) &&
		!action.HasAnyQuestStartedOrCompleted(quest.Act2RadamentsLair, quest.Act2TheSevenTombs) &&
		a.ctx.Data.PlayerUnit.Area.Act() == 1)

	needToTalkToWarriv := a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].HasStatus(quest.StatusCompletedBefore) && !a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].HasStatus(quest.StatusRewardGranted)
	if needToTalkToWarriv {
		return SequencerOk
	}

	questCompleted := a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed()
	if (farmingRun && !questCompleted) || (!farmingRun && (questCompleted && !needLeaveTown)) {
		return SequencerSkip
	}
	return SequencerOk
}

func (a Andariel) Run(parameters *RunParameters) error {
	_, isLevelingChar := a.ctx.Char.(context.LevelingCharacter)

	if IsQuestRun(parameters) {
		needLeaveTown := a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].HasStatus(quest.StatusRewardGranted+quest.StatusLeaveTown+quest.StatusEnterArea) || a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].HasStatus(quest.StatusCompletedBefore)
		if needLeaveTown {
			a.goToAct2()
			return nil
		}
	}

	a.ctx.Logger.Info("Moving to Catacombs 4")
	err := action.WayPoint(area.CatacombsLevel2)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.CatacombsLevel3)
	action.MoveToArea(area.CatacombsLevel4)
	if err != nil {
		return err
	}

	if !isLevelingChar {

		if a.ctx.CharacterCfg.Game.Andariel.ClearRoom {
			a.ctx.Logger.Info("Clearing inside room (OLD/SIMPLE LOGIC)")
			action.MoveToCoords(simpleAndarielClearPos1)
			action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())
			action.MoveToCoords(simpleAndarielClearPos2)
			action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())
			action.MoveToCoords(simpleAndarielClearPos3)
			action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())
			action.MoveToCoords(simpleAndarielClearPos4)
			action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())
			action.MoveToCoords(simpleAndarielClearPos5)
			action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())
			action.MoveToCoords(simpleAndarielAttackPos1)
			action.ClearAreaAroundPlayer(20, data.MonsterAnyFilter())
		} else {
			action.MoveToCoords(simpleAndarielStartingPosition)
		}

		a.ctx.DisableItemPickup()
		action.MoveToCoords(simpleAndarielAttackPos1)

	} else {

		if a.ctx.CharacterCfg.Game.Andariel.ClearRoom {

			a.ctx.Logger.Info("Clearing inside room (NEW/ROBUST LOGIC)")
			action.MoveToCoords(andarielClearPos1)
			action.ClearAreaAroundPlayer(25, data.MonsterAnyFilter())
			action.MoveToCoords(andarielClearPos2)
			action.ClearAreaAroundPlayer(25, data.MonsterAnyFilter())
			action.MoveToCoords(andarielClearPos3)
			action.ClearAreaAroundPlayer(25, data.MonsterAnyFilter())
			action.MoveToCoords(andarielClearPos4)
			action.ClearAreaAroundPlayer(25, data.MonsterAnyFilter())
			action.MoveToCoords(andarielClearPos5)
			action.ClearAreaAroundPlayer(25, data.MonsterAnyFilter())
			action.MoveToCoords(andarielClearPos6)
			action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())
			action.MoveToCoords(andarielClearPos7)
			action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())
			action.MoveToCoords(andarielClearPos8)
			action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())
			action.MoveToCoords(andarielClearPos9)
			action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())
			action.MoveToCoords(andarielClearPos10)
			action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())
			action.MoveToCoords(andarielClearPos11)
			action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())

			if a.ctx.CharacterCfg.Game.Andariel.UseAntidoes {
				reHidePortraits := false
				action.ReturnTown()

				potsToBuy := 4
				if a.ctx.Data.MercHPPercent() > 0 {
					potsToBuy = 8
					if a.ctx.CharacterCfg.HidePortraits && !a.ctx.Data.OpenMenus.PortraitsShown {
						a.ctx.CharacterCfg.HidePortraits = false
						reHidePortraits = true
						a.ctx.HID.PressKey(a.ctx.Data.KeyBindings.ShowPortraits.Key1[0])
					}
				}

				action.VendorRefill(action.VendorRefillOpts{ForceRefill: true, SellJunk: true, BuyConsumables: true})
				action.BuyAtVendor(npc.Akara, action.VendorItemRequest{
					Item:     "AntidotePotion",
					Quantity: potsToBuy,
					Tab:      4,
				})

				a.ctx.HID.PressKeyBinding(a.ctx.Data.KeyBindings.Inventory)

				x := 0
				for _, itm := range a.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
					if itm.Name != "AntidotePotion" {
						continue
					}
					pos := ui.GetScreenCoordsForItem(itm)
					utils.Sleep(500)

					if x > 3 {

						a.ctx.HID.Click(game.LeftButton, pos.X, pos.Y)
						utils.Sleep(300)
						if a.ctx.Data.LegacyGraphics {
							a.ctx.HID.Click(game.LeftButton, ui.MercAvatarPositionXClassic, ui.MercAvatarPositionYClassic)
						} else {
							a.ctx.HID.Click(game.LeftButton, ui.MercAvatarPositionX, ui.MercAvatarPositionY)
						}

					} else {
						a.ctx.HID.Click(game.RightButton, pos.X, pos.Y)
					}
					x++
				}
				step.CloseAllMenus()

				if reHidePortraits {
					a.ctx.CharacterCfg.HidePortraits = true
				}
				action.HidePortraits()
				a.ctx.DisableItemPickup()
				action.UsePortalInTown()
			}

			a.ctx.DisableItemPickup()
			action.MoveToCoords(andarielAttackPos1)

			originalBackToTownCfg := a.ctx.CharacterCfg.BackToTown
			a.ctx.CharacterCfg.BackToTown.NoHpPotions = false
			a.ctx.CharacterCfg.BackToTown.NoMpPotions = false
			a.ctx.CharacterCfg.BackToTown.EquipmentBroken = false
			a.ctx.CharacterCfg.BackToTown.MercDied = false

			defer func() {
				a.ctx.CharacterCfg.BackToTown = originalBackToTownCfg
				a.ctx.Logger.Info("Restored original back-to-town checks after Andariel fight.")
			}()
		}

		if !a.ctx.CharacterCfg.Game.Andariel.ClearRoom {
			action.MoveToCoords(andarielAttackPos1)
		}
	}

	a.ctx.Logger.Info("Killing Andariel")
	err = a.ctx.Char.KillAndariel()

	a.ctx.EnableItemPickup()
	if err == nil {
		action.ItemPickup(30)
	}

	if IsQuestRun(parameters) {
		a.goToAct2()
	}

	return err
}

func (a Andariel) goToAct2() {
	action.ReturnTown()
	action.InteractNPC(npc.Warriv)
	a.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
	utils.Sleep(1000)
	action.HoldKey(win.VK_SPACE, 2000)
	utils.Sleep(1000)
}
