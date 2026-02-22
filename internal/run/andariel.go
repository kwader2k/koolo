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

const antidotePotionsToDrink = 10 // Number of antidote potions to consume (counts separately for the character and the mercenary)

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
	useAntidotes := a.ctx.CharacterCfg.Game.Andariel.UseAntidotes || isLevelingChar

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

	// Buy and consume antidotes right after entering Catacombs 4.
	if useAntidotes {
		if err := action.ReturnTown(); err != nil {
			return err
		}
		mercAlive := a.ctx.Data.MercHPPercent() > 0
		if err := a.buyAndDrinkAntidotePotions(mercAlive); err != nil {
			return err
		}
		if err := action.UsePortalInTown(); err != nil {
			return err
		}
	}

	// Regular characters use the simpler clear pathing, leveling uses the robust variant.
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

// Consume antidotes from the inventory only, optionally feeding the mercenary.
func (a Andariel) drinkAntidotePotions(selfTarget, mercTarget int) (int, int) {
	mercAlive := a.ctx.Data.MercHPPercent() > 0
	if selfTarget < 0 {
		selfTarget = 0
	}
	if mercTarget < 0 {
		mercTarget = 0
	}
	if !mercAlive {
		mercTarget = 0
	}
	if selfTarget == 0 && mercTarget == 0 {
		return 0, 0
	}
	shouldGiveMerc := mercAlive && mercTarget > 0
	reHidePortraits := false
	if shouldGiveMerc && a.ctx.CharacterCfg.HidePortraits && !a.ctx.Data.OpenMenus.PortraitsShown {
		a.ctx.CharacterCfg.HidePortraits = false
		reHidePortraits = true
		a.ctx.HID.PressKey(a.ctx.Data.KeyBindings.ShowPortraits.Key1[0])
		utils.Sleep(200)
	}

	a.ctx.HID.PressKeyBinding(a.ctx.Data.KeyBindings.Inventory)
	utils.Sleep(300)

	selfCount := 0
	mercCount := 0
	for _, itm := range a.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if itm.Name != "AntidotePotion" {
			continue
		}
		pos := ui.GetScreenCoordsForItem(itm)
		utils.Sleep(500)

		if selfCount < selfTarget {
			a.ctx.HID.Click(game.RightButton, pos.X, pos.Y)
			selfCount++
			continue
		}

		if mercCount < mercTarget {
			a.ctx.HID.Click(game.LeftButton, pos.X, pos.Y)
			utils.Sleep(300)
			if a.ctx.Data.LegacyGraphics {
				a.ctx.HID.Click(game.LeftButton, ui.MercAvatarPositionXClassic, ui.MercAvatarPositionYClassic)
			} else {
				a.ctx.HID.Click(game.LeftButton, ui.MercAvatarPositionX, ui.MercAvatarPositionY)
			}
			mercCount++
			continue
		}

		a.ctx.HID.Click(game.RightButton, pos.X, pos.Y)
		selfCount++
	}
	step.CloseAllMenus()

	if reHidePortraits {
		a.ctx.CharacterCfg.HidePortraits = true
		_ = action.HidePortraits()
	}

	return selfCount, mercCount
}

// Count only free, unlocked inventory cells.
func (a Andariel) countFreeInventorySlots() int {
	occupied := [4][10]bool{}
	for _, i := range a.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		w := i.Desc().InventoryWidth
		h := i.Desc().InventoryHeight
		for y := 0; y < h; y++ {
			for x := 0; x < w; x++ {
				if i.Position.Y+y < 4 && i.Position.X+x < 10 {
					occupied[i.Position.Y+y][i.Position.X+x] = true
				}
			}
		}
	}

	for y, row := range a.ctx.CharacterCfg.Inventory.InventoryLock {
		if y < 4 {
			for x, cell := range row {
				if x < 10 && cell == 0 {
					occupied[y][x] = true
				}
			}
		}
	}

	free := 0
	for y := 0; y < 4; y++ {
		for x := 0; x < 10; x++ {
			if !occupied[y][x] {
				free++
			}
		}
	}
	return free
}

// Buy antidotes in batches based on free inventory space, then consume them.
func (a Andariel) buyAndDrinkAntidotePotions(mercAlive bool) error {
	selfTarget := antidotePotionsToDrink
	mercTarget := 0
	if mercAlive {
		mercTarget = antidotePotionsToDrink
	}

	drinkAndLog := func(selfTarget, mercTarget int) (int, int) {
		selfCount, mercCount := a.drinkAntidotePotions(selfTarget, mercTarget)
		a.ctx.Logger.Info("Antidote potions consumed", "self", selfCount, "merc", mercCount)
		return selfCount, mercCount
	}

	selfCount, mercCount := drinkAndLog(selfTarget, mercTarget)
	selfTarget -= selfCount
	mercTarget -= mercCount
	if selfTarget < 0 {
		selfTarget = 0
	}
	if mercTarget < 0 {
		mercTarget = 0
	}

	for selfTarget > 0 || mercTarget > 0 {
		freeSlots := a.countFreeInventorySlots()
		if freeSlots == 0 {
			a.ctx.Logger.Warn("Not enough inventory space to buy antidote potions", "free", freeSlots, "required", selfTarget+mercTarget)
			return nil
		}

		batch := min(freeSlots, selfTarget+mercTarget)
		if err := a.buyAntidotePotions(batch); err != nil {
			return err
		}
		a.ctx.RefreshGameData()

		selfCount, mercCount = drinkAndLog(selfTarget, mercTarget)
		if selfCount == 0 && mercCount == 0 {
			a.ctx.Logger.Warn("Failed to consume antidote potions after purchase", "remaining", selfTarget+mercTarget)
			return nil
		}

		selfTarget -= selfCount
		mercTarget -= mercCount
		if selfTarget < 0 {
			selfTarget = 0
		}
		if mercTarget < 0 {
			mercTarget = 0
		}
	}

	return nil
}

func (a Andariel) buyAntidotePotions(quantity int) error {
	if quantity <= 0 {
		return nil
	}
	a.ctx.Logger.Info("Buying antidote potions from Akara", "quantity", quantity)
	if err := action.InteractNPC(npc.Akara); err != nil {
		return err
	}
	if err := action.BuyAtVendor(npc.Akara, action.VendorItemRequest{
		Item:     "AntidotePotion",
		Quantity: quantity,
		Tab:      4,
	}); err != nil {
		return err
	}
	return step.CloseAllMenus()
}

func (a Andariel) goToAct2() {
	action.ReturnTown()
	action.InteractNPC(npc.Warriv)
	a.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
	utils.Sleep(1000)
	action.HoldKey(win.VK_SPACE, 2000)
	utils.Sleep(1000)
}
