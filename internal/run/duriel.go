package run

import (
	"errors"
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/mode"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

const (
	maxOrificeAttempts = 10
	orificeCheckDelay  = 200
)

var talTombs = []area.ID{area.TalRashasTomb1, area.TalRashasTomb2, area.TalRashasTomb3, area.TalRashasTomb4, area.TalRashasTomb5, area.TalRashasTomb6, area.TalRashasTomb7}

type Duriel struct {
	ctx *context.Status
}

func NewDuriel() *Duriel {
	return &Duriel{
		ctx: context.Get(),
	}
}

func (d Duriel) Name() string {
	return string(config.DurielRun)
}

func (d Duriel) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		if d.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
			return SequencerOk
		}
		return SequencerSkip
	} else if d.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerSkip
	}

	horadricStaffQuestCompleted := d.ctx.Data.Quests[quest.Act2TheHoradricStaff].Completed()
	summonerQuestCompleted := d.ctx.Data.Quests[quest.Act2TheSummoner].Completed()
	if horadricStaffQuestCompleted && summonerQuestCompleted {
		_, found := d.ctx.Data.Inventory.Find("HoradricStaff", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube)
		if found {
			return SequencerOk
		}
	}
	return SequencerStop
}

func (d Duriel) Run(parameters *RunParameters) error {

	if IsQuestRun(parameters) {
		//Try completing quest and early exit if possible
		d.tryTalkToJerhyn()
		if d.tryTalkToMeshif() {
			return nil
		}

		//prepare for quest
		if err := d.prepareStaff(); err != nil {
			return err
		}
	}

	err := action.WayPoint(area.CanyonOfTheMagi)
	if err != nil {
		return err
	}

	// Find and move to the real Tal Rasha tomb
	realTalRashaTomb, err := d.findRealTomb()
	if err != nil {
		return err
	}

	err = action.MoveToArea(realTalRashaTomb)
	if err != nil {
		return err
	}

	// Wait for area to fully load and get synchronized
	utils.Sleep(500)
	d.ctx.RefreshGameData()

	// Find orifice with retry logic
	var orifice data.Object
	var found bool

	for attempts := 0; attempts < maxOrificeAttempts; attempts++ {
		orifice, found = d.ctx.Data.Objects.FindOne(object.HoradricOrifice)
		if found && orifice.Mode == mode.ObjectModeOpened {
			break
		}
		utils.Sleep(orificeCheckDelay)
		d.ctx.RefreshGameData()
	}

	if !found {
		return errors.New("failed to find Duriel's Lair entrance after multiple attempts")
	}

	// Move to orifice and clear the area
	err = action.MoveToCoords(orifice.Position)
	if err != nil {
		return err
	}

	staff, ok := d.ctx.Data.Inventory.Find("HoradricStaff", item.LocationInventory)
	if !d.ctx.Data.Quests[quest.Act2TheHoradricStaff].Completed() && ok {

		err = action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())
		if err != nil {
			return err
		}

		action.InteractObject(orifice, func() bool {
			return d.ctx.Data.OpenMenus.Anvil
		})

		screenPos := ui.GetScreenCoordsForItem(staff)

		d.ctx.HID.Click(game.LeftButton, screenPos.X, screenPos.Y)
		utils.Sleep(300)
		if d.ctx.Data.LegacyGraphics {
			d.ctx.HID.Click(game.LeftButton, ui.AnvilCenterXClassic, ui.AnvilCenterYClassic)
			utils.Sleep(500)
			d.ctx.HID.Click(game.LeftButton, ui.AnvilBtnXClassic, ui.AnvilBtnYClassic)
		} else {
			d.ctx.HID.Click(game.LeftButton, ui.AnvilCenterX, ui.AnvilCenterY)
			utils.Sleep(500)
			d.ctx.HID.Click(game.LeftButton, ui.AnvilBtnX, ui.AnvilBtnY)
		}
		utils.Sleep(20000)
	}

	_, isLevelingChar := d.ctx.Char.(context.LevelingCharacter)
	if isLevelingChar && d.ctx.CharacterCfg.Game.Difficulty != difficulty.Hell {

		action.ClearAreaAroundPlayer(20, data.MonsterAnyFilter())

		action.ReturnTown()
		action.IdentifyAll(false)
		action.Stash(false)
		action.ReviveMerc()
		action.Repair()
		action.VendorRefill(true, true)

		err = action.UsePortalInTown()
		if err != nil {
			return err
		}
	}

	for _, obj := range d.ctx.Data.Areas[realTalRashaTomb].Objects {
		if obj.Name == object.HoradricOrifice {
			action.MoveToCoords(obj.Position)
		}
	}

	// Get thawing potions before entering Duriel's lair
	if d.ctx.CharacterCfg.Game.Duriel.UseThawing {
		d.ctx.Logger.Info("Returning to town for thawing potions before Duriel")

		// Use the existing portal from the previous town prep
		err = action.ReturnTown()
		if err != nil {
			return err
		}

		potsToBuy := 6
		if d.ctx.Data.MercHPPercent() > 0 && !d.ctx.CharacterCfg.HidePortraits {
			potsToBuy = 12
		}

		// Explicitly interact with Lysander for thawing potions
		d.ctx.Logger.Info("Buying thawing potions from Lysander")
		err = action.InteractNPC(npc.Lysander)
		if err != nil {
			return err
		}

		err = action.BuyAtVendor(npc.Lysander, action.VendorItemRequest{
			Item:     "ThawingPotion",
			Quantity: potsToBuy,
			Tab:      4,
		})
		if err != nil {
			return err
		}

		d.ctx.HID.PressKeyBinding(d.ctx.Data.KeyBindings.Inventory)
		utils.Sleep(300)

		x := 0
		for _, itm := range d.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
			if itm.Name != "ThawingPotion" {
				continue
			}
			pos := ui.GetScreenCoordsForItem(itm)
			utils.Sleep(500)

			if x > 5 {
				d.ctx.HID.Click(game.LeftButton, pos.X, pos.Y)
				utils.Sleep(300)
				if d.ctx.Data.LegacyGraphics {
					d.ctx.HID.Click(game.LeftButton, ui.MercAvatarPositionXClassic, ui.MercAvatarPositionYClassic)
				} else {
					d.ctx.HID.Click(game.LeftButton, ui.MercAvatarPositionX, ui.MercAvatarPositionY)
				}
			} else {
				d.ctx.HID.Click(game.RightButton, pos.X, pos.Y)
			}
			x++
		}
		step.CloseAllMenus()

		// Go back through the portal
		d.ctx.Logger.Info("Returning through portal to Duriel's entrance")
		err = action.UsePortalInTown()
		if err != nil {
			return err
		}

		// Move back to the orifice area
		for _, obj := range d.ctx.Data.Areas[realTalRashaTomb].Objects {
			if obj.Name == object.HoradricOrifice {
				action.MoveToCoords(obj.Position)
			}
		}
	}

	duriellair, found := d.ctx.Data.Objects.FindOne(object.DurielsLairPortal)
	if found {
		// Now enter Duriel's lair with thawing potions active
		action.InteractObject(duriellair, func() bool {
			return d.ctx.Data.PlayerUnit.Area == area.DurielsLair && d.ctx.Data.AreaData.IsInside(d.ctx.Data.PlayerUnit.Position)
		})
	}
	d.ctx.Logger.Debug(fmt.Sprintf("Quest Status %v", d.ctx.Data.Quests[quest.Act2TheSevenTombs]))

	d.ctx.Logger.Info("Killing Duriel")
	// Final refresh before fight
	d.ctx.RefreshGameData()

	utils.Sleep(700)

	if err := d.ctx.Char.KillDuriel(); err != nil {
		return err
	}

	if IsQuestRun(parameters) {
		action.ClearAreaAroundPlayer(30, d.durielFilter())

		duriel, found := d.ctx.Data.Monsters.FindOne(npc.Duriel, data.MonsterTypeUnique)
		if !found || duriel.Stats[stat.Life] <= 0 || d.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusInProgress3) {
			action.MoveToCoords(data.Position{
				X: 22577,
				Y: 15600,
			})
			action.InteractNPC(npc.Tyrael)
		}

		action.ReturnTown()

		action.UpdateQuestLog(false)

		if !d.tryTalkToJerhyn() {
			return errors.New("failed to talk to jerhyn")
		}

		if !d.tryTalkToMeshif() {
			return errors.New("failed to talk to meshif")
		}
	}

	return nil
}

func (d Duriel) findRealTomb() (area.ID, error) {
	var realTomb area.ID

	for _, tomb := range talTombs {
		for _, obj := range d.ctx.Data.Areas[tomb].Objects {
			if obj.Name == object.HoradricOrifice {
				realTomb = tomb
				break
			}
		}
	}

	if realTomb == 0 {
		return 0, errors.New("failed to find the real Tal Rasha tomb")
	}

	return realTomb, nil
}

func (d Duriel) prepareStaff() error {
	horadricStaff, found := d.ctx.Data.Inventory.Find("HoradricStaff", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if found {
		d.ctx.Logger.Info("Horadric Staff found!")
		if horadricStaff.Location.LocationType == item.LocationStash {
			d.ctx.Logger.Info("It's in the stash, let's pick it up")

			bank, found := d.ctx.Data.Objects.FindOne(object.Bank)
			if !found {
				d.ctx.Logger.Info("bank object not found")
			}

			err := action.InteractObject(bank, func() bool {
				return d.ctx.Data.OpenMenus.Stash
			})
			if err != nil {
				return err
			}

			screenPos := ui.GetScreenCoordsForItem(horadricStaff)
			d.ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
			utils.Sleep(300)
			step.CloseAllMenus()

			return nil
		}
	}

	staff, found := d.ctx.Data.Inventory.Find("StaffOfKings", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube)
	if !found {
		d.ctx.Logger.Info("Staff of Kings not found, skipping")
		return nil
	}

	amulet, found := d.ctx.Data.Inventory.Find("AmuletOfTheViper", item.LocationInventory, item.LocationStash, item.LocationEquipped, item.LocationCube)
	if !found {
		d.ctx.Logger.Info("Amulet of the Viper not found, skipping")
		return nil
	}

	err := action.CubeAddItems(staff, amulet)
	if err != nil {
		return err
	}

	err = action.CubeTransmute()
	if err != nil {
		return err
	}

	return nil
}

func (d Duriel) durielFilter() data.MonsterFilter {
	return func(a data.Monsters) []data.Monster {
		var filteredMonsters []data.Monster
		for _, mo := range a {
			if mo.Name == npc.Duriel {
				filteredMonsters = append(filteredMonsters, mo)
			}
		}

		return filteredMonsters
	}
}

func (d Duriel) tryTalkToJerhyn() bool {
	if d.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusInProgress5) {
		d.ctx.Logger.Info("The Seven Tombs quest in progress 5. Speaking to Jerhyn.")
		action.MoveToCoords(data.Position{
			X: 5092,
			Y: 5144,
		})
		action.InteractNPC(npc.Jerhyn)
		utils.Sleep(500)
		return true
	}
	return false
}

func (d Duriel) tryTalkToMeshif() bool {
	if d.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusInProgress6) {
		d.ctx.Logger.Info("Act 2, The Seven Tombs quest completed. Moving to Act 3.")
		action.MoveToCoords(data.Position{
			X: 5195,
			Y: 5060,
		})
		action.InteractNPC(npc.Meshif)
		d.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
		utils.Sleep(1000)
		action.HoldKey(win.VK_SPACE, 2000) // Hold the Escape key (VK_ESCAPE or 0x1B) for 2000 milliseconds (2 seconds)
		utils.Sleep(1000)
		return true
	}
	return false
}
