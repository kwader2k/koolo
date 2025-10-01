package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/memory"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

func (a Leveling) act2() error {
	running := false

	if running || a.ctx.Data.PlayerUnit.Area != area.LutGholein {
		return nil
	}

	running = true

	action.UpdateQuestLog()

	// Buy a 12 slot belt if we don't have one
	if err := buyAct2Belt(a.ctx); err != nil {
		return err
	}

	a.AdjustDifficultyConfig()

	action.VendorRefill(true, true)

	lvl, _ := a.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)

	// Priority 0: Check if Act 2 is fully completed (Seven Tombs quest completed)
	if a.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() && lvl.Value >= 24 {
		a.ctx.Logger.Info("Act 2, The Seven Tombs quest completed. Moving to Act 3.")
		action.MoveToCoords(data.Position{
			X: 5195,
			Y: 5060,
		})
		action.InteractNPC(npc.Meshif)
		a.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
		utils.Sleep(1000)
		a.HoldKey(win.VK_SPACE, 2000) // Hold the Escape key (VK_ESCAPE or 0x1B) for 2000 milliseconds (2 seconds)
		utils.Sleep(1000)

		return nil
	}

	// Gold Farming Logic (and immediate return if farming is needed)
	if (a.ctx.CharacterCfg.Game.Difficulty == difficulty.Nightmare && a.ctx.Data.PlayerUnit.TotalPlayerGold() < 50000) ||
		(a.ctx.CharacterCfg.Game.Difficulty == difficulty.Hell && a.ctx.Data.PlayerUnit.TotalPlayerGold() < 70000) {

		return NewMausoleum().Run()
	}

	if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal && a.ctx.Data.PlayerUnit.TotalPlayerGold() < 10000 {
		return NewQuests().killRadamentQuest()
	}

	if a.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusInProgress6) {
		a.ctx.Logger.Info("Act 2, The Seven Tombs quest completed. Need to talk to Meshif and then move to Act 3.")
		action.MoveToCoords(data.Position{
			X: 5195,
			Y: 5060,
		})
		action.InteractNPC(npc.Meshif)
		a.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
		utils.Sleep(1000)
		a.HoldKey(win.VK_SPACE, 2000) // Hold the Escape key (VK_ESCAPE or 0x1B) for 2000 milliseconds (2 seconds)
		utils.Sleep(1000)
		return nil
	}

	// Frozen Aura Merc can be hired only in Nightmare difficulty
	if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Nightmare && a.ctx.Data.MercHPPercent() > 0 && a.ctx.CharacterCfg.Character.ShouldHireAct2MercFrozenAura {
		a.ctx.Logger.Info("Start Hiring merc with Frozen Aura")
		action.DrinkAllPotionsInInventory()

		a.ctx.Logger.Info("Un-equipping merc")
		if err := action.UnEquipMercenary(); err != nil {
			a.ctx.Logger.Error(fmt.Sprintf("Failed to unequip mercenary: %s", err.Error()))
			return err
		}

		// merc list works only in legacy mode
		if !a.ctx.Data.LegacyGraphics {
			a.ctx.Logger.Info("Switching to legacy mode to hire merc")
			a.ctx.HID.PressKey(a.ctx.Data.KeyBindings.LegacyToggle.Key1[0])
			utils.Sleep(500)
		}

		a.ctx.Logger.Info("Interacting with mercenary NPC")
		if err := action.InteractNPC(town.GetTownByArea(a.ctx.Data.PlayerUnit.Area).MercContractorNPC()); err != nil {
			return err
		}
		a.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
		utils.Sleep(2000)

		a.ctx.Logger.Info("Getting merc list")
		mercList := a.ctx.GameReader.GetMercList()

		// get the first with fronzen aura
		var mercToHire *memory.MercOption
		for i := range mercList {
			if mercList[i].Skill.ID == skill.HolyFreeze {
				mercToHire = &mercList[i]
				break
			}
		}

		if mercToHire == nil {
			a.ctx.Logger.Info("No merc with Frozen Aura found, cannot hire")
			return nil
		}

		a.ctx.Logger.Info(fmt.Sprintf("Hiring merc: %s with skill %s", mercToHire.Name, mercToHire.Skill.Name))
		keySequence := []byte{win.VK_HOME}
		for i := 0; i < mercToHire.Index; i++ {
			keySequence = append(keySequence, win.VK_DOWN)
		}
		keySequence = append(keySequence, win.VK_RETURN, win.VK_UP, win.VK_RETURN) // Select merc and confirm hire
		a.ctx.HID.KeySequence(keySequence...)

		a.ctx.CharacterCfg.Character.ShouldHireAct2MercFrozenAura = false

		if !a.ctx.CharacterCfg.ClassicMode && a.ctx.Data.LegacyGraphics {
			a.ctx.Logger.Info("Switching back to non-legacy mode")
			a.ctx.HID.PressKey(a.ctx.Data.KeyBindings.LegacyToggle.Key1[0])
			utils.Sleep(500)
		}

		if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
			a.ctx.Logger.Error(fmt.Sprintf("Failed to save character configuration: %s", err.Error()))
		}

		a.ctx.Logger.Info("Merc hired successfully, re-equipping merc")
		action.AutoEquip()
	}

	// Priority 2: Check if Duriel is defeated but not yet reported (StatusInProgress5)
	if a.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusInProgress5) {
		a.ctx.Logger.Info("The Seven Tombs quest in progress 5. Speaking to Jerhyn.")
		action.MoveToCoords(data.Position{
			X: 5092,
			Y: 5144,
		})
		action.InteractNPC(npc.Jerhyn)

		a.ctx.Logger.Info("Act 2, The Seven Tombs quest completed. Moving to Act 3.")
		action.MoveToCoords(data.Position{
			X: 5195,
			Y: 5060,
		})
		action.InteractNPC(npc.Meshif)
		a.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
		utils.Sleep(1000)
		a.HoldKey(win.VK_SPACE, 2000) // Hold the Escape key (VK_ESCAPE or 0x1B) for 2000 milliseconds (2 seconds)
		utils.Sleep(1000)
		return nil

	}

	// Priority 3: Your new logic - if Horadric Staff AND Summoner quests are done,
	// then we're ready for Duriel or leveling. This will only run if the above
	// (post-Duriel) conditions are false.
	horadricStaffQuestCompleted := a.ctx.Data.Quests[quest.Act2TheHoradricStaff].Completed()
	summonerQuestCompleted := a.ctx.Data.Quests[quest.Act2TheSummoner].Completed()

	if horadricStaffQuestCompleted && summonerQuestCompleted {
		a.ctx.Logger.Info("Horadric Staff and Summoner quests are completed. Proceeding to Duriel or leveling.")
		if lvl, _ := a.ctx.Data.PlayerUnit.FindStat(stat.Level, 0); lvl.Value < 24 {
			a.ctx.Logger.Info("Player not level 24, farming tombs before Duriel.")
			return NewTalRashaTombs().Run()
		}
		a.prepareStaff() // Make sure staff is prepared before Duriel
		return a.duriel()
	}

	// Find Horadric Cube
	_, found := a.ctx.Data.Inventory.Find("HoradricCube", item.LocationInventory, item.LocationStash)
	if found {
		a.ctx.Logger.Info("Horadric Cube found, skipping quest")
	} else {
		a.ctx.Logger.Info("Horadric Cube not found, starting quest")
		return NewQuests().getHoradricCube()
	}

	if lvl, _ := a.ctx.Data.PlayerUnit.FindStat(stat.Level, 0); lvl.Value < 18 {
		a.ctx.Logger.Info("Not yet level 18 yet. Leveling in Sewers.")
		a.ctx.CharacterCfg.Character.ClearPathDist = 10
		if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
			a.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())
		}

		return NewQuests().killRadamentQuest()

	}

	if lvl, _ := a.ctx.Data.PlayerUnit.FindStat(stat.Level, 0); lvl.Value == 18 {
		a.ctx.CharacterCfg.Character.ClearPathDist = 10
		if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
			a.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())
		}
	}

	if a.ctx.Data.Quests[quest.Act2TheHoradricStaff].HasStatus(quest.StatusInProgress4) {
		action.InteractNPC(npc.Drognan)
	}

	if !a.ctx.Data.Quests[quest.Act2TheHoradricStaff].Completed() {
		_, horadricStaffFound := a.ctx.Data.Inventory.Find("HoradricStaff", item.LocationInventory, item.LocationStash, item.LocationEquipped)

		// Find Staff of Kings
		_, found = a.ctx.Data.Inventory.Find("StaffOfKings", item.LocationInventory, item.LocationStash, item.LocationEquipped)
		if found || horadricStaffFound {
			a.ctx.Logger.Info("StaffOfKings found, skipping quest")
		} else {
			a.ctx.Logger.Info("StaffOfKings not found, starting quest")
			return a.findStaff()
		}

		// Find Amulet
		_, found = a.ctx.Data.Inventory.Find("AmuletOfTheViper", item.LocationInventory, item.LocationStash, item.LocationEquipped)
		if found || horadricStaffFound {
			a.ctx.Logger.Info("Amulet of the Viper found, skipping quest")
		} else {
			a.ctx.Logger.Info("Amulet of the Viper not found, starting quest")
			return a.findAmulet()
		}
	}

	if !a.ctx.Data.Quests[quest.Act2TheSummoner].Completed() && a.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusQuestNotStarted) {
		a.ctx.Logger.Info("Starting summoner quest (Summoner not yet completed).")
		action.InteractNPC(npc.Drognan)
		err := NewSummoner().Run()
		if err != nil {
			return err
		}

		// This block can be removed when https://github.com/hectorgimenez/koolo/pull/642 gets merged
		tome, found := a.ctx.Data.Objects.FindOne(object.YetAnotherTome)
		if !found {
			a.ctx.Logger.Error("YetAnotherTome (journal) not found after Summoner kill. This is unexpected.")
			return err // Or a more specific error/recovery
		}

		a.ctx.Logger.Debug("Interacting with the journal to open the portal.")
		// Try to use the portal and discover the waypoint
		err = action.InteractObject(tome, func() bool {
			_, found := a.ctx.Data.Objects.FindOne(object.PermanentTownPortal)
			return found
		})
		if err != nil {
			return err
		}
		a.ctx.Logger.Debug("Moving to red portal")
		portal, _ := a.ctx.Data.Objects.FindOne(object.PermanentTownPortal)

		err = action.InteractObject(portal, func() bool {
			return a.ctx.Data.PlayerUnit.Area == area.CanyonOfTheMagi && a.ctx.Data.AreaData.IsInside(a.ctx.Data.PlayerUnit.Position)
		})
		if err != nil {
			return err
		}
		// End of block for removal
		a.ctx.Logger.Info("Discovering Canyon of the Magi waypoint.")
		err = action.DiscoverWaypoint()
		if err != nil {
			return err
		}
		a.ctx.Logger.Info("Summoner quest chain (journal, portal, WP) completed.")
		action.UpdateQuestLog()
		return nil // Return to re-evaluate after completing this chain.
	}

	if a.ctx.Data.Quests[quest.Act2TheSummoner].Completed() && a.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusQuestNotStarted) {
		err := NewTalRashaTombs().Run()
		if err != nil {
			return err
		}

		// This block can be removed when https://github.com/hectorgimenez/koolo/pull/642 gets merged
		tome, found := a.ctx.Data.Objects.FindOne(object.YetAnotherTome)
		if !found {
			a.ctx.Logger.Error("YetAnotherTome (journal) not found after Summoner kill. This is unexpected.")
			return err // Or a more specific error/recovery
		}

		a.ctx.Logger.Debug("Interacting with the journal to open the portal.")
		// Try to use the portal and discover the waypoint
		err = action.InteractObject(tome, func() bool {
			_, found := a.ctx.Data.Objects.FindOne(object.PermanentTownPortal)
			return found
		})
		if err != nil {
			return err
		}
		a.ctx.Logger.Debug("Moving to red portal")
		portal, _ := a.ctx.Data.Objects.FindOne(object.PermanentTownPortal)

		err = action.InteractObject(portal, func() bool {
			return a.ctx.Data.PlayerUnit.Area == area.CanyonOfTheMagi && a.ctx.Data.AreaData.IsInside(a.ctx.Data.PlayerUnit.Position)
		})
		if err != nil {
			return err
		}
		// End of block for removal
		a.ctx.Logger.Info("Discovering Canyon of the Magi waypoint.")
		err = action.DiscoverWaypoint()
		if err != nil {
			return err
		}
		a.ctx.Logger.Info("Summoner quest chain (journal, portal, WP) completed.")
		action.UpdateQuestLog()
		return nil // Return to re-evaluate after completing this chain.
	}

	if !a.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusQuestNotStarted) {
		// Try to get level 24 before moving to Duriel and Act3

		if lvl, _ := a.ctx.Data.PlayerUnit.FindStat(stat.Level, 0); lvl.Value < 24 {
			a.ctx.Logger.Info("Player not level 24, farming tombs before Duriel.")
			return NewTalRashaTombs().Run()
		}

		a.prepareStaff()

		return a.duriel()
	}

	a.ctx.Logger.Debug("act2() function completed, no specific action triggered this tick. Returning nil.")
	return nil
}

// All other functions (findStaff, findAmulet, prepareStaff, duriel) remain EXACTLY as you provided them
// (no changes needed for these, as the core logic is in act2()).

func (a Leveling) findStaff() error {
	err := action.WayPoint(area.FarOasis)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.MaggotLairLevel1)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.MaggotLairLevel2)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.MaggotLairLevel3)
	if err != nil {
		return err
	}

	err = action.MoveTo(func() (data.Position, bool) {
		chest, found := a.ctx.Data.Objects.FindOne(object.StaffOfKingsChest)
		if found {
			a.ctx.Logger.Info("Staff Of Kings chest found, moving to that room")
			return chest.Position, true
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	if a.ctx.CharacterCfg.Game.Difficulty != difficulty.Hell {
		action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())
	}

	obj, found := a.ctx.Data.Objects.FindOne(object.StaffOfKingsChest)
	if !found {
		return err
	}

	err = action.InteractObject(obj, func() bool {
		updatedObj, found := a.ctx.Data.Objects.FindOne(object.StaffOfKingsChest)
		if found {
			return !updatedObj.Selectable
		}
		return false
	})
	if err != nil {
		return err
	}

	utils.Sleep(200)
	action.ItemPickup(-1)

	return nil
}

func (a Leveling) findAmulet() error {

	action.UpdateQuestLog()

	action.InteractNPC(npc.Drognan)

	err := action.WayPoint(area.LostCity)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.ValleyOfSnakes)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.ClawViperTempleLevel1)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.ClawViperTempleLevel2)
	if err != nil {
		return err
	}
	err = action.MoveTo(func() (data.Position, bool) {
		chest, found := a.ctx.Data.Objects.FindOne(object.TaintedSunAltar)
		if found {
			a.ctx.Logger.Info("Tainted Sun Altar found, moving to that room")
			return chest.Position, true
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	//action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())

	obj, found := a.ctx.Data.Objects.FindOne(object.TaintedSunAltar)
	if !found {
		return err
	}

	err = action.InteractObject(obj, func() bool {
		updatedObj, found := a.ctx.Data.Objects.FindOne(object.TaintedSunAltar)
		if found {
			if !updatedObj.Selectable {
				a.ctx.Logger.Debug("Interacted with Tainted Sun Altar")
			}
			return !updatedObj.Selectable
		}
		return false
	})
	if err != nil {
		return err
	}

	action.ReturnTown()

	// This stops us being blocked from getting into Palace
	action.InteractNPC(npc.Drognan)

	action.UpdateQuestLog()

	return nil
}

func (a Leveling) prepareStaff() error {
	horadricStaff, found := a.ctx.Data.Inventory.Find("HoradricStaff", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if found {
		a.ctx.Logger.Info("Horadric Staff found!")
		if horadricStaff.Location.LocationType == item.LocationStash {
			a.ctx.Logger.Info("It's in the stash, let's pick it up")

			bank, found := a.ctx.Data.Objects.FindOne(object.Bank)
			if !found {
				a.ctx.Logger.Info("bank object not found")
			}

			err := action.InteractObject(bank, func() bool {
				return a.ctx.Data.OpenMenus.Stash
			})
			if err != nil {
				return err
			}

			screenPos := ui.GetScreenCoordsForItem(horadricStaff)
			a.ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
			utils.Sleep(300)
			step.CloseAllMenus()

			return nil
		}
	}

	staff, found := a.ctx.Data.Inventory.Find("StaffOfKings", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if !found {
		a.ctx.Logger.Info("Staff of Kings not found, skipping")
		return nil
	}

	amulet, found := a.ctx.Data.Inventory.Find("AmuletOfTheViper", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if !found {
		a.ctx.Logger.Info("Amulet of the Viper not found, skipping")
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

func (a Leveling) duriel() error {
	a.ctx.Logger.Info("Starting Duriel....")
	a.ctx.CharacterCfg.Game.Duriel.UseThawing = true
	if err := NewDuriel().Run(); err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(30, a.DurielFilter())

	duriel, found := a.ctx.Data.Monsters.FindOne(npc.Duriel, data.MonsterTypeUnique)
	if !found || duriel.Stats[stat.Life] <= 0 || a.ctx.Data.Quests[quest.Act2TheSevenTombs].HasStatus(quest.StatusInProgress3) {

		action.MoveToCoords(data.Position{
			X: 22577,
			Y: 15600,
		})
		action.InteractNPC(npc.Tyrael)

	}

	action.ReturnTown()

	action.UpdateQuestLog()

	return nil
}

func buyAct2Belt(ctx *context.Status) error {
	// Check equipped and inventory for a suitable belt first
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationEquipped) {
		if itm.Name == "Belt" || itm.Name == "HeavyBelt" || itm.Name == "PlatedBelt" {
			ctx.Logger.Info("Already have a 12+ slot belt equipped, skipping.")
			return nil
		}
	}
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if itm.Name == "Belt" || itm.Name == "HeavyBelt" || itm.Name == "PlatedBelt" {
			ctx.Logger.Info("Already have a 12+ slot belt in inventory, skipping.")
			return nil
		}
	}

	// Check for gold before visiting the vendor
	if ctx.Data.PlayerUnit.TotalPlayerGold() < 1000 {
		ctx.Logger.Info("Not enough gold to buy a belt, skipping.")
		return nil
	}

	ctx.Logger.Info("No 12 slot belt found, trying to buy one from Fara.")
	if err := action.InteractNPC(npc.Fara); err != nil {
		return err
	}
	defer step.CloseAllMenus()

	ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN) // Interact with Fara
	utils.Sleep(1000)

	// Switch to armor tab and refresh game data to see the new items
	action.SwitchStashTab(1)
	ctx.RefreshGameData()
	utils.Sleep(500)

	// Find a suitable belt to buy from vendor
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationVendor) {
		// We are looking for a "Belt", which has 12 slots.
		if itm.Name == "Belt" {
			strReq := itm.Desc().RequiredStrength
			ctx.Logger.Debug("Vendor item found", "name", itm.Name, "strReq", strReq)

			if strReq <= 25 {
				ctx.Logger.Info("Found a suitable belt, buying it.", "name", itm.Name)
				town.BuyItem(itm, 1)
				ctx.Logger.Info("Belt purchased, running AutoEquip.")
				if err := action.AutoEquip(); err != nil {
					ctx.Logger.Error("AutoEquip failed after buying belt", "error", err)
				}

				return nil
			}
		}
	}

	ctx.Logger.Info("No suitable belt found at Fara.")
	return nil
}

func (a Leveling) RockyWaste() error {
	a.ctx.Logger.Info("Entering Rocky Waste for gold farming...")

	// Use action.MoveToArea to navigate to Rocky Waste, similar to the Izual quest.
	err := action.MoveToArea(area.RockyWaste)
	if err != nil {
		a.ctx.Logger.Error("Failed to move to Rocky Waste area: %v", err)
		return err // Return the error if navigation fails
	}
	a.ctx.Logger.Info("Successfully reached Rocky Waste.")

	// Attempt to clear the current level (Rocky Waste).
	err = action.ClearCurrentLevel(false, data.MonsterAnyFilter())
	if err != nil {
		a.ctx.Logger.Error("Failed to clear Rocky Waste area: %v", err)
		return err // Return the error if clearing fails
	}
	a.ctx.Logger.Info("Successfully cleared Rocky Waste area.")

	return nil // Return nil if all actions succeed
}

func (a Leveling) FarOasis() error {

	action.WayPoint(area.FarOasis)

	// Attempt to clear the current level (Far Oasis).
	err := action.ClearCurrentLevel(false, data.MonsterEliteFilter())
	if err != nil {
		a.ctx.Logger.Error("Failed to clear Far Oasis area: %v", err)
		return err // Return the error if clearing fails
	}

	return nil // Return nil if all actions succeed
}

func (a Leveling) DurielFilter() data.MonsterFilter {
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
