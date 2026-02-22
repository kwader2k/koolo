package run

import (
	"fmt"
	"math"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
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

// act1 is the main function for Act 1 leveling
func (a Leveling) act1() error {
	if a.ctx.Data.PlayerUnit.Area != area.RogueEncampment {
		return nil
	}

	// Check player level and set configuration for level 1
	lvl, _ := a.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)

	// Refill potions and ensure bindings for players level > 1
	if lvl.Value > 1 {
		action.VendorRefill(action.VendorRefillOpts{SellJunk: true, BuyConsumables: true})
		if err := action.EnsureSkillBindings(); err != nil {
			a.ctx.Logger.Error(fmt.Sprintf("Error ensuring skill bindings after vendor refill: %s", err.Error()))
		}
	}

	// --- Quest and Farming Logic ---

	// in case we're farming already, directly skip to a4 (we end up in a1 if we die while farming mausoleum)
	if a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed() {

		a.ctx.Logger.Info("Attempting to reach Act 4 via The Pandemonium Fortress waypoint.")
		err := action.WayPoint(area.ThePandemoniumFortress)
		if err == nil {
			a.ctx.Logger.Info("Successfully reached Act 4 via waypoint.")
			return nil
		} else {
			a.ctx.Logger.Info("Could not use waypoint to The Pandemonium Fortress. Falling back to manual portal entry.")
		}
	}

	// Farming for low gold
	if a.ctx.Data.PlayerUnit.TotalPlayerGold() < 50000 {
		if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Nightmare {
			return a.stonyField()
		}
		if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Hell {

			if a.ctx.Data.PlayerUnit.TotalPlayerGold() < 5000 {
				//set clearpathdistance high in eco run (also for sorc as we must assume, she may not be allowed to tele)
				a.ctx.CharacterCfg.Character.ClearPathDist = 20

			}

			return NewMausoleum().Run(nil)
		}
	}

	if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Hell && lvl.Value <= 75 {

		return NewMausoleum().Run(nil)
	}

	if !a.ctx.Data.Quests[quest.Act1DenOfEvil].Completed() && a.ctx.CharacterCfg.Game.Difficulty != difficulty.Hell {
		a.ctx.Logger.Debug("Completing Den of Evil")
		return NewQuests().clearDenQuest()
	}

	// Farming for normal difficulty below 300 gold
	if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal && a.ctx.Data.PlayerUnit.TotalPlayerGold() < 300 && !a.ctx.Data.Quests[quest.Act1SistersBurialGrounds].Completed() {
		return NewTristramEarlyGoldfarm().Run(nil)
	}

	// Blood Raven quest
	if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal && !a.ctx.Data.Quests[quest.Act1SistersBurialGrounds].Completed() {
		return a.killRavenGetMerc()
	}

	if !a.ctx.CharacterCfg.Character.UseMerc && a.ctx.Data.Quests[quest.Act1SistersBurialGrounds].Completed() {
		a.ctx.CharacterCfg.Character.UseMerc = true

		action.InteractNPC(npc.Kashya)
		if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
			a.ctx.Logger.Error(fmt.Sprintf("Failed to save character configuration: %s", err.Error()))

		}
	}

	// Buy a 9 slot belt if we are level 9 and don't have one yet
	if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal && a.ctx.Data.PlayerUnit.TotalPlayerGold() > 3000 && lvl.Value >= 9 && lvl.Value < 12 {
		if err := gambleAct1Belt(a.ctx); err != nil {
			return err
		}
	}

	const scrollInifussUnitID = 539
	const scrollInifussAfterAkara = 540

	found539InInv := false
	found540InInv := false

	for _, itm := range a.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if itm.ID == scrollInifussUnitID {
			found539InInv = true
		}
		if itm.ID == scrollInifussAfterAkara {
			found540InInv = true
		}
	}

	if found539InInv {
		a.ctx.Logger.Info("Unidentified Scroll of Inifuss found (ID 539). Interacting with Akara to proceed.")

		err := action.InteractNPC(npc.Akara)
		if err != nil {
			return err
		}
		found540InInv = true
	}
	// Cain quest: entering Tristram
	if ((a.ctx.Data.Quests[quest.Act1TheSearchForCain].HasStatus(quest.StatusStarted+quest.StatusLeaveTown) && !a.ctx.Data.Quests[quest.Act1TheSearchForCain].Completed()) ||
		found540InInv || a.ctx.Data.Quests[quest.Act1TheSearchForCain].HasStatus(quest.StatusLeaveTown+quest.StatusEnterArea)) &&
		!a.ctx.Data.Quests[quest.Act1TheSearchForCain].Completed() && a.ctx.CharacterCfg.Game.Difficulty != difficulty.Hell {
		return NewTristram().Run(nil)
	}

	// Cain quest: talking to Akara
	if !a.isCainInTown() && !a.ctx.Data.Quests[quest.Act1TheSearchForCain].Completed() && a.ctx.CharacterCfg.Game.Difficulty != difficulty.Hell {
		return NewQuests().rescueCainQuest()
	}

	// Tristram only until lvl 6, then Trist + Countess
	if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal && lvl.Value < 12 {

		if a.ctx.CharacterCfg.Character.Class == "sorceress_leveling" {
			a.ctx.CharacterCfg.Character.ClearPathDist = 4
		}

		NewTristram().Run(nil)

	}

	// Countess farming for runes
	if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal && a.ctx.Data.Quests[quest.Act1TheSearchForCain].Completed() && lvl.Value >= 6 && (lvl.Value < 12 || lvl.Value < 16 && (a.ctx.CharacterCfg.Character.Class == "paladin" || a.ctx.CharacterCfg.Character.Class == "necromancer" || a.ctx.CharacterCfg.Character.Class == "barb_leveling")) {
		// Special case for Barbarian Leveling
		if a.ctx.CharacterCfg.Character.Class == "barb_leveling" {
			if a.barbRunewords() {
				a.ctx.Logger.Info("Farming Countess for Steel/Malice runes.")
				return NewCountess().Run(nil)
			}
		} else {
			a.ctx.Logger.Info("Farming Countess for runes.")
			if a.ctx.CharacterCfg.Character.Class == "sorceress_leveling" {
				a.ctx.CharacterCfg.Character.ClearPathDist = 15
			}
			return NewCountess().Run(nil)
		}
	}

	if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Nightmare && lvl.Value < 50 && a.ctx.Data.Quests[quest.Act1DenOfEvil].Completed() && a.shouldFarmCountessForRunes() {
		a.ctx.Logger.Info("Farming Countess for required runes.")
		return NewCountess().Run(nil)
	}

	// Andariel or Act 2 transition
	if a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed() {
		// Go to Act 2
		return a.goToAct2()
	} else {
		// Run Andariel to complete quest

		if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal {

			a.ctx.CharacterCfg.Character.ClearPathDist = 15

			a.ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "mana"}
			if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
				a.ctx.Logger.Error(fmt.Sprintf("Failed to save character configuration: %s", err.Error()))

			}
		}
		return NewAndariel().Run(nil)
	}
}

// goToAct2 handles the transition to Act 2.
func (a Leveling) goToAct2() error {
	a.ctx.Logger.Info("Act 1 completed. Moving to Act 2.")
	action.ReturnTown()

	// Do Den of Evil if not complete before moving acts
	if !a.ctx.Data.Quests[quest.Act1DenOfEvil].Completed() && a.ctx.CharacterCfg.Game.Difficulty != difficulty.Hell {
		if err := NewQuests().clearDenQuest(); err != nil {
			return err
		}
	}
	// Rescue Cain if not already done
	if !a.isCainInTown() && a.ctx.CharacterCfg.Game.Difficulty != difficulty.Hell {
		if err := NewQuests().rescueCainQuest(); err != nil {
			return err
		}
	}

	action.InteractNPC(npc.Warriv)
	a.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
	utils.Sleep(1000)
	a.HoldKey(win.VK_SPACE, 2000)
	utils.Sleep(1000)
	return nil
}

// stonyField handles clearing Stony Field
func (a Leveling) stonyField() error {
	err := action.WayPoint(area.StonyField)
	if err != nil {
		return err
	}
	return action.ClearCurrentLevel(false, data.MonsterAnyFilter())
}

// isCainInTown checks if Deckard Cain is in town
func (a Leveling) isCainInTown() bool {
	_, found := a.ctx.Data.Monsters.FindOne(npc.DeckardCain5, data.MonsterTypeNone)
	return found
}

// killRavenGetMerc efficiently finds and kills Blood Raven by pathing near the Mausoleum entrance.
func (a Leveling) killRavenGetMerc() error {
	ctx := a.ctx
	ctx.SetLastAction("killRavenGetMerc")

	if err := action.WayPoint(area.ColdPlains); err != nil {
		return fmt.Errorf("failed to move to Cold Plains: %w", err)
	}

	if err := action.MoveToArea(area.BurialGrounds); err != nil {
		return fmt.Errorf("failed to move to Burial Grounds: %w", err)
	}

	originalBackToTownCfg := a.ctx.CharacterCfg.BackToTown
	a.ctx.CharacterCfg.BackToTown.NoMpPotions = false
	a.ctx.CharacterCfg.Health.HealingPotionAt = 55

	defer func() {
		a.ctx.CharacterCfg.BackToTown = originalBackToTownCfg
		a.ctx.Logger.Info("Restored original back-to-town checks after Blood Raven fight.")
	}()

	areaData := a.ctx.Data.Areas[area.BurialGrounds]
	bloodRavenNPC, found := areaData.NPCs.FindOne(805)

	if !found || len(bloodRavenNPC.Positions) == 0 {
		a.ctx.Logger.Info("Blood Raven position not found")
		return nil
	}

	action.MoveToCoords(bloodRavenNPC.Positions[0])

	for {
		bloodRaven, found := a.ctx.Data.Monsters.FindOne(npc.BloodRaven, data.MonsterTypeNone)

		if !found {
			break
		}

		a.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			return bloodRaven.UnitID, true
		}, nil)
	}

	action.ItemPickup(30)

	return nil
}

func gambleAct1Belt(ctx *context.Status) error {

	// Check if level 9. Some wiggle room for over leveling, but then stops for level 11+
	lvl, _ := ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
	if lvl.Value < 9 || lvl.Value >= 11 {
		ctx.Logger.Info("Not level 9 to 11, skipping belt gamble.")
		return nil
	}

	// Check equipped and inventory for a suitable belt first
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationEquipped) {
		if itm.Name == "Belt" || itm.Name == "HeavyBelt" || itm.Name == "PlatedBelt" {
			ctx.Logger.Info("Already have a 9 slot belt equipped, skipping.")
			return nil
		}
	}
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if itm.Name == "Belt" || itm.Name == "HeavyBelt" || itm.Name == "PlatedBelt" {
			ctx.Logger.Info("Already have a 9 slot belt in inventory, skipping.")
			return nil
		}
	}

	// Check for gold before visiting the vendor
	if ctx.Data.PlayerUnit.TotalPlayerGold() < 3000 {
		ctx.Logger.Info("Not enough gold to buy a belt, skipping.")
		return nil
	}

	// Go to Gheed and get the gambling menu
	ctx.Logger.Info("No 12 slot belt found, trying to buy one from Gheed.")
	if err := action.InteractNPC(npc.Gheed); err != nil {
		return err
	}
	defer step.CloseAllMenus()

	ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_DOWN, win.VK_RETURN)
	utils.Sleep(1000)

	// Check if the shop menu is open
	if !ctx.Data.OpenMenus.NPCShop {
		ctx.Logger.Debug("failed opening gambling window")
	}

	// Define the item to gamble for
	itemsToGamble := []string{"Belt"}

	// Loop until the desired item is found and purchased
	for {
		// Check for any of the desired items in the vendor's inventory
		for _, itmName := range itemsToGamble {
			itm, found := ctx.Data.Inventory.Find(item.Name(itmName), item.LocationVendor)
			if found {
				town.BuyItem(itm, 1)
				ctx.Logger.Info("Belt purchased, running AutoEquip.")
				if err := action.AutoEquip(); err != nil {
					ctx.Logger.Error("AutoEquip failed after buying belt", "error", err)
				}
				return nil
			}
		}

		// If no desired item was found, refresh the gambling window
		ctx.Logger.Info("Desired items not found in gambling window, refreshing...")
		if ctx.Data.LegacyGraphics {
			ctx.HID.Click(game.LeftButton, ui.GambleRefreshButtonXClassic, ui.GambleRefreshButtonYClassic)
		} else {
			ctx.HID.Click(game.LeftButton, ui.GambleRefreshButtonX, ui.GambleRefreshButtonY)
		}
		utils.Sleep(500)
	}
}

// atDistance is a helper function to calculate a position a certain distance away from a target.
func atDistance(start, end data.Position, distance int) data.Position {
	dx := float64(end.X - start.X)
	dy := float64(end.Y - start.Y)
	dist := math.Sqrt(dx*dx + dy*dy)

	if dist == 0 {
		return start
	}

	ratio := float64(distance) / dist
	newX := float64(start.X) + dx*ratio
	newY := float64(start.Y) + dy*ratio

	return data.Position{X: int(newX), Y: int(newY)}
}

// shouldFarmCountessForRunes checks if the character should farm Countess for runes in Nightmare difficulty.
func (a Leveling) shouldFarmCountessForRunes() bool {
	requiredRunes := map[string]int{
		"TalRune":   3,
		"ThulRune":  2,
		"OrtRune":   2,
		"AmnRune":   2,
		"TirRune":   1,
		"SolRune":   3,
		"RalRune":   2,
		"NefRune":   2,
		"ShaelRune": 3,
		"IoRune":    1,
		"EldRune":   1,
	}

	ownedRunes := make(map[string]int)
	itemsInStash := a.ctx.Data.Inventory.ByLocation(item.LocationInventory, item.LocationStash, item.LocationSharedStash)

	a.ctx.Logger.Debug("--- Checking for required runes ---")
	for _, itm := range itemsInStash {
		itemName := string(itm.Name)
		if _, isRequired := requiredRunes[itemName]; isRequired {
			a.ctx.Logger.Debug(fmt.Sprintf("Found a required rune: %s. Incrementing count.", itemName))
			ownedRunes[itemName]++
		}
	}
	a.ctx.Logger.Debug(fmt.Sprintf("Final owned rune counts: %v", ownedRunes))

	for runeName, requiredCount := range requiredRunes {
		if ownedRunes[runeName] < requiredCount {
			a.ctx.Logger.Info(fmt.Sprintf("Missing runes, farming Countess. Need %d of %s, but have %d.", requiredCount, runeName, ownedRunes[runeName]))
			return true
		}
	}

	a.ctx.Logger.Info("All required runes are present. Skipping Countess farm.")
	return false
}

func (a Leveling) barbRunewords() bool {
	equippedItems := a.ctx.Data.Inventory.ByLocation(item.LocationEquipped)
	for _, itm := range equippedItems {
		if itm.IsRuneword && (itm.RunewordName == item.RunewordSteel || itm.RunewordName == item.RunewordMalice) {
			return false
		}
	}

	requiredRunes := map[string]int{
		"TirRune": 2,
		"ElRune":  4,
		"IthRune": 2,
		"EthRune": 3,
		"Ralune":  1,
		"OrtRune": 1,
		"TalRune": 1,
	}

	ownedRunes := make(map[string]int)
	itemsInStash := a.ctx.Data.Inventory.ByLocation(item.LocationInventory, item.LocationStash, item.LocationSharedStash)

	for _, itm := range itemsInStash {
		itemName := string(itm.Name)
		if _, isRequired := requiredRunes[itemName]; isRequired {
			ownedRunes[itemName]++
		}
	}

	for runeName, requiredCount := range requiredRunes {
		if ownedRunes[runeName] < requiredCount {
			return true
		}
	}

	return false
}
