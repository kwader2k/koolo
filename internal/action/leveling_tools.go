package action

import (
	"fmt"
	"log/slog"
	"slices"
	"strings"

	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/memory"
)

var uiStatButtonPosition = map[stat.ID]data.Position{
	stat.Strength:  {X: 240, Y: 210},
	stat.Dexterity: {X: 240, Y: 290},
	stat.Vitality:  {X: 240, Y: 380},
	stat.Energy:    {X: 240, Y: 430},
}

var uiSkillPagePosition = [3]data.Position{
	{X: 1100, Y: 140},
	{X: 1010, Y: 140},
	{X: 910, Y: 140},
}

var uiSkillRowPosition = [6]int{190, 250, 310, 365, 430, 490}
var uiSkillColumnPosition = [3]int{920, 1010, 1095}

var uiStatButtonPositionLegacy = map[stat.ID]data.Position{
	stat.Strength:  {X: 430, Y: 180},
	stat.Dexterity: {X: 430, Y: 250},
	stat.Vitality:  {X: 430, Y: 360},
	stat.Energy:    {X: 430, Y: 435},
}

var uiSkillPagePositionLegacy = [3]data.Position{
	{X: 970, Y: 510},
	{X: 970, Y: 390},
	{X: 970, Y: 260},
}

var uiSkillRowPositionLegacy = [6]int{110, 195, 275, 355, 440, 520}
var uiSkillColumnPositionLegacy = [3]int{690, 770, 855}

var uiQuestLogActButtonsD2R = map[int]data.Position{
	1: {X: 137, Y: 125},
	2: {X: 205, Y: 125},
	3: {X: 272, Y: 125},
	4: {X: 340, Y: 125},
	5: {X: 408, Y: 125},
}

var uiQuestLogActButtonsLegacy = map[int]data.Position{
	1: {X: 300, Y: 87},
	2: {X: 373, Y: 87},
	3: {X: 450, Y: 87},
	4: {X: 520, Y: 87},
	5: {X: 598, Y: 87},
}

// New helper function to get equipped item coordinates based on body location and graphics mode
func getEquippedSlotCoords(bodyLoc item.LocationType, legacyGraphics bool) (data.Position, bool) {
	if legacyGraphics {
		switch bodyLoc {
		case item.LocHead:
			return data.Position{X: ui.EquipHeadClassicX, Y: ui.EquipHeadClassicY}, true
		case item.LocNeck:
			return data.Position{X: ui.EquipNeckClassicX, Y: ui.EquipNeckClassicY}, true
		case item.LocTorso:
			return data.Position{X: ui.EquipTorsClassicX, Y: ui.EquipTorsClassicY}, true
		case item.LocRightArm:
			return data.Position{X: ui.EquipRArmClassicX, Y: ui.EquipRArmClassicY}, true
		case item.LocLeftArm:
			return data.Position{X: ui.EquipLArmClassicX, Y: ui.EquipLArmClassicY}, true
		case item.LocRightRing:
			return data.Position{X: ui.EquipRRinClassicX, Y: ui.EquipRRinClassicY}, true
		case item.LocLeftRing:
			return data.Position{X: ui.EquipLRinClassicX, Y: ui.EquipLRinClassicY}, true
		case item.LocBelt:
			return data.Position{X: ui.EquipBeltClassicX, Y: ui.EquipBeltClassicY}, true
		case item.LocFeet:
			return data.Position{X: ui.EquipFeetClassicX, Y: ui.EquipFeetClassicY}, true
		case item.LocGloves:
			return data.Position{X: ui.EquipGlovClassicX, Y: ui.EquipGlovClassicY}, true
		default:
			return data.Position{}, false
		}
	} else {
		switch bodyLoc {
		case item.LocHead:
			return data.Position{X: ui.EquipHeadX, Y: ui.EquipHeadY}, true
		case item.LocNeck:
			return data.Position{X: ui.EquipNeckX, Y: ui.EquipNeckY}, true
		case item.LocTorso:
			return data.Position{X: ui.EquipTorsX, Y: ui.EquipTorsY}, true
		case item.LocRightArm:
			return data.Position{X: ui.EquipRArmX, Y: ui.EquipRArmY}, true
		case item.LocLeftArm:
			return data.Position{X: ui.EquipLArmX, Y: ui.EquipLArmY}, true
		case item.LocRightRing:
			return data.Position{X: ui.EquipRRinX, Y: ui.EquipRRinY}, true
		case item.LocLeftRing:
			return data.Position{X: ui.EquipLRinX, Y: ui.EquipLRinY}, true
		case item.LocBelt:
			return data.Position{X: ui.EquipBeltX, Y: ui.EquipBeltY}, true
		case item.LocFeet:
			return data.Position{X: ui.EquipFeetX, Y: ui.EquipFeetY}, true
		case item.LocGloves:
			return data.Position{X: ui.EquipGlovX, Y: ui.EquipGlovY}, true
		default:
			return data.Position{}, false
		}
	}
}

// dropItemFromInventoryUI is a helper function to drop an item that is already in the inventory
// It assumes the inventory is already open and does NOT close it afterward.
func dropItemFromInventoryUI(i data.Item) error {
	ctx := context.Get()

	// Define a list of item types to exclude from dropping.
	var excludedTypes = []string{
		"jave", "tkni", "taxe", "spea", "pole", "mace",
		"club", "hamm", "swor", "knif", "axe", "wand", "staff", "scep",
		"h2h", "h2h2", "orb", "shie", "ashd", // Shields
	}

	// Check if the item's type is in the list of excluded types.
	if slices.Contains(excludedTypes, string(i.Desc().Type)) {
		ctx.Logger.Debug(fmt.Sprintf("EXCLUDING: Skipping drop for %s (ID: %d) as it is an excluded item type.", i.Name, i.ID))
		return nil
	}

	if i.Name == "TomeOfTownPortal" || i.Name == "TomeOfIdentify" {
		ctx.Logger.Debug(fmt.Sprintf("EXCLUDING: Skipping drop for %s (ID: %d) as per rule.", i.Name, i.ID))
		return nil
	}

	screenPos := ui.GetScreenCoordsForItem(i)
	ctx.HID.MovePointer(screenPos.X, screenPos.Y)
	utils.Sleep(100)
	ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
	utils.Sleep(300)

	return nil
}

func EnsureStatPoints() error {
	ctx := context.Get()
	ctx.SetLastAction("EnsureStatPoints")
	char, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	if !isLevelingChar {
		if !ctx.CharacterCfg.Character.AutoStatSkill.Enabled {
			return nil
		}
	}

	ctx.IsAllocatingStatsOrSkills.Store(true)
	defer ctx.IsAllocatingStatsOrSkills.Store(false)

	statPoints, hasUnusedPoints := ctx.Data.PlayerUnit.FindStat(stat.StatPoints, 0)
	if !hasUnusedPoints || statPoints.Value == 0 {
		return nil
	}

	if !isLevelingChar {
		return ensureConfiguredStatPoints(statPoints.Value)
	}

	// Check if we should use packet mode for any leveling class
	usePacketMode := false
	switch ctx.CharacterCfg.Character.Class {
	case "sorceress_leveling":
		usePacketMode = ctx.CharacterCfg.Character.SorceressLeveling.UsePacketLearning
	case "assassin":
		usePacketMode = ctx.CharacterCfg.Character.AssassinLeveling.UsePacketLearning
	case "amazon_leveling":
		usePacketMode = ctx.CharacterCfg.Character.AmazonLeveling.UsePacketLearning
	case "druid_leveling":
		usePacketMode = ctx.CharacterCfg.Character.DruidLeveling.UsePacketLearning
	case "necromancer":
		usePacketMode = ctx.CharacterCfg.Character.NecromancerLeveling.UsePacketLearning
	case "paladin":
		usePacketMode = ctx.CharacterCfg.Character.PaladinLeveling.UsePacketLearning
	}

	remainingPoints := statPoints.Value
	allocations := char.StatPoints()
	for _, allocation := range allocations {
		if statPoints.Value == 0 {
			break
		}

		currentValue, _ := ctx.Data.PlayerUnit.BaseStats.FindStat(allocation.Stat, 0)
		if currentValue.Value >= allocation.Points {
			continue
		}

		// Calculate how many points we can actually spend
		pointsToSpend := min(allocation.Points-currentValue.Value, remainingPoints)
		failures := 0
		for pointsToSpend > 0 && remainingPoints > 0 {
			var spent int
			if usePacketMode {
				err := AllocateStatPointPacket(allocation.Stat)
				if err != nil {
					ctx.Logger.Error(fmt.Sprintf("Failed to spend point in %v via packet: %v", allocation.Stat, err))
				} else {
					spent = 1
				}
			} else {
				useBulk := pointsToSpend >= 5 && remainingPoints >= 5
				spent = spendStatPoint(allocation.Stat, useBulk)
				if spent == 0 {
					ctx.Logger.Error(fmt.Sprintf("Failed to spend point in %v", allocation.Stat))
				}
			}

			if spent <= 0 {
				failures++
				if failures >= 3 {
					break
				}
				continue
			}

			failures = 0
			if spent > pointsToSpend {
				ctx.Logger.Warn(fmt.Sprintf("Spent more stat points than requested in %v (spent=%d, needed=%d)", allocation.Stat, spent, pointsToSpend))
				spent = pointsToSpend
			}
			remainingPoints -= spent
			pointsToSpend -= spent

			updatedValue, _ := ctx.Data.PlayerUnit.BaseStats.FindStat(allocation.Stat, 0)
			if updatedValue.Value >= allocation.Points {
				ctx.Logger.Debug(fmt.Sprintf("Increased %v to target %d (%d total points remaining)",
					allocation.Stat, allocation.Points, remainingPoints))
			}
		}
	}

	if !usePacketMode {
		return step.CloseAllMenus()
	}
	return nil
}

func ensureConfiguredStatPoints(remainingPoints int) error {
	ctx := context.Get()

	statKeyToID := map[string]stat.ID{
		"strength":  stat.Strength,
		"dexterity": stat.Dexterity,
		"vitality":  stat.Vitality,
		"energy":    stat.Energy,
	}

	usePacketMode := false
	for _, entry := range ctx.CharacterCfg.Character.AutoStatSkill.Stats {
		if remainingPoints <= 0 {
			break
		}
		statKey := strings.ToLower(strings.TrimSpace(entry.Stat))
		statID, ok := statKeyToID[statKey]
		if !ok {
			ctx.Logger.Warn(fmt.Sprintf("Unknown stat key in auto stat config: %s", entry.Stat))
			continue
		}
		if entry.Target <= 0 {
			continue
		}

		currentValue, _ := ctx.Data.PlayerUnit.BaseStats.FindStat(statID, 0)
		if currentValue.Value >= entry.Target {
			continue
		}

		pointsToSpend := min(entry.Target-currentValue.Value, remainingPoints)
		failures := 0
		for pointsToSpend > 0 && remainingPoints > 0 {
			useBulk := pointsToSpend >= 5 && remainingPoints >= 5
			spent := spendStatPoint(statID, useBulk)
			if spent <= 0 {
				failures++
				if failures >= 3 {
					break
				}
				continue
			}
			failures = 0
			if spent > pointsToSpend {
				ctx.Logger.Warn(fmt.Sprintf("Spent more stat points than requested in %v (spent=%d, needed=%d)", statID, spent, pointsToSpend))
				spent = pointsToSpend
			}
			remainingPoints -= spent
			pointsToSpend -= spent
		}
	}

	if !usePacketMode {
		return step.CloseAllMenus()
	}
	return nil
}

func spendStatPoint(statID stat.ID, useBulk bool) int {
	ctx := context.Get()
	beforePoints, _ := ctx.Data.PlayerUnit.FindStat(stat.StatPoints, 0)

	if !ctx.Data.OpenMenus.Character {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.CharacterScreen)
		utils.Sleep(100)
	}

	statBtnPosition := uiStatButtonPosition[statID]
	if ctx.Data.LegacyGraphics {
		statBtnPosition = uiStatButtonPositionLegacy[statID]
	}

	if useBulk {
		ctx.HID.ClickWithModifier(game.LeftButton, statBtnPosition.X, statBtnPosition.Y, game.CtrlKey)
	} else {
		ctx.HID.Click(game.LeftButton, statBtnPosition.X, statBtnPosition.Y)
	}
	utils.Sleep(300)

	afterPoints, _ := ctx.Data.PlayerUnit.FindStat(stat.StatPoints, 0)
	spent := beforePoints.Value - afterPoints.Value
	if spent == 0 && useBulk {
		ctx.RefreshGameData()
		afterPoints, _ = ctx.Data.PlayerUnit.FindStat(stat.StatPoints, 0)
		spent = beforePoints.Value - afterPoints.Value
		if spent == 0 {
			ctx.HID.Click(game.LeftButton, statBtnPosition.X, statBtnPosition.Y)
			utils.Sleep(300)
			ctx.RefreshGameData()
			afterPoints, _ = ctx.Data.PlayerUnit.FindStat(stat.StatPoints, 0)
			spent = beforePoints.Value - afterPoints.Value
		}
	}
	if spent < 0 {
		return 0
	}
	return spent
}

func getAvailableSkillKB() []data.KeyBinding {
	availableSkillKB := make([]data.KeyBinding, 0)
	ctx := context.Get()
	ctx.SetLastAction("getAvailableSkillKB")

	for _, sb := range ctx.Data.KeyBindings.Skills {
		if sb.SkillID == -1 && ((sb.Key1[0] != 0 && sb.Key1[0] != 255) || (sb.Key2[0] != 0 && sb.Key2[0] != 255)) {
			availableSkillKB = append(availableSkillKB, sb.KeyBinding)
		}
	}

	return availableSkillKB
}

func ResetBindings() error {
	ctx := context.Get()
	ctx.SetLastAction("BindTomeOfTownPortalToSkillActions")

	// 1. Check if Tome of Town Portal is available in inventory (inventory-based check for legacy compatibility)
	if _, found := ctx.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory); !found {
		ctx.Logger.Debug("TomeOfTownPortal tome not found in inventory. Skipping skill action binding sequence.")
		return nil
	}

	// Determine the skill position once, as it's always TomeOfTownPortal
	skillPosition, found := calculateSkillPositionInUI(false, skill.TomeOfTownPortal)
	if !found {
		ctx.Logger.Error("TomeOfTownPortal skill UI position not found. Cannot proceed with skill action binding.")
		step.CloseAllMenus()
		return fmt.Errorf("TomeOfTownPortal skill UI position not found")
	}

	// Loop for skill action 1 through 16 based on configured keybindings.
	for i, skillBinding := range ctx.Data.KeyBindings.Skills {
		if (skillBinding.Key1[0] == 0 || skillBinding.Key1[0] == 255) && (skillBinding.Key2[0] == 0 || skillBinding.Key2[0] == 255) {
			ctx.Logger.Debug(fmt.Sprintf("Skipping skill action %d: no keybinding assigned.", i+1))
			continue
		}
		ctx.Logger.Info(fmt.Sprintf("Attempting to bind TomeOfTownPortal to skill action %d", i+1))

		// 2. Open the secondary skill assignment UI
		if ctx.GameReader.LegacyGraphics() {
			ctx.HID.Click(game.LeftButton, ui.SecondarySkillButtonXClassic, ui.SecondarySkillButtonYClassic)
		} else {
			ctx.HID.Click(game.LeftButton, ui.SecondarySkillButtonX, ui.SecondarySkillButtonY)
		}
		utils.Sleep(300) // Give time for UI to open

		// 3. Move mouse to skill position (hover)
		ctx.HID.MovePointer(skillPosition.X, skillPosition.Y)
		utils.Sleep(500) // Delay for mouse to move and for the hover effect

		// 4. Press current skill action keybinding to assign the skill
		ctx.HID.PressKeyBinding(skillBinding.KeyBinding)
		utils.Sleep(700) // Delay for the binding to register

		// 5. Close the skill assignment menu
		step.CloseAllMenus()

		utils.Sleep(500) // Delay after closing for the next iteration
	}

	ctx.Logger.Info("TomeOfTownPortal binding to skill action 1-16 sequence completed.")
	return nil
}

// isMercenaryPresent checks for the existence of an Act 2 mercenary
func isMercenaryPresent(mercName npc.ID) bool {
	ctx := context.Get()
	for _, monster := range ctx.Data.Monsters {
		if monster.IsMerc() && monster.Name == mercName {
			ctx.Logger.Debug(fmt.Sprintf("Mercenary of type %v is already present.", mercName))
			return true
		}
	}
	return false
}

func HireMerc() error {
	ctx := context.Get()
	ctx.SetLastAction("HireMerc")

	_, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	if isLevelingChar && ctx.CharacterCfg.Character.UseMerc {
		// Check if we already have a suitable mercenary
		if isMercenaryPresent(npc.Guard) && ctx.Data.MercHPPercent() > 0 {
			ctx.Logger.Debug("An Act 2 merc is already present and alive, no need to hire a new one.")
			return nil
		}

		// Only hire in Normal difficulty
		if ctx.CharacterCfg.Game.Difficulty == difficulty.Normal {
			if ctx.Data.PlayerUnit.Area != area.LutGholein {
				if err := WayPoint(area.LutGholein); err != nil {
					if strings.Contains(err.Error(), "no available waypoint found to reach destination") {
						ctx.Logger.Debug("Lut Gholein waypoint not unlocked, skipping merc hire.")
						return nil
					}
					return err
				}
			}

			ctx.Logger.Info("Attempting to hire 'Prayer' mercenary...")

			if err := InteractNPC(town.GetTownByArea(ctx.Data.PlayerUnit.Area).MercContractorNPC()); err != nil {
				return err
			}

			ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
			utils.Sleep(2000)

			mercList := ctx.GameReader.GetMercList()

			var mercToHire *memory.MercOption
			for i := range mercList {
				if mercList[i].Skill.ID == skill.Prayer { // Targeting the Prayer skill ID
					mercToHire = &mercList[i]
					break
				}
			}

			if mercToHire != nil {
				currentGold := ctx.Data.PlayerUnit.TotalPlayerGold()
				if currentGold < mercToHire.Cost {
					ctx.Logger.Info(fmt.Sprintf("Not enough gold to hire merc (gold: %d, cost: %d).", currentGold, mercToHire.Cost))
				} else {
					ctx.Logger.Info(fmt.Sprintf("Hiring merc: %s with skill %s", mercToHire.Name, mercToHire.Skill.Name))
					keySequence := []byte{win.VK_HOME}
					for i := 0; i < mercToHire.Index; i++ {
						keySequence = append(keySequence, win.VK_DOWN)
					}
					keySequence = append(keySequence, win.VK_RETURN, win.VK_UP, win.VK_RETURN)
					ctx.HID.KeySequence(keySequence...)
					utils.Sleep(1000)
				}
			} else {
				ctx.Logger.Info("No merc with Prayer found.")
				utils.Sleep(1000)
			}

			step.CloseAllMenus()

			ctx.Logger.Info("Mercenary hiring routine complete.")
			AutoEquip()
		}
	}

	return nil
}

func ResetStats() error {
	ctx := context.Get()
	ctx.SetLastAction("ResetStats")

	ch, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	if isLevelingChar && ch.ShouldResetSkills() {
		currentArea := ctx.Data.PlayerUnit.Area
		if ctx.Data.PlayerUnit.Area != area.RogueEncampment {
			err := WayPoint(area.RogueEncampment)
			if err != nil {
				return err
			}
		}

		ctx.DisableItemPickup()
		ctx.Logger.Info("Stashing all equipped items before skill reset.")

		// 1. Open Stash and Inventory to prepare for item transfer
		if err := OpenStash(); err != nil {
			step.CloseAllMenus()
			return fmt.Errorf("could not open stash: %w", err)
		}
		if !ctx.Data.OpenMenus.Inventory {
			ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
			utils.Sleep(500)
		}
		ctx.GameReader.GetData() // Refresh data to confirm menus are open

		// 2. Loop the stashing process three times for robustness
		for i := 0; i < 3; i++ {
			ctx.Logger.Info(fmt.Sprintf("Stashing equipped items, attempt %d/3...", i+1))

			// Get the list of currently equipped items for this attempt
			equippedItemsToProcess := make([]data.Item, 0)
			for _, eqItem := range ctx.Data.Inventory.ByLocation(item.LocationEquipped) {
				equippedItemsToProcess = append(equippedItemsToProcess, eqItem)
			}

			// If no items are equipped, we are done and can exit the loop early
			if len(equippedItemsToProcess) == 0 {
				ctx.Logger.Info("All equipped items successfully stashed.")
				break
			}

			// Unequip and immediately stash each remaining equipped item
			for _, eqItem := range equippedItemsToProcess {
				ctx.Logger.Debug(fmt.Sprintf("Processing equipped item: %s from %s (ID: %d)", eqItem.Name, eqItem.Location.BodyLocation, eqItem.ID))
				slotCoords, found := getEquippedSlotCoords(eqItem.Location.BodyLocation, ctx.Data.LegacyGraphics)
				if !found {
					ctx.Logger.Warn(fmt.Sprintf("Could not find coordinates for equipped slot %s, skipping unequip for %s", eqItem.Location.BodyLocation, eqItem.Name))
					continue
				}

				// Ctrl-click to unequip the item to stash directly
				ctx.HID.ClickWithModifier(game.LeftButton, slotCoords.X, slotCoords.Y, game.CtrlKey)
				utils.Sleep(500)

				utils.Sleep(250)
				ctx.GameReader.GetData()
			}
			// Small delay before the next full attempt
			utils.Sleep(500)
		}

		step.CloseAllMenus() // Close stash and inventory
		utils.Sleep(500)

		// 3. Interact with Akara for the reset
		InteractNPC(npc.Akara)
		ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_DOWN, win.VK_RETURN)
		utils.Sleep(1000)
		ctx.HID.KeySequence(win.VK_HOME, win.VK_RETURN)
		utils.Sleep(1000)
		ctx.GameReader.GetData() // Refresh data to update skill values

		// 4. Now, drop any remaining items directly in the inventory
		ctx.Logger.Info("Dropping all remaining inventory items.")
		if !ctx.Data.OpenMenus.Inventory {
			ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
			utils.Sleep(500)
			ctx.GameReader.GetData()
		}

		inventoryItems := ctx.Data.Inventory.ByLocation(item.LocationInventory)
		for _, invItem := range inventoryItems {
			if invItem.Name == "Gold" {
				continue
			}

			if IsInLockedInventorySlot(invItem) {
				ctx.Logger.Debug(fmt.Sprintf("Skipping locked item %s in inventory", invItem.Name))
				continue
			}
			ctx.Logger.Debug(fmt.Sprintf("Dropping remaining inventory item: %s", invItem.Name))
			if err := dropItemFromInventoryUI(invItem); err != nil {
				ctx.Logger.Error(fmt.Sprintf("Failed to drop inventory item %s: %v", invItem.Name, err))
			}
			utils.Sleep(300)
			ctx.GameReader.GetData()
		}
		ctx.Logger.Debug("All remaining inventory items processed for drop.")

		step.CloseAllMenus()
		utils.Sleep(500)
		ctx.PathFinder.RandomMovement() // Avoid being considered stuck
		utils.Sleep(500)

		// 5. Finalize the reset process
		err := ResetBindings()
		if err != nil {
			ctx.Logger.Error("Failed to bind TomeOfTownPortal after stats reset", slog.Any("error", err))
		}
		utils.Sleep(500)
		ctx.PathFinder.RandomMovement() // Avoid being considered stuck
		utils.Sleep(500)

		EnsureStatPoints()
		utils.Sleep(500)
		ctx.PathFinder.RandomMovement() // Avoid being considered stuck
		utils.Sleep(500)
		ctx.GameReader.GetData() // Refresh data to update stat values

		EnsureSkillPoints()
		utils.Sleep(500)
		ctx.PathFinder.RandomMovement() // Avoid being considered stuck
		utils.Sleep(500)
		ctx.GameReader.GetData() // Refresh data to update skill values

		EnsureSkillBindings()
		utils.Sleep(500)
		ctx.PathFinder.RandomMovement() // Avoid being considered stuck
		utils.Sleep(500)

		ctx.EnableItemPickup()

		// 6. Pick up dropped items and auto-equip
		utils.Sleep(500)
		ItemPickup(-1)
		utils.Sleep(500)
		ItemPickup(-1)
		utils.Sleep(500)
		AutoEquip()
		utils.Sleep(500)
		ItemPickup(-1)
		utils.Sleep(500)
		AutoEquip()
		utils.Sleep(500)

		if currentArea != area.RogueEncampment {
			return WayPoint(currentArea)
		}
	}

	return nil
}

func WaitForAllMembersWhenLeveling() error {
	ctx := context.Get()
	ctx.SetLastAction("WaitForAllMembersWhenLeveling")

	for {
		_, isLeveling := ctx.Char.(context.LevelingCharacter)
		if ctx.CharacterCfg.Companion.Leader && !ctx.Data.PlayerUnit.Area.IsTown() && isLeveling {
			allMembersAreaCloseToMe := true
			for _, member := range ctx.Data.Roster {
				if member.Name != ctx.Data.PlayerUnit.Name && ctx.PathFinder.DistanceFromMe(member.Position) > 20 {
					allMembersAreaCloseToMe = false
				}
			}

			if allMembersAreaCloseToMe {
				return nil
			}

			ClearAreaAroundPlayer(5, data.MonsterAnyFilter())
		} else {
			return nil
		}
	}
}

func IsLowGold() bool {
	ctx := context.Get()

	var playerLevel int
	if lvl, found := ctx.Data.PlayerUnit.FindStat(stat.Level, 0); found {
		playerLevel = lvl.Value
	} else {
		playerLevel = 1
	}

	return ctx.Data.PlayerUnit.TotalPlayerGold() < playerLevel*1000
}

func IsBelowGoldPickupThreshold() bool {
	ctx := context.Get()

	var playerLevel int
	if lvl, found := ctx.Data.PlayerUnit.FindStat(stat.Level, 0); found {
		playerLevel = lvl.Value
	} else {
		playerLevel = 1
	}

	return ctx.Data.PlayerUnit.TotalPlayerGold() < playerLevel*5000
}

func GetCastersCommonRunewords() []string {
	castersRunewords := []string{"Stealth", "Spirit", "Heart of the Oak"}
	return castersRunewords
}

func TryConsumeStaminaPot() {
	ctx := context.Get()
	if ctx.HealthManager.IsLowStamina() {
		if staminaPotion, found := ctx.Data.Inventory.Find("StaminaPotion", item.LocationInventory); found {
			ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
			screenPos := ui.GetScreenCoordsForItem(staminaPotion)
			ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
			step.CloseAllMenus()
		}
	}
}

func HasAnyQuestStartedOrCompleted(startQuest, endQuest quest.Quest) bool {
	ctx := context.Get()
	for i := int(startQuest); i <= int(endQuest); i++ {
		q := ctx.Data.Quests[quest.Quest(i)]
		if !q.NotStarted() || q.Completed() {
			return true
		}
	}
	return false
}
