package action

import (
	"fmt"
	"log/slog"
	"slices"

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
	char, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	if !isLevelingChar {
		return nil
	}

	statPoints, hasUnusedPoints := ctx.Data.PlayerUnit.FindStat(stat.StatPoints, 0)
	if !hasUnusedPoints || statPoints.Value == 0 {
		return nil
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
		for i := 0; i < pointsToSpend; i++ {

			if !spendStatPoint(allocation.Stat) {
				ctx.Logger.Error(fmt.Sprintf("Failed to spend point in %v", allocation.Stat))
				continue
			}

			remainingPoints--

			updatedValue, _ := ctx.Data.PlayerUnit.BaseStats.FindStat(allocation.Stat, 0)
			if updatedValue.Value >= allocation.Points {
				ctx.Logger.Debug(fmt.Sprintf("Increased %v to target %d (%d total points remaining)",
					allocation.Stat, allocation.Points, remainingPoints))
			}
		}
	}
	return step.CloseAllMenus()

}

func EnsureSkillPoints() error {
	ctx := context.Get()

	char, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	skillPoints, hasUnusedPoints := ctx.Data.PlayerUnit.FindStat(stat.SkillPoints, 0)

	if !isLevelingChar || !hasUnusedPoints || skillPoints.Value == 0 {
		if ctx.Data.OpenMenus.SkillTree {
			step.CloseAllMenus()
		}
		return nil
	}

	remainingPoints := skillPoints.Value
	skillsToAssign := char.SkillPoints()

	for _, skillID := range skillsToAssign {
		if remainingPoints <= 0 {
			break
		}

		characterPoints, found := ctx.Data.PlayerUnit.Skills[skillID]
		currentLevel := 0
		if found {
			currentLevel = int(characterPoints.Level)
		}

		// Try to spend a point in this skill
		if spendSkillPoint(skillID) {
			remainingPoints--
			ctx.Logger.Debug(fmt.Sprintf("Increased skill %v to level %d (%d total points remaining)",
				skill.SkillNames[skillID], currentLevel+1, remainingPoints))
		} else {
			ctx.Logger.Error(fmt.Sprintf("Failed to spend point in skill %v", skill.SkillNames[skillID]))
		}
	}

	return step.CloseAllMenus()
}

func spendStatPoint(statID stat.ID) bool {
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

	ctx.HID.Click(game.LeftButton, statBtnPosition.X, statBtnPosition.Y)
	utils.Sleep(300)

	afterPoints, _ := ctx.Data.PlayerUnit.FindStat(stat.StatPoints, 0)
	return beforePoints.Value-afterPoints.Value == 1
}

func spendSkillPoint(skillID skill.ID) bool {
	ctx := context.Get()
	beforePoints, _ := ctx.Data.PlayerUnit.FindStat(stat.SkillPoints, 0)

	if !ctx.Data.OpenMenus.SkillTree {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.SkillTree)
		utils.Sleep(100)
	}

	skill, found := skill.Skills[skillID]
	skillDesc := skill.Desc()

	if !found {
		ctx.Logger.Error(fmt.Sprintf("skill not found for character: %v", skillID))
		return false
	}

	if ctx.Data.LegacyGraphics {
		ctx.HID.Click(game.LeftButton, uiSkillPagePositionLegacy[skillDesc.Page-1].X, uiSkillPagePositionLegacy[skillDesc.Page-1].Y)
	} else {
		ctx.HID.Click(game.LeftButton, uiSkillPagePosition[skillDesc.Page-1].X, uiSkillPagePosition[skillDesc.Page-1].Y)
	}
	utils.Sleep(200)

	if ctx.Data.LegacyGraphics {
		ctx.HID.Click(game.LeftButton, uiSkillColumnPositionLegacy[skillDesc.Column-1], uiSkillRowPositionLegacy[skillDesc.Row-1])
	} else {
		ctx.HID.Click(game.LeftButton, uiSkillColumnPosition[skillDesc.Column-1], uiSkillRowPosition[skillDesc.Row-1])
	}
	utils.Sleep(300)

	afterPoints, _ := ctx.Data.PlayerUnit.FindStat(stat.SkillPoints, 0)
	return beforePoints.Value-afterPoints.Value == 1
}

func getAvailableSkillKB() []data.KeyBinding {
	availableSkillKB := make([]data.KeyBinding, 0)
	ctx := context.Get()
	ctx.SetLastAction("getAvailableSkillKB")

	for _, sb := range ctx.Data.KeyBindings.Skills {
		if sb.SkillID == -1 && (sb.Key1[0] != 0 && sb.Key1[0] != 255) || (sb.Key2[0] != 0 && sb.Key2[0] != 255) {
			availableSkillKB = append(availableSkillKB, sb.KeyBinding)
		}
	}

	return availableSkillKB
}

func EnsureSkillBindings() error {
	ctx := context.Get()
	ctx.SetLastAction("EnsureSkillBindings")

	char, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	if !isLevelingChar {
		return nil
	}

	level, _ := ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
	mainSkill, skillsToBind := char.SkillsToBind()

	if level.Value < 15 {
		ctx.Logger.Debug("Player under level 15, forcing 'Attack' as main (left-click) skill.")
		mainSkill = skill.AttackSkill
	}

	notBoundSkills := make([]skill.ID, 0)
	for _, sk := range skillsToBind {
		// Only add skills that are not already bound AND are either TomeOfTownPortal or the player has learned them.
		// The check for skill.TomeOfTownPortal ensures it's considered even if not "learned" via skill points.
		if _, found := ctx.Data.KeyBindings.KeyBindingForSkill(sk); !found && (sk == skill.TomeOfTownPortal || ctx.Data.PlayerUnit.Skills[sk].Level > 0) {
			notBoundSkills = append(notBoundSkills, sk)
			slices.Sort(notBoundSkills)
			notBoundSkills = slices.Compact(notBoundSkills) // In case we have duplicates (tp tome)
		}
	}

	clvl, _ := ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
	// Hacky way to find if we're lvling a sorc at clvl 1
	str, _ := ctx.Data.PlayerUnit.FindStat(stat.Strength, 0)

	// This block handles binding skills to F-keys if they are not already bound.
	if len(notBoundSkills) > 0 || (clvl.Value == 1 && str.Value == 10) {
		ctx.Logger.Debug("Unbound skills found, trying to bind")
		if ctx.GameReader.LegacyGraphics() {
			ctx.HID.Click(game.LeftButton, ui.SecondarySkillButtonXClassic, ui.SecondarySkillButtonYClassic)
		} else {
			ctx.HID.Click(game.LeftButton, ui.SecondarySkillButtonX, ui.SecondarySkillButtonY)
		}
		utils.Sleep(300) // Give time for the secondary skill menu to open

		availableKB := getAvailableSkillKB()
		ctx.Logger.Debug(fmt.Sprintf("Available KB: %v", availableKB))
		if len(notBoundSkills) > 0 {
			for i, sk := range notBoundSkills {
				if i >= len(availableKB) { // Prevent out-of-bounds if more skills than available keybindings
					ctx.Logger.Warn(fmt.Sprintf("Not enough available keybindings for skill %v", skill.SkillNames[sk]))
					break
				}
				skillPosition, found := calculateSkillPositionInUI(false, sk)
				if !found {
					ctx.Logger.Error(fmt.Sprintf("Skill %v UI position not found for binding.", skill.SkillNames[sk]))
					continue
				}

				if sk == skill.TomeOfTownPortal {
					gfx := "D2R"
					if ctx.GameReader.LegacyGraphics() {
						gfx = "Legacy"
					}
					ctx.Logger.Info(fmt.Sprintf("TomeOfTownPortal will be bound now at (%d,%d) [%s]", skillPosition.X, skillPosition.Y, gfx))
					ctx.Logger.Info(fmt.Sprintf("EnsureSkillBindings Tome coords (secondary): X=%d Y=%d [Legacy=%v]", skillPosition.X, skillPosition.Y, ctx.GameReader.LegacyGraphics()))
				}

				ctx.HID.MovePointer(skillPosition.X, skillPosition.Y)
				utils.Sleep(100)
				ctx.HID.PressKeyBinding(availableKB[i])
				utils.Sleep(300)
				if sk == skill.TomeOfTownPortal {
					ctx.GameReader.GetData()
					utils.Sleep(150)
					if _, ok := ctx.Data.KeyBindings.KeyBindingForSkill(skill.TomeOfTownPortal); ok {
						ctx.Logger.Info("TomeOfTownPortal binding verified")
					} else {
						ctx.Logger.Warn("TomeOfTownPortal binding verification failed after click")
					}
				}
			}
		} else {
			if _, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.FireBolt); !found {
				ctx.Logger.Debug("Lvl 1 sorc found - forcing fire bolt bind")
				if ctx.GameReader.LegacyGraphics() {
					ctx.HID.MovePointer(1000, 530) // Position for Fire Bolt in Legacy
				} else {
					ctx.HID.MovePointer(685, 545) // Position for Fire Bolt in Resurrected
				}
				utils.Sleep(100)
				// Assuming availableKB[0] is the first available F-key for Fire Bolt
				if len(availableKB) > 0 {
					ctx.HID.PressKeyBinding(availableKB[0])
					utils.Sleep(300)
				} else {
					ctx.Logger.Warn("No available keybindings to bind Fire Bolt for level 1 sorceress.")
				}
			}
		}
		// Close the skill assignment menu if it was opened for binding F-keys
		step.CloseAllMenus()
		utils.Sleep(300)
	}

	// Set left (main) skill
	if ctx.GameReader.LegacyGraphics() {
		ctx.HID.Click(game.LeftButton, ui.MainSkillButtonXClassic, ui.MainSkillButtonYClassic)
	} else {
		ctx.HID.Click(game.LeftButton, ui.MainSkillButtonX, ui.MainSkillButtonY)
	}
	utils.Sleep(300) // Give time for the main skill assignment UI to open

	skillPosition, found := calculateSkillPositionInUI(true, mainSkill)
	if found {
		ctx.HID.Click(game.LeftButton, skillPosition.X, skillPosition.Y)
		utils.Sleep(300)
	} else {
		ctx.Logger.Error(fmt.Sprintf("Failed to find UI position for main skill %v (ID: %d)", skill.SkillNames[mainSkill], mainSkill))
	}

	return step.CloseAllMenus()
}

func ResetBindings() error {
	ctx := context.Get()
	ctx.SetLastAction("BindTomeOfTownPortalToFKeys") // Updated action name

	// 1. Check if Tome of Town Portal is available in inventory (inventory-based check for legacy compatibility)
	if _, found := ctx.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory); !found {
		ctx.Logger.Debug("TomeOfTownPortal tome not found in inventory. Skipping F-key binding sequence.")
		return nil
	}

	// Determine the skill position once, as it's always TomeOfTownPortal
	skillPosition, found := calculateSkillPositionInUI(false, skill.TomeOfTownPortal)
	if !found {
		ctx.Logger.Error("TomeOfTownPortal skill UI position not found. Cannot proceed with F-key binding.")
		step.CloseAllMenus()
		return fmt.Errorf("TomeOfTownPortal skill UI position not found")
	}

	// Loop for F1 through F8
	for i := 0; i < 8; i++ {
		fKey := byte(win.VK_F1 + i)                            // win.VK_F1 is 0x70, win.VK_F2 is 0x71, and so on.
		fKeyBinding := data.KeyBinding{Key1: [2]byte{fKey, 0}} // Assuming 0 for no modifier key
		ctx.Logger.Info(fmt.Sprintf("Attempting to bind TomeOfTownPortal to F%d", i+1))

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

		// 4. Press current F-key to assign the skill
		ctx.HID.PressKeyBinding(fKeyBinding)
		utils.Sleep(700) // Delay for the binding to register

		// 5. Close the skill assignment menu
		step.CloseAllMenus()

		utils.Sleep(500) // Delay after closing for the next iteration
	}

	ctx.Logger.Info("TomeOfTownPortal binding to F1-F8 sequence completed.")
	return nil
}

func calculateSkillPositionInUI(mainSkill bool, skillID skill.ID) (data.Position, bool) {
	ctx := context.Get()

	foundInSkills := true
	if _, found := ctx.Data.PlayerUnit.Skills[skillID]; !found {
		if skillID == skill.TomeOfTownPortal {
			foundInSkills = false
		} else {
			return data.Position{}, false
		}
	}

	targetSkill := skill.Skills[skillID]
	descs := make(map[skill.ID]skill.Skill)
	totalRows := make([]int, 0)
	pageSkills := make(map[int][]skill.ID)
	row := 0
	column := 0

	for skID := range ctx.Data.PlayerUnit.Skills {
		sk := skill.Skills[skID]
		// Skip skills that can not be bind
		if sk.Desc().ListRow < 0 {
			continue
		}

		// Skip skills that can not be bind to current mouse button
		if (mainSkill && !sk.LeftSkill) || (!mainSkill && !sk.RightSkill) {
			continue
		}

		descs[skID] = sk
		if sk.Desc().Page == targetSkill.Desc().Page {
			pageSkills[sk.Desc().Page] = append(pageSkills[sk.Desc().Page], skID)
		}
		totalRows = append(totalRows, sk.Desc().ListRow)

	}

	if !foundInSkills {
		totalRows = append(totalRows, targetSkill.Desc().ListRow)
		pageSkills[targetSkill.Desc().Page] = append(pageSkills[targetSkill.Desc().Page], skillID)
	}

	if ctx.GameReader.LegacyGraphics() && !mainSkill && skillID == skill.TomeOfTownPortal {
		if _, hasIdentify := ctx.Data.Inventory.Find(item.TomeOfIdentify, item.LocationInventory); hasIdentify {
			if _, identifyInSkills := ctx.Data.PlayerUnit.Skills[skill.TomeOfIdentify]; !identifyInSkills {
				identifyDesc := skill.Skills[skill.TomeOfIdentify].Desc()
				totalRows = append(totalRows, identifyDesc.ListRow)
				pageSkills[targetSkill.Desc().Page] = append(pageSkills[targetSkill.Desc().Page], skill.TomeOfIdentify)
			}
		}
	}

	slices.Sort(totalRows)
	totalRows = slices.Compact(totalRows)

	for i, currentRow := range totalRows {
		if currentRow == targetSkill.Desc().ListRow {
			row = i
			break
		}
	}

	skillsInPage := pageSkills[targetSkill.Desc().Page]
	slices.Sort(skillsInPage)
	for i, skills := range skillsInPage {
		if skills == targetSkill.ID {
			column = i
			break
		}
	}

	// Special handling for Legacy + secondary list + TomeOfTownPortal:
	// Column is determined by presence of TomeOfIdentify (left shift by one slot when present)
	if ctx.GameReader.LegacyGraphics() && !mainSkill && skillID == skill.TomeOfTownPortal {
		if _, hasIdentify := ctx.Data.Inventory.Find(item.TomeOfIdentify, item.LocationInventory); hasIdentify {
			column = 1
		} else {
			column = 0
		}
	}

	if ctx.GameReader.LegacyGraphics() {
		skillOffsetX := ui.MainSkillListFirstSkillXClassic + (ui.SkillListSkillOffsetClassic * column)
		if !mainSkill {
			if skillID == skill.TomeOfTownPortal {
				if column == 0 {
					return data.Position{X: 1000, Y: ui.SkillListFirstSkillYClassic - ui.SkillListSkillOffsetClassic*row}, true
				}
				if column == 1 {
					return data.Position{X: 940, Y: ui.SkillListFirstSkillYClassic - ui.SkillListSkillOffsetClassic*row}, true
				}
			}
			skillOffsetX = ui.SecondarySkillListFirstSkillXClassic - (ui.SkillListSkillOffsetClassic * column)
		}

		return data.Position{
			X: skillOffsetX,
			Y: ui.SkillListFirstSkillYClassic - ui.SkillListSkillOffsetClassic*row,
		}, true
	} else {
		skillOffsetX := ui.MainSkillListFirstSkillX - (ui.SkillListSkillOffset * (len(skillsInPage) - (column + 1)))
		if !mainSkill {
			skillOffsetX = ui.SecondarySkillListFirstSkillX + (ui.SkillListSkillOffset * (len(skillsInPage) - (column + 1))) // Order is reversed in resurrected gfx
		}

		return data.Position{
			X: skillOffsetX,
			Y: ui.SkillListFirstSkillY - ui.SkillListSkillOffset*row,
		}, true
	}
}

func UpdateQuestLog() error {
	ctx := context.Get()
	ctx.SetLastAction("UpdateQuestLog")

	if _, isLevelingChar := ctx.Char.(context.LevelingCharacter); !isLevelingChar {
		return nil
	}

	ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.QuestLog)
	utils.Sleep(1000)

	currentAct := ctx.Data.PlayerUnit.Area.Act()

	var actButtonPositions map[int]data.Position
	if ctx.Data.LegacyGraphics {
		actButtonPositions = uiQuestLogActButtonsLegacy
	} else {
		actButtonPositions = uiQuestLogActButtonsD2R
	}

	if pos, found := actButtonPositions[currentAct]; found {
		ctx.Logger.Debug(fmt.Sprintf("Clicking Quest Log Act %d button at (%d, %d)", currentAct, pos.X, pos.Y))

		ctx.HID.Click(game.LeftButton, pos.X, pos.Y)
		utils.Sleep(300)
	} else {
		ctx.Logger.Warn(fmt.Sprintf("Could not find Quest Log button coordinates for current Act: %d", currentAct))
	}

	return step.CloseAllMenus()
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
		if ctx.CharacterCfg.Game.Difficulty == difficulty.Normal && ctx.Data.PlayerUnit.TotalPlayerGold() > 5000 && ctx.Data.PlayerUnit.Area == area.LutGholein {
			ctx.Logger.Info("Attempting to hire 'Prayer' mercenary...")

			isLegacy := ctx.Data.LegacyGraphics
			if !isLegacy {
				ctx.Logger.Info("Switching to legacy mode to hire merc")
				ctx.HID.PressKey(ctx.Data.KeyBindings.LegacyToggle.Key1[0])
				utils.Sleep(500)
			}

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
				ctx.Logger.Info(fmt.Sprintf("Hiring merc: %s with skill %s", mercToHire.Name, mercToHire.Skill.Name))
				keySequence := []byte{win.VK_HOME}
				for i := 0; i < mercToHire.Index; i++ {
					keySequence = append(keySequence, win.VK_DOWN)
				}
				keySequence = append(keySequence, win.VK_RETURN, win.VK_UP, win.VK_RETURN)
				ctx.HID.KeySequence(keySequence...)
				utils.Sleep(1000)
			} else {
				ctx.Logger.Info("No merc with Prayer found.")
				utils.Sleep(1000)
			}

			step.CloseAllMenus()

			if !isLegacy && !ctx.CharacterCfg.ClassicMode {
				ctx.Logger.Info("Switching back to non-legacy mode")
				ctx.HID.PressKey(ctx.Data.KeyBindings.LegacyToggle.Key1[0])
				utils.Sleep(500)
			}

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

				// Shift-click to unequip the item to inventory
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

		// 5. Finalize the reset process
		err := ResetBindings()
		if err != nil {
			ctx.Logger.Error("Failed to bind TomeOfTownPortal to F8 after stats reset", slog.Any("error", err))
		}
		utils.Sleep(500)

		EnsureStatPoints()
		utils.Sleep(500)
		EnsureSkillPoints()
		utils.Sleep(500)

		EnsureSkillBindings()
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
