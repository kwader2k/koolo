package action

import (
	"fmt"
	"slices"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// HasSkillPointsToUse checks if the character has unused skill points and is leveling.
func HasSkillPointsToUse() bool {
	ctx := context.Get()
	_, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	skillPoints, hasUnusedPoints := ctx.Data.PlayerUnit.FindStat(stat.SkillPoints, 0)
	if !isLevelingChar || !hasUnusedPoints || skillPoints.Value == 0 {
		return false
	}
	return true
}

// EnsureSkillPoints allocates skill points according to the leveling character's build.
func EnsureSkillPoints() error {
	ctx := context.Get()
	ctx.SetLastAction("EnsureSkillPoints")
	char, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	if !isLevelingChar {
		if !ctx.CharacterCfg.Character.AutoStatSkill.Enabled {
			return nil
		}
	}

	ctx.IsAllocatingStatsOrSkills.Store(true)
	defer ctx.IsAllocatingStatsOrSkills.Store(false)

	// New: avoid opening skill UI on a brand-new character; this is where crashes happen.
	clvl, _ := ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
	if clvl.Value <= 1 {
		ctx.Logger.Debug("Level 1 character detected, skipping EnsureSkillBindings for now.")
		return nil
	}
	skillPoints, hasUnusedPoints := ctx.Data.PlayerUnit.FindStat(stat.SkillPoints, 0)
	remainingPoints := skillPoints.Value
	if !hasUnusedPoints || remainingPoints == 0 {
		if ctx.Data.OpenMenus.SkillTree {
			step.CloseAllMenus()
		}
		return nil
	}
	if !isLevelingChar {
		return ensureConfiguredSkillPoints(remainingPoints)
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
	skillsBuild := char.SkillPoints()
	targetLevels := make(map[skill.ID]int)
	for _, sk := range skillsBuild {
		targetLevels[sk]++
		currentSkillLevel := 0
		if skillData, found := ctx.Data.PlayerUnit.Skills[sk]; found {
			currentSkillLevel = int(skillData.Level)
		}
		if currentSkillLevel < targetLevels[sk] {
			spent := 0
			if usePacketMode {
				// Use packet mode
				err := LearnSkillPacket(sk)
				if err != nil {
					ctx.Logger.Error(fmt.Sprintf("Failed to learn skill %v via packet: %v", sk, err))
					break
				}
				spent = 1
			} else {
				// Use traditional UI mode
				spent = spendSkillPoint(sk, false)
				if spent <= 0 {
					break
				}
			}
			if spent <= 0 {
				break
			}
			remainingPoints -= spent
			if remainingPoints <= 0 {
				break
			}
		}
	}
	if !usePacketMode {
		return step.CloseAllMenus()
	}
	return nil
}

func ensureConfiguredSkillPoints(remainingPoints int) error {
	ctx := context.Get()

	learned := make(map[skill.ID]bool, len(ctx.Data.PlayerUnit.Skills))
	for id, skillData := range ctx.Data.PlayerUnit.Skills {
		if skillData.Level > 0 {
			learned[id] = true
		}
	}

	skillKeyToID := make(map[string]skill.ID, len(skill.SkillNames))
	for id, name := range skill.SkillNames {
		skillKeyToID[strings.ToLower(name)] = id
	}
	skillNameToID := make(map[string]skill.ID, len(skill.Skills))
	for id, sk := range skill.Skills {
		if sk.Name == "" {
			continue
		}
		skillNameToID[strings.ToLower(sk.Name)] = id
	}

	usePacketMode := false
	for _, entry := range ctx.CharacterCfg.Character.AutoStatSkill.Skills {
		if remainingPoints <= 0 {
			break
		}
		if entry.Target <= 0 {
			continue
		}
		skillID, ok := skillKeyToID[strings.ToLower(strings.TrimSpace(entry.Skill))]
		if !ok {
			ctx.Logger.Warn(fmt.Sprintf("Unknown skill key in auto skill config: %s", entry.Skill))
			continue
		}

		currentSkillLevel := 0
		if skillData, found := ctx.Data.PlayerUnit.Skills[skillID]; found {
			currentSkillLevel = int(skillData.Level)
		}
		bulkStep := 0
		failures := 0
		for currentSkillLevel < entry.Target && remainingPoints > 0 {
			if ok := ensureSkillPrereqs(skillID, skillNameToID, learned, &remainingPoints); !ok {
				break
			}
			pointsNeeded := entry.Target - currentSkillLevel
			useBulk := false
			if bulkStep > 1 {
				useBulk = pointsNeeded >= bulkStep && remainingPoints >= bulkStep
			} else if pointsNeeded >= 20 && remainingPoints >= 20 {
				useBulk = true
			}
			spent := spendSkillPoint(skillID, useBulk)
			if spent <= 0 {
				failures++
				if failures >= 3 {
					break
				}
				continue
			}
			failures = 0
			if spent > pointsNeeded {
				ctx.Logger.Warn(fmt.Sprintf("Spent more skill points than requested for %s (spent=%d, needed=%d)", entry.Skill, spent, pointsNeeded))
				spent = pointsNeeded
			}
			if useBulk && bulkStep == 0 && spent > 1 {
				bulkStep = spent
			}
			currentSkillLevel += spent
			remainingPoints -= spent
			learned[skillID] = true
		}
	}

	if !usePacketMode {
		return step.CloseAllMenus()
	}
	return nil
}

func ensureSkillPrereqs(skillID skill.ID, skillNameToID map[string]skill.ID, learned map[skill.ID]bool, remainingPoints *int) bool {
	ctx := context.Get()

	visiting := make(map[skill.ID]bool)
	var ensurePrereq func(skill.ID) bool
	ensurePrereq = func(target skill.ID) bool {
		if *remainingPoints <= 0 {
			return false
		}
		if visiting[target] {
			return false
		}
		visiting[target] = true
		defer delete(visiting, target)

		sk := skill.Skills[target]
		reqs := []string{sk.ReqSkill1, sk.ReqSkill2}
		for _, reqName := range reqs {
			if strings.TrimSpace(reqName) == "" {
				continue
			}
			reqID, ok := skillNameToID[strings.ToLower(reqName)]
			if !ok {
				ctx.Logger.Warn(fmt.Sprintf("Prereq skill name not found: %s", reqName))
				return false
			}
			if learned[reqID] {
				continue
			}
			if skillData, found := ctx.Data.PlayerUnit.Skills[reqID]; found && skillData.Level > 0 {
				learned[reqID] = true
				continue
			}
			if !ensurePrereq(reqID) {
				return false
			}
			if *remainingPoints <= 0 {
				return false
			}
			spent := spendSkillPoint(reqID, false)
			if spent <= 0 {
				ctx.Logger.Warn(fmt.Sprintf("Failed to learn prereq skill: %s", reqName))
				return false
			}
			if spent > *remainingPoints {
				spent = *remainingPoints
			}
			*remainingPoints -= spent
			learned[reqID] = true
		}
		return true
	}

	return ensurePrereq(skillID)
}

// spendSkillPoint spends a skill point on the given skill using the in-game UI.
func spendSkillPoint(skillID skill.ID, useBulk bool) int {
	ctx := context.Get()
	beforePoints, _ := ctx.Data.PlayerUnit.FindStat(stat.SkillPoints, 0)
	if !ctx.Data.OpenMenus.SkillTree {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.SkillTree)
		utils.Sleep(100)
	}
	sk, found := skill.Skills[skillID]
	skillDesc := sk.Desc()
	if !found {
		ctx.Logger.Error(fmt.Sprintf("skill not found for character: %v", skillID))
		return 0
	}
	if ctx.Data.LegacyGraphics {
		ctx.HID.Click(game.LeftButton, uiSkillPagePositionLegacy[skillDesc.Page-1].X, uiSkillPagePositionLegacy[skillDesc.Page-1].Y)
	} else {
		ctx.HID.Click(game.LeftButton, uiSkillPagePosition[skillDesc.Page-1].X, uiSkillPagePosition[skillDesc.Page-1].Y)
	}
	utils.Sleep(200)
	if ctx.Data.LegacyGraphics {
		if useBulk {
			ctx.HID.ClickWithModifier(game.LeftButton, uiSkillColumnPositionLegacy[skillDesc.Column-1], uiSkillRowPositionLegacy[skillDesc.Row-1], game.ShiftKey)
		} else {
			ctx.HID.Click(game.LeftButton, uiSkillColumnPositionLegacy[skillDesc.Column-1], uiSkillRowPositionLegacy[skillDesc.Row-1])
		}
	} else {
		if useBulk {
			ctx.HID.ClickWithModifier(game.LeftButton, uiSkillColumnPosition[skillDesc.Column-1], uiSkillRowPosition[skillDesc.Row-1], game.ShiftKey)
		} else {
			ctx.HID.Click(game.LeftButton, uiSkillColumnPosition[skillDesc.Column-1], uiSkillRowPosition[skillDesc.Row-1])
		}
	}
	utils.Sleep(300)
	afterPoints, _ := ctx.Data.PlayerUnit.FindStat(stat.SkillPoints, 0)
	spent := beforePoints.Value - afterPoints.Value
	if spent == 0 && useBulk {
		ctx.RefreshGameData()
		afterPoints, _ = ctx.Data.PlayerUnit.FindStat(stat.SkillPoints, 0)
		spent = beforePoints.Value - afterPoints.Value
		if spent == 0 {
			if ctx.Data.LegacyGraphics {
				ctx.HID.Click(game.LeftButton, uiSkillColumnPositionLegacy[skillDesc.Column-1], uiSkillRowPositionLegacy[skillDesc.Row-1])
			} else {
				ctx.HID.Click(game.LeftButton, uiSkillColumnPosition[skillDesc.Column-1], uiSkillRowPosition[skillDesc.Row-1])
			}
			utils.Sleep(300)
			ctx.RefreshGameData()
			afterPoints, _ = ctx.Data.PlayerUnit.FindStat(stat.SkillPoints, 0)
			spent = beforePoints.Value - afterPoints.Value
		}
	}
	if spent < 0 {
		return 0
	}
	return spent
}

// EnsureSkillBindings ensures that all required skills are bound to hotkeys and the main skill is set.
func EnsureSkillBindings() error {
	ctx := context.Get()
	ctx.SetLastAction("EnsureSkillBindings")
	char, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	// New: avoid opening skill UI on a brand-new character; this is where crashes happen.
	clvl, _ := ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
	if clvl.Value <= 1 {
		ctx.Logger.Debug("Level 1 character detected, skipping EnsureSkillBindings for now.")
		return nil
	}
	var mainSkill skill.ID
	var skillsToBind []skill.ID
	if isLevelingChar {
		mainSkill, skillsToBind = char.SkillsToBind()
	} else {
		skillsToBind = ctx.Char.CheckKeyBindings()
	}
	notBoundSkills := make([]skill.ID, 0, len(skillsToBind))
	for _, sk := range skillsToBind {
		// Only add skills that are not already bound AND are either TomeOfTownPortal or the player has learned them.
		if _, found := ctx.Data.KeyBindings.KeyBindingForSkill(sk); !found && (sk == skill.TomeOfTownPortal || ctx.Data.PlayerUnit.Skills[sk].Level > 0) {
			notBoundSkills = append(notBoundSkills, sk)
		}
	}
	if len(notBoundSkills) > 1 {
		slices.Sort(notBoundSkills)
		notBoundSkills = slices.Compact(notBoundSkills)
	}
	legacyGraphics := ctx.GameReader.LegacyGraphics()
	// state for skill menu operations
	menuOpen := false
	menuIsMain := false
	openSkillMenu := func(bindOnLeft bool) {
		if menuOpen && menuIsMain == bindOnLeft {
			return
		}
		if legacyGraphics {
			if bindOnLeft {
				ctx.HID.Click(game.LeftButton, ui.MainSkillButtonXClassic, ui.MainSkillButtonYClassic)
			} else {
				ctx.HID.Click(game.LeftButton, ui.SecondarySkillButtonXClassic, ui.SecondarySkillButtonYClassic)
			}
		} else {
			if bindOnLeft {
				ctx.HID.Click(game.LeftButton, ui.MainSkillButtonX, ui.MainSkillButtonY)
			} else {
				ctx.HID.Click(game.LeftButton, ui.SecondarySkillButtonX, ui.SecondarySkillButtonY)
			}
		}
		utils.Sleep(300)
		menuOpen = true
		menuIsMain = bindOnLeft
	}
	closeSkillMenu := func() {
		if !menuOpen {
			return
		}
		step.CloseAllMenus()
		utils.Sleep(300)
		menuOpen = false
	}

	preferLeftBindings := mainSkill != skill.AttackSkill
	resolveBindOnLeft := func(skillID skill.ID) (bool, bool) {
		if skillID == skill.TomeOfTownPortal {
			return false, true
		}
		skillDesc, found := skill.Skills[skillID]
		if !found {
			ctx.Logger.Error(fmt.Sprintf("Skill metadata not found for binding: %v", skillID))
			return false, false
		}
		switch {
		case skillDesc.LeftSkill && skillDesc.RightSkill:
			return preferLeftBindings, true
		case skillDesc.LeftSkill:
			return true, true
		case skillDesc.RightSkill:
			return false, true
		default:
			ctx.Logger.Warn(fmt.Sprintf("Skill cannot be bound to left or right: %v", skill.SkillNames[skillID]))
			return false, false
		}
	}
	// Bind F-key skills
	if len(notBoundSkills) > 0 {
		ctx.Logger.Debug("Unbound skills found, trying to bind")
		availableKB := getAvailableSkillKB()
		ctx.Logger.Debug(fmt.Sprintf("Available KB: %v", availableKB))
		for i, sk := range notBoundSkills {
			if i >= len(availableKB) {
				ctx.Logger.Warn(fmt.Sprintf("Not enough available keybindings for skill %v", skill.SkillNames[sk]))
				break
			}
			bindOnLeft, ok := resolveBindOnLeft(sk)
			if !ok {
				continue
			}
			openSkillMenu(bindOnLeft)
			skillPosition, found := calculateSkillPositionInUI(bindOnLeft, sk)
			if !found {
				ctx.Logger.Error(fmt.Sprintf("Skill %v UI position not found for binding.", skill.SkillNames[sk]))
				continue
			}
			if sk == skill.TomeOfTownPortal {
				gfx := "D2R"
				if legacyGraphics {
					gfx = "Legacy"
				}
				ctx.Logger.Info(fmt.Sprintf("TomeOfTownPortal will be bound now at (%d,%d) [%s]", skillPosition.X, skillPosition.Y, gfx))
				ctx.Logger.Info(fmt.Sprintf("EnsureSkillBindings Tome coords (secondary): X=%d Y=%d [Legacy=%v]", skillPosition.X, skillPosition.Y, legacyGraphics))
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
		// Close the skill assignment menu if it was opened for binding F-keys
		closeSkillMenu()
	} else if isLevelingChar {
		if _, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.FireBolt); !found {
			if _, known := ctx.Data.PlayerUnit.Skills[skill.FireBolt]; !known {
				ctx.Logger.Debug("Fire Bolt not learned; skipping Fire Bolt binding.")
			} else {
				ctx.Logger.Debug("Fire Bolt not bound; attempting to bind.")
				availableKB := getAvailableSkillKB()
				if len(availableKB) == 0 {
					ctx.Logger.Warn("No available keybindings to bind Fire Bolt.")
				} else {
					bindOnLeft, ok := resolveBindOnLeft(skill.FireBolt)
					if ok {
						openSkillMenu(bindOnLeft)
						skillPosition, found := calculateSkillPositionInUI(bindOnLeft, skill.FireBolt)
						if !found {
							ctx.Logger.Error("Fire Bolt UI position not found for binding.")
						} else {
							ctx.HID.MovePointer(skillPosition.X, skillPosition.Y)
							utils.Sleep(100)
							ctx.HID.PressKeyBinding(availableKB[0])
							utils.Sleep(300)
						}
						closeSkillMenu()
					}
				}
			}
		}
	}
	if isLevelingChar {
		mainSkillExists := false
		if mainSkill == skill.TomeOfTownPortal {
			mainSkillExists = true
		} else if skillData, found := ctx.Data.PlayerUnit.Skills[mainSkill]; found && skillData.Level > 0 {
			mainSkillExists = true
		}
		if !mainSkillExists {
			ctx.Logger.Debug(fmt.Sprintf("Main skill %v not yet available (level %d), skipping left-hand binding", skill.SkillNames[mainSkill], clvl.Value))
			return nil
		}
		openSkillMenu(true)
		skillPosition, found := calculateSkillPositionInUI(true, mainSkill)
		if found {
			ctx.HID.Click(game.LeftButton, skillPosition.X, skillPosition.Y)
			utils.Sleep(300)
		} else {
			ctx.Logger.Error(fmt.Sprintf("Failed to find UI position for main skill %v (ID: %d)", skill.SkillNames[mainSkill], mainSkill))
		}
		closeSkillMenu()
	}
	return nil
}

// calculateSkillPositionInUI computes the UI position for the given skill in the skill menu.
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
	nonClassSkills := make(map[int][]skill.ID)
	row := 0
	column := 0
	for skID := range ctx.Data.PlayerUnit.Skills {
		sk := skill.Skills[skID]
		// Skip skills that cannot be bound
		if sk.Desc().ListRow < 0 {
			continue
		}
		// Skip skills that cannot be bound to the current mouse button
		if (mainSkill && !sk.LeftSkill) || (!mainSkill && !sk.RightSkill) {
			continue
		}
		// Skip skills with charges
		if ctx.Data.PlayerUnit.Skills[skID].Charges > 0 || ctx.Data.PlayerUnit.Skills[skID].Quantity > 0 {
			if !ctx.GameReader.LegacyGraphics() {
				nonClassSkills[sk.Desc().Page] = append(nonClassSkills[sk.Desc().Page], skID)

				if sk.ID == skill.TomeOfTownPortal {
					totalRows = append(totalRows, sk.Desc().ListRow)
				}
			}

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
		if ctx.GameReader.LegacyGraphics() {
			pageSkills[targetSkill.Desc().Page] = append(pageSkills[targetSkill.Desc().Page], skillID)
		}
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

	isChargeOrQuantitySkill := ctx.Data.PlayerUnit.Skills[skillID].Charges > 0 || ctx.Data.PlayerUnit.Skills[skillID].Quantity > 0
	skillsInPage := pageSkills[targetSkill.Desc().Page]
	nonClassSkillsInRow := make([]skill.ID, 0)
	skillsInRow := make([]skill.ID, 0)

	if !isChargeOrQuantitySkill || ctx.GameReader.LegacyGraphics() {
		slices.Sort(skillsInPage)
		for _, skID := range skillsInPage {
			if skill.Skills[skID].Desc().ListRow == targetSkill.Desc().ListRow {
				skillsInRow = append(skillsInRow, skID)
			}
		}
		slices.Sort(skillsInRow)
		for i, skills := range skillsInRow {
			if skills == targetSkill.ID {
				column = i
				break
			}
		}
	} else {
		for _, skills := range nonClassSkills {
			nonClassSkillsInRow = append(nonClassSkillsInRow, skills...)
		}
		slices.Sort(nonClassSkillsInRow)

		for i, skills := range nonClassSkillsInRow {
			if skills == targetSkill.ID {
				column = i
				break
			}
		}
	}

	// Special handling for Legacy + secondary list + TomeOfTownPortal:
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
	}
	skillOffsetX := ui.MainSkillListFirstSkillX - (ui.SkillListSkillOffset * (len(skillsInRow) - (column + 1)))
	if !mainSkill {
		if !isChargeOrQuantitySkill {
			skillOffsetX = ui.SecondarySkillListFirstSkillX + (ui.SkillListSkillOffset * (len(skillsInRow) - (column + 1)))
		} else {
			skillOffsetX = ui.SecondarySkillListFirstSkillX + (ui.SkillListSkillOffset * (len(nonClassSkillsInRow) - (column + 1)))
		}
	}

	return data.Position{
		X: skillOffsetX,
		Y: ui.SkillListFirstSkillY - ui.SkillListSkillOffset*row,
	}, true
}

// GetSkillTotalLevel returns the total level of a skill, including bonuses.
func GetSkillTotalLevel(skillID skill.ID) uint {
	ctx := context.Get()
	skillLevel := ctx.Data.PlayerUnit.Skills[skillID].Level
	if singleSkill, skillFound := ctx.Data.PlayerUnit.Stats.FindStat(stat.SingleSkill, int(skillID)); skillFound {
		skillLevel += uint(singleSkill.Value)
	}
	if skillLevel > 0 {
		if allSkill, allFound := ctx.Data.PlayerUnit.Stats.FindStat(stat.AllSkills, 0); allFound {
			skillLevel += uint(allSkill.Value)
		}
		// Assume it's a player class skill for now
		if classSkills, classFound := ctx.Data.PlayerUnit.Stats.FindStat(stat.AddClassSkills, int(ctx.Data.PlayerUnit.Class)); classFound {
			skillLevel += uint(classSkills.Value)
		}
	}
	return skillLevel
}
