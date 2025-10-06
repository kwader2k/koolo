package character

import (
	"log/slog"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data/mode"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type DruidLeveling struct {
	BaseCharacter           // Inherits common character functionality
	lastCastTime  time.Time // Tracks the last time a skill was cast
}

// Verify that required skills are bound to keys
func (s DruidLeveling) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{}
	missingKeybindings := make([]skill.ID, 0)

	for _, cskill := range requireKeybindings {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(cskill); !found {
			missingKeybindings = append(missingKeybindings, cskill)
		}
	}

	if len(missingKeybindings) > 0 {
		s.Logger.Debug("There are missing required key bindings.", slog.Any("Bindings", missingKeybindings))
	}

	return missingKeybindings // Returns list of unbound required skills
}

// Ensure casting animation finishes before proceeding
func (s DruidLeveling) waitForCastComplete() bool {
	ctx := context.Get()
	startTime := time.Now()

	for time.Since(startTime) < castingTimeout {
		ctx.RefreshGameData()

		if ctx.Data.PlayerUnit.Mode != mode.CastingSkill && // Check if not casting
			time.Since(s.lastCastTime) > 150*time.Millisecond { // Ensure enough time has passed
			return true
		}

		time.Sleep(16 * time.Millisecond) // Small delay to avoid busy-waiting
	}

	return false // Returns false if timeout is reached
}

// Handle the main combat loop for attacking monsters
func (s DruidLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool), // Function to select target monster
	skipOnImmunities []stat.Resist, // Resistances to skip if monster is immune
) error {
	ctx := context.Get()
	lastRefresh := time.Now()
	completedAttackLoops := 0
	var currentTargetID data.UnitID

	defer func() { // Ensures Tornado is set as active skill on exit
		if kb, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.Tornado); found {
			ctx.HID.PressKeyBinding(kb)
		}
	}()

	for {
		if time.Since(lastRefresh) > time.Millisecond*100 {
			ctx.RefreshGameData()
			lastRefresh = time.Now()
		}

		ctx.PauseIfNotPriority() // Pause if not the priority task

		if completedAttackLoops >= druMaxAttacksLoop {
			return nil // Exit if max attack loops reached
		}

		if currentTargetID == 0 { // Select a new target if none exists
			id, found := monsterSelector(*s.Data)
			if !found {
				return nil // Exit if no target found
			}
			currentTargetID = id
		}

		monster, found := s.Data.Monsters.FindByID(currentTargetID)
		if !found || monster.Stats[stat.Life] <= 0 { // Check if target is dead or missing
			currentTargetID = 0
			return nil
		}

		if !s.preBattleChecks(currentTargetID, skipOnImmunities) { // Perform pre-combat checks
			return nil
		}

		s.RecastBuffs() // Refresh buffs before attacking

		lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
		mana, _ := s.Data.PlayerUnit.FindStat(stat.Mana, 0)

		mainAttackSkill := skill.Firestorm
		secondaryAttackSkill := skill.Firestorm
		if lvl.Value >= 12 {
			mainAttackSkill = skill.Fissure
		}

		if mainAttackSkill != secondaryAttackSkill && s.Data.PlayerUnit.States.HasState(state.Cooldown) {
			if s.Data.PlayerUnit.Skills[skill.Firestorm].Level > 0 {
				if s.Data.PlayerUnit.Mode != mode.CastingSkill {
					step.SecondaryAttack(secondaryAttackSkill, currentTargetID, 1, step.Distance(levelingminDistance, levelingmaxDistance))
				} else {
					time.Sleep(time.Millisecond * 50)
				}
			}
		} else {
			if s.Data.PlayerUnit.Skills[mainAttackSkill].Level > 0 && mana.Value > 2 {
				step.SecondaryAttack(mainAttackSkill, currentTargetID, 1, step.Distance(levelingminDistance, levelingmaxDistance))
			} else {
				// Fallback to primary skill (basic attack) at close range when out of mana.
				step.PrimaryAttack(currentTargetID, 1, true, step.Distance(1, 3))
			}
		}
		/*
			if kb, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.Tornado); found {
				ctx.HID.PressKeyBinding(kb) // Set Tornado as active skill
				if err := step.PrimaryAttack(currentTargetID, 1, true, attackOpts...); err == nil {
					if !s.waitForCastComplete() { // Wait for cast to complete
						continue
					}
					s.lastCastTime = time.Now() // Update last cast time
					completedAttackLoops++
				}
			} else {
				return fmt.Errorf("tornado skill not bound")
			}
		*/
	}
}

// Helper for killing a specific monster by NPC ID and type
func (s DruidLeveling) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}
		return m.UnitID, true
	}, nil)
}

// Reapplies active buffs if they’ve expired
func (s DruidLeveling) RecastBuffs() {
	ctx := context.Get()

	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
	skills := []skill.ID{}
	states := []state.State{}

	if lvl.Value >= 37 {
		skills = []skill.ID{skill.Hurricane, skill.OakSage, skill.CycloneArmor}
		states = []state.State{state.Hurricane, state.Oaksage, state.Cyclonearmor}
	} else if lvl.Value >= 24 {
		skills = []skill.ID{skill.OakSage, skill.CycloneArmor}
		states = []state.State{state.Oaksage, state.Cyclonearmor}
	} else if lvl.Value >= 5 {
		skills = []skill.ID{skill.OakSage}
		states = []state.State{state.Oaksage}
	}

	for i, druSkill := range skills {
		if kb, found := ctx.Data.KeyBindings.KeyBindingForSkill(druSkill); found {
			if !ctx.Data.PlayerUnit.States.HasState(states[i]) { // Check if buff is missing
				ctx.HID.PressKeyBinding(kb)             // Activate skill
				utils.Sleep(180)                        // Small delay
				s.HID.Click(game.RightButton, 640, 340) // Cast skill at center screen
				utils.Sleep(100)                        // Delay to ensure cast completes
			}
		}
	}
}

// Return a list of available buff skills
func (s DruidLeveling) BuffSkills() []skill.ID {
	buffs := make([]skill.ID, 0)
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.CycloneArmor); found {
		buffs = append(buffs, skill.CycloneArmor)
	}
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Raven); found {
		buffs = append(buffs, skill.Raven, skill.Raven, skill.Raven, skill.Raven, skill.Raven) // Summon 5 ravens
	}
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Hurricane); found {
		buffs = append(buffs, skill.Hurricane)
	}
	return buffs
}

// Dynamically determines pre-combat buffs and summons
func (s DruidLeveling) PreCTABuffSkills() []skill.ID {
	needsBear := true
	wolves := 5
	direWolves := 3
	needsOak := true
	needsCreeper := true

	for _, monster := range s.Data.Monsters { // Check existing pets
		if monster.IsPet() {
			switch monster.Name {
			case npc.DruBear:
				needsBear = false
			case npc.DruFenris:
				direWolves--
			case npc.DruSpiritWolf:
				wolves--
			case npc.OakSage:
				needsOak = false
			case npc.DruCycleOfLife, npc.VineCreature, npc.DruPlaguePoppy:
				needsCreeper = false
			}
		}
	}

	skills := make([]skill.ID, 0)
	if s.Data.PlayerUnit.States.HasState(state.Oaksage) {
		needsOak = false
	}

	// Add summoning skills based on need and key bindings
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.SummonSpiritWolf); found {
		for i := 0; i < wolves; i++ {
			skills = append(skills, skill.SummonSpiritWolf)
		}
	}
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.SummonDireWolf); found {
		for i := 0; i < direWolves; i++ {
			skills = append(skills, skill.SummonDireWolf)
		}
	}
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.SummonGrizzly); found && needsBear {
		skills = append(skills, skill.SummonGrizzly)
	}
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.OakSage); found && needsOak {
		skills = append(skills, skill.OakSage)
	}
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.SolarCreeper); found && needsCreeper {
		skills = append(skills, skill.SolarCreeper)
	}
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.CarrionVine); found && needsCreeper {
		skills = append(skills, skill.CarrionVine)
	}
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.PoisonCreeper); found && needsCreeper {
		skills = append(skills, skill.PoisonCreeper)
	}

	return skills
}

func (s DruidLeveling) ShouldResetSkills() bool {
	return false
}

func (s DruidLeveling) SkillsToBind() (skill.ID, []skill.ID) {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	// Primary skill will be the basic attack for interacting with objects and as a fallback.
	mainSkill := skill.AttackSkill
	skillBindings := []skill.ID{}

	if lvl.Value >= 2 {
		skillBindings = append(skillBindings, skill.Firestorm)
	}

	if lvl.Value >= 5 {
		skillBindings = append(skillBindings, skill.OakSage)
	}

	if lvl.Value >= 12 {
		// Wake of Fire becomes the main secondary attack, replacing Fire Blast as the primary one.
		newBindings := []skill.ID{skill.Fissure}
		for _, sk := range skillBindings {
			if sk != skill.Firestorm {
				newBindings = append(newBindings, sk)
			}
		}
		skillBindings = newBindings
	}

	if lvl.Value >= 24 {
		skillBindings = append(skillBindings, skill.CycloneArmor)
	}

	if lvl.Value >= 38 {
		skillBindings = append(skillBindings, skill.Hurricane)
	}

	if s.Data.PlayerUnit.Skills[skill.BattleCommand].Level > 0 {
		skillBindings = append(skillBindings, skill.BattleCommand)
	}

	if s.Data.PlayerUnit.Skills[skill.BattleOrders].Level > 0 {
		skillBindings = append(skillBindings, skill.BattleOrders)
	}

	_, found := s.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	if found {
		skillBindings = append(skillBindings, skill.TomeOfTownPortal)
	}

	s.Logger.Info("Skills bound", "mainSkill", mainSkill, "skillBindings", skillBindings)
	return mainSkill, skillBindings
}

func (s DruidLeveling) StatPoints() []context.StatAllocation {
	stats := []context.StatAllocation{
		{Stat: stat.Energy, Points: 35},
		{Stat: stat.Vitality, Points: 55},
		{Stat: stat.Strength, Points: 35},
		{Stat: stat.Vitality, Points: 95},
		{Stat: stat.Strength, Points: 60},
		{Stat: stat.Vitality, Points: 130},
		{Stat: stat.Strength, Points: 125},
		{Stat: stat.Vitality, Points: 140},
		{Stat: stat.Strength, Points: 156},
		{Stat: stat.Vitality, Points: 999},
	}
	s.Logger.Debug("Stat point allocation plan", "stats", stats)
	return stats
}

func (s DruidLeveling) SkillPoints() []skill.ID {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	//Fire Druid
	skillSequence := []skill.ID{
		skill.Firestorm, skill.Firestorm, skill.Firestorm,
		skill.MoltenBoulder,
		skill.OakSage,
		skill.Firestorm, skill.Firestorm, skill.Firestorm, skill.Firestorm, skill.Firestorm, skill.Firestorm,
		skill.Fissure, skill.Fissure, skill.Fissure, skill.Fissure, skill.Fissure,
		skill.Fissure, skill.Fissure, skill.Fissure, skill.Fissure, skill.Fissure, skill.Fissure,
		skill.ArcticBlast,
		skill.CycloneArmor,
		skill.Volcano,
		skill.OakSage, skill.OakSage, skill.OakSage, skill.OakSage,
		skill.Fissure, skill.Fissure, skill.Fissure, skill.Fissure, skill.Fissure,
		skill.Fissure, skill.Fissure, skill.Fissure, skill.Fissure,
		skill.Twister,
		skill.Tornado,
		skill.Hurricane,
		skill.OakSage, skill.OakSage, skill.OakSage, skill.OakSage, skill.OakSage,
		skill.OakSage, skill.OakSage, skill.OakSage, skill.OakSage, skill.OakSage,
		skill.Volcano, skill.Volcano, skill.Volcano, skill.Volcano, skill.Volcano,
		skill.Volcano, skill.Volcano, skill.Volcano, skill.Volcano, skill.Volcano,
		skill.Volcano, skill.Volcano, skill.Volcano, skill.Volcano, skill.Volcano,
		skill.Volcano, skill.Volcano, skill.Volcano, skill.Volcano,
		skill.Firestorm, skill.Firestorm, skill.Firestorm, skill.Firestorm, skill.Firestorm, skill.Firestorm,
		skill.Firestorm, skill.Firestorm, skill.Firestorm, skill.Firestorm, skill.Firestorm,
		skill.OakSage, skill.OakSage, skill.OakSage, skill.OakSage, skill.OakSage,
		skill.CycloneArmor, skill.CycloneArmor, skill.CycloneArmor, skill.CycloneArmor, skill.CycloneArmor,
		skill.CycloneArmor, skill.CycloneArmor, skill.CycloneArmor, skill.CycloneArmor, skill.CycloneArmor,
		skill.CycloneArmor, skill.CycloneArmor, skill.CycloneArmor, skill.CycloneArmor, skill.CycloneArmor,
		skill.CycloneArmor, skill.CycloneArmor, skill.CycloneArmor, skill.CycloneArmor,
	}

	questSkillPoints := 0
	if s.Data.Quests[quest.Act1DenOfEvil].Completed() {
		questSkillPoints++
	}
	if s.Data.Quests[quest.Act2RadamentsLair].Completed() {
		questSkillPoints++
	}
	if s.Data.Quests[quest.Act4TheFallenAngel].Completed() {
		questSkillPoints += 2
	}

	totalPoints := (int(lvl.Value) - 1) + questSkillPoints
	if totalPoints < 0 {
		totalPoints = 0
	}

	var skillsToAllocateBasedOnLevel []skill.ID
	if totalPoints < len(skillSequence) {
		skillsToAllocateBasedOnLevel = skillSequence[:totalPoints]
	} else {
		skillsToAllocateBasedOnLevel = skillSequence
	}

	targetLevels := make(map[skill.ID]int)
	for _, sk := range skillsToAllocateBasedOnLevel {
		targetLevels[sk]++
	}

	skillsToAllocate := make([]skill.ID, 0)

	var uniqueSkills []skill.ID
	seenSkills := make(map[skill.ID]bool)
	for _, sk := range skillSequence {
		if _, seen := seenSkills[sk]; !seen {
			uniqueSkills = append(uniqueSkills, sk)
			seenSkills[sk] = true
		}
	}

	for _, sk := range uniqueSkills {
		target := targetLevels[sk]
		if target == 0 {
			continue
		}

		currentLevel := 0
		if skillData, found := s.Data.PlayerUnit.Skills[sk]; found {
			currentLevel = int(skillData.Level)
		}

		pointsToAdd := target - currentLevel
		if pointsToAdd > 0 {
			for i := 0; i < pointsToAdd; i++ {
				skillsToAllocate = append(skillsToAllocate, sk)
			}
		}
	}

	if len(skillsToAllocate) > 0 {
		s.Logger.Info("Skill allocation plan", "skills", skillsToAllocate)
	}

	return skillsToAllocate
}

func (s DruidLeveling) KillCountess() error {
	return s.killMonster(npc.DarkStalker, data.MonsterTypeSuperUnique)
}

func (s DruidLeveling) KillAndariel() error {
	return s.killMonster(npc.Andariel, data.MonsterTypeUnique)
}

func (s DruidLeveling) KillSummoner() error {
	return s.killMonster(npc.Summoner, data.MonsterTypeUnique)
}

func (s DruidLeveling) KillDuriel() error {
	return s.killMonster(npc.Duriel, data.MonsterTypeUnique)
}

// Targets multiple council members, sorted by distance
func (s DruidLeveling) KillCouncil() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		var councilMembers []data.Monster
		for _, m := range d.Monsters {
			if m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3 {
				councilMembers = append(councilMembers, m)
			}
		}

		sort.Slice(councilMembers, func(i, j int) bool {
			return s.PathFinder.DistanceFromMe(councilMembers[i].Position) < s.PathFinder.DistanceFromMe(councilMembers[j].Position)
		})

		for _, m := range councilMembers {
			return m.UnitID, true
		}

		return 0, false
	}, nil)
}

func (s DruidLeveling) KillMephisto() error {
	return s.killMonster(npc.Mephisto, data.MonsterTypeUnique)
}

func (s DruidLeveling) KillIzual() error {
	return s.killMonster(npc.Izual, data.MonsterTypeUnique)
}

// KillDiablo includes a timeout and detection logic
func (s DruidLeveling) KillDiablo() error {
	timeout := time.Second * 20
	startTime := time.Now()
	diabloFound := false

	for {
		if time.Since(startTime) > timeout && !diabloFound {
			s.Logger.Error("Diablo was not found, timeout reached")
			return nil
		}

		diablo, found := s.Data.Monsters.FindOne(npc.Diablo, data.MonsterTypeUnique)
		if !found || diablo.Stats[stat.Life] <= 0 { // Check if Diablo is dead or missing
			// Diablo is dead
			if diabloFound {
				return nil
			}
			// Keep waiting..
			time.Sleep(200 * time.Millisecond)
			continue
		}

		diabloFound = true
		s.Logger.Info("Diablo detected, attacking")
		return s.killMonster(npc.Diablo, data.MonsterTypeUnique)
	}
}

func (s DruidLeveling) KillPindle() error {
	return s.killMonster(npc.DefiledWarrior, data.MonsterTypeSuperUnique)
}

func (s DruidLeveling) KillAncients() error {
	originalBackToTownCfg := s.CharacterCfg.BackToTown
	s.CharacterCfg.BackToTown.NoHpPotions = false
	s.CharacterCfg.BackToTown.NoMpPotions = false
	s.CharacterCfg.BackToTown.EquipmentBroken = false
	s.CharacterCfg.BackToTown.MercDied = false

	for _, m := range s.Data.Monsters.Enemies(data.MonsterEliteFilter()) {
		foundMonster, found := s.Data.Monsters.FindOne(m.Name, data.MonsterTypeSuperUnique)
		if !found {
			continue
		}
		step.MoveTo(data.Position{X: 10062, Y: 12639})

		s.killMonster(foundMonster.Name, data.MonsterTypeSuperUnique)

	}

	s.CharacterCfg.BackToTown = originalBackToTownCfg
	s.Logger.Info("Restored original back-to-town checks after Ancients fight.")
	return nil
}

func (s DruidLeveling) KillNihlathak() error {
	return s.killMonster(npc.Nihlathak, data.MonsterTypeSuperUnique)
}

func (s DruidLeveling) KillBaal() error {
	return s.killMonster(npc.BaalCrab, data.MonsterTypeUnique)
}
