package character

import (
	"fmt"
	"log/slog"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

const (
	assassinMaxAttacksLoop = 3
	levelingminDistance = 10
	levelingmaxDistance = 15
)

type AssassinLeveling struct {
	BaseCharacter
}

func (s AssassinLeveling) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{}
	missingKeybindings := []skill.ID{}

	for _, cskill := range requireKeybindings {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(cskill); !found {
			missingKeybindings = append(missingKeybindings, cskill)
		}
	}

	if len(missingKeybindings) > 0 {
		s.Logger.Debug("There are missing required key bindings.", slog.Any("Bindings", missingKeybindings))
	}

	return missingKeybindings
}

func (s AssassinLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0

	for {
		id, found := monsterSelector(*s.Data)
		if !found {
			return nil
		}
		if previousUnitID != int(id) {
			completedAttackLoops = 0
		}

		if !s.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		if completedAttackLoops >= assassinMaxAttacksLoop {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			s.Logger.Info("Monster not found", slog.String("monster", fmt.Sprintf("%v", monster)))
			return nil
		}

		lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
		mana, _ := s.Data.PlayerUnit.FindStat(stat.Mana, 0)

		mainAttackSkill := skill.FireBlast // Default before we get Wake of Fire.
		if lvl.Value >= 12 {
			mainAttackSkill = skill.WakeOfFire
		}

		if lvl.Value < 48 {
			if s.Data.PlayerUnit.Skills[mainAttackSkill].Level > 0 && mana.Value > 2 {
				step.SecondaryAttack(mainAttackSkill, id, 5, step.Distance(levelingminDistance, levelingmaxDistance))
			} else {
				// Fallback to primary skill (basic attack) at close range when out of mana.
				step.PrimaryAttack(id, 1, true, step.Distance(1, 3))
			}
		} else {
			// Post-reset Trapsin logic.
			opts := []step.AttackOption{step.Distance(levelingminDistance, levelingmaxDistance)}
			step.SecondaryAttack(skill.LightningSentry, id, 3, opts...)
			step.SecondaryAttack(skill.DeathSentry, id, 2, opts...)
			step.SecondaryAttack(skill.FireBlast, id, 2, opts...) 
		}

		completedAttackLoops++
		previousUnitID = int(id)
	}
}

func (s AssassinLeveling) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}

		return m.UnitID, true
	}, nil)
}

func (s AssassinLeveling) BuffSkills() []skill.ID {
	skillsList := make([]skill.ID, 0)
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
	
	if lvl.Value < 18 {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.BurstOfSpeed); found {
			skillsList = append(skillsList, skill.BurstOfSpeed)
		}
	} else { 
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Fade); found {
			skillsList = append(skillsList, skill.Fade)
		}
	}

	return skillsList
}
func (s AssassinLeveling) PreCTABuffSkills() []skill.ID {
	armor := skill.ShadowWarrior
	armors := []skill.ID{skill.ShadowWarrior, skill.ShadowMaster}
	hasShadow := false
	for _, arm := range armors {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(arm); found {
			armor = arm
			hasShadow = true
		}
	}

	if hasShadow {
		return []skill.ID{armor}
	}

	return []skill.ID{}
}

func (s AssassinLeveling) ShouldResetSkills() bool {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
	if lvl.Value == 48 && s.Data.PlayerUnit.Skills[skill.WakeOfFire].Level > 15 {
		s.Logger.Info("Resetting skills: Level 48 and Wake of Fire level > 15")
		return true
	}

	return false
}

func (s AssassinLeveling) SkillsToBind() (skill.ID, []skill.ID) {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	// Primary skill will be the basic attack for interacting with objects and as a fallback.
	mainSkill := skill.AttackSkill
	skillBindings := []skill.ID{} 

	if lvl.Value >= 2 {
		skillBindings = append(skillBindings, skill.FireBlast)
	}

	if lvl.Value >= 6 {
		skillBindings = append(skillBindings, skill.BurstOfSpeed)
	}

	if lvl.Value >= 12 {
		// Wake of Fire becomes the main secondary attack, replacing Fire Blast as the primary one.
		newBindings := []skill.ID{skill.WakeOfFire}
		for _, sk := range skillBindings {
			if sk != skill.FireBlast {
				newBindings = append(newBindings, sk)
			}
		}
		skillBindings = newBindings
	}

	if lvl.Value >= 18 {
		skillBindings = append(skillBindings, skill.Fade)
	}

	if lvl.Value >= 48 {
		// Post-reset Trapsin build.
		mainSkill = skill.AttackSkill
		skillBindings = []skill.ID{
			skill.LightningSentry,
			skill.DeathSentry,
			skill.FireBlast, 
		}
	}

	_, found := s.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	if found {
		skillBindings = append(skillBindings, skill.TomeOfTownPortal)
	}

	s.Logger.Info("Skills bound", "mainSkill", mainSkill, "skillBindings", skillBindings)
	return mainSkill, skillBindings
}

func (s AssassinLeveling) StatPoints() []context.StatAllocation {
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

func (s AssassinLeveling) SkillPoints() []skill.ID {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	var skillSequence []skill.ID

	if lvl.Value < 48 {
		skillSequence = []skill.ID{
			skill.FireBlast,
			// Den of Evil reward
			skill.ClawMastery,
			// Continue with the build
			skill.FireBlast, skill.FireBlast, skill.FireBlast,
			skill.BurstOfSpeed, skill.BurstOfSpeed,
			skill.FireBlast,
			skill.BurstOfSpeed, skill.BurstOfSpeed, skill.BurstOfSpeed,
			skill.WakeOfFire, skill.WakeOfFire,
			skill.WakeOfFire, // Radament
			skill.WakeOfFire, skill.WakeOfFire, skill.WakeOfFire,
			skill.Fade,
			skill.WakeOfFire,
			skill.Fade, // Izual
			skill.WakeOfFire, skill.WakeOfFire, skill.WakeOfFire, skill.WakeOfFire, skill.WakeOfFire,
			skill.Fade,
			skill.WakeOfFire, skill.WakeOfFire, skill.WakeOfFire, skill.WakeOfFire, skill.WakeOfFire, skill.WakeOfFire,
			skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast,
			skill.WakeOfFire, skill.WakeOfFire,
			skill.WeaponBlock,
			skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade,
		}
	} else {
		// TRAPSIN BUILD (LVL 48+)
		skillSequence = []skill.ID{
			skill.PsychicHammer, skill.ClawMastery, skill.CloakOfShadows, skill.MindBlast,
			skill.BurstOfSpeed, skill.FireBlast, skill.ShockWeb, skill.ChargedBoltSentry, skill.LightningSentry, skill.Fade,
			skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade,
			skill.LightningSentry, skill.LightningSentry, skill.LightningSentry, skill.LightningSentry, skill.LightningSentry,
			skill.LightningSentry, skill.LightningSentry, skill.LightningSentry, skill.LightningSentry, skill.LightningSentry,
			skill.LightningSentry, skill.LightningSentry, skill.LightningSentry, skill.LightningSentry, skill.LightningSentry,
			skill.DeathSentry, skill.DeathSentry, skill.DeathSentry, skill.DeathSentry, skill.DeathSentry,
			skill.DeathSentry, skill.DeathSentry, skill.DeathSentry, skill.DeathSentry,
			skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry,
			skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry,
			skill.LightningSentry, skill.LightningSentry, skill.LightningSentry, skill.LightningSentry, // Max LS
			skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry, skill.ChargedBoltSentry, // Max CBS
			skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, skill.ShockWeb, // Max Shock Web
			skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade, skill.Fade, // Max Fade
			skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, skill.FireBlast, // Max Fire Blast
		}
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

func (s AssassinLeveling) killBoss(bossNPC npc.ID, timeout time.Duration) error {
	s.Logger.Info(fmt.Sprintf("Starting kill sequence for %s...", bossNPC))
	startTime := time.Now()

	for time.Since(startTime) < timeout {
		boss, found := s.Data.Monsters.FindOne(bossNPC, data.MonsterTypeUnique)

		if found && boss.Stats[stat.Life] > 0 {
			s.Logger.Info(fmt.Sprintf("%s has been found! Engaging...", bossNPC))

			for {
				boss, stillAlive := s.Data.Monsters.FindOne(bossNPC, data.MonsterTypeUnique)
				if !stillAlive || boss.Stats[stat.Life] <= 0 {
					s.Logger.Info(fmt.Sprintf("%s has been defeated.", bossNPC))
					if bossNPC == npc.BaalCrab {
						s.Logger.Info("Waiting...")
						time.Sleep(time.Second * 1)
					}
					return nil
				}
				s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
					return boss.UnitID, true
				}, nil)
			}
		}

		time.Sleep(time.Second) 
	}

	s.Logger.Error(fmt.Sprintf("Timed out waiting for %s.", bossNPC))
	return fmt.Errorf("%s timeout", bossNPC)
}

func (s AssassinLeveling) killMonsterByName(id npc.ID, monsterType data.MonsterType, skipOnImmunities []stat.Resist) error {
	s.Logger.Info(fmt.Sprintf("Starting persistent kill sequence for %s...", id))
	
	for {
		monster, found := s.Data.Monsters.FindOne(id, monsterType)
		if !found {
			s.Logger.Info(fmt.Sprintf("%s not found, assuming dead.", id))
			return nil
		}

		if monster.Stats[stat.Life] <= 0 {
			s.Logger.Info(fmt.Sprintf("%s is dead.", id))
			return nil
		}

		err := s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			m, found := d.Monsters.FindOne(id, monsterType)
			if !found {
				return 0, false
			}
			return m.UnitID, true
		}, skipOnImmunities)
		
		if err != nil {
			s.Logger.Warn(fmt.Sprintf("Error during KillMonsterSequence for %s: %v", id, err))
		}

		time.Sleep(time.Millisecond * 250)
	}
}

func (s AssassinLeveling) KillCountess() error {
	return s.killMonsterByName(npc.DarkStalker, data.MonsterTypeSuperUnique, nil)
}

func (s AssassinLeveling) KillAndariel() error {
	return s.killMonsterByName(npc.Andariel, data.MonsterTypeUnique, nil)
}

func (s AssassinLeveling) KillSummoner() error {
	return s.killMonsterByName(npc.Summoner, data.MonsterTypeUnique, nil)
}

func (s AssassinLeveling) KillDuriel() error {
	return s.killMonsterByName(npc.Duriel, data.MonsterTypeUnique, nil)
}

func (s AssassinLeveling) KillCouncil() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		var councilMembers []data.Monster
		for _, m := range d.Monsters {
			if m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3 {
				councilMembers = append(councilMembers, m)
			}
		}

		sort.Slice(councilMembers, func(i, j int) bool {
			distanceI := s.PathFinder.DistanceFromMe(councilMembers[i].Position)
			distanceJ := s.PathFinder.DistanceFromMe(councilMembers[j].Position)

			return distanceI < distanceJ
		})

		for _, m := range councilMembers {
			return m.UnitID, true
		}

		return 0, false
	}, nil)
}

func (s AssassinLeveling) KillMephisto() error {
	return s.killMonsterByName(npc.Mephisto, data.MonsterTypeUnique, nil)
}

func (s AssassinLeveling) KillIzual() error {
	return s.killMonsterByName(npc.Izual, data.MonsterTypeUnique, nil)
}

func (s AssassinLeveling) KillDiablo() error {
	return s.killBoss(npc.Diablo, time.Second*220)
}


func (s AssassinLeveling) KillPindle() error {
	return s.killMonsterByName(npc.DefiledWarrior, data.MonsterTypeSuperUnique, nil)
}

func (s AssassinLeveling) KillAncients() error {
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

func (s AssassinLeveling) KillNihlathak() error {
	return s.killMonsterByName(npc.Nihlathak, data.MonsterTypeSuperUnique, nil)
}

func (s AssassinLeveling) KillBaal() error {
	return s.killBoss(npc.BaalCrab, time.Second*240)
}