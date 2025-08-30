package character

import (
	"errors"
	"fmt"
	"log/slog"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

const (
	paladinLevelingMaxAttacksLoop = 10
)

type PaladinLeveling struct {
	BaseCharacter
}

func (s PaladinLeveling) CheckKeyBindings() []skill.ID {
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

func (s PaladinLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	const priorityMonsterSearchRange = 15
	completedAttackLoops := 0
	previousUnitID := 0

	priorityMonsters := []npc.ID{npc.FallenShaman, npc.MummyGenerator, npc.BaalSubjectMummy, npc.FetishShaman, npc.CarverShaman}

	for {
		var id data.UnitID
		var found bool

		var closestPriorityMonster data.Monster
		minDistance := -1

		for _, monsterNpcID := range priorityMonsters {

			for _, m := range s.Data.Monsters {
				if m.Name == monsterNpcID && m.Stats[stat.Life] > 0 {
					distance := s.PathFinder.DistanceFromMe(m.Position)
					if distance < priorityMonsterSearchRange {
						if minDistance == -1 || distance < minDistance {
							minDistance = distance
							closestPriorityMonster = m
						}
					}
				}
			}
		}

		if minDistance != -1 {
			id = closestPriorityMonster.UnitID
			found = true
			s.Logger.Debug("Priority monster found", "name", closestPriorityMonster.Name, "distance", minDistance)
		}

		if !found {
			id, found = monsterSelector(*s.Data)
		}

		if !found {
			return nil
		}

		if previousUnitID != int(id) {
			completedAttackLoops = 0
		}

		if !s.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		if completedAttackLoops >= paladinLevelingMaxAttacksLoop {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			s.Logger.Info("Monster not found", slog.String("monster", fmt.Sprintf("%v", monster)))
			return nil
		}

		numOfAttacks := 5

		if s.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 {
			s.Logger.Debug("Using Blessed Hammer")
			// Add a random movement, maybe hammer is not hitting the target
			if previousUnitID == int(id) {
				if monster.Stats[stat.Life] > 0 {
					s.PathFinder.RandomMovement()
				}
				return nil
			}
			step.PrimaryAttack(id, numOfAttacks, false, step.Distance(2, 7), step.EnsureAura(skill.Concentration))

			// Perform random movement to reposition for the next attack
			s.Logger.Debug("Performing random movement to reposition.")
			s.PathFinder.RandomMovement()
			time.Sleep(time.Millisecond * 150)
		} else {
			if s.Data.PlayerUnit.Skills[skill.Zeal].Level > 0 {
				s.Logger.Debug("Using Zeal")
				numOfAttacks = 1
			}
			s.Logger.Debug("Using primary attack with Holy Fire aura")
			step.PrimaryAttack(id, numOfAttacks, false, step.Distance(1, 3), step.EnsureAura(skill.HolyFire))
		}

		completedAttackLoops++
		previousUnitID = int(id)
	}
}

func (s PaladinLeveling) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}

		return m.UnitID, true
	}, nil)
}

func (s PaladinLeveling) BuffSkills() []skill.ID {
	skillsList := make([]skill.ID, 0)
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.HolyShield); found {
		skillsList = append(skillsList, skill.HolyShield)
	}
	s.Logger.Info("Buff skills", "skills", skillsList)
	return skillsList
}

func (s PaladinLeveling) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

func (s PaladinLeveling) ShouldResetSkills() bool {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
	if lvl.Value == 24 && s.Data.PlayerUnit.Skills[skill.HolyFire].Level > 10 {
		s.Logger.Info("Resetting skills: Level 24 and Holy Fire level > 10")
		return true
	}

	return false
}

func (s PaladinLeveling) SkillsToBind() (skill.ID, []skill.ID) {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
	mainSkill := skill.AttackSkill
	skillBindings := []skill.ID{}

	if lvl.Value >= 6 {
		skillBindings = append(skillBindings, skill.Vigor)
	}

	if lvl.Value >= 30 {
		skillBindings = append(skillBindings, skill.HolyShield)
	}

	if s.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 && lvl.Value >= 18 {
		mainSkill = skill.BlessedHammer
	} else if s.Data.PlayerUnit.Skills[skill.Zeal].Level > 0 {
		mainSkill = skill.Zeal
	}

	if s.Data.PlayerUnit.Skills[skill.Concentration].Level > 0 && lvl.Value >= 18 {
		skillBindings = append(skillBindings, skill.Concentration)
	} else {
		if _, found := s.Data.PlayerUnit.Skills[skill.HolyFire]; found {
			skillBindings = append(skillBindings, skill.HolyFire)
		} else if _, found := s.Data.PlayerUnit.Skills[skill.Might]; found {
			skillBindings = append(skillBindings, skill.Might)
		}
	}

	s.Logger.Info("Skills bound", "mainSkill", mainSkill, "skillBindings", skillBindings)
	return mainSkill, skillBindings
}

func (s PaladinLeveling) StatPoints() []context.StatAllocation {

	// Define target totals (including base stats)
	targets := []context.StatAllocation{
		{Stat: stat.Vitality, Points: 30},
		{Stat: stat.Strength, Points: 30},
		{Stat: stat.Vitality, Points: 35},
		{Stat: stat.Strength, Points: 35},
		{Stat: stat.Vitality, Points: 40},
		{Stat: stat.Strength, Points: 40},
		{Stat: stat.Vitality, Points: 50},
		{Stat: stat.Strength, Points: 80},
		{Stat: stat.Vitality, Points: 100},
		{Stat: stat.Strength, Points: 95},
		{Stat: stat.Vitality, Points: 250},
		{Stat: stat.Strength, Points: 156},
		{Stat: stat.Vitality, Points: 999},
	}

	return targets
}

func (s PaladinLeveling) SkillPoints() []skill.ID {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	var skillSequence []skill.ID

	if lvl.Value < 23 {
		// Holy Fire build allocation for levels 1-23
		skillSequence = []skill.ID{
			skill.Might,       // Lvl 2
			skill.Sacrifice,   // Lvl 3
			skill.ResistFire,  // Lvl 4
			skill.ResistFire,  // Lvl 5
			skill.ResistFire,  // Lvl 6
			skill.HolyFire,    // Lvl 7
			skill.HolyFire,    // Lvl 8
			skill.HolyFire,    // Lvl 9
			skill.HolyFire,    // Lvl 10
			skill.HolyFire,    // Lvl 11
			skill.HolyFire,    // Lvl 12
			skill.Zeal,        // Lvl 13
			skill.HolyFire,    // Lvl 14
			skill.HolyFire,    // Lvl 15
			skill.HolyFire,    // Lvl 16
			skill.HolyFire,    // Lvl 17
			skill.HolyFire,    // Lvl 18
			skill.HolyFire,    // Lvl 19
			skill.HolyFire,    // Lvl 20
			skill.HolyFire,    // Lvl 21
			skill.HolyFire,    // Lvl 22
			skill.HolyFire,    // Lvl 23
		}
	} else {
		// Hammerdin build allocation for levels 24+
		skillSequence = []skill.ID{
			// Prerequisites for core skills
			skill.Might, skill.HolyBolt, skill.Prayer, skill.Defiance, skill.BlessedAim,
			skill.Cleansing, skill.Concentration, skill.Vigor, skill.Smite, skill.Charge,
			skill.BlessedHammer,

			// Points from level 24-29
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,

			// Level 30 point
			skill.HolyShield,

			// Continue maxing Blessed Hammer
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,

			// Max Vigor (Synergy)
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,

			// Max Blessed Aim (Synergy)
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,

			// Max Concentration (Aura)
			skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration,
			skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration,
			skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration,
			skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration,

			// Rest into Holy Shield
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield,
		}
	}

	// This logic now applies to both builds
	skillsToAllocate := make([]skill.ID, 0)
	targetLevels := make(map[skill.ID]int)
	for _, sk := range skillSequence {
		targetLevels[sk]++
		currentLevel := 0
		if skillData, found := s.Data.PlayerUnit.Skills[sk]; found {
			currentLevel = int(skillData.Level)
		}

		// If the character's current level for this skill is less than the target,
		// add it to the list of skills we need to allocate points to.
		if currentLevel < targetLevels[sk] {
			skillsToAllocate = append(skillsToAllocate, sk)
		}
	}

	if len(skillsToAllocate) > 0 {
		s.Logger.Info("Skill allocation plan", "skills", skillsToAllocate)
	}

	return skillsToAllocate
}

func (s PaladinLeveling) KillCountess() error {
	return s.killMonster(npc.DarkStalker, data.MonsterTypeSuperUnique)
}

func (s PaladinLeveling) KillAndariel() error {
		s.Logger.Info("Starting Andariel kill sequence...")
	timeout := time.Second * 160
	startTime := time.Now()

	for {
		andariel, found := s.Data.Monsters.FindOne(npc.Andariel, data.MonsterTypeUnique)
		if !found {
			if time.Since(startTime) > timeout {
				s.Logger.Error("Andariel was not found, timeout reached.")
				return errors.New("Andariel not found within the time limit")
			}
			time.Sleep(time.Second / 2)
			continue
		}

		if andariel.Stats[stat.Life] <= 0 {
			s.Logger.Info("Andariel is dead.")
			return nil
		}

		numOfAttacks := 5
		if s.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 {
			step.PrimaryAttack(andariel.UnitID, numOfAttacks, false, step.Distance(2, 7), step.EnsureAura(skill.Concentration))
			s.Logger.Debug("Performing random movement to reposition.")
			s.PathFinder.RandomMovement()
			time.Sleep(time.Millisecond * 250)
		} else {
			if s.Data.PlayerUnit.Skills[skill.Zeal].Level > 0 {
				numOfAttacks = 1 // Zeal is a multi-hit skill, 1 click is a sequence of attacks
			}
			step.PrimaryAttack(andariel.UnitID, numOfAttacks, false, step.Distance(1, 3), step.EnsureAura(skill.HolyFire))
		}
	}
}

func (s PaladinLeveling) KillSummoner() error {
	return s.killMonster(npc.Summoner, data.MonsterTypeUnique)
}

func (s PaladinLeveling) KillDuriel() error {
	s.Logger.Info("Starting Duriel kill sequence...")
	timeout := time.Second * 120
	startTime := time.Now()

	for {
		duriel, found := s.Data.Monsters.FindOne(npc.Duriel, data.MonsterTypeUnique)
		if !found {
			if time.Since(startTime) > timeout {
				s.Logger.Error("Duriel was not found, timeout reached.")
				return errors.New("Duriel not found within the time limit")
			}
			time.Sleep(time.Second / 2)
			continue
		}

		if duriel.Stats[stat.Life] <= 0 {
			s.Logger.Info("Duriel is dead.")
			return nil
		}

		numOfAttacks := 5
		if s.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 {
			step.PrimaryAttack(duriel.UnitID, numOfAttacks, false, step.Distance(2, 7), step.EnsureAura(skill.Concentration))
			s.Logger.Debug("Performing random movement to reposition.")
			s.PathFinder.RandomMovement()
			time.Sleep(time.Millisecond * 250)
		} else {
			if s.Data.PlayerUnit.Skills[skill.Zeal].Level > 0 {
				numOfAttacks = 1 // Zeal is a multi-hit skill, 1 click is a sequence of attacks
			}
			step.PrimaryAttack(duriel.UnitID, numOfAttacks, false, step.Distance(1, 3), step.EnsureAura(skill.HolyFire))
		}
	}
}

func (s PaladinLeveling) KillCouncil() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		var councilMembers []data.Monster
		for _, m := range d.Monsters {
			if m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3 {
				councilMembers = append(councilMembers, m)
			}
		}

		// Order council members by distance
		sort.Slice(councilMembers, func(i, j int) bool {
			distanceI := s.PathFinder.DistanceFromMe(councilMembers[i].Position)
			distanceJ := s.PathFinder.DistanceFromMe(councilMembers[j].Position)

			return distanceI < distanceJ
		})

		if len(councilMembers) > 0 {
			s.Logger.Debug("Targeting Council member", "id", councilMembers[0].UnitID)
			return councilMembers[0].UnitID, true
		}

		s.Logger.Debug("No Council members found")
		return 0, false
	}, nil)
}

func (s PaladinLeveling) KillMephisto() error {
	s.Logger.Info("Starting Mephisto kill sequence...")
	timeout := time.Second * 160
	startTime := time.Now()

	for {
		mephisto, found := s.Data.Monsters.FindOne(npc.Mephisto, data.MonsterTypeUnique)
		if !found {
			if time.Since(startTime) > timeout {
				s.Logger.Error("Mephisto was not found, timeout reached.")
				return errors.New("Mephisto not found within the time limit")
			}
			time.Sleep(time.Second / 2)
			continue
		}

		if mephisto.Stats[stat.Life] <= 0 {
			s.Logger.Info("Mephisto is dead.")
			return nil
		}

		numOfAttacks := 5
		if s.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 {
			step.PrimaryAttack(mephisto.UnitID, numOfAttacks, false, step.Distance(2, 7), step.EnsureAura(skill.Concentration))
			s.Logger.Debug("Performing random movement to reposition.")
			s.PathFinder.RandomMovement()
			time.Sleep(time.Millisecond * 250)
		} else {
			if s.Data.PlayerUnit.Skills[skill.Zeal].Level > 0 {
				numOfAttacks = 1 // Zeal is a multi-hit skill, 1 click is a sequence of attacks
			}
			step.PrimaryAttack(mephisto.UnitID, numOfAttacks, false, step.Distance(1, 3), step.EnsureAura(skill.HolyFire))
		}
	}
}

func (s PaladinLeveling) KillIzual() error {
	s.Logger.Info("Starting Izual kill sequence...")
	timeout := time.Second * 120
	startTime := time.Now()

	for {
		izual, found := s.Data.Monsters.FindOne(npc.Izual, data.MonsterTypeUnique)
		if !found {
			if time.Since(startTime) > timeout {
				s.Logger.Error("Izual was not found, timeout reached.")
				return errors.New("Izual not found within the time limit")
			}
			time.Sleep(time.Second / 2)
			continue
		}

		distance := s.PathFinder.DistanceFromMe(izual.Position)
		if distance > 7 {
			// Izual is too far, move closer to him instead of waiting.
			s.Logger.Debug(fmt.Sprintf("Izual is too far away (%d), moving closer.", distance))
			step.MoveTo(izual.Position)
			continue // Restart the loop to re-evaluate distance
		}

		if izual.Stats[stat.Life] <= 0 {
			s.Logger.Info("Izual is dead.")
			return nil
		}

		numOfAttacks := 5
		if s.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 {
			step.PrimaryAttack(izual.UnitID, numOfAttacks, false, step.Distance(2, 7), step.EnsureAura(skill.Concentration))
			s.Logger.Debug("Performing random movement to reposition.")
			s.PathFinder.RandomMovement()
			time.Sleep(time.Millisecond * 250)
		} else {
			if s.Data.PlayerUnit.Skills[skill.Zeal].Level > 0 {
				numOfAttacks = 1 // Zeal is a multi-hit skill, 1 click is a sequence of attacks
			}
			step.PrimaryAttack(izual.UnitID, numOfAttacks, false, step.Distance(1, 3), step.EnsureAura(skill.HolyFire))
		}
	}
}

func (s PaladinLeveling) KillDiablo() error {
	s.Logger.Info("Starting Diablo kill sequence...")
	// Increased timeout to find Diablo, giving more time for him to spawn.
	timeout := time.Second * 120
	startTime := time.Now()

	// This is now a persistent loop that will continue until Diablo is dead.
	for {
		// Step 1: Find Diablo.
		diablo, found := s.Data.Monsters.FindOne(npc.Diablo, data.MonsterTypeUnique)

		// If Diablo is not found, wait and retry until the timeout is reached.
		if !found {
			if time.Since(startTime) > timeout {
				s.Logger.Error("Diablo was not found, timeout reached.")
				return errors.New("diablo not found within the time limit")
			}
			time.Sleep(time.Second / 2) // Wait half a second before checking again.
			continue
		}

		// Step 2: Check if Diablo is already dead.
		if diablo.Stats[stat.Life] <= 0 {
			s.Logger.Info("Diablo is dead.")
			return nil // Success!
		}

		numOfAttacks := 10
		if s.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 {
			step.PrimaryAttack(diablo.UnitID, numOfAttacks, false, step.Distance(2, 7), step.EnsureAura(skill.Concentration))
			time.Sleep(time.Millisecond * 250)
		} else {
			if s.Data.PlayerUnit.Skills[skill.Zeal].Level > 0 {
				numOfAttacks = 1
			}
			step.PrimaryAttack(diablo.UnitID, numOfAttacks, false, step.Distance(1, 3), step.EnsureAura(skill.HolyFire))
		}
	}
}

func (s PaladinLeveling) KillPindle() error {
	return s.killMonster(npc.DefiledWarrior, data.MonsterTypeSuperUnique)
}

func (s PaladinLeveling) KillNihlathak() error {
	return s.killMonster(npc.Nihlathak, data.MonsterTypeSuperUnique)
}

func (s PaladinLeveling) KillAncients() error {
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

func (s PaladinLeveling) KillBaal() error {

	s.Logger.Info("Starting Baal kill sequence...")
	// Increased timeout to find Baal, giving more time for him to spawn.
	timeout := time.Second * 600
	startTime := time.Now()

	// This is now a persistent loop that will continue until Baal is dead.
	for {
		baal, found := s.Data.Monsters.FindOne(npc.BaalCrab, data.MonsterTypeUnique)

		if !found {
			if time.Since(startTime) > timeout {
				s.Logger.Error("Baal was not found, timeout reached.")
				return errors.New("Baal not found within the time limit")
			}
			time.Sleep(time.Second / 2) // Wait half a second before checking again.
			continue
		}

		if baal.Stats[stat.Life] <= 0 {
			s.Logger.Info("Baal is dead.")
			return nil // Success!
		}

		numOfAttacks := 5
		if s.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 {
			step.PrimaryAttack(baal.UnitID, numOfAttacks, false, step.Distance(2, 7), step.EnsureAura(skill.Concentration))
			s.Logger.Debug("Performing random movement to reposition.")
			s.PathFinder.RandomMovement()
			time.Sleep(time.Millisecond * 250)
		} else {
			if s.Data.PlayerUnit.Skills[skill.Zeal].Level > 0 {
				numOfAttacks = 1 // Zeal is a multi-hit skill, 1 click is a sequence of attacks
			}
			step.PrimaryAttack(baal.UnitID, numOfAttacks, false, step.Distance(1, 3), step.EnsureAura(skill.HolyFire))
		}
	}
}
