package character

import (
	"errors"
	"fmt"
	"log/slog"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

var _ context.LevelingCharacter = (*PaladinLeveling)(nil)

const (
	paladinLevelingMaxAttacksLoop = 10
)

type PaladinLeveling struct {
	BaseCharacter
}

func (s PaladinLeveling) ShouldIgnoreMonster(m data.Monster) bool {
	return false
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
		context.Get().PauseIfNotPriority()

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
		lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

		if s.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 {
			s.Logger.Debug("Using Blessed Hammer")
			if previousUnitID == int(id) {
				if monster.Stats[stat.Life] > 0 {
					s.PathFinder.RandomMovement()
				}
				return nil
			}
			step.PrimaryAttack(id, numOfAttacks, false, step.Distance(2, 7), step.EnsureAura(skill.Concentration))

			s.Logger.Debug("Performing random movement to reposition.")
			s.PathFinder.RandomMovement()
			time.Sleep(time.Millisecond * 150)
		} else if lvl.Value < 6 {
			s.Logger.Debug("Using Might and Sacrifice")
			numOfAttacks = 1
			step.PrimaryAttack(id, numOfAttacks, false, step.Distance(1, 3), step.EnsureAura(skill.Might))
		} else if lvl.Value >= 6 && lvl.Value < 12 {
			s.Logger.Debug("Using Holy Fire and Sacrifice")
			numOfAttacks = 1
			step.PrimaryAttack(id, numOfAttacks, false, step.Distance(1, 3), step.EnsureAura(skill.HolyFire))
		} else { // 12-24
			s.Logger.Debug("Using Holy Fire and Zeal")
			numOfAttacks = 1
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

	if lvl.Value >= 24 {
		skillBindings = append(skillBindings, skill.BlessedHammer)
	}

	if s.Data.PlayerUnit.Skills[skill.HolyShield].Level > 0 {
		skillBindings = append(skillBindings, skill.HolyShield)
	}

	if s.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 && lvl.Value >= 18 {
		mainSkill = skill.BlessedHammer
	} else if lvl.Value < 6 {
		mainSkill = skill.Sacrifice
	} else if lvl.Value >= 6 && lvl.Value < 12 {
		mainSkill = skill.Sacrifice
	} else {
		mainSkill = skill.Zeal
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

	if s.Data.PlayerUnit.Skills[skill.Concentration].Level > 0 && lvl.Value >= 18 {
		skillBindings = append(skillBindings, skill.Concentration)
	} else {
		if lvl.Value < 6 {
			if _, found := s.Data.PlayerUnit.Skills[skill.Might]; found {
				skillBindings = append(skillBindings, skill.Might)
			}
		} else {
			if _, found := s.Data.PlayerUnit.Skills[skill.HolyFire]; found {
				skillBindings = append(skillBindings, skill.HolyFire)
			}
		}
	}

	s.Logger.Info("Skills bound", "mainSkill", mainSkill, "skillBindings", skillBindings)
	return mainSkill, skillBindings
}

func (s PaladinLeveling) StatPoints() []context.StatAllocation {

	// Define target totals (including base stats)
	targets := []context.StatAllocation{
		{Stat: stat.Vitality, Points: 30},   // lvl 3
		{Stat: stat.Strength, Points: 30},   // lvl 4
		{Stat: stat.Vitality, Points: 35},   // lvl 5
		{Stat: stat.Strength, Points: 35},   // lvl 6
		{Stat: stat.Vitality, Points: 40},   // lvl 7
		{Stat: stat.Strength, Points: 40},   // lvl 8
		{Stat: stat.Vitality, Points: 50},   // lvl 10
		{Stat: stat.Strength, Points: 80},   // lvl 16
		{Stat: stat.Vitality, Points: 100},  // lvl 26
		{Stat: stat.Strength, Points: 95},   // lvl 29
		{Stat: stat.Vitality, Points: 205},  // lvl 50
		{Stat: stat.Dexterity, Points: 100}, // lvl 66
		{Stat: stat.Vitality, Points: 999},
	}

	return targets
}

func (s PaladinLeveling) SkillPoints() []skill.ID {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	var skillSequence []skill.ID

	if lvl.Value < 24 {
		// Holy Fire build allocation for levels 1-23
		skillSequence = []skill.ID{
			skill.Might, skill.Sacrifice, skill.ResistFire, skill.ResistFire, skill.ResistFire,
			skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire,
			skill.HolyFire, skill.Zeal, skill.HolyFire, skill.HolyFire, skill.HolyFire,
			skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire,
			skill.HolyFire, skill.HolyFire, skill.HolyFire,
		}
	} else {
		// Hammerdin build allocation for levels 24+
		skillSequence = []skill.ID{
			skill.Might, skill.HolyBolt, skill.Prayer, skill.Defiance, skill.BlessedAim,
			skill.Cleansing, skill.Concentration, skill.Vigor, skill.Smite, skill.Charge,
			skill.BlessedHammer,
			skill.HolyShield,
			skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
			skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
			skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
			skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
			skill.BlessedHammer, skill.Concentration,
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
			skill.BlessedHammer,
			skill.Concentration, skill.Concentration, skill.Concentration,
			skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration,
			skill.Concentration, skill.Concentration, skill.Concentration,
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield,
		}
	}

	return skillSequence
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
			s.Logger.Debug(fmt.Sprintf("Izual is too far away (%d), moving closer.", distance))
			step.MoveTo(izual.Position, step.WithIgnoreMonsters())
			continue
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
				numOfAttacks = 1
			}
			step.PrimaryAttack(izual.UnitID, numOfAttacks, false, step.Distance(1, 3), step.EnsureAura(skill.HolyFire))
		}
	}
}

func (s PaladinLeveling) KillDiablo() error {
	s.Logger.Info("Starting Diablo kill sequence...")
	timeout := time.Second * 120
	startTime := time.Now()

	for {
		diablo, found := s.Data.Monsters.FindOne(npc.Diablo, data.MonsterTypeUnique)

		if !found {
			if time.Since(startTime) > timeout {
				s.Logger.Error("Diablo was not found, timeout reached.")
				return errors.New("diablo not found within the time limit")
			}
			time.Sleep(time.Second / 2)
			continue
		}

		if diablo.Stats[stat.Life] <= 0 {
			s.Logger.Info("Diablo is dead.")
			return nil
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
		step.MoveTo(data.Position{X: 10062, Y: 12639}, step.WithIgnoreMonsters())

		s.killMonster(foundMonster.Name, data.MonsterTypeSuperUnique)

	}

	s.CharacterCfg.BackToTown = originalBackToTownCfg
	s.Logger.Info("Restored original back-to-town checks after Ancients fight.")
	return nil
}

func (s PaladinLeveling) KillBaal() error {

	s.Logger.Info("Starting Baal kill sequence...")
	timeout := time.Second * 600
	startTime := time.Now()

	for {
		baal, found := s.Data.Monsters.FindOne(npc.BaalCrab, data.MonsterTypeUnique)

		if !found {
			if time.Since(startTime) > timeout {
				s.Logger.Error("Baal was not found, timeout reached.")
				return errors.New("Baal not found within the time limit")
			}
			time.Sleep(time.Second / 2)
			continue
		}

		if baal.Stats[stat.Life] <= 0 {
			s.Logger.Info("Baal is dead.")
			return nil
		}

		numOfAttacks := 5
		if s.Data.PlayerUnit.Skills[skill.BlessedHammer].Level > 0 {
			step.PrimaryAttack(baal.UnitID, numOfAttacks, false, step.Distance(2, 7), step.EnsureAura(skill.Concentration))
			s.Logger.Debug("Performing random movement to reposition.")
			s.PathFinder.RandomMovement()
			time.Sleep(time.Millisecond * 250)
		} else {
			if s.Data.PlayerUnit.Skills[skill.Zeal].Level > 0 {
				numOfAttacks = 1
			}
			step.PrimaryAttack(baal.UnitID, numOfAttacks, false, step.Distance(1, 3), step.EnsureAura(skill.HolyFire))
		}
	}

}

func (s PaladinLeveling) GetAdditionalRunewords() []string {
	additionalRunewords := action.GetCastersCommonRunewords()
	additionalRunewords = append(additionalRunewords, "Steel")
	return additionalRunewords
}
