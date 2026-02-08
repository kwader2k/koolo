package character

import (
	"fmt"
	"log/slog"
	"slices"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

var _ context.LevelingCharacter = (*BarbLeveling)(nil)

const (
	barbLevelingMaxAttacksLoop = 10
)

type BarbLeveling struct {
	BaseCharacter
}

func (s BarbLeveling) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s BarbLeveling) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{}
	missingKeybindings := []skill.ID{}

	hasHowl := s.Data.PlayerUnit.Skills[skill.Howl].Level > 0
	if s.CharacterCfg.Character.BarbLeveling.UseHowl && hasHowl {
		requireKeybindings = append(requireKeybindings, skill.Howl)
	}

	hasBattleCry := s.Data.PlayerUnit.Skills[skill.BattleCry].Level > 0
	if s.CharacterCfg.Character.BarbLeveling.UseBattleCry && hasBattleCry {
		requireKeybindings = append(requireKeybindings, skill.BattleCry)
	}

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

func (s BarbLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	const priorityMonsterSearchRange = 15
	completedAttackLoops := 0
	previousUnitID := 0

	priorityMonsters := []npc.ID{npc.DarkShaman, npc.FallenShaman, npc.MummyGenerator, npc.BaalSubjectMummy, npc.FetishShaman, npc.CarverShaman}
	blacklistedMonsters := make(map[data.UnitID]bool)

	// Skill cooldowns
	var lastFrenzyCast time.Time
	var lastBattleCryCast time.Time
	var lastHowlCast time.Time
	var lastDoubleSwingCast time.Time
	var lastWarCryCast time.Time
	var lastBerserkCast time.Time
	leapAttackExecuted := false

	for {
		context.Get().PauseIfNotPriority()
		var id data.UnitID
		var found bool

		excludedAreas := []area.ID{area.DenOfEvil, area.BloodMoor, area.ColdPlains, area.StonyField, area.TamoeHighland}
		skipPrioritySearch := slices.Contains(excludedAreas, s.Data.PlayerUnit.Area)

		var closestPriorityMonster data.Monster
		minDistance := -1

		if !skipPrioritySearch {
			for _, monsterNpcID := range priorityMonsters {

				for _, m := range s.Data.Monsters {
					if m.Name == monsterNpcID && m.Stats[stat.Life] > 0 && !blacklistedMonsters[m.UnitID] {
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
			}
		}

		if !found {
			id, found = monsterSelector(*s.Data)
			if found && blacklistedMonsters[id] {
				found = false
			}
		}

		if !found {
			return nil
		}

		if previousUnitID != int(id) {
			completedAttackLoops = 0
			leapAttackExecuted = false
		}

		if !s.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		if completedAttackLoops >= barbLevelingMaxAttacksLoop {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found || monster.Stats[stat.Life] <= 0 {
			return nil
		}

		isBoss := monster.Name == npc.Andariel || monster.Name == npc.Duriel || monster.Name == npc.Mephisto ||
			monster.Name == npc.Diablo || monster.Name == npc.Izual || monster.Name == npc.BaalCrab
		if isBoss {
			hasDualOneHand := s.hasDualOneHand()
			s.executeAttackBoss(id, monster.Name, hasDualOneHand, &lastHowlCast, &lastBattleCryCast, &lastWarCryCast, &lastBerserkCast, &leapAttackExecuted)
			completedAttackLoops++
			previousUnitID = int(id)
			time.Sleep(time.Millisecond * 100)
			continue
		}

		isImmuneToAll := monster.IsImmune(stat.FireImmune) &&
			monster.IsImmune(stat.ColdImmune) &&
			monster.IsImmune(stat.LightImmune) &&
			monster.IsImmune(stat.PoisonImmune) &&
			monster.IsImmune(stat.MagicImmune)

		physicalResist, foundPhysicalResist := monster.Stats[stat.DamageReduced]
		hasHighPhysicalResist := foundPhysicalResist && physicalResist >= 100

		isPhysicalImmune := isImmuneToAll || hasHighPhysicalResist

		hasBerserk := s.hasSkill(skill.Berserk)
		if isPhysicalImmune && !hasBerserk {
			blacklistedMonsters[id] = true
			continue
		}

		playerLevel, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

		if playerLevel.Value <= 5 {
			s.executeAttackUnderLevel6(id, &lastHowlCast)
			completedAttackLoops++
			previousUnitID = int(id)
			time.Sleep(time.Millisecond * 100)
			continue
		}

		hasDualOneHand := s.hasDualOneHand()

		hasWarCry := s.hasSkill(skill.WarCry)

		if isPhysicalImmune && hasBerserk {
			s.executeAttackPhysicalImmune(id, &lastHowlCast, &leapAttackExecuted)
		} else if hasWarCry {
			s.executeAttackWarcry(id, &lastHowlCast, &lastBattleCryCast, &lastWarCryCast, &leapAttackExecuted)
		} else {
			s.executeAttackPreWarcry(id, hasDualOneHand, &lastHowlCast, &lastBattleCryCast, &lastFrenzyCast, &lastDoubleSwingCast, &leapAttackExecuted)
		}

		completedAttackLoops++
		previousUnitID = int(id)
		time.Sleep(time.Millisecond * 100)
	}
}

func (s BarbLeveling) killBoss(bossNPC npc.ID) error {
	s.Logger.Info(fmt.Sprintf("Starting kill sequence for %d...", bossNPC))
	lastHowlCast := time.Time{}
	lastBattleCryCast := time.Time{}
	lastWarCryCast := time.Time{}
	lastBerserkCast := time.Time{}
	leapAttackExecuted := false

	timeout := time.Now().Add(30 * time.Second)
	notFoundCount := 0
	const maxNotFoundChecks = 10

	for {
		context.Get().PauseIfNotPriority()
		if s.Data.PlayerUnit.Area.IsTown() {
			timeout = time.Now().Add(30 * time.Second)
			time.Sleep(time.Second / 2)
			continue
		}

		boss, found := s.Data.Monsters.FindOne(bossNPC, data.MonsterTypeUnique)
		if !found {
			notFoundCount++
			if notFoundCount >= maxNotFoundChecks || time.Now().After(timeout) {
				s.Logger.Info(fmt.Sprintf("Boss %d not found, assuming dead.", bossNPC))
				return nil
			}
			time.Sleep(time.Second / 2)
			continue
		}

		notFoundCount = 0

		if boss.Stats[stat.Life] <= 0 {
			s.Logger.Info(fmt.Sprintf("%d is dead.", bossNPC))
			return nil
		}

		hasDualOneHand := s.hasDualOneHand()

		s.executeAttackBoss(boss.UnitID, bossNPC, hasDualOneHand, &lastHowlCast, &lastBattleCryCast, &lastWarCryCast, &lastBerserkCast, &leapAttackExecuted)

		time.Sleep(time.Millisecond * 100)
	}
}

func (s BarbLeveling) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}

		return m.UnitID, true
	}, nil)
}

func (s BarbLeveling) BuffSkills() []skill.ID {
	skillsList := make([]skill.ID, 0)
	if s.Data.PlayerUnit.Skills[skill.Shout].Level > 0 {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Shout); found {
			skillsList = append(skillsList, skill.Shout)
		}
	}

	if s.Data.PlayerUnit.Skills[skill.BattleOrders].Level > 0 {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.BattleOrders); found {
			skillsList = append(skillsList, skill.BattleOrders)
		}
	}

	if s.Data.PlayerUnit.Skills[skill.BattleCommand].Level > 0 {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.BattleCommand); found {
			skillsList = append(skillsList, skill.BattleCommand)
		}
	}
	return skillsList
}

func (s BarbLeveling) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

func (s BarbLeveling) ShouldResetSkills() bool {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
	if lvl.Value == 31 && s.Data.PlayerUnit.Skills[skill.WarCry].Level == 0 {
		return true
	}

	return false
}

func (s BarbLeveling) SkillsToBind() (skill.ID, []skill.ID) {
	mainSkill := skill.AttackSkill
	skillBindings := []skill.ID{}
	playerLevel := 0
	if lvl, found := s.Data.PlayerUnit.FindStat(stat.Level, 0); found {
		playerLevel = lvl.Value
	}

	onlyUntilLevel30 := playerLevel < 31
	if s.Data.PlayerUnit.Skills[skill.Bash].Level > 0 {
		skillBindings = append(skillBindings, skill.Bash)
	}

	if s.Data.PlayerUnit.Skills[skill.Howl].Level > 0 {
		skillBindings = append(skillBindings, skill.Howl)
	}

	if onlyUntilLevel30 {
		if s.Data.PlayerUnit.Skills[skill.DoubleSwing].Level > 0 {
			skillBindings = append(skillBindings, skill.DoubleSwing)
		}

		if s.Data.PlayerUnit.Skills[skill.Frenzy].Level > 0 {
			skillBindings = append(skillBindings, skill.Frenzy)
		}

		if s.Data.PlayerUnit.Skills[skill.LeapAttack].Level > 0 {
			skillBindings = append(skillBindings, skill.LeapAttack)
		}
	}

	if s.Data.PlayerUnit.Skills[skill.WarCry].Level > 0 {
		skillBindings = append(skillBindings, skill.WarCry)
	}

	if s.Data.PlayerUnit.Skills[skill.BattleCommand].Level > 0 {
		skillBindings = append(skillBindings, skill.BattleCommand)
	}

	if s.Data.PlayerUnit.Skills[skill.BattleOrders].Level > 0 {
		skillBindings = append(skillBindings, skill.BattleOrders)
	}

	if s.Data.PlayerUnit.Skills[skill.BattleCry].Level > 0 {
		skillBindings = append(skillBindings, skill.BattleCry)
	}

	if s.Data.PlayerUnit.Skills[skill.Shout].Level > 0 {
		skillBindings = append(skillBindings, skill.Shout)
	}

	if s.Data.PlayerUnit.Skills[skill.Berserk].Level > 0 {
		skillBindings = append(skillBindings, skill.Berserk)
	}

	_, found := s.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	if found {
		skillBindings = append(skillBindings, skill.TomeOfTownPortal)
	}

	s.Logger.Info("Skills bound", "mainSkill", mainSkill, "skillBindings", skillBindings)
	return mainSkill, skillBindings
}

func (s BarbLeveling) GetReusableKeyBindings(skillsToBind []skill.ID) []data.KeyBinding {
	reusableKB := make([]data.KeyBinding, 0)
	skillsToBindMap := make(map[skill.ID]bool)
	for _, sk := range skillsToBind {
		skillsToBindMap[sk] = true
	}
	return reusableKB
}

func (s BarbLeveling) StatPoints() []context.StatAllocation {

	// Define target totals (including base stats)
	targets := []context.StatAllocation{
		{Stat: stat.Dexterity, Points: 25}, // lvl 3

		{Stat: stat.Vitality, Points: 50}, // lvl 10
		{Stat: stat.Strength, Points: 45}, // lvl 10

		{Stat: stat.Strength, Points: 55},  // lvl 18
		{Stat: stat.Dexterity, Points: 45}, // lvl 18
		{Stat: stat.Vitality, Points: 60},  // lvl 18

	
		{Stat: stat.Vitality, Points: 90}, // lvl 30
		{Stat: stat.Strength, Points: 60},  // lvl 30
		{Stat: stat.Vitality, Points: 115}, // lvl 30
		{Stat: stat.Energy, Points: 30},    // lvl 30

		{Stat: stat.Strength, Points: 95},   // lvl 99
		{Stat: stat.Dexterity, Points: 100}, // lvl 99
		{Stat: stat.Vitality, Points: 999},  // lvl 99
	}

	return targets
}

func (s BarbLeveling) SkillPoints() []skill.ID {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
	var skillSequence []skill.ID
	hasFrenzy := s.Data.PlayerUnit.Skills[skill.Frenzy].Level > 0
	hasWarCry := s.Data.PlayerUnit.Skills[skill.WarCry].Level > 0
	if lvl.Value < 31 || (lvl.Value >= 31 && hasFrenzy && !hasWarCry) {
		skillSequence = []skill.ID{
			// 1 - 6
			skill.Bash, skill.Howl,
			skill.MaceMastery, skill.MaceMastery, skill.MaceMastery,
			skill.DoubleSwing,

			// 7-12
			skill.Shout,
			skill.DoubleSwing, skill.DoubleSwing, skill.DoubleSwing, skill.MaceMastery,
			skill.Taunt,

			// 13-18
			skill.IncreasedStamina, skill.Leap,
			skill.DoubleSwing, skill.DoubleSwing, skill.MaceMastery, skill.LeapAttack,

			// 19-24
			skill.MaceMastery, skill.DoubleThrow,
			skill.BattleCry, skill.IronSkin,
			skill.MaceMastery, skill.MaceMastery,
			skill.BattleOrders,

			// 25-31
			skill.Frenzy, skill.IncreasedSpeed, skill.BattleOrders,
			skill.DoubleSwing, skill.DoubleSwing, skill.DoubleSwing, skill.DoubleSwing,
			skill.BattleOrders,
		}
	} else {
		skillSequence = []skill.ID{
			// Respec at 31
			skill.Howl, skill.Taunt, skill.Shout, skill.BattleCry,
			skill.BattleOrders, skill.BattleCommand, skill.WarCry,

			skill.IncreasedStamina, skill.IncreasedSpeed,
			skill.IronSkin, skill.NaturalResistance,

			skill.Bash, skill.Stun, skill.Concentrate, skill.Berserk,

			skill.WarCry,
			skill.BattleOrders, skill.BattleOrders, skill.BattleOrders, skill.BattleOrders,
			skill.BattleCry, skill.BattleCry, skill.BattleCry, skill.BattleCry,
			skill.BattleCry, skill.BattleCry, skill.BattleCry, skill.BattleCry, skill.BattleCry,
			skill.BattleCry, skill.BattleCry,
			skill.BattleOrders, skill.BattleOrders, skill.BattleOrders,

			skill.WarCry, skill.WarCry, skill.WarCry, skill.WarCry,
			skill.WarCry, skill.WarCry, skill.WarCry, skill.WarCry,

			skill.BattleOrders, skill.BattleOrders, skill.WarCry, skill.WarCry,

			skill.BattleCry, skill.WarCry, skill.BattleCry, skill.WarCry,
			skill.BattleCry, skill.WarCry,
			skill.BattleCry, skill.WarCry, skill.BattleCry, skill.WarCry,
			skill.BattleCry, skill.WarCry, skill.BattleCry, skill.WarCry,
			skill.BattleCry, skill.WarCry,

			skill.BattleOrders, skill.BattleOrders, skill.BattleOrders, skill.BattleOrders, skill.BattleOrders,

			skill.Taunt, skill.Taunt, skill.Taunt, skill.Taunt, skill.Taunt,
			skill.Taunt, skill.Taunt, skill.Taunt, skill.Taunt, skill.Taunt,
			skill.Taunt, skill.Taunt, skill.Taunt, skill.Taunt, skill.Taunt,
			skill.Taunt, skill.Taunt, skill.Taunt, skill.Taunt, skill.Taunt,

			skill.BattleOrders, skill.BattleOrders, skill.BattleOrders, skill.BattleOrders, skill.BattleOrders,
		}
	}

	return skillSequence
}

func (s BarbLeveling) KillCountess() error {
	return s.killMonster(npc.DarkStalker, data.MonsterTypeSuperUnique)
}

func (s BarbLeveling) KillAndariel() error {
	s.equipBossEquipment(npc.Andariel)
	defer s.restoreEquipment()
	return s.killBoss(npc.Andariel)
}

func (s BarbLeveling) KillSummoner() error {
	return s.killMonster(npc.Summoner, data.MonsterTypeUnique)
}

func (s BarbLeveling) KillDuriel() error {
	s.equipBossEquipment(npc.Duriel)
	defer s.restoreEquipment()
	return s.killBoss(npc.Duriel)
}

func (s BarbLeveling) KillCouncil() error {
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
			return councilMembers[0].UnitID, true
		}

		return 0, false
	}, nil)
}

func (s BarbLeveling) KillMephisto() error {
	s.equipBossEquipment(npc.Mephisto)
	defer s.restoreEquipment()
	return s.killBoss(npc.Mephisto)
}

func (s BarbLeveling) KillIzual() error {
	s.equipBossEquipment(npc.Izual)
	defer s.restoreEquipment()
	return s.killBoss(npc.Izual)
}

func (s BarbLeveling) KillDiablo() error {
	s.equipBossEquipment(npc.Diablo)
	defer s.restoreEquipment()
	return s.killBoss(npc.Diablo)
}

func (s BarbLeveling) KillPindle() error {
	return s.killMonster(npc.DefiledWarrior, data.MonsterTypeSuperUnique)
}

func (s BarbLeveling) KillNihlathak() error {
	return s.killMonster(npc.Nihlathak, data.MonsterTypeSuperUnique)
}

func (s BarbLeveling) KillAncients() error {
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

func (s BarbLeveling) KillBaal() error {
	s.equipBossEquipment(npc.BaalCrab)
	defer s.restoreEquipment()
	return s.killBoss(npc.BaalCrab)
}

func (s BarbLeveling) GetAdditionalRunewords() []string {
	additionalRunewords := action.GetCastersCommonRunewords()
	additionalRunewords = append(additionalRunewords, "Steel")
	additionalRunewords = append(additionalRunewords, "Strength")
	additionalRunewords = append(additionalRunewords, "Malice")
	additionalRunewords = append(additionalRunewords, "Black")
	additionalRunewords = append(additionalRunewords, "Rhyme")
	return additionalRunewords
}

func (s BarbLeveling) InitialCharacterConfigSetup() {
	ctx := context.Get()
	hasWarCry := s.Data.PlayerUnit.Skills[skill.WarCry].Level > 0

	if hasWarCry {
		ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "mana"}
		ctx.CharacterCfg.Inventory.HealingPotionCount = 6
		ctx.CharacterCfg.Inventory.ManaPotionCount = 6
	} else {
		ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "healing", "mana"}
		ctx.CharacterCfg.Inventory.HealingPotionCount = 8
		ctx.CharacterCfg.Inventory.ManaPotionCount = 4
	}
	ctx.CharacterCfg.Health.ManaPotionAt = 50
	ctx.CharacterCfg.CubeRecipes.Enabled = false
}

func (s BarbLeveling) AdjustCharacterConfig() {
	ctx := context.Get()
	hasWarCry := s.Data.PlayerUnit.Skills[skill.WarCry].Level > 0

	if hasWarCry {
		ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "mana"}
		ctx.CharacterCfg.Inventory.HealingPotionCount = 6
		ctx.CharacterCfg.Inventory.ManaPotionCount = 6
	} else {
		ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "healing", "mana"}
		ctx.CharacterCfg.Inventory.HealingPotionCount = 8
		ctx.CharacterCfg.Inventory.ManaPotionCount = 4
	}
	ctx.CharacterCfg.Health.ManaPotionAt = 50
	ctx.CharacterCfg.CubeRecipes.Enabled = false
}
