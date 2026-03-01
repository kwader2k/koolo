package character

import (
	"fmt"
	"log/slog"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

var _ context.LevelingCharacter = (*WarlockLeveling)(nil)

const (
	warlockMaxAttacksLoop = 1
	warlockMinDistance    = 1
	warlockMaxDistance    = 4
	warlockDangerDistance = 1
	warlockSafeDistance   = 3
)

type WarlockLeveling struct {
	BaseCharacter
}

func (s WarlockLeveling) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s WarlockLeveling) CheckKeyBindings() []skill.ID {
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

func (s WarlockLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	var lastReposition time.Time
	var lastBuff time.Time
	var lastSumon time.Time
	for {
		context.Get().PauseIfNotPriority()

		if s.Context.Data.PlayerUnit.IsDead() {
			return nil
		}

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

		if completedAttackLoops >= warlockMaxAttacksLoop {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			s.Logger.Info("Monster not found", slog.String("monster", fmt.Sprintf("%v", monster)))
			return nil
		}

		lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
		mana, _ := s.Data.PlayerUnit.FindStat(stat.Mana, 0)
		onCooldown := s.Data.PlayerUnit.States.HasState(state.Cooldown)

		canReposition := lvl.Value > 12 && time.Since(lastReposition) > time.Second*3
		if canReposition {
			isAnyEnemyNearby, _ := action.IsAnyEnemyAroundPlayer(warlockDangerDistance)
			if isAnyEnemyNearby {
				if safePos, found := action.FindSafePosition(monster, warlockDangerDistance, warlockSafeDistance, warlockMinDistance, warlockMaxDistance); found {
					step.MoveTo(safePos, step.WithStationaryDistance(warlockMinDistance, warlockMaxDistance))
					lastReposition = time.Now()
				}
			}
		}

		if s.Data.PlayerUnit.Skills[skill.Cleave].Level > 0 && mana.Value > 2 {
			step.SelectLeftSkill(skill.Cleave)
		} else {
			step.SelectLeftSkill(skill.AttackSkill)
		}

		step.PrimaryAttack(id, 1, true, step.Distance(warlockMinDistance, warlockMaxDistance))

		if !onCooldown {
			if time.Since(lastBuff) > time.Second*4 {
				s.BuffSkills()
				lastBuff = time.Now()
			}
			if time.Since(lastSumon) > time.Second*10 {
				s.PreCTABuffSkills()
				lastSumon = time.Now()
			}
		}

		completedAttackLoops++
		previousUnitID = int(id)
		utils.Sleep(100, 100)
	}
}

func (s WarlockLeveling) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}
		return m.UnitID, true
	}, nil)
}

func (s WarlockLeveling) BuffSkills() []skill.ID {
	return []skill.ID{skill.HexBane, skill.HexPurge, skill.SigilDeath, skill.Consume}
}

func (s WarlockLeveling) PreCTABuffSkills() []skill.ID {
	// TODO: Summons temporarily disabled
	return []skill.ID{skill.SummonGoatman, skill.SummonTainted, skill.SummonDefiler, skill.Engorge}
}

func (s WarlockLeveling) ShouldResetSkills() bool {
	return false
}

func (s WarlockLeveling) SkillsToBind() (skill.ID, []skill.ID) {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	mainSkill := skill.AttackSkill
	skillBindings := []skill.ID{}

	if HexBane, found := s.Data.PlayerUnit.Skills[skill.HexBane]; found && HexBane.Level > 0 {
		skillBindings = append(skillBindings, skill.HexBane)
	}

	if SummonGoatman, found := s.Data.PlayerUnit.Skills[skill.SummonGoatman]; found && SummonGoatman.Level > 0 {
		skillBindings = append(skillBindings, skill.SummonGoatman)
	}

	if lvl.Value >= 6 {
		skillBindings = append(skillBindings, skill.Cleave)
		skillBindings = append(skillBindings, skill.SigilLethargy)
		mainSkill = skill.Cleave
	}

	if lvl.Value >= 12 {
		mainSkill = skill.Cleave
		skillBindings = append(skillBindings, skill.HexPurge)
		skillBindings = append(skillBindings, skill.SummonTainted)
		skillBindings = append(skillBindings, skill.SigilRancor)
	}

	if lvl.Value >= 18 {
		mainSkill = skill.Cleave
		skillBindings = append(skillBindings, skill.SummonDefiler)
		skillBindings = append(skillBindings, skill.SigilDeath)
	}

	if lvl.Value >= 24 {
		mainSkill = skill.Cleave
		skillBindings = []skill.ID{}
		skillBindings = append(skillBindings, skill.SigilDeath)
		skillBindings = append(skillBindings, skill.HexPurge)
		skillBindings = append(skillBindings, skill.SummonDefiler)
	}

	if lvl.Value >= 30 {
		mainSkill = skill.Cleave
		skillBindings = []skill.ID{}
		skillBindings = append(skillBindings, skill.SigilDeath)
		skillBindings = append(skillBindings, skill.HexPurge)
		skillBindings = append(skillBindings, skill.SummonDefiler)
		skillBindings = append(skillBindings, skill.MirroredBlades)
		skillBindings = append(skillBindings, skill.BindDemon)
		skillBindings = append(skillBindings, skill.Consume)
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

func (s WarlockLeveling) StatPoints() []context.StatAllocation {
	stats := []context.StatAllocation{
		{Stat: stat.Vitality, Points: 30},
		{Stat: stat.Energy, Points: 35},
		{Stat: stat.Vitality, Points: 45},
		{Stat: stat.Strength, Points: 30},
		{Stat: stat.Vitality, Points: 85},
		{Stat: stat.Strength, Points: 35},
		{Stat: stat.Vitality, Points: 90},
		{Stat: stat.Strength, Points: 40},
		{Stat: stat.Vitality, Points: 999},
	}
	s.Logger.Debug("Stat point allocation plan", "stats", stats)
	return stats
}

func (s WarlockLeveling) SkillPoints() []skill.ID {

	var skillSequence []skill.ID

	skillSequence = []skill.ID{
		// Levels 2-5: MiasmaBolt
		skill.HexBane, skill.Levitate, skill.SummonGoatman, skill.DemonicMastery, skill.HexBane, // Den of Evil
		skill.Cleave, skill.Cleave, skill.SigilLethargy, skill.BloodOath, skill.DeathMark,
		skill.Cleave, skill.Cleave, // lv12
		skill.SigilRancor, skill.EchoingStrike, skill.HexPurge, skill.SummonTainted, skill.Cleave, //17
		// Level 18:
		skill.BladeWarp, skill.PsychicWard, skill.BloodBoil, skill.SummonDefiler, skill.HexPurge, //22
		skill.HexPurge, skill.HexPurge, //24
		skill.EldritchBlast, skill.Engorge, skill.BloodOath, skill.BloodOath, skill.BloodOath,
		//30
		skill.MirroredBlades, skill.Consume, skill.BindDemon, //32
		skill.Cleave, skill.Cleave, skill.Cleave, skill.Cleave, skill.Cleave, //37
		skill.Cleave, skill.Cleave, skill.Cleave, skill.Cleave, skill.Cleave, //42
		skill.Cleave, skill.Cleave, skill.Cleave, skill.Cleave, skill.Cleave, //47
		skill.HexPurge, skill.HexPurge, skill.HexPurge, skill.HexPurge, skill.HexPurge, //52
		skill.HexPurge, skill.HexPurge, skill.HexPurge, skill.HexPurge, skill.HexPurge, //57
		skill.HexPurge, skill.HexPurge, skill.HexPurge, skill.HexPurge, skill.HexPurge, //62
		skill.DemonicMastery, skill.DemonicMastery, skill.DemonicMastery, skill.DemonicMastery, skill.DemonicMastery, //67
		skill.MirroredBlades, skill.MirroredBlades, skill.MirroredBlades, skill.MirroredBlades, skill.MirroredBlades, //72
		skill.MirroredBlades, skill.MirroredBlades, skill.MirroredBlades, skill.MirroredBlades, skill.MirroredBlades, //77
		skill.MirroredBlades, skill.MirroredBlades, skill.MirroredBlades, skill.MirroredBlades, skill.MirroredBlades, //82
		skill.SigilDeath, skill.SigilDeath, skill.SigilDeath, skill.SigilDeath, skill.SigilDeath, //87
	}

	return skillSequence
}

func (s WarlockLeveling) killBoss(bossNPC npc.ID, timeout time.Duration) error {
	s.Logger.Info(fmt.Sprintf("Starting kill sequence for %v...", bossNPC))
	startTime := time.Now()
	var lastBuff time.Time
	var lastSumon time.Time
	for {
		context.Get().PauseIfNotPriority()

		if time.Since(startTime) > timeout {
			s.Logger.Error(fmt.Sprintf("Timed out waiting for %v.", bossNPC))
			return fmt.Errorf("%v timeout", bossNPC)
		}

		if s.Context.Data.PlayerUnit.IsDead() {
			s.Logger.Info("Player detected as dead, stopping boss kill sequence.")
			return nil
		}

		boss, found := s.Data.Monsters.FindOne(bossNPC, data.MonsterTypeUnique)
		if !found {
			utils.Sleep(1000, 500)
			continue
		}

		if boss.Stats[stat.Life] <= 0 {
			s.Logger.Info(fmt.Sprintf("%v has been defeated.", bossNPC))
			if bossNPC == npc.BaalCrab {
				s.Logger.Info("Waiting...")
				utils.Sleep(1000, 500)
			}
			return nil
		}

		mana, _ := s.Data.PlayerUnit.FindStat(stat.Mana, 0)
		onCooldown := s.Data.PlayerUnit.States.HasState(state.Cooldown)

		if s.Data.PlayerUnit.Skills[skill.Cleave].Level > 0 && mana.Value > 2 {
			step.SelectLeftSkill(skill.Cleave)
		} else {
			step.SelectLeftSkill(skill.AttackSkill)
		}

		step.PrimaryAttack(boss.UnitID, 1, true, step.Distance(warlockMinDistance, warlockMaxDistance))

		if !onCooldown {
			if time.Since(lastBuff) > time.Second*4 {
				s.BuffSkills()
				lastBuff = time.Now()
			}
			if time.Since(lastSumon) > time.Second*10 {
				s.PreCTABuffSkills()
				lastSumon = time.Now()
			}
		}

		utils.Sleep(100, 100)
	}
}

func (s WarlockLeveling) killMonsterByName(id npc.ID, monsterType data.MonsterType, skipOnImmunities []stat.Resist) error {
	s.Logger.Info(fmt.Sprintf("Starting persistent kill sequence for %v...", id))

	for {
		monster, found := s.Data.Monsters.FindOne(id, monsterType)
		if !found {
			s.Logger.Info(fmt.Sprintf("%v not found, assuming dead.", id))
			return nil
		}

		if monster.Stats[stat.Life] <= 0 {
			s.Logger.Info(fmt.Sprintf("%v is dead.", id))
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
			s.Logger.Warn(fmt.Sprintf("Error during KillMonsterSequence for %v: %v", id, err))
		}

		utils.Sleep(250, 100)
	}
}

func (s WarlockLeveling) KillCountess() error {
	return s.killMonsterByName(npc.DarkStalker, data.MonsterTypeSuperUnique, nil)
}

func (s WarlockLeveling) KillAndariel() error {
	return s.killBoss(npc.Andariel, time.Second*220)
}

func (s WarlockLeveling) KillSummoner() error {
	return s.killMonsterByName(npc.Summoner, data.MonsterTypeUnique, nil)
}

func (s WarlockLeveling) KillDuriel() error {
	return s.killBoss(npc.Duriel, time.Second*220)
}

func (s WarlockLeveling) KillCouncil() error {
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

		if len(councilMembers) > 0 {
			return councilMembers[0].UnitID, true
		}

		return 0, false
	}, nil)
}

func (s WarlockLeveling) KillMephisto() error {
	return s.killBoss(npc.Mephisto, time.Second*220)
}

func (s WarlockLeveling) KillIzual() error {
	return s.killBoss(npc.Izual, time.Second*220)
}

func (s WarlockLeveling) KillDiablo() error {
	return s.killBoss(npc.Diablo, time.Second*220)
}

func (s WarlockLeveling) KillPindle() error {
	return s.killMonsterByName(npc.DefiledWarrior, data.MonsterTypeSuperUnique, nil)
}

func (s WarlockLeveling) KillAncients() error {
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

func (s WarlockLeveling) KillNihlathak() error {
	return s.killMonsterByName(npc.Nihlathak, data.MonsterTypeSuperUnique, nil)
}

func (s WarlockLeveling) KillBaal() error {
	return s.killBoss(npc.BaalCrab, time.Second*240)
}

func (s WarlockLeveling) GetAdditionalRunewords() []string {
	return action.GetCastersCommonRunewords()
}

func (s WarlockLeveling) InitialCharacterConfigSetup() {
}

func (s WarlockLeveling) AdjustCharacterConfig() {
}
