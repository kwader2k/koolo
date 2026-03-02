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

var _ context.LevelingCharacter = (*WarlockCleaveLeveling)(nil)

const (
	cleaveMaxAttacksLoop = 3
	cleaveMinDistance    = 1
	cleaveMaxDistance    = 4
	cleaveDangerDistance = 2
	cleaveSafeDistance   = 3
)

type WarlockCleaveLeveling struct {
	BaseCharacter
}

func (s WarlockCleaveLeveling) ShouldIgnoreMonster(m data.Monster) bool {
	return !m.IsPet()
}

func (s WarlockCleaveLeveling) CheckKeyBindings() []skill.ID {
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

func (s WarlockCleaveLeveling) IsMandatoryKill(m data.Monster) bool {
	switch m.Name {
	case npc.Andariel:
	case npc.Duriel:
	case npc.Mephisto:
	case npc.Diablo:
	case npc.BaalCrab:
	case npc.Summoner:
	case npc.CouncilMember:
	case npc.CouncilMember2:
	case npc.CouncilMember3:
		return true
	}
	return false
}

func (s WarlockCleaveLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	ctx := context.Get()
	completedAttackLoops := 0
	previousUnitID := 0
	var currentTargetID data.UnitID
	var lastReposition time.Time
	lastHealthPercent := 100
	for {
		context.Get().PauseIfNotPriority()

		if s.Context.Data.PlayerUnit.IsDead() {
			return nil
		}

		ctx.PauseIfNotPriority() // Pause if not the priority task

		if currentTargetID == 0 { // Select a new target if none exists
			id, found := monsterSelector(*s.Data)
			if !found {
				utils.Sleep(100, 100)
				return nil // Exit if no target found
			}

			currentTargetID = id
			completedAttackLoops = 0
		}

		monster, found := s.Data.Monsters.FindByID(currentTargetID)
		if !found || monster.Stats[stat.Life] <= 0 || monster.IsPet() {
			//s.Logger.Info("Monster not found", slog.String("monster", fmt.Sprintf("%v", monster)))
			currentTargetID = 0
			utils.Sleep(100, 100)
			return nil
		}

		if previousUnitID != int(currentTargetID) {
			completedAttackLoops = 0
		}

		if completedAttackLoops >= cleaveMaxAttacksLoop {
			return nil
		}

		if !s.preBattleChecks(currentTargetID, skipOnImmunities) {
			return nil
		}

		//lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
		mana, _ := s.Data.PlayerUnit.FindStat(stat.Mana, 0)
		healthPercent := s.Data.PlayerUnit.HPPercent()
		canReposition := lastHealthPercent-healthPercent > 10 && time.Since(lastReposition) > time.Second*1
		if canReposition {
			if safePos, found := action.FindSafePosition(monster, cleaveDangerDistance, cleaveSafeDistance, cleaveMinDistance, cleaveMaxDistance); found {
				step.MoveTo(safePos, step.WithStationaryDistance(cleaveMinDistance, cleaveMaxDistance))
				lastReposition = time.Now()
			}
		}
		lastHealthPercent = healthPercent

		s.CombatSupportSkills(monster) // Refresh buffs before attacking

		if s.Data.PlayerUnit.Skills[skill.Cleave].Level > 0 && mana.Value > 10 {
			step.SelectLeftSkill(skill.Cleave)
		} else {
			step.SelectLeftSkill(skill.AttackSkill)
		}

		step.PrimaryAttack(currentTargetID, 1, true, step.Distance(cleaveMinDistance, cleaveMaxDistance))

		completedAttackLoops++
		previousUnitID = int(currentTargetID)
		utils.Sleep(100, 100)
	}
}

func (s WarlockCleaveLeveling) HexSkills() []skill.ID {
	return []skill.ID{skill.HexBane, skill.HexPurge, skill.HexSiphon}
}

func (s WarlockCleaveLeveling) HexStates() []state.State {
	return []state.State{state.Hexbane, state.Hexpurge, state.Hexsiphon}
}

func (s WarlockCleaveLeveling) SigilSkills() []skill.ID {
	return []skill.ID{skill.SigilDeath, skill.SigilRancor, skill.SigilLethargy}
}

// func (s WarlockCleaveLeveling) SigilStates() []state.State {
// 	return []state.State{state.Sigildeath, state.Sigillethargy, state.Sigilrancor}
// }

func (s WarlockCleaveLeveling) SumonSkills() []skill.ID {
	return []skill.ID{skill.SummonDefiler, skill.SummonTainted, skill.SummonGoatman}
}

// func (s WarlockCleaveLeveling) SumonStates() []state.State {
// 	return []state.State{state.Consume, state.DeathMark, state.Engorge}
// }

func (s WarlockCleaveLeveling) CombatSupportSkills(monster data.Monster) {

	if monster.UnitID == 0 {
		return
	}

	ctx := context.Get()
	var lastDemonBind time.Time
	var lastSigil time.Time

	skills := []skill.ID{}

	isMandatoryKill := s.IsMandatoryKill(monster)

	if time.Since(lastSigil) > time.Second*5 {
		for _, sigil := range s.SigilSkills() {
			if s.Data.PlayerUnit.Skills[sigil].Level > 0 {
				skills = append(skills, sigil)
				lastSigil = time.Now()
				break
			}
		}
	}

	deathMark := true
	if ctx.Data.PlayerUnit.Skills[skill.DeathMark].Level == 0 || !isMandatoryKill {
		deathMark = false
	}
	if deathMark {
		skills = append(skills, skill.DeathMark)
	}

	monsterHPPercent := float32(monster.Stats[stat.Life]) / float32(monster.Stats[stat.MaxLife]) * 100
	demonbind := true
	if ctx.Data.PlayerUnit.Skills[skill.BindDemon].Level == 0 || isMandatoryKill || monsterHPPercent > 60 {
		demonbind = false
	}
	if demonbind && time.Since(lastDemonBind) > time.Second*10 {
		skills = append(skills, skill.BindDemon)
		lastDemonBind = time.Now()
	}

	for _, sk := range skills {
		step.SecondaryAttack(sk, monster.UnitID, 1, step.Distance(cleaveMinDistance, cleaveMaxDistance)) // Activate skill
		utils.Sleep(180, 100)                                                                            // Small delay
	}

	//utils.Sleep(100, 100)
}

func (s WarlockCleaveLeveling) BuffSkills() []skill.ID {
	buffs := make([]skill.ID, 0)
	return buffs
}

// Dynamically determines pre-combat buffs and summons
func (s WarlockCleaveLeveling) PreCTABuffSkills() []skill.ID {
	ctx := context.Get()
	skills := make([]skill.ID, 0)
	var lastCast time.Time

	var HexStates = s.HexStates()
	for i, hex := range s.HexSkills() {
		if s.Data.PlayerUnit.Skills[hex].Level > 0 {
			if !ctx.Data.PlayerUnit.States.HasState(HexStates[i]) {
				skills = append(skills, hex)
			}
		}
	}

	var sumonSkillId = skill.AttackSkill
	for _, sumonSkill := range s.SumonSkills() {
		if s.Data.PlayerUnit.Skills[sumonSkill].Level > 0 {
			sumonSkillId = sumonSkill
			break
		}
	}

	needpet := false
	engorge := false
	consume := false
	if sumonSkillId != skill.AttackSkill {
		for _, monster := range s.Data.Monsters { // Check existing pets
			if !monster.IsPet() {
				continue
			}

			petHPPercent := float32(monster.Stats[stat.Life]) / float32(monster.Stats[stat.MaxLife]) * 100
			if petHPPercent < 60 {
				engorge = true
			}
			if petHPPercent < 30 {
				consume = true
			}

			switch sumonSkillId {
			case skill.SummonGoatman:
				if monster.Name != npc.WarGoatman {
					needpet = true
				}
			case skill.SummonTainted:
				if monster.Name != npc.Tainted {
					needpet = true
				}
			case skill.SummonDefiler:
				if monster.Name != npc.WarDefiler {
					needpet = true
				}
			}
		}
	}

	if needpet {
		skills = append(skills, sumonSkillId)
	}

	if time.Since(lastCast) < time.Second*5 {
		if engorge {
			if s.Data.PlayerUnit.Skills[skill.Engorge].Level > 0 {
				skills = append(skills, skill.Engorge)
				lastCast = time.Now()
			}
		}

		if consume {
			if s.Data.PlayerUnit.Skills[skill.Consume].Level > 0 {
				skills = append(skills, skill.Consume)
				lastCast = time.Now()
			}
		}
	}
	return skills
}

func (s WarlockCleaveLeveling) ShouldResetSkills() bool {
	return false
}

func (s WarlockCleaveLeveling) SkillsToBind() (skill.ID, []skill.ID) {
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

func (s WarlockCleaveLeveling) StatPoints() []context.StatAllocation {
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

func (s WarlockCleaveLeveling) SkillPoints() []skill.ID {

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

func (s WarlockCleaveLeveling) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}
		return m.UnitID, true
	}, nil)
}

func (s WarlockCleaveLeveling) killBoss(bossNPC npc.ID, timeout time.Duration) error {
	s.Logger.Info(fmt.Sprintf("Starting kill sequence for %v...", bossNPC))
	startTime := time.Now()
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
			utils.Sleep(500, 500)
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

		return s.killMonster(bossNPC, data.MonsterTypeUnique)
	}
}

func (s WarlockCleaveLeveling) KillCountess() error {
	return s.killMonster(npc.DarkStalker, data.MonsterTypeSuperUnique)
}

func (s WarlockCleaveLeveling) KillAndariel() error {
	return s.killBoss(npc.Andariel, time.Second*220)
}

func (s WarlockCleaveLeveling) KillSummoner() error {
	return s.killMonster(npc.Summoner, data.MonsterTypeUnique)
}

func (s WarlockCleaveLeveling) KillDuriel() error {
	return s.killBoss(npc.Duriel, time.Second*220)
}

func (s WarlockCleaveLeveling) KillCouncil() error {
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

func (s WarlockCleaveLeveling) KillMephisto() error {
	return s.killBoss(npc.Mephisto, time.Second*220)
}

func (s WarlockCleaveLeveling) KillIzual() error {
	return s.killBoss(npc.Izual, time.Second*220)
}

func (s WarlockCleaveLeveling) KillDiablo() error {
	return s.killBoss(npc.Diablo, time.Second*220)
}

func (s WarlockCleaveLeveling) KillPindle() error {
	return s.killMonster(npc.DefiledWarrior, data.MonsterTypeSuperUnique)
}

func (s WarlockCleaveLeveling) KillAncients() error {
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

func (s WarlockCleaveLeveling) KillNihlathak() error {
	return s.killMonster(npc.Nihlathak, data.MonsterTypeSuperUnique)
}

func (s WarlockCleaveLeveling) KillBaal() error {
	return s.killBoss(npc.BaalCrab, time.Second*240)
}

func (s WarlockCleaveLeveling) GetAdditionalRunewords() []string {
	return action.GetCastersCommonRunewords()
}

func (s WarlockCleaveLeveling) InitialCharacterConfigSetup() {
}

func (s WarlockCleaveLeveling) AdjustCharacterConfig() {
}
