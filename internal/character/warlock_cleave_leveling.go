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
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/utils"
)

var _ context.LevelingCharacter = (*WarlockCleaveLeveling)(nil)

const (
	eStrikeMaxAttacksLoop = 5
	eStrikeMinDistance    = 8
	eStrikeMaxDistance    = 12
	eStrikeDangerDistance = 6
	eStrikeSafeDistance   = 10
)

type WarlockCleaveLeveling struct {
	BaseCharacter
	combatState CleaveCombatState
}

func (s WarlockCleaveLeveling) ShouldIgnoreMonster(m data.Monster) bool {
	return m.IsPet()
}

func (s WarlockCleaveLeveling) CheckKeyBindings() []skill.ID {
	_, requireKeybindings := s.SkillsToBind()
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

type CleaveCombatState struct {
	nextSigil     time.Time
	nextDemonBind time.Time
	nextDeathMark time.Time
	nextPetSkill  time.Time
	nextBuff      time.Time
}

func (s *WarlockCleaveLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	ctx := context.Get()
	completedAttackLoops := 0
	var currentTargetID data.UnitID
	var lastReposition time.Time
	lastHealthPercent := 100

	for {

		ctx.PauseIfNotPriority() // Pause if not the priority task

		if s.Context.Data.PlayerUnit.IsDead() {
			return nil
		}

		if currentTargetID == 0 { // Select a new target if none exists
			id, found := monsterSelector(*s.Data)
			if !found {
				utils.Sleep(50, 50)
				return nil // Exit if no target found
			}

			currentTargetID = id
			completedAttackLoops = 0
		}

		monster, found := s.Data.Monsters.FindByID(currentTargetID)
		if !found || monster.Stats[stat.Life] <= 0 || monster.IsPet() {
			//s.Logger.Info("Monster not found", slog.String("monster", fmt.Sprintf("%v", monster)))
			currentTargetID = 0
			completedAttackLoops = 0
			//utils.Sleep(100, 100)
			return nil
		}

		if completedAttackLoops >= eStrikeMaxAttacksLoop {
			completedAttackLoops = 0
			return nil
		}

		if !s.preBattleChecks(currentTargetID, skipOnImmunities) {
			return nil
		}

		//lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
		healthPercent := s.Data.PlayerUnit.HPPercent()
		canReposition := lastHealthPercent-healthPercent > 10 && time.Since(lastReposition) > utils.RandomDurationMs(1000, 2000)
		if canReposition {
			if safePos, found := action.FindSafePosition(monster, eStrikeDangerDistance, eStrikeSafeDistance, eStrikeMinDistance, eStrikeMaxDistance); found {
				lastReposition = time.Now()

				if s.CheckMana(skill.BladeWarp) && s.Data.PlayerUnit.Skills[skill.BladeWarp].Level > 0 {
					step.CastAtPosition(skill.BladeWarp, true, safePos)
				} else {
					step.MoveTo(safePos, step.WithStationaryDistance(eStrikeMinDistance, eStrikeMaxDistance))
				}
			}
		}
		lastHealthPercent = healthPercent

		s.MainAttack(currentTargetID)

		completedAttackLoops++
		utils.Sleep(20, 50)
		s.CombatSupportSkills(monster) // summon
	}
}

func (s WarlockCleaveLeveling) MainAttack(currentTargetID data.UnitID) {
	ctx := context.Get()

	if s.Data.PlayerUnit.Skills[skill.EchoingStrike].Level > 0 && s.CheckMana(skill.EchoingStrike) {
		if s.Data.PlayerUnit.RightSkill == skill.EchoingStrike {
			step.SecondaryAttack(skill.EchoingStrike, currentTargetID, 1, step.RangedDistance(eStrikeMinDistance, eStrikeMaxDistance))
			return
		}
		EchoingStrike, found := s.Data.KeyBindings.KeyBindingForSkill(skill.EchoingStrike)
		if found {
			ctx.HID.PressKeyBinding(EchoingStrike)
			return
		}
	}

	if s.Data.PlayerUnit.Skills[skill.Cleave].Level > 0 && s.CheckMana(skill.Cleave) {
		if s.Data.PlayerUnit.RightSkill == skill.Cleave {
			step.SecondaryAttack(skill.Cleave, currentTargetID, 1, step.Distance(1, 4))
			return
		}
		Cleave, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Cleave)
		if found {
			ctx.HID.PressKeyBinding(Cleave)
			return
		}
	}

	if action.GetSkillTotalLevel(skill.MiasmaBolt) > 0 && s.CheckMana(skill.MiasmaBolt) {
		if s.Data.PlayerUnit.RightSkill == skill.MiasmaBolt {
			step.SecondaryAttack(skill.MiasmaBolt, currentTargetID, 1, step.RangedDistance(eStrikeMinDistance, eStrikeMaxDistance))
			return
		}
		MiasmaBolt, found := s.Data.KeyBindings.KeyBindingForSkill(skill.MiasmaBolt)
		if found {
			ctx.HID.PressKeyBinding(MiasmaBolt)
			return
		}
	}

	step.PrimaryAttack(currentTargetID, 1, true, step.Distance(1, 3))
}

func (s WarlockCleaveLeveling) HexSkills() []skill.ID {
	return []skill.ID{skill.HexSiphon, skill.HexPurge, skill.HexBane}
}

func (s WarlockCleaveLeveling) HexStates() []state.State {
	return []state.State{state.Hexsiphon, state.Hexpurge, state.Hexbane}
}

func (s WarlockCleaveLeveling) SigilSkills() []skill.ID {
	return []skill.ID{skill.SigilDeath, skill.SigilRancor, skill.SigilLethargy}
}

// func (s WarlockCleaveLeveling) SigilStates() []state.State {
// 	return []state.State{state.Sigildeath, state.Sigilrancor, state.Sigillethargy}
// }

func (s WarlockCleaveLeveling) SumonSkills() []skill.ID {
	return []skill.ID{skill.SummonDefiler, skill.SummonTainted, skill.SummonGoatman}
}

// func (s WarlockCleaveLeveling) SumonStates() []state.State {
// 	return []state.State{state.Consume, state.DeathMark, state.Engorge}
// }

func (s WarlockCleaveLeveling) CheckMana(skillId skill.ID) bool {
	mana, _ := s.Data.PlayerUnit.FindStat(stat.Mana, 0)
	level := int(action.GetSkillTotalLevel(skillId))
	manaRequire := 0
	switch skillId {
	case skill.MiasmaBolt:
		manaRequire = int(level/2 + 2)
	case skill.SigilLethargy:
		manaRequire = 4
	case skill.SigilRancor:
		manaRequire = 12
	case skill.SigilDeath:
		manaRequire = level/2 + 10
	case skill.HexBane:
		manaRequire = level/2 + 6
	case skill.HexPurge:
		manaRequire = level + 25
	case skill.HexSiphon:
		manaRequire = level/2 + 13
	case skill.Cleave:
		manaRequire = 3
	case skill.EchoingStrike:
		manaRequire = level/2 + 9
	case skill.BladeWarp:
		manaRequire = 15
	case skill.MirroredBlades:
		manaRequire = level/2 + 5
	case skill.PsychicWard:
		manaRequire = level + 20
	case skill.EldritchBlast:
		manaRequire = level/2 + 6
	case skill.SummonGoatman:
		manaRequire = level/2 + 15
	case skill.SummonTainted:
		manaRequire = level/2 + 25
	case skill.SummonDefiler:
		manaRequire = level/2 + 35
	case skill.BindDemon:
		manaRequire = level + 25
	case skill.DeathMark:
		manaRequire = level/3 + 2
	case skill.BloodBoil:
		manaRequire = level/3 + 4
	case skill.Engorge:
		manaRequire = level/3 + 3
	case skill.Consume:
		manaRequire = level + 25
	}
	return mana.Value > manaRequire
}

func (s *WarlockCleaveLeveling) CombatSupportSkills(target data.Monster) {

	if target.UnitID == 0 {
		return
	}

	skills := []skill.ID{}
	isMandatoryKill := s.IsMandatoryKill(target)
	isUnique := target.Type != data.MonsterTypeNone && target.Type != data.MonsterTypeMinion
	targetHPPercent := float32(target.Stats[stat.Life]) / float32(target.Stats[stat.MaxLife]) * 100
	monsterCount := 0
	playerPos := s.Data.PlayerUnit.Position

	var engorgePetId data.UnitID
	var consumePetId data.UnitID
	maxpet := int(action.GetSkillTotalLevel(skill.DemonicMastery)/10 + 1)
	petCount := 0

	for _, monster := range s.Data.Monsters {
		HPPercent := float32(monster.Stats[stat.Life]) / float32(monster.Stats[stat.MaxLife]) * 100
		if monster.IsPet() {
			petCount++
			if HPPercent < 30 {
				consumePetId = monster.UnitID
			} else if HPPercent < 60 {
				engorgePetId = monster.UnitID
			}
		} else if HPPercent > 1 {
			distance := pather.DistanceFromPoint(playerPos, monster.Position)
			if distance < eStrikeMaxDistance {
				monsterCount++
			}
		}
	}

	//support skills
	if time.Now().After(s.combatState.nextSigil) && (isUnique || monsterCount > 2) {
		castSigil := false
		for _, sigil := range s.SigilSkills() {
			if s.Data.PlayerUnit.Skills[sigil].Level > 0 && s.CheckMana(sigil) {
				skills = append(skills, sigil)
				castSigil = true
			}
		}
		if castSigil {
			s.combatState.nextSigil = time.Now().Add(utils.RandomDurationMs(3000, 6000))
		}
	}

	deathMark := true
	if !s.CheckMana(skill.DeathMark) || petCount == 0 ||
		s.Data.PlayerUnit.Skills[skill.DeathMark].Level == 0 ||
		!isUnique || !time.Now().After(s.combatState.nextDeathMark) {
		deathMark = false
	}
	if deathMark {
		skills = append(skills, skill.DeathMark)
		s.combatState.nextDeathMark = time.Now().Add(utils.RandomDurationMs(5000, 6000))
	}

	demonbind := true
	if !s.CheckMana(skill.BindDemon) || s.Data.PlayerUnit.Skills[skill.BindDemon].Level == 0 ||
		isMandatoryKill || targetHPPercent > 60 || petCount >= maxpet {
		demonbind = false
	}
	if petCount > 0 && target.Type == data.MonsterTypeNone {
		demonbind = false
	}
	if demonbind && time.Now().After(s.combatState.nextDemonBind) {
		skills = append(skills, skill.BindDemon)
		s.combatState.nextDemonBind = time.Now().Add(utils.RandomDurationMs(5000, 11000))
	}

	// if s.CheckMana(skill.MirroredBlades) && time.Now().After(s.combatState.nextMirroBlade) {
	// 	skills = append(skills, skill.MirroredBlades)
	// 	s.combatState.nextMirroBlade = time.Now().Add(utils.RandomDurationMs(1000, 2000))
	// }

	//perform
	for _, sk := range skills {
		step.SecondaryAttack(sk, target.UnitID, 1, step.Distance(0, eStrikeMaxDistance)) // Activate skill
		utils.Sleep(100, 100)                                                            // Small delay
	}

	//sumon skills
	if !time.Now().After(s.combatState.nextPetSkill) {
		return
	}
	s.combatState.nextPetSkill = time.Now().Add(utils.RandomDurationMs(4000, 6000))

	if s.Data.PlayerUnit.Skills[skill.SummonGoatman].Level <= 0 {
		return
	}

	if petCount < maxpet {
		for _, sumonSkill := range s.SumonSkills() {
			if petCount < maxpet && s.Data.PlayerUnit.Skills[sumonSkill].Level > 0 && s.CheckMana(sumonSkill) {
				petCount++
				step.CastAtPosition(sumonSkill, true, target.Position)
				utils.Sleep(100, 100)
			}
		}
	}

	if s.CheckMana(skill.Engorge) && engorgePetId != 0 {
		if s.Data.PlayerUnit.Skills[skill.Engorge].Level > 0 {
			step.SecondaryAttack(skill.Engorge, engorgePetId, 1, step.RangedDistance(0, eStrikeMaxDistance))
			utils.Sleep(100, 100)
		}
	}

	if s.CheckMana(skill.Consume) && consumePetId != 0 {
		if s.Data.PlayerUnit.Skills[skill.Consume].Level > 0 {
			step.SecondaryAttack(skill.Consume, consumePetId, 1, step.RangedDistance(0, eStrikeMaxDistance))
			utils.Sleep(100, 100)
		}
	}

	//utils.Sleep(100, 100)
}

func (s WarlockCleaveLeveling) BuffSkills() []skill.ID {
	buffs := make([]skill.ID, 0)
	return buffs
}

// Dynamically determines pre-combat buffs and summons
func (s *WarlockCleaveLeveling) PreCTABuffSkills() []skill.ID {
	skills := make([]skill.ID, 0)

	if s.Data.PlayerUnit.RightSkill == skill.TownPortal ||
		s.Data.PlayerUnit.RightSkill == skill.TownportalOSkill {
		return skills
	}

	if !time.Now().After(s.combatState.nextBuff) {
		return skills
	}

	s.combatState.nextBuff = time.Now().Add(utils.RandomDurationMs(5000, 10000))
	//buffs
	var HexStates = s.HexStates()
	for i, hex := range s.HexSkills() {
		if s.CheckMana(hex) && s.Data.PlayerUnit.Skills[hex].Level > 0 {
			if !s.Data.PlayerUnit.States.HasState(HexStates[i]) {
				skills = append(skills, hex)
				break
			}
		}
	}

	if s.CheckMana(skill.PsychicWard) &&
		s.Data.PlayerUnit.Skills[skill.PsychicWard].Level > 0 {
		skills = append(skills, skill.PsychicWard)
	}

	if s.CheckMana(skill.EldritchBlast) &&
		s.Data.PlayerUnit.Skills[skill.EldritchBlast].Level > 0 {
		skills = append(skills, skill.EldritchBlast)
	}

	return skills
}

func (s WarlockCleaveLeveling) ShouldResetSkills() bool {
	return false
}

func (s WarlockCleaveLeveling) SkillsToBind() (skill.ID, []skill.ID) {

	mainSkill := skill.AttackSkill
	skillBindings := []skill.ID{}

	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	for _, hex := range s.HexSkills() {
		if action.GetSkillTotalLevel(hex) > 0 {
			skillBindings = append(skillBindings, hex)
			break
		}
	}

	for _, sumonSkill := range s.SumonSkills() {
		if action.GetSkillTotalLevel(sumonSkill) > 0 {
			skillBindings = append(skillBindings, sumonSkill)
		}
	}

	for _, sigil := range s.SigilSkills() {
		if action.GetSkillTotalLevel(sigil) > 0 {
			skillBindings = append(skillBindings, sigil)
		}
	}

	if lvl.Value < 12 {
		if action.GetSkillTotalLevel(skill.MiasmaBolt) > 0 {
			skillBindings = append(skillBindings, skill.MiasmaBolt)
		}

		if action.GetSkillTotalLevel(skill.Cleave) > 0 {
			skillBindings = append(skillBindings, skill.Cleave)
			//mainSkill = skill.Cleave
		}
	}

	if action.GetSkillTotalLevel(skill.EchoingStrike) > 0 {
		skillBindings = append(skillBindings, skill.EchoingStrike)
		//mainSkill = skill.EchoingStrike
	}

	// if MirroredBlades, found := s.Data.PlayerUnit.Skills[skill.MirroredBlades]; found && MirroredBlades.Level > 0 {
	// 	skillBindings = append(skillBindings, skill.MirroredBlades)
	// }

	if action.GetSkillTotalLevel(skill.BindDemon) > 0 {
		skillBindings = append(skillBindings, skill.BindDemon)
	}

	if action.GetSkillTotalLevel(skill.Consume) > 0 {
		skillBindings = append(skillBindings, skill.Consume)
	}

	if action.GetSkillTotalLevel(skill.DeathMark) > 0 {
		skillBindings = append(skillBindings, skill.DeathMark)
	}

	if action.GetSkillTotalLevel(skill.Engorge) > 0 {
		skillBindings = append(skillBindings, skill.Engorge)
	}

	if action.GetSkillTotalLevel(skill.PsychicWard) > 0 {
		skillBindings = append(skillBindings, skill.PsychicWard)
	}

	if action.GetSkillTotalLevel(skill.EldritchBlast) > 0 {
		skillBindings = append(skillBindings, skill.EldritchBlast)
	}

	_, found := s.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	if found {
		skillBindings = append(skillBindings, skill.TomeOfTownPortal)
	}

	if action.GetSkillTotalLevel(skill.BattleCommand) > 0 {
		skillBindings = append(skillBindings, skill.BattleCommand)
	}

	if action.GetSkillTotalLevel(skill.BattleOrders) > 0 {
		skillBindings = append(skillBindings, skill.BattleOrders)
	}

	s.Logger.Info("Skills bound", "mainSkill", mainSkill, "skillBindings", skillBindings)
	return mainSkill, skillBindings
}

func (s WarlockCleaveLeveling) StatPoints() []context.StatAllocation {
	stats := []context.StatAllocation{
		{Stat: stat.Strength, Points: 30},
		{Stat: stat.Vitality, Points: 45},
		{Stat: stat.Strength, Points: 35},
		{Stat: stat.Vitality, Points: 60},
		{Stat: stat.Strength, Points: 40},
		{Stat: stat.Vitality, Points: 999},
	}
	s.Logger.Debug("Stat point allocation plan", "stats", stats)
	return stats
}

func (s WarlockCleaveLeveling) SkillPoints() []skill.ID {

	var skillSequence []skill.ID

	skillSequence = []skill.ID{
		// Levels 2-5:HexBane
		skill.HexBane, skill.SummonGoatman, skill.Levitate, skill.Levitate, skill.DemonicMastery, //5
		//6
		skill.Cleave, skill.BloodOath, skill.DeathMark, skill.SigilLethargy, //9
		skill.HexBane, skill.Levitate, //11
		//12
		skill.EchoingStrike, skill.SummonTainted, //13
		skill.EchoingStrike, skill.Levitate, skill.Levitate, skill.Levitate, //17
		//18
		skill.BladeWarp, skill.PsychicWard, skill.BloodBoil, skill.SummonDefiler, //21
		skill.Levitate, skill.Levitate, //23
		//24
		skill.EldritchBlast, skill.Engorge, //25
		skill.EchoingStrike, skill.Levitate, skill.EchoingStrike, skill.Levitate, //29
		//30
		skill.MirroredBlades, skill.Consume, //31
		skill.EchoingStrike, skill.HexBane, skill.MirroredBlades, skill.EchoingStrike, //35
		skill.EchoingStrike, skill.HexBane, skill.MirroredBlades, skill.HexBane, skill.EchoingStrike, //40
		skill.EchoingStrike, skill.HexBane, skill.MirroredBlades, skill.HexBane, skill.EchoingStrike, //45
		skill.EchoingStrike, skill.HexBane, skill.MirroredBlades, skill.HexBane, skill.EchoingStrike, //50
		skill.EchoingStrike, skill.HexBane, skill.MirroredBlades, skill.HexBane, skill.EchoingStrike, //55
		skill.EchoingStrike, skill.HexBane, skill.MirroredBlades, skill.HexBane, skill.EchoingStrike, //60
		skill.EchoingStrike, skill.HexBane, skill.MirroredBlades, skill.HexBane, skill.EchoingStrike, //65
		skill.EchoingStrike, skill.HexBane, skill.MirroredBlades, skill.HexBane, skill.EchoingStrike, //70
		skill.MirroredBlades, skill.HexBane, skill.MirroredBlades, skill.HexBane, skill.HexBane, //75
		skill.MirroredBlades, skill.Consume, skill.MirroredBlades, skill.Consume, skill.MirroredBlades, //80
		skill.MirroredBlades, skill.Consume, skill.MirroredBlades, skill.Consume, skill.MirroredBlades, //85
		skill.MirroredBlades, skill.Consume, skill.MirroredBlades, skill.Consume, skill.MirroredBlades, //90
		skill.Consume, skill.Consume, skill.Consume, skill.SigilLethargy, //94
		skill.SigilLethargy, skill.SigilLethargy, skill.SigilLethargy, skill.SigilLethargy, skill.SigilLethargy, //99
		skill.SigilLethargy, skill.SigilLethargy, skill.SigilLethargy, //102
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
	additionalRunewords := []string{"Ancients' Pledge", "Lore", "Insight", "Smoke", "Treachery",
		"Call to Arms", "Bulwark", "Hustle", "Stealth", "Spirit", "Heart of the Oak", "Leaf", "Steel",
		"Strength", "Nadir", "Cure", "Rhyme"}

	return additionalRunewords
}

func (s WarlockCleaveLeveling) InitialCharacterConfigSetup() {
	s.AdjustCharacterConfig()
}

func (s WarlockCleaveLeveling) AdjustCharacterConfig() {
	ctx := context.Get()
	ctx.CharacterCfg.Game.UseCainIdentify = true
	ctx.CharacterCfg.Game.InteractWithSuperChests = true
	ctx.CharacterCfg.Game.InteractWithShrines = true
	ctx.CharacterCfg.BackToTown.EquipmentBroken = true
	// ctx.CharacterCfg.BackToTown.NoMpPotions = true
	ctx.CharacterCfg.Character.UseTeleport = true
	ctx.CharacterCfg.Character.StashToShared = true
	// ctx.CharacterCfg.Character.UseMerc = true
}
