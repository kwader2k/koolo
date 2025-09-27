package character

import (
	"fmt"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

const (
	AmplifyDamageMinDistance   = 18
	AmplifyDamageMaxDistance   = 25
	CorpseExplosionMaxDistance = 12
)

type NecromancerLeveling struct {
	BaseCharacter
	lastAmplifyDamageCast time.Time
}

func (n *NecromancerLeveling) CheckKeyBindings() []skill.ID {
	return []skill.ID{}
}

func (n *NecromancerLeveling) BuffSkills() []skill.ID {
	return []skill.ID{skill.BoneArmor}
}

func (n *NecromancerLeveling) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

func (n *NecromancerLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	id, found := monsterSelector(*n.Data)
	if !found {
		return nil
	}

	if !n.preBattleChecks(id, skipOnImmunities) {
		return nil
	}

	monster, found := n.Data.Monsters.FindByID(id)
	if !found {
		return nil
	}

	lvl, _ := n.Data.PlayerUnit.FindStat(stat.Level, 0)

	// Level 1: Basic attack only
	if lvl.Value < 2 {
		step.PrimaryAttack(monster.UnitID, 1, false, step.Distance(1, 3))
		return nil
	}

	if lvl.Value >= 11 && !monster.States.HasState(state.Amplifydamage) && time.Since(n.lastAmplifyDamageCast) > time.Second*2 {
		step.SecondaryAttack(skill.AmplifyDamage, monster.UnitID, 1, step.Distance(AmplifyDamageMinDistance, AmplifyDamageMaxDistance))
		n.lastAmplifyDamageCast = time.Now()
		return nil
	}

	if lvl.Value >= 17 {
		isCorpseNearby := false
		radiusSquared := float64(5 * 5)
		for _, c := range n.Data.Corpses {
			dx := float64(monster.Position.X - c.Position.X)
			dy := float64(monster.Position.Y - c.Position.Y)
			if (dx*dx + dy*dy) < radiusSquared {
				isCorpseNearby = true
				break
			}
		}

		if isCorpseNearby {
			step.SecondaryAttack(skill.CorpseExplosion, monster.UnitID, 1, step.Distance(0, CorpseExplosionMaxDistance))
			return nil
		}
	}

	boneSpearRange := step.Distance(0, 15)
	if lvl.Value >= 18 {
		step.PrimaryAttack(monster.UnitID, 3, true, boneSpearRange)
	} else {
		step.PrimaryAttack(monster.UnitID, 5, true, boneSpearRange)
	}

	return nil
}

func (n *NecromancerLeveling) ShouldResetSkills() bool {
	lvl, _ := n.Data.PlayerUnit.FindStat(stat.Level, 0)
	return lvl.Value == 48
}

func (n *NecromancerLeveling) SkillsToBind() (skill.ID, []skill.ID) {
	lvl, _ := n.Data.PlayerUnit.FindStat(stat.Level, 0)

	mainSkill := skill.AttackSkill
	skillBindings := []skill.ID{}

	if lvl.Value >= 2 {
		mainSkill = skill.Teeth
	}
	if lvl.Value >= 6 {
		skillBindings = append(skillBindings, skill.ClayGolem)
	}
	if lvl.Value >= 11 {
		skillBindings = append(skillBindings, skill.AmplifyDamage)
	}
	if lvl.Value >= 12 {
		skillBindings = append(skillBindings, skill.IronMaiden)
	}
	if lvl.Value >= 14 {
		skillBindings = append(skillBindings, skill.BoneArmor)
	}
	if lvl.Value >= 17 {
		skillBindings = append(skillBindings, skill.CorpseExplosion)
	}
	if lvl.Value >= 18 {
		mainSkill = skill.BoneSpear
	}
	if lvl.Value >= 26 {
		skillBindings = append(skillBindings, skill.BonePrison)
	}

	if lvl.Value >= 48 {
		mainSkill = skill.BoneSpear
		skillBindings = []skill.ID{
			skill.BoneSpear,
			skill.CorpseExplosion,
			skill.AmplifyDamage,
			skill.BoneArmor,
			skill.ClayGolem,
			skill.BonePrison,
		}
	}

	_, found := n.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	if found {
		skillBindings = append(skillBindings, skill.TomeOfTownPortal)
	}

	return mainSkill, skillBindings
}

func (n *NecromancerLeveling) StatPoints() []context.StatAllocation {
	return []context.StatAllocation{
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
}

func (n *NecromancerLeveling) SkillPoints() []skill.ID {
	lvl, _ := n.Data.PlayerUnit.FindStat(stat.Level, 0)
	var skillSequence []skill.ID

	if lvl.Value < 48 {
		skillSequence = []skill.ID{
			skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, // 2-5
			skill.Teeth, // Den of Evil
			skill.ClayGolem,                  // 6
			skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, // 7-10
			skill.AmplifyDamage, // 11
			skill.IronMaiden,                 // 12
			skill.Teeth,                      // 13
			skill.BoneArmor,                  // 14
			skill.BoneWall,                   // Radament
			skill.CorpseExplosion,            // 15
			skill.BoneWall, skill.BoneWall, // 16-17
			skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, // 18-20
			skill.CorpseExplosion,            // Izual
			skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, // 21-25
			skill.BonePrison, // 26
			skill.ClayGolem, skill.ClayGolem, skill.ClayGolem, // 27-29
			skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, // 30-35
			skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, // 36-40
			skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, // 41-47
		}
	} else {
		// Level 48+ Respec
		skillSequence = []skill.ID{
			// Prerequisites and main skills
			skill.ClayGolem, skill.Teeth, skill.CorpseExplosion, skill.BoneSpear, skill.BoneArmor, skill.BoneWall, skill.BonePrison, skill.AmplifyDamage,
			// Main skill allocation
			skill.ClayGolem, skill.ClayGolem, skill.ClayGolem, skill.ClayGolem,
			skill.GolemMastery, skill.GolemMastery, skill.GolemMastery,
			skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion,
			skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear,
			skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison,
			// Maxing out skills
			skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, // Max Bone Prison
			skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, // Max Bone Wall
			skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, // Max Teeth
			skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, // Max Bone Spirit
		}
	}

	questSkillPoints := 0
	if n.Data.Quests[quest.Act1DenOfEvil].Completed() {
		questSkillPoints++
	}
	if n.Data.Quests[quest.Act2RadamentsLair].Completed() {
		questSkillPoints++
	}
	if n.Data.Quests[quest.Act4TheFallenAngel].Completed() {
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
	for _, sk := range skillsToAllocateBasedOnLevel {
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
		if skillData, found := n.Data.PlayerUnit.Skills[sk]; found {
			currentLevel = int(skillData.Level)
		}

		pointsToAdd := target - currentLevel
		if pointsToAdd > 0 {
			for i := 0; i < pointsToAdd; i++ {
				skillsToAllocate = append(skillsToAllocate, sk)
			}
		}
	}

	return skillsToAllocate
}

func (n *NecromancerLeveling) killBoss(bossNPC npc.ID, timeout time.Duration) error {
	startTime := time.Now()
	var lastPrisonCast, lastGolemCast time.Time

	for time.Since(startTime) < timeout {
		boss, found := n.Data.Monsters.FindOne(bossNPC, data.MonsterTypeUnique)
		if !found {
			time.Sleep(time.Second)
			continue
		}

		if boss.Stats[stat.Life] <= 0 {
			return nil
		}

		primaryAttackRange := step.Distance(1, 20)
		lvl, _ := n.Data.PlayerUnit.FindStat(stat.Level, 0)

		if time.Since(lastGolemCast) > time.Second*5 {
			step.SecondaryAttack(skill.ClayGolem, boss.UnitID, 1)
			lastGolemCast = time.Now()
		}
		if !boss.States.HasState(state.Ironmaiden) {
			step.SecondaryAttack(skill.IronMaiden, boss.UnitID, 1)
		}

		if lvl.Value >= 26 {
			if time.Since(lastPrisonCast) > time.Second*2 {
				step.SecondaryAttack(skill.BonePrison, boss.UnitID, 1)
				lastPrisonCast = time.Now()
			}
			step.PrimaryAttack(boss.UnitID, 5, true, primaryAttackRange)
		} else {
			step.PrimaryAttack(boss.UnitID, 3, true, primaryAttackRange)
		}
	}
	return fmt.Errorf("%s timeout", bossNPC)
}

func (n *NecromancerLeveling) killMonsterByName(id npc.ID, monsterType data.MonsterType, skipOnImmunities []stat.Resist) error {
	for {
		monster, found := n.Data.Monsters.FindOne(id, monsterType)
		if !found || monster.Stats[stat.Life] <= 0 {
			return nil
		}
		n.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			m, found := d.Monsters.FindOne(id, monsterType)
			if !found {
				return 0, false
			}
			return m.UnitID, true
		}, skipOnImmunities)
		time.Sleep(time.Millisecond * 250)
	}
}

func (n *NecromancerLeveling) KillCountess() error {
	return n.killMonsterByName(npc.DarkStalker, data.MonsterTypeSuperUnique, nil)
}

func (n *NecromancerLeveling) KillAndariel() error {
	return n.killBoss(npc.Andariel, time.Second*180)
}

func (n *NecromancerLeveling) KillSummoner() error {
	return n.killMonsterByName(npc.Summoner, data.MonsterTypeUnique, nil)
}

func (n *NecromancerLeveling) KillDuriel() error {
	return n.killBoss(npc.Duriel, time.Second*180)
}

func (n *NecromancerLeveling) KillCouncil() error {
	return n.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		var councilMembers []data.Monster
		for _, m := range d.Monsters {
			if m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3 {
				councilMembers = append(councilMembers, m)
			}
		}

		sort.Slice(councilMembers, func(i, j int) bool {
			distanceI := n.PathFinder.DistanceFromMe(councilMembers[i].Position)
			distanceJ := n.PathFinder.DistanceFromMe(councilMembers[j].Position)

			return distanceI < distanceJ
		})

		for _, m := range councilMembers {
			return m.UnitID, true
		}

		return 0, false
	}, nil)
}

func (n *NecromancerLeveling) KillMephisto() error {
	return n.killBoss(npc.Mephisto, time.Second*180)
}

func (n *NecromancerLeveling) KillIzual() error {
	return n.killMonsterByName(npc.Izual, data.MonsterTypeUnique, nil)
}

func (n *NecromancerLeveling) KillDiablo() error {
	return n.killBoss(npc.Diablo, time.Second*220)
}

func (n *NecromancerLeveling) KillPindle() error {
	return n.killMonsterByName(npc.DefiledWarrior, data.MonsterTypeSuperUnique, nil)
}

func (n *NecromancerLeveling) KillAncients() error {
	originalBackToTownCfg := n.CharacterCfg.BackToTown
	n.CharacterCfg.BackToTown.NoHpPotions = false
	n.CharacterCfg.BackToTown.NoMpPotions = false
	n.CharacterCfg.BackToTown.EquipmentBroken = false
	n.CharacterCfg.BackToTown.MercDied = false

	for _, m := range n.Data.Monsters.Enemies(data.MonsterEliteFilter()) {
		foundMonster, found := n.Data.Monsters.FindOne(m.Name, data.MonsterTypeSuperUnique)
		if !found {
			continue
		}
		step.MoveTo(data.Position{X: 10062, Y: 12639})
		n.killMonsterByName(foundMonster.Name, data.MonsterTypeSuperUnique, nil)
	}

	n.CharacterCfg.BackToTown = originalBackToTownCfg
	return nil
}

func (n *NecromancerLeveling) KillNihlathak() error {
	return n.killMonsterByName(npc.Nihlathak, data.MonsterTypeSuperUnique, nil)
}

func (n *NecromancerLeveling) KillBaal() error {
	return n.killBoss(npc.BaalCrab, time.Second*240)
}
