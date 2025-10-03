package character

import (
	"fmt"
	"slices"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	AmplifyDamageMaxDistance           = 25
	BoneSpearMaxDistance               = 25
	CorpseExplosionRadiusAroundMonster = 5
	NecroLevelingMaxAttacksLoop        = 100
	BonePrisonMaxDistance              = 25
	LevelToResetSkills                 = 26
)

var (
	boneSpearRange         = step.Distance(0, BoneSpearMaxDistance)
	amplifyDamageRange     = step.Distance(0, AmplifyDamageMaxDistance)
	bonePrisonRange        = step.Distance(0, BonePrisonMaxDistance)
	bonePrisonAllowedAreas = []area.ID{
		area.CatacombsLevel4, area.Tristram, area.MooMooFarm,
		area.RockyWaste, area.DryHills, area.FarOasis,
		area.LostCity, area.ValleyOfSnakes, area.DurielsLair,
		area.SpiderForest, area.GreatMarsh, area.FlayerJungle,
		area.LowerKurast, area.KurastBazaar, area.UpperKurast,
		area.KurastCauseway, area.DuranceOfHateLevel3, area.OuterSteppes,
		area.PlainsOfDespair, area.CityOfTheDamned, area.ChaosSanctuary,
		area.BloodyFoothills, area.FrigidHighlands, area.ArreatSummit,
		area.NihlathaksTemple, area.TheWorldStoneKeepLevel1, area.TheWorldStoneKeepLevel2,
		area.TheWorldStoneKeepLevel3, area.ThroneOfDestruction,
	}
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

func (n *NecromancerLeveling) hasSkill(sk skill.ID) bool {
	skill, found := n.Data.PlayerUnit.Skills[sk]
	return found && skill.Level > 0
}

func (n *NecromancerLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	bonePrisonnedMonsters := make(map[data.UnitID]time.Time)

	for {
		id, found := monsterSelector(*n.Data)
		if !found {
			return nil
		}

		if previousUnitID != int(id) {
			completedAttackLoops = 0
		}

		if !n.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		if completedAttackLoops >= NecroLevelingMaxAttacksLoop {
			n.Logger.Debug("Max attack loops reached")
			return nil
		}

		targetMonster, found := n.Data.Monsters.FindByID(id)
		if !found {
			return nil
		}

		if n.hasSkill(skill.BonePrison) && targetMonster.IsElite() && slices.Contains(bonePrisonAllowedAreas, n.Data.PlayerUnit.Area) {
			if lastPrisonCast, found := bonePrisonnedMonsters[targetMonster.UnitID]; !found || time.Since(lastPrisonCast) > time.Second*4 {
				step.SecondaryAttack(skill.BonePrison, targetMonster.UnitID, 1, bonePrisonRange)
				bonePrisonnedMonsters[targetMonster.UnitID] = time.Now()
				n.Logger.Debug("Casting Bone Prison")
				utils.Sleep(150)
			}
		}

		if n.hasSkill(skill.AmplifyDamage) && !targetMonster.States.HasState(state.Amplifydamage) && time.Since(n.lastAmplifyDamageCast) > time.Second*2 {
			step.SecondaryAttack(skill.AmplifyDamage, targetMonster.UnitID, 1, amplifyDamageRange)
			n.Logger.Debug("Casting Amplify Damage")
			utils.Sleep(150)
			n.lastAmplifyDamageCast = time.Now()
		}

		if n.hasSkill(skill.CorpseExplosion) {
			radius := 3.0 + float64(n.Data.PlayerUnit.Skills[skill.CorpseExplosion].Level-1)*0.3
			radiusSquared := float64(radius * radius)
			corpseExplosionMaxDistance := float64(BoneSpearMaxDistance) + radius

			isCorpseNearby := false
			for _, c := range n.Data.Corpses {
				dx := float64(targetMonster.Position.X - c.Position.X)
				dy := float64(targetMonster.Position.Y - c.Position.Y)
				if (dx*dx+dy*dy) < radiusSquared && float64(n.PathFinder.DistanceFromMe(c.Position)) < corpseExplosionMaxDistance {
					isCorpseNearby = true
					break
				}
			}

			if isCorpseNearby {
				step.SecondaryAttack(skill.CorpseExplosion, targetMonster.UnitID, 1, step.Distance(1, int(corpseExplosionMaxDistance)))
				n.Logger.Debug("Casting Corpse Explosion")
				utils.Sleep(150)
				completedAttackLoops++
				previousUnitID = int(id)
				continue
			}
		}

		lvl, _ := n.Data.PlayerUnit.FindStat(stat.Level, 0)

		if n.Data.PlayerUnit.MPPercent() < 15 && lvl.Value < 12 || lvl.Value < 2 {
			step.PrimaryAttack(targetMonster.UnitID, 1, false, step.Distance(1, 2))
			n.Logger.Debug("Using Basic attack")
			utils.Sleep(150)
		} else if n.hasSkill(skill.BoneSpear) {
			step.PrimaryAttack(targetMonster.UnitID, 3, true, boneSpearRange)
			n.Logger.Debug("Casting Bone Spear")
			utils.Sleep(150)
		} else if n.hasSkill(skill.Teeth) {
			step.SecondaryAttack(skill.Teeth, targetMonster.UnitID, 3, boneSpearRange)
			n.Logger.Debug("Casting Teeth")
			utils.Sleep(150)
		}

		completedAttackLoops++
		previousUnitID = int(id)
	}
}

func (n *NecromancerLeveling) ShouldResetSkills() bool {
	lvl, _ := n.Data.PlayerUnit.FindStat(stat.Level, 0)
	return lvl.Value == LevelToResetSkills && n.Data.PlayerUnit.Skills[skill.Teeth].Level > 9
}

func (n *NecromancerLeveling) SkillsToBind() (skill.ID, []skill.ID) {
	lvl, _ := n.Data.PlayerUnit.FindStat(stat.Level, 0)

	mainSkill := skill.AttackSkill
	skillBindings := []skill.ID{}

	if lvl.Value >= LevelToResetSkills {
		mainSkill = skill.BoneSpear

		skillBindings = []skill.ID{
			skill.CorpseExplosion,
			skill.BoneArmor,
			skill.BonePrison,
		}

		if n.hasSkill(skill.AmplifyDamage) {
			skillBindings = append(skillBindings, skill.AmplifyDamage)
		}

	} else {
		if n.hasSkill(skill.Teeth) {
			skillBindings = append(skillBindings, skill.Teeth)
		}
		if n.hasSkill(skill.AmplifyDamage) {
			skillBindings = append(skillBindings, skill.AmplifyDamage)
		}
		if n.hasSkill(skill.BoneArmor) {
			skillBindings = append(skillBindings, skill.BoneArmor)
		}
		if n.hasSkill(skill.CorpseExplosion) {
			skillBindings = append(skillBindings, skill.CorpseExplosion)
		}
		if n.hasSkill(skill.BoneSpear) {
			mainSkill = skill.BoneSpear
		}
		if n.hasSkill(skill.BonePrison) {
			skillBindings = append(skillBindings, skill.BonePrison)
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
		{Stat: stat.Vitality, Points: 20},
		{Stat: stat.Strength, Points: 20},
		{Stat: stat.Vitality, Points: 30},
		{Stat: stat.Strength, Points: 30},
		{Stat: stat.Vitality, Points: 40},
		{Stat: stat.Strength, Points: 40},
		{Stat: stat.Vitality, Points: 50},
		{Stat: stat.Strength, Points: 50},
		{Stat: stat.Vitality, Points: 100},
		{Stat: stat.Strength, Points: 95},
		{Stat: stat.Vitality, Points: 250},
		{Stat: stat.Strength, Points: 156},
		{Stat: stat.Vitality, Points: 999},
	}
}

func (n *NecromancerLeveling) SkillPoints() []skill.ID {
	lvl, _ := n.Data.PlayerUnit.FindStat(stat.Level, 0)
	var skillSequence []skill.ID

	if lvl.Value < LevelToResetSkills {
		skillSequence = []skill.ID{
			skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth,
			skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth,
			skill.AmplifyDamage,
			skill.AmplifyDamage,
			skill.BoneArmor,
			skill.BoneWall, skill.BoneWall, skill.BoneWall,
			skill.CorpseExplosion,
			skill.BoneSpear, skill.BoneSpear, skill.BoneSpear,
			skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion,
			skill.BonePrison, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion,
			skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear,
			skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion,
			skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear,
		}
	} else {
		skillSequence = []skill.ID{
			skill.Teeth,
			skill.BoneArmor,
			skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion,
			skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion, skill.CorpseExplosion,
			skill.BoneWall,
			skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear,
			skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear,
			skill.BonePrison, skill.BonePrison, skill.BonePrison,
			skill.BoneSpear, skill.BonePrison, skill.BoneSpear, skill.BonePrison, skill.BoneSpear, skill.BonePrison,
			skill.BoneSpear, skill.BonePrison, skill.BoneSpear, skill.BonePrison, skill.BoneSpear, skill.BonePrison,
			skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear, skill.BoneSpear,
			skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison,
			skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison, skill.BonePrison,
			skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall,
			skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall,
			skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall, skill.BoneWall,
			skill.AmplifyDamage,
			skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth,
			skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth, skill.Teeth,
			skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit,
			skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit,
			skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit, skill.BoneSpirit,
		}
	}

	questSkillPoints := 0

	if n.CharacterCfg.Game.Difficulty == difficulty.Nightmare {
		questSkillPoints += 4
	}

	if n.CharacterCfg.Game.Difficulty == difficulty.Hell {
		questSkillPoints += 4
	}

	if n.Data.Quests[quest.Act1DenOfEvil].Completed() {
		questSkillPoints += 1
	}
	if n.Data.Quests[quest.Act2RadamentsLair].Completed() {
		questSkillPoints += 2
	}
	if n.Data.Quests[quest.Act4TheFallenAngel].Completed() {
		questSkillPoints += 4
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

func (n *NecromancerLeveling) killBoss(bossNPC npc.ID) error {
	startTime := time.Now()
	timeout := time.Second * 20

	for time.Since(startTime) < timeout {
		boss, found := n.Data.Monsters.FindOne(bossNPC, data.MonsterTypeUnique)
		if !found {
			time.Sleep(time.Second)
			continue
		}

		if boss.Stats[stat.Life] <= 0 {
			return nil
		}

		return n.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			m, found := d.Monsters.FindOne(bossNPC, data.MonsterTypeUnique)
			if !found {
				return 0, false
			}
			return m.UnitID, true
		}, nil)
	}

	return fmt.Errorf("boss with ID: %d not found", bossNPC)
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
	return n.killBoss(npc.Andariel)
}

func (n *NecromancerLeveling) KillSummoner() error {
	return n.killMonsterByName(npc.Summoner, data.MonsterTypeUnique, nil)
}

func (n *NecromancerLeveling) KillDuriel() error {
	return n.killBoss(npc.Duriel)
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
	return n.killBoss(npc.Mephisto)
}

func (n *NecromancerLeveling) KillIzual() error {
	return n.killMonsterByName(npc.Izual, data.MonsterTypeUnique, nil)
}

func (n *NecromancerLeveling) KillDiablo() error {
	return n.killBoss(npc.Diablo)
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
	return n.killBoss(npc.BaalCrab)
}
