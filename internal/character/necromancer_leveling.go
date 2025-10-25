package character

import (
	"fmt"
	"math"
	"slices"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
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

var _ context.LevelingCharacter = (*NecromancerLeveling)(nil)

const (
	AmplifyDamageMaxDistance    = 25
	BoneSpearMaxDistance        = 25
	NecroLevelingMaxAttacksLoop = 100
	BonePrisonMaxDistance       = 25
	LevelToResetSkills          = 26
	RaiseSkeletonMaxDistance    = 25
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
	lastAmplifyDamageCast   time.Time
	lastLineOfSight         map[data.UnitID]time.Time
	lastBoneArmorCast       time.Time
	lastBonePrisonCast      map[data.UnitID]time.Time
	lastCorpseExplosionCast map[data.UnitID]time.Time
}

func (s NecromancerLeveling) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (n *NecromancerLeveling) GetAdditionalRunewords() []string {
	return append(action.GetCastersCommonRunewords(), "White")
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

func (n *NecromancerLeveling) hasSkeletonNearby() bool {
	for _, m := range n.Data.Monsters {
		if m.Name == npc.NecroSkeleton && m.IsPet() {
			distance := n.PathFinder.DistanceFromMe(m.Position)
			if distance <= RaiseSkeletonMaxDistance {
				return true
			}
		}
	}
	return false
}

// ensureBoneArmor checks if Bone Armor is active and recasts if needed
func (n *NecromancerLeveling) ensureBoneArmor() {
	if !n.hasSkill(skill.BoneArmor) {
		return
	}

	// Check if Bone Armor is active
	if !n.Data.PlayerUnit.States.HasState(state.Bonearmor) {
		// Cast Bone Armor immediately if it's down
		step.SecondaryAttack(skill.BoneArmor, n.Data.PlayerUnit.ID, 1, step.Distance(0, 1))
		n.lastBoneArmorCast = time.Now()
		n.Logger.Debug("Casting Bone Armor (buff expired)")
		utils.Sleep(200)
		return
	}

	// Recast periodically even if active (to maintain max charges)
	if time.Since(n.lastBoneArmorCast) > time.Second*10 {
		step.SecondaryAttack(skill.BoneArmor, n.Data.PlayerUnit.ID, 1, step.Distance(0, 1))
		n.lastBoneArmorCast = time.Now()
		n.Logger.Debug("Refreshing Bone Armor")
		utils.Sleep(200)
	}
}

// shouldRetreat checks if we need to escape due to low health
func (n *NecromancerLeveling) shouldRetreat() bool {
	// Don't retreat during Ancients fight (can't TP out)
	if n.Data.PlayerUnit.Area == area.ArreatSummit {
		return false
	}

	// Retreat if health is critically low
	if n.Data.PlayerUnit.HPPercent() < 30 {
		n.Logger.Warn("Health critically low, attempting to retreat")
		return true
	}

	return false
}

// castDefensiveBonePrison casts Bone Prison on the boss for defensive purposes
func (n *NecromancerLeveling) castDefensiveBonePrison(boss data.Monster) {
	if !n.hasSkill(skill.BonePrison) {
		return
	}

	// Initialize map if needed
	if n.lastBonePrisonCast == nil {
		n.lastBonePrisonCast = make(map[data.UnitID]time.Time)
	}

	// Check if we recently cast on this boss
	if lastCast, found := n.lastBonePrisonCast[boss.UnitID]; found && time.Since(lastCast) < time.Second*3 {
		return
	}

	// Cast Bone Prison to trap the boss
	step.SecondaryAttack(skill.BonePrison, boss.UnitID, 1, bonePrisonRange)
	n.lastBonePrisonCast[boss.UnitID] = time.Now()
	n.Logger.Debug("Casting defensive Bone Prison", "boss", boss.Name)
	utils.Sleep(150)
}

func (n *NecromancerLeveling) castCorpseSkill(skillID skill.ID, corpse *data.Monster, maxDistance int) {
	ctx := context.Get()

	if ctx.PathFinder.DistanceFromMe(corpse.Position) > maxDistance {
		return
	}

	if !ctx.PathFinder.LineOfSight(ctx.Data.PlayerUnit.Position, corpse.Position) {
		return
	}

	if ctx.Data.PlayerUnit.RightSkill != skillID {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.MustKBForSkill(skillID))
		utils.Sleep(50)
	}

	ctx.HID.KeyDown(ctx.Data.KeyBindings.StandStill)
	x, y := ctx.PathFinder.GameCoordsToScreenCords(corpse.Position.X, corpse.Position.Y)
	ctx.HID.Click(game.RightButton, x, y)
	ctx.HID.KeyUp(ctx.Data.KeyBindings.StandStill)
	utils.Sleep(50)
}

// findSafeBossPosition finds a safe position to attack the boss from
func (n *NecromancerLeveling) findSafeBossPosition(boss data.Monster, currentDistance int) (data.Position, bool) {
	// Define safe casting distance based on boss proximity
	minSafeDistance := 10
	maxCastDistance := BoneSpearMaxDistance

	// If boss is too close, increase safe distance
	if currentDistance < 8 {
		minSafeDistance = 12
	}

	// Use the action package helper to find a safe position
	safePos, found := action.FindSafePosition(
		boss,
		8,               // dangerDistance - minimum distance from boss
		minSafeDistance, // safeDistance - preferred distance from boss
		10,              // minAttackDistance - min range for attacks
		maxCastDistance, // maxAttackDistance - max range for attacks
	)

	return safePos, found
}

func (n *NecromancerLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	const priorityMonsterSearchRange = 15
	completedAttackLoops := 0
	previousUnitID := 0
	bonePrisonnedMonsters := make(map[data.UnitID]time.Time)
	ctx := context.Get()
	// Initialize line of sight tracking map
	if n.lastLineOfSight == nil {
		n.lastLineOfSight = make(map[data.UnitID]time.Time)
	}

	if n.lastCorpseExplosionCast == nil {
		n.lastCorpseExplosionCast = make(map[data.UnitID]time.Time)
	}

	for {
		var id data.UnitID
		var found bool

		ctx.PauseIfNotPriority()

		id, found = monsterSelector(*n.Data)

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

		// Check if we should switch targets due to lost line of sight
		if action.ShouldSwitchTarget(id, targetMonster, n.lastLineOfSight) {
			completedAttackLoops = 0
			continue
		}

		// Cast Bone Prison on elites (in allowed areas) and on Duriel
		shouldCastBonePrison := n.hasSkill(skill.BonePrison) && slices.Contains(bonePrisonAllowedAreas, n.Data.PlayerUnit.Area)
		if shouldCastBonePrison && (targetMonster.IsElite() || targetMonster.Name == npc.Duriel) {
			if lastPrisonCast, found := bonePrisonnedMonsters[targetMonster.UnitID]; !found || time.Since(lastPrisonCast) > time.Second*4 {
				step.SecondaryAttack(skill.BonePrison, targetMonster.UnitID, 1, bonePrisonRange)
				bonePrisonnedMonsters[targetMonster.UnitID] = time.Now()
				n.Logger.Debug("Casting Bone Prison", "target", targetMonster.Name)
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
			corpses := n.getUsableCorpses()
			radius := 3.0 + float64(n.Data.PlayerUnit.Skills[skill.CorpseExplosion].Level-1)*0.3
			radiusSquared := radius * radius
			corpseExplosionMaxDistance := float64(BoneSpearMaxDistance) + radius
			var nearbyCorpse *data.Monster

			for i := range corpses {
				corpse := &corpses[i]
				dx := float64(targetMonster.Position.X - corpse.Position.X)
				dy := float64(targetMonster.Position.Y - corpse.Position.Y)
				if (dx*dx+dy*dy) < radiusSquared && float64(n.PathFinder.DistanceFromMe(corpse.Position)) < corpseExplosionMaxDistance {
					nearbyCorpse = corpse
					break
				}
			}

			if nearbyCorpse != nil {
				if lastCastTime, found := n.lastCorpseExplosionCast[targetMonster.UnitID]; !found || time.Since(lastCastTime) > time.Second*2 {
					n.castCorpseSkill(skill.CorpseExplosion, nearbyCorpse, int(math.Ceil(corpseExplosionMaxDistance)))
					n.Logger.Debug("Casting Corpse Explosion")
					utils.Sleep(150)
					n.lastCorpseExplosionCast[targetMonster.UnitID] = time.Now()
					completedAttackLoops++
					previousUnitID = int(id)
					continue
				}
			}
		}

		lvl, _ := n.Data.PlayerUnit.FindStat(stat.Level, 0)

		// Before learning Teeth, use Raise Skeleton + basic attack
		if !n.hasSkill(skill.Teeth) {
			if !n.hasSkeletonNearby() {
				corpses := n.getUsableCorpses()
				var nearbyCorpse *data.Monster

				for i := range corpses {
					corpse := &corpses[i]
					if n.PathFinder.DistanceFromMe(corpse.Position) <= RaiseSkeletonMaxDistance {
						nearbyCorpse = corpse
						break
					}
				}

				if nearbyCorpse != nil {
					n.castCorpseSkill(skill.RaiseSkeleton, nearbyCorpse, RaiseSkeletonMaxDistance)
					n.Logger.Debug("Casting Raise Skeleton")
					utils.Sleep(300) // Give time for skeleton to spawn
					completedAttackLoops++
					previousUnitID = int(id)
					continue
				}
			}

			step.PrimaryAttack(targetMonster.UnitID, 1, false, step.Distance(1, 3))
			n.Logger.Debug("Using Basic attack (pre-Teeth)")
			utils.Sleep(150)
		} else if n.Data.PlayerUnit.MPPercent() < 15 && lvl.Value < 12 || lvl.Value < 2 {
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
		// Before Teeth, bind Raise Skeleton
		if !n.hasSkill(skill.Teeth) {
			skillBindings = append(skillBindings, skill.ID(70)) // Raise Skeleton
		}

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

	return skillSequence
}

// killBossSequence handles boss encounters with defensive tactics
func (n *NecromancerLeveling) killBossSequence(bossNPC npc.ID, monsterType data.MonsterType) error {
	completedAttackLoops := 0
	const maxBossAttackLoops = 200
	lastRepositionTime := time.Time{}
	repositionCooldown := time.Second * 2

	n.Logger.Info("Starting boss fight", "boss", bossNPC)

	for {
		context.Get().PauseIfNotPriority()

		// Check if we should retreat (except for Ancients)
		if n.shouldRetreat() {
			n.Logger.Warn("Retreating from boss fight due to low health")
			// Try to use TP scroll - just return error, chicken system will handle it
			return fmt.Errorf("retreated due to low health")
		}

		// Ensure Bone Armor is always active
		n.ensureBoneArmor()

		// Find the boss
		boss, found := n.Data.Monsters.FindOne(bossNPC, monsterType)
		if !found {
			n.Logger.Debug("Boss not found, may be dead")
			return nil
		}

		// Check if boss is dead
		if boss.Stats[stat.Life] <= 0 {
			n.Logger.Info("Boss defeated", "boss", bossNPC)
			return nil
		}

		// Check max attack loops to prevent infinite combat
		if completedAttackLoops >= maxBossAttackLoops {
			n.Logger.Warn("Max boss attack loops reached", "loops", completedAttackLoops)
			return fmt.Errorf("max boss attack loops reached")
		}

		// Calculate distance to boss
		distanceToBoss := n.PathFinder.DistanceFromMe(boss.Position)

		// Cast Bone Prison if boss is too close or as defensive measure
		if distanceToBoss < 15 {
			n.castDefensiveBonePrison(boss)
		}

		// Reposition if needed and not on cooldown
		if time.Since(lastRepositionTime) > repositionCooldown {
			shouldReposition := false

			// Reposition if boss is too close
			if distanceToBoss < 6 {
				n.Logger.Debug("Boss too close, repositioning", "distance", distanceToBoss)
				shouldReposition = true
			}

			// Reposition if health is getting low (but not critical)
			if n.Data.PlayerUnit.HPPercent() < 50 && distanceToBoss < 10 {
				n.Logger.Debug("Health low, repositioning to safer distance")
				shouldReposition = true
			}

			// Check for other monsters nearby
			enemyNearby, _ := action.IsAnyEnemyAroundPlayer(5)
			if enemyNearby && distanceToBoss > 5 {
				n.Logger.Debug("Enemies nearby, repositioning")
				shouldReposition = true
			}

			if shouldReposition {
				safePos, found := n.findSafeBossPosition(boss, distanceToBoss)
				if found {
					n.Logger.Debug("Moving to safe position")
					step.MoveTo(safePos, step.WithIgnoreMonsters())
					lastRepositionTime = time.Now()
					utils.Sleep(150)
					continue
				}
			}
		}

		// Cast Amplify Damage on boss
		if n.hasSkill(skill.AmplifyDamage) && !boss.States.HasState(state.Amplifydamage) && time.Since(n.lastAmplifyDamageCast) > time.Second*2 {
			step.SecondaryAttack(skill.AmplifyDamage, boss.UnitID, 1, amplifyDamageRange)
			n.Logger.Debug("Casting Amplify Damage on boss")
			n.lastAmplifyDamageCast = time.Now()
			utils.Sleep(150)
		}

		// Attack with appropriate skill
		lvl, _ := n.Data.PlayerUnit.FindStat(stat.Level, 0)

		if n.hasSkill(skill.BoneSpear) {
			// Ensure we have line of sight
			if !n.PathFinder.LineOfSight(n.Data.PlayerUnit.Position, boss.Position) {
				// Move to get line of sight
				n.Logger.Debug("No line of sight, moving closer")
				step.MoveTo(boss.Position)
				utils.Sleep(200)
				continue
			}

			step.PrimaryAttack(boss.UnitID, 3, true, boneSpearRange)
			n.Logger.Debug("Casting Bone Spear on boss")
			utils.Sleep(150)
		} else if n.hasSkill(skill.Teeth) {
			if !n.PathFinder.LineOfSight(n.Data.PlayerUnit.Position, boss.Position) {
				n.Logger.Debug("No line of sight, moving closer")
				step.MoveTo(boss.Position)
				utils.Sleep(200)
				continue
			}

			step.SecondaryAttack(skill.Teeth, boss.UnitID, 3, boneSpearRange)
			n.Logger.Debug("Casting Teeth on boss")
			utils.Sleep(150)
		} else if lvl.Value < 2 || n.Data.PlayerUnit.MPPercent() < 15 {
			// Basic attack for very low level or out of mana
			step.PrimaryAttack(boss.UnitID, 2, false, step.Distance(1, 5))
			n.Logger.Debug("Using basic attack on boss")
			utils.Sleep(150)
		} else {
			// This shouldn't happen, but fallback to basic attack
			step.PrimaryAttack(boss.UnitID, 2, false, step.Distance(1, 5))
			utils.Sleep(150)
		}

		completedAttackLoops++
	}
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

		// Use the new boss sequence for improved combat
		return n.killBossSequence(bossNPC, data.MonsterTypeUnique)
	}

	return fmt.Errorf("boss with ID: %d not found", bossNPC)
}

func (n *NecromancerLeveling) killMonsterByName(id npc.ID, monsterType data.MonsterType, skipOnImmunities []stat.Resist) error {
	// Check if this is a boss/super unique - use boss sequence
	if monsterType == data.MonsterTypeSuperUnique || monsterType == data.MonsterTypeUnique {
		return n.killBossSequence(id, monsterType)
	}

	// Regular monster - use normal sequence
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
	// Council is a special case - multiple bosses
	// Kill them one by one using boss sequence
	councilNPCs := []npc.ID{npc.CouncilMember, npc.CouncilMember2, npc.CouncilMember3}

	for {
		// Find closest living council member
		var closestCouncil data.Monster
		minDistance := 999
		found := false

		for _, councilNPC := range councilNPCs {
			for _, m := range n.Data.Monsters {
				if m.Name == councilNPC && m.Stats[stat.Life] > 0 {
					distance := n.PathFinder.DistanceFromMe(m.Position)
					if distance < minDistance {
						minDistance = distance
						closestCouncil = m
						found = true
					}
				}
			}
		}

		// No more council members alive
		if !found {
			n.Logger.Info("All council members defeated")
			return nil
		}

		// Kill the closest council member using boss tactics
		n.Logger.Info("Engaging council member", "name", closestCouncil.Name)
		err := n.killBossSequence(closestCouncil.Name, data.MonsterTypeSuperUnique)
		if err != nil {
			return err
		}
	}
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
	// Ancients is special - can't retreat, must fight to the death
	// Disable back to town checks temporarily
	originalBackToTownCfg := n.CharacterCfg.BackToTown
	n.CharacterCfg.BackToTown.NoHpPotions = false
	n.CharacterCfg.BackToTown.NoMpPotions = false
	n.CharacterCfg.BackToTown.EquipmentBroken = false
	n.CharacterCfg.BackToTown.MercDied = false

	n.Logger.Info("Starting Ancients fight")

	// Kill ancients one by one, focusing on closest
	for {
		// Find all living ancients
		var ancients []data.Monster
		for _, m := range n.Data.Monsters.Enemies(data.MonsterEliteFilter()) {
			if m.Stats[stat.Life] > 0 {
				ancients = append(ancients, m)
			}
		}

		// No more ancients alive
		if len(ancients) == 0 {
			n.Logger.Info("All ancients defeated")
			break
		}

		// Find closest ancient
		var closest data.Monster
		minDistance := 999
		for _, ancient := range ancients {
			distance := n.PathFinder.DistanceFromMe(ancient.Position)
			if distance < minDistance {
				minDistance = distance
				closest = ancient
			}
		}

		// Move to safe position near platform center for better positioning
		if minDistance > 15 {
			step.MoveTo(data.Position{X: 10062, Y: 12639}, step.WithIgnoreMonsters())
			utils.Sleep(200)
		}

		n.Logger.Info("Engaging ancient", "name", closest.Name, "distance", minDistance)

		// Use boss sequence for the ancient
		err := n.killBossSequence(closest.Name, data.MonsterTypeSuperUnique)
		if err != nil {
			n.Logger.Error("Error fighting ancient", "error", err)
			// Continue fighting even on error for ancients
		}
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

func (n *NecromancerLeveling) getUsableCorpses() []data.Monster {
	corpses := []data.Monster{}

	unusableCorpseStates := []state.State{
		state.CorpseNoselect,
		state.CorpseNodraw,
		state.Revive,
		state.Redeemed,
		state.Shatter,
		state.Freeze,
		state.Restinpeace,
	}

	for _, c := range n.Data.Corpses {
		if c.IsMerc() {
			continue
		}

		skipCorpse := false
		for _, unusableState := range unusableCorpseStates {
			if c.States.HasState(unusableState) {
				skipCorpse = true
				break
			}
		}

		if skipCorpse {
			continue
		}

		corpses = append(corpses, c)
	}

	return corpses
}
