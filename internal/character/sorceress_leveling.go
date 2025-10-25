package character

import (
	"fmt"
	"log/slog"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/mode"
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

const (
	SorceressLevelingMaxAttacksLoop     = 50
	SorceressLevelingMaxBossAttacksLoop = 300
	SorceressLevelingMinDistance        = 10
	SorceressLevelingMaxDistance        = 45
	SorceressLevelingMeleeDistance      = 2
	SorceressLevelingSafeDistanceLevel  = 24
	SorceressLevelingThreatDistance     = 15
	AndarielRepositionLength            = 9

	SorceressLevelingDangerDistance = 4
	SorceressLevelingSafeDistance   = 6

	StaticFieldEffectiveRange = 4 // Maximum distance for Static Field to reliably hit
)

type SorceressLeveling struct {
	BaseCharacter
	blizzardCasts                map[data.UnitID]int  // To track Blizzard casts on SuperUnique monsters
	blizzardPhaseCompleted       map[data.UnitID]bool // New: To track if 2 Blizzards have been cast for a SU
	staticPhaseCompleted         map[data.UnitID]bool // New: To track if Static Field threshold reached for bosses
	blizzardInitialCastCompleted map[data.UnitID]bool
	andarielMoves                int
	andarielSafePositions        []data.Position
}

// fireSkillSequence defines the skill allocation for levels < 32
var fireSkillSequence = []skill.ID{
	skill.FrozenArmor, // Lvl 2 (1st point)
	skill.FireBolt,    // Lvl 2 (+1 from Den of Evil, 2nd point)
	skill.FireBolt,    // Lvl 3 (3rd point)
	skill.FireBolt,    // Lvl 4 (4th point)
	skill.FireBolt,    // Lvl 5 (1st point)
	skill.FireBolt,    // Lvl 6 (1st point)
	skill.FireBolt,    // Lvl 7 (1st point)
	skill.FireBolt,    // Lvl 8 (5th point)
	skill.FireBolt,    // Lvl 9 (6th point)
	skill.FireBolt,    // Lvl 10 (7th point)
	skill.FireBolt,    // Lvl 11 (8th point)
	skill.FireBall,    // Lvl 12 (9th point)
	skill.FireBall,    // Lvl 13 (1st point)
	skill.FireBolt,    // Lvl 13 (At this point we should have rada so we need to spend another point but can't spend into Fireball because of lvl restrictions)
	skill.FireBall,    // Lvl 14 (2nd point)
	skill.FireBall,    // Lvl 15 (3rd point)
	skill.FireBall,    // Lvl 16 (4th point)
	skill.Telekinesis, // Lvl 17 (1st point)
	skill.Teleport,    // Lvl 18 (1st point)
	skill.FireBall,    // Lvl 19 (5th point)
	skill.FireBall,    // Lvl 20 (6th point)
	skill.FireBall,    // Lvl 21 (7th point)
	skill.FireBall,    // Lvl 22 (8th point)
	skill.FireBall,    // Lvl 23 (9th point)
	skill.FireBall,    // Lvl 24 (10th point)
	skill.FireBall,    // Lvl 25 (11th point)
	skill.FireBall,    // Lvl 26 (12th point)
	skill.FireBall,    // Lvl 27 (13th point)
	skill.FireBall,    // Lvl 28 (14th point)
	skill.FireBall,    // Lvl 29 (15th point)
	skill.FireMastery, // Lvl 30 (1st point)
	skill.FireBall,    // Lvl 31 (17th point)
	skill.FireBall,    // Lvl 32 (18th point)
	skill.FireBall,    // Lvl 33 (19th point)
	skill.FireBall,    // Lvl 34 (20th point)
	skill.FireBall,    // Lvl 35 (20th point)
}

// blizzardSkillSequence defines the skill allocation for levels >= 26
var blizzardSkillSequence = []skill.ID{
	// Utility/Prerequisite skills (often 1 point)
	skill.StaticField,  // Utility
	skill.Telekinesis,  // Utility
	skill.Teleport,     // Utility
	skill.FrozenArmor,  // Utility
	skill.Warmth,       // Utility
	skill.FrostNova,    // Prerequisite for Blizzard
	skill.IceBolt,      // Prerequisite for Blizzard
	skill.IceBlast,     // Prerequisite for Blizzard
	skill.GlacialSpike, // Prerequisite for Blizzard     // spent 9

	skill.Blizzard, //spent 10

	skill.GlacialSpike, skill.GlacialSpike, skill.GlacialSpike, skill.GlacialSpike, skill.GlacialSpike,
	skill.GlacialSpike, skill.GlacialSpike, //spent 17

	skill.IceBlast, skill.IceBlast, skill.IceBlast, skill.IceBlast, skill.IceBlast, //spent 22
	skill.IceBlast, skill.IceBlast, skill.IceBlast, skill.IceBlast, // spent 26 (all points that are available at level 24 (23+2 den of evil + rada))

	skill.Blizzard, skill.Blizzard, skill.Blizzard, skill.Blizzard, skill.Blizzard, //spent 31
	skill.Blizzard, skill.Blizzard, skill.Blizzard, skill.ColdMastery, skill.Blizzard, //spent 36

	skill.Blizzard, skill.Blizzard, skill.Blizzard, skill.Blizzard, skill.Blizzard, //spent 41
	skill.Blizzard, skill.Blizzard, skill.Blizzard, skill.Blizzard, skill.Blizzard, //spent 46

	skill.ColdMastery, skill.ColdMastery, skill.ColdMastery, skill.ColdMastery, //spent 50
	skill.ColdMastery, skill.ColdMastery, skill.ColdMastery, skill.ColdMastery, skill.ColdMastery, //spent 55
	skill.ColdMastery, skill.ColdMastery, skill.ColdMastery, skill.ColdMastery, skill.ColdMastery, //spent 60
	skill.ColdMastery, skill.ColdMastery,

	skill.GlacialSpike, skill.GlacialSpike, skill.GlacialSpike, skill.GlacialSpike, skill.GlacialSpike, // Total 10 points in Glacial Spike
	skill.ChargedBolt, skill.Lightning, skill.ChainLightning, skill.EnergyShield,
	skill.GlacialSpike, skill.GlacialSpike, skill.GlacialSpike, skill.GlacialSpike, skill.GlacialSpike,
	skill.GlacialSpike, skill.GlacialSpike,

	// Phase 6: Ice Blast (to max at 20, for synergy)
	skill.IceBlast, skill.IceBlast, skill.IceBlast, skill.IceBlast, skill.IceBlast,
	skill.IceBlast, skill.IceBlast, skill.IceBlast, skill.IceBlast, skill.IceBlast,
	skill.IceBlast,

	// Phase 7: Ice Bolt (to max at 20, for synergy)
	skill.IceBolt, skill.IceBolt, skill.IceBolt, skill.IceBolt, skill.IceBolt,
	skill.IceBolt, skill.IceBolt, skill.IceBolt, skill.IceBolt, skill.IceBolt,
	skill.IceBolt, skill.IceBolt, skill.IceBolt, skill.IceBolt, skill.IceBolt,
	skill.IceBolt, skill.IceBolt, skill.IceBolt, skill.IceBolt,

	skill.ColdMastery, skill.ColdMastery, skill.ColdMastery,
}

// --- End Skill Point Sequences ---
func (s SorceressLeveling) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s SorceressLeveling) isPlayerDead() bool {
	return s.Data.PlayerUnit.HPPercent() <= 0
}

func (s SorceressLeveling) CheckKeyBindings() []skill.ID {
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

// findDangerousMonsters identifies and returns a list of monsters that are too close to the player.
func (s SorceressLeveling) findDangerousMonsters() []data.Monster {
	dangerousMonsters := []data.Monster{}
	for _, monster := range s.Data.Monsters {
		if monster.Stats[stat.Life] > 0 && pather.DistanceFromPoint(s.Data.PlayerUnit.Position, monster.Position) < SorceressLevelingDangerDistance {
			dangerousMonsters = append(dangerousMonsters, monster)
		}
	}
	return dangerousMonsters
}

func (s SorceressLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	lastReposition := time.Now()

	s.andarielSafePositions = []data.Position{
		{X: 22547, Y: 9591},
		{X: 22547, Y: 9600},
		{X: 22547, Y: 9609},
	}

	staticFieldTargets := map[npc.ID]struct{}{
		npc.Andariel:          {},
		npc.Duriel:            {},
		npc.Izual:             {},
		npc.Diablo:            {},
		npc.BaalCrab:          {},
		npc.AncientBarbarian:  {},
		npc.AncientBarbarian2: {},
		npc.AncientBarbarian3: {},
	}

	for {
		context.Get().PauseIfNotPriority()

		completedAttackLoops++
		s.Logger.Info("Completed Attack Loops", slog.Int("completedAttackLoops", completedAttackLoops))

		if s.isPlayerDead() {
			s.Logger.Info("Player detected as dead, stopping KillMonsterSequence.")
			return nil
		}

		id, found := monsterSelector(*s.Data)
		if !found {
			return nil
		}
		if previousUnitID != int(id) {
			completedAttackLoops = 0
			s.blizzardCasts = make(map[data.UnitID]int)
			s.blizzardPhaseCompleted = make(map[data.UnitID]bool)
			s.staticPhaseCompleted = make(map[data.UnitID]bool)
			s.blizzardInitialCastCompleted = make(map[data.UnitID]bool)
			s.andarielMoves = 0
		}

		if !s.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			s.Logger.Info("Target monster not found or died", slog.String("monsterID", fmt.Sprintf("%v", id)))
			time.Sleep(500 * time.Millisecond)
			return nil
		}

		// Repositioning logic for Andariel on Normal difficulty only
		if s.CharacterCfg.Game.Difficulty == difficulty.Normal && monster.Name == npc.Andariel {
			distanceToMonster := pather.DistanceFromPoint(s.Data.PlayerUnit.Position, monster.Position)

			// Check if we are too close and have moves left in our sequence
			if distanceToMonster < SorceressLevelingMinDistance && s.andarielMoves < len(s.andarielSafePositions) {

				targetPos := s.andarielSafePositions[s.andarielMoves] // Get the next fixed position

				s.andarielMoves++
				s.Logger.Debug(fmt.Sprintf("Andariel is too close, moving away to fixed coordinate. Move %d of %d.", s.andarielMoves, len(s.andarielSafePositions)))

				step.MoveTo(targetPos, step.WithIgnoreMonsters())
				time.Sleep(time.Millisecond * 200)
				continue // Re-evaluate in the next loop
			}
		}

		// Determine Monster Type Flags
		isColdImmuneNotLightImmune := monster.IsImmune(stat.ColdImmune) && !monster.IsImmune(stat.LightImmune)
		_, isBossTarget := staticFieldTargets[monster.Name]
		isFireImmune := monster.IsImmune(stat.FireImmune)

		if !isBossTarget && !isColdImmuneNotLightImmune {
			for _, otherMonster := range s.Data.Monsters {
				if otherMonster.UnitID == monster.UnitID {
					continue // Skip the current target.
				}

				currentMonsterDistance := pather.DistanceFromPoint(s.Data.PlayerUnit.Position, monster.Position)
				otherMonsterDistance := pather.DistanceFromPoint(s.Data.PlayerUnit.Position, otherMonster.Position)

				proximityThreshold := SorceressLevelingMinDistance + 5

				if otherMonsterDistance < currentMonsterDistance && otherMonsterDistance < proximityThreshold {
					s.Logger.Info("New, closer monster detected. Reprioritizing target.",
						slog.String("old_target_id", fmt.Sprintf("%v", id)),
						slog.Int("old_distance", currentMonsterDistance),
						slog.String("new_target_id", fmt.Sprintf("%v", otherMonster.UnitID)),
						slog.Int("new_distance", otherMonsterDistance),
					)
					continue
				}
			}
		}

		currentMaxAttacksLoop := SorceressLevelingMaxAttacksLoop
		if _, isBoss := staticFieldTargets[monster.Name]; isBoss {
			currentMaxAttacksLoop = SorceressLevelingMaxBossAttacksLoop
		}

		if completedAttackLoops >= currentMaxAttacksLoop {
			s.Logger.Info(fmt.Sprintf("Max attack loops (%d) reached for monster", currentMaxAttacksLoop), slog.String("monsterID", fmt.Sprintf("%v", id)))
			return nil // Exit the loop and move on from this monster
		}

		lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

		var attackOption step.AttackOption = step.Distance(SorceressLevelingMinDistance, SorceressLevelingMaxDistance)
		var glacialSpikeAttackOption step.AttackOption = step.Distance(SorceressLevelingMinDistance, SorceressLevelingMaxDistance)

		isColdImmuneNotLightImmune = monster.IsImmune(stat.ColdImmune) && !monster.IsImmune(stat.LightImmune)
		if !isColdImmuneNotLightImmune {
			needsRepos, dangerousMonster := action.IsAnyEnemyAroundPlayer(SorceressLevelingDangerDistance)
			if needsRepos && time.Since(lastReposition) > time.Second*1 {
				lastReposition = time.Now()

				if s.isPlayerDead() {
					s.Logger.Info("Player detected as dead, stopping KillMonsterSequence.")
					return nil
				}

				targetID, found := monsterSelector(*s.Data)
				if !found {
					return nil
				}

				targetMonster, found := s.Data.Monsters.FindByID(targetID)
				if !found {
					s.Logger.Info("Target monster not found for repositioning, likely died.")
					return nil
				}

				s.Logger.Info(fmt.Sprintf("Dangerous monster detected at distance %d from player, repositioning...",
					pather.DistanceFromPoint(s.Data.PlayerUnit.Position, dangerousMonster.Position)))

				safePos, found := action.FindSafePosition(targetMonster, SorceressLevelingDangerDistance, SorceressLevelingSafeDistance, SorceressLevelingMinDistance, SorceressLevelingMaxDistance)
				if found {
					s.Logger.Info(fmt.Sprintf("Teleporting to safe position: %v", safePos))
					if s.Data.PlayerUnit.Skills[skill.Teleport].Level > 0 {
						if _, ok := s.Data.KeyBindings.KeyBindingForSkill(skill.Teleport); ok && !s.Data.PlayerUnit.States.HasState(state.Cooldown) {
							if s.isPlayerDead() {
								s.Logger.Info("Player detected as dead, stopping KillMonsterSequence.")
								return nil
							}
							step.MoveTo(safePos, step.WithIgnoreMonsters())
							time.Sleep(time.Millisecond * 200)
							continue
						} else {
							s.Logger.Debug("Teleport skill not found or on cooldown, cannot reposition.")
						}
					}
				} else {
					s.Logger.Info("Could not find safe position for repositioning.")
				}
			}
		}

		isBossTarget = false
		if _, isBoss := staticFieldTargets[monster.Name]; isBoss {
			isBossTarget = true
		}

		canCastStaticField := s.Data.PlayerUnit.Skills[skill.StaticField].Level > 0
		_, isStaticFieldBound := s.Data.KeyBindings.KeyBindingForSkill(skill.StaticField)
		_, isBlizzardBound := s.Data.KeyBindings.KeyBindingForSkill(skill.Blizzard)

		if isBossTarget {
			_, initialBlizzardCasted := s.blizzardInitialCastCompleted[id]
			if !initialBlizzardCasted {
				if isBlizzardBound && !s.Data.PlayerUnit.States.HasState(state.Cooldown) {
					if s.Data.PlayerUnit.Mode != mode.CastingSkill {
						s.Logger.Debug("Boss: Casting initial Blizzard.")
						step.SecondaryAttack(skill.Blizzard, id, 1, attackOption)
						s.blizzardInitialCastCompleted[id] = true
						time.Sleep(time.Millisecond * 100)
						continue
					} else {
						s.Logger.Debug("Boss: Player busy, waiting for initial Blizzard cast.")
						time.Sleep(time.Millisecond * 50)
						continue
					}
				}
			}

			if s.staticPhaseCompleted[id] {
				s.Logger.Debug("Boss: Static Field phase already completed, proceeding to main attack.")
			} else {
				monsterLifePercent := float64(monster.Stats[stat.Life]) / float64(monster.Stats[stat.MaxLife]) * 100
				requiredLifePercent := 0.0
				switch s.CharacterCfg.Game.Difficulty {
				case difficulty.Normal, difficulty.Nightmare:
					requiredLifePercent = 40.0
				case difficulty.Hell:
					requiredLifePercent = 70.0
				}

				if monsterLifePercent > requiredLifePercent {
					if canCastStaticField && isStaticFieldBound {
						distanceToMonster := pather.DistanceFromPoint(s.Data.PlayerUnit.Position, monster.Position)
						if distanceToMonster > StaticFieldEffectiveRange {
							s.Logger.Debug("Boss: Too far for Static Field, repositioning closer.",
								slog.String("target", fmt.Sprintf("%v", monster.Name)),
								slog.Int("distance", distanceToMonster),
								slog.Int("requiredRange", StaticFieldEffectiveRange),
							)
							step.MoveTo(monster.Position, step.WithIgnoreMonsters())
							time.Sleep(time.Millisecond * 100)
							continue
						}

						// We are in range, cast static field
						if s.Data.PlayerUnit.Mode != mode.CastingSkill {
							s.Logger.Debug("Boss: Using Static Field on target.")
							step.SecondaryAttack(skill.StaticField, id, 1, step.Distance(0, StaticFieldEffectiveRange))
							time.Sleep(time.Millisecond * 100)
							continue // Re-evaluate monster life from the top of the loop
						} else {
							s.Logger.Debug("Boss: Player busy, skipping Static Field for this tick.")
							time.Sleep(time.Millisecond * 50)
							continue
						}
					} else {
						// We have no Static Field skill, proceed with main attack
						s.staticPhaseCompleted[id] = true
						s.Logger.Info("Boss: Static Field skill not available. Transitioning to main attack.")
						// Fall through to the main attack logic below.
					}
				} else {
					s.staticPhaseCompleted[id] = true
					s.Logger.Info("Boss: Static Field threshold reached. Transitioning to main attack.")
					// Fall through to the main attack logic below.
				}
			}
		} else if isColdImmuneNotLightImmune {
			// Case 2: All Cold Immune monsters (excluding designated bosses)
			if s.Data.MercHPPercent() <= 0 {
				s.Logger.Info("Mercenary is dead, skipping attack on Cold Immune monster.", slog.String("monsterID", fmt.Sprintf("%v", id)))
				return nil
			}

			if s.blizzardCasts == nil {
				s.blizzardCasts = make(map[data.UnitID]int)
			}
			if s.blizzardPhaseCompleted == nil {
				s.blizzardPhaseCompleted = make(map[data.UnitID]bool)
			}

			// Determine how many times to cast Blizzard based on monster type
			blizzardCastsRequired := 1
			if monster.Type == data.MonsterTypeSuperUnique {
				blizzardCastsRequired = 2
			}

			// Try to cast the required number of Blizzards to initiate combat
			castCount := s.blizzardCasts[id]
			if castCount < blizzardCastsRequired && !s.blizzardPhaseCompleted[id] {
				if s.Data.PlayerUnit.Skills[skill.Blizzard].Level > 0 && isBlizzardBound && !s.Data.PlayerUnit.States.HasState(state.Cooldown) {
					s.Logger.Debug(fmt.Sprintf("CI: Casting initial Blizzard (cast %d/%d).", castCount+1, blizzardCastsRequired))
					step.SecondaryAttack(skill.Blizzard, id, 1, attackOption)
					s.blizzardCasts[id]++
					time.Sleep(time.Millisecond * 100)
					continue // Re-evaluate, now Static Phase will begin
				}
			} else {
				s.blizzardPhaseCompleted[id] = true
			}

			// After initial blizzard cast(s), telestomp with Static Field
			if canCastStaticField && isStaticFieldBound {
				distanceToMonster := pather.DistanceFromPoint(s.Data.PlayerUnit.Position, monster.Position)
				if distanceToMonster > StaticFieldEffectiveRange {
					s.Logger.Debug("CI: Too far for Static Field, repositioning closer.",
						slog.String("target", fmt.Sprintf("%v", monster.Name)),
						slog.Int("distance", distanceToMonster),
					)
					step.MoveTo(monster.Position, step.WithIgnoreMonsters())
					time.Sleep(time.Millisecond * 100)
					continue
				}

				if s.Data.PlayerUnit.Mode != mode.CastingSkill {
					s.Logger.Debug("CI: Using Static Field until dead.")
					step.SecondaryAttack(skill.StaticField, id, 1, step.Distance(0, StaticFieldEffectiveRange))
					time.Sleep(time.Millisecond * 100)
					continue
				} else {
					s.Logger.Debug("CI: Player busy, skipping Static Field.")
					time.Sleep(time.Millisecond * 50)
				}
			} else {
				s.Logger.Info("CI: Static Field skill not available. Transitioning to main attack.")
			}
		}

		// If none of the special Static Field/Blizzard cases apply, or if they fall through,
		// execution continues to the main attack logic below.

		// --- Main Attack Logic ---

		if lvl.Value < 12 && isFireImmune {
			s.Logger.Debug("Under level 12 and facing a Fire Immune monster, using primary attack.")
			step.PrimaryAttack(id, 1, false, step.Distance(1, SorceressLevelingMeleeDistance))
			previousUnitID = int(id)
			time.Sleep(time.Millisecond * 50)
			continue // Continue the loop to re-evaluate the monster
		}

		if s.Data.PlayerUnit.MPPercent() < 15 && lvl.Value < 12 {
			if s.isPlayerDead() {
				s.Logger.Info("Player detected as dead, stopping KillMonsterSequence.")
				return nil
			}
			s.Logger.Debug("Low mana, using primary attack (left-click skill, e.g., Attack/Fire Bolt)")
			step.PrimaryAttack(id, 1, false, step.Distance(1, SorceressLevelingMeleeDistance))
			previousUnitID = int(id)
			continue
		}

		if s.Data.PlayerUnit.Skills[skill.Blizzard].Level > 0 {
			if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Blizzard); found {
				if s.Data.PlayerUnit.States.HasState(state.Cooldown) {
					if s.Data.PlayerUnit.Skills[skill.GlacialSpike].Level > 0 {

						if s.isPlayerDead() {
							return nil
						}
						if s.Data.PlayerUnit.Mode != mode.CastingSkill {
							s.Logger.Debug("Blizzard on cooldown, attempting to cast Glacial Spike (Main).")
							step.PrimaryAttack(id, 2, true, glacialSpikeAttackOption)
						} else {
							s.Logger.Debug("Player is busy, waiting to cast Glacial Spike (Main).")
							time.Sleep(time.Millisecond * 50)
						}

					}
				} else {
					if s.isPlayerDead() {
						return nil
					}
					if s.Data.PlayerUnit.Mode != mode.CastingSkill {
						s.Logger.Debug("Using Blizzard (Main)")
						step.SecondaryAttack(skill.Blizzard, id, 1, attackOption)
					} else {
						s.Logger.Debug("Player is busy, waiting to cast Blizzard (Main).")
						time.Sleep(time.Millisecond * 50)
					}
				}
			}
		} else {
			currentAttackSkillUsed := skill.AttackSkill
			if s.Data.PlayerUnit.Skills[skill.Meteor].Level > 0 {
				if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Meteor); found {
					currentAttackSkillUsed = skill.Meteor
				}
			} else if s.Data.PlayerUnit.Skills[skill.FireBall].Level > 0 {
				if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.FireBall); found {
					currentAttackSkillUsed = skill.FireBall
				}
			} else if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.FireBolt); found {
				currentAttackSkillUsed = skill.FireBolt
			}

			if currentAttackSkillUsed != skill.AttackSkill {
				if s.isPlayerDead() {
					return nil
				}
				if s.Data.PlayerUnit.Mode != mode.CastingSkill {
					s.Logger.Debug(fmt.Sprintf("Using %v (fallback)", currentAttackSkillUsed))
					step.SecondaryAttack(currentAttackSkillUsed, id, 1, attackOption)
				} else {
					s.Logger.Debug(fmt.Sprintf("Player is busy, skipping %v (fallback) for this tick.", currentAttackSkillUsed))
					time.Sleep(time.Millisecond * 50)
				}
			} else {
				if s.isPlayerDead() {
					return nil
				}
				s.Logger.Debug("No secondary skills available, using primary attack (fallback)")
				step.PrimaryAttack(id, 1, false, step.Distance(1, SorceressLevelingMeleeDistance))
			}
		}

		previousUnitID = int(id)
		time.Sleep(time.Millisecond * 50)
	}
}

func (s *SorceressLeveling) killMonsterByName(id npc.ID, monsterType data.MonsterType, skipOnImmunities []stat.Resist) error {
	// while the monster is alive, keep attacking it
	for {
		// Check if the monster exists and get its current state
		if m, found := s.Data.Monsters.FindOne(id, monsterType); found {
			// If the monster's life is 0 or less, it's dead, so break the loop
			if m.Stats[stat.Life] <= 0 {
				fmt.Printf("Monster %s (ID: %d) is dead. Breaking attack loop.\n", m.Name, m.UnitID)
				break
			}

			err := s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
				if currentM, currentFound := d.Monsters.FindOne(id, monsterType); currentFound {
					return currentM.UnitID, true
				}
				return 0, false
			}, skipOnImmunities)

			if err != nil {
				// Handle errors from KillMonsterSequence, e.g., monster vanished during attack
				fmt.Printf("Error during KillMonsterSequence: %v. Breaking attack loop.\n", err)
				break
			}

			// Add a small delay to prevent busy-looping if the monster is very tanky
			time.Sleep(20 * time.Millisecond)

		} else {
			// Monster not found, it might have died or moved out of detection range
			fmt.Printf("Monster (ID: %d, Type: %s) not found. Assuming it's dead or vanished. Breaking loop.\n", id, monsterType)
			break
		}
	}
	return nil
}

func (s SorceressLeveling) BuffSkills() []skill.ID {
	skillsList := make([]skill.ID, 0)
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.EnergyShield); found {
		skillsList = append(skillsList, skill.EnergyShield)
	}

	armors := []skill.ID{skill.ChillingArmor, skill.ShiverArmor, skill.FrozenArmor}
	for _, armor := range armors {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(armor); found {
			skillsList = append(skillsList, armor)
			return skillsList
		}
	}

	return skillsList
}

func (s SorceressLeveling) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

func (s SorceressLeveling) ShouldResetSkills() bool {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
	if lvl.Value == 24 && s.Data.PlayerUnit.Skills[skill.FireBall].Level > 7 && s.Data.PlayerUnit.Skills[skill.FireBolt].Level > 7 {
		s.Logger.Info("Respecing to Blizzard: Level 32+ and FireBall level > 9")
		return true
	}
	return false
}

func (s SorceressLeveling) SkillsToBind() (skill.ID, []skill.ID) {
	level, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	skillBindings := []skill.ID{
		skill.FireBolt,
	}

	if level.Value >= 2 {
		skillBindings = append(skillBindings, skill.FrozenArmor)
	}
	if level.Value >= 6 {
		skillBindings = append(skillBindings, skill.StaticField)
	}
	if level.Value >= 18 {
		skillBindings = append(skillBindings, skill.Teleport)
	}

	if level.Value >= 24 {
		skillBindings = append(skillBindings, skill.Blizzard)
	} else if s.Data.PlayerUnit.Skills[skill.Meteor].Level > 0 {
		skillBindings = append(skillBindings, skill.Meteor)
	} else if s.Data.PlayerUnit.Skills[skill.Hydra].Level > 0 {
		skillBindings = append(skillBindings, skill.Hydra)
	} else if s.Data.PlayerUnit.Skills[skill.FireBall].Level > 0 {
		skillBindings = append(skillBindings, skill.FireBall)
	}

	if s.Data.PlayerUnit.Skills[skill.EnergyShield].Level > 0 {
		skillBindings = append(skillBindings, skill.EnergyShield)
	}

	if s.Data.PlayerUnit.Skills[skill.BattleCommand].Level > 0 {
		skillBindings = append(skillBindings, skill.BattleCommand)
	}

	if s.Data.PlayerUnit.Skills[skill.BattleOrders].Level > 0 {
		skillBindings = append(skillBindings, skill.BattleOrders)
	}

	mainSkill := skill.AttackSkill
	if level.Value >= 24 {
		mainSkill = skill.GlacialSpike
	}

	_, found := s.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	if found {
		skillBindings = append(skillBindings, skill.TomeOfTownPortal)
	}

	s.Logger.Info("Skills bound", "mainSkill", mainSkill, "skillBindings", skillBindings)
	return mainSkill, skillBindings

}

func (s SorceressLeveling) StatPoints() []context.StatAllocation {

	targets := []context.StatAllocation{

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

	return targets
}

func (s SorceressLeveling) SkillPoints() []skill.ID {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	var activeSkillSequence []skill.ID
	if lvl.Value < 24 {
		activeSkillSequence = fireSkillSequence
	} else {
		activeSkillSequence = blizzardSkillSequence
	}

	return activeSkillSequence
}

func (s SorceressLeveling) KillCountess() error {
	return s.killMonsterByName(npc.DarkStalker, data.MonsterTypeSuperUnique, nil)
}

func (s SorceressLeveling) KillAndariel() error {

	if s.CharacterCfg.Game.Difficulty == difficulty.Hell {

		originalBackToTownCfg := s.CharacterCfg.BackToTown

		s.CharacterCfg.BackToTown.MercDied = true

		defer func() {
			s.CharacterCfg.BackToTown = originalBackToTownCfg
			s.Logger.Info("Restored original back-to-town checks after Duriel fight.")
		}()
	}
	return s.killMonsterByName(npc.Andariel, data.MonsterTypeUnique, nil)
}

func (s SorceressLeveling) KillSummoner() error {
	originalBackToTownCfg := s.CharacterCfg.BackToTown
	s.CharacterCfg.BackToTown.NoHpPotions = false
	s.CharacterCfg.BackToTown.NoMpPotions = false
	s.CharacterCfg.BackToTown.EquipmentBroken = false
	s.CharacterCfg.BackToTown.MercDied = false

	defer func() {
		s.CharacterCfg.BackToTown = originalBackToTownCfg
		s.Logger.Info("Restored original back-to-town checks after Mephisto fight.")
	}()

	return s.killMonsterByName(npc.Summoner, data.MonsterTypeUnique, nil)
}

func (s SorceressLeveling) KillDuriel() error {

	return s.killMonsterByName(npc.Duriel, data.MonsterTypeUnique, nil)

}

func (s SorceressLeveling) KillCouncil() error {

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

func (s SorceressLeveling) KillMephisto() error {

	if s.CharacterCfg.Character.SorceressLeveling.UseStaticOnMephisto {

		var attackOption step.AttackOption = step.Distance(SorceressLevelingMinDistance, SorceressLevelingMaxDistance)

		staticFieldRange := step.Distance(0, StaticFieldEffectiveRange)
		err := step.MoveTo(data.Position{X: 17565, Y: 8065}, step.WithIgnoreMonsters())

		monster, found := s.Data.Monsters.FindOne(npc.Mephisto, data.MonsterTypeUnique)
		if !found {
			s.Logger.Error("Mephisto not found at initial approach, aborting kill.")
			return nil
		}

		if s.Data.PlayerUnit.Skills[skill.Blizzard].Level > 0 {
			s.Logger.Info("Applying initial Blizzard cast.")
			step.SecondaryAttack(skill.Blizzard, monster.UnitID, 1, attackOption)
			time.Sleep(time.Millisecond * 300) // Wait for cast to register and apply chill
		}

		canCastStaticField := s.Data.PlayerUnit.Skills[skill.StaticField].Level > 0
		_, isStaticFieldBound := s.Data.KeyBindings.KeyBindingForSkill(skill.StaticField)

		if canCastStaticField && isStaticFieldBound {
			s.Logger.Info("Starting aggressive Static Field phase on Mephisto.")

			requiredLifePercent := 0.0
			switch s.CharacterCfg.Game.Difficulty {
			case difficulty.Normal, difficulty.Nightmare:
				requiredLifePercent = 40.0
			case difficulty.Hell:
				requiredLifePercent = 70.0
			}

			maxStaticAttacks := 50
			staticAttackCount := 0

			for staticAttackCount < maxStaticAttacks {
				monster, found = s.Data.Monsters.FindOne(npc.Mephisto, data.MonsterTypeUnique)
				if !found || monster.Stats[stat.Life] <= 0 {
					s.Logger.Info("Mephisto died or vanished during Static Phase.")
					break
				}

				monsterLifePercent := float64(monster.Stats[stat.Life]) / float64(monster.Stats[stat.MaxLife]) * 100

				if monsterLifePercent <= requiredLifePercent {
					s.Logger.Info(fmt.Sprintf("Mephisto life threshold (%.0f%%) reached. Transitioning to moat movement.", requiredLifePercent))
					break
				}

				distanceToMonster := pather.DistanceFromPoint(s.Data.PlayerUnit.Position, monster.Position)

				if distanceToMonster > StaticFieldEffectiveRange && s.Data.PlayerUnit.Skills[skill.Teleport].Level > 0 {
					s.Logger.Debug("Mephisto too far for Static Field, repositioning closer.")

					step.MoveTo(monster.Position, step.WithIgnoreMonsters())
					utils.Sleep(150)
					continue
				}

				if s.Data.PlayerUnit.Mode != mode.CastingSkill {
					s.Logger.Debug("Using Static Field on Mephisto.")
					step.SecondaryAttack(skill.StaticField, monster.UnitID, 1, staticFieldRange)
					time.Sleep(time.Millisecond * 150)
				} else {
					time.Sleep(time.Millisecond * 50)
				}
				staticAttackCount++
			}
		} else {
			s.Logger.Info("Static Field not available or bound, skipping Static Phase.")
		}

		err = step.MoveTo(data.Position{X: 17563, Y: 8072}, step.WithIgnoreMonsters())
		if err != nil {
			return err
		}

	}

	if !s.CharacterCfg.Character.SorceressLeveling.UseMoatTrick {

		return s.killMonsterByName(npc.Mephisto, data.MonsterTypeUnique, nil)

	} else {

		ctx := context.Get()
		opts := step.Distance(15, 80)
		ctx.ForceAttack = true

		defer func() {
			ctx.ForceAttack = false
		}()

		type positionAndWaitTime struct {
			x        int
			y        int
			duration int
		}

		// Move to initial position
		utils.Sleep(350)
		err := step.MoveTo(data.Position{X: 17563, Y: 8072}, step.WithIgnoreMonsters())
		if err != nil {
			return err
		}

		utils.Sleep(350)

		// Initial movement sequence
		initialPositions := []positionAndWaitTime{
			{17575, 8086, 350}, {17584, 8088, 1200},
			{17600, 8090, 550}, {17609, 8090, 2500},
		}

		for _, pos := range initialPositions {
			err := step.MoveTo(data.Position{X: pos.x, Y: pos.y}, step.WithIgnoreMonsters())
			if err != nil {
				return err
			}
			utils.Sleep(pos.duration)
		}

		// Clear area around position
		err = action.ClearAreaAroundPosition(data.Position{X: 17609, Y: 8090}, 10, data.MonsterAnyFilter())
		if err != nil {
			return err
		}

		err = step.MoveTo(data.Position{X: 17609, Y: 8090}, step.WithIgnoreMonsters())
		if err != nil {
			return err
		}

		maxAttack := 100
		attackCount := 0

		for attackCount < maxAttack {
			ctx.PauseIfNotPriority()

			monster, found := s.Data.Monsters.FindOne(npc.Mephisto, data.MonsterTypeUnique)

			if !found {
				return nil
			}

			if s.Data.PlayerUnit.States.HasState(state.Cooldown) {
				step.PrimaryAttack(monster.UnitID, 2, true, opts)
				utils.Sleep(50)
			}

			step.SecondaryAttack(skill.Blizzard, monster.UnitID, 1, opts)
			utils.Sleep(100)
			attackCount++
		}
		return nil

	}
}

func (s SorceressLeveling) KillIzual() error {

	return s.killMonsterByName(npc.Izual, data.MonsterTypeUnique, nil)
}

func (s SorceressLeveling) KillDiablo() error {
	timeout := time.Second * 20
	startTime := time.Now()
	diabloFound := false

	for {
		if time.Since(startTime) > timeout && !diabloFound {
			s.Logger.Error("Diablo was not found, timeout reached")
			return nil
		}

		diablo, found := s.Data.Monsters.FindOne(npc.Diablo, data.MonsterTypeUnique)
		if !found || diablo.Stats[stat.Life] <= 0 {
			if diabloFound {
				return nil
			}
			time.Sleep(200)
			continue
		}

		diabloFound = true
		s.Logger.Info("Diablo detected, attacking")

		originalBackToTownCfg := s.CharacterCfg.BackToTown
		s.CharacterCfg.BackToTown.NoHpPotions = false
		s.CharacterCfg.BackToTown.NoMpPotions = false
		s.CharacterCfg.BackToTown.EquipmentBroken = false
		s.CharacterCfg.BackToTown.MercDied = false

		defer func() {
			s.CharacterCfg.BackToTown = originalBackToTownCfg
			s.Logger.Info("Restored original back-to-town checks after Diablo fight.")
		}()

		return s.killMonsterByName(npc.Diablo, data.MonsterTypeUnique, nil)
	}
}

func (s SorceressLeveling) KillPindle() error {
	return s.killMonsterByName(npc.DefiledWarrior, data.MonsterTypeSuperUnique, s.CharacterCfg.Game.Pindleskin.SkipOnImmunities)
}

func (s SorceressLeveling) KillNihlathak() error {
	return s.killMonsterByName(npc.Nihlathak, data.MonsterTypeSuperUnique, nil)
}

func (s SorceressLeveling) KillAncients() error {
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

		s.killMonsterByName(foundMonster.Name, data.MonsterTypeSuperUnique, nil)

	}

	s.CharacterCfg.BackToTown = originalBackToTownCfg
	s.Logger.Info("Restored original back-to-town checks after Ancients fight.")
	return nil
}

func (s SorceressLeveling) KillBaal() error {

	return s.killMonsterByName(npc.BaalCrab, data.MonsterTypeUnique, nil)
}

func (s SorceressLeveling) GetAdditionalRunewords() []string {
	additionalRunewords := action.GetCastersCommonRunewords()
	return additionalRunewords
}
