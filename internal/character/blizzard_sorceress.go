package character

import (
	"fmt"
	"log/slog"
	"math"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/mode"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/health"
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	sorceressMaxAttacksLoop         = 40
	minBlizzSorceressAttackDistance = 1  // Blizzard has no minimum range
	maxBlizzSorceressAttackDistance = 18 // Maximum cast range for Blizzard
	dangerDistance                  = 5  // Monsters closer than this trigger repositioning
	safeDistance                    = 10 // Distance to teleport away to

	// Blizzard Mechanics
	blizzardMaxRange        = maxBlizzSorceressAttackDistance // Maximum cast range for Blizzard
	blizzardEffectiveRadius = 7                               // AoE damage radius when Blizzard hits ground

	// Lead Targeting (Prediction) - optimized for CPU
	baseLeadDistance        = 3 // Base tiles to predict monster movement
	fastMonsterLeadDistance = 5 // Lead distance for fast-moving monsters
	chasingMonsterLeadBonus = 1 // Extra lead if monster is chasing player

	// Pack Detection - reduced for multi-instance performance
	packBuildRadius        = 12 // Radius to gather monsters into a pack
	eliteAnchorWeight      = 3  // Weight multiplier for elite monsters
	championAnchorWeight   = 2  // Weight multiplier for champion monsters
	packCenterSearchRadius = 5  // Search radius for optimal cast position (CPU optimized)

	// Repositioning - anti-pingpong
	repositionCooldown      = 2.0 // Seconds between repositions
	repositionMinDistance   = 6   // Minimum distance to move when repositioning
	backwardsRepositionBias = 1.5 // Prefer moving AWAY from unexplored areas
	merceAggroWaitTime      = 0.5 // Seconds to wait for mercenary aggro
	maxRepositionsPerTarget = 1   // Max repositions per target

	// Safety
	minSafeMonsterDistance = 3 // Never teleport closer than this to any monster
)

type BlizzardSorceress struct {
	BaseCharacter
}

func (s BlizzardSorceress) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s BlizzardSorceress) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{skill.Blizzard, skill.Teleport, skill.TomeOfTownPortal, skill.ShiverArmor, skill.StaticField}
	missingKeybindings := []skill.ID{}

	for _, cskill := range requireKeybindings {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(cskill); !found {
			switch cskill {
			// Since we can have one of 3 armors:
			case skill.ShiverArmor:
				_, found1 := s.Data.KeyBindings.KeyBindingForSkill(skill.FrozenArmor)
				_, found2 := s.Data.KeyBindings.KeyBindingForSkill(skill.ChillingArmor)
				if !found1 && !found2 {
					missingKeybindings = append(missingKeybindings, skill.ShiverArmor)
				}
			default:
				missingKeybindings = append(missingKeybindings, cskill)
			}
		}
	}

	if len(missingKeybindings) > 0 {
		s.Logger.Debug("There are missing required key bindings.", slog.Any("Bindings", missingKeybindings))
	}

	return missingKeybindings
}

func (s BlizzardSorceress) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	lastRepositionTime := time.Time{}
	repositionCountForTarget := 0

	for {
		context.Get().PauseIfNotPriority()

		// Death check
		if s.Context.Data.PlayerUnit.IsDead() {
			s.Logger.Info("Player detected as dead during KillMonsterSequence, stopping actions.")
			time.Sleep(500 * time.Millisecond)
			return health.ErrDied
		}

		// Step 1: Get target monster.
		// IMPORTANT:
		// - If a selector is provided (boss / kill-only-elite runs), we MUST only target what it returns.
		// - If no selector is provided, fall back to nearest-in-range.
		var targetMonster data.Monster
		var targetFound bool

		if monsterSelector != nil {
			id, found := monsterSelector(*s.Data)
			if !found {
				return nil
			}

			targetMonster, targetFound = s.Data.Monsters.FindByID(id)
			if !targetFound {
				return nil
			}
		} else {
			targetMonster, targetFound = s.findNearestMonster()
			if !targetFound {
				return nil
			}
		}

		// Targeting should only consider monsters inside Blizzard cast range.
		// If the closest/selected target is outside max Blizzard range, exit so the caller can continue navigation (e.g. seals).
		if gridDistance(s.Data.PlayerUnit.Position, targetMonster.Position) > blizzardMaxRange {
			// NOTE:
			// We intentionally stop combat outside Blizzard cast range so the caller can continue navigation
			// (e.g. clicking seals instead of clearing the whole Chaos Sanctuary).
			//
			// Seal elites (Vizier / Lord De Seis / Infector) can spawn "far" and must still be killable;
			// allow them so the combat routine can reposition/teleport into casting range.
			if !action.IsMonsterSealElite(targetMonster) {
				return nil
			}
		}

		// Reset reposition counter if target changed
		if previousUnitID != int(targetMonster.UnitID) {
			completedAttackLoops = 0
			repositionCountForTarget = 0
			previousUnitID = int(targetMonster.UnitID)
		}

		// Pre-battle checks (immunities, etc.)
		if !s.preBattleChecks(targetMonster.UnitID, skipOnImmunities) {
			return nil
		}

		// Max attacks safety limit
		if completedAttackLoops >= sorceressMaxAttacksLoop {
			s.Logger.Debug("Max attack loops reached for target, moving to next")
			return nil
		}

		// Step 2: Build pack around target
		pack := s.buildPack(targetMonster.Position)

		// Step 3: Check if we need repositioning (backwards, defensive)
		// ONLY reposition if monsters are VERY close AND we have teleport
		needsRepos, dangerousMonster := s.needsRepositioning()
		canReposition := time.Since(lastRepositionTime) > time.Duration(repositionCooldown*float64(time.Second))
		hasTeleport := s.Data.PlayerUnit.Skills[skill.Teleport].Level > 0

		if needsRepos && canReposition && hasTeleport && repositionCountForTarget < maxRepositionsPerTarget {
			distToDanger := gridDistance(s.Data.PlayerUnit.Position, dangerousMonster.Position)

			s.Logger.Debug("Dangerous monsters detected, repositioning backwards",
				slog.Int("dangerDist", distToDanger))

			// Find backwards position (away from next pack, towards cleared area)
			backwardsPos, found := s.findBackwardsPosition(targetMonster)
			if found {
				// Check if backwards position is significantly different from current position
				distToNewPos := gridDistance(s.Data.PlayerUnit.Position, backwardsPos)
				if distToNewPos >= repositionMinDistance {
					err := step.MoveTo(backwardsPos, step.WithIgnoreMonsters())
					if err == nil {
						lastRepositionTime = time.Now()
						repositionCountForTarget++

						// Wait for mercenary to grab aggro after repositioning
						s.Logger.Debug("Waiting for mercenary to grab aggro",
							slog.Float64("waitSeconds", merceAggroWaitTime))
						time.Sleep(time.Duration(merceAggroWaitTime * float64(time.Second)))
					} else {
						s.Logger.Debug("Backwards reposition failed", slog.String("error", err.Error()))
					}
				} else {
					s.Logger.Debug("Backwards position too close to current position, skipping reposition",
						slog.Int("distance", distToNewPos))
				}
			} else {
				s.Logger.Debug("Could not find valid backwards position, continuing attack")
			}
		}

		// Step 4: Find optimal Blizzard cast position
		blizzardCastPos, posFound := s.findOptimalBlizzardCastPosition(pack)
		if !posFound {
			// Fallback: try to cast at target's current position
			if s.validateBlizzardLoS(targetMonster.Position) {
				blizzardCastPos = targetMonster.Position
				posFound = true
			} else {
				s.Logger.Debug("No valid Blizzard cast position found, using target position anyway")
				blizzardCastPos = targetMonster.Position
				posFound = true
			}
		}

		// Step 5: If on cooldown, use Glacial Spike as filler
		if s.Data.PlayerUnit.States.HasState(state.Cooldown) {
			attackOpts := step.StationaryDistance(1, 20)
			// Use Glacial Spike as filler during cooldown
			if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.GlacialSpike); found {
				step.SecondaryAttack(skill.GlacialSpike, targetMonster.UnitID, 1, attackOpts)
			} else {
				// Fallback to Ice Blast if Glacial Spike not available
				step.SecondaryAttack(skill.IceBlast, targetMonster.UnitID, 1, attackOpts)
			}
		}

		// Step 6: ALWAYS cast Blizzard at optimal position
		// Find monster closest to optimal cast position to use as target for SecondaryAttack
		castTargetMonster := targetMonster
		minDistToCastPos := gridDistance(targetMonster.Position, blizzardCastPos)

		for _, m := range pack {
			if m.Stats[stat.Life] <= 0 {
				continue
			}
			dist := gridDistance(m.Position, blizzardCastPos)
			if dist < minDistToCastPos {
				minDistToCastPos = dist
				castTargetMonster = m
			}
		}

		// Cast Blizzard using SecondaryAttack (target-based, but we've chosen best target for position)
		attackOpts := step.StationaryDistance(minBlizzSorceressAttackDistance, maxBlizzSorceressAttackDistance)
		step.SecondaryAttack(skill.Blizzard, castTargetMonster.UnitID, 1, attackOpts)

		s.Logger.Debug("Cast Blizzard",
			slog.Int("targetX", blizzardCastPos.X),
			slog.Int("targetY", blizzardCastPos.Y),
			slog.Int("packSize", len(pack)),
			slog.Any("targetMonster", castTargetMonster.Name))

		utils.Sleep(50) // Small delay

		// Increment loop counter
		completedAttackLoops++
	}
}

func (s BlizzardSorceress) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}

		return m.UnitID, true
	}, nil)
}

func (s BlizzardSorceress) killMonsterByName(id npc.ID, monsterType data.MonsterType, skipOnImmunities []stat.Resist) error {
	// while the monster is alive, keep attacking it
	for {
		if m, found := s.Data.Monsters.FindOne(id, monsterType); found {
			if m.Stats[stat.Life] <= 0 {
				break
			}

			s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
				if m, found := d.Monsters.FindOne(id, monsterType); found {
					return m.UnitID, true
				}

				return 0, false
			}, skipOnImmunities)
		} else {
			break
		}
	}
	return nil
}

func (s BlizzardSorceress) BuffSkills() []skill.ID {
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

func (s BlizzardSorceress) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

func (s BlizzardSorceress) KillCountess() error {
	return s.killMonsterByName(npc.DarkStalker, data.MonsterTypeSuperUnique, nil)
}

func (s BlizzardSorceress) KillAndariel() error {
	return s.killMonsterByName(npc.Andariel, data.MonsterTypeUnique, nil)
}

func (s BlizzardSorceress) KillSummoner() error {
	return s.killMonsterByName(npc.Summoner, data.MonsterTypeUnique, nil)
}

func (s BlizzardSorceress) KillDuriel() error {
	return s.killMonsterByName(npc.Duriel, data.MonsterTypeUnique, nil)
}

func (s BlizzardSorceress) KillCouncil() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		// Exclude monsters that are not council members
		var councilMembers []data.Monster
		var coldImmunes []data.Monster
		for _, m := range d.Monsters.Enemies() {
			if m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3 {
				if m.IsImmune(stat.ColdImmune) {
					coldImmunes = append(coldImmunes, m)
				} else {
					councilMembers = append(councilMembers, m)
				}
			}
		}

		councilMembers = append(councilMembers, coldImmunes...)

		for _, m := range councilMembers {
			return m.UnitID, true
		}

		return 0, false
	}, nil)
}

/*
func (s BlizzardSorceress) KillMephisto() error {
    // Find Mephisto
    mephisto, found := s.Data.Monsters.FindOne(npc.Mephisto, data.MonsterTypeUnique)
    if !found || mephisto.Stats[stat.Life] <= 0 {
        // If Mephisto is not found or already dead, just return (or handle as needed)
        return nil
    }

    s.Logger.Info("Mephisto detected, applying Static Field")

    // Apply Static Field to Mephisto
    // The parameters (unitID, attacks, distance options) are similar to Diablo's Static Field usage
    _ = step.SecondaryAttack(skill.StaticField, mephisto.UnitID, 5, step.Distance(3, 8))

    // Now, proceed with the regular monster killing sequence (Blizzard etc.)
    return s.killMonsterByName(npc.Mephisto, data.MonsterTypeUnique, nil)
}
*/

func (s BlizzardSorceress) KillMephisto() error {

	if s.CharacterCfg.Character.BlizzardSorceress.UseStaticOnMephisto {

		staticFieldRange := step.Distance(0, 4)
		var attackOption step.AttackOption = step.Distance(SorceressLevelingMinDistance, SorceressLevelingMaxDistance)
		err := step.MoveTo(data.Position{X: 17563, Y: 8072}, step.WithIgnoreMonsters())
		if err != nil {
			return err
		}

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

	if !s.CharacterCfg.Character.BlizzardSorceress.UseMoatTrick {

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

func (s BlizzardSorceress) KillIzual() error {
	m, _ := s.Data.Monsters.FindOne(npc.Izual, data.MonsterTypeUnique)
	_ = step.SecondaryAttack(skill.StaticField, m.UnitID, 4, step.Distance(5, 8))

	return s.killMonsterByName(npc.Izual, data.MonsterTypeUnique, nil)
}

func (s BlizzardSorceress) KillDiablo() error {
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
			// Already dead
			if diabloFound {
				return nil
			}

			// Keep waiting...
			time.Sleep(200 * time.Millisecond)
			continue
		}

		diabloFound = true
		s.Logger.Info("Diablo detected, attacking")

		_ = step.SecondaryAttack(skill.StaticField, diablo.UnitID, 5, step.Distance(3, 8))

		return s.killMonsterByName(npc.Diablo, data.MonsterTypeUnique, nil)
	}
}

func (s BlizzardSorceress) KillPindle() error {
	return s.killMonsterByName(npc.DefiledWarrior, data.MonsterTypeSuperUnique, s.CharacterCfg.Game.Pindleskin.SkipOnImmunities)
}

func (s BlizzardSorceress) KillNihlathak() error {
	return s.killMonsterByName(npc.Nihlathak, data.MonsterTypeSuperUnique, nil)
}

func (s BlizzardSorceress) KillBaal() error {
	m, _ := s.Data.Monsters.FindOne(npc.BaalCrab, data.MonsterTypeUnique)
	step.SecondaryAttack(skill.StaticField, m.UnitID, 4, step.Distance(5, 8))

	return s.killMonsterByName(npc.BaalCrab, data.MonsterTypeUnique, nil)
}

// buildPack: Gather all monsters within packBuildRadius into a single pack
func (s BlizzardSorceress) buildPack(anchor data.Position) []data.Monster {
	pack := []data.Monster{}
	for _, m := range s.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}
		if gridDistance(anchor, m.Position) <= packBuildRadius {
			pack = append(pack, m)
		}
	}
	return pack
}

// findPackCenter: Calculate weighted center of pack (elites have higher weight)
func (s BlizzardSorceress) findPackCenter(pack []data.Monster) data.Position {
	if len(pack) == 0 {
		return s.Data.PlayerUnit.Position
	}

	sumX, sumY, totalWeight := 0, 0, 0

	for _, m := range pack {
		weight := 1

		// Elites and champions are more important - anchor to them
		if m.IsElite() {
			weight = eliteAnchorWeight
		} else if m.Type == data.MonsterTypeChampion || m.Type == data.MonsterTypeMinion {
			weight = championAnchorWeight
		}

		sumX += m.Position.X * weight
		sumY += m.Position.Y * weight
		totalWeight += weight
	}

	if totalWeight == 0 {
		return pack[0].Position
	}

	return data.Position{
		X: sumX / totalWeight,
		Y: sumY / totalWeight,
	}
}

// predictMonsterPosition: Predict where monster will be when Blizzard lands
func (s BlizzardSorceress) predictMonsterPosition(monster data.Monster) data.Position {
	// If monster is dead, no prediction needed
	if monster.Stats[stat.Life] <= 0 {
		return monster.Position
	}

	playerPos := s.Data.PlayerUnit.Position

	// Calculate vector from monster to player (assume monster is chasing)
	dirX := playerPos.X - monster.Position.X
	dirY := playerPos.Y - monster.Position.Y

	// Calculate distance
	length := math.Sqrt(float64(dirX*dirX + dirY*dirY))
	if length == 0 {
		return monster.Position
	}

	// Determine lead distance based on monster distance
	leadDist := baseLeadDistance
	distToPlayer := gridDistance(monster.Position, playerPos)

	// Fast monsters need more lead (simplified check for CPU)
	if monster.Name == npc.SerpentMagus || monster.Name == npc.SaberCat {
		leadDist = fastMonsterLeadDistance
	}

	// If monster is very close (chasing), add bonus lead
	if distToPlayer < dangerDistance {
		leadDist += chasingMonsterLeadBonus
	}

	// Calculate predicted position
	normalizedX := float64(dirX) / length
	normalizedY := float64(dirY) / length

	predictedX := monster.Position.X + int(normalizedX*float64(leadDist))
	predictedY := monster.Position.Y + int(normalizedY*float64(leadDist))

	predictedPos := data.Position{X: predictedX, Y: predictedY}

	// Validate predicted position is walkable, otherwise use original
	if !s.Data.AreaData.IsWalkable(predictedPos) {
		return monster.Position
	}

	return predictedPos
}

// validateBlizzardLoS: Check if we have line of sight to cast Blizzard at target position
func (s BlizzardSorceress) validateBlizzardLoS(targetPos data.Position) bool {
	ctx := context.Get()
	playerPos := s.Data.PlayerUnit.Position

	// Check distance first
	distance := gridDistance(playerPos, targetPos)
	if distance > blizzardMaxRange {
		return false
	}

	// Check line of sight using pathfinder
	return ctx.PathFinder.LineOfSight(playerPos, targetPos)
}

// findBackwardsPosition: Find safe position BACKWARDS (away from next pack, towards cleared area)
func (s BlizzardSorceress) findBackwardsPosition(targetMonster data.Monster) (data.Position, bool) {
	ctx := context.Get()
	playerPos := s.Data.PlayerUnit.Position

	// Calculate vector FROM target TO player (backwards direction)
	vectorX := playerPos.X - targetMonster.Position.X
	vectorY := playerPos.Y - targetMonster.Position.Y

	// Normalize
	length := math.Sqrt(float64(vectorX*vectorX + vectorY*vectorY))
	if length == 0 {
		return data.Position{}, false
	}

	normalizedX := float64(vectorX) / length
	normalizedY := float64(vectorY) / length

	type scoredPos struct {
		pos   data.Position
		score float64
	}

	candidates := make([]scoredPos, 0, 50) // Pre-allocate for performance

	// Try positions BEHIND player (away from target) - CPU optimized, fewer candidates
	for distance := repositionMinDistance; distance <= safeDistance; distance += 2 { // Skip every other distance
		// Main backwards direction
		backX := playerPos.X + int(normalizedX*float64(distance))
		backY := playerPos.Y + int(normalizedY*float64(distance))

		// Add some variation around the backwards direction (reduced grid)
		for offsetX := -2; offsetX <= 2; offsetX++ {
			for offsetY := -2; offsetY <= 2; offsetY++ {
				candidatePos := data.Position{
					X: backX + offsetX,
					Y: backY + offsetY,
				}

				// Must be walkable
				if !s.Data.AreaData.IsWalkable(candidatePos) {
					continue
				}

				// Must have LoS to target
				if !ctx.PathFinder.LineOfSight(candidatePos, targetMonster.Position) {
					continue
				}

				// Calculate minimum distance to any monster
				minMonsterDist := math.MaxFloat64
				for _, m := range s.Data.Monsters.Enemies() {
					if m.Stats[stat.Life] <= 0 {
						continue
					}
					dist := float64(gridDistance(candidatePos, m.Position))
					if dist < minMonsterDist {
						minMonsterDist = dist
					}
				}

				// Skip if too close to monsters
				if minMonsterDist < minSafeMonsterDistance {
					continue
				}

				// Calculate distance to target
				distToTarget := gridDistance(candidatePos, targetMonster.Position)

				// Skip if completely out of reasonable range
				if distToTarget > maxBlizzSorceressAttackDistance {
					continue
				}

				// Calculate score (simplified for CPU performance):
				// - Higher safety from monsters (distance)
				// - Prefer backwards direction (bias multiplier)
				distFromPlayer := float64(gridDistance(candidatePos, playerPos))

				score := minMonsterDist*3.0 + distFromPlayer*backwardsRepositionBias

				candidates = append(candidates, scoredPos{
					pos:   candidatePos,
					score: score,
				})
			}
		}
	}

	if len(candidates) == 0 {
		return data.Position{}, false
	}

	// Sort by score (highest first)
	sort.Slice(candidates, func(i, j int) bool {
		return candidates[i].score > candidates[j].score
	})

	return candidates[0].pos, true
}

// findNearestMonster: Return nearest monster to player (for nearest-first attack strategy)
func (s BlizzardSorceress) findNearestMonster() (data.Monster, bool) {
	playerPos := s.Data.PlayerUnit.Position
	var nearest data.Monster
	minDistance := math.MaxInt32
	found := false

	for _, m := range s.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}

		dist := gridDistance(playerPos, m.Position)

		// Only consider monsters inside Blizzard cast range.
		if dist > blizzardMaxRange {
			continue
		}

		// Check if this is closer than current nearest
		if dist < minDistance {
			minDistance = dist
			nearest = m
			found = true
		}
	}

	return nearest, found
}

// findOptimalBlizzardCastPosition: Find best position to cast Blizzard for pack coverage
func (s BlizzardSorceress) findOptimalBlizzardCastPosition(pack []data.Monster) (data.Position, bool) {
	if len(pack) == 0 {
		return data.Position{}, false
	}

	// Calculate pack center (weighted by elites)
	packCenter := s.findPackCenter(pack)

	// For single monster, predict their movement
	if len(pack) == 1 {
		predicted := s.predictMonsterPosition(pack[0])
		if s.validateBlizzardLoS(predicted) {
			return predicted, true
		}
		// Fallback to current position if prediction failed
		if s.validateBlizzardLoS(pack[0].Position) {
			return pack[0].Position, true
		}
		return data.Position{}, false
	}

	// For multiple monsters, find position that hits the most
	type scoredPos struct {
		pos      data.Position
		hitCount int
		score    float64
	}

	candidates := make([]scoredPos, 0, 50) // Pre-allocate

	// Search around pack center (CPU optimized - smaller grid, skip positions)
	for offsetX := -packCenterSearchRadius; offsetX <= packCenterSearchRadius; offsetX += 2 { // Skip every other
		for offsetY := -packCenterSearchRadius; offsetY <= packCenterSearchRadius; offsetY += 2 {
			candidatePos := data.Position{
				X: packCenter.X + offsetX,
				Y: packCenter.Y + offsetY,
			}

			// Must have LoS from player
			if !s.validateBlizzardLoS(candidatePos) {
				continue
			}

			// Count how many monsters would be hit
			hitCount := 0
			for _, m := range pack {
				// Predict where monster will be
				predictedPos := s.predictMonsterPosition(m)
				dist := gridDistance(candidatePos, predictedPos)
				if dist <= blizzardEffectiveRadius {
					hitCount++
				}
			}

			// Calculate distance to pack center
			distToCenter := gridDistance(candidatePos, packCenter)

			// Score: prioritize hit count, then proximity to center
			score := float64(hitCount)*10.0 - float64(distToCenter)*0.5

			candidates = append(candidates, scoredPos{
				pos:      candidatePos,
				hitCount: hitCount,
				score:    score,
			})
		}
	}

	if len(candidates) == 0 {
		// Fallback: try casting at pack center directly
		if s.validateBlizzardLoS(packCenter) {
			return packCenter, true
		}
		// Last resort: nearest monster's position
		nearest, found := s.findNearestMonster()
		if found && s.validateBlizzardLoS(nearest.Position) {
			return nearest.Position, true
		}
		return data.Position{}, false
	}

	// Sort by score (highest first)
	sort.Slice(candidates, func(i, j int) bool {
		return candidates[i].score > candidates[j].score
	})

	return candidates[0].pos, true
}

func (s BlizzardSorceress) needsRepositioning() (bool, data.Monster) {
	for _, monster := range s.Data.Monsters.Enemies() {
		if monster.Stats[stat.Life] <= 0 {
			continue
		}

		distance := gridDistance(s.Data.PlayerUnit.Position, monster.Position)
		if distance < dangerDistance {
			return true, monster
		}
	}

	return false, data.Monster{}
}
