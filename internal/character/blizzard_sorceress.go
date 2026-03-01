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
	minBlizzSorceressAttackDistance = 8
	maxBlizzSorceressAttackDistance = 16
	dangerDistance                  = 8  // Monsters closer than this are considered dangerous
	safeDistance                    = 10 // Distance to teleport away to
)

// Threat assessment constants
const (
	threatBaseWeight      = 1.0
	threatEliteMultiplier = 3.0
	threatAuraMultiplier  = 2.5
	threatProximityFactor = 1.5
	threatCountThreshold  = 3

	threatLevelLow    = 3.0
	threatLevelMedium = 8.0
	threatLevelHigh   = 15.0

	minRepositionCooldown = 200 * time.Millisecond
	maxRepositionCooldown = 1200 * time.Millisecond
)

type threatInfo struct {
	needsReposition bool
	threatScore     float64
	centroid        data.Position
	monsterCount    int
	hasElite        bool
	hasAura         bool
}

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

// monsterHasDangerousAura checks offensive auras that make nearby mobs lethal to a sorc.
func monsterHasDangerousAura(m data.Monster) bool {
	return m.States.HasState(state.Fanaticism) ||
		m.States.HasState(state.Might) ||
		m.States.HasState(state.Conviction) ||
		m.States.HasState(state.Holyfire) ||
		m.States.HasState(state.Holyshock) ||
		m.States.HasState(state.Holywindcold)
}

// assessThreat iterates all enemies once and computes a weighted threat score,
// centroid of danger, and flags for elite/aura presence.
func (s BlizzardSorceress) assessThreat() threatInfo {
	playerPos := s.Data.PlayerUnit.Position
	var (
		totalWeight float64
		weightedX   float64
		weightedY   float64
		count       int
		hasElite    bool
		hasAura     bool
	)

	for _, monster := range s.Data.Monsters.Enemies() {
		if monster.Stats[stat.Life] <= 0 {
			continue
		}

		distance := pather.DistanceFromPoint(playerPos, monster.Position)
		if distance > dangerDistance {
			continue
		}

		weight := threatBaseWeight

		// Closer monsters are more threatening
		if distance > 0 {
			weight *= threatProximityFactor * (float64(dangerDistance) / float64(distance))
		} else {
			weight *= threatProximityFactor * float64(dangerDistance)
		}

		if monster.IsElite() {
			weight *= threatEliteMultiplier
			hasElite = true
		}

		if monsterHasDangerousAura(monster) {
			weight *= threatAuraMultiplier
			hasAura = true
		}

		totalWeight += weight
		weightedX += float64(monster.Position.X) * weight
		weightedY += float64(monster.Position.Y) * weight
		count++
	}

	if count == 0 {
		return threatInfo{}
	}

	centroid := data.Position{
		X: int(weightedX / totalWeight),
		Y: int(weightedY / totalWeight),
	}

	needsRepos := (count > 0 && totalWeight >= threatLevelLow) || count >= threatCountThreshold

	return threatInfo{
		needsReposition: needsRepos,
		threatScore:     totalWeight,
		centroid:        centroid,
		monsterCount:    count,
		hasElite:        hasElite,
		hasAura:         hasAura,
	}
}

// getRepositionCooldown interpolates between min and max cooldown based on HP and threat.
func (s BlizzardSorceress) getRepositionCooldown(threat threatInfo) time.Duration {
	hpPercent := float64(s.Data.PlayerUnit.HPPercent()) / 100.0
	if hpPercent > 1.0 {
		hpPercent = 1.0
	}
	if hpPercent < 0.0 {
		hpPercent = 0.0
	}

	// Normalize threat score: 0 at low, 1 at high+
	threatNorm := (threat.threatScore - threatLevelLow) / (threatLevelHigh - threatLevelLow)
	if threatNorm < 0 {
		threatNorm = 0
	}
	if threatNorm > 1 {
		threatNorm = 1
	}

	// HP factor: lower HP = lower urgencyFactor (shorter cooldown)
	// Threat factor: higher threat = lower urgencyFactor (shorter cooldown)
	// Weight: HP 60%, threat 40%
	urgencyFactor := hpPercent*0.6 + (1.0-threatNorm)*0.4

	cooldownRange := float64(maxRepositionCooldown - minRepositionCooldown)
	cooldown := float64(minRepositionCooldown) + cooldownRange*urgencyFactor

	return time.Duration(cooldown)
}

func (s BlizzardSorceress) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	lastReposition := time.Now()

	attackOpts := step.StationaryDistance(minBlizzSorceressAttackDistance, maxBlizzSorceressAttackDistance)

	for {
		context.Get().PauseIfNotPriority()

		if s.Context.Data.PlayerUnit.IsDead() {
			s.Logger.Info("Player detected as dead during KillMonsterSequence, stopping actions.")
			time.Sleep(500 * time.Millisecond)
			return health.ErrDied
		}

		// Assess threat once per iteration — shared by pre-attack and cooldown phases
		threat := s.assessThreat()

		// Pre-attack reposition if dangerous and cooldown elapsed
		if threat.needsReposition && time.Since(lastReposition) > s.getRepositionCooldown(threat) {
			lastReposition = time.Now()

			targetID, found := monsterSelector(*s.Data)
			if !found {
				return nil
			}

			targetMonster, found := s.Data.Monsters.FindByID(targetID)
			if !found {
				return nil
			}

			safePos, found := s.findSafePosition(targetMonster, threat)
			if found {
				step.MoveTo(safePos, step.WithIgnoreMonsters())
			}
		}

		// Get the monster to attack
		id, found := monsterSelector(*s.Data)
		if !found {
			return nil
		}

		// If the monster has changed, reset the attack loop counter
		if previousUnitID != int(id) {
			completedAttackLoops = 0
		}

		if !s.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		// If we've exceeded the maximum number of attacks, finish the loop.
		if completedAttackLoops >= sorceressMaxAttacksLoop {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			s.Logger.Info("Monster not found", slog.String("monster", fmt.Sprintf("%v", monster)))
			return nil
		}

		// If we're on cooldown, reposition if dangerous, otherwise primary attack
		if s.Data.PlayerUnit.States.HasState(state.Cooldown) {
			if threat.needsReposition {
				safePos, found := s.findSafePosition(monster, threat)
				if found {
					step.MoveTo(safePos, step.WithIgnoreMonsters())
					lastReposition = time.Now()
				}
			} else {
				step.PrimaryAttack(id, 2, true, attackOpts)
			}
			continue
		}

		step.SecondaryAttack(skill.Blizzard, id, 1, attackOpts)

		completedAttackLoops++
		previousUnitID = int(id)
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

// stutterStepStaticField alternates between casting Static Field at close range
// and teleporting back to safe Blizzard distance. Aborts if HP drops below 50%.
func (s BlizzardSorceress) stutterStepStaticField(bossID data.UnitID, _ data.Position, numCasts int) {
	canCastStaticField := s.Data.PlayerUnit.Skills[skill.StaticField].Level > 0
	_, isStaticFieldBound := s.Data.KeyBindings.KeyBindingForSkill(skill.StaticField)
	if !canCastStaticField || !isStaticFieldBound {
		return
	}

	for i := 0; i < numCasts; i++ {
		// HP-gated retreat
		if s.Data.PlayerUnit.HPPercent() < 50 {
			s.Logger.Info("HP below 50% during stutter-step, retreating")
			threat := s.assessThreat()
			boss, found := s.Data.Monsters.FindByID(bossID)
			if found {
				safePos, posFound := s.findSafePosition(boss, threat)
				if posFound {
					step.MoveTo(safePos, step.WithIgnoreMonsters())
				}
			}
			return
		}

		// Check boss is still alive
		boss, found := s.Data.Monsters.FindByID(bossID)
		if !found || boss.Stats[stat.Life] <= 0 {
			return
		}

		// Static Field can't reduce below ~33% in Hell; stop wasting casts
		if boss.Stats[stat.MaxLife] > 0 {
			bossHPPercent := float64(boss.Stats[stat.Life]) / float64(boss.Stats[stat.MaxLife]) * 100
			if bossHPPercent <= 35 {
				return
			}
		}

		// Cast Static Field at close range
		step.SecondaryAttack(skill.StaticField, bossID, 1, step.Distance(3, 8))

		// Teleport back to safe Blizzard distance
		threat := s.assessThreat()
		safePos, posFound := s.findSafePosition(boss, threat)
		if posFound {
			step.MoveTo(safePos, step.WithIgnoreMonsters())
		}
	}
}

func (s BlizzardSorceress) KillIzual() error {
	m, found := s.Data.Monsters.FindOne(npc.Izual, data.MonsterTypeUnique)
	if !found {
		s.Logger.Error("Izual not found")
		return nil
	}
	s.stutterStepStaticField(m.UnitID, m.Position, 4)
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

		s.stutterStepStaticField(diablo.UnitID, diablo.Position, 5)

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
	m, found := s.Data.Monsters.FindOne(npc.BaalCrab, data.MonsterTypeUnique)
	if !found {
		s.Logger.Error("Baal not found")
		return nil
	}
	s.stutterStepStaticField(m.UnitID, m.Position, 4)
	return s.killMonsterByName(npc.BaalCrab, data.MonsterTypeUnique, nil)
}

// findSafePosition finds a safe position away from the threat centroid while maintaining
// line of sight and attack range to the target monster. Uses a directional cone search
// centered on the escape vector from the threat centroid, falling back to a full circle.
func (s BlizzardSorceress) findSafePosition(targetMonster data.Monster, threat threatInfo) (data.Position, bool) {
	ctx := context.Get()
	playerPos := s.Data.PlayerUnit.Position

	const minSafeMonsterDistance = 2

	// Escape vector: flee from threat centroid (not single target)
	escapeFrom := threat.centroid
	if threat.monsterCount == 0 {
		escapeFrom = targetMonster.Position
	}

	vectorX := playerPos.X - escapeFrom.X
	vectorY := playerPos.Y - escapeFrom.Y
	escapeAngle := math.Atan2(float64(vectorY), float64(vectorX))

	candidatePositions := []data.Position{}

	// Phase 1: 180-degree cone centered on escape direction, 10-degree increments
	for offsetDeg := -90; offsetDeg <= 90; offsetDeg += 10 {
		radians := escapeAngle + float64(offsetDeg)*math.Pi/180

		for distance := minSafeMonsterDistance; distance <= safeDistance+5; distance += 2 {
			dx := int(math.Cos(radians) * float64(distance))
			dy := int(math.Sin(radians) * float64(distance))

			basePos := data.Position{
				X: playerPos.X + dx,
				Y: playerPos.Y + dy,
			}

			for offsetX := -1; offsetX <= 1; offsetX++ {
				for offsetY := -1; offsetY <= 1; offsetY++ {
					candidatePos := data.Position{
						X: basePos.X + offsetX,
						Y: basePos.Y + offsetY,
					}
					if s.Data.AreaData.IsWalkable(candidatePos) {
						candidatePositions = append(candidatePositions, candidatePos)
					}
				}
			}
		}
	}

	// Phase 2: Fallback to full circle at 20-degree increments if cone found nothing walkable
	if len(candidatePositions) == 0 {
		for angle := 0; angle < 360; angle += 20 {
			radians := float64(angle) * math.Pi / 180

			for distance := minSafeMonsterDistance; distance <= safeDistance+5; distance += 2 {
				dx := int(math.Cos(radians) * float64(distance))
				dy := int(math.Sin(radians) * float64(distance))

				candidatePos := data.Position{
					X: playerPos.X + dx,
					Y: playerPos.Y + dy,
				}
				if s.Data.AreaData.IsWalkable(candidatePos) {
					candidatePositions = append(candidatePositions, candidatePos)
				}
			}
		}
	}

	if len(candidatePositions) == 0 {
		return data.Position{}, false
	}

	type scoredPosition struct {
		pos   data.Position
		score float64
	}

	scoredPositions := []scoredPosition{}

	for _, pos := range candidatePositions {
		// Check line of sight to target
		if !ctx.PathFinder.LineOfSight(pos, targetMonster.Position) {
			continue
		}

		// Minimum distance to any monster
		minMonsterDist := math.MaxFloat64
		nearbyCount := 0
		nearbyThreatScore := 0.0
		for _, monster := range s.Data.Monsters.Enemies() {
			if monster.Stats[stat.Life] <= 0 {
				continue
			}

			monsterDistance := float64(pather.DistanceFromPoint(pos, monster.Position))
			if monsterDistance < minMonsterDist {
				minMonsterDist = monsterDistance
			}
			if monsterDistance < float64(dangerDistance) {
				nearbyCount++
				if monster.IsElite() || monsterHasDangerousAura(monster) {
					nearbyThreatScore += 1.0
				}
			}
		}

		if minMonsterDist < float64(minSafeMonsterDistance) {
			continue
		}

		targetDistance := pather.DistanceFromPoint(pos, targetMonster.Position)
		distanceFromPlayer := pather.DistanceFromPoint(pos, playerPos)
		centroidDist := float64(pather.DistanceFromPoint(pos, threat.centroid))

		// Attack range score
		attackRangeScore := 0.0
		if targetDistance >= minBlizzSorceressAttackDistance && targetDistance <= maxBlizzSorceressAttackDistance {
			attackRangeScore = 10.0
		} else {
			attackRangeScore = -math.Abs(float64(targetDistance) - float64(minBlizzSorceressAttackDistance+maxBlizzSorceressAttackDistance)/2.0)
		}

		// Scoring formula
		score := minMonsterDist*3.0 +
			attackRangeScore*2.0 -
			float64(distanceFromPlayer)*0.5 +
			centroidDist*2.5 -
			float64(nearbyCount)*3.0 -
			nearbyThreatScore*1.5

		// Extra bonus for positions far from monsters
		if minMonsterDist > float64(dangerDistance) {
			score += 5.0
		}

		// Wall-blocked LoS bonus: only compute when elites/auras present
		if threat.hasElite || threat.hasAura {
			wallBlocked := 0
			for _, monster := range s.Data.Monsters.Enemies() {
				if monster.Stats[stat.Life] <= 0 {
					continue
				}
				dist := pather.DistanceFromPoint(pos, monster.Position)
				if dist < dangerDistance*2 && (monster.IsElite() || monsterHasDangerousAura(monster)) {
					if !ctx.PathFinder.LineOfSight(pos, monster.Position) {
						wallBlocked++
					}
				}
			}
			score += float64(wallBlocked) * 1.0
		}

		scoredPositions = append(scoredPositions, scoredPosition{
			pos:   pos,
			score: score,
		})
	}

	sort.Slice(scoredPositions, func(i, j int) bool {
		return scoredPositions[i].score > scoredPositions[j].score
	})

	if len(scoredPositions) > 0 {
		return scoredPositions[0].pos, true
	}

	return data.Position{}, false
}
