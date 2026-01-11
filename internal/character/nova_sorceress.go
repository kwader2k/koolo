package character

import (
	"fmt"
	"log/slog"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

const (
	// Default / safe Nova spacing
	NovaMinDistance = 6
	NovaMaxDistance = 9

	// Aggressive Nova spacing:
	// We do NOT want the engine to "step away" from the elite/pack just to satisfy distance.
	// If your step.RangedDistance has issues with min=0, set it to 1.
	NovaAggroMinDistance = 0
	NovaAggroMaxDistance = 8

	// Real Nova hit radius (tiles) used for scoring and leftover ignore.
	NovaSpellRadius = 8

	StaticMinDistance    = 13
	StaticMaxDistance    = 22
	NovaMaxAttacksLoop   = 10
	StaticFieldThreshold = 67 // Cast Static Field if monster HP is above this percentage

	// Pack construction radius (tiles) around a seed/anchor.
	NovaPackRadius = 15

	// Pre-computed squared radii to avoid repeated multiplications in hot paths
	novaSpellRadiusSq = NovaSpellRadius * NovaSpellRadius
	novaPackRadiusSq  = NovaPackRadius * NovaPackRadius

	// Tuning constants for aggressive Nova positioning
	repositionCooldown      = 650 * time.Millisecond // Cooldown between repositions to prevent wall-fail spam
	maxTeleportDistance     = 18                     // Don't teleport further than this unless we reach desired hits
	monsterProximityPenalty = 1.2                    // Score penalty for standing on top of monsters
	killSequenceTimeout     = 120 * time.Second      // Safety timeout for KillMonsterSequence to prevent infinite loops
)

// -------------------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------------------

type NovaSorceress struct {
	BaseCharacter
}

// gridDistance returns Chebyshev distance on the tile grid (max of |dx|,|dy|).
func gridDistance(a, b data.Position) int {
	dx := a.X - b.X
	if dx < 0 {
		dx = -dx
	}
	dy := a.Y - b.Y
	if dy < 0 {
		dy = -dy
	}
	if dx > dy {
		return dx
	}
	return dy
}

// squaredDistance returns Euclidean distance squared (dx*dx + dy*dy).
func squaredDistance(a, b data.Position) int {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return dx*dx + dy*dy
}

// packKey creates a stable "engagement key" based on an anchor position (quantized).
// This makes positioning stable even if target ID changes inside the same pack.
func packKey(pos data.Position) int64 {
	qx := int64(pos.X >> 3) // 8-tile buckets
	qy := int64(pos.Y >> 3)
	return (qx << 32) ^ qy
}

// countHitsAt counts how many monsters are within NovaSpellRadius from `pos`.
func countHitsAt(pos data.Position, pack []data.Monster) int {
	hits := 0
	for i := range pack {
		if pack[i].Stats[stat.Life] <= 0 {
			continue
		}
		if squaredDistance(pos, pack[i].Position) <= novaSpellRadiusSq {
			hits++
		}
	}
	return hits
}

// countOpenTiles returns how many tiles in a (2r+1)x(2r+1) square are walkable.
// Cheap "wall/corridor penalty": low openness = likely awkward near walls.
func countOpenTiles(pos data.Position, r int, isWalkable func(data.Position) bool) int {
	open := 0
	for x := pos.X - r; x <= pos.X+r; x++ {
		for y := pos.Y - r; y <= pos.Y+r; y++ {
			if isWalkable(data.Position{X: x, Y: y}) {
				open++
			}
		}
	}
	return open
}

func desiredHitsForPack(packSize int) int {
	// User thresholds:
	// big 10+, medium 7+, small 3+
	switch {
	case packSize >= 10:
		return 10
	case packSize >= 7:
		return 7
	case packSize >= 3:
		return 3
	default:
		return 0
	}
}

func maxRepositionsForPack(packSize int) int {
	// Fast clear: no dancing.
	// big/medium: 1 decisive reposition, small: allow 2.
	if packSize >= 7 {
		return 1
	}
	return 2
}

// -------------------------------------------------------------------------------------
// Pack selection + Elite Anchor
// -------------------------------------------------------------------------------------

// pickDenseSeed chooses a dense monster near the current target.
// We don't want the entire screen; we want the current "engagement area".
func pickDenseSeed(playerPos, targetPos data.Position, enemies []data.Monster) (seed data.Position, ok bool) {
	alive := make([]data.Monster, 0, len(enemies))
	for i := range enemies {
		if enemies[i].Stats[stat.Life] > 0 {
			alive = append(alive, enemies[i])
		}
	}
	if len(alive) == 0 {
		return data.Position{}, false
	}

	// Focus only on monsters not too far from target to avoid mixing two packs.
	const focusRadius = 22
	const densityRadius = 8

	const focusR2 = focusRadius * focusRadius
	const densityR2 = densityRadius * densityRadius

	bestIdx := -1
	bestNeighbors := -1
	bestTie := 1 << 30

	for i := range alive {
		cPos := alive[i].Position
		if squaredDistance(cPos, targetPos) > focusR2 {
			continue
		}

		neighbors := 0
		for j := range alive {
			if i == j {
				continue
			}
			if squaredDistance(cPos, alive[j].Position) <= densityR2 {
				neighbors++
			}
		}

		// Tie-break: closer to target, then closer to player (stable).
		tie := gridDistance(cPos, targetPos)*10 + gridDistance(cPos, playerPos)
		if neighbors > bestNeighbors || (neighbors == bestNeighbors && tie < bestTie) {
			bestNeighbors = neighbors
			bestIdx = i
			bestTie = tie
		}
	}

	// Fallback: if nothing was inside focus (rare), seed = closest alive to target.
	if bestIdx < 0 {
		bestIdx = 0
		bestD := 1 << 30
		for i := range alive {
			d := gridDistance(alive[i].Position, targetPos)
			if d < bestD {
				bestD = d
				bestIdx = i
			}
		}
	}

	return alive[bestIdx].Position, true
}

// buildPack builds a pack around a seed (NovaPackRadius).
func buildPack(seed data.Position, enemies []data.Monster) []data.Monster {
	pack := make([]data.Monster, 0, len(enemies))
	for i := range enemies {
		if enemies[i].Stats[stat.Life] <= 0 {
			continue
		}
		if squaredDistance(seed, enemies[i].Position) <= novaPackRadiusSq {
			pack = append(pack, enemies[i])
		}
	}
	return pack
}

func centroidOf(pack []data.Monster) data.Position {
	if len(pack) == 0 {
		return data.Position{}
	}
	sumX, sumY := 0, 0
	n := 0
	for i := range pack {
		if pack[i].Stats[stat.Life] <= 0 {
			continue
		}
		sumX += pack[i].Position.X
		sumY += pack[i].Position.Y
		n++
	}
	if n == 0 {
		return data.Position{}
	}
	return data.Position{X: sumX / n, Y: sumY / n}
}

// chooseAnchorForPack:
// - For big packs (>=10): anchor on an elite/champion inside the pack if possible
// - Otherwise anchor on target if it's elite
// - Otherwise anchor on the densest seed
// Note: cent (centroid) is passed in to avoid recalculating it later
func chooseAnchorForPack(target data.Monster, pack []data.Monster, seed data.Position, cent data.Position) data.Position {
	packSize := len(pack)

	// If big pack, elite anchor is king.
	if packSize >= 10 {
		var bestElite *data.Monster
		bestTie := 1 << 30

		for i := range pack {
			if pack[i].Stats[stat.Life] <= 0 {
				continue
			}
			if !pack[i].IsElite() {
				continue
			}

			// Prefer elite closer to centroid (more "center of pack")
			tie := gridDistance(pack[i].Position, cent)
			if bestElite == nil || tie < bestTie {
				m := pack[i]
				bestElite = &m
				bestTie = tie
			}
		}

		if bestElite != nil {
			return bestElite.Position
		}
	}

	// If current target is elite, that's a good anchor in most real scenarios.
	if target.IsElite() {
		return target.Position
	}

	// Otherwise: if any elite exists in pack, anchor to it.
	for i := range pack {
		if pack[i].Stats[stat.Life] <= 0 {
			continue
		}
		if pack[i].IsElite() {
			return pack[i].Position
		}
	}

	// Fallback to seed.
	return seed
}

// -------------------------------------------------------------------------------------
// Positioning (Aggressive Nova)
// -------------------------------------------------------------------------------------

type novaPosEval struct {
	ok          bool
	bestPos     data.Position
	bestHits    int
	currentHits int
	packSize    int
	engKey      int64
	anchorPos   data.Position
}

// evalAggressiveNovaPosition finds one best "entry" position for the current pack.
// Core goal: maximize Nova hits WITHOUT moving away from the elite anchor.
func (s NovaSorceress) evalAggressiveNovaPosition(target data.Monster) novaPosEval {
	ctx := context.Get()
	playerPos := ctx.Data.PlayerUnit.Position
	enemies := ctx.Data.Monsters.Enemies()
	if len(enemies) == 0 {
		return novaPosEval{}
	}

	seed, ok := pickDenseSeed(playerPos, target.Position, enemies)
	if !ok {
		return novaPosEval{}
	}

	pack := buildPack(seed, enemies)
	if len(pack) == 0 {
		return novaPosEval{}
	}

	// Calculate centroid once and pass it to chooseAnchorForPack
	cent := centroidOf(pack)
	anchor := chooseAnchorForPack(target, pack, seed, cent)
	key := packKey(anchor)

	packSize := len(pack)
	currentHits := countHitsAt(playerPos, pack)

	// If pack is tiny, no need to reposition here.
	if packSize < 3 {
		return novaPosEval{
			ok:          false,
			packSize:    packSize,
			currentHits: currentHits,
			engKey:      key,
			anchorPos:   anchor,
		}
	}

	// Radii based on pack size (big packs allow slightly larger search).
	anchorRadius := 7
	searchRadiusFromPlayer := 16
	if packSize >= 10 {
		anchorRadius = 9
		searchRadiusFromPlayer = 20
	} else if packSize >= 7 {
		anchorRadius = 8
		searchRadiusFromPlayer = 18
	}

	isWalkable := ctx.Data.AreaData.IsWalkable
	seen := make(map[int64]struct{}, 1024)
	add := func(p data.Position, out *[]data.Position) {
		if !isWalkable(p) {
			return
		}
		k := (int64(p.X) << 32) ^ int64(p.Y)
		if _, exists := seen[k]; exists {
			return
		}
		seen[k] = struct{}{}
		*out = append(*out, p)
	}

	candidates := make([]data.Position, 0, 1024)

	// 1) Ring around anchor (main).
	for x := anchor.X - anchorRadius; x <= anchor.X+anchorRadius; x++ {
		for y := anchor.Y - anchorRadius; y <= anchor.Y+anchorRadius; y++ {
			p := data.Position{X: x, Y: y}
			if gridDistance(anchor, p) > anchorRadius {
				continue
			}
			// Keep search local from current position (avoid long pointless teleports).
			if gridDistance(playerPos, p) > searchRadiusFromPlayer {
				continue
			}
			add(p, &candidates)
		}
	}

	// 2) Small ring around centroid (helps in weird layouts), but still near player.
	centRadius := 5
	if packSize >= 10 {
		centRadius = 7
	}
	for x := cent.X - centRadius; x <= cent.X+centRadius; x++ {
		for y := cent.Y - centRadius; y <= cent.Y+centRadius; y++ {
			p := data.Position{X: x, Y: y}
			if gridDistance(cent, p) > centRadius {
				continue
			}
			if gridDistance(playerPos, p) > searchRadiusFromPlayer {
				continue
			}
			add(p, &candidates)
		}
	}

	// 3) Local around player (micro adjustment).
	for x := playerPos.X - 3; x <= playerPos.X+3; x++ {
		for y := playerPos.Y - 3; y <= playerPos.Y+3; y++ {
			add(data.Position{X: x, Y: y}, &candidates)
		}
	}

	if len(candidates) == 0 {
		return novaPosEval{}
	}

	// Hard rule: do NOT move away from elite anchor.
	// We allow tiny sideways drift, but never "escape the pack".
	currentAnchorDist := gridDistance(playerPos, anchor)
	maxAllowedAnchorDist := currentAnchorDist + 1
	if packSize < 10 {
		// For non-big packs allow slightly more lateral movement.
		maxAllowedAnchorDist = currentAnchorDist + 2
	}

	// Scoring tuned for "fast clear":
	// - Hits dominate
	// - Strong attraction to anchor (elite center)
	// - Openness to avoid walls
	// - Light movement penalty
	hitsW := 16.0
	anchorW := 0.95
	openW := 0.55
	moveW := 0.55
	centroidW := 0.08

	if packSize >= 10 {
		hitsW = 18.0
		anchorW = 1.10
		openW = 0.70
		moveW = 0.50
		centroidW = 0.10
	} else if packSize >= 7 {
		hitsW = 17.0
		anchorW = 1.00
		openW = 0.60
		moveW = 0.52
		centroidW = 0.09
	}

	bestPos := playerPos
	bestHits := currentHits
	bestScore := -1e18

	for _, p := range candidates {
		// Hard guard: don't increase distance from anchor beyond a tiny slack.
		da := gridDistance(p, anchor)
		if da > maxAllowedAnchorDist {
			continue
		}

		hits := countHitsAt(p, pack)
		if hits == 0 {
			continue
		}

		dp := float64(gridDistance(playerPos, p))
		dAnchor := float64(da)
		dCent := float64(gridDistance(p, cent))
		open := float64(countOpenTiles(p, 1, isWalkable)) // 0..9

		// Score: maximize hits, stay near elite anchor, avoid walls, avoid long teleports.
		score := float64(hits)*hitsW -
			dAnchor*anchorW -
			dp*moveW -
			dCent*centroidW +
			open*openW

		// Slight penalty if standing on top of monsters (micro bump issues).
		// Approx: if we are within 1 tile of any monster, subtract a bit.
		for i := range pack {
			if pack[i].Stats[stat.Life] <= 0 {
				continue
			}
			if gridDistance(p, pack[i].Position) <= 1 {
				score -= monsterProximityPenalty
				break
			}
		}

		if score > bestScore {
			bestScore = score
			bestPos = p
			bestHits = hits
		}
	}

	// If we did not find anything better than current, we still return eval (for stats),
	// but "ok" is true so caller may decide based on hits/gain.
	return novaPosEval{
		ok:          true,
		bestPos:     bestPos,
		bestHits:    bestHits,
		currentHits: currentHits,
		packSize:    packSize,
		engKey:      key,
		anchorPos:   anchor,
	}
}

// -------------------------------------------------------------------------------------
// Character interface
// -------------------------------------------------------------------------------------

func (s NovaSorceress) CheckKeyBindings() []skill.ID {
	requiredKeybindings := []skill.ID{
		skill.Nova,
		skill.Teleport,
		skill.TomeOfTownPortal,
		skill.StaticField,
	}

	missingKeybindings := make([]skill.ID, 0)
	for _, cskill := range requiredKeybindings {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(cskill); !found {
			missingKeybindings = append(missingKeybindings, cskill)
		}
	}

	armorSkills := []skill.ID{skill.FrozenArmor, skill.ShiverArmor, skill.ChillingArmor}
	hasArmor := false
	for _, armor := range armorSkills {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(armor); found {
			hasArmor = true
			break
		}
	}
	if !hasArmor {
		missingKeybindings = append(missingKeybindings, skill.FrozenArmor)
	}

	if len(missingKeybindings) > 0 {
		s.Logger.Debug("There are missing required key bindings.", slog.Any("Bindings", missingKeybindings))
	}

	return missingKeybindings
}

func (s NovaSorceress) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	ctx := context.Get()

	completedAttackLoops := 0
	staticFieldCast := false

	// Pack-based engagement state
	var lastEngKey int64 = 0
	repositionCount := 0
	attackedThisEngagement := false
	lastRepositionAt := time.Time{}

	// Safety timeout to prevent infinite loops
	startTime := time.Now()

	for {
		// Safety check: prevent infinite loop if something goes wrong
		if time.Since(startTime) > killSequenceTimeout {
			s.Logger.Error("KillMonsterSequence timeout reached, breaking loop to prevent crash",
				slog.Duration("elapsed", time.Since(startTime)))
			return fmt.Errorf("kill sequence timeout after %v", killSequenceTimeout)
		}

		ctx.PauseIfNotPriority()

		id, found := monsterSelector(*s.Data)
		if !found {
			return nil
		}

		if !s.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found || monster.Stats[stat.Life] <= 0 {
			return nil
		}

		// Aggressive Nova positioning:
		// One decisive reposition per pack, anchored to elite center (when available),
		// then "nova bum bum" without dancing.
		//
		// OPTIMIZATION: Once we start attacking, skip expensive evalAggressiveNovaPosition()
		// and just spam Nova as fast as possible. Only do lightweight pack-change detection.
		if ctx.CharacterCfg.Character.NovaSorceress.AggressiveNovaPositioning {
			// Lightweight pack-change detection based on target position
			// If target moved significantly, we might be engaging a new pack
			currentTargetKey := packKey(monster.Position)

			if attackedThisEngagement {
				// Already attacking - only check if we switched to a completely different pack
				if currentTargetKey != lastEngKey && gridDistance(monster.Position, ctx.Data.PlayerUnit.Position) > NovaSpellRadius {
					// Target is far and in different grid bucket - likely new pack, reset state
					lastEngKey = currentTargetKey
					repositionCount = 0
					attackedThisEngagement = false
					lastRepositionAt = time.Time{}
				}
				// Otherwise skip expensive evaluation and continue spamming Nova
			} else {
				// Not yet attacking - do full evaluation to find best position
				ev := s.evalAggressiveNovaPosition(monster)

				if ev.engKey != 0 && ev.engKey != lastEngKey {
					lastEngKey = ev.engKey
					repositionCount = 0
					lastRepositionAt = time.Time{}
				}

				if ev.ok {
					need := desiredHitsForPack(ev.packSize)
					maxRep := maxRepositionsForPack(ev.packSize)

					if need > 0 && repositionCount < maxRep && ev.currentHits < need {
						// Cooldown prevents wall-fail spam.
						if lastRepositionAt.IsZero() || time.Since(lastRepositionAt) > repositionCooldown {
							gain := ev.bestHits - ev.currentHits

							// Big packs: demand meaningful improvement.
							worthIt := false
							if ev.bestHits > ev.currentHits {
								if ev.bestHits >= need {
									worthIt = true
								} else {
									if ev.packSize >= 10 {
										worthIt = gain >= 2
									} else {
										worthIt = gain >= 1
									}
								}
							}

							// Do not waste time on long teleports unless it reaches desired hits.
							dist := gridDistance(ctx.Data.PlayerUnit.Position, ev.bestPos)
							if dist >= maxTeleportDistance && ev.bestHits < need {
								worthIt = false
							}

							// Don't bother if position is basically the same.
							if dist == 0 {
								worthIt = false
							}

							if worthIt {
								if err := step.MoveTo(ev.bestPos); err != nil {
									s.Logger.Debug("Aggressive Nova reposition failed", slog.String("error", err.Error()))
									repositionCount++
								} else {
									lastRepositionAt = time.Now()
									repositionCount++
								}
							}
						}
					}
				}
			}
		}

		// Static Field first if needed.
		if !staticFieldCast && s.shouldCastStaticField(monster) {
			staticOpts := []step.AttackOption{
				step.RangedDistance(StaticMinDistance, StaticMaxDistance),
			}

			if err := step.SecondaryAttack(skill.StaticField, monster.UnitID, 1, staticOpts...); err == nil {
				staticFieldCast = true
				attackedThisEngagement = true
				continue
			}
		}

		// Choose Nova distance based on config (aggressive / normal).
		novaMin := NovaMinDistance
		novaMax := NovaMaxDistance
		if ctx.CharacterCfg.Character.NovaSorceress.AggressiveNovaPositioning {
			novaMin = NovaAggroMinDistance
			novaMax = NovaAggroMaxDistance
		}

		novaOpts := []step.AttackOption{
			step.RangedDistance(novaMin, novaMax),
		}

		if err := step.SecondaryAttack(skill.Nova, monster.UnitID, 1, novaOpts...); err == nil {
			completedAttackLoops++
			attackedThisEngagement = true
		}

		if completedAttackLoops >= NovaMaxAttacksLoop {
			completedAttackLoops = 0
			staticFieldCast = false
		}
	}
}

func (s NovaSorceress) shouldCastStaticField(monster data.Monster) bool {
	maxLife := monster.Stats[stat.MaxLife]
	if maxLife == 0 {
		return false
	}
	// Integer comparison - mathematically equivalent to float division
	return monster.Stats[stat.Life]*100 > maxLife*StaticFieldThreshold
}

func (s NovaSorceress) killBossWithStatic(bossID npc.ID, monsterType data.MonsterType) error {
	ctx := context.Get()

	for {
		ctx.PauseIfNotPriority()

		boss, found := s.Data.Monsters.FindOne(bossID, monsterType)
		if !found || boss.Stats[stat.Life] <= 0 {
			return nil
		}

		bossHP := boss.Stats[stat.Life]
		bossMaxHP := boss.Stats[stat.MaxLife]
		threshold := ctx.CharacterCfg.Character.NovaSorceress.BossStaticThreshold

		// Cast Static Field until boss HP is below threshold.
		// Integer comparison - mathematically equivalent to float division
		if bossHP*100 > bossMaxHP*threshold {
			staticOpts := []step.AttackOption{
				step.Distance(StaticMinDistance, StaticMaxDistance),
			}

			err := step.SecondaryAttack(skill.StaticField, boss.UnitID, 1, staticOpts...)
			if err != nil {
				s.Logger.Warn("Failed to cast Static Field", slog.String("error", err.Error()))
			}

			continue
		}

		// Switch to Nova once boss HP is low enough.
		return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			return boss.UnitID, true
		}, nil)
	}
}

func (s NovaSorceress) killMonsterByName(id npc.ID, monsterType data.MonsterType, skipOnImmunities []stat.Resist) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		if m, found := d.Monsters.FindOne(id, monsterType); found {
			return m.UnitID, true
		}
		return 0, false
	}, skipOnImmunities)
}

func (s NovaSorceress) BuffSkills() []skill.ID {
	skillsList := make([]skill.ID, 0)

	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.EnergyShield); found {
		skillsList = append(skillsList, skill.EnergyShield)
	}

	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.ThunderStorm); found {
		skillsList = append(skillsList, skill.ThunderStorm)
	}

	// Add one of the armor skills.
	for _, armor := range []skill.ID{skill.ChillingArmor, skill.ShiverArmor, skill.FrozenArmor} {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(armor); found {
			skillsList = append(skillsList, armor)
			break
		}
	}

	return skillsList
}

func (s NovaSorceress) PreCTABuffSkills() []skill.ID { return []skill.ID{} }

// ShouldIgnoreMonster skips tiny leftover packs in aggressive mode (<3 normals nearby).
func (s NovaSorceress) ShouldIgnoreMonster(m data.Monster) bool {
	ctx := context.Get()

	// If aggressive Nova is not enabled, never ignore.
	if !ctx.CharacterCfg.Character.NovaSorceress.AggressiveNovaPositioning {
		return false
	}

	// Never ignore elites / bosses / important monsters.
	if m.IsElite() {
		return false
	}

	// Dead or invalid monsters do not matter here.
	if m.Stats[stat.Life] <= 0 || m.Stats[stat.MaxLife] <= 0 {
		return false
	}

	// Count how many normal (non-elite) monsters are within Nova radius around this monster.
	// Early exit optimization - stop counting once threshold reached (same result, less iterations)
	normalCount := 0
	mPos := m.Position

	for _, other := range ctx.Data.Monsters.Enemies() {
		if other.Stats[stat.Life] <= 0 || other.Stats[stat.MaxLife] <= 0 {
			continue
		}
		if other.IsElite() {
			continue
		}
		if gridDistance(mPos, other.Position) <= NovaSpellRadius {
			normalCount++
			if normalCount >= 3 {
				return false // Early exit - same result as original
			}
		}
	}

	// If fewer than 3 normals around it, treat as leftover.
	return normalCount < 3
}

func (s NovaSorceress) KillAndariel() error {
	return s.killBossWithStatic(npc.Andariel, data.MonsterTypeUnique)
}

func (s NovaSorceress) KillDuriel() error {
	return s.killBossWithStatic(npc.Duriel, data.MonsterTypeUnique)
}

func (s NovaSorceress) KillMephisto() error {
	return s.killBossWithStatic(npc.Mephisto, data.MonsterTypeUnique)
}

func (s NovaSorceress) KillDiablo() error {
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

			time.Sleep(200 * time.Millisecond)
			continue
		}

		diabloFound = true
		s.Logger.Info("Diablo detected, attacking")
		return s.killBossWithStatic(npc.Diablo, data.MonsterTypeUnique)
	}
}

func (s NovaSorceress) KillBaal() error {
	return s.killBossWithStatic(npc.BaalCrab, data.MonsterTypeUnique)
}

func (s NovaSorceress) KillCountess() error {
	return s.killMonsterByName(npc.DarkStalker, data.MonsterTypeSuperUnique, nil)
}

func (s NovaSorceress) KillSummoner() error {
	return s.killMonsterByName(npc.Summoner, data.MonsterTypeUnique, nil)
}

func (s NovaSorceress) KillIzual() error {
	return s.killBossWithStatic(npc.Izual, data.MonsterTypeUnique)
}

func (s NovaSorceress) KillCouncil() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		for _, m := range d.Monsters.Enemies() {
			if m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3 {
				return m.UnitID, true
			}
		}
		return 0, false
	}, nil)
}

func (s NovaSorceress) KillPindle() error {
	return s.killMonsterByName(
		npc.DefiledWarrior,
		data.MonsterTypeSuperUnique,
		s.CharacterCfg.Game.Pindleskin.SkipOnImmunities,
	)
}

func (s NovaSorceress) KillNihlathak() error {
	return s.killMonsterByName(npc.Nihlathak, data.MonsterTypeSuperUnique, nil)
}
