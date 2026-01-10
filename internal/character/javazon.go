package character

import (
	"fmt"
	"log/slog"
	"sort"
	"sync"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	maxJavazonAttackLoops = 10
	minJavazonDistance    = 10
	maxJavazonDistance    = 30

	// Density killer (endgame pack clearing) internal knobs.
	// User-facing settings are in CharacterCfg.Character.Javazon.
	jzDkDefaultIgnoreWhitesBelow = 5
	jzDkScreenRadius             = 16 // Roughly "on-screen" in game tiles.
	jzDkPackRadius               = 8
	jzDkApproachIfFurtherThan    = 10
	jzDkStandoffDistance         = 4
	jzDkFuryMaxDistance          = 12
	jzDkEliteCleanupMaxEnemies   = 3

	// Hard stops to avoid infinite loops when a forced elite cannot be reached.
	jzDkMaxForcedEliteMoveAttempts = 6
)

type Javazon struct {
	BaseCharacter
}

func (s Javazon) densityKillerEnabled() bool {
	ctx := context.Get()
	if ctx == nil || ctx.CharacterCfg == nil {
		return false
	}
	return ctx.CharacterCfg.Character.Javazon.DensityKillerEnabled
}

func (s Javazon) densityIgnoreWhitesBelow() int {
	ctx := context.Get()
	if ctx == nil || ctx.CharacterCfg == nil {
		return jzDkDefaultIgnoreWhitesBelow
	}
	val := ctx.CharacterCfg.Character.Javazon.DensityKillerIgnoreWhitesBelow
	if val <= 0 {
		return jzDkDefaultIgnoreWhitesBelow
	}
	// Keep sanity bounds to avoid extreme values breaking navigation.
	if val > 25 {
		return 25
	}
	return val
}

// --- Density killer helpers (performance + safety) ---
const (
	jzDkMaxClusterCandidates = 40 // cap to avoid O(n^2) spikes in dense areas / multi-client
	jzDkMinSkillSwapDelayMS  = 70 // avoids skipped casts on high CPU usage
	jzDkMinClickDelayMS      = 50
)

func jzAliveMonster(m data.Monster) bool {
	return m.Stats[stat.Life] > 0
}

// Performance: cache dense-pack detection used by ShouldIgnoreMonster.
// That method can be called many times per tick; caching avoids repeated O(n^2) scans.
const jzDkDenseClusterCacheTTL = 150 * time.Millisecond

type jzDkDenseClusterCache struct {
	at          time.Time
	me          data.Position
	ignoreBelow int
	hasDense    bool
}

var jzDkDenseCache struct {
	mu sync.Mutex
	v  jzDkDenseClusterCache
}

func (s Javazon) jzDkHasDenseWhiteCluster(ignoreBelow int) bool {
	ctx := context.Get()
	if ctx == nil || ctx.Data == nil {
		return false
	}
	me := ctx.Data.PlayerUnit.Position

	jzDkDenseCache.mu.Lock()
	cv := jzDkDenseCache.v
	if cv.ignoreBelow == ignoreBelow && cv.me == me && time.Since(cv.at) < jzDkDenseClusterCacheTTL {
		res := cv.hasDense
		jzDkDenseCache.mu.Unlock()
		return res
	}
	jzDkDenseCache.mu.Unlock()

	// Build on-screen whites and cap candidates to keep work bounded.
	// ✅ CRITICAL FIX: Only count monsters we can actually attack (have LoS)
	whites := make([]data.Monster, 0, 32)
	for _, mo := range ctx.Data.Monsters {
		if mo.IsPet() || mo.IsMerc() || mo.IsGoodNPC() || mo.IsSkip() {
			continue
		}
		if !jzAliveMonster(mo) {
			continue
		}
		if mo.IsElite() {
			continue
		}
		if pather.DistanceFromPoint(mo.Position, me) <= jzDkScreenRadius {
			// ✅ CRITICAL: Only add if we have LoS (can actually attack it)
			// This prevents considering cow packs behind fences as "dense clusters"
			if s.jzDkHasLoS(me, mo.Position) {
				whites = append(whites, mo)
			}
		}
	}

	if len(whites) == 0 {
		jzDkDenseCache.mu.Lock()
		jzDkDenseCache.v = jzDkDenseClusterCache{at: time.Now(), me: me, ignoreBelow: ignoreBelow, hasDense: false}
		jzDkDenseCache.mu.Unlock()
		return false
	}

	if len(whites) > jzDkMaxClusterCandidates {
		sort.Slice(whites, func(i, j int) bool {
			di := jzDkGridDist(me, whites[i].Position)
			dj := jzDkGridDist(me, whites[j].Position)
			return di < dj
		})
		whites = whites[:jzDkMaxClusterCandidates]
	}

	if ignoreBelow < 2 {
		ignoreBelow = 2
	}
	hasDense := false
	for _, c := range whites {
		cluster := 0
		for _, o := range whites {
			if jzDkGridDist(c.Position, o.Position) <= jzDkPackRadius {
				cluster++
				if cluster >= ignoreBelow {
					hasDense = true
					break
				}
			}
		}
		if hasDense {
			break
		}
	}

	jzDkDenseCache.mu.Lock()
	jzDkDenseCache.v = jzDkDenseClusterCache{at: time.Now(), me: me, ignoreBelow: ignoreBelow, hasDense: hasDense}
	jzDkDenseCache.mu.Unlock()
	return hasDense
}

func (s Javazon) ShouldIgnoreMonster(m data.Monster) bool {
	// Default (safe) Javazon should behave exactly like upstream, without ignoring.
	if !s.densityKillerEnabled() {
		return false
	}

	ctx := context.Get()
	if ctx == nil || ctx.Data == nil {
		return false
	}

	// Never ignore elites.
	if m.IsElite() {
		return false
	}

	// Keep the immediate vicinity safe: if a normal monster is close, don't ignore it.
	// ✅ CRITICAL FIX: Also check LoS to prevent PathFinder waiting for mobs behind walls
	const closeThreatRadius = 6
	me := ctx.Data.PlayerUnit.Position
	if pather.DistanceFromPoint(m.Position, me) <= closeThreatRadius {
		// Only block PathFinder if we can actually attack the mob (have LoS)
		// Mobs behind walls will be ignored until we reposition
		if s.jzDkHasLoS(me, m.Position) {
			return false // Can attack this mob - don't ignore
		}
		// Mob is behind wall - let PathFinder move, we'll handle it after reposition
		return true
	}

	ignoreBelow := s.densityIgnoreWhitesBelow()
	// If there is a dense white cluster on screen, do NOT ignore (we want to keep clearing).
	if s.jzDkHasDenseWhiteCluster(ignoreBelow) {
		return false
	}

	// No dense cluster: ignore normal monsters (speed-up in endgame density mode).
	return true
}

func (s Javazon) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{skill.LightningFury, skill.ChargedStrike, skill.TomeOfTownPortal}
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

func (s Javazon) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	if s.densityKillerEnabled() {
		return s.killMonsterSequenceDensity(monsterSelector, skipOnImmunities)
	}
	return s.killMonsterSequenceSafe(monsterSelector, skipOnImmunities)
}

func (s Javazon) killMonsterSequenceSafe(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	const numOfAttacks = 5

	for {
		context.Get().PauseIfNotPriority()

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

		if completedAttackLoops >= maxJavazonAttackLoops {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			s.Logger.Info("Monster not found", slog.String("monster", fmt.Sprintf("%v", monster)))
			return nil
		}

		closeMonsters := 0
		for _, mob := range s.Data.Monsters {
			if mob.IsPet() || mob.IsMerc() || mob.IsGoodNPC() || mob.IsSkip() {
				continue
			}
			if !jzAliveMonster(mob) {
				continue
			}
			if pather.DistanceFromPoint(mob.Position, monster.Position) <= 15 {
				closeMonsters++
			}
			if closeMonsters >= 3 {
				break
			}
		}

		if closeMonsters >= 3 {
			step.SecondaryAttack(skill.LightningFury, id, numOfAttacks, step.Distance(minJavazonDistance, maxJavazonDistance))
		} else {
			if s.Data.PlayerUnit.Skills[skill.ChargedStrike].Level > 0 {
				s.chargedStrikeAccurate(id, numOfAttacks)
			} else {
				step.PrimaryAttack(id, numOfAttacks, false, step.Distance(1, 1))
			}
		}

		completedAttackLoops++
		previousUnitID = int(id)
	}
}

func (s Javazon) killMonsterSequenceDensity(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	ctx := context.Get()
	completedAttackLoops := 0
	previousUnitID := data.UnitID(0)
	const numOfAttacks = 5

	forcedEliteMoveAttempts := 0
	forcedEliteLastID := data.UnitID(0)

	// Stuck detection: only clear close blockers when we are not moving.
	lastMePos := data.Position{}
	samePosTicks := 0
	blockerClearCooldown := 0
	losRepositionCooldown := 0

	for {
		ctx.PauseIfNotPriority()
		ctx.RefreshGameData()

		meNow := s.Data.PlayerUnit.Position
		if meNow == lastMePos {
			samePosTicks++
		} else {
			samePosTicks = 0
			lastMePos = meNow
		}
		if blockerClearCooldown > 0 {
			blockerClearCooldown--
		}
		if losRepositionCooldown > 0 {
			losRepositionCooldown--
		}

		// Loot-guard: during ItemPickup, clear only around valuable items that Pickit actually wants (tight radius).
		lootGuard := false
		lootGuardPos := data.Position{}
		if ctx.CurrentGame != nil && ctx.CurrentGame.IsPickingItems {
			if pos, ok := javazonNearestValuablePickupPos(30); ok {
				lootGuard = true
				lootGuardPos = pos
			}
		}

		// 1) If the caller is forcing an elite target (e.g., Diablo/seal bosses), prioritize reaching it.
		forcedID, forcedFound := monsterSelector(*s.Data)
		if forcedFound && forcedID != 0 {
			if forcedID != forcedEliteLastID {
				forcedEliteLastID = forcedID
				forcedEliteMoveAttempts = 0
			}
			if s.jzDkHandleForcedElite(forcedID, &forcedEliteMoveAttempts, skipOnImmunities, numOfAttacks) {
				// Forced elite handled (attack or move). Re-evaluate.
				continue
			}
		}

		// 2) Build an "on-screen" enemy set and remove targets that would cause wall-stucks (no LoS).
		visible := s.jzDkVisibleEnemies(jzDkScreenRadius)
		engageable := s.jzDkEngageableEnemies(visible)

		// If we can see enemies but none are engageable (LoS blocked by walls/fences),
		// do a small reposition when stuck so we don't wait for the merc.
		if !lootGuard && len(visible) > 0 && len(engageable) == 0 && samePosTicks >= 2 && losRepositionCooldown == 0 {
			nearest := visible[0]
			best := ctx.PathFinder.DistanceFromMe(nearest.Position)
			for i := 1; i < len(visible); i++ {
				d := ctx.PathFinder.DistanceFromMe(visible[i].Position)
				if d < best {
					best = d
					nearest = visible[i]
				}
			}
			_ = action.MoveToCoords(nearest.Position, step.WithDistanceToFinish(jzDkApproachIfFurtherThan))
			losRepositionCooldown = 3
			continue
		}

		whites := 0
		hasElite := false
		for _, m := range engageable {
			if m.IsElite() {
				hasElite = true
				continue
			}
			whites++
		}

		// Loot-guard: keep the pickup area safe without roaming or clearing the whole screen.
		if lootGuard {
			nearLoot := make([]data.Monster, 0, 12)
			for _, m := range s.jzDkVisibleEnemies(30) {
				if m.IsPet() || m.IsMerc() || m.IsGoodNPC() || m.IsSkip() {
					continue
				}
				if !jzAliveMonster(m) {
					continue
				}
				if pather.DistanceFromPoint(m.Position, lootGuardPos) > 4 && pather.DistanceFromPoint(m.Position, s.Data.PlayerUnit.Position) > 6 {
					continue
				}
				if !s.jzDkHasLoS(s.Data.PlayerUnit.Position, m.Position) {
					continue
				}
				nearLoot = append(nearLoot, m)
			}

			if len(nearLoot) == 0 {
				// Let ItemPickup proceed.
				return nil
			}

			targetID, _, ok := s.jzDkPickFuryTarget(nearLoot)
			if !ok {
				targetID = nearLoot[0].UnitID
			}
			if !s.preBattleChecks(targetID, skipOnImmunities) {
				return nil
			}
			_ = step.SecondaryAttack(skill.LightningFury, targetID, 3, step.Distance(1, jzDkFuryMaxDistance))
			continue
		}

		// 3) If only elites remain (no normal monsters), finish them with Charged Strike.
		if hasElite && whites == 0 && len(engageable) <= jzDkEliteCleanupMaxEnemies && s.Data.PlayerUnit.Skills[skill.ChargedStrike].Level > 0 {
			eliteID, ok := s.jzDkPickNearestElite(engageable)
			if ok {
				if !s.preBattleChecks(eliteID, skipOnImmunities) {
					return nil
				}
				s.chargedStrikeAccurate(eliteID, numOfAttacks)
				completedAttackLoops++
				previousUnitID = eliteID
				continue
			}
		}

		// 4) Decide if we should ignore leftovers (no elite present and no dense cluster).
		ignoreBelow := s.densityIgnoreWhitesBelow()
		targetID, clusterSize, hasTarget := s.jzDkPickFuryTarget(engageable)

		if !hasElite && (clusterSize < ignoreBelow || !hasTarget) {
			// ✅ FIX: ALWAYS clear immediate threat zone before exiting combat
			// This syncs with ShouldIgnoreMonster() to prevent PathFinder deadlock
			const closeThreatRadius = 6
			me := s.Data.PlayerUnit.Position

			// Collect monsters within closeThreatRadius that we can ACTUALLY attack (have LoS)
			// This prevents trying to attack monsters behind walls which causes stuck
			immediateThreat := make([]data.Monster, 0, 8)
			for _, m := range s.jzDkVisibleEnemies(closeThreatRadius + 2) {
				if m.IsPet() || m.IsMerc() || m.IsGoodNPC() || m.IsSkip() {
					continue
				}
				if m.IsElite() {
					continue
				}
				if !jzAliveMonster(m) {
					continue
				}
				if pather.DistanceFromPoint(m.Position, me) <= closeThreatRadius {
					// ✅ CRITICAL: Only add if we have LoS (can actually attack it)
					if s.jzDkHasLoS(me, m.Position) {
						immediateThreat = append(immediateThreat, m)
					}
				}
			}

			// If there are monsters in the threat zone that we CAN attack, clear them FIRST
			if len(immediateThreat) > 0 {
				// Sort by distance (closest first)
				sort.Slice(immediateThreat, func(i, j int) bool {
					di := pather.DistanceFromPoint(immediateThreat[i].Position, me)
					dj := pather.DistanceFromPoint(immediateThreat[j].Position, me)
					return di < dj
				})

				// Attack the closest one (just 1 Fury cast, not spam)
				targetID := immediateThreat[0].UnitID
				if s.preBattleChecks(targetID, skipOnImmunities) {
					_ = step.SecondaryAttack(skill.LightningFury, targetID, 1, step.Distance(1, jzDkFuryMaxDistance))
					continue // Re-evaluate, don't exit yet
				}
			}

			// If no immediate threat with LoS, safe to exit
			// PathFinder will handle repositioning to get LoS if needed
			// Monsters behind walls will be handled after repositioning

			// Anti-stuck: only clear close blockers when we appear stuck (no movement across iterations).
			if samePosTicks >= 2 && blockerClearCooldown == 0 {
				blockers := make([]data.Monster, 0, 6)
				for _, m := range engageable {
					if m.IsElite() {
						continue
					}
					if pather.DistanceFromPoint(m.Position, me) <= 3 {
						blockers = append(blockers, m)
					}
				}
				if len(blockers) > 0 {
					targetID := blockers[0].UnitID
					if !s.preBattleChecks(targetID, skipOnImmunities) {
						return nil
					}
					_ = step.SecondaryAttack(skill.LightningFury, targetID, 2, step.Distance(1, jzDkFuryMaxDistance))
					// Cooldown to avoid back-and-forth behaviour with ignore logic.
					blockerClearCooldown = 3
					continue
				}
			}

			// If a white mob is physically blocking the pickup path (common in corridors),
			// clear the closest blocker instead of waiting for the merc.
			if samePosTicks >= 2 {
				closestID := data.UnitID(0)
				best := 999999
				for _, m := range s.jzDkVisibleEnemies(15) {
					if m.IsPet() || m.IsMerc() || m.IsGoodNPC() || m.IsSkip() {
						continue
					}
					if m.IsElite() {
						continue
					}
					if !jzAliveMonster(m) {
						continue
					}
					d := pather.DistanceFromPoint(m.Position, me)
					if d <= 6 {
						dd := ctx.PathFinder.DistanceFromMe(m.Position)
						if dd < best {
							best = dd
							closestID = m.UnitID
						}
					}
				}
				if closestID != 0 {
					if s.preBattleChecks(closestID, skipOnImmunities) {
						_ = step.SecondaryAttack(skill.LightningFury, closestID, 2, step.Distance(1, jzDkFuryMaxDistance))
						continue
					}
				}
			}

			// Now safe to exit - immediate threat zone is clear
			return nil
		}

		// 5) Cast Fury into the densest reachable cluster.
		if hasTarget {
			if previousUnitID != targetID {
				completedAttackLoops = 0
			}
			if completedAttackLoops >= maxJavazonAttackLoops {
				// Hard stop is risky on slow systems; re-evaluate instead of breaking combat.
				completedAttackLoops = 0
				previousUnitID = 0
				continue
			}
			if !s.preBattleChecks(targetID, skipOnImmunities) {
				return nil
			}
			if !s.jzDkLightningFury(targetID, numOfAttacks) {
				// If we can't safely attack (LoS/range), let higher-level movement logic continue.
				return nil
			}
			completedAttackLoops++
			previousUnitID = targetID
			continue
		}

		// No valid Fury target. If an elite is present, try to resolve it (Fury first, then CS when alone).
		if hasElite {
			eliteID, ok := s.jzDkPickNearestElite(engageable)
			if ok {
				if !s.preBattleChecks(eliteID, skipOnImmunities) {
					return nil
				}
				_ = step.SecondaryAttack(skill.LightningFury, eliteID, numOfAttacks, step.Distance(1, jzDkFuryMaxDistance))
				completedAttackLoops++
				previousUnitID = eliteID
				continue
			}
		}

		return nil
	}
}

func (s Javazon) KillBossSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	const numOfAttacks = 5

	for {
		context.Get().PauseIfNotPriority()

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

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			return nil
		}

		if !jzAliveMonster(monster) {
			return nil
		}

		if completedAttackLoops >= maxJavazonAttackLoops {
			completedAttackLoops = 0
			continue
		}

		if s.Data.PlayerUnit.Skills[skill.ChargedStrike].Level > 0 {
			for i := 0; i < numOfAttacks; i++ {
				s.chargedStrike(id)
			}
		} else {
			step.PrimaryAttack(id, numOfAttacks, false, step.Distance(1, 1))
		}

		completedAttackLoops++
		previousUnitID = int(id)
	}
}

func (s Javazon) chargedStrike(monsterID data.UnitID) {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	monster, found := s.Data.Monsters.FindByID(monsterID)
	if !found {
		return
	}

	distance := ctx.PathFinder.DistanceFromMe(monster.Position)
	if distance > 5 {
		_ = action.MoveToCoords(monster.Position, step.WithDistanceToFinish(3))
		ctx.RefreshGameData()
		monster, found = s.Data.Monsters.FindByID(monsterID)
		if !found {
			return
		}
	}

	csKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.ChargedStrike)
	if found && s.Data.PlayerUnit.RightSkill != skill.ChargedStrike {
		ctx.HID.PressKeyBinding(csKey)
		utils.Sleep(jzDkMinSkillSwapDelayMS)
	}

	screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(monster.Position.X, monster.Position.Y)
	ctx.HID.Click(game.RightButton, screenX, screenY)
}

func (s Javazon) chargedStrikeBossFast(monsterID data.UnitID) {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	monster, found := s.Data.Monsters.FindByID(monsterID)
	if !found || !jzAliveMonster(monster) {
		return
	}

	// Ensure we are close enough for Charged Strike.
	if ctx.PathFinder.DistanceFromMe(monster.Position) > 5 {
		_ = action.MoveToCoords(monster.Position, step.WithDistanceToFinish(3))
		ctx.RefreshGameData()
		monster, found = s.Data.Monsters.FindByID(monsterID)
		if !found || !jzAliveMonster(monster) {
			return
		}
	}

	csKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.ChargedStrike)
	if found && s.Data.PlayerUnit.RightSkill != skill.ChargedStrike {
		ctx.HID.PressKeyBinding(csKey)
		utils.Sleep(jzDkMinSkillSwapDelayMS)
	}

	screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(monster.Position.X, monster.Position.Y)
	ctx.HID.Click(game.RightButton, screenX, screenY)
	utils.Sleep(jzDkMinClickDelayMS)
}

// chargedStrikeAccurate is used for non-boss situations (DK cleanup / general clearing)
// to avoid "ground-click spam" and unintended walking. Boss sequences intentionally keep the fast spam.
func (s Javazon) chargedStrikeAccurate(targetID data.UnitID, attacks int) {
	if attacks <= 0 {
		return
	}

	ctx := context.Get()
	ctx.PauseIfNotPriority()

	monster, found := s.Data.Monsters.FindByID(targetID)
	if !found || !jzAliveMonster(monster) {
		return
	}

	csKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.ChargedStrike)
	if found && s.Data.PlayerUnit.RightSkill != skill.ChargedStrike {
		ctx.HID.PressKeyBinding(csKey)
		utils.Sleep(jzDkMinSkillSwapDelayMS)
	}

	// Use the step routine so range/LoS and click semantics are handled consistently.
	_ = step.SecondaryAttack(skill.ChargedStrike, targetID, attacks, step.Distance(1, 3))
}

func (s Javazon) jzDkHandleForcedElite(
	forcedID data.UnitID,
	moveAttempts *int,
	skipOnImmunities []stat.Resist,
	numOfAttacks int,
) bool {
	ctx := context.Get()
	monster, found := s.Data.Monsters.FindByID(forcedID)
	if !found || !jzAliveMonster(monster) {
		return false
	}

	// Only treat elites as forced objectives. White mobs are handled by density rules.
	if !monster.IsElite() && !action.IsMonsterSealElite(monster) {
		return false
	}

	if !s.preBattleChecks(forcedID, skipOnImmunities) {
		return false
	}

	me := s.Data.PlayerUnit.Position
	// If the forced elite is already on-screen and visible, let the normal density logic decide
	// whether to Fury the pack first or finish with Charged Strike.
	if !action.IsMonsterSealElite(monster) &&
		pather.DistanceFromPoint(me, monster.Position) <= jzDkScreenRadius &&
		s.jzDkHasLoS(me, monster.Position) {
		return false
	}

	if moveAttempts != nil && *moveAttempts >= jzDkMaxForcedEliteMoveAttempts {
		return false
	}

	// Seal elites can spawn in edge cases (Vizier off-grid). The generic attack step handles those,
	// and it will also reposition if we're blocked by walls.
	if action.IsMonsterSealElite(monster) {
		_ = step.SecondaryAttack(skill.LightningFury, forcedID, numOfAttacks, step.Distance(1, jzDkFuryMaxDistance))
		if moveAttempts != nil {
			*moveAttempts = *moveAttempts + 1
		}
		return true
	}

	// For other forced elites, close distance once, then re-evaluate with on-screen logic.
	if ctx.PathFinder.DistanceFromMe(monster.Position) > jzDkFuryMaxDistance || !s.jzDkHasLoS(me, monster.Position) {
		_ = action.MoveToCoords(monster.Position, step.WithDistanceToFinish(jzDkStandoffDistance))
		if moveAttempts != nil {
			*moveAttempts = *moveAttempts + 1
		}
		return true
	}

	_ = step.SecondaryAttack(skill.LightningFury, forcedID, numOfAttacks, step.Distance(1, jzDkFuryMaxDistance))
	if moveAttempts != nil {
		*moveAttempts = *moveAttempts + 1
	}
	return true
}

func (s Javazon) jzDkVisibleEnemies(radius int) []data.Monster {
	me := s.Data.PlayerUnit.Position
	out := make([]data.Monster, 0, 16)
	for _, m := range s.Data.Monsters {
		if m.IsPet() || m.IsMerc() || m.IsGoodNPC() || m.IsSkip() {
			continue
		}
		if !jzAliveMonster(m) {
			continue
		}
		if pather.DistanceFromPoint(m.Position, me) <= radius {
			out = append(out, m)
		}
	}
	return out
}

func (s Javazon) jzDkEngageableEnemies(visible []data.Monster) []data.Monster {
	me := s.Data.PlayerUnit.Position
	out := make([]data.Monster, 0, len(visible))
	for _, m := range visible {
		// Avoid "shooting through walls" for normal pack clearing.
		if s.jzDkHasLoS(me, m.Position) {
			out = append(out, m)
		}
	}
	return out
}

func (s Javazon) jzDkPickFuryTarget(enemies []data.Monster) (data.UnitID, int, bool) {
	if len(enemies) == 0 {
		return 0, 0, false
	}

	me := s.Data.PlayerUnit.Position

	// Cap candidates to avoid O(n^2) spikes on very dense screens.
	if len(enemies) > jzDkMaxClusterCandidates {
		sort.Slice(enemies, func(i, j int) bool {
			di := jzDkGridDist(me, enemies[i].Position)
			dj := jzDkGridDist(me, enemies[j].Position)
			return di < dj
		})
		enemies = enemies[:jzDkMaxClusterCandidates]
	}

	// Prefer white mobs as Fury "seeds".
	candidates := make([]data.Monster, 0, len(enemies))
	for _, m := range enemies {
		if !m.IsElite() {
			candidates = append(candidates, m)
		}
	}
	if len(candidates) == 0 {
		candidates = enemies
	}

	bestID := data.UnitID(0)
	bestCluster := 0
	bestDist := 999999

	// Early-exit: we do not need exact counts above this, only "big enough".
	stopAt := s.densityIgnoreWhitesBelow() + 2
	if stopAt < 8 {
		stopAt = 8
	}
	if stopAt > 25 {
		stopAt = 25
	}

	for _, c := range candidates {
		// ✅ CRITICAL FIX: Only consider candidates we can actually attack (have LoS)
		// This prevents stuck on Cow Level when large pack is behind walls
		if !s.jzDkHasLoS(me, c.Position) {
			continue
		}

		cluster := 0
		for _, o := range enemies {
			if o.IsElite() {
				continue
			}
			if jzDkGridDist(o.Position, c.Position) <= jzDkPackRadius {
				cluster++
				if cluster >= stopAt {
					break
				}
			}
		}
		d := jzDkGridDist(me, c.Position)
		if cluster > bestCluster || (cluster == bestCluster && d < bestDist) {
			bestCluster = cluster
			bestDist = d
			bestID = c.UnitID
		}
	}

	if bestID == 0 {
		return 0, 0, false
	}
	return bestID, bestCluster, true
}

func (s Javazon) jzDkPickNearestElite(enemies []data.Monster) (data.UnitID, bool) {
	me := s.Data.PlayerUnit.Position
	bestID := data.UnitID(0)
	bestDist := 999999
	for _, m := range enemies {
		if !m.IsElite() {
			continue
		}
		d := jzDkGridDist(me, m.Position)
		if d < bestDist {
			bestDist = d
			bestID = m.UnitID
		}
	}
	return bestID, bestID != 0
}

func (s Javazon) jzDkLightningFury(targetID data.UnitID, attacks int) bool {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	monster, found := s.Data.Monsters.FindByID(targetID)
	if !found || !jzAliveMonster(monster) {
		return false
	}

	me := s.Data.PlayerUnit.Position
	distance := ctx.PathFinder.DistanceFromMe(monster.Position)
	if distance > jzDkFuryMaxDistance {
		_ = action.MoveToCoords(monster.Position, step.WithDistanceToFinish(jzDkStandoffDistance))
		ctx.RefreshGameData()
		monster, found = s.Data.Monsters.FindByID(targetID)
		if !found || !jzAliveMonster(monster) {
			return false
		}
	}

	// Final safety gate: if we still don't have LoS, don't waste casts (avoids wall-stucks).
	if !s.jzDkHasLoS(me, monster.Position) {
		return false
	}

	lfKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.LightningFury)
	if found && s.Data.PlayerUnit.RightSkill != skill.LightningFury {
		ctx.HID.PressKeyBinding(lfKey)
		utils.Sleep(jzDkMinSkillSwapDelayMS)
	}

	for i := 0; i < attacks; i++ {
		ctx.PauseIfNotPriority()
		ctx.RefreshGameData()
		m, ok := s.Data.Monsters.FindByID(targetID)
		if !ok || !jzAliveMonster(m) {
			return true
		}
		// Prevent unnecessary telestomp/walk jitter: if we drift out of Fury range, let the caller reposition.
		if ctx.PathFinder.DistanceFromMe(m.Position) > jzDkFuryMaxDistance {
			return false
		}
		x, y := ctx.PathFinder.GameCoordsToScreenCords(m.Position.X, m.Position.Y)
		ctx.HID.Click(game.RightButton, x, y)
		utils.Sleep(jzDkMinClickDelayMS)
	}

	return true
}

func (s Javazon) jzDkHasLoS(from, to data.Position) bool {
	for _, p := range s.jzDkRaycast(from, to) {
		if !s.Data.AreaData.IsWalkable(p) {
			return false
		}
	}
	return true
}

// jzDkRaycast returns a list of grid points between two positions (inclusive).
// It is used as a lightweight line-of-sight check.
func (s Javazon) jzDkRaycast(from, to data.Position) []data.Position {
	x0, y0 := from.X, from.Y
	x1, y1 := to.X, to.Y
	dx := abs(x1 - x0)
	dy := -abs(y1 - y0)
	sx := -1
	if x0 < x1 {
		sx = 1
	}
	sy := -1
	if y0 < y1 {
		sy = 1
	}
	err := dx + dy

	points := make([]data.Position, 0, 16)
	for {
		points = append(points, data.Position{X: x0, Y: y0})
		if x0 == x1 && y0 == y1 {
			break
		}
		e2 := 2 * err
		if e2 >= dy {
			err += dy
			x0 += sx
		}
		if e2 <= dx {
			err += dx
			y0 += sy
		}
	}
	return points
}

func javazonNearestValuablePickupPos(maxDistance int) (data.Position, bool) {
	ctx := context.Get()
	if ctx == nil || ctx.Data == nil {
		return data.Position{}, false
	}

	items := action.GetItemsToPickup(maxDistance)
	if len(items) == 0 {
		return data.Position{}, false
	}

	me := ctx.Data.PlayerUnit.Position
	bestDist := 9999
	bestPos := data.Position{}
	found := false

	for _, it := range items {
		// Do not treat refills as "valuable loot" for the purposes of loot-guard clearing.
		if it.IsPotion() || it.Name == "Gold" {
			continue
		}
		d := pather.DistanceFromPoint(me, it.Position)
		if d < bestDist {
			bestDist = d
			bestPos = it.Position
			found = true
		}
	}

	return bestPos, found
}

func jzDkGridDist(a, b data.Position) int {
	dx := abs(a.X - b.X)
	dy := abs(a.Y - b.Y)
	if dx > dy {
		return dx
	}
	return dy
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func (s Javazon) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}

		return m.UnitID, true
	}, nil)
}

func (s Javazon) killBoss(npc npc.ID, t data.MonsterType) error {
	return s.KillBossSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}

		return m.UnitID, true
	}, nil)
}

func (s Javazon) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

func (s Javazon) shouldSummonValkyrie() bool {
	if s.Data.PlayerUnit.Area.IsTown() {
		return false
	}

	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Valkyrie); found {
		needsValkyrie := true

		for _, monster := range s.Data.Monsters {
			if monster.IsPet() {
				switch monster.Name {
				case npc.Valkyrie:
					needsValkyrie = false
				}
			}
		}
		return needsValkyrie
	}

	return false
}

func (s Javazon) BuffSkills() []skill.ID {
	if s.shouldSummonValkyrie() {
		return []skill.ID{skill.Valkyrie}
	}
	return []skill.ID{}
}

func (s Javazon) KillCountess() error {
	return s.killMonster(npc.DarkStalker, data.MonsterTypeSuperUnique)
}

func (s Javazon) KillAndariel() error {
	return s.killBoss(npc.Andariel, data.MonsterTypeUnique)
}

func (s Javazon) KillSummoner() error {
	return s.killMonster(npc.Summoner, data.MonsterTypeUnique)
}

func (s Javazon) KillDuriel() error {
	return s.killBoss(npc.Duriel, data.MonsterTypeUnique)
}

func (s Javazon) KillCouncil() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		// Exclude monsters that are not council members
		var councilMembers []data.Monster
		for _, m := range d.Monsters {
			if m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3 {
				councilMembers = append(councilMembers, m)
			}
		}

		// Order council members by distance
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

func (s Javazon) KillMephisto() error {
	return s.killBoss(npc.Mephisto, data.MonsterTypeUnique)
}

func (s Javazon) KillIzual() error {
	return s.killBoss(npc.Izual, data.MonsterTypeUnique)
}

func (s Javazon) KillDiablo() error {
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

		return s.killMonster(npc.Diablo, data.MonsterTypeUnique)
	}
}

func (s Javazon) KillPindle() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc.DefiledWarrior, data.MonsterTypeSuperUnique)
		if !found {
			return 0, false
		}
		return m.UnitID, true
	}, nil)
}

func (s Javazon) KillNihlathak() error {
	return s.killBoss(npc.Nihlathak, data.MonsterTypeSuperUnique)
}

func (s Javazon) KillBaal() error {
	return s.killBoss(npc.BaalCrab, data.MonsterTypeUnique)
}

func (s Javazon) KillUberDuriel() error {
	return s.killBoss(npc.UberDuriel, data.MonsterTypeUnique)
}

func (s Javazon) KillUberIzual() error {
	return s.killBoss(npc.UberIzual, data.MonsterTypeUnique)
}

func (s Javazon) KillLilith() error {
	return s.killBoss(npc.Lilith, data.MonsterTypeUnique)
}

func (s Javazon) KillUberMephisto() error {
	return s.killBoss(npc.UberMephisto, data.MonsterTypeUnique)
}

func (s Javazon) KillUberDiablo() error {
	return s.killBoss(npc.UberDiablo, data.MonsterTypeUnique)
}

func (s Javazon) KillUberBaal() error {
	return s.killBoss(npc.UberBaal, data.MonsterTypeUnique)
}
