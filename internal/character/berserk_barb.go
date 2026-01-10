package character

import (
	"fmt"
	"log/slog"
	"sync/atomic"
	"time"

	"github.com/hectorgimenez/koolo/internal/action"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

// =============================================================================
// Constants
// =============================================================================

const (
	// Combat ranges
	maxHorkRange  = 40 // Maximum range for horking corpses
	meleeRange    = 5  // Maximum distance before moving closer to monster
	findItemRange = 5  // Range for Find Item skill

	// Attack limits
	maxAttackAttempts = 20 // Max attacks on single target before giving up

	// Safety timeouts to prevent infinite loops
	berserkKillTimeout      = 120 * time.Second // Global timeout for KillMonsterSequence
	killAllCouncilTimeout   = 180 * time.Second // Timeout for killing all council members
	weaponSwapTimeout       = 2 * time.Second   // Max time to spend on weapon swap attempts
	weaponSwapMaxAttempts   = 6                 // Max attempts to swap weapon
	weaponSwapRetryDelay    = 200 * time.Millisecond
	weaponSwapRecoveryDelay = 100 * time.Millisecond

	// Skill delays
	skillActivationDelay = 50 * time.Millisecond  // Delay after activating a skill
	preSkillDelay        = 100 * time.Millisecond // Delay before using Howl/BattleCry
	postSkillDelay       = 300 * time.Millisecond // Delay after using Howl/BattleCry
	postHorkDelay        = 200 * time.Millisecond // Delay after horking a corpse
	postMoveDelay        = 100 * time.Millisecond // Delay after moving to corpse
	attackLoopDelay      = 50 * time.Millisecond  // Delay between attack attempts

	// Council killing delays
	councilCorpseSettleDelay = 500 * time.Millisecond // Wait for corpses after council kill
	councilHorkPassDelay     = 300 * time.Millisecond // Delay between hork passes
	councilFinalPickupDelay  = 500 * time.Millisecond // Wait before final item pickup
	councilPostPickupDelay   = 300 * time.Millisecond // Wait after item pickup

	// Horking safety
	safeMonstersForHork = 1  // Allow up to 1 stray mob nearby before horking
	maxCorpsesToCheck   = 30 // Limit corpses to check for performance

	// Skill ranges
	howlRange      = 4 // Range to check for monsters before using Howl
	battleCryRange = 4 // Range to check for monsters before using BattleCry

	// Mana threshold
	minManaPercentForBattleCry = 20 // Minimum mana % to cast Battle Cry

	// Default config values (used when config value <= 0)
	defaultHowlCooldown          = 6
	defaultHowlMinMonsters       = 4
	defaultBattleCryCooldown     = 6
	defaultBattleCryMinMonsters  = 4
	defaultHorkMonsterCheckRange = 7
)

// Unhorkable states - corpses with these states cannot be horked
// Package-level to avoid allocation on every call
var unhorkableStates = []state.State{
	state.CorpseNoselect,
	state.CorpseNodraw,
	state.Revive,
	state.Redeemed,
	state.Shatter,
	state.Freeze,
	state.Restinpeace,
}

// =============================================================================
// Berserker struct
// =============================================================================

type Berserker struct {
	BaseCharacter
	isKillingCouncil atomic.Bool          // Flag to prevent horking during council fight
	horkedCorpses    map[data.UnitID]bool // Track already horked corpses
}

func (s *Berserker) IsKillingCouncil() bool {
	return s.isKillingCouncil.Load()
}

// ShouldIgnoreMonster implements Character interface
// Note: Uses value receiver as required by Character interface
func (s Berserker) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s *Berserker) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{
		skill.BattleCommand,
		skill.BattleOrders,
		skill.Shout,
		skill.FindItem,
		skill.Berserk,
	}
	missingKeybindings := []skill.ID{}

	// Add Howl if configured and available
	if s.CharacterCfg.Character.BerserkerBarb.UseHowl {
		if s.Data.PlayerUnit.Skills[skill.Howl].Level > 0 {
			requireKeybindings = append(requireKeybindings, skill.Howl)
		}
	}

	// Add Battle Cry if configured and available
	if s.CharacterCfg.Character.BerserkerBarb.UseBattleCry {
		if s.Data.PlayerUnit.Skills[skill.BattleCry].Level > 0 {
			requireKeybindings = append(requireKeybindings, skill.BattleCry)
		}
	}

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

// =============================================================================
// Main Kill Sequence
// =============================================================================

func (s *Berserker) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	// Safety check: ensure we're on primary weapon before fighting
	s.EnsurePrimaryWeaponEquipped()

	// Initialize horked corpses map if needed
	if s.horkedCorpses == nil {
		s.horkedCorpses = make(map[data.UnitID]bool)
	}

	monsterDetected := false
	var previousEnemyId data.UnitID
	var lastHowlCast time.Time
	var lastBattleCryCast time.Time

	// Safety timeout to prevent infinite loops
	startTime := time.Now()

	for attackAttempts := 0; attackAttempts < maxAttackAttempts; attackAttempts++ {
		// CRITICAL: Global timeout check - prevents infinite loop crashes
		if time.Since(startTime) > berserkKillTimeout {
			s.Logger.Error("KillMonsterSequence timeout reached, breaking loop to prevent crash",
				slog.Duration("elapsed", time.Since(startTime)))
			return fmt.Errorf("kill sequence timeout after %v", berserkKillTimeout)
		}

		ctx := context.Get()
		ctx.PauseIfNotPriority()

		id, found := monsterSelector(*s.Data)
		if !found {
			s.tryHorkNearbyCorpses(monsterDetected)
			return nil
		}

		// Reset attack counter on new target, but NOT the global timeout
		if id != previousEnemyId {
			previousEnemyId = id
			attackAttempts = 0
		}

		monsterDetected = true

		if !s.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		monster, monsterFound := s.Data.Monsters.FindByID(id)
		if !monsterFound || monster.Stats[stat.Life] <= 0 {
			continue
		}

		// Move closer if too far for melee
		distance := s.PathFinder.DistanceFromMe(monster.Position)
		if distance > meleeRange {
			if err := step.MoveTo(monster.Position, step.WithIgnoreMonsters()); err != nil {
				s.Logger.Warn("Failed to move to monster", slog.String("error", err.Error()))
				continue
			}
		}

		// Perform war cries if configured
		s.tryPerformWarCries(id, &lastHowlCast, &lastBattleCryCast)

		// Execute Berserk attack
		s.PerformBerserkAttack(monster.UnitID)
		time.Sleep(attackLoopDelay)
	}

	// After max attempts, try horking if safe
	s.tryHorkNearbyCorpses(monsterDetected)

	return nil
}

// tryHorkNearbyCorpses attempts to hork corpses if conditions are met
// Does NOT hork during council fight (isKillingCouncil flag)
func (s *Berserker) tryHorkNearbyCorpses(monsterDetected bool) {
	if !monsterDetected {
		return
	}

	// Don't hork during council fight - we'll do it all at once after
	if s.isKillingCouncil.Load() {
		return
	}

	monstersNearby := s.countMonstersInRange(s.getHorkCheckRange(), safeMonstersForHork+1)
	if monstersNearby <= safeMonstersForHork {
		s.FindItemOnNearbyCorpses(maxHorkRange)
		// Verify we're back on primary weapon after horking
		s.EnsurePrimaryWeaponEquipped()
	}
}

// tryPerformWarCries executes Howl and Battle Cry if configured and conditions are met
func (s *Berserker) tryPerformWarCries(targetID data.UnitID, lastHowlCast, lastBattleCryCast *time.Time) {
	cfg := s.CharacterCfg.Character.BerserkerBarb

	// Try Howl
	if cfg.UseHowl && s.Data.PlayerUnit.Skills[skill.Howl].Level > 0 {
		s.PerformHowl(targetID, lastHowlCast)
	}

	// Try Battle Cry
	if cfg.UseBattleCry && s.Data.PlayerUnit.Skills[skill.BattleCry].Level > 0 {
		s.PerformBattleCry(targetID, lastBattleCryCast)
	}
}

// =============================================================================
// Combat Skills
// =============================================================================

func (s *Berserker) PerformBerserkAttack(monsterID data.UnitID) {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	monster, found := s.Data.Monsters.FindByID(monsterID)
	if !found {
		return
	}

	// Ensure Berserk skill is active
	berserkKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Berserk)
	if found && s.Data.PlayerUnit.RightSkill != skill.Berserk {
		ctx.HID.PressKeyBinding(berserkKey)
		time.Sleep(skillActivationDelay)
	}

	screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(monster.Position.X, monster.Position.Y)
	ctx.HID.Click(game.LeftButton, screenX, screenY)
}

func (s *Berserker) PerformHowl(targetID data.UnitID, lastHowlCast *time.Time) bool {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	// Check cooldown
	cooldown := s.getHowlCooldown()
	if !lastHowlCast.IsZero() && time.Since(*lastHowlCast) < cooldown {
		return false
	}

	// Check minimum monsters requirement with early exit
	minMonsters := s.getHowlMinMonsters()
	closeMonsters := s.countMonstersInRange(howlRange, minMonsters)
	if closeMonsters < minMonsters {
		return false
	}

	// Check keybinding
	if _, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.Howl); !found {
		return false
	}

	*lastHowlCast = time.Now()
	time.Sleep(preSkillDelay)

	err := step.SecondaryAttack(skill.Howl, targetID, 1, step.Distance(1, 10))
	if err != nil {
		return false
	}

	time.Sleep(postSkillDelay)
	return true
}

func (s *Berserker) PerformBattleCry(monsterID data.UnitID, lastBattleCryCast *time.Time) bool {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	// Check mana
	if s.getManaPercentage() < minManaPercentForBattleCry {
		return false
	}

	// Check cooldown
	cooldown := s.getBattleCryCooldown()
	if !lastBattleCryCast.IsZero() && time.Since(*lastBattleCryCast) < cooldown {
		return false
	}

	// Check minimum monsters requirement with early exit
	minMonsters := s.getBattleCryMinMonsters()
	closeMonsters := s.countMonstersInRange(battleCryRange, minMonsters)
	if closeMonsters < minMonsters {
		return false
	}

	// Check keybinding
	if _, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.BattleCry); !found {
		return false
	}

	*lastBattleCryCast = time.Now()
	time.Sleep(preSkillDelay)

	err := step.SecondaryAttack(skill.BattleCry, monsterID, 1, step.Distance(1, 5))
	if err != nil {
		return false
	}

	time.Sleep(postSkillDelay)
	return true
}

// =============================================================================
// Find Item (Horking)
// =============================================================================

func (s *Berserker) FindItemOnNearbyCorpses(maxRange int) {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	findItemKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.FindItem)
	if !found {
		return
	}

	corpses := s.getHorkableCorpses(s.Data.Corpses, maxRange)
	if len(corpses) == 0 {
		return
	}

	// Initialize horked corpses map if needed
	if s.horkedCorpses == nil {
		s.horkedCorpses = make(map[data.UnitID]bool)
	}

	originalSlot := ctx.Data.ActiveWeaponSlot
	swapped := false

	// Helper to ensure we stay on hork slot
	keepHorkSlot := func() {
		if !ctx.CharacterCfg.Character.BerserkerBarb.FindItemSwitch {
			return
		}
		ctx.RefreshGameData()
		if ctx.Data.ActiveWeaponSlot != 1 {
			s.Logger.Debug("switch hork slot", "current", ctx.Data.ActiveWeaponSlot)
			s.SwapToSlot(1)
		}
	}

	// Swap to hork slot if configured
	if ctx.CharacterCfg.Character.BerserkerBarb.FindItemSwitch {
		if s.SwapToSlot(1) {
			swapped = true
		} else {
			s.Logger.Warn("Failed to swap to secondary weapon for horking, continuing with current weapon")
		}
	}

	// Use defer to ALWAYS ensure we swap back, even if panic or early return
	if swapped {
		defer func() {
			if !s.SwapToSlot(originalSlot) {
				s.Logger.Error("CRITICAL: Failed to swap back to original weapon slot",
					"original", originalSlot,
					"current", ctx.Data.ActiveWeaponSlot)
				// Force multiple swap attempts as last resort with timeout
				swapStart := time.Now()
				for i := 0; i < 10 && time.Since(swapStart) < weaponSwapTimeout; i++ {
					ctx.HID.PressKey('W')
					time.Sleep(weaponSwapRetryDelay)
					ctx.RefreshGameData()
					if ctx.Data.ActiveWeaponSlot == originalSlot {
						s.Logger.Info("Successfully recovered weapon swap after multiple attempts")
						return
					}
				}
			}
		}()
	}

	// Hork each corpse
	for _, corpse := range corpses {
		ctx.PauseIfNotPriority()
		keepHorkSlot()

		// Skip already horked corpses
		if s.horkedCorpses[corpse.UnitID] {
			continue
		}

		// Move to corpse if too far
		distance := s.PathFinder.DistanceFromMe(corpse.Position)
		if distance > findItemRange {
			err := step.MoveTo(corpse.Position, step.WithIgnoreMonsters(), step.WithDistanceToFinish(findItemRange))
			if err != nil {
				continue
			}
			time.Sleep(postMoveDelay)

			// Verify we're close enough after move
			distance = s.PathFinder.DistanceFromMe(corpse.Position)
			if distance > findItemRange {
				continue
			}
		}

		keepHorkSlot()

		// Make sure Find Item is on right-click
		if s.Data.PlayerUnit.RightSkill != skill.FindItem {
			ctx.HID.PressKeyBinding(findItemKey)
			time.Sleep(skillActivationDelay)
		}

		// Click on corpse
		clickPos := s.getOptimalClickPosition(corpse)
		screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(clickPos.X, clickPos.Y)
		ctx.HID.Click(game.RightButton, screenX, screenY)

		s.horkedCorpses[corpse.UnitID] = true
		time.Sleep(postHorkDelay)
	}
}

// getHorkableCorpses returns corpses that can be horked, sorted by distance
// Optimized: uses linear scan for finding closest instead of full sort when few corpses
func (s *Berserker) getHorkableCorpses(corpses data.Monsters, maxRange int) []data.Monster {
	type corpseWithDistance struct {
		corpse   data.Monster
		distance int
	}

	// Limit corpses to check for performance
	corpsesToCheck := corpses
	if len(corpsesToCheck) > maxCorpsesToCheck {
		corpsesToCheck = corpsesToCheck[:maxCorpsesToCheck]
	}

	// Collect horkable corpses with distances
	horkableCorpses := make([]corpseWithDistance, 0, len(corpsesToCheck))
	for i := range corpsesToCheck {
		if !s.isCorpseHorkable(corpsesToCheck[i]) {
			continue
		}
		distance := s.PathFinder.DistanceFromMe(corpsesToCheck[i].Position)
		if distance <= maxRange {
			horkableCorpses = append(horkableCorpses, corpseWithDistance{
				corpse:   corpsesToCheck[i],
				distance: distance,
			})
		}
	}

	if len(horkableCorpses) == 0 {
		return nil
	}

	// Sort by distance (closest first)
	// For small slices this is fast enough, and we need sorted order for efficiency
	if len(horkableCorpses) > 1 {
		// Simple insertion sort for small slices - faster than sort.Slice for n < 20
		for i := 1; i < len(horkableCorpses); i++ {
			j := i
			for j > 0 && horkableCorpses[j-1].distance > horkableCorpses[j].distance {
				horkableCorpses[j-1], horkableCorpses[j] = horkableCorpses[j], horkableCorpses[j-1]
				j--
			}
		}
	}

	// Extract sorted corpses
	result := make([]data.Monster, len(horkableCorpses))
	for i, cwd := range horkableCorpses {
		result[i] = cwd.corpse
	}

	return result
}

// isCorpseHorkable checks if a corpse can be horked
func (s *Berserker) isCorpseHorkable(corpse data.Monster) bool {
	// Check unhorkable states (using package-level slice to avoid allocation)
	for _, st := range unhorkableStates {
		if corpse.States.HasState(st) {
			return false
		}
	}

	// Always hork elite corpses
	if corpse.Type == data.MonsterTypeMinion ||
		corpse.Type == data.MonsterTypeChampion ||
		corpse.Type == data.MonsterTypeUnique ||
		corpse.Type == data.MonsterTypeSuperUnique {
		return true
	}

	// Hork normal monsters only if configured
	if s.CharacterCfg.Character.BerserkerBarb.HorkNormalMonsters {
		return true
	}

	return false
}

func (s *Berserker) getOptimalClickPosition(corpse data.Monster) data.Position {
	return data.Position{X: corpse.Position.X, Y: corpse.Position.Y + 1}
}

// =============================================================================
// Weapon Swap
// =============================================================================

// SwapToSlot swaps to the specified weapon slot (0 = primary, 1 = secondary)
func (s *Berserker) SwapToSlot(slot int) bool {
	ctx := context.Get()

	// If weapon switch for find item is disabled, no-op
	if !ctx.CharacterCfg.Character.BerserkerBarb.FindItemSwitch {
		return false
	}

	// Already on desired slot
	if ctx.Data.ActiveWeaponSlot == slot {
		return true
	}

	swapStart := time.Now()
	for attempt := 0; attempt < weaponSwapMaxAttempts; attempt++ {
		// Timeout check
		if time.Since(swapStart) > weaponSwapTimeout {
			break
		}

		// Refresh and check if already on correct slot
		ctx.RefreshGameData()
		if ctx.Data.ActiveWeaponSlot == slot {
			return true
		}

		ctx.HID.PressKey('W')
		time.Sleep(weaponSwapRetryDelay)
		ctx.RefreshGameData()

		if ctx.Data.ActiveWeaponSlot == slot {
			s.Logger.Debug("Successfully swapped weapon slot",
				"slot", slot,
				"attempts", attempt+1)
			return true
		}

		// Extra delay after multiple failed attempts
		if attempt >= 2 {
			time.Sleep(weaponSwapRecoveryDelay)
		}
	}

	s.Logger.Error("Failed to swap weapon slot after all attempts",
		"desired", slot,
		"current", ctx.Data.ActiveWeaponSlot,
		"attempts", weaponSwapMaxAttempts)

	return false
}

// EnsurePrimaryWeaponEquipped ensures we're on primary weapon slot
func (s *Berserker) EnsurePrimaryWeaponEquipped() bool {
	ctx := context.Get()

	// If not using weapon switch feature, nothing to check
	if !ctx.CharacterCfg.Character.BerserkerBarb.FindItemSwitch {
		return true
	}

	// If we're on slot 1 (secondary) when we shouldn't be, swap back
	if ctx.Data.ActiveWeaponSlot == 1 {
		s.Logger.Warn("Detected stuck on secondary weapon, attempting to fix")
		return s.SwapToSlot(0)
	}

	return true
}

// =============================================================================
// Helper Functions
// =============================================================================

func (s *Berserker) getManaPercentage() int {
	currentMana, foundMana := s.Data.PlayerUnit.FindStat(stat.Mana, 0)
	maxMana, foundMaxMana := s.Data.PlayerUnit.FindStat(stat.MaxMana, 0)
	if !foundMana || !foundMaxMana || maxMana.Value == 0 {
		return 0
	}
	// Integer math to avoid float
	return currentMana.Value * 100 / maxMana.Value
}

// countMonstersInRange counts alive monsters within range, with early exit optimization
// If earlyExitThreshold > 0, returns as soon as count reaches threshold
func (s *Berserker) countMonstersInRange(rangeYards int, earlyExitThreshold int) int {
	count := 0
	for _, m := range s.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}
		distance := s.PathFinder.DistanceFromMe(m.Position)
		if distance <= rangeYards {
			count++
			// Early exit if we've reached threshold
			if earlyExitThreshold > 0 && count >= earlyExitThreshold {
				return count
			}
		}
	}
	return count
}

// Config getters with defaults
func (s *Berserker) getHorkCheckRange() int {
	r := s.CharacterCfg.Character.BerserkerBarb.HorkMonsterCheckRange
	if r <= 0 {
		return defaultHorkMonsterCheckRange
	}
	return r
}

func (s *Berserker) getHowlCooldown() time.Duration {
	seconds := s.CharacterCfg.Character.BerserkerBarb.HowlCooldown
	if seconds <= 0 {
		seconds = defaultHowlCooldown
	}
	return time.Duration(seconds) * time.Second
}

func (s *Berserker) getHowlMinMonsters() int {
	min := s.CharacterCfg.Character.BerserkerBarb.HowlMinMonsters
	if min <= 0 {
		return defaultHowlMinMonsters
	}
	return min
}

func (s *Berserker) getBattleCryCooldown() time.Duration {
	seconds := s.CharacterCfg.Character.BerserkerBarb.BattleCryCooldown
	if seconds <= 0 {
		seconds = defaultBattleCryCooldown
	}
	return time.Duration(seconds) * time.Second
}

func (s *Berserker) getBattleCryMinMonsters() int {
	min := s.CharacterCfg.Character.BerserkerBarb.BattleCryMinMonsters
	if min <= 0 {
		return defaultBattleCryMinMonsters
	}
	return min
}

// =============================================================================
// Buff Skills
// =============================================================================

func (s *Berserker) BuffSkills() []skill.ID {
	skillsList := make([]skill.ID, 0, 3)

	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.BattleCommand); found {
		skillsList = append(skillsList, skill.BattleCommand)
	}
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Shout); found {
		skillsList = append(skillsList, skill.Shout)
	}
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.BattleOrders); found {
		skillsList = append(skillsList, skill.BattleOrders)
	}

	return skillsList
}

func (s *Berserker) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

// =============================================================================
// Boss Kills
// =============================================================================

func (s *Berserker) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}
		return m.UnitID, true
	}, nil)
}

func (s *Berserker) KillCountess() error {
	return s.killMonster(npc.DarkStalker, data.MonsterTypeSuperUnique)
}

func (s *Berserker) KillAndariel() error {
	return s.killMonster(npc.Andariel, data.MonsterTypeUnique)
}

func (s *Berserker) KillSummoner() error {
	return s.killMonster(npc.Summoner, data.MonsterTypeUnique)
}

func (s *Berserker) KillDuriel() error {
	return s.killMonster(npc.Duriel, data.MonsterTypeUnique)
}

func (s *Berserker) KillMephisto() error {
	return s.killMonster(npc.Mephisto, data.MonsterTypeUnique)
}

func (s *Berserker) KillDiablo() error {
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

		return s.killMonster(npc.Diablo, data.MonsterTypeUnique)
	}
}

func (s *Berserker) KillIzual() error {
	return s.killMonster(npc.Izual, data.MonsterTypeUnique)
}

func (s *Berserker) KillPindle() error {
	return s.killMonster(npc.DefiledWarrior, data.MonsterTypeSuperUnique)
}

func (s *Berserker) KillNihlathak() error {
	return s.killMonster(npc.Nihlathak, data.MonsterTypeSuperUnique)
}

func (s *Berserker) KillBaal() error {
	return s.killMonster(npc.BaalCrab, data.MonsterTypeUnique)
}

// =============================================================================
// Council Kill (Travincal) - Special handling with horking
// =============================================================================

func (s *Berserker) KillCouncil() error {
	// Set flag to prevent horking during council fight
	s.isKillingCouncil.Store(true)
	defer s.isKillingCouncil.Store(false)

	// Clear horked corpses map for fresh council run
	// This ensures we don't skip corpses due to stale data from previous runs
	s.horkedCorpses = make(map[data.UnitID]bool)

	err := s.killAllCouncilMembers()
	if err != nil {
		return err
	}

	context.Get().EnableItemPickup()

	// Wait for corpses to settle
	time.Sleep(councilCorpseSettleDelay)

	// Perform horking in two passes
	for i := 0; i < 2; i++ {
		s.FindItemOnNearbyCorpses(maxHorkRange)
		time.Sleep(councilHorkPassDelay)

		// Refresh game data to catch any new corpses
		context.Get().RefreshGameData()
	}

	// Final wait for items to drop
	time.Sleep(councilFinalPickupDelay)

	// Final item pickup
	err = action.ItemPickup(maxHorkRange)
	if err != nil {
		s.Logger.Warn("Error during final item pickup after horking", "error", err)
		return err
	}

	// Wait a moment to ensure all items are picked up
	time.Sleep(councilPostPickupDelay)

	return nil
}

func (s *Berserker) killAllCouncilMembers() error {
	context.Get().DisableItemPickup()

	// Safety timeout to prevent infinite loop
	startTime := time.Now()

	for {
		// CRITICAL: Timeout check
		if time.Since(startTime) > killAllCouncilTimeout {
			s.Logger.Error("killAllCouncilMembers timeout reached, breaking loop",
				slog.Duration("elapsed", time.Since(startTime)))
			return fmt.Errorf("council kill timeout after %v", killAllCouncilTimeout)
		}

		if !s.anyCouncilMemberAlive() {
			s.Logger.Info("All council members have been defeated!!!!")
			return nil
		}

		err := s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			// Find closest alive council member
			return s.findClosestCouncilMember(d)
		}, nil)

		if err != nil {
			return err
		}
	}
}

// findClosestCouncilMember finds the nearest alive council member
// O(n) linear scan instead of creating slice and sorting
func (s *Berserker) findClosestCouncilMember(d game.Data) (data.UnitID, bool) {
	var closestID data.UnitID
	closestDist := -1

	for _, m := range d.Monsters.Enemies() {
		// Only council members
		if m.Name != npc.CouncilMember && m.Name != npc.CouncilMember2 && m.Name != npc.CouncilMember3 {
			continue
		}

		// Must be alive
		if m.Stats[stat.Life] <= 0 {
			continue
		}

		dist := s.PathFinder.DistanceFromMe(m.Position)
		if closestDist < 0 || dist < closestDist {
			closestDist = dist
			closestID = m.UnitID
		}
	}

	return closestID, closestDist >= 0
}

func (s *Berserker) anyCouncilMemberAlive() bool {
	for _, m := range s.Data.Monsters.Enemies() {
		if m.Name != npc.CouncilMember && m.Name != npc.CouncilMember2 && m.Name != npc.CouncilMember3 {
			continue
		}
		if m.Stats[stat.Life] > 0 {
			return true
		}
	}
	return false
}
