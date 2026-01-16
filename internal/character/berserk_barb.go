package character

import (
	"log/slog"
	"sort"
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

type Berserker struct {
	BaseCharacter
	isKillingCouncil atomic.Bool
	horkedCorpses    map[data.UnitID]bool
}

const (
	maxHorkRange      = 40
	meleeRange        = 5
	maxAttackAttempts = 20
	findItemRange     = 5
)

func (s Berserker) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s *Berserker) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{skill.BattleCommand, skill.BattleOrders, skill.Shout, skill.FindItem, skill.Berserk}
	missingKeybindings := []skill.ID{}

	hasHowl := s.Data.PlayerUnit.Skills[skill.Howl].Level > 0
	if s.CharacterCfg.Character.BerserkerBarb.UseHowl && hasHowl {
		requireKeybindings = append(requireKeybindings, skill.Howl)
	}

	hasBattleCry := s.Data.PlayerUnit.Skills[skill.BattleCry].Level > 0
	if s.CharacterCfg.Character.BerserkerBarb.UseBattleCry && hasBattleCry {
		requireKeybindings = append(requireKeybindings, skill.BattleCry)
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

func (s *Berserker) IsKillingCouncil() bool {
	return s.isKillingCouncil.Load()
}

const safeMonstersForHork = 1 // allow up to 1 stray mob nearby

// Call this at the start of KillMonsterSequence to ensure we're not stuck
func (s *Berserker) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	// Safety check: ensure we're on primary weapon before fighting
	s.EnsurePrimaryWeaponEquipped()

	monsterDetected := false
	var previousEnemyId data.UnitID
	var lastHowlCast time.Time
	var lastBattleCryCast time.Time

	if s.horkedCorpses == nil {
		s.horkedCorpses = make(map[data.UnitID]bool)
	}

	for attackAttempts := 0; attackAttempts < maxAttackAttempts; attackAttempts++ {
		ctx := context.Get()
		ctx.PauseIfNotPriority()

		id, found := monsterSelector(*s.Data)
		if !found {
			if monsterDetected && !s.isKillingCouncil.Load() {
				monstersNearby := s.countInRange(s.horkRange())
				if monstersNearby <= safeMonstersForHork {
					s.FindItemOnNearbyCorpses(maxHorkRange)
					// Verify we're back on primary weapon after horking
					s.EnsurePrimaryWeaponEquipped()
				}
			}
			return nil
		}

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

		distance := s.PathFinder.DistanceFromMe(monster.Position)
		if distance > meleeRange {
			if err := step.MoveTo(monster.Position, step.WithIgnoreMonsters()); err != nil {
				s.Logger.Warn("Failed to move to monster", slog.String("error", err.Error()))
				continue
			}
		}

		hasHowl := s.Data.PlayerUnit.Skills[skill.Howl].Level > 0
		if s.CharacterCfg.Character.BerserkerBarb.UseHowl && hasHowl {
			s.PerformHowl(id, &lastHowlCast)
		}

		hasBattleCry := s.Data.PlayerUnit.Skills[skill.BattleCry].Level > 0
		if s.CharacterCfg.Character.BerserkerBarb.UseBattleCry && hasBattleCry {
			s.PerformBattleCry(id, &lastBattleCryCast)
		}

		s.PerformBerserkAttack(monster.UnitID)
		time.Sleep(50 * time.Millisecond)
	}

	if monsterDetected && !s.isKillingCouncil.Load() {
		monstersNearby := s.countInRange(s.horkRange())
		if monstersNearby <= safeMonstersForHork {
			s.FindItemOnNearbyCorpses(maxHorkRange)
			// Verify we're back on primary weapon after horking
			s.EnsurePrimaryWeaponEquipped()
		}
	}

	return nil
}

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
		time.Sleep(50 * time.Millisecond)
	}

	screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(monster.Position.X, monster.Position.Y)
	ctx.HID.Click(game.LeftButton, screenX, screenY)
}

// Helper functions
func (s *Berserker) getManaPercentage() float64 {
	currentMana, foundMana := s.Data.PlayerUnit.FindStat(stat.Mana, 0)
	maxMana, foundMaxMana := s.Data.PlayerUnit.FindStat(stat.MaxMana, 0)
	if !foundMana || !foundMaxMana || maxMana.Value == 0 {
		return 0
	}
	return float64(currentMana.Value) / float64(maxMana.Value) * 100
}

func (s *Berserker) PerformHowl(targetID data.UnitID, lastHowlCast *time.Time) bool {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	howlCooldownSeconds := s.CharacterCfg.Character.BerserkerBarb.HowlCooldown
	if howlCooldownSeconds <= 0 {
		howlCooldownSeconds = 6
	}
	howlCooldown := time.Duration(howlCooldownSeconds) * time.Second

	minMonsters := s.CharacterCfg.Character.BerserkerBarb.HowlMinMonsters
	if minMonsters <= 0 {
		minMonsters = 4
	}

	if !lastHowlCast.IsZero() && time.Since(*lastHowlCast) < howlCooldown {
		return false
	}

	const howlRange = 4
	closeMonsters := 0

	for _, m := range ctx.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}

		distance := s.PathFinder.DistanceFromMe(m.Position)
		if distance <= howlRange {
			closeMonsters++
		}
	}

	if closeMonsters < minMonsters {
		return false
	}

	_, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.Howl)
	if !found {
		return false
	}

	*lastHowlCast = time.Now()

	time.Sleep(100 * time.Millisecond)

	err := step.SecondaryAttack(skill.Howl, targetID, 1, step.Distance(1, 10))
	if err != nil {
		return false
	}

	time.Sleep(300 * time.Millisecond)

	return true
}

func (s *Berserker) PerformBattleCry(monsterID data.UnitID, lastBattleCryCast *time.Time) bool {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	manaPercentage := s.getManaPercentage()
	if manaPercentage < 20 {
		return false
	}

	battleCryCooldownSeconds := s.CharacterCfg.Character.BerserkerBarb.BattleCryCooldown
	if battleCryCooldownSeconds <= 0 {
		battleCryCooldownSeconds = 6
	}
	battleCryCooldown := time.Duration(battleCryCooldownSeconds) * time.Second

	if !lastBattleCryCast.IsZero() && time.Since(*lastBattleCryCast) < battleCryCooldown {
		return false
	}

	minMonsters := s.CharacterCfg.Character.BerserkerBarb.BattleCryMinMonsters
	if minMonsters <= 0 {
		minMonsters = 4
	}

	const battleCryRange = 4
	closeMonsters := 0

	for _, m := range ctx.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}

		distance := s.PathFinder.DistanceFromMe(m.Position)
		if distance <= battleCryRange {
			closeMonsters++
		}
	}

	if closeMonsters < minMonsters {
		return false
	}

	if _, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.BattleCry); found {
		*lastBattleCryCast = time.Now()

		time.Sleep(100 * time.Millisecond)

		err := step.SecondaryAttack(skill.BattleCry, monsterID, 1, step.Distance(1, 5))
		if err != nil {
			return false
		}

		time.Sleep(300 * time.Millisecond)

		return true
	}

	return false
}

// Improved FindItemOnNearbyCorpses with better swap handling
func (s *Berserker) FindItemOnNearbyCorpses(maxRange int) {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	// Disable item pickup while we move between corpses and cast Find Item.
	// Otherwise movement/attack clicks can accidentally grab items under the cursor.
	ctx.DisableItemPickup()
	if !s.isKillingCouncil.Load() {
		defer ctx.EnableItemPickup()
	}


	findItemKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.FindItem)
	if !found {
		return
	}

	if s.horkedCorpses == nil {
		s.horkedCorpses = make(map[data.UnitID]bool)
	}

	// Prevent the horked-corpses cache from becoming stale across games.
	// UnitIDs can be reused between games, so if everything in-range appears "already horked",
	// reset the cache and proceed.
	if len(s.horkedCorpses) > 5000 {
		s.horkedCorpses = make(map[data.UnitID]bool)
	}

	// Check if there is anything to hork before swapping weapons.
	ctx.RefreshGameData()
	initialCorpses := s.getHorkableCorpses(ctx.Data.Corpses, maxRange)
	if len(initialCorpses) == 0 {
		return
	}

	// If every corpse in-range is marked as horked, the cache is likely stale (new game / reused UnitIDs).
	allMarked := true
	for _, c := range initialCorpses {
		if !s.horkedCorpses[c.UnitID] {
			allMarked = false
			break
		}
	}
	if allMarked {
		s.horkedCorpses = make(map[data.UnitID]bool)
	}

	originalSlot := ctx.Data.ActiveWeaponSlot
	swapped := false

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
			// If swap fails, log warning but continue with current weapon
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
				// Force multiple swap attempts as last resort
				for i := 0; i < 10; i++ {
					ctx.HID.PressKey('W')
					time.Sleep(200 * time.Millisecond)
					ctx.RefreshGameData()
					if ctx.Data.ActiveWeaponSlot == originalSlot {
						s.Logger.Info("Successfully recovered weapon swap after multiple attempts")
						return
					}
				}
			}
		}()
	}

	// Safety cap: avoid infinite loops if move keeps failing.
	const maxHorkOps = 60

	for ops := 0; ops < maxHorkOps; ops++ {
		ctx.PauseIfNotPriority()
		keepHorkSlot()

		// Recompute ordering from CURRENT position.
		ctx.RefreshGameData()
		corpses := s.getHorkableCorpses(ctx.Data.Corpses, maxRange)

		// Pick nearest corpse that hasn't been processed yet.
		var corpse data.Monster
		foundCorpse := false
		for _, c := range corpses {
			if s.horkedCorpses[c.UnitID] {
				continue
			}
			corpse = c
			foundCorpse = true
			break
		}

		if !foundCorpse {
			return
		}

		distance := s.PathFinder.DistanceFromMe(corpse.Position)
		if distance > findItemRange {
			err := step.MoveTo(corpse.Position, step.WithIgnoreMonsters(), step.WithDistanceToFinish(findItemRange))
			if err != nil {
				// Mark as processed to avoid ping-pong on unreachable corpses.
				s.horkedCorpses[corpse.UnitID] = true
				continue
			}
			time.Sleep(time.Millisecond * 100)
			distance = s.PathFinder.DistanceFromMe(corpse.Position)
			if distance > findItemRange {
				s.horkedCorpses[corpse.UnitID] = true
				continue
			}
		}

		keepHorkSlot()

		// Make sure Find Item is on right-click
		if s.Data.PlayerUnit.RightSkill != skill.FindItem {
			ctx.HID.PressKeyBinding(findItemKey)
			time.Sleep(50 * time.Millisecond)
		}

		clickPos := s.getOptimalClickPosition(corpse)
		screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(clickPos.X, clickPos.Y)
		ctx.HID.Click(game.RightButton, screenX, screenY)

		s.horkedCorpses[corpse.UnitID] = true
		time.Sleep(200 * time.Millisecond)
	}
}

func (s *Berserker) getHorkableCorpses(corpses data.Monsters, maxRange int) []data.Monster {
	type corpseWithDistance struct {
		corpse   data.Monster
		distance int
	}
	var horkableCorpses []corpseWithDistance

	for _, corpse := range corpses {
		if !s.isCorpseHorkable(corpse) {
			continue
		}

		distance := s.PathFinder.DistanceFromMe(corpse.Position)
		if distance <= maxRange {
			horkableCorpses = append(horkableCorpses, corpseWithDistance{corpse: corpse, distance: distance})
		}
	}

	if len(horkableCorpses) > 1 {
		sort.Slice(horkableCorpses, func(i, j int) bool {
			return horkableCorpses[i].distance < horkableCorpses[j].distance
		})
	}

	// Cap AFTER sorting to avoid skipping nearby corpses because of arbitrary slice order.
	const maxCorpsesToCheck = 30
	if len(horkableCorpses) > maxCorpsesToCheck {
		horkableCorpses = horkableCorpses[:maxCorpsesToCheck]
	}

	result := make([]data.Monster, len(horkableCorpses))
	for i, cwd := range horkableCorpses {
		result[i] = cwd.corpse
	}

	return result
}

func (s *Berserker) isCorpseHorkable(corpse data.Monster) bool {
	unhorkableStates := []state.State{
		state.CorpseNoselect,
		state.CorpseNodraw,
		state.Revive,
		state.Redeemed,
		state.Shatter,
		state.Freeze,
		state.Restinpeace,
	}

	for _, st := range unhorkableStates {
		if corpse.States.HasState(st) {
			return false
		}
	}

	if corpse.Type == data.MonsterTypeMinion ||
		corpse.Type == data.MonsterTypeChampion ||
		corpse.Type == data.MonsterTypeUnique ||
		corpse.Type == data.MonsterTypeSuperUnique {
		return true
	}

	if s.CharacterCfg.Character.BerserkerBarb.HorkNormalMonsters {
		return true
	}

	return false
}

func (s *Berserker) getOptimalClickPosition(corpse data.Monster) data.Position {
	return data.Position{X: corpse.Position.X, Y: corpse.Position.Y + 1}
}

func (s *Berserker) countInRange(rangeYards int) int {
	count := 0
	for _, m := range s.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}
		distance := s.PathFinder.DistanceFromMe(m.Position)
		if distance <= rangeYards {
			count++
		}
	}
	return count
}

func (s *Berserker) horkRange() int {
	r := s.CharacterCfg.Character.BerserkerBarb.HorkMonsterCheckRange
	if r <= 0 {
		return 7
	}
	return r
}

// slot 0 = primary weapon, slot 1 = secondary weapon
// Improved SwapToSlot with more robust retry logic
func (s *Berserker) SwapToSlot(slot int) bool {
	ctx := context.Get()

	// If we don't want to switch for find item, just say "no-op"
	if !ctx.CharacterCfg.Character.BerserkerBarb.FindItemSwitch {
		return false
	}

	// Already on desired slot
	if ctx.Data.ActiveWeaponSlot == slot {
		return true
	}

	const maxAttempts = 6                     // Increased from 4
	const retryDelay = 200 * time.Millisecond // Increased from 150ms

	for attempt := 0; attempt < maxAttempts; attempt++ {
		// Refresh data before checking to ensure we have current state
		ctx.RefreshGameData()

		// Check if we're already on the desired slot (in case previous attempt worked)
		if ctx.Data.ActiveWeaponSlot == slot {
			return true
		}

		ctx.HID.PressKey('W')
		time.Sleep(retryDelay)
		ctx.RefreshGameData()

		if ctx.Data.ActiveWeaponSlot == slot {
			s.Logger.Debug("Successfully swapped weapon slot",
				"slot", slot,
				"attempts", attempt+1)
			return true
		}

		// If we're not on the right slot after multiple attempts, add extra delay
		if attempt >= 2 {
			time.Sleep(100 * time.Millisecond)
		}
	}

	s.Logger.Error("Failed to swap weapon slot after all attempts",
		"desired", slot,
		"current", ctx.Data.ActiveWeaponSlot,
		"attempts", maxAttempts)

	return false
}

// Alternative: Add a safety check function to call periodically
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
func (s *Berserker) BuffSkills() []skill.ID {

	skillsList := make([]skill.ID, 0)
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

func (s *Berserker) KillCouncil() error {
	s.isKillingCouncil.Store(true)
	defer s.isKillingCouncil.Store(false)

	err := s.killAllCouncilMembers()
	if err != nil {
		return err
	}

	// Keep item pickup disabled during horking movement to prevent accidental pickups.

	// Wait for corpses to settle
	time.Sleep(500 * time.Millisecond)

	// Perform horking in two passes
	for i := 0; i < 2; i++ {
		s.FindItemOnNearbyCorpses(maxHorkRange)

		// Wait between passes
		time.Sleep(300 * time.Millisecond)

		// Refresh game data to catch any new corpses
		context.Get().RefreshGameData()
	}

	// Final wait for items to drop
	time.Sleep(500 * time.Millisecond)

	// Enable item pickup only for the actual pickup routine.
	context.Get().EnableItemPickup()

	// Final item pickup
	err = action.ItemPickup(maxHorkRange)
	if err != nil {
		s.Logger.Warn("Error during final item pickup after horking", "error", err)
		return err
	}

	// Wait a moment to ensure all items are picked up
	time.Sleep(300 * time.Millisecond)

	return nil
}

func (s *Berserker) killAllCouncilMembers() error {
	context.Get().DisableItemPickup()
	for {
		if !s.anyCouncilMemberAlive() {
			s.Logger.Info("All council members have been defeated!!!!")
			return nil
		}

		err := s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			for _, m := range d.Monsters.Enemies() {
				if (m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3) && m.Stats[stat.Life] > 0 {
					return m.UnitID, true
				}
			}
			return 0, false
		}, nil)

		if err != nil {
			return err
		}
	}
}

func (s *Berserker) anyCouncilMemberAlive() bool {
	for _, m := range s.Data.Monsters.Enemies() {
		if (m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3) && m.Stats[stat.Life] > 0 {
			return true
		}

	}
	return false
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
