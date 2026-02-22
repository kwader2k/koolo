package character

import (
	"log/slog"
	"math"
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

type WhirlwindBarb struct {
	BaseCharacter
	isKillingCouncil atomic.Bool
	horkedCorpses    map[data.UnitID]bool
}

func (s WhirlwindBarb) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s *WhirlwindBarb) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{skill.BattleCommand, skill.BattleOrders, skill.Shout, skill.Whirlwind, skill.FindItem, skill.Berserk}
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

func (s *WhirlwindBarb) IsKillingCouncil() bool {
	return s.isKillingCouncil.Load()
}

func (s *WhirlwindBarb) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	// Safety check: ensure we're on primary weapon before fighting
	s.EnsurePrimaryWeaponEquipped()

	monsterDetected := false
	var previousEnemyId data.UnitID

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

		isImmuneToAll := monster.IsImmune(stat.FireImmune) &&
			monster.IsImmune(stat.ColdImmune) &&
			monster.IsImmune(stat.LightImmune) &&
			monster.IsImmune(stat.PoisonImmune) &&
			monster.IsImmune(stat.MagicImmune)

		physicalResist, foundPhysicalResist := monster.Stats[stat.DamageReduced]
		hasHighPhysicalResist := foundPhysicalResist && physicalResist >= 100

		isPhysicalImmune := isImmuneToAll || hasHighPhysicalResist

		//hasBerserk := s.hasSkill(skill.Berserk)
		// Already checked character has Berserk skill at key binding

		if isPhysicalImmune {
			s.PerformBerserkAttack(monster.UnitID)
		} else {
			s.PerformWhirlwindAttack(monster.UnitID)
		}
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

func (s *WhirlwindBarb) PerformWhirlwindAttack(monsterID data.UnitID) {
	ctx := context.Get()
	ctx.PauseIfNotPriority()
	monster, found := s.Data.Monsters.FindByID(monsterID)
	if !found {
		return
	}

	// Ensure Whirlwind skill is active
	whirlwindKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Whirlwind)
	if found && s.Data.PlayerUnit.RightSkill != skill.Whirlwind {
		ctx.HID.PressKeyBinding(whirlwindKey)
		time.Sleep(50 * time.Millisecond)
	}

	// Whirlwind position calculation, credit to d2bs/Kolbot
	angles := []float64{180, 175, -175, 170, -170, 165, -165, 150, -150, 135, -135, 45, -45, 90, -90}
	angles = append([]float64{120}, angles...)
	angle := math.Round(math.Atan2(float64(s.Data.PlayerUnit.Position.Y-monster.Position.Y),
		float64(s.Data.PlayerUnit.Position.X-monster.Position.X)))

	for i := 0; i < len(angles); i++ {
		coords := []float64{
			math.Round(math.Cos((angle+angles[i]*math.Pi/180))*4 + float64(monster.Position.X)),
			math.Round(math.Sin((angle+angles[i]*math.Pi/180))*4 + float64(monster.Position.Y))}
		screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(int(coords[0]), int(coords[1]))
		ctx.HID.Click(game.RightButton, screenX, screenY)
	}
}

func (s *WhirlwindBarb) PerformBerserkAttack(monsterID data.UnitID) {
	ctx := context.Get()
	ctx.PauseIfNotPriority()
	monster, found := s.Data.Monsters.FindByID(monsterID)
	if !found {
		return
	}

	// Ensure Berserk skill is active
	berserkKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Berserk)
	if found && s.Data.PlayerUnit.LeftSkill != skill.Berserk {
		ctx.HID.PressKeyBinding(berserkKey)
		time.Sleep(50 * time.Millisecond)
	}

	screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(monster.Position.X, monster.Position.Y)
	ctx.HID.Click(game.LeftButton, screenX, screenY)
}

func (s *WhirlwindBarb) FindItemOnNearbyCorpses(maxRange int) {
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

	if s.horkedCorpses == nil {
		s.horkedCorpses = make(map[data.UnitID]bool)
	}

	for _, corpse := range corpses {
		ctx.PauseIfNotPriority()

		if s.horkedCorpses[corpse.UnitID] {
			continue
		}

		distance := s.PathFinder.DistanceFromMe(corpse.Position)
		if distance > findItemRange {
			err := step.MoveTo(corpse.Position, step.WithIgnoreMonsters(), step.WithDistanceToFinish(findItemRange))
			if err != nil {
				continue
			}
			time.Sleep(time.Millisecond * 100)
			distance = s.PathFinder.DistanceFromMe(corpse.Position)
			if distance > findItemRange {
				continue
			}
		}
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

func (s *WhirlwindBarb) getHorkableCorpses(corpses data.Monsters, maxRange int) []data.Monster {
	type corpseWithDistance struct {
		corpse   data.Monster
		distance int
	}
	var horkableCorpses []corpseWithDistance
	maxCorpsesToCheck := 30
	corpsesToCheck := corpses
	if len(corpsesToCheck) > maxCorpsesToCheck {
		corpsesToCheck = corpsesToCheck[:maxCorpsesToCheck]
	}

	for _, corpse := range corpsesToCheck {
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

	result := make([]data.Monster, len(horkableCorpses))
	for i, cwd := range horkableCorpses {
		result[i] = cwd.corpse
	}

	return result
}

func (s *WhirlwindBarb) isCorpseHorkable(corpse data.Monster) bool {
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

	if s.CharacterCfg.Character.WhirlwindBarb.HorkNormalMonsters {
		return true
	}

	return false
}

func (s *WhirlwindBarb) getOptimalClickPosition(corpse data.Monster) data.Position {
	return data.Position{X: corpse.Position.X, Y: corpse.Position.Y + 1}
}

func (s *WhirlwindBarb) countInRange(rangeYards int) int {
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

func (s *WhirlwindBarb) horkRange() int {
	r := s.CharacterCfg.Character.WhirlwindBarb.HorkMonsterCheckRange
	if r <= 0 {
		return 7
	}
	return r
}

func (s *WhirlwindBarb) SwapToSlot(slot int) bool {
	ctx := context.Get()

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
func (s *WhirlwindBarb) EnsurePrimaryWeaponEquipped() bool {
	ctx := context.Get()

	// If we're on slot 1 (secondary) when we shouldn't be, swap back
	if ctx.Data.ActiveWeaponSlot == 1 {
		s.Logger.Warn("Detected stuck on secondary weapon, attempting to fix")
		return s.SwapToSlot(0)
	}

	return true
}
func (s *WhirlwindBarb) BuffSkills() []skill.ID {

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

func (s *WhirlwindBarb) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

func (s *WhirlwindBarb) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}
		return m.UnitID, true
	}, nil)
}

func (s *WhirlwindBarb) KillCountess() error {
	return s.killMonster(npc.DarkStalker, data.MonsterTypeSuperUnique)
}

func (s *WhirlwindBarb) KillAndariel() error {
	return s.killMonster(npc.Andariel, data.MonsterTypeUnique)
}

func (s *WhirlwindBarb) KillSummoner() error {
	return s.killMonster(npc.Summoner, data.MonsterTypeUnique)
}

func (s *WhirlwindBarb) KillDuriel() error {
	return s.killMonster(npc.Duriel, data.MonsterTypeUnique)
}

func (s *WhirlwindBarb) KillMephisto() error {
	return s.killMonster(npc.Mephisto, data.MonsterTypeUnique)
}

func (s *WhirlwindBarb) KillDiablo() error {
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

func (s *WhirlwindBarb) KillCouncil() error {
	s.isKillingCouncil.Store(true)
	defer s.isKillingCouncil.Store(false)

	err := s.killAllCouncilMembers()
	if err != nil {
		return err
	}

	context.Get().EnableItemPickup()

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

func (s *WhirlwindBarb) killAllCouncilMembers() error {
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

func (s *WhirlwindBarb) anyCouncilMemberAlive() bool {
	for _, m := range s.Data.Monsters.Enemies() {
		if (m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3) && m.Stats[stat.Life] > 0 {
			return true
		}

	}
	return false
}

func (s *WhirlwindBarb) KillIzual() error {
	return s.killMonster(npc.Izual, data.MonsterTypeUnique)
}

func (s *WhirlwindBarb) KillPindle() error {
	return s.killMonster(npc.DefiledWarrior, data.MonsterTypeSuperUnique)
}

func (s *WhirlwindBarb) KillNihlathak() error {
	return s.killMonster(npc.Nihlathak, data.MonsterTypeSuperUnique)
}

func (s *WhirlwindBarb) KillBaal() error {
	return s.killMonster(npc.BaalCrab, data.MonsterTypeUnique)
}
