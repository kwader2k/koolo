package character

import (
	"log/slog"
	"slices"
	"sort"
	"sync/atomic"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

const (
	WarcryBarbMaxAttacksLoop = 10
	warcryBarbHorkRange      = 40
	grimWardRange            = 8
	grimWardCooldown         = 15 * time.Second
	grimWardMaxAttempts      = 3
	corpseRange              = 5
)

type WarcryBarb struct {
	BaseCharacter
	isKillingCouncil atomic.Bool
	horkedCorpses    map[data.UnitID]bool
	grimWardCasted   map[data.UnitID]bool
	lastGrimWardCast time.Time
}

func (s *WarcryBarb) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s *WarcryBarb) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{skill.BattleCommand, skill.BattleOrders, skill.Shout, skill.FindItem, skill.WarCry}

	if s.CharacterCfg.Character.WarcryBarb.UseGrimWard {
		requireKeybindings = append(requireKeybindings, skill.GrimWard)
	}
	missingKeybindings := []skill.ID{}

	if s.CharacterCfg.Character.WarcryBarb.UseHowl {
		requireKeybindings = append(requireKeybindings, skill.Howl)
	}

	if s.CharacterCfg.Character.WarcryBarb.UseBattleCry {
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

func (s *WarcryBarb) IsKillingCouncil() bool {
	return s.isKillingCouncil.Load()
}

func (s *WarcryBarb) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	blacklistedMonsters := make(map[data.UnitID]bool)

	if s.horkedCorpses == nil {
		s.horkedCorpses = make(map[data.UnitID]bool)
	}
	if s.grimWardCasted == nil {
		s.grimWardCasted = make(map[data.UnitID]bool)
	}

	var lastBattleCryCast time.Time
	var lastHowlCast time.Time
	var lastWarCryCast time.Time

	for {
		context.Get().PauseIfNotPriority()
		s.tryGrimWard()

		var id data.UnitID
		var found bool

		id, found = monsterSelector(*s.Data)
		if !found {
			if !s.isKillingCouncil.Load() {
				monstersNearby := s.countInRange(s.horkRange())
				if monstersNearby == 0 {
					s.horkCorpses(warcryBarbHorkRange)
				}
			}
			return nil
		}
		s.primaryWeapon()

		if blacklistedMonsters[id] {
			continue
		}

		if previousUnitID != int(id) {
			completedAttackLoops = 0
		}

		if !s.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		if completedAttackLoops >= WarcryBarbMaxAttacksLoop {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			continue
		}

		if monster.Stats[stat.Life] <= 0 {
			continue
		}

		s.attackWarcry(id, &lastHowlCast, &lastBattleCryCast, &lastWarCryCast)

		completedAttackLoops++
		previousUnitID = int(id)
		time.Sleep(time.Millisecond * 100)

		if !s.isKillingCouncil.Load() {
			monstersNearby := s.countInRange(s.horkRange())
			if monstersNearby <= 3 {
				s.horkCorpses(warcryBarbHorkRange)
			}
		}
	}
}

func (s *WarcryBarb) attackWarcry(
	id data.UnitID,
	lastHowlCast *time.Time,
	lastBattleCryCast *time.Time,
	lastWarCryCast *time.Time,
) {
	if s.tryHowl(id, lastHowlCast) {
		return
	}

	if s.tryBattleCry(id, lastBattleCryCast) {
		return
	}

	s.castWarCry(id, lastWarCryCast)
}

func (s *WarcryBarb) countInRange(rangeYards int) int {
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

func (s *WarcryBarb) horkRange() int {
	r := s.CharacterCfg.Character.WarcryBarb.HorkMonsterCheckRange
	if r <= 0 {
		return 7
	}
	return r
}

func (s *WarcryBarb) tryHowl(id data.UnitID, lastHowlCast *time.Time) bool {
	hasHowl := s.Data.PlayerUnit.Skills[skill.Howl].Level > 0
	if !s.CharacterCfg.Character.WarcryBarb.UseHowl || !hasHowl {
		return false
	}

	ctx := context.Get()
	ctx.PauseIfNotPriority()

	howlCooldownSeconds := s.CharacterCfg.Character.WarcryBarb.HowlCooldown
	if howlCooldownSeconds <= 0 {
		howlCooldownSeconds = 8
	}
	howlCooldown := time.Duration(howlCooldownSeconds) * time.Second

	if !lastHowlCast.IsZero() && time.Since(*lastHowlCast) < howlCooldown {
		return false
	}

	minMonsters := s.CharacterCfg.Character.WarcryBarb.HowlMinMonsters
	if minMonsters <= 0 {
		minMonsters = 4
	}

	const howlRange = 4
	if s.countInRange(howlRange) < minMonsters {
		return false
	}

	_, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.Howl)
	if !found {
		return false
	}

	*lastHowlCast = time.Now()
	time.Sleep(100 * time.Millisecond)

	err := step.SecondaryAttack(skill.Howl, id, 1, step.Distance(1, 10))
	if err != nil {
		return false
	}

	time.Sleep(300 * time.Millisecond)
	return true
}

func (s *WarcryBarb) tryBattleCry(id data.UnitID, lastBattleCryCast *time.Time) bool {
	hasBattleCry := s.Data.PlayerUnit.Skills[skill.BattleCry].Level > 0
	if !s.CharacterCfg.Character.WarcryBarb.UseBattleCry || !hasBattleCry {
		return false
	}

	ctx := context.Get()
	ctx.PauseIfNotPriority()

	manaPercentage := s.getManaPct()
	if manaPercentage < 20 {
		return false
	}

	battleCryCooldownSeconds := s.CharacterCfg.Character.WarcryBarb.BattleCryCooldown
	if battleCryCooldownSeconds <= 0 {
		battleCryCooldownSeconds = 6
	}
	battleCryCooldown := time.Duration(battleCryCooldownSeconds) * time.Second

	if !lastBattleCryCast.IsZero() && time.Since(*lastBattleCryCast) < battleCryCooldown {
		return false
	}

	minMonsters := s.CharacterCfg.Character.WarcryBarb.BattleCryMinMonsters
	if minMonsters <= 0 {
		minMonsters = 1
	}

	const battleCryRange = 4
	if s.countInRange(battleCryRange) < minMonsters {
		return false
	}

	if _, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.BattleCry); found {
		*lastBattleCryCast = time.Now()
		time.Sleep(100 * time.Millisecond)

		err := step.SecondaryAttack(skill.BattleCry, id, 1, step.Distance(1, 5))
		if err != nil {
			return false
		}

		time.Sleep(300 * time.Millisecond)
		return true
	}

	return false
}

func (s *WarcryBarb) castWarCry(id data.UnitID, lastWarCryCast *time.Time) {
	if s.hasSkill(skill.WarCry) {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.WarCry); found {
			step.SecondaryAttack(skill.WarCry, id, 1, step.Distance(1, 3))
			*lastWarCryCast = time.Now()
			time.Sleep(300 * time.Millisecond)
			return
		}
	}

	step.PrimaryAttack(id, 1, false, step.Distance(1, 3))
}

func (s *WarcryBarb) tryGrimWard() {
	if !s.CharacterCfg.Character.WarcryBarb.UseGrimWard {
		return
	}

	if !s.lastGrimWardCast.IsZero() && time.Since(s.lastGrimWardCast) < grimWardCooldown {
		return
	}

	if !s.hasSkill(skill.GrimWard) {
		return
	}

	grimWardKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.GrimWard)
	if !found {
		return
	}

	startTime := time.Now()
	timeout := 200 * time.Millisecond

	ctx := context.Get()
	maxCorpsesToCheck := 10

	corpsesToCheck := s.Data.Corpses
	if len(corpsesToCheck) > maxCorpsesToCheck {
		corpsesToCheck = corpsesToCheck[:maxCorpsesToCheck]
	}

	for _, corpse := range corpsesToCheck {
		if time.Since(startTime) > timeout {
			return
		}

		if s.grimWardCasted[corpse.UnitID] {
			continue
		}

		distance := s.PathFinder.DistanceFromMe(corpse.Position)
		if distance > grimWardRange {
			continue
		}

		if !s.grimWardValid(corpse) {
			continue
		}

		if distance > 5 {
			if time.Since(startTime) > timeout {
				return
			}
			err := step.MoveTo(corpse.Position, step.WithIgnoreMonsters())
			if err != nil {
				continue
			}
			if time.Since(startTime) > timeout {
				return
			}
			time.Sleep(time.Millisecond * 100)
			ctx.RefreshGameData()
			newDistance := s.PathFinder.DistanceFromMe(corpse.Position)
			if newDistance > grimWardRange {
				continue
			}
		}

		if time.Since(startTime) > timeout {
			return
		}

		if s.Data.PlayerUnit.RightSkill != skill.GrimWard {
			ctx.HID.PressKeyBinding(grimWardKey)
			time.Sleep(time.Millisecond * 50)
		}

		clickPos := s.clickPos(corpse)
		screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(clickPos.X, clickPos.Y)
		ctx.HID.Click(game.RightButton, screenX, screenY)

		s.grimWardCasted[corpse.UnitID] = true
		s.lastGrimWardCast = time.Now()
		return
	}

	s.lastGrimWardCast = time.Now()
}

func (s *WarcryBarb) undead(corpse data.Monster) bool {
	undeadNPCs := []npc.ID{
		npc.Zombie, npc.Skeleton, npc.SkeletonArcher,
		npc.BoneWarrior, npc.BoneArcher, npc.BoneMage,
		npc.Wraith, npc.Ghost,
		npc.Unraveler,
		npc.GhoulLord, npc.DarkOne,
		npc.Wraith2,
		npc.UndeadStygianDoll, npc.UndeadStygianDoll2,
		npc.UndeadSoulKiller, npc.UndeadSoulKiller2,
	}

	return slices.Contains(undeadNPCs, corpse.Name)
}

func (s *WarcryBarb) grimWardValid(corpse data.Monster) bool {
	if s.undead(corpse) {
		return false
	}

	if corpse.Type == data.MonsterTypeChampion ||
		corpse.Type == data.MonsterTypeUnique ||
		corpse.Type == data.MonsterTypeSuperUnique {
		return false
	}

	if corpse.Name == npc.CouncilMember || corpse.Name == npc.CouncilMember2 || corpse.Name == npc.CouncilMember3 {
		return false
	}

	invalidNPCs := []npc.ID{
		npc.OblivionKnight,
		npc.AbyssKnight,
		npc.BloodHawkNest,
		npc.DoomKnight,
		npc.GargoyleTrap,
		npc.ThornedHulk,
		npc.FlyingScimitar,
		npc.FrozenHorror,
		npc.Fetish,
		npc.HellSwarm,
		npc.BaalTentacle,
		npc.BaalTentacle2,
		npc.BaalTentacle3,
		npc.BaalTentacle4,
		npc.BaalTentacle5,
	}

	return !slices.Contains(invalidNPCs, corpse.Name)
}

func (s *WarcryBarb) clickPos(corpse data.Monster) data.Position {
	return data.Position{X: corpse.Position.X, Y: corpse.Position.Y + 1}
}

func (s *WarcryBarb) horkCorpses(maxRange int) {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	findItemKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.FindItem)
	if !found {
		return
	}

	corpses := s.horkableCorpses(s.Data.Corpses, maxRange)
	if len(corpses) == 0 {
		return
	}

	if s.horkedCorpses == nil {
		s.horkedCorpses = make(map[data.UnitID]bool)
	}
	originalSlot := ctx.Data.ActiveWeaponSlot
	swapped := false

	keepHorkSlot := func() {
		if !ctx.CharacterCfg.Character.WarcryBarb.FindItemSwitch {
			return
		}
		ctx.RefreshGameData()
		if ctx.Data.ActiveWeaponSlot != 1 {
			s.Logger.Debug("switch hork slot", "current", ctx.Data.ActiveWeaponSlot)
			s.SwapToSlot(1)
		}
	}

	if ctx.CharacterCfg.Character.WarcryBarb.FindItemSwitch {
		if s.SwapToSlot(1) {
			swapped = true
		} else {
			s.Logger.Warn("Failed to swap to secondary weapon for horking, continuing with current weapon")
		}
	}

	if swapped && !ctx.CharacterCfg.Character.WarcryBarb.FindItemSwitch {
		defer func() {
			if !s.SwapToSlot(originalSlot) {
				s.Logger.Error("CRITICAL: Failed to swap back to original weapon slot",
					"original", originalSlot,
					"current", ctx.Data.ActiveWeaponSlot)
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

	for _, corpse := range corpses {
		ctx.PauseIfNotPriority()
		keepHorkSlot()

		if s.horkedCorpses[corpse.UnitID] {
			continue
		}

		distance := s.PathFinder.DistanceFromMe(corpse.Position)
		if distance > corpseRange {
			err := step.MoveTo(corpse.Position, step.WithIgnoreMonsters(), step.WithDistanceToFinish(corpseRange))
			if err != nil {
				continue
			}
			time.Sleep(time.Millisecond * 100)
			distance = s.PathFinder.DistanceFromMe(corpse.Position)
			if distance > corpseRange {
				continue
			}
		}

		keepHorkSlot()

		if s.Data.PlayerUnit.RightSkill != skill.FindItem {
			ctx.HID.PressKeyBinding(findItemKey)
			time.Sleep(time.Millisecond * 50)
		}

		clickPos := s.clickPos(corpse)
		screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(clickPos.X, clickPos.Y)
		ctx.HID.Click(game.RightButton, screenX, screenY)

		s.horkedCorpses[corpse.UnitID] = true
		time.Sleep(time.Millisecond * 200)
	}
}

func (s *WarcryBarb) horkableCorpses(corpses data.Monsters, maxRange int) []data.Monster {
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
		if !s.isHorkable(corpse) {
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

func (s *WarcryBarb) isHorkable(corpse data.Monster) bool {
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

	if s.CharacterCfg.Character.WarcryBarb.HorkNormalMonsters {
		return true
	}

	return false
}

func (s *WarcryBarb) SwapToSlot(slot int) bool {
	ctx := context.Get()
	if !ctx.CharacterCfg.Character.WarcryBarb.FindItemSwitch {
		return false
	}
	if ctx.Data.ActiveWeaponSlot == slot {
		return true
	}

	const maxAttempts = 6
	const retryDelay = 200 * time.Millisecond

	for attempt := 0; attempt < maxAttempts; attempt++ {
		ctx.RefreshGameData()
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

func (s *WarcryBarb) primaryWeapon() bool {
	ctx := context.Get()

	if !ctx.CharacterCfg.Character.WarcryBarb.FindItemSwitch {
		return true
	}

	if ctx.Data.ActiveWeaponSlot == 1 {
		s.Logger.Warn("Detected stuck on secondary weapon, attempting to fix")
		return s.SwapToSlot(0)
	}

	return true
}

func (s *WarcryBarb) BuffSkills() []skill.ID {
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

func (s *WarcryBarb) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

func (s *WarcryBarb) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}
		return m.UnitID, true
	}, nil)
}

func (s *WarcryBarb) KillCountess() error {
	return s.killMonster(npc.DarkStalker, data.MonsterTypeSuperUnique)
}

func (s *WarcryBarb) KillAndariel() error {
	return s.killMonster(npc.Andariel, data.MonsterTypeUnique)
}

func (s *WarcryBarb) KillSummoner() error {
	return s.killMonster(npc.Summoner, data.MonsterTypeUnique)
}

func (s *WarcryBarb) KillDuriel() error {
	return s.killMonster(npc.Duriel, data.MonsterTypeUnique)
}

func (s *WarcryBarb) KillMephisto() error {
	return s.killMonster(npc.Mephisto, data.MonsterTypeUnique)
}

func (s *WarcryBarb) KillDiablo() error {
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
		return s.killMonster(npc.Diablo, data.MonsterTypeUnique)
	}
}

func (s *WarcryBarb) KillCouncil() error {
	s.isKillingCouncil.Store(true)
	defer s.isKillingCouncil.Store(false)

	err := s.killCouncil()
	if err != nil {
		return err
	}

	context.Get().EnableItemPickup()

	time.Sleep(500 * time.Millisecond)
	for i := 0; i < 2; i++ {
		s.horkCorpses(warcryBarbHorkRange)
		time.Sleep(300 * time.Millisecond)
		context.Get().RefreshGameData()
	}

	time.Sleep(500 * time.Millisecond)
	err = action.ItemPickup(warcryBarbHorkRange)
	if err != nil {
		s.Logger.Warn("Error during final item pickup after horking", "error", err)
		return err
	}

	time.Sleep(300 * time.Millisecond)
	return nil
}

func (s *WarcryBarb) killCouncil() error {
	context.Get().DisableItemPickup()
	for {
		if !s.councilAlive() {
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

func (s *WarcryBarb) councilAlive() bool {
	for _, m := range s.Data.Monsters.Enemies() {
		if (m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3) && m.Stats[stat.Life] > 0 {
			return true
		}
	}
	return false
}

func (s *WarcryBarb) KillIzual() error {
	return s.killMonster(npc.Izual, data.MonsterTypeUnique)
}

func (s *WarcryBarb) KillPindle() error {
	return s.killMonster(npc.DefiledWarrior, data.MonsterTypeSuperUnique)
}

func (s *WarcryBarb) KillNihlathak() error {
	return s.killMonster(npc.Nihlathak, data.MonsterTypeSuperUnique)
}

func (s *WarcryBarb) KillBaal() error {
	return s.killMonster(npc.BaalCrab, data.MonsterTypeUnique)
}

func (s *WarcryBarb) hasSkill(sk skill.ID) bool {
	return s.Data.PlayerUnit.Skills[sk].Level > 0
}

func (s *WarcryBarb) getManaPct() float64 {
	currentMana, foundMana := s.Data.PlayerUnit.FindStat(stat.Mana, 0)
	maxMana, foundMaxMana := s.Data.PlayerUnit.FindStat(stat.MaxMana, 0)
	if !foundMana || !foundMaxMana || maxMana.Value == 0 {
		return 0
	}
	return float64(currentMana.Value) / float64(maxMana.Value) * 100
}
