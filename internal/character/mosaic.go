package character

import (
	"fmt"
	"log/slog"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

const (
	// Charge thresholds for martial arts skills
	// These values are critical for Mosaic build mechanics - DO NOT CHANGE
	maxChargesTigerStrike    = 3
	maxChargesCobraStrike    = 3
	maxChargesPhoenixStrike  = 2 // Phoenix Strike uses only 2 charges for optimal meteor proc
	maxChargesClawsOfThunder = 3
	maxChargesBladesOfIce    = 3
	maxChargesFistsOfFire    = 3

	// Combat distance settings
	mosaicMeleeRange     = 3 // Max distance before moving closer to monster
	mosaicAttackMinRange = 1 // Minimum attack distance
	mosaicAttackMaxRange = 2 // Maximum attack distance

	// Timing and safety
	mosaicRefreshInterval = 100 * time.Millisecond // Game data refresh rate (10 FPS)
	mosaicKillTimeout     = 120 * time.Second      // Safety timeout to prevent infinite loops
)

type MosaicSin struct {
	BaseCharacter
}

func (s MosaicSin) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s MosaicSin) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{
		skill.TigerStrike,
		skill.CobraStrike,
		skill.PhoenixStrike,
		skill.ClawsOfThunder,
		skill.BladesOfIce,
		skill.DragonTalon, // Finisher kick - required for packet-based skill selection
		skill.TomeOfTownPortal,
	}
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

// chargeState holds the current charge information for all martial arts skills
// This is used to track progress and determine which skill to use next
type chargeState struct {
	tigerCharges   int
	cobraCharges   int
	phoenixCharges int
	clawsCharges   int
	bladesCharges  int
	fistsCharges   int

	// Whether the stat was found (important for edge case handling)
	tigerFound   bool
	cobraFound   bool
	phoenixFound bool
	clawsFound   bool
	bladesFound  bool
	fistsFound   bool

	// Whether the state is active (used for GUI display)
	hasTigerState   bool
	hasCobraState   bool
	hasPhoenixState bool
	hasClawsState   bool
	hasBladesState  bool
	hasFistsState   bool
}

// getChargeState reads all charge-related stats and states from player unit
// This consolidates all lookups into a single function for clarity
func (s MosaicSin) getChargeState(ctx *context.Status) chargeState {
	cs := chargeState{}
	playerUnit := ctx.Data.PlayerUnit

	// Read charge counts from stats (preserving found flag for original logic compatibility)
	if tigerStat, found := playerUnit.Stats.FindStat(stat.ProgressiveDamage, 0); found {
		cs.tigerCharges = tigerStat.Value
		cs.tigerFound = true
	}
	if cobraStat, found := playerUnit.Stats.FindStat(stat.ProgressiveSteal, 0); found {
		cs.cobraCharges = cobraStat.Value
		cs.cobraFound = true
	}
	if phoenixStat, found := playerUnit.Stats.FindStat(stat.ProgressiveOther, 0); found {
		cs.phoenixCharges = phoenixStat.Value
		cs.phoenixFound = true
	}
	if clawsStat, found := playerUnit.Stats.FindStat(stat.ProgressiveLightning, 0); found {
		cs.clawsCharges = clawsStat.Value
		cs.clawsFound = true
	}
	if bladesStat, found := playerUnit.Stats.FindStat(stat.ProgressiveCold, 0); found {
		cs.bladesCharges = bladesStat.Value
		cs.bladesFound = true
	}
	if fistsStat, found := playerUnit.Stats.FindStat(stat.ProgressiveFire, 0); found {
		cs.fistsCharges = fistsStat.Value
		cs.fistsFound = true
	}

	// Read states (these indicate if we have ANY charges - important for GUI display)
	cs.hasTigerState = playerUnit.States.HasState(state.Tigerstrike)
	cs.hasCobraState = playerUnit.States.HasState(state.Cobrastrike)
	cs.hasPhoenixState = playerUnit.States.HasState(state.Phoenixstrike)
	cs.hasClawsState = playerUnit.States.HasState(state.Clawsofthunder)
	cs.hasBladesState = playerUnit.States.HasState(state.Bladesofice)
	cs.hasFistsState = playerUnit.States.HasState(state.Fistsoffire)

	return cs
}

// needsMoreCharges checks if a skill needs more charges to reach its maximum
// Original logic: !hasState || (statFound && charges < max)
// This means: build if no state, OR if we have stat tracking and charges aren't full
func needsMoreCharges(hasState bool, statFound bool, currentCharges, maxCharges int) bool {
	return !hasState || (statFound && currentCharges < maxCharges)
}

// allChargesReady checks if all enabled skills have reached their charge thresholds
func (s MosaicSin) allChargesReady(ctx *context.Status, cs chargeState) bool {
	cfg := ctx.CharacterCfg.Character.MosaicSin

	// Tiger Strike check
	if cfg.UseTigerStrike && needsMoreCharges(cs.hasTigerState, cs.tigerFound, cs.tigerCharges, maxChargesTigerStrike) {
		return false
	}

	// Cobra Strike check
	if cfg.UseCobraStrike && needsMoreCharges(cs.hasCobraState, cs.cobraFound, cs.cobraCharges, maxChargesCobraStrike) {
		return false
	}

	// Phoenix Strike is always used - check it
	if needsMoreCharges(cs.hasPhoenixState, cs.phoenixFound, cs.phoenixCharges, maxChargesPhoenixStrike) {
		return false
	}

	// Claws of Thunder check
	if cfg.UseClawsOfThunder && needsMoreCharges(cs.hasClawsState, cs.clawsFound, cs.clawsCharges, maxChargesClawsOfThunder) {
		return false
	}

	// Blades of Ice check
	if cfg.UseBladesOfIce && needsMoreCharges(cs.hasBladesState, cs.bladesFound, cs.bladesCharges, maxChargesBladesOfIce) {
		return false
	}

	// Fists of Fire check
	if cfg.UseFistsOfFire && needsMoreCharges(cs.hasFistsState, cs.fistsFound, cs.fistsCharges, maxChargesFistsOfFire) {
		return false
	}

	return true
}

func (s MosaicSin) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	ctx := context.Get()
	ctx.RefreshGameData()
	lastRefresh := time.Now()

	// Safety timeout to prevent infinite loops (fixes crash issue)
	startTime := time.Now()

	for {
		// CRITICAL: Safety timeout check - prevents infinite loop crashes
		if time.Since(startTime) > mosaicKillTimeout {
			s.Logger.Error("KillMonsterSequence timeout reached, breaking loop to prevent crash",
				slog.Duration("elapsed", time.Since(startTime)))
			return fmt.Errorf("kill sequence timeout after %v", mosaicKillTimeout)
		}

		ctx.PauseIfNotPriority()

		// Throttle game data refresh to reduce CPU usage (10 times per second max)
		if time.Since(lastRefresh) > mosaicRefreshInterval {
			ctx.RefreshGameData()
			lastRefresh = time.Now()
		}

		// Find target monster
		id, found := monsterSelector(*s.Data)
		if !found {
			return nil
		}

		// Pre-battle checks (immunities, etc.)
		if !s.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		// Validate monster exists and get its data
		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			s.Logger.Debug("Monster not found", slog.Any("monsterID", id))
			return nil
		}

		// Check if monster is still alive
		if monster.Stats[stat.Life] <= 0 {
			return nil
		}

		// Move closer if we're too far for melee
		if ctx.PathFinder.DistanceFromMe(monster.Position) > mosaicMeleeRange {
			if err := step.MoveTo(monster.Position, step.WithIgnoreMonsters()); err != nil {
				s.Logger.Debug("Failed to move to monster position", slog.String("error", err.Error()))
				continue
			}
		}

		// Re-check monster alive after movement (monster might have died or despawned)
		if !s.isMonsterAlive(id) {
			return nil
		}

		// Get current charge state for all skills
		cs := s.getChargeState(ctx)
		cfg := ctx.CharacterCfg.Character.MosaicSin

		// ============================================================
		// CHARGE BUILDING SEQUENCE
		// Order is important for Mosaic build optimization!
		// Each skill is checked and built to max charges before moving to next
		// ============================================================

		// 1. Tiger Strike - 3 charges (life steal synergy)
		if cfg.UseTigerStrike {
			if needsMoreCharges(cs.hasTigerState, cs.tigerFound, cs.tigerCharges, maxChargesTigerStrike) {
				step.SecondaryAttack(skill.TigerStrike, id, 1)
				continue
			}
		}

		// Check if monster died during charge building
		if !s.isMonsterAlive(id) {
			return nil
		}

		// 2. Cobra Strike - 3 charges (mana steal synergy)
		if cfg.UseCobraStrike {
			if needsMoreCharges(cs.hasCobraState, cs.cobraFound, cs.cobraCharges, maxChargesCobraStrike) {
				step.SecondaryAttack(skill.CobraStrike, id, 1)
				continue
			}
		}

		// Check if monster died during charge building
		if !s.isMonsterAlive(id) {
			return nil
		}

		// 3. Phoenix Strike - 2 charges (meteor proc - core damage skill)
		// NOTE: Always used regardless of config - this is the main damage dealer
		if needsMoreCharges(cs.hasPhoenixState, cs.phoenixFound, cs.phoenixCharges, maxChargesPhoenixStrike) {
			step.SecondaryAttack(skill.PhoenixStrike, id, 1)
			continue
		}

		// Check if monster died during charge building
		if !s.isMonsterAlive(id) {
			return nil
		}

		// 4. Claws of Thunder - 3 charges (lightning damage)
		if cfg.UseClawsOfThunder {
			if needsMoreCharges(cs.hasClawsState, cs.clawsFound, cs.clawsCharges, maxChargesClawsOfThunder) {
				step.SecondaryAttack(skill.ClawsOfThunder, id, 1)
				continue
			}
		}

		// Check if monster died during charge building
		if !s.isMonsterAlive(id) {
			return nil
		}

		// 5. Blades of Ice - 3 charges (cold damage + freeze)
		if cfg.UseBladesOfIce {
			if needsMoreCharges(cs.hasBladesState, cs.bladesFound, cs.bladesCharges, maxChargesBladesOfIce) {
				step.SecondaryAttack(skill.BladesOfIce, id, 1)
				continue
			}
		}

		// Check if monster died during charge building
		if !s.isMonsterAlive(id) {
			return nil
		}

		// 6. Fists of Fire - 3 charges (fire damage)
		if cfg.UseFistsOfFire {
			if needsMoreCharges(cs.hasFistsState, cs.fistsFound, cs.fistsCharges, maxChargesFistsOfFire) {
				step.SecondaryAttack(skill.FistsOfFire, id, 1)
				continue
			}
		}

		// Final alive check before finishing attack
		if !s.isMonsterAlive(id) {
			return nil
		}

		// ============================================================
		// RELEASE CHARGES - All charges are ready, execute finisher!
		// Dragon Talon on left click releases all stored charges via Mosaic runeword
		// ============================================================
		opts := step.Distance(mosaicAttackMinRange, mosaicAttackMaxRange)
		step.PrimaryAttack(id, 1, false, opts)
	}
}

// isMonsterAlive checks if the monster with given ID exists and has health > 0
// Consolidated check to avoid code duplication
func (s MosaicSin) isMonsterAlive(id data.UnitID) bool {
	monster, found := s.Data.Monsters.FindByID(id)
	return found && monster.Stats[stat.Life] > 0
}

func (s MosaicSin) BuffSkills() []skill.ID {
	skillsList := make([]skill.ID, 0, 2)

	// Fade takes priority over Burst of Speed
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Fade); found {
		skillsList = append(skillsList, skill.Fade)
	} else if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.BurstOfSpeed); found {
		skillsList = append(skillsList, skill.BurstOfSpeed)
	}

	return skillsList
}

func (s MosaicSin) PreCTABuffSkills() []skill.ID {
	// Shadow Master is preferred over Shadow Warrior
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.ShadowMaster); found {
		return []skill.ID{skill.ShadowMaster}
	}
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.ShadowWarrior); found {
		return []skill.ID{skill.ShadowWarrior}
	}
	return []skill.ID{}
}

func (s MosaicSin) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}
		return m.UnitID, true
	}, nil)
}

// findClosestCouncilMember finds the nearest council member without sorting the entire slice
// This is O(n) instead of O(n log n) for sorting
func (s MosaicSin) findClosestCouncilMember(d game.Data) (data.UnitID, bool) {
	var closestID data.UnitID
	closestDist := -1

	for _, m := range d.Monsters {
		if m.Name != npc.CouncilMember && m.Name != npc.CouncilMember2 && m.Name != npc.CouncilMember3 {
			continue
		}

		// Skip dead council members
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

func (s MosaicSin) KillCountess() error {
	return s.killMonster(npc.DarkStalker, data.MonsterTypeSuperUnique)
}

func (s MosaicSin) KillAndariel() error {
	return s.killMonster(npc.Andariel, data.MonsterTypeUnique)
}

func (s MosaicSin) KillSummoner() error {
	return s.killMonster(npc.Summoner, data.MonsterTypeUnique)
}

func (s MosaicSin) KillDuriel() error {
	return s.killMonster(npc.Duriel, data.MonsterTypeUnique)
}

func (s MosaicSin) KillCouncil() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		return s.findClosestCouncilMember(d)
	}, nil)
}

func (s MosaicSin) KillMephisto() error {
	return s.killMonster(npc.Mephisto, data.MonsterTypeUnique)
}

func (s MosaicSin) KillIzual() error {
	return s.killMonster(npc.Izual, data.MonsterTypeUnique)
}

func (s MosaicSin) KillDiablo() error {
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

func (s MosaicSin) KillPindle() error {
	return s.killMonster(npc.DefiledWarrior, data.MonsterTypeSuperUnique)
}

func (s MosaicSin) KillNihlathak() error {
	return s.killMonster(npc.Nihlathak, data.MonsterTypeSuperUnique)
}

func (s MosaicSin) KillBaal() error {
	return s.killMonster(npc.BaalCrab, data.MonsterTypeUnique)
}
