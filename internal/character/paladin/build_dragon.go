package paladin

import (
	"errors"
	"log/slog"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/character/core"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

//region Constants

const (
	dragondinMeleeRange            = 5
	dragondinEngageRange           = 30
	dragondinMaxOutOfRangeAttempts = 20
	// If something is this close, don't pause to move first; swing immediately.
	dragondinImmediateThreatRange = dragondinMeleeRange + 1
)

//endregion Constants

//region Types

type PaladinDragon struct {
	PaladinBase
}

//endregion Types

//region Construction

func NewDragon(base core.CharacterBase) *PaladinDragon {
	opts := paladinOptionsForBuild(base.CharacterCfg, "paladin_dragon", false)
	return &PaladinDragon{
		PaladinBase: PaladinBase{
			CharacterBase: base,
			Options:       opts,
		},
	}
}

//endregion Construction

//region Interface

func (p *PaladinDragon) ShouldIgnoreMonster(m data.Monster) bool {
	// Ignore dead monsters.
	if m.Stats[stat.Life] <= 0 {
		return true
	}

	distance := p.PathFinder.DistanceFromMe(m.Position)
	// Let the general combat logic consider targets in a wider radius.
	return distance > dragondinEngageRange
}

func (p *PaladinDragon) CheckKeyBindings() []skill.ID {
	required := []skill.ID{}

	// Zeal is expected to be learned; it's this build's default skill.
	required = p.AppendUniqueSkill(required, skill.Zeal)
	if p.CanUseSkill(p.Options.ZealAura) {
		required = p.AppendUniqueSkill(required, p.Options.ZealAura)
	}

	if p.shouldUseChargeMovement() {
		required = p.AppendUniqueSkill(required, skill.Charge)
	}
	if p.CanUseSkill(skill.Vigor) {
		// Vigor is the default movement aura in town.
		required = p.AppendUniqueSkill(required, skill.Vigor)
	}
	if p.CanUseSkill(p.Options.MovementAura) {
		required = p.AppendUniqueSkill(required, p.Options.MovementAura)
	}

	// Holy Shield is expected to be learned for this build.
	required = p.AppendUniqueSkill(required, skill.HolyShield)

	if p.CanUseNonClassSkill(skill.BattleCommand) {
		required = p.AppendUniqueSkill(required, skill.BattleCommand)
	}
	if p.CanUseNonClassSkill(skill.BattleOrders) {
		required = p.AppendUniqueSkill(required, skill.BattleOrders)
	}

	required = p.AppendUniqueSkill(required, skill.TomeOfTownPortal)

	if (p.Options.UseRedemptionOnRaisers || p.Options.UseRedemptionToReplenish) && p.CanUseSkill(skill.Redemption) {
		required = p.AppendUniqueSkill(required, skill.Redemption)
	}
	if p.CanUseSkill(skill.Cleansing) {
		required = p.AppendUniqueSkill(required, skill.Cleansing)
	}

	return p.MissingKeyBindings(required)
}

//endregion Interface

//region Combat

func (p *PaladinDragon) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	outOfRangeAttempts := 0
	var lastTargetID data.UnitID
	var targetStart time.Time
	ctx := context.Get()
	defer p.resetTeleportOverride()

	for {
		ctx.PauseIfNotPriority()

		id, found := monsterSelector(*p.Data)
		if !found {
			return nil
		}
		if id != lastTargetID {
			lastTargetID = id
			targetStart = time.Now()
		}
		monster, found := p.Data.Monsters.FindByID(id)
		if !found || monster.Stats[stat.Life] <= 0 {
			continue
		}
		if p.ShouldIgnoreMonster(monster) {
			continue
		}

		if !p.PreBattleChecks(id, skipOnImmunities) {
			p.Logger.Warn("KillMonsterSequence ended", "reason", "pre_battle_checks_failed", "targetID", id, "targetName", monster.Name)
			return nil
		}
		if elapsed := time.Since(targetStart); elapsed > core.TargetTimeout {
			p.Logger.Warn("KillMonsterSequence ended", "reason", "target_timeout", "targetID", id, "targetName", monster.Name, "elapsed", elapsed)
			return nil
		}

		distance := p.PathFinder.DistanceFromMe(monster.Position)
		if distance > dragondinMeleeRange {
			// If something is already close enough to be dangerous, hit it now.
			// This avoids the "stand still for ~0.5s then decide to attack" pause.
			if p.tryKillNearby(skipOnImmunities, dragondinImmediateThreatRange) {
				continue
			}

			// Path blockers: we handle threats ourselves, so disable MoveTo's "monsters in path" early exit.
			if err := step.MoveTo(monster.Position, step.WithClearPathOverride(0)); err != nil {
				if !errors.Is(err, step.ErrMonstersInPath) {
					p.Logger.Debug("Unable to move into melee range", slog.String("error", err.Error()))
				}

				// If movement fails (monsters in path or stuck on corners), clear the closest target and retry.
				// Slightly wider than immediate-threat range, but keeps the reaction snappy.
				if p.tryKillNearby(skipOnImmunities, dragondinMeleeRange+3) {
					outOfRangeAttempts = 0
				} else {
					outOfRangeAttempts++
				}
			} else {
				// Made progress toward the target.
				outOfRangeAttempts = 0
			}
			if outOfRangeAttempts >= dragondinMaxOutOfRangeAttempts {
				p.Logger.Warn("KillMonsterSequence ended", "reason", "out_of_range_attempts", "targetID", id, "targetName", monster.Name, "attempts", outOfRangeAttempts)
				return nil
			}
			continue
		}

		_ = p.useZeal(monster)
		outOfRangeAttempts = 0
	}
}

//endregion Combat

//region Helpers

// tryKillNearby attempts to clear the closest enemy in the immediate vicinity.
// Used for both immediate-threat handling and path-blocker clearing.
func (p *PaladinDragon) tryKillNearby(skipOnImmunities []stat.Resist, maxDist int) bool {
	closestFound := false
	var closest data.Monster
	closestDist := 9999

	for _, m := range p.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}

		dist := p.PathFinder.DistanceFromMe(m.Position)
		if dist <= maxDist && dist < closestDist {
			closest = m
			closestDist = dist
			closestFound = true
		}
	}

	if !closestFound {
		return false
	}

	// If we have immunity rules, respect them for blockers as well.
	if !p.PreBattleChecks(closest.UnitID, skipOnImmunities) {
		return false
	}

	return p.useZeal(closest)
}

//endregion Helpers
