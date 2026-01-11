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
	// If something is this close, don't "think" about moving first - swing immediately.
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
	// Ignore dead stuff.
	if m.Stats[stat.Life] <= 0 {
		return true
	}

	distance := p.PathFinder.DistanceFromMe(m.Position)
	// Let the general combat logic consider targets in a wider radius.
	return distance > dragondinEngageRange
}

func (p *PaladinDragon) CheckKeyBindings() []skill.ID {
	required := []skill.ID{}

	// It is expected that Zeal is learned since it's the default skill of this build
	required = p.AppendUniqueSkill(required, skill.Zeal)
	if p.CanUseSkill(p.Options.ZealAura) {
		required = p.AppendUniqueSkill(required, p.Options.ZealAura)
	}

	if p.shouldUseChargeMovement() {
		required = p.AppendUniqueSkill(required, skill.Charge)
	}
	if p.CanUseSkill(skill.Vigor) {
		// Vigor is the default Movement Aura in Town
		required = p.AppendUniqueSkill(required, skill.Vigor)
	}
	if p.CanUseSkill(p.Options.MovementAura) {
		required = p.AppendUniqueSkill(required, p.Options.MovementAura)
	}

	// It is expected that HolyShield is learned when using this build
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
			return nil
		}
		if time.Since(targetStart) > core.TargetTimeout {
			return nil
		}

		distance := p.PathFinder.DistanceFromMe(monster.Position)
		if distance > dragondinMeleeRange {
			// If something is already close enough to be dangerous, hit it NOW.
			// This avoids the "stand still for ~0.5s then decide to attack" behavior.
			if p.tryKillNearby(skipOnImmunities, dragondinImmediateThreatRange) {
				continue
			}

			// Fight blockers: we handle threats ourselves, so disable MoveTo's "monsters in path" early-exit.
			if err := step.MoveTo(monster.Position, step.WithClearPathOverride(0)); err != nil {
				if !errors.Is(err, step.ErrMonstersInPath) {
					p.Logger.Debug("Unable to move into melee range", slog.String("error", err.Error()))
				}

				// If movement fails (monsters in path / stuck on corners), clear the closest nearby and retry.
				// Slightly wider than immediate-threat range, but still keeps the reaction snappy.
				if p.tryKillNearby(skipOnImmunities, dragondinMeleeRange+3) {
					outOfRangeAttempts = 0
				} else {
					outOfRangeAttempts++
				}
			} else {
				// Made progress towards the target.
				outOfRangeAttempts = 0
			}
			if outOfRangeAttempts >= dragondinMaxOutOfRangeAttempts {
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

// tryKillNearby attempts to clear the closest enemy in our immediate vicinity.
// Used both as an "immediate threat" handler and as a "path blocker" handler.
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

	// If we have immunity rules, respect them for blockers too.
	if !p.PreBattleChecks(closest.UnitID, skipOnImmunities) {
		return false
	}

	return p.useZeal(closest)
}

//endregion Helpers
