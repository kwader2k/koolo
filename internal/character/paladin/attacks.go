package paladin

import (
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/character/core"
)

//region FoH

func (p *PaladinBase) useFoh(monster data.Monster) bool {
	step.SelectSkill(skill.FistOfTheHeavens)
	aura := p.applyAuraOverride(p.Options.FohAura)
	if err := step.PrimaryAttack(monster.UnitID, 1, true, step.StationaryDistance(5, core.ScreenRange), step.EnsureAura(aura)); err != nil {
		return false
	}
	if !p.WaitForCastComplete() {
		return false
	}

	return true
}

//endregion FoH

//region Holy Bolt

func (p *PaladinBase) useHolyBolt(monster data.Monster) bool {
	step.SelectSkill(skill.HolyBolt)
	aura := p.applyAuraOverride(p.Options.HolyBoltAura)
	if err := step.PrimaryAttack(monster.UnitID, 1, true, step.StationaryDistance(5, core.ScreenRange), step.EnsureAura(aura)); err != nil {
		return false
	}
	if !p.WaitForCastComplete() {
		return false
	}

	return true
}

//endregion Holy Bolt

//region Blessed Hammer

func (p *PaladinBase) useHammer(monster data.Monster) bool {
	targetID := monster.UnitID
	if p.hammerLastTarget != targetID {
		p.hammerConsecutiveAttacks = 0
		p.hammerLastTarget = targetID
	}

	step.SelectSkill(skill.BlessedHammer)
	aura := p.applyAuraOverride(p.Options.HammerAura)
	if err := step.PrimaryAttack(targetID, 1, true, step.StationaryDistance(2, 2), step.EnsureAura(aura)); err != nil {
		return false
	}

	p.hammerConsecutiveAttacks++
	if p.hammerConsecutiveAttacks >= 5 {
		p.PathFinder.RandomMovement()
		time.Sleep(200 * time.Millisecond)
		p.hammerConsecutiveAttacks = 0
	}

	return true
}

//endregion Blessed Hammer

//region Smite

func (p *PaladinBase) useSmite(monster data.Monster) bool {
	step.SelectSkill(skill.Smite)
	aura := p.applyAuraOverride(p.Options.SmiteAura)
	if err := step.PrimaryAttack(monster.UnitID, 1, false, step.Distance(1, 3), step.EnsureAura(aura)); err != nil {
		return false
	}

	return true
}

//endregion Smite

//region Zeal

func (p *PaladinBase) useZeal(monster data.Monster) bool {
	step.SelectSkill(skill.Zeal)
	aura := p.applyAuraOverride(p.Options.ZealAura)
	if err := step.PrimaryAttack(monster.UnitID, 1, false, step.Distance(1, 3), step.EnsureAura(aura)); err != nil {
		return false
	}

	return true
}

//endregion Zeal
