package paladin

import (
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
)

//region Helpers

func (p *PaladinBase) canUseAura(aura skill.ID) bool {
	// Invalid aura skill ID.
	if aura <= 0 {
		return false
	}
	// Auras must be learned.
	if !p.CanUseSkill(aura) {
		return false
	}
	// Auras must be bound to a keybind.
	_, bound := p.Data.KeyBindings.KeyBindingForSkill(aura)
	return bound
}

func (p *PaladinBase) applyAuraOverride(aura skill.ID) skill.ID {
	auraToUse := skill.ID(0) // Returning 0 leaves the aura unchanged.

	if p.shouldUseUberMephistoAura() {
		auraToUse = p.Options.UberMephAura
	} else if p.shouldUseRedemptionAuraRaiser() || p.shouldUseRedemptionAuraReplenish() {
		auraToUse = skill.Redemption
	} else if p.canUseAura(aura) {
		auraToUse = aura
	}

	p.updateTeleportOverride(auraToUse == skill.Redemption)

	return auraToUse
}

//endregion Helpers

//region Movement

func (p *PaladinBase) MovementSetup() (skill.ID, skill.ID) {
	movementSkill := skill.ID(0)
	if p.shouldUseChargeMovement() {
		movementSkill = skill.Charge
	}

	// In town, prefer Vigor regardless of MovementAura.
	if p.Data.PlayerUnit.Area.IsTown() && p.canUseAura(skill.Vigor) {
		return movementSkill, p.applyAuraOverride(skill.Vigor)
	}

	return movementSkill, p.applyAuraOverride(p.Options.MovementAura)
}

func (p *PaladinBase) shouldUseChargeMovement() bool {
	if !p.Options.UseChargeMovement {
		return false
	}
	if !p.CanUseSkill(skill.Charge) {
		return false
	}
	switch p.Data.PlayerUnit.Area {
	case area.FrigidHighlands, area.ArreatPlateau, area.FrozenTundra:
		return false
	}

	return !p.Data.CanTeleport()
}

//endregion Movement

//region Uber Mephisto

func (p *PaladinBase) shouldUseUberMephistoAura() bool {
	if !p.canUseAura(p.Options.UberMephAura) {
		return false
	}

	return p.HasUberMephistoNearby(p.ScreenRange())
}

//endregion Uber Mephisto

//region Redemption

func (p *PaladinBase) shouldUseRedemptionAuraReplenish() bool {
	if !p.Options.UseRedemptionToReplenish {
		return false
	}
	if !p.canUseAura(skill.Redemption) {
		return false
	}
	if p.Data.PlayerUnit.HPPercent() > p.Options.RedemptionHpThreshold &&
		p.Data.PlayerUnit.MPPercent() > p.Options.RedemptionManaThreshold {
		return false
	}

	return p.HasRedeemableCorpseNearby(redemptionReplenishRange)
}

func (p *PaladinBase) shouldUseRedemptionAuraRaiser() bool {
	if !p.Options.UseRedemptionOnRaisers {
		return false
	}
	if !p.canUseAura(skill.Redemption) {
		return false
	}
	raisersRange := p.redemptionRaisersRange()
	if !p.HasRaiserNearby(raisersRange) {
		return false
	}

	return p.HasRedeemableCorpseNearby(raisersRange)
}

//endregion Redemption
