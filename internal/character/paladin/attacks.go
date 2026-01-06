package paladin

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/character/core"
)

//region FoH

func (p *PaladinBase) useFoh(monster data.Monster) bool {
	aura := p.applyAuraOverride(p.Options.FohAura)
	step.SelectSkill(skill.FistOfTheHeavens)
	opts := step.StationaryDistance(core.ScreenRange/2, core.ScreenRange)
	if aura == skill.Conviction {
		opts = step.StationaryDistance(convictionRange/2, convictionRange)
	}
	if err := step.PrimaryAttack(monster.UnitID, 1, true, opts, step.EnsureAura(aura)); err != nil {
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
	targetID := monster.UnitID
	usePacketCasting := p.CharacterCfg.PacketCasting.UseForEntitySkills && p.PacketSender != nil
	if !usePacketCasting && p.holyBoltLastTarget != targetID {
		p.holyBoltLastTarget = targetID
		p.holyBoltConsecutiveOffTargetHovers = 0
	}

	aura := p.applyAuraOverride(p.Options.HolyBoltAura)
	step.SelectSkill(skill.HolyBolt)
	if err := step.PrimaryAttack(targetID, 1, true, step.StationaryDistance(core.ScreenRange/2, core.ScreenRange), step.EnsureAura(aura)); err != nil {
		return false
	}
	if !p.WaitForCastComplete() {
		return false
	}

	// Refresh after the cast to read hover data; packet casting bypasses hover tracking entirely.
	if !usePacketCasting {
		p.Context.RefreshGameData()
		hoverData := p.Data.HoverData
		isHoveringTarget := hoverData.IsHovered && hoverData.UnitType == 1 && hoverData.UnitID == targetID
		if !isHoveringTarget {
			p.holyBoltConsecutiveOffTargetHovers++
			if p.holyBoltConsecutiveOffTargetHovers >= 3 {
				p.PathFinder.RandomMovement()
				p.Context.RefreshGameData()
			}
		} else {
			p.holyBoltConsecutiveOffTargetHovers = 0
		}
	}

	return true
}

func (p *PaladinBase) canBeDamagedByHolyBolt(monster data.Monster) bool {
	if monster.IsUndead() {
		return true
	}
	if monster.IsDemon() && !monster.IsImmune(stat.MagicImmune) {
		return true
	}

	return false
}

//endregion Holy Bolt

//region Blessed Hammer

func (p *PaladinBase) useHammer(monster data.Monster) bool {
	targetID := monster.UnitID
	if p.hammerLastTarget != targetID {
		p.hammerLastTarget = targetID
		p.hammerConsecutiveAttacks = 0
		p.hammerLastPosition = monster.Position
	}

	if p.hammerConsecutiveAttacks >= 2 {
		p.PathFinder.RandomMovement()
		p.Context.RefreshGameData()
	}

	aura := p.applyAuraOverride(p.Options.HammerAura)
	step.SelectSkill(skill.BlessedHammer)
	if err := step.PrimaryAttack(targetID, 1, true, step.StationaryDistance(2, 2), step.EnsureAura(aura)); err != nil {
		return false
	}

	p.hammerConsecutiveAttacks++

	return true
}

func (p *PaladinBase) useHammerPlayer() bool {
	// Refresh game data if we appear to be in the same position to confirm that is still true.
	if p.hammerLastPosition == p.Data.PlayerUnit.Position {
		p.Context.RefreshGameData()
	}

	if p.hammerLastPosition != p.Data.PlayerUnit.Position {
		p.hammerLastPosition = p.Data.PlayerUnit.Position
		p.hammerConsecutiveAttacks = 0
		p.hammerLastTarget = p.Data.PlayerUnit.ID
	}

	if p.hammerConsecutiveAttacks >= 2 {
		p.PathFinder.RandomMovement()
		p.Context.RefreshGameData()
	}

	aura := p.applyAuraOverride(p.Options.HammerAura)
	step.SelectSkill(aura)
	if !step.CastAtPosition(skill.BlessedHammer, true, p.Data.PlayerUnit.Position) {
		return false
	}

	p.hammerConsecutiveAttacks++

	return true
}

//endregion Blessed Hammer

//region Smite

func (p *PaladinBase) useSmite(monster data.Monster) bool {
	aura := p.applyAuraOverride(p.Options.SmiteAura)
	step.SelectSkill(skill.Smite)
	if err := step.PrimaryAttack(monster.UnitID, 1, false, step.Distance(1, core.MeleeRange), step.EnsureAura(aura)); err != nil {
		return false
	}

	return true
}

//endregion Smite

//region Zeal

func (p *PaladinBase) useZeal(monster data.Monster) bool {
	aura := p.applyAuraOverride(p.Options.ZealAura)
	step.SelectSkill(skill.Zeal)
	if err := step.PrimaryAttack(monster.UnitID, 1, false, step.Distance(1, core.MeleeRange), step.EnsureAura(aura)); err != nil {
		return false
	}

	return true
}

//endregion Zeal
