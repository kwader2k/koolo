package paladin

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/character/core"
	"github.com/hectorgimenez/koolo/internal/pather"
)

//region Types

type paladinMainSkill int

type paladinPackInfo struct {
	total                        int
	lightningImmune              int
	magicImmune                  int
	undeadOrDemon                int
	undeadOrDemonLightningImmune int
	undeadOrDemonMagicImmune     int
	beast                        int
	beastLightningImmune         int
	beastMagicImmune             int
	meleeRange                   int
}

//endregion Types

//region Constants

const paladinMainSkillMinLevel = 2 // We require at least 2 hard points before treating a skill as a relevant damage source (except Smite).

const (
	paladinMainSmite paladinMainSkill = iota
	paladinMainHammer
	paladinMainFoh
)

//endregion Constants

//region Filtering

// retrieveDefaultMainSkill follows the Hammer/FoH/Smite priority: FoH > Hammer > Smite.
func (p *PaladinBase) retrieveDefaultMainSkill() paladinMainSkill {
	smiteLevel := p.BaseSkillLevel(skill.Smite)
	hammerLevel := p.BaseSkillLevel(skill.BlessedHammer)
	fohLevel := p.BaseSkillLevel(skill.FistOfTheHeavens)

	// Main FoH
	if fohLevel > hammerLevel && fohLevel > smiteLevel {
		return paladinMainFoh
	}
	// Main Hammer
	if hammerLevel > fohLevel && hammerLevel > smiteLevel {
		return paladinMainHammer
	}
	// Main Smite
	if smiteLevel > fohLevel && smiteLevel > hammerLevel {
		return paladinMainSmite
	}
	// Hybrids
	if fohLevel >= paladinMainSkillMinLevel || (fohLevel > 0 && fohLevel == hammerLevel) {
		return paladinMainFoh
	}
	if hammerLevel >= paladinMainSkillMinLevel || (hammerLevel > 0 && hammerLevel == smiteLevel) {
		return paladinMainHammer
	}

	// Default
	return paladinMainSmite
}

func (p *PaladinBase) shouldIgnoreMonsterDefault(monster data.Monster) bool {
	// Ignore monsters that are already dead.
	if monster.Stats[stat.Life] <= 0 {
		return true
	}

	// Never ignore Ubers; they always take priority.
	if monster.IsUber() {
		return false
	}

	// Apply target-selection rules based on the current main skill.
	switch p.retrieveDefaultMainSkill() {
	case paladinMainFoh:
		pack := p.scanSurroundingEnemiesDefault()
		// If at least one Undead or Demon is present, ignore all other monsters to prioritize FoH + Holy Bolt weaving on valid targets.
		if pack.undeadOrDemon > 0 {
			return !monster.IsUndeadOrDemon()
		}
		// If only Beasts remain, ignore lightning-immune monsters.
		if pack.total > pack.lightningImmune {
			return monster.IsImmune(stat.LightImmune)
		}

		// Until they are the only targets left.
		return false
	case paladinMainHammer:
		pack := p.scanSurroundingEnemiesDefault()
		// When surrounded in melee range and teleport is unavailable, do not ignore any targets (Blessed Hammer behaves like a melee skill).
		if pack.meleeRange >= 4 && !p.Data.CanTeleport() {
			return false
		}
		// Ignore magic-immune monsters when they make up 55% or less of the pack, prioritizing non-magic-immune targets.
		if pack.total > pack.magicImmune && pack.total*55 > pack.magicImmune*100 {
			return monster.IsImmune(stat.MagicImmune)
		}

		// Until they are the only targets left.
		return false
	case paladinMainSmite:
		// Smite follow default priorities
		return false
	default:
		return false
	}
}

//endregion Filtering

//region Rotation

// scanSurroundingEnemiesDefault checks enemies nearby to drive rotation decisions.
func (p *PaladinBase) scanSurroundingEnemiesDefault() paladinPackInfo {
	info := paladinPackInfo{}
	for _, m := range p.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}
		if pather.DistanceFromPoint(m.Position, p.Data.PlayerUnit.Position) > core.ScreenRange {
			continue
		}

		info.total++
		if pather.DistanceFromPoint(m.Position, p.Data.PlayerUnit.Position) <= core.MeleeRange {
			info.meleeRange++
		}
		if m.IsUndeadOrDemon() {
			info.undeadOrDemon++
			if m.IsImmune(stat.LightImmune) {
				info.lightningImmune++
				info.undeadOrDemonLightningImmune++
			}
			if m.IsImmune(stat.MagicImmune) {
				info.magicImmune++
				info.undeadOrDemonMagicImmune++
			}
		} else {
			info.beast++
			if m.IsImmune(stat.LightImmune) {
				info.lightningImmune++
				info.beastLightningImmune++
			}
			if m.IsImmune(stat.MagicImmune) {
				info.magicImmune++
				info.beastMagicImmune++
			}
		}
	}

	return info
}

// executeDefaultRotation applies the default paladin combat rotation rules.
func (p *PaladinBase) executeDefaultRotation(monster data.Monster) bool {
	// No matter the build, we always use Smite on Ubers.
	if monster.IsUber() {
		return p.useSmite(monster)
	}

	switch p.retrieveDefaultMainSkill() {
	case paladinMainFoh:
		return p.executeFohRotation(monster)
	case paladinMainHammer:
		return p.executeHammerRotation(monster)
	case paladinMainSmite:
		return p.useSmite(monster)
	default:
		return p.useSmite(monster)
	}
}

// executeFohRotation is the rotation for builds with FoH investment.
func (p *PaladinBase) executeFohRotation(monster data.Monster) bool {
	// We always use FoH if not on cooldown unless the target is a beast immune to lightning.
	if !p.Data.PlayerUnit.States.HasState(state.Cooldown) && (monster.IsUndeadOrDemon() || !monster.IsImmune(stat.LightImmune)) {
		return p.useFoh(monster)
	}

	pack := p.scanSurroundingEnemiesDefault()

	// Use Blessed Hammer if there are 3 or more non-magic-immune monsters nearby and we invested at least 2 hard points.
	if p.BaseSkillLevel(skill.BlessedHammer) >= paladinMainSkillMinLevel && pack.total-pack.magicImmune >= 3 {
		return p.useHammer(monster)
	}

	// Use Holy Bolt on Undead/Demon.
	if monster.IsUndeadOrDemon() {
		if monster.IsPrimeEvil() && (p.Options.FohAura != skill.Conviction || !p.canUseAura(skill.Conviction)) {
			// When Conviction isn’t active, it’s generally better to cast Holy Bolt on Prime Evils.
			// However, Holy Bolt spam can be unreliable because it may auto-target the mercenary if he is too close to the monster.
			// To avoid this, we don’t rely exclusively on Holy Bolt and instead cast FoH once every 5 Holy Bolts (≈ every 2 seconds).
			for range 4 {
				p.useHolyBolt(monster)
				if monster.Stats[stat.Life] <= 0 {
					return true
				}
			}
		}
		return p.useHolyBolt(monster)
	}

	// Use Blessed Hammer on non-magic-immune if we invested at least 2 hard points.
	if p.BaseSkillLevel(skill.BlessedHammer) >= paladinMainSkillMinLevel && !monster.IsImmune(stat.MagicImmune) {
		return p.useHammer(monster)
	}

	// Use Smite otherwise.
	return p.useSmite(monster)
}

// executeHammerRotation is the rotation for builds with Blessed Hammer investment.
func (p *PaladinBase) executeHammerRotation(monster data.Monster) bool {
	// We always use Hammer unless the target is immune to magic.
	if !monster.IsImmune(stat.MagicImmune) {
		return p.useHammer(monster)
	}

	// Use Smite otherwise.
	return p.useSmite(monster)
}

//endregion Rotation
