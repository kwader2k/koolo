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
	total                 int
	lightningImmune       int
	magicImmune           int
	meleeRange            int
	meleeRangeMagicImmune int
	holyBoltSensible      int
}

//endregion Types

//region Constants

const paladinRelevantSkillMinLevel = 2 // Require at least 2 hard points before treating a skill as a relevant damage source (except Smite).

const (
	paladinMainSmite paladinMainSkill = iota
	paladinMainHammer
	paladinMainFoh
)

//endregion Constants

//region Filtering

// retrieveDefaultMainSkill follows the FoH > Hammer > Smite priority.
func (p *PaladinBase) retrieveDefaultMainSkill() paladinMainSkill {
	// FoH primary.
	if p.BaseSkillLevel(skill.FistOfTheHeavens) >= 1 || p.BaseSkillLevel(skill.HolyBolt) >= p.BaseSkillLevel(skill.BlessedHammer) {
		return paladinMainFoh
	}
	// Hammer primary.
	if p.BaseSkillLevel(skill.BlessedHammer) >= p.BaseSkillLevel(skill.Smite) {
		return paladinMainHammer
	}

	// Default to Smite.
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
	screenRange := p.ScreenRange()
	switch p.retrieveDefaultMainSkill() {
	case paladinMainFoh:
		packPlayer := p.scanSurroundingEnemiesDefault(p.Data.PlayerUnit.Position, screenRange)
		// If at least one undead or demon is present, ignore other monsters to prioritize FoH + Holy Bolt weaving on valid targets.
		if packPlayer.holyBoltSensible > 0 {
			return !monster.IsUndeadOrDemon()
		}
		// If only beasts remain, ignore lightning-immune monsters.
		if packPlayer.total > packPlayer.lightningImmune {
			return monster.IsImmune(stat.LightImmune)
		}

		// Leave them alone until they are the only targets left.
		return false
	case paladinMainHammer:
		packPlayer := p.scanSurroundingEnemiesDefault(p.Data.PlayerUnit.Position, screenRange)
		// When surrounded in melee range and Teleport is unavailable, do not ignore any targets (Blessed Hammer behaves like a melee skill).
		if packPlayer.meleeRange >= 4 && !p.Data.CanTeleport() {
			return false
		}
		// Ignore magic-immune monsters when they make up 55% or less of the pack to prioritize non-magic-immune targets.
		if packPlayer.total > packPlayer.magicImmune && packPlayer.total*55 > packPlayer.magicImmune*100 {
			return monster.IsImmune(stat.MagicImmune)
		}

		// Leave them alone until they are the only targets left.
		return false
	case paladinMainSmite:
		// Smite follows default priorities.
		return false
	default:
		return false
	}
}

//endregion Filtering

//region Rotation

// scanSurroundingEnemiesDefault scans nearby enemies to drive rotation decisions.
func (p *PaladinBase) scanSurroundingEnemiesDefault(center data.Position, maxRange int) paladinPackInfo {
	info := paladinPackInfo{}
	for _, monster := range p.Data.Monsters.Enemies() {
		if monster.Stats[stat.Life] <= 0 {
			continue
		}

		distanceToMonster := pather.DistanceFromPoint(center, monster.Position)
		if distanceToMonster > maxRange {
			continue
		}

		// Global counts.
		info.total++
		monsterIsLightningImmune := monster.IsImmune(stat.LightImmune)
		monsterIsMagicImmune := monster.IsImmune(stat.MagicImmune)
		monsterIsHolyBoltSensible := p.canBeDamagedByHolyBolt(monster)
		if monsterIsLightningImmune {
			info.lightningImmune++
		}
		if monsterIsMagicImmune {
			info.magicImmune++
		}
		if monsterIsHolyBoltSensible {
			info.holyBoltSensible++
		}

		// Melee counts.
		if distanceToMonster <= core.MeleeRange {
			info.meleeRange++
			if monsterIsMagicImmune {
				info.meleeRangeMagicImmune++
			}
		}
	}

	return info
}

// executeDefaultRotation applies the default paladin combat rotation rules.
func (p *PaladinBase) executeDefaultRotation(monster data.Monster) bool {
	// Always use Smite on Ubers, regardless of build.
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
	screenRange := p.ScreenRange()
	packTarget := p.scanSurroundingEnemiesDefault(monster.Position, screenRange)
	packPlayer := p.scanSurroundingEnemiesDefault(p.Data.PlayerUnit.Position, screenRange)
	distanceToMonster := pather.DistanceFromPoint(monster.Position, p.Data.PlayerUnit.Position)
	hammerEnabled := p.BaseSkillLevel(skill.BlessedHammer) >= paladinRelevantSkillMinLevel // Disable Hammer if it is not relevant

	// Prioritize single-target over FoH unless Conviction is active or Holy Bolt can splash.
	if (p.Options.FohAura != skill.Conviction || !p.canUseAura(skill.Conviction)) && packTarget.holyBoltSensible <= 2 && (!hammerEnabled || packTarget.total-packTarget.magicImmune <= 3) {
		// Holy Bolt hits harder than FoH without Conviction.
		if p.CanUseSkill(skill.HolyBolt) && p.canBeDamagedByHolyBolt(monster) {
			return p.useHolyBolt(monster)
		}
		// Blessed Hammer hits harder than FoH without Conviction, but only if we're already in melee range or can Teleport to the target.
		if hammerEnabled && !monster.IsImmune(stat.MagicImmune) && (distanceToMonster <= core.MeleeRange || p.Data.CanTeleport()) {
			return p.useHammer(monster)
		}
	}

	// Use Fist of the Heavens for Holy Bolt splash or lightning damage if the target is not lightning-immune.
	if p.CanUseSkill(skill.FistOfTheHeavens) && !p.Data.PlayerUnit.States.HasState(state.Cooldown) && (!monster.IsImmune(stat.LightImmune) || packTarget.holyBoltSensible >= 3) {
		return p.useFoh(monster)
	}

	// Blessed Hammer pack logic.
	if hammerEnabled {
		// If we cannot Teleport, use Blessed Hammer around the player when 2 or more non-magic-immune monsters are in melee range.
		if !p.Data.CanTeleport() && packPlayer.meleeRange-packPlayer.meleeRangeMagicImmune >= 2 {
			return p.useHammerPlayer()
		}

		// Use Blessed Hammer around the player when 4 or more non-magic-immune monsters are in screen range and we're already in melee range of the target.
		if packPlayer.total-packPlayer.magicImmune >= 4 && distanceToMonster <= core.MeleeRange {
			return p.useHammerPlayer()
		}
	}

	// Use Holy Bolt against sensible.
	if p.CanUseSkill(skill.HolyBolt) && p.canBeDamagedByHolyBolt(monster) {
		return p.useHolyBolt(monster)
	}

	// Use Blessed Hammer against non-magic-immune targets.
	if hammerEnabled && !monster.IsImmune(stat.MagicImmune) {
		return p.useHammer(monster)
	}

	// Use Smite otherwise.
	return p.useSmite(monster)
}

// executeHammerRotation is the rotation for builds with Blessed Hammer investment.
func (p *PaladinBase) executeHammerRotation(monster data.Monster) bool {
	// If we cannot Teleport, use Blessed Hammer around the player when 2 or more non-magic-immune monsters are in melee range.
	if !p.Data.CanTeleport() {
		packPlayer := p.scanSurroundingEnemiesDefault(p.Data.PlayerUnit.Position, p.ScreenRange())
		if packPlayer.meleeRange-packPlayer.meleeRangeMagicImmune >= 2 {
			return p.useHammerPlayer()
		}
	}

	// Use Blessed Hammer unless the target is immune to magic.
	if !monster.IsImmune(stat.MagicImmune) {
		return p.useHammer(monster)
	}

	// Use Smite otherwise.
	return p.useSmite(monster)
}

//endregion Rotation
