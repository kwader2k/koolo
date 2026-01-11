package paladin

import (
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/character/core"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

var _ context.LevelingCharacter = (*PaladinLeveling)(nil)

//region Types

type PaladinLeveling struct {
	PaladinBase
}

//endregion Types

//region Construction

func NewLeveling(base core.CharacterBase) *PaladinLeveling {
	opts := paladinOptionsForBuild(base.CharacterCfg, "paladin_leveling", true)
	return &PaladinLeveling{
		PaladinBase: PaladinBase{
			CharacterBase: base,
			Options:       opts,
		},
	}
}

//endregion Construction

//region Interface

func (p *PaladinLeveling) CheckKeyBindings() []skill.ID {
	return []skill.ID{} // They are auto-binded
}

func (p *PaladinLeveling) ShouldIgnoreMonster(monster data.Monster) bool {
	// Post-respec leveling follows the same immunity deferral as the default build.
	if p.isPostRespec() {
		return p.shouldIgnoreMonsterDefault(monster)
	}

	// Dead monsters should be ignored.
	if monster.Stats[stat.Life] <= 0 {
		return true
	}

	return false
}

//endregion Interface

//region Combat

func (p *PaladinLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	ctx := context.Get()
	defer p.resetTeleportOverride()
	var lastTargetID data.UnitID
	var targetStart time.Time

	for {
		ctx.PauseIfNotPriority()

		monsterId, monsterIdFound := monsterSelector(*p.Data)
		if !monsterIdFound {
			return nil
		}
		if monsterId != lastTargetID {
			lastTargetID = monsterId
			targetStart = time.Now()
		}
		monster, monsterFound := p.Data.Monsters.FindByID(monsterId)
		if !monsterFound {
			continue
		}
		if !p.PreBattleChecks(monsterId, skipOnImmunities) {
			return nil
		}
		if time.Since(targetStart) > core.TargetTimeout {
			return nil
		}

		if p.isPostRespec() {
			// Post-respec: use the default hammer rotation.
			if !p.executeDefaultRotation(monster) {
				continue
			}
		} else {
			// Pre-respec: Holy Fire (or Might) + Zeal until the respec condition is met.
			if !p.executeHolyFireRotation(monster) {
				continue
			}
		}
	}
}

//endregion Combat

//region Leveling Flow

func (p *PaladinLeveling) SkillsToBind() (skill.ID, []skill.ID) {
	p.updateOptionsForLeveling()

	playerLevel, _ := p.Data.PlayerUnit.FindStat(stat.Level, 0)
	mainSkill := skill.AttackSkill
	skillBindings := []skill.ID{}

	if p.isPostRespec() {
		if p.CanUseSkill(skill.BlessedHammer) {
			mainSkill = skill.BlessedHammer
			skillBindings = p.AppendUniqueSkill(skillBindings, skill.BlessedHammer)
			skillBindings = p.AppendUniqueSkill(skillBindings, p.Options.HammerAura)
		}
		if p.CanUseSkill(skill.Smite) {
			skillBindings = p.AppendUniqueSkill(skillBindings, skill.Smite)
			skillBindings = p.AppendUniqueSkill(skillBindings, p.Options.SmiteAura)
		}
	} else {
		if playerLevel.Value < 12 {
			mainSkill = skill.Sacrifice
			skillBindings = p.AppendUniqueSkill(skillBindings, skill.Sacrifice)
		} else {
			mainSkill = skill.Zeal
			skillBindings = p.AppendUniqueSkill(skillBindings, skill.Zeal)
		}
		skillBindings = p.AppendUniqueSkill(skillBindings, p.Options.ZealAura)
	}

	if p.CanUseNonClassSkill(skill.Teleport) && p.CharacterCfg.Character.UseTeleport {
		skillBindings = p.AppendUniqueSkill(skillBindings, skill.Teleport)
	} else if p.shouldUseChargeMovement() {
		skillBindings = p.AppendUniqueSkill(skillBindings, skill.Charge)
	}
	if p.CanUseSkill(skill.Vigor) {
		skillBindings = p.AppendUniqueSkill(skillBindings, skill.Vigor)
	}

	if p.CanUseSkill(skill.HolyShield) {
		skillBindings = p.AppendUniqueSkill(skillBindings, skill.HolyShield)
	}

	// TODO: Need to handle weapon swap binding for those
	// if p.CanUseNonClassSkill(skill.BattleCommand) {
	// 	skillBindings = p.AppendUniqueSkill(skillBindings, skill.BattleCommand)
	// }
	// if p.CanUseNonClassSkill(skill.BattleOrders) {
	// 	skillBindings = p.AppendUniqueSkill(skillBindings, skill.BattleOrders)
	// }

	if _, found := p.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory); found {
		skillBindings = p.AppendUniqueSkill(skillBindings, skill.TomeOfTownPortal)
	}

	if p.CanUseSkill(skill.Cleansing) {
		skillBindings = p.AppendUniqueSkill(skillBindings, skill.Cleansing)
	}
	if (p.Options.UseRedemptionOnRaisers || p.Options.UseRedemptionToReplenish) && p.CanUseSkill(skill.Redemption) {
		skillBindings = p.AppendUniqueSkill(skillBindings, skill.Redemption)
	}

	p.Logger.Info("Skills bound", "mainSkill", mainSkill, "skillBindings", skillBindings)
	return mainSkill, skillBindings
}

func (p *PaladinLeveling) ShouldResetSkills() bool {
	playerLevel, _ := p.Data.PlayerUnit.FindStat(stat.Level, 0)

	// To Hammer
	if playerLevel.Value >= 24 && p.BaseSkillLevel(skill.HolyFire) >= 2 {
		p.Logger.Info("Resetting skills to Hammer: Level 24 and Holy Fire level >= 2")
		return true
	}

	return false
}

func (p *PaladinLeveling) StatPoints() []context.StatAllocation {
	return []context.StatAllocation{
		{Stat: stat.Vitality, Points: 30},
		{Stat: stat.Strength, Points: 30},
		{Stat: stat.Vitality, Points: 35},
		{Stat: stat.Strength, Points: 35},
		{Stat: stat.Vitality, Points: 40},
		{Stat: stat.Strength, Points: 40},
		{Stat: stat.Vitality, Points: 50},
		{Stat: stat.Strength, Points: 80},
		{Stat: stat.Vitality, Points: 100},
		{Stat: stat.Strength, Points: 95},
		{Stat: stat.Vitality, Points: 205},
		{Stat: stat.Dexterity, Points: 100},
		{Stat: stat.Vitality, Points: 999},
	}
}

func (p *PaladinLeveling) SkillPoints() []skill.ID {
	playerLevel, _ := p.Data.PlayerUnit.FindStat(stat.Level, 0)

	if playerLevel.Value < 24 {
		return []skill.ID{
			skill.Might, skill.Sacrifice, skill.ResistFire, skill.ResistFire, skill.ResistFire,
			skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire,
			skill.HolyFire, skill.Zeal, skill.HolyFire, skill.HolyFire, skill.HolyFire,
			skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire,
			skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire,
			skill.HolyFire, skill.ResistFire, skill.ResistFire, skill.ResistFire, skill.ResistFire,
		}
	}

	return []skill.ID{
		skill.Prayer, skill.Defiance, skill.Cleansing, skill.Vigor,
		skill.Might, skill.BlessedAim, skill.Concentration,
		skill.Smite, skill.HolyBolt, skill.Charge, skill.BlessedHammer, skill.HolyShield,
		skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
		skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
		skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
		skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
		skill.BlessedHammer, skill.Concentration,
		skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
		skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
		skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration,
		skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration,
		skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
		skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
		skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
		skill.Vigor, skill.Redemption,
		skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
		skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
		skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
		skill.BlessedAim,
		skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
		skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
		skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
		skill.HolyShield,
	}
}

//endregion Leveling Flow

//region Boss Helpers

func (p *PaladinLeveling) KillAncients() error {
	originalBackToTownCfg := p.CharacterCfg.BackToTown
	p.CharacterCfg.BackToTown.NoHpPotions = false
	p.CharacterCfg.BackToTown.NoMpPotions = false
	p.CharacterCfg.BackToTown.EquipmentBroken = false
	p.CharacterCfg.BackToTown.MercDied = false

	for _, m := range p.Data.Monsters.Enemies(data.MonsterEliteFilter()) {
		foundMonster, found := p.Data.Monsters.FindOne(m.Name, data.MonsterTypeSuperUnique)
		if !found {
			continue
		}
		step.MoveTo(data.Position{X: 10062, Y: 12639}, step.WithIgnoreMonsters())
		p.KillMonsterByName(foundMonster.Name, data.MonsterTypeSuperUnique, nil)
	}

	p.CharacterCfg.BackToTown = originalBackToTownCfg
	p.Logger.Info("Restored original back-to-town checks after Ancients fight.")
	return nil
}

//endregion Boss Helpers

//region Runewords & Config

func (p *PaladinLeveling) GetAdditionalRunewords() []string {
	additionalRunewords := action.GetCastersCommonRunewords()
	additionalRunewords = append(additionalRunewords, "Steel")
	return additionalRunewords
}

func (p *PaladinLeveling) InitialCharacterConfigSetup() {
}

func (p *PaladinLeveling) AdjustCharacterConfig() {
}

//endregion Runewords & Config

//region Helpers

func (p *PaladinLeveling) isPostRespec() bool {
	return p.CanUseSkill(skill.BlessedHammer)
}

func (p *PaladinLeveling) updateOptionsForLeveling() {
	opts := paladinDefaultOptions()

	if p.isPostRespec() {
		opts.SmiteAura = skill.Concentration
		opts.UseRedemptionOnRaisers = true
		opts.UseRedemptionToReplenish = true
	} else {
		if p.CanUseSkill(skill.HolyFire) {
			opts.MovementAura = skill.HolyFire
			opts.ZealAura = skill.HolyFire
		} else if p.CanUseSkill(skill.Might) {
			opts.MovementAura = skill.Might
			opts.ZealAura = skill.Might
		} else {
			opts.MovementAura = skill.ID(0)
			opts.ZealAura = skill.ID(0)
		}
	}
	opts.UseChargeMovement = true

	p.UpdateOptions(opts)

	// Gear is important for Paladin, we should always go back to town to repair it
	p.CharacterCfg.BackToTown.EquipmentBroken = true
	// Force revive of the Merc
	p.CharacterCfg.BackToTown.MercDied = true
	// As Holy Fire, we do not want to go back to town for Mana Potions, it's a waste of time
	if p.isPostRespec() {
		p.CharacterCfg.BackToTown.NoMpPotions = true
	} else {
		p.CharacterCfg.BackToTown.NoMpPotions = false
	}
}

//endregion Helpers

//region Rotations

func (p *PaladinLeveling) executeHolyFireRotation(monster data.Monster) bool {
	skillToUse := skill.AttackSkill
	if p.CanUseSkill(skill.Sacrifice) {
		skillToUse = skill.Sacrifice
	}
	if p.CanUseSkill(skill.Zeal) {
		skillToUse = skill.Zeal
	}
	aura := p.applyAuraOverride(p.Options.ZealAura)
	p.Logger.Debug("Leveling attack selected", "skill", skill.SkillNames[skillToUse], "aura", skill.SkillNames[aura])

	step.SelectSkill(skillToUse)
	if err := step.PrimaryAttack(monster.UnitID, 1, false, step.Distance(1, 3), step.EnsureAura(aura)); err != nil {
		return false
	}

	return true
}

//endregion Rotations
