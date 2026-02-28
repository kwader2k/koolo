package paladin

import (
	"strings"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/character/core"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

var _ context.LevelingCharacter = (*PaladinLeveling)(nil)

//region Types

type paladinLevelingBuild string

const (
	paladinLevelingBuildHammer paladinLevelingBuild = "hammer"
	paladinLevelingBuildFoh    paladinLevelingBuild = "foh"
)

type PaladinLeveling struct {
	PaladinBase
	runewordRuleInjector *DynamicRuleInjector
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
		runewordRuleInjector: NewDynamicRuleInjector(paladinLevelingDynamicRuleSource),
	}
}

//endregion Construction

//region Interface

func (p *PaladinLeveling) CheckKeyBindings() []skill.ID {
	return []skill.ID{} // Keybindings are auto-bound.
}

func (p *PaladinLeveling) ShouldIgnoreMonster(monster data.Monster) bool {
	// Post-respec follows the default build's immunity deferral.
	if p.isIntermediateSpec() || p.isEndgameSpec() {
		return p.shouldIgnoreMonsterDefault(monster)
	}

	// Ignore dead monsters.
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
		monster, monsterFound := p.Data.Monsters.FindByID(monsterId)
		if !monsterFound {
			continue
		}

		if retargeted, ok := p.RetargetIfBlockedBeyondRange(monster, p.MainSkillRange()); ok {
			monsterId = retargeted.UnitID
			monster = retargeted
		}
		if monsterId != lastTargetID {
			lastTargetID = monsterId
			targetStart = time.Now()
		}

		if !p.PreBattleChecks(monsterId, skipOnImmunities) {
			p.Logger.Warn("KillMonsterSequence ended", "reason", "pre_battle_checks_failed", "targetID", monsterId, "targetName", monster.Name)
			return nil
		}
		if elapsed := time.Since(targetStart); elapsed > core.TargetTimeout {
			p.Logger.Warn("KillMonsterSequence ended", "reason", "target_timeout", "targetID", monsterId, "targetName", monster.Name, "elapsed", elapsed)
			return nil
		}

		if p.isIntermediateSpec() || p.isEndgameSpec() {
			if !p.executeDefaultRotation(monster) {
				continue
			}
		} else {
			if !p.executeMeleeRotation(monster) {
				continue
			}
		}
	}
}

//endregion Combat

//region Leveling Flow

func (p *PaladinLeveling) SkillsToBind() (skill.ID, []skill.ID) {
	p.updateOptionsForLeveling()

	mainSkill := skill.AttackSkill
	skillBindings := []skill.ID{}

	if p.isIntermediateSpec() || p.isEndgameSpec() {
		if p.CanUseSkill(skill.Smite) {
			mainSkill = skill.Smite
			skillBindings = p.AppendUniqueSkill(skillBindings, skill.Smite)
			skillBindings = p.AppendUniqueSkill(skillBindings, p.Options.SmiteAura)
		}
		if p.CanUseSkill(skill.BlessedHammer) {
			mainSkill = skill.BlessedHammer
			skillBindings = p.AppendUniqueSkill(skillBindings, skill.BlessedHammer)
			skillBindings = p.AppendUniqueSkill(skillBindings, p.Options.HammerAura)
		}
		if p.levelingBuild() == paladinLevelingBuildFoh {
			if p.CanUseSkill(skill.HolyBolt) {
				mainSkill = skill.HolyBolt
				skillBindings = p.AppendUniqueSkill(skillBindings, skill.HolyBolt)
				skillBindings = p.AppendUniqueSkill(skillBindings, p.Options.HolyBoltAura)
			}
			if p.CanUseSkill(skill.FistOfTheHeavens) {
				mainSkill = skill.FistOfTheHeavens
				skillBindings = p.AppendUniqueSkill(skillBindings, skill.FistOfTheHeavens)
				skillBindings = p.AppendUniqueSkill(skillBindings, p.Options.FohAura)
			}
		}
	} else {
		if p.CanUseSkill(skill.Zeal) {
			mainSkill = skill.Zeal
			skillBindings = p.AppendUniqueSkill(skillBindings, skill.Zeal)
		} else if p.CanUseSkill(skill.Sacrifice) {
			mainSkill = skill.Sacrifice
			skillBindings = p.AppendUniqueSkill(skillBindings, skill.Sacrifice)
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
	if p.CanUseSkill(skill.Meditation) {
		skillBindings = p.AppendUniqueSkill(skillBindings, skill.Meditation)
	}

	if p.CanUseSkill(skill.HolyShield) {
		skillBindings = p.AppendUniqueSkill(skillBindings, skill.HolyShield)
	}

	// TODO: Handle weapon-swap bindings for these skills.
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
	// From Holy Fire to Intermediate after Act 3 Lam Esens quest in Normal.
	if p.CharacterCfg.Game.Difficulty == difficulty.Normal && p.BaseSkillLevel(skill.HolyFire) >= 1 && p.Data.Quests[quest.Act3LamEsensTome].Completed() {
		p.Logger.Info("Resetting skills to Intermediate: Normal difficulty, Holy Fire >= 1, Act 3 Lam Esens completed.")
		return true
	}

	// Transition to endgame.
	if p.levelingBuild() == paladinLevelingBuildFoh {
		// From Intermediate to Endgame after killing Andariel in Nightmare.
		if p.CharacterCfg.Game.Difficulty == difficulty.Nightmare && p.BaseSkillLevel(skill.ResistFire) >= 1 && p.Data.Quests[quest.Act1SistersToTheSlaughter].Completed() {
			p.Logger.Info("Resetting skills to Endgame FoH: Not in Normal, ResistFire >= 1, Andariel killed.")
			return true
		}
	} else if p.levelingBuild() == paladinLevelingBuildHammer {
		// From Intermediate to Endgame after merc have Insight runeword.
		if p.BaseSkillLevel(skill.Meditation) >= 1 && p.Data.MercHasRuneword(item.RunewordInsight) {
			p.Logger.Info("Resetting skills to Endgame Hammer: Meditation >= 1, Merc have Insight.")
			return true
		}
	}

	return false
}

func (p *PaladinLeveling) StatPoints() []context.StatAllocation {
	playerLevel, _ := p.Data.PlayerUnit.FindStat(stat.Level, 0)

	// Until the endgame respec, allocate extra Energy for easier mana management (to use Charge efficiently).
	if playerLevel.Value <= 35 || p.BaseSkillLevel(skill.HolyFire) >= 1 || p.BaseSkillLevel(skill.ResistFire) >= 1 || p.BaseSkillLevel(skill.Meditation) >= 1 {
		return []context.StatAllocation{
			{Stat: stat.Vitality, Points: 45},   // Level 5
			{Stat: stat.Strength, Points: 35},   // Level 7
			{Stat: stat.Vitality, Points: 60},   // Level 10
			{Stat: stat.Dexterity, Points: 21},  // To wear a Scimitar (Steel Runeword)
			{Stat: stat.Energy, Points: 44},     // Level 16
			{Stat: stat.Vitality, Points: 80},   // Level 20
			{Stat: stat.Strength, Points: 50},   // Level 23
			{Stat: stat.Vitality, Points: 140},  // Level 34
			{Stat: stat.Dexterity, Points: 51},  // Level 40
			{Stat: stat.Strength, Points: 70},   // Level 44
			{Stat: stat.Dexterity, Points: 106}, // Level 55
			{Stat: stat.Vitality, Points: 185},  // Level 63
			{Stat: stat.Dexterity, Points: 141}, // Level 70
			{Stat: stat.Vitality, Points: 210},  // Level 73
			{Stat: stat.Strength, Points: 95},   // Level 80
			{Stat: stat.Vitality, Points: 999},
		}
	}

	// Endgame spec.
	return []context.StatAllocation{
		{Stat: stat.Vitality, Points: 45},   // Level 5
		{Stat: stat.Strength, Points: 35},   // Level 7
		{Stat: stat.Vitality, Points: 60},   // Level 10
		{Stat: stat.Strength, Points: 50},   // Level 13
		{Stat: stat.Vitality, Points: 170},  // Level 34
		{Stat: stat.Dexterity, Points: 50},  // Level 40
		{Stat: stat.Strength, Points: 70},   // Level 44
		{Stat: stat.Dexterity, Points: 105}, // Level 55
		{Stat: stat.Vitality, Points: 215},  // Level 63
		{Stat: stat.Dexterity, Points: 140}, // Level 70
		{Stat: stat.Vitality, Points: 240},  // Level 73
		{Stat: stat.Strength, Points: 95},   // Level 80
		{Stat: stat.Vitality, Points: 999},
	}
}

func (p *PaladinLeveling) SkillPoints() []skill.ID {
	playerLevel, _ := p.Data.PlayerUnit.FindStat(stat.Level, 0)

	// Holy Fire starting build.
	if playerLevel.Value < 10 || p.BaseSkillLevel(skill.HolyFire) >= 1 {
		return []skill.ID{
			skill.Might, skill.Sacrifice, skill.Smite, skill.ResistFire, skill.ResistFire,
			skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire,
			skill.HolyFire, skill.Zeal, skill.Charge, skill.HolyFire, skill.HolyFire,
			skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire,
			skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire, skill.HolyFire,
			skill.HolyFire, skill.HolyFire, skill.ResistFire, skill.ResistFire, skill.ResistFire,
			skill.ResistFire, skill.ResistFire, skill.ResistFire, skill.ResistFire, skill.ResistFire,
			skill.ResistFire, skill.ResistFire, skill.ResistFire, skill.ResistFire, skill.ResistFire,
			skill.ResistFire, skill.ResistFire, skill.ResistFire, skill.ResistFire, skill.ResistFire,
			skill.Salvation, skill.Salvation, skill.Salvation, skill.Salvation, skill.Salvation,
			skill.Salvation, skill.Salvation, skill.Salvation, skill.Salvation, skill.Salvation,
			skill.Salvation, skill.Salvation, skill.Salvation, skill.Salvation, skill.Salvation,
			skill.Salvation, skill.Salvation, skill.Salvation, skill.Salvation, skill.Salvation,
			skill.Zeal, skill.Zeal, skill.Zeal, skill.Zeal, skill.Zeal,
			skill.Zeal, skill.Zeal, skill.Zeal, skill.Zeal, skill.Zeal,
			skill.Zeal, skill.Zeal, skill.Zeal, skill.Zeal, skill.Zeal,
			skill.Zeal, skill.Zeal, skill.Zeal, skill.Zeal,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
		}
	}

	if p.levelingBuild() == paladinLevelingBuildFoh {
		if playerLevel.Value <= 35 || p.BaseSkillLevel(skill.ResistFire) >= 1 {
			// FoH Hammer intermediate build.
			return []skill.ID{
				skill.ResistFire, skill.Smite, skill.Charge,
				skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt,
				skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
				skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
				skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt,
				skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt,
				skill.HolyShield,
				skill.Sacrifice, skill.Zeal, skill.Vengeance, skill.Conversion,
				skill.FistOfTheHeavens, skill.Salvation, skill.HolyBolt,
				skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens,
				skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, // Level 40 (Act 1 NM)
				skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens,
				skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens,
				skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt,
				skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
				skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
				skill.Prayer, skill.Defiance, skill.Cleansing,
				skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
				skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
				skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
				skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
				skill.Redemption, skill.Might, skill.BlessedAim, skill.Concentration, skill.Fanaticism, // Level 87 (Act 2 Hell)
				skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
				skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
				skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, // Level 99
			}
		}

		// FoH Hammer endgame build.
		return []skill.ID{
			skill.Sacrifice, skill.Smite,
			skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt,
			skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt,
			skill.Zeal, skill.Charge, skill.Vengeance, skill.BlessedHammer, skill.Conversion, skill.HolyShield,
			skill.FistOfTheHeavens, skill.Salvation,
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
			skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens,
			skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, // Level 40 (Act 1 NM)
			skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens,
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, // Level 50 (Act 2 NM)
			skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens, skill.FistOfTheHeavens,
			skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt,
			skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, skill.HolyBolt, // Level 61 (Act 4 NM)
			skill.Prayer, skill.Defiance, skill.Cleansing,
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
			skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
			skill.Redemption, skill.Might, skill.BlessedAim, skill.Concentration, skill.Fanaticism, // Level 87 (Act 2 Hell)
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, // Level 99
		}
	}

	if p.levelingBuild() == paladinLevelingBuildHammer {
		if playerLevel.Value <= 35 || p.BaseSkillLevel(skill.Meditation) >= 1 {
			// Hammer intermediate build.
			return []skill.ID{
				skill.Prayer, skill.Defiance, skill.Cleansing, skill.Vigor, skill.Meditation,
				skill.Might, skill.BlessedAim, skill.Concentration,
				skill.Smite, skill.HolyBolt, skill.Charge, skill.BlessedHammer, skill.HolyShield,
				skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
				skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
				skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
				skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Concentration,
				skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
				skill.Redemption, skill.Concentration,
				skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
				skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
				skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
				skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
				skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration,
				skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
				skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration, skill.Concentration,
				skill.Vigor, skill.Vigor, skill.Vigor, skill.Vigor,
				skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
				skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
				skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
				skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
				skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
				skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
				skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
				skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			}
		}

		// Hammer endgame build.
		return []skill.ID{
			skill.Prayer, skill.Defiance, skill.Cleansing, skill.Vigor,
			skill.Might, skill.BlessedAim, skill.Concentration,
			skill.Smite, skill.HolyBolt, skill.Charge, skill.BlessedHammer, skill.HolyShield,
			skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Vigor,
			skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Vigor,
			skill.BlessedHammer, skill.Concentration, skill.BlessedHammer, skill.Vigor,
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
			skill.Redemption, skill.Concentration,
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
			skill.BlessedHammer, skill.BlessedHammer, skill.BlessedHammer,
			skill.Vigor, skill.Concentration, skill.Vigor, skill.Concentration, skill.Vigor, skill.Concentration, skill.Vigor,
			skill.Vigor, skill.Concentration, skill.Vigor, skill.Concentration, skill.Vigor, skill.Concentration,
			skill.Vigor, skill.Concentration, skill.Vigor, skill.Concentration, skill.Vigor, skill.Concentration,
			skill.Vigor, skill.Concentration, skill.Vigor, skill.Concentration, skill.Vigor, skill.Concentration,
			skill.Vigor, skill.Concentration, skill.Vigor, skill.Concentration, skill.Vigor, skill.Concentration,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.BlessedAim, skill.BlessedAim, skill.BlessedAim, skill.BlessedAim,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
			skill.HolyShield, skill.HolyShield, skill.HolyShield, skill.HolyShield,
		}
	}

	p.Logger.Error("No SkillPoints found for Paladin Leveling")

	return []skill.ID{}
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
	p.updateOptionsForLeveling()
	return []string{}
}

func (p *PaladinLeveling) InitialCharacterConfigSetup() {
	p.updateOptionsForLeveling()
}

func (p *PaladinLeveling) AdjustCharacterConfig() {
	p.updateOptionsForLeveling()
}

//endregion Runewords & Config

//region Helpers

func (p *PaladinLeveling) levelingBuild() paladinLevelingBuild {
	build := strings.ToLower(strings.TrimSpace(p.CharacterCfg.Character.PaladinLeveling.LevelingBuild))

	switch build {
	case "foh":
		return paladinLevelingBuildFoh
	default:
		return paladinLevelingBuildHammer
	}
}

func (p *PaladinLeveling) isIntermediateSpec() bool {
	playerLevel, _ := p.Data.PlayerUnit.FindStat(stat.Level, 0)

	if playerLevel.Value < 10 || p.CanUseSkill(skill.HolyFire) {
		return false
	} else if p.levelingBuild() == paladinLevelingBuildFoh {
		return p.CanUseSkill(skill.ResistFire)
	} else if p.levelingBuild() == paladinLevelingBuildHammer {
		return p.CanUseSkill(skill.Meditation)
	}

	return false
}

func (p *PaladinLeveling) isEndgameSpec() bool {
	playerLevel, _ := p.Data.PlayerUnit.FindStat(stat.Level, 0)

	if playerLevel.Value < 10 || p.CanUseSkill(skill.HolyFire) {
		return false
	} else if p.levelingBuild() == paladinLevelingBuildFoh {
		return !p.CanUseSkill(skill.ResistFire)
	} else if p.levelingBuild() == paladinLevelingBuildHammer {
		return !p.CanUseSkill(skill.Meditation)
	}

	return false
}

func (p *PaladinLeveling) updateOptionsForLeveling() {
	opts := paladinDefaultOptions()

	if p.isIntermediateSpec() || p.isEndgameSpec() {
		if p.levelingBuild() == paladinLevelingBuildFoh {
			// TODO: Handle the case where we have 75+ in every res in Hell to use something else than Salvation (+ UseRedemptionToReplenish).
			if p.CanUseSkill(skill.Salvation) {
				opts.FohAura = skill.Salvation
				opts.HolyBoltAura = skill.Salvation
				opts.HammerAura = skill.Salvation
				opts.MovementAura = skill.Salvation
				opts.SmiteAura = skill.Salvation
				opts.UberMephAura = skill.Salvation
			} else if p.CanUseSkill(skill.ResistFire) {
				opts.FohAura = skill.ResistFire
				opts.HolyBoltAura = skill.ResistFire
				opts.HammerAura = skill.ResistFire
				opts.MovementAura = skill.ResistFire
				opts.SmiteAura = skill.ResistFire
				opts.UberMephAura = skill.ResistFire
			} else {
				p.Logger.Error("No suitable aura found for FoH build")
			}

			// Enable Charge only if we have at least one Spirit.
			maxMana, maxManaFound := p.Data.PlayerUnit.FindStat(stat.MaxMana, 0)
			if maxManaFound && maxMana.Value >= 180 {
				opts.UseChargeMovement = true
			} else {
				opts.UseChargeMovement = false
			}
		} else if p.levelingBuild() == paladinLevelingBuildHammer {
			opts.SmiteAura = skill.Concentration
			opts.UseRedemptionOnRaisers = true
			opts.UseRedemptionToReplenish = true

			if p.isIntermediateSpec() {
				if p.Data.MercHasRuneword(item.RunewordInsight) {
					opts.MovementAura = skill.Vigor
				} else {
					opts.MovementAura = skill.Meditation
				}
				opts.UseChargeMovement = true // We always have Meditation and a bit of Energy, so allow Charge.
			} else {
				// Enable Charge only if we have at least one Spirit or our merc has Insight equipped.
				maxMana, maxManaFound := p.Data.PlayerUnit.FindStat(stat.MaxMana, 0)
				if (maxManaFound && maxMana.Value >= 180) || p.Data.MercHasRuneword(item.RunewordInsight) {
					opts.UseChargeMovement = true
				} else {
					opts.UseChargeMovement = false
				}
			}
		}
	} else {
		// Starting spec (Holy Fire).
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
		opts.UseChargeMovement = true
	}

	p.UpdateOptions(opts)

	// Until we become a caster, avoid going back to town for mana potions; it's a waste of time.
	if p.isIntermediateSpec() || p.isEndgameSpec() {
		p.CharacterCfg.BackToTown.NoMpPotions = true
	} else {
		p.CharacterCfg.BackToTown.NoMpPotions = false
	}
	p.CharacterCfg.BackToTown.NoHpPotions = true
	p.CharacterCfg.BackToTown.MercDied = true
	p.CharacterCfg.BackToTown.EquipmentBroken = true
	p.CharacterCfg.Character.ShouldHireAct2MercFrozenAura = false // Keep the Act 2 merc from Normal (Prayer aura).

	p.CharacterCfg.Game.Leveling.EnsurePointsAllocation = true
	p.CharacterCfg.Game.Leveling.EnsureKeyBinding = true
	p.CharacterCfg.Game.Leveling.AutoEquip = true
	p.CharacterCfg.Game.Leveling.AutoEquipFromSharedStash = true

	p.updateLevelingRunewords()

	p.CharacterCfg.CubeRecipes.Enabled = true
	p.CharacterCfg.CubeRecipes.EnabledRecipes = []string{"Perfect Amethyst", "Caster Amulet"}
	p.CharacterCfg.CubeRecipes.JewelsToKeep = 2

	p.CharacterCfg.Gambling.Enabled = true
	p.CharacterCfg.MaxGameLength = 1200 // 20mins
	p.CharacterCfg.Game.MinGoldPickupThreshold = 2000
	p.CharacterCfg.Game.InteractWithShrines = true
	p.CharacterCfg.Game.InteractWithChests = false
	p.CharacterCfg.Game.InteractWithSuperChests = true
	p.CharacterCfg.Character.UseExtraBuffs = true
	p.CharacterCfg.Character.BuffAfterWP = true
	p.CharacterCfg.Character.BuffOnNewArea = true
	if p.isIntermediateSpec() || p.isEndgameSpec() {
		p.CharacterCfg.Character.ClearPathDist = 10
		// Getting Staff and Amulet can take quite a long time
		if p.Data.Quests[quest.Act2RadamentsLair].Completed() && !p.Data.Quests[quest.Act2TaintedSun].Completed() {
			p.CharacterCfg.MaxGameLength = 1800 // 30mins
		}
		// Act 2, Act3 and Act 5 have many Animals
		if (p.Data.Quests[quest.Act2RadamentsLair].Completed() && !p.Data.Quests[quest.Act3TheBlackenedTemple].Completed()) ||
			(p.Data.Quests[quest.Act5SiegeOnHarrogath].Completed() && !p.Data.Quests[quest.Act5RiteOfPassage].Completed()) {
			p.CharacterCfg.Character.ClearPathDist = 5
		}
	} else {
		p.CharacterCfg.Character.ClearPathDist = 15
	}

	p.CharacterCfg.Game.Tristram.ClearPortal = true
	p.CharacterCfg.Game.Tristram.FocusOnElitePacks = true
	p.CharacterCfg.Game.Countess.ClearFloors = false
	p.CharacterCfg.Game.Pit.MoveThroughBlackMarsh = !p.Data.CanTeleport() // Due to Monastery Gate / Tamoe Highlands bug, it's better to walk from Black Marsh
	p.CharacterCfg.Game.Pit.OpenChests = true
	p.CharacterCfg.Game.Pit.FocusOnElitePacks = false
	p.CharacterCfg.Game.Pit.OnlyClearLevel2 = true
	p.CharacterCfg.Game.Andariel.ClearRoom = true
	p.CharacterCfg.Game.Andariel.UseAntidotes = !p.Data.Quests[quest.Act1SistersToTheSlaughter].Completed()
	p.CharacterCfg.Game.Duriel.UseThawing = p.CharacterCfg.Game.Difficulty == difficulty.Normal && !p.Data.Quests[quest.Act2TheSevenTombs].Completed()
	p.CharacterCfg.Game.Mephisto.KillCouncilMembers = false
	p.CharacterCfg.Game.Mephisto.OpenChests = false
	p.CharacterCfg.Game.Mephisto.ExitToA4 = !p.Data.Quests[quest.Act3TheGuardian].Completed()
	p.CharacterCfg.Game.Eldritch.KillShenk = false
	p.CharacterCfg.Game.Baal.ClearFloors = false
	p.CharacterCfg.Game.Baal.DollQuit = p.CharacterCfg.Game.Difficulty == difficulty.Hell
	p.CharacterCfg.Game.Baal.SoulQuit = p.CharacterCfg.Game.Difficulty == difficulty.Hell
	p.CharacterCfg.Game.Baal.OnlyElites = false
}

//endregion Helpers

//region Rotations

func (p *PaladinLeveling) executeMeleeRotation(monster data.Monster) bool {
	skillToUse := skill.AttackSkill
	if p.CanUseSkill(skill.Sacrifice) {
		skillToUse = skill.Sacrifice
	}
	if p.CanUseSkill(skill.Zeal) {
		skillToUse = skill.Zeal
	}

	aura := p.applyAuraOverride(p.Options.ZealAura)
	step.SelectSkill(skillToUse)
	if err := step.PrimaryAttack(monster.UnitID, 1, false, step.Distance(1, 3), step.EnsureAura(aura)); err != nil {
		return false
	}

	return true
}

//endregion Rotations
