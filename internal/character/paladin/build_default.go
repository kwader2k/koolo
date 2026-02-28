package paladin

import (
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/character/core"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

//region Types

type PaladinDefault struct {
	PaladinBase
}

//endregion Types

//region Construction

func NewDefault(base core.CharacterBase) *PaladinDefault {
	opts := paladinOptionsForBuild(base.CharacterCfg, "paladin_default", false)
	return &PaladinDefault{
		PaladinBase: PaladinBase{
			CharacterBase: base,
			Options:       opts,
		},
	}
}

//endregion Construction

//region Keybindings

func (p *PaladinDefault) CheckKeyBindings() []skill.ID {
	required := []skill.ID{}
	if p.BaseSkillLevel(skill.FistOfTheHeavens) >= paladinRelevantSkillMinLevel {
		required = p.AppendUniqueSkill(required, skill.FistOfTheHeavens)
		required = p.AppendUniqueSkill(required, p.Options.FohAura)
		required = p.AppendUniqueSkill(required, skill.HolyBolt)
		required = p.AppendUniqueSkill(required, p.Options.HolyBoltAura)
	}
	if p.BaseSkillLevel(skill.BlessedHammer) >= paladinRelevantSkillMinLevel {
		required = p.AppendUniqueSkill(required, skill.BlessedHammer)
		required = p.AppendUniqueSkill(required, p.Options.HammerAura)
	}
	// Smite is expected to be learned; it's this build's fallback skill.
	required = p.AppendUniqueSkill(required, skill.Smite)
	if p.CanUseSkill(p.Options.SmiteAura) {
		required = p.AppendUniqueSkill(required, p.Options.SmiteAura)
	}

	if p.CanUseNonClassSkill(skill.Teleport) && p.CharacterCfg.Character.UseTeleport {
		required = p.AppendUniqueSkill(required, skill.Teleport)
	} else if p.shouldUseChargeMovement() {
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

	if p.CanUseSkill(p.Options.UberMephAura) {
		required = p.AppendUniqueSkill(required, p.Options.UberMephAura)
	}
	if (p.Options.UseRedemptionOnRaisers || p.Options.UseRedemptionToReplenish) && p.CanUseSkill(skill.Redemption) {
		required = p.AppendUniqueSkill(required, skill.Redemption)
	}
	if p.CanUseSkill(skill.Cleansing) {
		required = p.AppendUniqueSkill(required, skill.Cleansing)
	}

	return p.MissingKeyBindings(required)
}

func (p *PaladinDefault) ShouldIgnoreMonster(monster data.Monster) bool {
	return p.shouldIgnoreMonsterDefault(monster)
}

//endregion Keybindings

//region Combat

func (p *PaladinDefault) KillMonsterSequence(
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

		if !p.executeDefaultRotation(monster) {
			continue
		}
	}
}

//endregion Combat
