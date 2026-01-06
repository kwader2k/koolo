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
	if p.BaseSkillLevel(skill.FistOfTheHeavens) >= paladinMainSkillMinLevel {
		required = p.AppendUniqueSkill(required, skill.FistOfTheHeavens)
		required = p.AppendUniqueSkill(required, p.Options.FohAura)
		required = p.AppendUniqueSkill(required, skill.HolyBolt)
		required = p.AppendUniqueSkill(required, p.Options.HolyBoltAura)
	}
	if p.BaseSkillLevel(skill.BlessedHammer) >= paladinMainSkillMinLevel {
		required = p.AppendUniqueSkill(required, skill.BlessedHammer)
		required = p.AppendUniqueSkill(required, p.Options.HammerAura)
	}
	// It is expected that Smite is learned since it's the default fallback skill of this build
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
		if !p.executeDefaultRotation(monster) {
			continue
		}
	}
}

//endregion Combat
