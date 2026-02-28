package paladin

import (
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/character/core"
	"github.com/hectorgimenez/koolo/internal/config"
)

//region Constants

const redemptionReplenishRange = 11 // Game says 10.6 yards.
const convictionRange = 14          // Game says 13.3 yards.

//endregion Constants

//region Types

// PaladinOptions groups paladin-specific configuration values.
type PaladinOptions struct {
	MovementAura             skill.ID
	UseChargeMovement        bool
	HammerAura               skill.ID
	SmiteAura                skill.ID
	FohAura                  skill.ID
	HolyBoltAura             skill.ID
	ZealAura                 skill.ID
	UberMephAura             skill.ID
	UseRedemptionOnRaisers   bool
	UseRedemptionToReplenish bool
	RedemptionHpThreshold    int // Percent (1-100) to trigger Redemption when life is low.
	RedemptionManaThreshold  int // Percent (1-100) to trigger Redemption when mana is low.
}

// PaladinBase centralizes paladin-only helpers shared across builds.
// Builds embed it to reuse aura resolution and attack routines.
type PaladinBase struct {
	core.CharacterBase
	Options                            PaladinOptions
	hammerConsecutiveAttacks           int
	hammerLastPosition                 data.Position
	hammerLastTarget                   data.UnitID
	holyBoltConsecutiveOffTargetHovers int
	holyBoltLastTarget                 data.UnitID
	teleportOverrideActive             bool
	teleportOverrideValue              bool
}

//endregion Types

//region Aura Options

type paladinAuraOption struct {
	Value string
	Label string
	ID    skill.ID
}

// auraOptions lists allowed paladin aura names.
var auraOptions = []paladinAuraOption{
	{Value: "blessed_aim", Label: "Blessed Aim", ID: skill.BlessedAim},
	{Value: "cleansing", Label: "Cleansing", ID: skill.Cleansing},
	{Value: "concentration", Label: "Concentration", ID: skill.Concentration},
	{Value: "conviction", Label: "Conviction", ID: skill.Conviction},
	{Value: "defiance", Label: "Defiance", ID: skill.Defiance},
	{Value: "fanaticism", Label: "Fanaticism", ID: skill.Fanaticism},
	{Value: "holy_fire", Label: "Holy Fire", ID: skill.HolyFire},
	{Value: "holy_freeze", Label: "Holy Freeze", ID: skill.HolyFreeze},
	{Value: "holy_shock", Label: "Holy Shock", ID: skill.HolyShock},
	{Value: "meditation", Label: "Meditation", ID: skill.Meditation},
	{Value: "might", Label: "Might", ID: skill.Might},
	{Value: "prayer", Label: "Prayer", ID: skill.Prayer},
	{Value: "redemption", Label: "Redemption", ID: skill.Redemption},
	{Value: "resist_cold", Label: "Resist Cold", ID: skill.ResistCold},
	{Value: "resist_fire", Label: "Resist Fire", ID: skill.ResistFire},
	{Value: "resist_lightning", Label: "Resist Lightning", ID: skill.ResistLightning},
	{Value: "salvation", Label: "Salvation", ID: skill.Salvation},
	{Value: "sanctuary", Label: "Sanctuary", ID: skill.Sanctuary},
	{Value: "thorns", Label: "Thorns", ID: skill.Thorns},
	{Value: "vigor", Label: "Vigor", ID: skill.Vigor},
}

var auraOptionsMap = func() map[string]skill.ID {
	m := make(map[string]skill.ID, len(auraOptions))
	for _, aura := range auraOptions {
		m[aura.Value] = aura.ID
	}
	return m
}()

func retrieveAuraId(value string) (skill.ID, bool) {
	normalizedValue := strings.ToLower(strings.TrimSpace(value))
	if aura, found := auraOptionsMap[normalizedValue]; found {
		return aura, true
	}

	return 0, false
}

//endregion Aura Options

//region Construction & Options

func paladinDefaultOptions() PaladinOptions {
	return PaladinOptions{
		MovementAura:             skill.Vigor,
		UseChargeMovement:        false,
		HammerAura:               skill.Concentration,
		SmiteAura:                skill.Fanaticism,
		FohAura:                  skill.Conviction,
		HolyBoltAura:             skill.Cleansing,
		ZealAura:                 skill.Fanaticism,
		UberMephAura:             skill.ResistLightning,
		UseRedemptionOnRaisers:   false,
		UseRedemptionToReplenish: false,
		RedemptionHpThreshold:    45,
		RedemptionManaThreshold:  25,
	}
}

func paladinOptionsForBuild(cfg *config.CharacterCfg, build string, isLevelingRun bool) PaladinOptions {
	build = strings.ToLower(build)
	opts := paladinDefaultOptions()

	// Force defaults during leveling since the leveling module drives options.
	if isLevelingRun && build == "paladin_leveling" {
		return opts
	}

	return applyPaladinConfigOverrides(cfg, opts)
}

func applyPaladinConfigOverrides(cfg *config.CharacterCfg, opts PaladinOptions) PaladinOptions {
	if cfg == nil {
		return opts
	}

	if aura, ok := retrieveAuraId(cfg.Character.Paladin.HammerAura); ok {
		opts.HammerAura = aura
	}
	if aura, ok := retrieveAuraId(cfg.Character.Paladin.FohAura); ok {
		opts.FohAura = aura
	}
	if aura, ok := retrieveAuraId(cfg.Character.Paladin.HolyBoltAura); ok {
		opts.HolyBoltAura = aura
	}
	if aura, ok := retrieveAuraId(cfg.Character.Paladin.MovementAura); ok {
		opts.MovementAura = aura
	}
	if aura, ok := retrieveAuraId(cfg.Character.Paladin.ZealAura); ok {
		opts.ZealAura = aura
	}
	if aura, ok := retrieveAuraId(cfg.Character.Paladin.SmiteAura); ok {
		opts.SmiteAura = aura
	}
	if aura, ok := retrieveAuraId(cfg.Character.Paladin.UberMephAura); ok {
		opts.UberMephAura = aura
	}
	opts.UseChargeMovement = cfg.Character.Paladin.UseChargeMovement
	opts.UseRedemptionOnRaisers = cfg.Character.Paladin.UseRedemptionOnRaisers
	opts.UseRedemptionToReplenish = cfg.Character.Paladin.UseRedemptionToReplenish
	opts.RedemptionHpThreshold = normalizePercentThreshold(cfg.Character.Paladin.RedemptionHpThreshold, opts.RedemptionHpThreshold)
	opts.RedemptionManaThreshold = normalizePercentThreshold(cfg.Character.Paladin.RedemptionManaThreshold, opts.RedemptionManaThreshold)

	return opts
}

func normalizePercentThreshold(value, fallback int) int {
	if value < 1 || value > 100 {
		return fallback
	}
	return value
}

func (p *PaladinBase) UpdateOptions(opts PaladinOptions) {
	p.Options = opts
}

//endregion Construction & Options

//region Base Behavior

func (p *PaladinBase) MainSkillRange() int {
	return 1
}

func (p *PaladinBase) redemptionRaisersRange() int {
	// Screen range plus a buffer in case we need to move.
	return p.ScreenRange() + core.MeleeRange
}

func (p *PaladinBase) BuffSkills() []skill.ID {
	if _, found := p.Data.KeyBindings.KeyBindingForSkill(skill.HolyShield); found {
		return []skill.ID{skill.HolyShield}
	}

	return []skill.ID{}
}

//endregion Base Behavior

//region Teleport overrides

func (p *PaladinBase) updateTeleportOverride(disableTeleport bool) {
	if disableTeleport {
		if !p.teleportOverrideActive && p.CharacterCfg.Character.UseTeleport {
			p.teleportOverrideValue = p.CharacterCfg.Character.UseTeleport
			p.teleportOverrideActive = true
			p.SetUseTeleport(false)
		}

		return
	}

	if p.teleportOverrideActive {
		p.SetUseTeleport(p.teleportOverrideValue)
		p.teleportOverrideActive = false
	}
}

func (p *PaladinBase) resetTeleportOverride() {
	p.updateTeleportOverride(false)
}

//endregion
