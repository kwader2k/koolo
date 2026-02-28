package paladin

import (
	"net/url"
	"strings"

	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/utils"
)

func ApplyUI(values url.Values, cfg *config.CharacterCfg) {
	defaults := auraDefaultsForBuild(cfg.Character.Class)
	cfg.Character.Paladin.HammerAura = auraValue(values, "paladinHammerAura", defaults.HammerAura)
	cfg.Character.Paladin.FohAura = auraValue(values, "paladinFohAura", defaults.FohAura)
	cfg.Character.Paladin.HolyBoltAura = auraValue(values, "paladinHolyBoltAura", defaults.HolyBoltAura)
	cfg.Character.Paladin.MovementAura = auraValue(values, "paladinMovementAura", defaults.MovementAura)
	cfg.Character.Paladin.ZealAura = auraValue(values, "paladinZealAura", defaults.ZealAura)
	cfg.Character.Paladin.SmiteAura = auraValue(values, "paladinSmiteAura", defaults.SmiteAura)
	cfg.Character.Paladin.UberMephAura = auraValue(values, "paladinUberMephAura", defaults.UberMephAura)
	cfg.Character.Paladin.UseChargeMovement = values.Has("paladinUseChargeMovement")
	cfg.Character.Paladin.UseRedemptionOnRaisers = values.Has("paladinUseRedemptionOnRaisers")
	cfg.Character.Paladin.UseRedemptionToReplenish = values.Has("paladinUseRedemptionToReplenish")
	cfg.Character.Paladin.RedemptionHpThreshold = utils.ParsePercentOrDefault(values.Get("paladinRedemptionHpThreshold"), 50)
	cfg.Character.Paladin.RedemptionManaThreshold = utils.ParsePercentOrDefault(values.Get("paladinRedemptionManaThreshold"), 50)
}

type paladinAuraDefaults struct {
	MovementAura string
	HammerAura   string
	SmiteAura    string
	FohAura      string
	HolyBoltAura string
	ZealAura     string
	UberMephAura string
}

func auraDefaultsForBuild(build string) paladinAuraDefaults {
	defaults := paladinAuraDefaults{
		MovementAura: "vigor",
		HammerAura:   "concentration",
		SmiteAura:    "fanaticism",
		FohAura:      "conviction",
		HolyBoltAura: "conviction",
		ZealAura:     "fanaticism",
		UberMephAura: "resist_lightning",
	}
	if strings.EqualFold(build, "paladin_dragon") {
		defaults.MovementAura = "conviction"
		defaults.ZealAura = "conviction"
	}
	return defaults
}

func auraValue(values url.Values, fieldName, fallback string) string {
	value := strings.TrimSpace(values.Get(fieldName))
	if value == "" {
		return fallback
	}
	return value
}

func IsClass(class string) bool {
	switch strings.ToLower(class) {
	case "paladin_leveling", "paladin_default", "paladin_dragon":
		return true
	default:
		return false
	}
}
