package character

import (
	"fmt"
	"log/slog"
	"strings"

	"github.com/hectorgimenez/koolo/internal/character/core"
	"github.com/hectorgimenez/koolo/internal/character/paladin"
	"github.com/hectorgimenez/koolo/internal/context"
)

// Factory for all character classes.
func BuildCharacter(ctx *context.Context) (context.Character, error) {
	base := core.CharacterBase{Context: ctx}
	bc := BaseCharacter{CharacterBase: base}

	build := strings.ToLower(ctx.CharacterCfg.Character.Class)
	// Leveling runs are configured by name in the run list (leveling/leveling_sequence).
	isLevelingRun := false
	for _, run := range ctx.CharacterCfg.Game.Runs {
		runName := strings.ToLower(string(run))
		if strings.Contains(runName, "leveling") || strings.Contains(runName, "leveling_sequence") {
			isLevelingRun = true
			break
		}
	}
	ctx.Logger.Info("Character build selected", slog.String("build", ctx.CharacterCfg.Character.Class), slog.Bool("levelingRun", isLevelingRun))

	if isLevelingRun {
		switch build {
		case "amazon_leveling":
			return AmazonLeveling{BaseCharacter: bc}, nil
		case "assassin":
			return AssassinLeveling{BaseCharacter: bc}, nil
		case "barb", "barb_leveling":
			return BarbLeveling{BaseCharacter: bc}, nil
		case "druid_leveling":
			return DruidLeveling{BaseCharacter: bc}, nil
		case "necromancer":
			return &NecromancerLeveling{BaseCharacter: bc}, nil
		case "paladin_leveling":
			return paladin.NewLeveling(base), nil
		case "sorceress_leveling":
			return SorceressLeveling{BaseCharacter: bc}, nil
		}

		return nil, fmt.Errorf("leveling build %s not implemented", ctx.CharacterCfg.Character.Class)
	}

	switch build {
	// Core
	case "development":
		return DevelopmentCharacter{BaseCharacter: bc}, nil
	case "mule":
		return MuleCharacter{BaseCharacter: bc}, nil

	// Amazon
	case "javazon":
		return Javazon{BaseCharacter: bc}, nil

	// Assassin
	case "trapsin":
		return Trapsin{BaseCharacter: bc}, nil
	case "mosaic":
		return MosaicSin{BaseCharacter: bc}, nil

	// Barbarian
	case "berserker":
		return &Berserker{BaseCharacter: bc}, nil
	case "warcry_barb":
		return &WarcryBarb{BaseCharacter: bc}, nil
	case "whirlwind_barb":
		return &WhirlwindBarb{BaseCharacter: bc}, nil

	// Druid
	case "winddruid":
		return WindDruid{BaseCharacter: bc}, nil

	// Paladin
	case "paladin_default":
		return paladin.NewDefault(base), nil
	case "paladin_dragon":
		return paladin.NewDragon(base), nil

	// Sorceress
	case "sorceress":
		return BlizzardSorceress{BaseCharacter: bc}, nil
	case "fireballsorc":
		return FireballSorceress{BaseCharacter: bc}, nil
	case "nova":
		return NovaSorceress{BaseCharacter: bc}, nil
	case "hydraorb":
		return HydraOrbSorceress{BaseCharacter: bc}, nil
	case "lightsorc":
		return LightningSorceress{BaseCharacter: bc}, nil
	}

	return nil, fmt.Errorf("build %s not implemented", ctx.CharacterCfg.Character.Class)
}
