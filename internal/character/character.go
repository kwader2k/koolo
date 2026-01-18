package character

import (
	"fmt"
	"log/slog"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/context"
)

func BuildCharacter(ctx *context.Context) (context.Character, error) {
	bc := BaseCharacter{
		Context: ctx,
	}

	if len(ctx.CharacterCfg.Game.Runs) > 0 && (ctx.CharacterCfg.Game.Runs[0] == "leveling" || ctx.CharacterCfg.Game.Runs[0] == "leveling_sequence") {
		switch strings.ToLower(ctx.CharacterCfg.Character.Class) {
		case "barb", "barb_leveling":
			return BarbLeveling{BaseCharacter: bc}, nil
		case "sorceress_leveling":
			return SorceressLeveling{BaseCharacter: bc}, nil
		case "necromancer":
			return &NecromancerLeveling{BaseCharacter: bc}, nil
		case "paladin":
			return PaladinLeveling{BaseCharacter: bc}, nil
		case "assassin":
			return AssassinLeveling{BaseCharacter: bc}, nil
		case "druid_leveling":
			return DruidLeveling{BaseCharacter: bc}, nil
		case "amazon_leveling":
			return AmazonLeveling{BaseCharacter: bc}, nil
		}

		return nil, fmt.Errorf("leveling only available for sorceress, assassin, necromancer, druid, barbarian and paladin")
	}

	switch strings.ToLower(ctx.CharacterCfg.Character.Class) {
	case "sorceress":
		return BlizzardSorceress{BaseCharacter: bc}, nil
	case "fireballsorc":
		return FireballSorceress{BaseCharacter: bc}, nil
	case "mule":
		return MuleCharacter{BaseCharacter: bc}, nil
	case "nova":
		return NovaSorceress{BaseCharacter: bc}, nil
	case "hydraorb":
		return HydraOrbSorceress{BaseCharacter: bc}, nil
	case "lightsorc":
		return LightningSorceress{BaseCharacter: bc}, nil
	case "hammerdin":
		return Hammerdin{BaseCharacter: bc}, nil
	case "foh":
		return Foh{BaseCharacter: bc}, nil
	case "dragondin":
		return Dragondin{BaseCharacter: bc}, nil
	case "smiter":
		return Smiter{BaseCharacter: bc}, nil
	case "trapsin":
		return Trapsin{BaseCharacter: bc}, nil
	case "mosaic":
		return MosaicSin{BaseCharacter: bc}, nil
	case "winddruid":
		return WindDruid{BaseCharacter: bc}, nil
	case "javazon":
		return Javazon{BaseCharacter: bc}, nil
	case "berserker":
		return &Berserker{BaseCharacter: bc}, nil // Return a pointer to Berserker
	case "warcry_barb":
		return &WarcryBarb{BaseCharacter: bc}, nil
	case "whirlwind_barb":
		return &WhirlwindBarb{BaseCharacter: bc}, nil
	case "development":
		return DevelopmentCharacter{BaseCharacter: bc}, nil
	}

	return nil, fmt.Errorf("class %s not implemented", ctx.CharacterCfg.Character.Class)
}

type BaseCharacter struct {
	*context.Context
}

func (bc BaseCharacter) KillUberIzual() error {
	return fmt.Errorf("character class %s does not support KillUberIzual", bc.CharacterCfg.Character.Class)
}

func (bc BaseCharacter) KillUberDuriel() error {
	return fmt.Errorf("character class %s does not support KillUberDuriel", bc.CharacterCfg.Character.Class)
}

func (bc BaseCharacter) KillLilith() error {
	return fmt.Errorf("character class %s does not support KillLilith", bc.CharacterCfg.Character.Class)
}

func (bc BaseCharacter) KillUberMephisto() error {
	return fmt.Errorf("character class %s does not support KillUberMephisto", bc.CharacterCfg.Character.Class)
}

func (bc BaseCharacter) KillUberDiablo() error {
	return fmt.Errorf("character class %s does not support KillUberDiablo", bc.CharacterCfg.Character.Class)
}

func (bc BaseCharacter) KillUberBaal() error {
	return fmt.Errorf("character class %s does not support KillUberBaal", bc.CharacterCfg.Character.Class)
}

func (bc BaseCharacter) preBattleChecks(id data.UnitID, skipOnImmunities []stat.Resist) bool {
	monster, found := bc.Data.Monsters.FindByID(id)
	if !found {
		return false
	}

	// Skip dead targets early.
	if monster.Stats[stat.Life] <= 0 {
		return false
	}

	// Special case: Vizier can spawn on weird/off-grid tiles in Chaos Sanctuary.
	isVizier := monster.Type == data.MonsterTypeSuperUnique && monster.Name == npc.StormCaster

	// Filter "underwater/off-grid" targets that exist in data but are not actually attackable/reachable.
	// Apply only for nearby targets to avoid changing long-range targeting behavior.
	const sanityRangeYards = 60
	if !isVizier && bc.PathFinder.DistanceFromMe(monster.Position) <= sanityRangeYards {
		if !bc.Data.AreaData.IsWalkable(monster.Position) {
			return false
		}

		// If we cannot teleport, ensure the target is reachable by pathing.
		if !bc.Data.CanTeleport() {
			_, _, pathFound := bc.PathFinder.GetPath(monster.Position)
			if !pathFound {
				return false
			}
		}
	}

	for _, i := range skipOnImmunities {
		if monster.IsImmune(i) {
			bc.Logger.Info("Monster is immune! skipping", slog.String("immuneTo", string(i)))
			return false
		}
	}

	return true
}
