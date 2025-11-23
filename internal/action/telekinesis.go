package action

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/context"
)

const (
	// TK distance thresholds based on Kolbot best practices
	MinTKDistance = 5  // Don't waste mana if closer
	MaxTKDistance = 21 // Move closer if farther (out of range)
)

// ShouldUseTelekinesis determines if telekinesis should be used for object interaction
// Returns true only if:
// 1. Character is Sorceress
// 2. UseTelekinesis config is enabled for this Sorceress build
// 3. Character has Telekinesis skill
// 4. Object is within optimal distance range (5-21 units)
func ShouldUseTelekinesis(obj data.Object) bool {
	ctx := context.Get()

	// Check if telekinesis is enabled in config for this Sorceress build
	if !isTelekinesisEnabledInConfig() {
		return false
	}

	// Check if character has Telekinesis skill (skill ID 42)
	tkSkill, found := ctx.Data.PlayerUnit.Skills[skill.Telekinesis]
	if !found || tkSkill.Level < 1 {
		return false
	}

	// Check distance - must be within 5-21 units
	distance := ctx.PathFinder.DistanceFromMe(obj.Position)
	if distance < MinTKDistance || distance > MaxTKDistance {
		return false
	}

	return true
}

// isTelekinesisEnabledInConfig checks if UseTelekinesis is enabled for the current Sorceress build
func isTelekinesisEnabledInConfig() bool {
	ctx := context.Get()

	switch ctx.CharacterCfg.Character.Class {
	case "sorceress":
		return ctx.CharacterCfg.Character.BlizzardSorceress.UseTelekinesis
	case "nova":
		return ctx.CharacterCfg.Character.NovaSorceress.UseTelekinesis
	case "lightsorc":
		return ctx.CharacterCfg.Character.LightningSorceress.UseTelekinesis
	case "hydraorb":
		return ctx.CharacterCfg.Character.HydraOrbSorceress.UseTelekinesis
	case "fireballsorc":
		return ctx.CharacterCfg.Character.FireballSorceress.UseTelekinesis
	case "sorceress_leveling":
		return ctx.CharacterCfg.Character.SorceressLeveling.UseTelekinesis
	default:
		return false
	}
}
