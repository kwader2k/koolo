package step

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

// CastAtPosition selects a skill (if bound) and casts it at the given position.
// Optionally holds stand-still to prevent movement while casting.
// Useful for pre-casting AoE skills (e.g., Blizzard, Blessed Hammer) between Baal waves.
// Returns true if a cast click was issued.
func CastAtPosition(skillID skill.ID, standStill bool, castPos data.Position) bool {
	ctx := context.Get()
	ctx.SetLastStep("CastAtPosition")

	// Temporarily force attack to bypass line-of-sight checks for pre-cast scenarios.
	prevForceAttack := ctx.ForceAttack
	ctx.ForceAttack = true
	defer func() { ctx.ForceAttack = prevForceAttack }()

	// Select the skill if neither mouse button currently has it.
	leftSelected := ctx.Data.PlayerUnit.LeftSkill == skillID
	rightSelected := ctx.Data.PlayerUnit.RightSkill == skillID
	if !leftSelected && !rightSelected {
		skillDesc, found := skill.Skills[skillID]
		if !found {
			ctx.Logger.Debug("CastAtPosition: skill metadata not found", "skillID", int(skillID))
			return false
		}
		leftAllowed := skillDesc.LeftSkill
		rightAllowed := skillDesc.RightSkill
		switch {
		case leftAllowed && !rightAllowed:
			if err := SelectLeftSkill(skillID); err != nil {
				ctx.Logger.Debug("CastAtPosition: failed to select left skill", "skillID", int(skillID), "error", err)
				return false
			}
		case rightAllowed && !leftAllowed:
			if err := SelectRightSkill(skillID); err != nil {
				ctx.Logger.Debug("CastAtPosition: failed to select right skill", "skillID", int(skillID), "error", err)
				return false
			}
		case leftAllowed && rightAllowed:
			// Prefer left to avoid overwriting right-side auras.
			if err := SelectLeftSkill(skillID); err != nil {
				ctx.Logger.Debug("CastAtPosition: failed to select left skill", "skillID", int(skillID), "error", err)
				return false
			}
		default:
			ctx.Logger.Debug("CastAtPosition: skill cannot be selected on either button", "skillID", int(skillID))
			return false
		}
		ctx.RefreshGameData()
		leftSelected = ctx.Data.PlayerUnit.LeftSkill == skillID
		rightSelected = ctx.Data.PlayerUnit.RightSkill == skillID
		if !leftSelected && !rightSelected {
			ctx.Logger.Debug("CastAtPosition: skill not selected after attempt", "skillID", int(skillID))
			return false
		}
	}

	// Hold stand-still if requested.
	if standStill {
		ctx.HID.KeyDown(ctx.Data.KeyBindings.StandStill)
		defer ctx.HID.KeyUp(ctx.Data.KeyBindings.StandStill)
	}

	x, y := ctx.PathFinder.GameCoordsToScreenCords(castPos.X, castPos.Y)
	castIssued := false
	if leftSelected {
		ctx.HID.Click(game.LeftButton, x, y)
		castIssued = true
	} else if rightSelected {
		ctx.HID.Click(game.RightButton, x, y)
		castIssued = true
	}
	return castIssued
}
