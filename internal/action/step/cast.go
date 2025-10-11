package step

import (
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

// CastAtPosition sets the right skill and casts it at a given game position using right-click.
// Optionally holds stand-still to prevent movement while casting.
// This is useful for pre-casting AoE skills (e.g., Blizzard, Blessed Hammer) between Baal waves.
func CastAtPosition(sk skill.ID, standStill bool, pos data.Position) {
	ctx := context.Get()
	ctx.SetLastStep("CastAtPosition")

	// Temporarily force attack to bypass LoS checks for pre-cast scenarios
	prevForce := ctx.ForceAttack
	ctx.ForceAttack = true
	defer func() { ctx.ForceAttack = prevForce }()

	// Set right skill if needed
	if kb, found := ctx.Data.KeyBindings.KeyBindingForSkill(sk); found {
		if ctx.Data.PlayerUnit.RightSkill != sk {
			ctx.HID.PressKeyBinding(kb)
			time.Sleep(10 * time.Millisecond)
		}
	} else {
		return // No keybinding for the requested skill
	}

	// Hold stand still if requested
	if standStill {
		ctx.HID.KeyDown(ctx.Data.KeyBindings.StandStill)
		defer ctx.HID.KeyUp(ctx.Data.KeyBindings.StandStill)
	}

	x, y := ctx.PathFinder.GameCoordsToScreenCords(pos.X, pos.Y)
	ctx.HID.Click(game.RightButton, x, y)
}
