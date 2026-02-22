package action

import (
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/mode"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	runWalkSampleTimeout = 350 * time.Millisecond
	runWalkSampleStep    = 40 * time.Millisecond
)

// EnsureRunMode tries to reset the run/walk toggle to running at game start.
// It samples a short forced step to detect walking before toggling to avoid flipping when already running.
func EnsureRunMode() {
	ctx := context.Get()
	ctx.SetLastAction("EnsureRunMode")

	if ctx.Data.PlayerUnit.ID == 0 || !ctx.Manager.InGame() {
		return
	}

	if ctx.Data.OpenMenus.LoadingScreen {
		ctx.WaitForGameToLoad()
	}
	if ctx.Data.OpenMenus.IsMenuOpen() {
		return
	}
	if !ctx.Data.PlayerUnit.Area.IsTown() {
		return
	}

	toggleKB, ok := runWalkBinding(ctx.Data.KeyBindings)
	if !ok {
		return
	}
	if !hasKeyBinding(ctx.Data.KeyBindings.ForceMove) {
		return
	}

	centerX := ctx.GameReader.GameAreaSizeX / 2
	centerY := ctx.GameReader.GameAreaSizeY / 2
	screenX := clampInt(centerX+40, 0, ctx.GameReader.GameAreaSizeX-1)
	screenY := clampInt(centerY-10, 0, ctx.GameReader.GameAreaSizeY-1)

	ctx.HID.MovePointer(screenX, screenY)
	ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.ForceMove)
	utils.PingSleep(utils.Light, 80)

	deadline := time.Now().Add(runWalkSampleTimeout)
	for time.Now().Before(deadline) {
		ctx.RefreshGameData()
		switch ctx.Data.PlayerUnit.Mode {
		case mode.Running:
			return
		case mode.Walking, mode.WalkingInTown:
			ctx.HID.PressKeyBinding(toggleKB)
			utils.PingSleep(utils.Light, 120)
			ctx.Logger.Debug("Run/walk toggle reset to running")
			return
		}
		time.Sleep(runWalkSampleStep)
	}
}

func runWalkBinding(bindings data.KeyBindings) (data.KeyBinding, bool) {
	if hasKeyBinding(bindings.ToggleRunWalk) {
		return bindings.ToggleRunWalk, true
	}
	if hasKeyBinding(bindings.Run) {
		return bindings.Run, true
	}

	return data.KeyBinding{}, false
}

func hasKeyBinding(kb data.KeyBinding) bool {
	return (kb.Key1[0] != 0 && kb.Key1[0] != 255) || (kb.Key2[0] != 0 && kb.Key2[0] != 255)
}

func clampInt(value, min, max int) int {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}
