package step

import (
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/drop"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// Drop: interruptDropIfRequested checks if a Drop is pending and returns an error to interrupt the current operation
// This function is injected into the moveTo function to enable response to Drop requests from the server API.

func interruptDropIfRequested() error {
	ctx := context.Get()
	if ctx == nil || ctx.Context == nil || ctx.Context.Drop == nil {
		return nil
	}

	if ctx.Context.Drop.Pending() != nil && ctx.Context.Drop.Active() == nil {
		// Exit game immediately to speed up Drop response
		if ctx.Manager.InGame() {
			ctx.Logger.Info("Drop request detected, exiting game immediately")
			ctx.Manager.ExitGame()
			utils.Sleep(150)
		}

		return drop.ErrInterrupt
	}
	return nil
}

// CleanupForDrop ensures menus/input are reset before transition to Drop flow.
func CleanupForDrop() {
	ctx := context.Get()
	ctx.SetLastStep("DropCleanup")
	_ = CloseAllMenus()
	utils.Sleep(200)
}
