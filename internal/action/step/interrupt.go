package step

import (
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/delivery"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// DELIVERY: interruptDeliveryIfRequested checks if a delivery is pending and returns an error to interrupt the current operation
// This function is injected into all major step functions (attack, move, pickup, interact) to enable
// immediate response to delivery requests from the server API.
func interruptDeliveryIfRequested() error {
	ctx := context.Get()
	if ctx == nil || ctx.Context == nil || ctx.Context.Delivery == nil {
		return nil
	}

	if ctx.Context.Delivery.Pending() != nil && ctx.Context.Delivery.Active() == nil {
		// Exit game immediately to speed up delivery response
		if ctx.Manager.InGame() {
			ctx.Logger.Info("Delivery request detected, exiting game immediately")
			ctx.Manager.ExitGame()
			utils.Sleep(150)
		}

		return delivery.ErrInterrupt
	}
	return nil
}

// CleanupForDelivery ensures menus/input are reset before transition to delivery flow.
func CleanupForDelivery() {
	ctx := context.Get()
	ctx.SetLastStep("DeliveryCleanup")
	_ = CloseAllMenus()
	utils.Sleep(200)
}
