package action

import (
	"log/slog"

	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	botCtx "github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/lxn/win"
)

func VendorRefill(forceRefill bool, sellJunk bool, tempLock ...[][]int) (err error) {
	ctx := botCtx.Get()
	ctx.SetLastAction("VendorRefill")

	// This is a special case, we want to sell junk, but we don't have enough space to unequip items
	if !forceRefill && !shouldVisitVendor() && len(tempLock) == 0 {
		return nil
	}

	ctx.Logger.Info("Visiting vendor...", slog.Bool("forceRefill", forceRefill))

	vendorNPC := town.GetTownByArea(ctx.Data.PlayerUnit.Area).RefillNPC()
	if vendorNPC == npc.Drognan {
		_, needsBuy := town.ShouldBuyKeys()
		if needsBuy {
			vendorNPC = npc.Lysander
		}
	}
	err = InteractNPC(vendorNPC)
	if err != nil {
		return err
	}

	// Jamella trade button is the first one
	if vendorNPC == npc.Jamella {
		ctx.HID.KeySequence(win.VK_HOME, win.VK_RETURN)
	} else {
		ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
	}

	if sellJunk {
		var lockConfig [][]int
		if len(tempLock) > 0 {
			lockConfig = tempLock[0]
			town.SellJunk(lockConfig)
		} else {
			town.SellJunk()
		}
	}
	SwitchStashTab(4)
	ctx.RefreshGameData()
	town.BuyConsumables(forceRefill)

	return step.CloseAllMenus()
}

func BuyAtVendor(vendor npc.ID, items ...VendorItemRequest) error {
	ctx := botCtx.Get()
	ctx.SetLastAction("BuyAtVendor")

	err := InteractNPC(vendor)
	if err != nil {
		return err
	}

	// Jamella trade button is the first one
	if vendor == npc.Jamella {
		ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
	} else {
		ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
	}

	for _, i := range items {
		SwitchStashTab(i.Tab)
		itm, found := ctx.Data.Inventory.Find(i.Item, item.LocationVendor)
		if found {
			town.BuyItem(itm, i.Quantity)
		} else {
			ctx.Logger.Warn("Item not found in vendor", slog.String("Item", string(i.Item)))
		}
	}

	return step.CloseAllMenus()
}

type VendorItemRequest struct {
	Item     item.Name
	Quantity int
	Tab      int
}

func shouldVisitVendor() bool {
	ctx := botCtx.Get() // ctx is of type *botCtx.Status
	ctx.SetLastStep("shouldVisitVendor")

	// Check if we should sell junk
	if len(town.ItemsToBeSold()) > 0 {
		// Pass the embedded Context field: ctx.Context
		if !hasTownPortalsInInventory(ctx.Context) { // <--- Renamed function call
			ctx.Logger.Debug("Skipping vendor visit (sell junk): No Town Portals available to get to town.")
			return false
		}
		return true
	}

	if ctx.Data.PlayerUnit.TotalPlayerGold() < 1000 {
		return false
	}

	if ctx.BeltManager.ShouldBuyPotions() || town.ShouldBuyTPs() || town.ShouldBuyIDs() {
		// Pass the embedded Context field: ctx.Context
		if !hasTownPortalsInInventory(ctx.Context) {
			ctx.Logger.Debug("Skipping vendor visit (buy consumables): No Town Portals available to get to town.")
			return false
		}
		return true
	}

	return false
}

func hasTownPortalsInInventory(ctx *botCtx.Context) bool { // <--- Renamed function definition
	portalTome, found := ctx.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	if !found {
		return false // No portal tome found, so no TPs, can't go to town.
	}

	qty, found := portalTome.FindStat(stat.Quantity, 0)
	// If quantity stat isn't found, or if quantity is exactly 0, then we can't make a TP.
	return qty.Value > 0 && found
}