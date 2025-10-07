package action

import (
	"log/slog"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
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
		if needsBuy && ctx.Data.PlayerUnit.Class != data.Assassin {
			vendorNPC = npc.Lysander
		}
	}
	if vendorNPC == npc.Ormus {
		_, needsBuy := town.ShouldBuyKeys()
		if needsBuy && ctx.Data.PlayerUnit.Class != data.Assassin {
			MoveToCoords(data.Position{X: 5224, Y: 5045})
			vendorNPC = npc.Hratli
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
	ctx := botCtx.Get()
	ctx.SetLastStep("shouldVisitVendor")

	if len(town.ItemsToBeSold()) > 0 {
		return true
	}

	if ctx.Data.PlayerUnit.TotalPlayerGold() < 1000 {
		return false
	}

	if ctx.BeltManager.ShouldBuyPotions() || town.ShouldBuyTPs() || town.ShouldBuyIDs() {
		return true
	}

	return false
}
