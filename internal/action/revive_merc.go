package action

import (
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	botCtx "github.com/hectorgimenez/koolo/internal/context" // ALIAS THIS IMPORT
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/lxn/win"
)

func ReviveMerc() {
	status := botCtx.Get()

	status.SetLastAction("ReviveMerc") // SetLastAction is a method on Status

	if status.CharacterCfg.Character.UseMerc && status.Data.MercHPPercent() <= 0 && NeedsTPsToContinue(status.Context) {

		status.Logger.Info("Merc is dead, let's revive it!")

		mercNPC := town.GetTownByArea(status.Data.PlayerUnit.Area).MercContractorNPC()

		InteractNPC(mercNPC)

		if mercNPC == npc.Tyrael2 {
			status.HID.KeySequence(win.VK_END, win.VK_UP, win.VK_RETURN, win.VK_ESCAPE)
		} else {
			status.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN, win.VK_ESCAPE)
		}
	}
}

// NeedsTPsToContinue now correctly accepts *botCtx.Context and checks for at least 1 TP
func NeedsTPsToContinue(ctx *botCtx.Context) bool {
	portalTome, found := ctx.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	if !found {
		return false // No portal tome found, so no TPs, can't go to town.
	}

	qty, found := portalTome.FindStat(stat.Quantity, 0)
	// If quantity stat isn't found, or if quantity is exactly 0, then we can't make a TP.
	return qty.Value > 0 && found
}
