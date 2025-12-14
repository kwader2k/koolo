package action

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"

	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/lxn/win"
)

// IsDropProtected determines which items must NOT be dropped
func IsDropProtected(i data.Item) bool {
	ctx := context.Get()
	selected := false
	DropperOnly := false
	filtersEnabled := false

	if ctx != nil && ctx.Context != nil {
		if ctx.Context.Drop != nil {
			filtersEnabled = ctx.Context.Drop.DropFiltersEnabled()
			if filtersEnabled {
				selected = ctx.Context.Drop.ShouldDropperItem(string(i.Name), i.Quality, i.Type().Code)
				DropperOnly = ctx.Context.Drop.DropperOnlySelected()
			}
		}
	}

	// Always keep the cube so the bot can continue farming afterward.
	if i.Name == "HoradricCube" {
		return true
	}

	if selected {
		if ctx != nil && ctx.Context != nil && ctx.Context.Drop != nil && !ctx.Context.Drop.HasRemainingDropQuota(string(i.Name)) {
			return true
		}
		return false
	}

	// Keep recipe materials configured in cube settings.
	if shouldKeepRecipeItem(i) {
		return true
	}

	if i.Name == "GrandCharm" && ctx != nil && HasGrandCharmRerollCandidate(ctx) {
		return true
	}

	if !filtersEnabled {
		return false
	}

	if DropperOnly {
		return true
	}

	// Everything else should be dropped for Drop to ensure the stash empties fully.
	return false
}

func RunDropCleanup() error {
	ctx := context.Get()

	ctx.RefreshGameData()

	if !ctx.Data.PlayerUnit.Area.IsTown() {
		if err := ReturnTown(); err != nil {
			return fmt.Errorf("failed to return to town for Drop cleanup: %w", err)
		}
		// Update town/NPC data after the town portal sequence.
		ctx.RefreshGameData()
	}
	RecoverCorpse()

	IdentifyAll(false)
	ctx.PauseIfNotPriority()
	Stash(false)
	ctx.PauseIfNotPriority()
	DropVendorRefill(false, true)
	ctx.PauseIfNotPriority() // Check after VendorRefill
	Stash(false)
	ctx.PauseIfNotPriority() // Check after Stash

	ctx.RefreshGameData()
	if ctx.Data.OpenMenus.IsMenuOpen() {
		step.CloseAllMenus()
	}
	return nil
}

// HasGrandCharmRerollCandidate indicates whether a reroll-able GrandCharm + perfect gems exist in stash.
func HasGrandCharmRerollCandidate(ctx *context.Status) bool {
	ctx.RefreshGameData()
	items := ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash)
	_, ok := hasItemsForGrandCharmReroll(ctx, items)
	return ok
}

// DropVendorRefill is a Drop-specific vendor helper.
// - Always interacts with Akara as the vendor.
// - Sells junk items (and excess keys) via town.SellJunk, respecting optional lockConfig.
// - Does not buy any consumables (no potions, TP or ID scrolls).
func DropVendorRefill(forceRefill bool, sellJunk bool, tempLock ...[][]int) error {
	ctx := context.Get()
	ctx.SetLastAction("DropVendorRefill")

	ctx.RefreshGameData()

	// Determine if there is anything to sell before visiting the vendor.
	var lockConfig [][]int
	if len(tempLock) > 0 {
		lockConfig = tempLock[0]
	}

	hasJunkToSell := false
	if sellJunk {
		if len(lockConfig) > 0 {
			if len(town.ItemsToBeSold(lockConfig)) > 0 {
				hasJunkToSell = true
			}
		} else if len(town.ItemsToBeSold()) > 0 {
			hasJunkToSell = true
		}
	}

	// If we are not selling anything and no temporary lock is provided, skip vendor entirely.
	if !hasJunkToSell {
		return nil
	}

	ctx.Logger.Info("Drop: Visiting Akara for junk sale...", "forceRefill", forceRefill)

	if err := InteractNPC(npc.Akara); err != nil {
		return err
	}

	// Akara trade menu: HOME -> DOWN -> ENTER
	ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)

	if sellJunk {
		if len(lockConfig) > 0 {
			town.SellJunk(lockConfig)
		} else {
			town.SellJunk()
		}
	}

	// Align with existing vendor flow: switch to tab 4 (shared stash area for vendor UI)
	SwitchStashTab(4)
	ctx.RefreshGameData()

	return step.CloseAllMenus()
}
