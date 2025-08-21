package town

import (
	"fmt"
	"slices"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/nip"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
)

var questItems = []item.Name{
	"StaffOfKings",
	"HoradricStaff",
	"AmuletOfTheViper",
	"KhalimsFlail",
	"KhalimsWill",
	"HellforgeHammer",
}

func BuyConsumables(forceRefill bool) {
	ctx := context.Get()

	missingHealingPots := ctx.BeltManager.GetMissingCount(data.HealingPotion)
	missingManaPots := ctx.BeltManager.GetMissingCount(data.ManaPotion)

	ctx.Logger.Debug(fmt.Sprintf("Buying: %d Healing potions and %d Mana potions", missingHealingPots, missingManaPots))

	// We traverse the items in reverse order because vendor has the best potions at the end
	pot, found := findFirstMatch("superhealingpotion", "greaterhealingpotion", "healingpotion", "lighthealingpotion", "minorhealingpotion")
	if found && missingHealingPots > 0 {
		BuyItem(pot, missingHealingPots)
		missingHealingPots = 0
	}

	pot, found = findFirstMatch("supermanapotion", "greatermanapotion", "manapotion", "lightmanapotion", "minormanapotion")
	// In Normal greater potions are expensive as we are low level, let's keep with cheap ones
	if found && missingManaPots > 0 {
		BuyItem(pot, missingManaPots)
		missingManaPots = 0
	}

	if ShouldBuyTPs() || forceRefill {
		if _, found := ctx.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory); !found && ctx.Data.PlayerUnit.TotalPlayerGold() > 450 {
			ctx.Logger.Info("TP Tome not found, buying one...")
			if itm, itmFound := ctx.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationVendor); itmFound {
				BuyItem(itm, 1)
			}
		}
		ctx.Logger.Debug("Filling TP Tome...")
		if itm, found := ctx.Data.Inventory.Find(item.ScrollOfTownPortal, item.LocationVendor); found {
			if ctx.Data.PlayerUnit.TotalPlayerGold() > 6000 {
				buyFullStack(itm, -1) // -1 for irrelevant currentKeysInInventory
			} else {
				BuyItem(itm, 1)
			}
		}
	}

	if ShouldBuyIDs() || forceRefill {
		if _, found := ctx.Data.Inventory.Find(item.TomeOfIdentify, item.LocationInventory); !found && ctx.Data.PlayerUnit.TotalPlayerGold() > 360 {
			ctx.Logger.Info("ID Tome not found, buying one...")
			if itm, itmFound := ctx.Data.Inventory.Find(item.TomeOfIdentify, item.LocationVendor); itmFound {
				BuyItem(itm, 1)
			}
		}
		ctx.Logger.Debug("Filling IDs Tome...")
		if itm, found := ctx.Data.Inventory.Find(item.ScrollOfIdentify, item.LocationVendor); found {
			if ctx.Data.PlayerUnit.TotalPlayerGold() > 16000 {
				buyFullStack(itm, -1) // -1 for irrelevant currentKeysInInventory
			} else {
				BuyItem(itm, 1)
			}
		}
	}

	keyQuantity, shouldBuyKeys := ShouldBuyKeys() // keyQuantity is total keys in inventory
	if ctx.Data.PlayerUnit.Class != data.Assassin && (shouldBuyKeys || forceRefill) {
		if itm, found := ctx.Data.Inventory.Find(item.Key, item.LocationVendor); found {
			ctx.Logger.Debug("Vendor with keys detected, provisioning...")

			// Only buy if vendor has keys and we have less than 12
			qtyVendor, _ := itm.FindStat(stat.Quantity, 0)
			if (qtyVendor.Value > 0) && (keyQuantity < 12) {
				// Pass keyQuantity to buyFullStack so it knows how many keys we had initially
				buyFullStack(itm, keyQuantity)
			}
		}
	}
}

func findFirstMatch(itemNames ...string) (data.Item, bool) {
	ctx := context.Get()
	for _, name := range itemNames {
		if itm, found := ctx.Data.Inventory.Find(item.Name(name), item.LocationVendor); found {
			return itm, true
		}
	}

	return data.Item{}, false
}

func ShouldBuyTPs() bool {
	portalTome, found := context.Get().Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	if !found {
		return true
	}

	qty, found := portalTome.FindStat(stat.Quantity, 0)

	return qty.Value < 5 || !found
}

func ShouldBuyIDs() bool {
	idTome, found := context.Get().Data.Inventory.Find(item.TomeOfIdentify, item.LocationInventory)
	if !found {
		return true
	}

	qty, found := idTome.FindStat(stat.Quantity, 0)

	return qty.Value < 10 || !found
}

func ShouldBuyKeys() (int, bool) {
	// Re-calculating total keys each time ShouldBuyKeys is called for accuracy
	ctx := context.Get()
	totalKeys := 0
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if itm.Name == item.Key {
			if qty, found := itm.FindStat(stat.Quantity, 0); found {
				totalKeys += qty.Value
			}
		}
	}

	if totalKeys == 0 {
		return 0, true // No keys found, so we should buy
	}

	// We only need to buy if we have less than 12 keys.
	return totalKeys, totalKeys < 12
}

func SellJunk(lockConfig ...[][]int) {
	ctx := context.Get()
	ctx.Logger.Debug("--- SellJunk() function entered ---")
	ctx.Logger.Debug("Selling junk items and excess keys...")

	// --- OPTIMIZED LOGIC FOR SELLING EXCESS KEYS ---
	var allKeyStacks []data.Item
	totalKeys := 0

	// Iterate through ALL items in the inventory to find all key stacks
	// Make sure to re-fetch inventory data before this loop if it hasn't been refreshed recently
	ctx.RefreshGameData() // Crucial to have up-to-date inventory
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if itm.Name == item.Key {
			if qty, found := itm.FindStat(stat.Quantity, 0); found {
				allKeyStacks = append(allKeyStacks, itm)
				totalKeys += qty.Value
			}
		}
	}

	ctx.Logger.Debug(fmt.Sprintf("Total keys found across all stacks in inventory: %d", totalKeys))

	if totalKeys > 12 {
		excessCount := totalKeys - 12
		ctx.Logger.Info(fmt.Sprintf("Found %d excess keys (total %d). Selling them.", excessCount, totalKeys))

		keysSold := 0

		// Sort key stacks by quantity in descending order to sell larger stacks first
		slices.SortFunc(allKeyStacks, func(a, b data.Item) int {
			qtyA, _ := a.FindStat(stat.Quantity, 0)
			qtyB, _ := b.FindStat(stat.Quantity, 0)
			return qtyB.Value - qtyA.Value // Descending order
		})

		// 1. Sell full stacks until we are close to the target
		stacksToProcess := make([]data.Item, len(allKeyStacks))
		copy(stacksToProcess, allKeyStacks)

		for _, keyStack := range stacksToProcess {
			if keysSold >= excessCount {
				break // We've sold enough
			}

			qtyInStack, found := keyStack.FindStat(stat.Quantity, 0)
			if !found {
				continue
			}

			// If selling this entire stack still leaves us with at least 12 keys
			// Or if this stack exactly equals the remaining excess to sell
			if (totalKeys-qtyInStack.Value >= 12) || (qtyInStack.Value == excessCount-keysSold) {
				ctx.Logger.Debug(fmt.Sprintf("Selling full stack of %d keys from %v", qtyInStack.Value, keyStack.Position))
				SellItemFullStack(keyStack)
				keysSold += qtyInStack.Value
				totalKeys -= qtyInStack.Value // Update total keys count
				ctx.RefreshGameData()         // Refresh after selling a full stack
				time.Sleep(200 * time.Millisecond) // Short delay for UI update
			}
		}

		// Re-evaluate total keys after selling full stacks
		ctx.RefreshGameData()
		totalKeys = 0
		allKeyStacks = []data.Item{} // Clear and re-populate allKeyStacks
		for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
			if itm.Name == item.Key {
				if qty, found := itm.FindStat(stat.Quantity, 0); found {
					allKeyStacks = append(allKeyStacks, itm)
					totalKeys += qty.Value
				}
			}
		}

		// 2. If there's still excess, sell individual keys from one of the remaining stacks
		if totalKeys > 12 {
			excessCount = totalKeys - 12 // Recalculate excess after full stack sales
			ctx.Logger.Info(fmt.Sprintf("Still have %d excess keys. Selling individually from a remaining stack.", excessCount))

			// Find *any* remaining key stack to sell from
			var remainingKeyStack data.Item
			for _, itm := range allKeyStacks {
				if itm.Name == item.Key {
					remainingKeyStack = itm
					break
				}
			}

			if remainingKeyStack.Name != "" { // Check if a stack was found
				for i := 0; i < excessCount; i++ {
					SellItem(remainingKeyStack)
					keysSold++
					ctx.RefreshGameData()
					time.Sleep(100 * time.Millisecond)
				}
			} else {
				ctx.Logger.Warn("No remaining key stacks found to sell individual keys from, despite excess reported.")
			}
		}

		ctx.Logger.Info(fmt.Sprintf("Finished selling excess keys. Keys sold: %d. Estimated remaining: %d", keysSold, totalKeys-keysSold))
	} else {
		ctx.Logger.Debug("No excess keys to sell (12 or less).")
	}
	// --- END OPTIMIZED LOGIC ---

	// Existing logic to sell other junk items, now with lockConfig support
	for _, i := range ItemsToBeSold(lockConfig...) {
		SellItem(i)
	}
}

// SellItem sells a single item by Control-Clicking it.
func SellItem(i data.Item) {
	ctx := context.Get()
	screenPos := ui.GetScreenCoordsForItem(i)

	ctx.Logger.Debug(fmt.Sprintf("Attempting to sell single item %s at screen coords X:%d Y:%d", i.Desc().Name, screenPos.X, screenPos.Y))

	time.Sleep(200 * time.Millisecond)
	ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
	time.Sleep(200 * time.Millisecond)
	ctx.Logger.Debug(fmt.Sprintf("Item %s [%s] sold", i.Desc().Name, i.Quality.ToString()))
}

// SellItemFullStack sells an entire stack of items by Ctrl-Clicking it.
func SellItemFullStack(i data.Item) {
	ctx := context.Get()
	screenPos := ui.GetScreenCoordsForItem(i)

	ctx.Logger.Debug(fmt.Sprintf("Attempting to sell full stack of item %s at screen coords X:%d Y:%d", i.Desc().Name, screenPos.X, screenPos.Y))

	time.Sleep(200 * time.Millisecond)
	ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
	time.Sleep(500 * time.Millisecond)
	ctx.Logger.Debug(fmt.Sprintf("Full stack of %s [%s] sold", i.Desc().Name, i.Quality.ToString()))
}

func BuyItem(i data.Item, quantity int) {
	ctx := context.Get()
	screenPos := ui.GetScreenCoordsForItem(i)

	time.Sleep(250 * time.Millisecond)
	for k := 0; k < quantity; k++ {
		ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
		time.Sleep(900 * time.Millisecond)
		ctx.Logger.Debug(fmt.Sprintf("Purchased %s [X:%d Y:%d]", i.Desc().Name, i.Position.X, i.Position.Y))
	}
}

// buyFullStack is for buying full stacks of items from a vendor (e.g., potions, scrolls, keys)
// For keys, currentKeysInInventory determines if a special double-click behavior is needed.
func buyFullStack(i data.Item, currentKeysInInventory int) {
	ctx := context.Get()
	screenPos := ui.GetScreenCoordsForItem(i)

	ctx.Logger.Debug(fmt.Sprintf("Attempting to buy full stack of %s from vendor at screen coords X:%d Y:%d", i.Desc().Name, screenPos.X, screenPos.Y))

	// First click: Standard Shift + Right Click for buying a stack from a vendor.
	// As per user's observation:
	// - If 0 keys: this buys 1 key.
	// - If >0 keys: this fills the current stack.
	ctx.HID.ClickWithModifier(game.RightButton, screenPos.X, screenPos.Y, game.ShiftKey)
	time.Sleep(500 * time.Millisecond)

	// Special handling for keys: only perform a second click if starting from 0 keys.
	if i.Name == item.Key {
		if currentKeysInInventory == 0 {
			// As per user: if 0 keys, first click buys 1, second click fills the stack.
			ctx.Logger.Debug("Initial keys were 0. Performing second Shift+Right Click to fill key stack.")
			ctx.HID.ClickWithModifier(game.RightButton, screenPos.X, screenPos.Y, game.ShiftKey)
			time.Sleep(500 * time.Millisecond) // Add another delay for the second click
		} else {
			// As per user: if > 0 keys, the first click should have already filled the stack.
			// No second click is needed to avoid buying an unnecessary extra key/stack.
			ctx.Logger.Debug("Initial keys were > 0. Single Shift+Right Click should have filled stack. No second click needed.")
		}
	}

	ctx.Logger.Debug(fmt.Sprintf("Finished full stack purchase attempt for %s", i.Desc().Name))
}

func ItemsToBeSold(lockConfig ...[][]int) (items []data.Item) {
	ctx := context.Get()

	var currentLockConfig [][]int
	if len(lockConfig) > 0 {
		currentLockConfig = lockConfig[0]
	} else {
		currentLockConfig = ctx.CharacterCfg.Inventory.InventoryLock
	}

	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		// Check if the item is in a locked slot, and if so, skip it.
		if len(currentLockConfig) > itm.Position.Y && len(currentLockConfig[itm.Position.Y]) > itm.Position.X {
			if currentLockConfig[itm.Position.Y][itm.Position.X] == 0 {
				continue
			}
		}

		isQuestItem := slices.Contains(questItems, itm.Name)
		if itm.IsFromQuest() || isQuestItem {
			continue
		}

		if itm.Name == item.TomeOfTownPortal || itm.Name == item.TomeOfIdentify || itm.Name == item.Key || itm.Name == "WirtsLeg" {
			continue
		}

		if itm.IsRuneword {
			continue
		}

		if _, result := ctx.Data.CharacterCfg.Runtime.Rules.EvaluateAll(itm); result == nip.RuleResultFullMatch && !itm.IsPotion() {
			continue
		}

		items = append(items, itm)
	}

	return
}