package action

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/nip"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

func doesExceedQuantity(rule nip.Rule) bool {
	ctx := context.Get()
	ctx.SetLastAction("doesExceedQuantity")

	stashItems := ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash)

	maxQuantity := rule.MaxQuantity()
	if maxQuantity == 0 {
		return false
	}

	if maxQuantity == 0 {
		return false
	}

	matchedItemsInStash := 0

	for _, stashItem := range stashItems {
		res, _ := rule.Evaluate(stashItem)
		if res == nip.RuleResultFullMatch {
			matchedItemsInStash += 1
		}
	}

	return matchedItemsInStash >= maxQuantity
}

func DropMouseItem() {
	ctx := context.Get()
	ctx.SetLastAction("DropMouseItem")

	if len(ctx.Data.Inventory.ByLocation(item.LocationCursor)) > 0 {
		utils.Sleep(1000)
		ctx.HID.Click(game.LeftButton, 500, 500)
		utils.Sleep(1000)
	}
}

// DropAndRecoverCursorItem drops any item on cursor and immediately picks it back up,
// bypassing pickit rules. Use this to recover accidentally stuck cursor items.
func DropAndRecoverCursorItem() {
	ctx := context.Get()
	ctx.SetLastAction("DropAndRecoverCursorItem")

	ctx.RefreshInventory()
	cursorItems := ctx.Data.Inventory.ByLocation(item.LocationCursor)
	if len(cursorItems) == 0 {
		return
	}

	droppedItem := cursorItems[0]
	droppedUnitID := droppedItem.UnitID
	ctx.Logger.Debug("Dropping cursor item for recovery", "item", droppedItem.Name, "unitID", droppedUnitID)

	// Drop the item
	utils.Sleep(500)
	ctx.HID.Click(game.LeftButton, 500, 500)
	utils.Sleep(500)

	// Wait for game to register the dropped item on ground
	ctx.RefreshGameData()
	utils.Sleep(300)

	// Retry loop to find and pick up the dropped item
	const maxAttempts = 5
	for attempt := 1; attempt <= maxAttempts; attempt++ {
		ctx.RefreshGameData()

		// Try to find by UnitID first
		var groundItem data.Item
		var found bool
		for _, gi := range ctx.Data.Inventory.ByLocation(item.LocationGround) {
			if gi.UnitID == droppedUnitID {
				groundItem = gi
				found = true
				break
			}
		}

		// Fallback: find by name near player if UnitID changed
		if !found {
			for _, gi := range ctx.Data.Inventory.ByLocation(item.LocationGround) {
				if gi.Name == droppedItem.Name {
					dist := ctx.PathFinder.DistanceFromMe(gi.Position)
					if dist < 10 {
						groundItem = gi
						found = true
						break
					}
				}
			}
		}

		if !found {
			ctx.Logger.Debug("Item not found on ground yet, retrying", "attempt", attempt)
			utils.Sleep(300)
			continue
		}

		ctx.Logger.Debug("Recovering dropped cursor item", "item", groundItem.Name, "attempt", attempt)
		if err := step.PickupItem(groundItem, attempt); err != nil {
			ctx.Logger.Warn("Pickup attempt failed", "error", err, "attempt", attempt)
			utils.Sleep(300)
			continue
		}

		// Verify pickup succeeded
		utils.Sleep(300)
		ctx.RefreshGameData()
		stillOnGround := false
		for _, gi := range ctx.Data.Inventory.ByLocation(item.LocationGround) {
			if gi.UnitID == groundItem.UnitID {
				stillOnGround = true
				break
			}
		}
		if !stillOnGround {
			ctx.Logger.Debug("Successfully recovered cursor item", "item", groundItem.Name)
			return
		}
	}

	ctx.Logger.Warn("Failed to recover cursor item after max attempts", "item", droppedItem.Name)
}

func DropInventoryItem(i data.Item) error {
	ctx := context.Get()
	ctx.SetLastAction("DropInventoryItem")

	closeAttempts := 0

	// Check if any other menu is open, except the inventory
	ctx.RefreshGameData()
	for ctx.Data.OpenMenus.IsMenuOpen() {

		// Press escape to close it
		ctx.HID.PressKey(0x1B) // ESC
		utils.Sleep(500)
		ctx.RefreshGameData()
		closeAttempts++

		if closeAttempts >= 5 {
			return fmt.Errorf("failed to close open menu after 5 attempts")
		}
	}

	if i.Location.LocationType == item.LocationInventory {

		// Check if the inventory is open, if not open it
		if !ctx.Data.OpenMenus.Inventory {
			ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		}

		// Wait a second
		utils.Sleep(1000)

		screenPos := ui.GetScreenCoordsForItem(i)
		ctx.HID.MovePointer(screenPos.X, screenPos.Y)
		utils.Sleep(250)
		ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
		utils.Sleep(500)

		// Close the inventory if its still open, which should be at this point
		if ctx.Data.OpenMenus.Inventory {
			ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		}
	}

	return nil
}
func IsInLockedInventorySlot(itm data.Item) bool {
	// Check if item is in inventory
	if itm.Location.LocationType != item.LocationInventory {
		return false
	}

	// Get the lock configuration from character config
	ctx := context.Get()
	lockConfig := ctx.CharacterCfg.Inventory.InventoryLock
	if len(lockConfig) == 0 {
		return false
	}

	// Calculate row and column in inventory
	row := itm.Position.Y
	col := itm.Position.X

	// Check if position is within bounds
	if row >= len(lockConfig) || col >= len(lockConfig[0]) {
		return false
	}

	// 0 means locked, 1 means unlocked
	return lockConfig[row][col] == 0
}

func DrinkAllPotionsInInventory() {
	ctx := context.Get()
	ctx.SetLastStep("DrinkPotionsInInventory")

	step.OpenInventory()

	for _, i := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if i.IsPotion() {
			if ctx.CharacterCfg.Inventory.InventoryLock[i.Position.Y][i.Position.X] == 0 {
				continue
			}

			screenPos := ui.GetScreenCoordsForItem(i)
			utils.Sleep(100)
			ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
			utils.Sleep(200)
		}
	}

	step.CloseAllMenus()
}
