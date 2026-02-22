package action

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
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

    stashItems := ctx.Data.Inventory.ByLocation(
        item.LocationStash,
        item.LocationSharedStash,
        item.LocationRunesTab,
        item.LocationGemsTab,
        item.LocationMaterialsTab,
    )
    stashItems = FilterDLCGhostItems(stashItems)

    maxQuantity := rule.MaxQuantity()
    if maxQuantity == 0 {
        return false
    }

    matchedItemsInStash := 0

    for _, stashItem := range stashItems {
        res, _ := rule.Evaluate(stashItem)
        if res == nip.RuleResultFullMatch {
            matchedItemsInStash += GetItemQuantity(stashItem)
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
	for ctx.Data.OpenMenus.IsMenuOpen() {

		// Press escape to close it
		ctx.HID.PressKey(0x1B) // ESC
		utils.Sleep(500)
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

// EnsureItemNotEquipped moves an equipped item to inventory (or stash when open) and returns its updated state.
func EnsureItemNotEquipped(itm data.Item) (data.Item, error) {
	if itm.Location.LocationType != item.LocationEquipped {
		return itm, nil
	}

	ctx := context.Get()
	ctx.SetLastAction("EnsureItemNotEquipped")
	needsPersonalStash := requiresPersonalStash(itm)

	if err := step.OpenInventory(); err != nil {
		return itm, err
	}

	updated, moved, err := tryUnequip(ctx, itm)
	if err != nil {
		return itm, err
	}
	if moved {
		return updated, nil
	}

	if !hasInventorySpaceFor(ctx, itm) {
		if !ctx.Data.PlayerUnit.Area.IsTown() {
			if needsPersonalStash {
				return itm, fmt.Errorf("failed to unequip %s from slot %v: inventory full and personal stash unavailable", itm.Name, itm.Location.BodyLocation)
			}
			return itm, fmt.Errorf("failed to unequip %s from slot %v: inventory full and stash unavailable", itm.Name, itm.Location.BodyLocation)
		}

		ctx.Logger.Debug("Inventory full while unequipping item, attempting to free space", "item", itm.Name, "requiresPersonalStash", needsPersonalStash)
		if err := OpenStash(); err != nil {
			return itm, fmt.Errorf("failed to open stash while unequipping %s: %w", itm.Name, err)
		}
		if needsPersonalStash {
			SwitchStashTab(1)
			utils.Sleep(300)
			ctx.RefreshGameData()
		}

		for attempts := 0; attempts < 5 && !hasInventorySpaceFor(ctx, itm); attempts++ {
			ctx.RefreshGameData()
			freedSpace := false
			for _, invItem := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
				if IsInLockedInventorySlot(invItem) || invItem.IsPotion() || isQuestItem(invItem) {
					continue
				}
				switch invItem.Name {
				case "TomeOfTownPortal", "TomeOfIdentify", "Key":
					continue
				}

				ctx.Logger.Debug("Moving inventory item to stash to free space", "item", invItem.Name)
				screenPos := ui.GetScreenCoordsForItem(invItem)
				ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
				utils.Sleep(300)
				ctx.RefreshGameData()

				updated, found := ctx.Data.Inventory.FindByID(invItem.UnitID)
				if !found || updated.Location.LocationType != item.LocationInventory {
					freedSpace = true
					break
				}
			}

			if !freedSpace {
				break
			}
		}
		if needsPersonalStash && !hasInventorySpaceFor(ctx, itm) {
			return itm, fmt.Errorf("failed to unequip %s from slot %v: personal stash full or no space to free inventory slots", itm.Name, itm.Location.BodyLocation)
		}
	}

	updated, moved, err = tryUnequip(ctx, itm)
	if err != nil {
		return itm, err
	}
	if moved {
		return updated, nil
	}

	if !hasInventorySpaceFor(ctx, itm) {
		return itm, fmt.Errorf("failed to unequip %s from slot %v: inventory full and no space could be freed", itm.Name, itm.Location.BodyLocation)
	}

	return itm, fmt.Errorf("failed to unequip %s from slot %v", itm.Name, itm.Location.BodyLocation)

}

func tryUnequip(ctx *context.Status, itm data.Item) (data.Item, bool, error) {
	equipBodyLoc := itm.Location.BodyLocation
	originalSlot := ctx.Data.ActiveWeaponSlot
	targetSlot := originalSlot

	switch equipBodyLoc {
	case item.LocLeftArmSecondary:
		equipBodyLoc = item.LocLeftArm
		targetSlot = 1
	case item.LocRightArmSecondary:
		equipBodyLoc = item.LocRightArm
		targetSlot = 1
	case item.LocLeftArm, item.LocRightArm:
		targetSlot = 0
	}

	swapped := false
	if targetSlot != originalSlot {
		ctx.Logger.Debug("Swapping weapon slot to unequip item", "item", itm.Name, "fromSlot", originalSlot, "toSlot", targetSlot)
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.SwapWeapons)
		utils.Sleep(200)
		ctx.RefreshGameData()

		if ctx.Data.ActiveWeaponSlot != targetSlot {
			return itm, false, fmt.Errorf("failed to switch to weapon slot %d to unequip %s", targetSlot, itm.Name)
		}
		swapped = true
	}
	if swapped {
		defer func() {
			ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.SwapWeapons)
			utils.Sleep(200)
			ctx.RefreshGameData()
		}()
	}

	slotPos := ui.GetEquipCoords(equipBodyLoc, item.LocationEquipped)
	if slotPos.X == 0 && slotPos.Y == 0 {
		return itm, false, fmt.Errorf("failed to resolve equip slot for %s in slot %v", itm.Name, itm.Location.BodyLocation)
	}

	ctx.Logger.Debug("Unequipping item", "item", itm.Name, "slot", itm.Location.BodyLocation)
	ctx.HID.ClickWithModifier(game.LeftButton, slotPos.X, slotPos.Y, game.ShiftKey)
	utils.Sleep(300)
	ctx.RefreshGameData()

	updated, found := ctx.Data.Inventory.FindByID(itm.UnitID)
	if !found {
		return itm, false, fmt.Errorf("failed to find %s after unequipping", itm.Name)
	}

	switch updated.Location.LocationType {
	case item.LocationInventory, item.LocationStash, item.LocationSharedStash:
		return updated, true, nil
	case item.LocationEquipped:
		return updated, false, nil
	default:
		return itm, false, fmt.Errorf("failed to unequip %s from slot %v", itm.Name, itm.Location.BodyLocation)
	}
}

func hasInventorySpaceFor(ctx *context.Status, itm data.Item) bool {
	inv := NewInventoryMask(10, 4)

	if len(ctx.CharacterCfg.Inventory.InventoryLock) > 0 {
		for y := 0; y < len(ctx.CharacterCfg.Inventory.InventoryLock) && y < inv.Height; y++ {
			for x := 0; x < len(ctx.CharacterCfg.Inventory.InventoryLock[y]) && x < inv.Width; x++ {
				if ctx.CharacterCfg.Inventory.InventoryLock[y][x] == 0 {
					inv.Place(x, y, 1, 1)
				}
			}
		}
	}

	for _, invItem := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		w, h := invItem.Desc().InventoryWidth, invItem.Desc().InventoryHeight
		x, y := invItem.Position.X, invItem.Position.Y
		if x < 0 || y < 0 || x+w > inv.Width || y+h > inv.Height {
			continue
		}
		inv.Place(x, y, w, h)
	}

	itemWidth := itm.Desc().InventoryWidth
	itemHeight := itm.Desc().InventoryHeight
	for y := 0; y <= inv.Height-itemHeight; y++ {
		for x := 0; x <= inv.Width-itemWidth; x++ {
			if inv.CanPlace(x, y, itemWidth, itemHeight) {
				return true
			}
		}
	}

	return false
}

//This allows certain quest-type items to be usable from the shared stash.
//This is a temporary fix and should be changed if there is a better approach.

func requiresPersonalStash(itm data.Item) bool {
	if isQuestItem(itm) {
		if _, ok := sharedStashQuestItems[itm.Name]; ok {
			return false
		}
		return true
	}

	return itm.Name == "HoradricCube"
}

var sharedStashQuestItems = map[item.Name]struct{}{
	item.Name("TwistedEssenceOfSuffering"):     {},
	item.Name("ChargedEssenceOfHatred"):        {},
	item.Name("BurningEssenceOfTerror"):        {},
	item.Name("FesteringEssenceOfDestruction"): {},
	item.Name("KeyOfTerror"):                   {},
	item.Name("KeyOfHate"):                     {},
	item.Name("KeyOfDestruction"):              {},
	item.Name("DiablosHorn"):                   {},
	item.Name("BaalsEye"):                      {},
	item.Name("MephistosBrain"):                {},
	item.Name("TokenofAbsolution"):             {},
}

func isQuestItem(itm data.Item) bool {
	if itm.IsFromQuest() {
		return true
	}

	for _, questItem := range questItems {
		if itm.Name == questItem {
			return true
		}
	}

	return false
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

func GetItemQuantity(itm data.Item) int {
	if qty, found := itm.FindStat(stat.Quantity, 0); found && qty.Value > 0 {
		return qty.Value
	}
	if itm.StackedQuantity > 0 {
		return itm.StackedQuantity
	}
	return 1
}

// FilterDLCGhostItems removes DLC tab items that have StackedQuantity == 0.
// The game keeps empty slot entries in DLC tabs after all copies are consumed;
// these ghost items must be excluded to prevent the bot from trying to pick up
// non-existent items.
func FilterDLCGhostItems(items []data.Item) []data.Item {
	filtered := make([]data.Item, 0, len(items))
	for _, itm := range items {
		switch itm.Location.LocationType {
		case item.LocationGemsTab, item.LocationMaterialsTab, item.LocationRunesTab:
			// DLC tab items are always stackable; StackedQuantity == 0 means
			// the slot is empty (ghost entry the game keeps in memory).
			if itm.StackedQuantity <= 0 {
				continue
			}
		}
		filtered = append(filtered, itm)
	}
	return filtered
}
