package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	mainWeaponSlot = 0
	swapWeaponSlot = 1
)

func ensureWeaponSlot(ctx *context.Status, slot int) error {
	if slot != mainWeaponSlot && slot != swapWeaponSlot {
		return fmt.Errorf("invalid weapon slot %d", slot)
	}

	ctx.RefreshGameData()
	if ctx.Data.ActiveWeaponSlot == slot {
		return nil
	}

	for attempt := 0; attempt < 3; attempt++ {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.SwapWeapons)
		utils.PingSleep(utils.Light, 150)
		ctx.RefreshGameData()
		if ctx.Data.ActiveWeaponSlot == slot {
			return nil
		}
	}

	return fmt.Errorf("failed to switch to weapon slot %d", slot)
}

func questWeaponSlot(itm data.Item) (int, bool) {
	if itm.Location.LocationType != item.LocationEquipped {
		return 0, false
	}

	switch itm.Location.BodyLocation {
	case item.LocLeftArm, item.LocRightArm:
		return mainWeaponSlot, true
	case item.LocLeftArmSecondary, item.LocRightArmSecondary:
		return swapWeaponSlot, true
	default:
		return 0, false
	}
}

func findEquippedQuestWeapon(ctx *context.Status, itemName item.Name) (data.Item, int, bool) {
	itm, found := ctx.Data.Inventory.Find(itemName, item.LocationEquipped)
	if !found {
		return data.Item{}, 0, false
	}

	slot, ok := questWeaponSlot(itm)
	return itm, slot, ok
}

func ensureQuestWeaponEquipped(ctx *context.Status, itemName item.Name, preferSlot int) (data.Item, int, error) {
	defer func() {
		if ctx.Data.ActiveWeaponSlot != mainWeaponSlot {
			if err := ensureWeaponSlot(ctx, mainWeaponSlot); err != nil {
				ctx.Logger.Warn("Failed to return to main weapon slot after quest equip", "item", itemName, "error", err)
			}
		}
		step.CloseAllMenus()
	}()

	ctx.RefreshGameData()
	if equipped, slot, ok := findEquippedQuestWeapon(ctx, itemName); ok {
		return equipped, slot, nil
	}

	itm, found := ctx.Data.Inventory.Find(itemName, item.LocationInventory, item.LocationStash, item.LocationSharedStash)
	if !found {
		return data.Item{}, 0, fmt.Errorf("%s not found in inventory or stash", itemName)
	}

	if itm.Location.LocationType == item.LocationStash || itm.Location.LocationType == item.LocationSharedStash {
		if err := action.TakeItemsFromStash([]data.Item{itm}); err != nil {
			return data.Item{}, 0, err
		}
		ctx.RefreshGameData()
		updated, found := ctx.Data.Inventory.FindByID(itm.UnitID)
		if !found {
			return data.Item{}, 0, fmt.Errorf("%s not found in inventory after stash move", itemName)
		}
		itm = updated
	}

	if itm.Location.LocationType != item.LocationInventory {
		return data.Item{}, 0, fmt.Errorf("%s not found in inventory for equip", itemName)
	}

	if err := ensureWeaponSlot(ctx, preferSlot); err != nil {
		return data.Item{}, 0, err
	}

	if !ctx.Data.OpenMenus.Inventory {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.Sleep(300)
		ctx.RefreshGameData()
	}

	screenPos := ui.GetScreenCoordsForItem(itm)
	ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.ShiftKey)
	utils.Sleep(300)
	ctx.RefreshGameData()

	equipped, slot, ok := findEquippedQuestWeapon(ctx, itemName)
	if !ok {
		return data.Item{}, 0, fmt.Errorf("failed to equip %s", itemName)
	}

	return equipped, slot, nil
}

func withQuestWeaponSlot(ctx *context.Status, itemName item.Name, fn func() error) error {
	ctx.RefreshGameData()
	_, slot, ok := findEquippedQuestWeapon(ctx, itemName)
	if !ok {
		return fmt.Errorf("%s is not equipped", itemName)
	}

	if err := ensureWeaponSlot(ctx, slot); err != nil {
		return err
	}
	defer func() {
		if err := ensureWeaponSlot(ctx, mainWeaponSlot); err != nil {
			ctx.Logger.Warn("Failed to return to main weapon slot after quest use", "item", itemName, "error", err)
		}
	}()

	return fn()
}

func prepareKhalimsWill(ctx *context.Status) error {
	ctx.RefreshGameData()
	if _, found := ctx.Data.Inventory.Find("KhalimsWill", item.LocationInventory, item.LocationStash, item.LocationEquipped); found {
		return nil
	}

	eye, found := ctx.Data.Inventory.Find("KhalimsEye", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if !found {
		ctx.Logger.Info("Khalim's Eye not found, skipping")
		return nil
	}

	brain, found := ctx.Data.Inventory.Find("KhalimsBrain", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if !found {
		ctx.Logger.Info("Khalim's Brain not found, skipping")
		return nil
	}

	heart, found := ctx.Data.Inventory.Find("KhalimsHeart", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if !found {
		ctx.Logger.Info("Khalim's Heart not found, skipping")
		return nil
	}

	flail, found := ctx.Data.Inventory.Find("KhalimsFlail", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if !found {
		ctx.Logger.Info("Khalim's Flail not found, skipping")
		return nil
	}

	if err := action.CubeAddItems(eye, brain, heart, flail); err != nil {
		return err
	}

	if err := action.CubeTransmute(); err != nil {
		return err
	}

	return nil
}
