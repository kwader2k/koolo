package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	mainWeaponSlot    = 0
	swapWeaponSlot    = 1
	weaponSlotUnknown = -1
)

// ensureActiveWeaponSlot swaps until the requested weapon set is active.
func ensureActiveWeaponSlot(ctx *context.Status, slot int) error {
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

func weaponSlotForEquippedItem(itm data.Item) (int, bool) {
	if itm.Location.LocationType != item.LocationEquipped {
		return 0, false
	}

	// Weapon set is derived from the body slot, which distinguishes primary vs. secondary.
	switch itm.Location.BodyLocation {
	case item.LocLeftArm, item.LocRightArm:
		return mainWeaponSlot, true
	case item.LocLeftArmSecondary, item.LocRightArmSecondary:
		return swapWeaponSlot, true
	default:
		return 0, false
	}
}

// findEquippedQuestWeapon returns the equipped quest weapon and its slot.
// If the item is equipped but the slot can't be derived, it returns weaponSlotUnknown.
func findEquippedQuestWeapon(ctx *context.Status, itemName item.Name) (data.Item, int, bool) {
	itm, found := ctx.Data.Inventory.Find(itemName, item.LocationEquipped)
	if !found {
		return data.Item{}, weaponSlotUnknown, false
	}

	slot, ok := weaponSlotForEquippedItem(itm)
	if ok {
		return itm, slot, true
	}
	return itm, weaponSlotUnknown, true
}

// resolveEquippedQuestWeaponSlot attempts to detect the equipped weapon slot, probing by swap if needed.
func resolveEquippedQuestWeaponSlot(ctx *context.Status, itemName item.Name) (data.Item, int, error) {
	ctx.RefreshGameData()
	itm, slot, found := findEquippedQuestWeapon(ctx, itemName)
	if !found {
		originalSlot := ctx.Data.ActiveWeaponSlot
		alternateSlot := mainWeaponSlot
		if originalSlot == mainWeaponSlot {
			alternateSlot = swapWeaponSlot
		}

		if err := ensureActiveWeaponSlot(ctx, alternateSlot); err != nil {
			ctx.Logger.Debug("Failed to swap while searching for equipped quest weapon", "item", itemName, "error", err)
		} else {
			ctx.RefreshGameData()
			itm, slot, found = findEquippedQuestWeapon(ctx, itemName)
		}

		if ctx.Data.ActiveWeaponSlot != originalSlot {
			if err := ensureActiveWeaponSlot(ctx, originalSlot); err != nil {
				ctx.Logger.Debug("Failed to restore weapon slot after quest weapon search", "item", itemName, "error", err)
			}
		}

		if !found {
			return data.Item{}, weaponSlotUnknown, fmt.Errorf("%s is not equipped", itemName)
		}
	}
	if slot != weaponSlotUnknown {
		return itm, slot, nil
	}

	ctx.Logger.Debug("Quest weapon slot unknown; probing via weapon swap", "item", itemName)

	originalSlot := ctx.Data.ActiveWeaponSlot
	alternateSlot := mainWeaponSlot
	if originalSlot == mainWeaponSlot {
		alternateSlot = swapWeaponSlot
	}

	if err := ensureActiveWeaponSlot(ctx, alternateSlot); err != nil {
		ctx.Logger.Debug("Failed to swap while resolving quest weapon slot", "item", itemName, "error", err)
		if ctx.Data.ActiveWeaponSlot != originalSlot {
			_ = ensureActiveWeaponSlot(ctx, originalSlot)
		}
		return itm, weaponSlotUnknown, nil
	}

	ctx.RefreshGameData()
	updated, updatedSlot, ok := findEquippedQuestWeapon(ctx, itemName)
	if err := ensureActiveWeaponSlot(ctx, originalSlot); err != nil {
		ctx.Logger.Debug("Failed to restore weapon slot after quest slot probe", "item", itemName, "error", err)
	}
	if ok && updatedSlot != weaponSlotUnknown {
		return updated, updatedSlot, nil
	}

	ctx.Logger.Debug("Quest weapon slot still unknown after probe", "item", itemName)
	return itm, weaponSlotUnknown, nil
}

// ensureQuestWeaponEquipped equips the quest weapon in the preferred slot and returns its equipped slot.
func ensureQuestWeaponEquipped(ctx *context.Status, itemName item.Name, preferSlot int) (data.Item, int, error) {
	defer func() {
		if ctx.Data.ActiveWeaponSlot != mainWeaponSlot {
			if err := ensureActiveWeaponSlot(ctx, mainWeaponSlot); err != nil {
				ctx.Logger.Warn("Failed to return to main weapon slot after quest equip", "item", itemName, "error", err)
			}
		}
		step.CloseAllMenus()
	}()

	ctx.RefreshGameData()
	if equipped, slot, err := resolveEquippedQuestWeaponSlot(ctx, itemName); err == nil {
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

	if err := ensureActiveWeaponSlot(ctx, preferSlot); err != nil {
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

	if slot == weaponSlotUnknown {
		return resolveEquippedQuestWeaponSlot(ctx, itemName)
	}

	return equipped, slot, nil
}

// withQuestWeaponSlot runs fn with the quest weapon active, retrying once on the other slot if needed.
func withQuestWeaponSlot(ctx *context.Status, itemName item.Name, fn func() error) error {
	_, slot, err := resolveEquippedQuestWeaponSlot(ctx, itemName)
	if err != nil {
		return err
	}

	originalLeftSkill := ctx.Data.PlayerUnit.LeftSkill
	leftSkillChanged := false
	ensureAttackSkill := func() {
		if ctx.Data.PlayerUnit.LeftSkill == skill.AttackSkill {
			return
		}
		if err := step.SelectLeftSkill(skill.AttackSkill); err != nil {
			ctx.Logger.Debug("Failed to select Attack skill before quest interaction", "item", itemName, "error", err)
			return
		}
		ctx.RefreshGameData()
		if ctx.Data.PlayerUnit.LeftSkill == skill.AttackSkill {
			leftSkillChanged = true
		}
	}

	defer func() {
		if err := ensureActiveWeaponSlot(ctx, mainWeaponSlot); err != nil {
			ctx.Logger.Warn("Failed to return to main weapon slot after quest use", "item", itemName, "error", err)
		}
		if leftSkillChanged && originalLeftSkill != skill.AttackSkill {
			if err := step.SelectLeftSkill(originalLeftSkill); err != nil {
				ctx.Logger.Debug("Failed to restore left skill after quest interaction", "item", itemName, "error", err)
			}
		}
	}()

	primarySlot := slot
	if primarySlot == weaponSlotUnknown {
		primarySlot = ctx.Data.ActiveWeaponSlot
	}

	if err := ensureActiveWeaponSlot(ctx, primarySlot); err != nil {
		return fmt.Errorf("failed to switch to quest weapon slot %d for %s: %w", primarySlot, itemName, err)
	}
	ensureAttackSkill()
	firstErr := fn()
	if firstErr == nil {
		return nil
	}
	ctx.Logger.Debug("Quest weapon interaction failed on primary slot; retrying other slot",
		"item", itemName,
		"slot", primarySlot,
		"error", firstErr)

	secondarySlot := mainWeaponSlot
	if primarySlot == mainWeaponSlot {
		secondarySlot = swapWeaponSlot
	}
	if secondarySlot == primarySlot {
		return fmt.Errorf("quest weapon interaction failed on slot %d for %s: %w", primarySlot, itemName, firstErr)
	}

	if swapErr := ensureActiveWeaponSlot(ctx, secondarySlot); swapErr != nil {
		return fmt.Errorf("failed to switch to quest weapon slot %d for %s: %w", secondarySlot, itemName, swapErr)
	}
	ensureAttackSkill()
	retryErr := fn()
	if retryErr == nil {
		return nil
	}
	ctx.Logger.Debug("Quest weapon interaction failed on secondary slot",
		"item", itemName,
		"slot", secondarySlot,
		"error", retryErr)

	return fmt.Errorf("quest weapon interaction failed on slots %d and %d for %s: %w", primarySlot, secondarySlot, itemName, retryErr)
}

// prepareKhalimsWill transmutes the quest parts into Khalim's Will if needed.
func prepareKhalimsWill(ctx *context.Status) error {
	ctx.RefreshGameData()
	if _, found := ctx.Data.Inventory.Find("KhalimsWill", item.LocationInventory, item.LocationStash, item.LocationSharedStash, item.LocationEquipped); found {
		return nil
	}

	eye, found := ctx.Data.Inventory.Find("KhalimsEye", item.LocationInventory, item.LocationStash, item.LocationSharedStash, item.LocationEquipped)
	if !found {
		ctx.Logger.Info("Khalim's Eye not found, skipping")
		return nil
	}

	brain, found := ctx.Data.Inventory.Find("KhalimsBrain", item.LocationInventory, item.LocationStash, item.LocationSharedStash, item.LocationEquipped)
	if !found {
		ctx.Logger.Info("Khalim's Brain not found, skipping")
		return nil
	}

	heart, found := ctx.Data.Inventory.Find("KhalimsHeart", item.LocationInventory, item.LocationStash, item.LocationSharedStash, item.LocationEquipped)
	if !found {
		ctx.Logger.Info("Khalim's Heart not found, skipping")
		return nil
	}

	flail, found := ctx.Data.Inventory.Find("KhalimsFlail", item.LocationInventory, item.LocationStash, item.LocationSharedStash, item.LocationEquipped)
	if !found {
		ctx.Logger.Info("Khalim's Flail not found, skipping")
		return nil
	}

	if flail.Location.LocationType == item.LocationEquipped {
		// The flail has to be in inventory before cubing, even if it's on weapon swap.
		var err error
		flail, err = action.EnsureItemNotEquipped(flail)
		if err != nil {
			return err
		}
	}

	if err := action.CubeAddItems(eye, brain, heart, flail); err != nil {
		return err
	}

	if err := action.CubeTransmute(); err != nil {
		return err
	}

	return nil
}
