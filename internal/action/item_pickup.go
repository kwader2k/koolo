package action

import (
	"errors"
	"fmt"
	"log/slog"
	"slices"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/nip"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/event"
)

func itemFitsInventory(i data.Item) bool {
	invMatrix := context.Get().Data.Inventory.Matrix()

	for y := 0; y <= len(invMatrix)-i.Desc().InventoryHeight; y++ {
		for x := 0; x <= len(invMatrix[0])-i.Desc().InventoryWidth; x++ {
			freeSpace := true
			for dy := 0; dy < i.Desc().InventoryHeight; dy++ {
				for dx := 0; dx < i.Desc().InventoryWidth; dx++ {
					if invMatrix[y+dy][x+dx] {
						freeSpace = false
						break
					}
				}
				if !freeSpace {
					break
				}
			}

			if freeSpace {
				return true
			}
		}
	}

	return false
}

// HasTPsAvailable checks if the player has at least one Town Portal in their tome.
func HasTPsAvailable() bool {
	ctx := context.Get()

	// Check for Tome of Town Portal
	portalTome, found := ctx.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	if !found {
		return false // No portal tome found at all
	}

	qty, found := portalTome.FindStat(stat.Quantity, 0)
	// Return true only if the quantity stat was found and the value is greater than 0
	return found && qty.Value > 0
}

func ItemPickup(maxDistance int) error {
	ctx := context.Get()
	ctx.SetLastAction("ItemPickup")

	const maxRetries = 5                                        // Base retries for various issues
	const maxItemTooFarAttempts = 5                             // Additional retries specifically for "item too far"
	const totalMaxAttempts = maxRetries + maxItemTooFarAttempts // Combined total attempts

	for {
		ctx.PauseIfNotPriority()
		itemsToPickup := GetItemsToPickup(maxDistance)

		if len(itemsToPickup) == 0 {
			return nil
		}

		var itemToPickup data.Item
		for _, i := range itemsToPickup {
			if itemFitsInventory(i) {
				itemToPickup = i
				break
			}
		}

		if itemToPickup.UnitID == 0 {
			ctx.Logger.Debug("No fitting items found for pickup after filtering.")
			if HasTPsAvailable() {
				_, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.TomeOfTownPortal)
				if found {
					ctx.Logger.Debug("TPs available and keybinding found, returning to town to sell junk and stash items.")
					if err := InRunReturnTownRoutine(); err != nil {
						ctx.Logger.Warn("Failed returning to town from ItemPickup", "error", err)
					}
					continue
				} else {
					ctx.Logger.Warn("TPs available but no keybinding found for TomeOfTownPortal. Skipping return to town.")
					return nil
				}
			} else {
				ctx.Logger.Warn("Inventory is full and NO Town Portals found. Skipping return to town and continuing current run (no more item pickups this cycle).")
				return nil
			}
		}

		ctx.Logger.Info(fmt.Sprintf(
			"Attempting to pickup item: %s [%d] at X:%d Y:%d",
			itemToPickup.Name,
			itemToPickup.Quality,
			itemToPickup.Position.X,
			itemToPickup.Position.Y,
		))

		// Try to pick up the item with retries
		var lastError error
		attempt := 1
		itemTooFarRetryCount := 0     // Tracks retries specifically for "item too far"
		totalAttemptCounter := 0      // New counter for overall attempts
		var consecutiveMoveErrors int // New variable to track consecutive ErrCastingMoving errors

		for totalAttemptCounter < totalMaxAttempts { // Loop until totalMaxAttempts is reached
			totalAttemptCounter++
			ctx.Logger.Debug(fmt.Sprintf("Item Pickup: Starting attempt %d (total: %d)", attempt, totalAttemptCounter))
			pickupStartTime := time.Now()

			// Clear monsters on each attempt
			ctx.Logger.Debug(fmt.Sprintf("Item Pickup: Clearing area around item. Attempt %d", attempt))
			ClearAreaAroundPosition(itemToPickup.Position, 4, data.MonsterAnyFilter())
			ctx.Logger.Debug(fmt.Sprintf("Item Pickup: Area cleared in %v. Attempt %d", time.Since(pickupStartTime), attempt))

			// Calculate position to move to based on attempt number
			// on 2nd and 3rd attempt try position left/right of item
			// on 4th and 5th attempt try position further away
			pickupPosition := itemToPickup.Position
			moveDistance := 3
			if attempt > 1 { // Use 'attempt' for the movement strategy, 'totalAttemptCounter' for overall limit
				switch attempt {
				case 2:
					pickupPosition = data.Position{
						X: itemToPickup.Position.X + moveDistance,
						Y: itemToPickup.Position.Y - 1,
					}
				case 3:
					pickupPosition = data.Position{
						X: itemToPickup.Position.X - moveDistance,
						Y: itemToPickup.Position.Y + 1,
					}
				case 4:
					pickupPosition = data.Position{
						X: itemToPickup.Position.X + moveDistance + 2,
						Y: itemToPickup.Position.Y - 3,
					}
				case 5:
					MoveToCoords(ctx.PathFinder.BeyondPosition(ctx.Data.PlayerUnit.Position, itemToPickup.Position, 4))
				}
			}

			distance := ctx.PathFinder.DistanceFromMe(itemToPickup.Position)
			if distance >= 7 || attempt > 1 {
				distanceToFinish := 3
				if attempt > 1 {
					distanceToFinish = 2
				}
				ctx.Logger.Debug(fmt.Sprintf("Item Pickup: Moving to coordinates X:%d Y:%d (distance: %d, distToFinish: %d). Attempt %d", pickupPosition.X, pickupPosition.Y, distance, distanceToFinish, attempt))
				if err := step.MoveTo(pickupPosition, step.WithDistanceToFinish(distanceToFinish)); err != nil {
					ctx.Logger.Debug(fmt.Sprintf("Item Pickup: Failed moving to item on attempt %d: %v", attempt, err))
					lastError = err

					continue // Go to next total attempt
				}
				ctx.Logger.Debug(fmt.Sprintf("Item Pickup: Move completed in %v. Attempt %d", time.Since(pickupStartTime), attempt))
			}

			// Try to pick up the item
			pickupActionStartTime := time.Now()
			ctx.Logger.Debug(fmt.Sprintf("Item Pickup: Initiating PickupItem action. Attempt %d", attempt))
			err := step.PickupItem(itemToPickup, attempt)
			if err == nil {
				ctx.Logger.Info(fmt.Sprintf("Successfully picked up item: %s [%d] in %v. Total attempts: %d", itemToPickup.Name, itemToPickup.Quality, time.Since(pickupActionStartTime), totalAttemptCounter))
				break // Success! Exit the inner retry loop
			}

			lastError = err
			ctx.Logger.Warn(fmt.Sprintf("Item Pickup: Pickup attempt %d failed: %v", attempt, err), slog.String("itemName", string(itemToPickup.Name))) // <--- FIXED: string(itemToPickup.Name)

			// Here's the fix: Add a pause and an error counter
			if errors.Is(err, step.ErrCastingMoving) {
				consecutiveMoveErrors++
				if consecutiveMoveErrors > 3 {
					// Give up on this item after 3 consecutive failed attempts due to movement
					ctx.Logger.Warn(fmt.Sprintf("Item Pickup: Giving up on item after %d consecutive failed pickup attempts due to movement.", consecutiveMoveErrors))
					lastError = fmt.Errorf("failed to pick up item after multiple attempts due to movement state: %w", err)
					break // Exit the inner loop to blacklist the item
				}
				// Pause to let the game state update from 'walking' to 'idle'
				time.Sleep(100 * time.Millisecond)
				continue
			}
			if errors.Is(err, step.ErrMonsterAroundItem) {
				continue
			}

			// Item too far retry logic: Use itemTooFarRetryCount for this specific error type
			if errors.Is(err, step.ErrItemTooFar) {
				itemTooFarRetryCount++
				ctx.Logger.Debug(fmt.Sprintf("Item Pickup: Item too far detected. ItemTooFar specific retry %d/%d.", itemTooFarRetryCount, maxItemTooFarAttempts))
				// We don't increment 'attempt' here to keep the movement strategy.
				if itemTooFarRetryCount < maxItemTooFarAttempts {
					ctx.PathFinder.RandomMovement() // Add random movement to break potential sticking
					continue                        // Continue in the total loop
				}
				// If we reached maxItemTooFarAttempts, we'll let the totalAttemptCounter handle the blacklist.
			}

			if errors.Is(err, step.ErrNoLOSToItem) {
				ctx.Logger.Debug("Item Pickup: No line of sight to item, moving closer",
					slog.String("item", string(itemToPickup.Desc().Name))) // <--- FIXED: string(itemToPickup.Desc().Name)

				// Try moving beyond the item for better line of sight
				beyondPos := ctx.PathFinder.BeyondPosition(ctx.Data.PlayerUnit.Position, itemToPickup.Position, 2+attempt)
				if mvErr := MoveToCoords(beyondPos); mvErr == nil {
					ctx.Logger.Debug(fmt.Sprintf("Item Pickup: Moved for LOS. Retrying pickup. Attempt %d", attempt))
					err = step.PickupItem(itemToPickup, attempt)
					if err == nil {
						ctx.Logger.Info(fmt.Sprintf("Successfully picked up item after LOS correction: %s [%d] in %v. Total attempts: %d", itemToPickup.Name, itemToPickup.Quality, time.Since(pickupActionStartTime), totalAttemptCounter))
						break
					}
					lastError = err
					ctx.Logger.Warn(fmt.Sprintf("Item Pickup: Pickup attempt %d failed even after LOS correction: %v", attempt, err), slog.String("itemName", string(itemToPickup.Name))) // <--- FIXED: string(itemToPickup.Name)
				} else {
					lastError = mvErr
					ctx.Logger.Warn(fmt.Sprintf("Item Pickup: Failed to move for LOS correction: %v", mvErr), slog.String("itemName", string(itemToPickup.Name))) // <--- FIXED: string(itemToPickup.Name)
				}
			}
			attempt++ // Only increment 'attempt' for general strategy change (movements, etc.)
		}

		// If all attempts failed (totalAttemptCounter reached limit and lastError is not nil)
		if totalAttemptCounter >= totalMaxAttempts && lastError != nil {
			ctx.CurrentGame.BlacklistedItems = append(ctx.CurrentGame.BlacklistedItems, itemToPickup)

			// Screenshot with show items on
			ctx.HID.KeyDown(ctx.Data.KeyBindings.ShowItems)
			// Small delay to ensure items are shown before screenshot
			time.Sleep(200 * time.Millisecond)
			screenshot := ctx.GameReader.Screenshot()
			event.Send(event.ItemBlackListed(event.WithScreenshot(ctx.Name, fmt.Sprintf("Item %s [%s] BlackListed in Area:%s", itemToPickup.Name, itemToPickup.Quality.ToString(), ctx.Data.PlayerUnit.Area.Area().Name), screenshot), data.Drop{Item: itemToPickup}))
			ctx.HID.KeyUp(ctx.Data.KeyBindings.ShowItems)

			ctx.Logger.Warn(
				"Failed picking up item after all attempts, blacklisting it",
				slog.String("itemName", string(itemToPickup.Desc().Name)), // <--- FIXED: string(itemToPickup.Desc().Name)
				slog.Int("unitID", int(itemToPickup.UnitID)),
				slog.String("lastError", lastError.Error()),
				slog.Int("totalAttempts", totalAttemptCounter),
			)
		} else if lastError == nil {
			// Item was successfully picked up, continue the outer loop to check for more items
			continue
		}
	}
}

func GetItemsToPickup(maxDistance int) []data.Item {
	ctx := context.Get()
	ctx.SetLastAction("GetItemsToPickup")

	missingHealingPotions := ctx.BeltManager.GetMissingCount(data.HealingPotion) + ctx.Data.MissingPotionCountInInventory(data.HealingPotion)
	missingManaPotions := ctx.BeltManager.GetMissingCount(data.ManaPotion) + ctx.Data.MissingPotionCountInInventory(data.ManaPotion)
	missingRejuvenationPotions := ctx.BeltManager.GetMissingCount(data.RejuvenationPotion) + ctx.Data.MissingPotionCountInInventory(data.RejuvenationPotion)

	var itemsToPickup []data.Item
	_, isLevelingChar := ctx.Char.(context.LevelingCharacter)

	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationGround) {
		// Skip itempickup on party leveling Maggot Lair, is too narrow and causes characters to get stuck
		if isLevelingChar && itm.Name != "StaffOfKings" && (ctx.Data.PlayerUnit.Area == area.MaggotLairLevel1 ||
			ctx.Data.PlayerUnit.Area == area.MaggotLairLevel2 ||
			ctx.Data.PlayerUnit.Area == area.MaggotLairLevel3 ||
			ctx.Data.PlayerUnit.Area == area.ArcaneSanctuary) {
			continue
		}

		// Skip potion pickup for Berserker Barb in Travincal if configured
		if ctx.CharacterCfg.Character.Class == "berserker" &&
			ctx.CharacterCfg.Character.BerserkerBarb.SkipPotionPickupInTravincal &&
			ctx.Data.PlayerUnit.Area == area.Travincal &&
			itm.IsPotion() {
			continue
		}

		// Skip items that are outside pickup radius, this is useful when clearing big areas to prevent
		// character going back to pickup potions all the time after using them
		itemDistance := ctx.PathFinder.DistanceFromMe(itm.Position)
		if maxDistance > 0 && itemDistance > maxDistance && itm.IsPotion() {
			continue
		}

		if itm.IsPotion() {
			if (itm.IsHealingPotion() && missingHealingPotions > 0) ||
				(itm.IsManaPotion() && missingManaPotions > 0) ||
				(itm.IsRejuvPotion() && missingRejuvenationPotions > 0) {
				if shouldBePickedUp(itm) {
					itemsToPickup = append(itemsToPickup, itm)
					switch {
					case itm.IsHealingPotion():
						missingHealingPotions--
					case itm.IsManaPotion():
						missingManaPotions--
					case itm.IsRejuvPotion():
						missingRejuvenationPotions--
					}
				}
			}
		} else if shouldBePickedUp(itm) {
			itemsToPickup = append(itemsToPickup, itm)
		}
	}

	// Remove blacklisted items from the list, we don't want to pick them up
	filteredItems := make([]data.Item, 0, len(itemsToPickup))
	for _, itm := range itemsToPickup {
		isBlacklisted := false
		for _, blacklistedItem := range ctx.CurrentGame.BlacklistedItems {
			if itm.UnitID == blacklistedItem.UnitID {
				isBlacklisted = true
				break
			}
		}
		if !isBlacklisted {
			filteredItems = append(filteredItems, itm)
		}
	}

	return filteredItems
}

func shouldBePickedUp(i data.Item) bool {
	ctx := context.Get()
	ctx.SetLastAction("shouldBePickedUp")

	// Always pickup Runewords and Wirt's Leg
	if i.IsRuneword || i.Name == "WirtsLeg" {
		return true
	}

	// Pick up quest items if we're in leveling or questing run
	specialRuns := slices.Contains(ctx.CharacterCfg.Game.Runs, "quests") || slices.Contains(ctx.CharacterCfg.Game.Runs, "leveling")
	if specialRuns {
		switch i.Name {
		case "Scroll of Inifuss", "ScrollOfInifuss", "LamEsensTome", "HoradricCube", "AmuletoftheViper", "StaffofKings", "HoradricStaff", "AJadeFigurine", "KhalimsEye", "KhalimsBrain", "KhalimsHeart", "KhalimsFlail":
			return true
		}
	}
	if i.ID == 552 { // Book of Skill doesnt work by name, so we find it by ID
		return true
	}

	if i.ID == 524 { // Scroll of Inifuss
		return true
	}
	// Skip picking up gold if we can not carry more
	gold, _ := ctx.Data.PlayerUnit.FindStat(stat.Gold, 0)
	if gold.Value >= ctx.Data.PlayerUnit.MaxGold() && i.Name == "Gold" {
		ctx.Logger.Debug("Skipping gold pickup, inventory full")
		return false
	}

	// Skip picking up gold, usually early game there are small amounts of gold in many places full of enemies, better
	// stay away of that
	// Leaving it for now, but can probably be removed due to the auto MinGoldPickupThreshold in leveling config
	_, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	if isLevelingChar && ctx.Data.PlayerUnit.TotalPlayerGold() < 2000 && i.Name != "Gold" {
		return true
	}

	// Pickup all magic or superior items if total gold is low, filter will not pass and items will be sold to vendor
	minGoldPickupThreshold := ctx.CharacterCfg.Game.MinGoldPickupThreshold
	if ctx.Data.PlayerUnit.TotalPlayerGold() < minGoldPickupThreshold && i.Quality >= item.QualityMagic {
		return true
	}

	// Evaluate item based on NIP rules
	matchedRule, result := ctx.Data.CharacterCfg.Runtime.Rules.EvaluateAll(i)
	if result == nip.RuleResultNoMatch {
		return false
	}
	if result == nip.RuleResultPartial {
		return true
	}

	// Blacklist item if it exceeds quantity limits according to pickit rules
	if doesExceedQuantity(matchedRule) {
		ctx.CurrentGame.BlacklistedItems = append(ctx.CurrentGame.BlacklistedItems, i)
		ctx.Logger.Debug(fmt.Sprintf("Blacklisted item %s (UnitID: %d) because it exceeds quantity limits defined in pickit.", i.Name, i.UnitID))
		return false // Do not pick up the item if it exceeds quantity
	}

	return true
}
