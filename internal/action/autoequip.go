package action

import (
	"errors"
	"fmt"
	"slices"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// Constants for equipment locations
const (
	EquipDelayMS = 500
	MaxRetries   = 2
)

var (
	ErrFailedToEquip  = errors.New("failed to equip item, quitting game")
	ErrNotEnoughSpace = errors.New("not enough inventory space")

	classItems = map[data.Class][]string{
		data.Amazon:      {"ajav", "abow", "aspe"},
		data.Sorceress:   {"orb"},
		data.Necromancer: {"head"},
		data.Paladin:     {"ashd"},
		data.Barbarian:   {"phlm"},
		data.Druid:       {"pelt"},
		data.Assassin:    {"h2h"},
	}

	// shieldTypes defines items that should be equipped in right arm (technically they can be left or right arm but we don't want to try and equip two shields)
	shieldTypes = []string{"shie", "ashd", "head"}

	// mercBodyLocs defines valid mercenary equipment locations
	// No support for A3 and A5 mercs
	mercBodyLocs = []item.LocationType{item.LocHead, item.LocTorso, item.LocLeftArm}

	// questItems defines items that shouldn't be equipped
	// TODO Fix IsFromQuest() and remove
	questItems = []item.Name{
		"StaffOfKings",
		"HoradricStaff",
		"AmuletOfTheViper",
		"KhalimsFlail",
	}
)

// EvaluateAllItems evaluates and equips items for both player and mercenary
func AutoEquip() error {
	ctx := context.Get()

	for i := 0; i < MaxRetries; i++ {
		allItems := ctx.Data.Inventory.ByLocation(
			item.LocationStash,
			item.LocationInventory,
			item.LocationEquipped,
			item.LocationMercenary,
		)

		playerItems := evaluateItems(allItems, item.LocationEquipped, PlayerScore)
		if err := equipBestItems(playerItems, item.LocationEquipped); err != nil {
			ctx.Logger.Error(fmt.Sprintf("Failed to equip player items: %v", err))
			continue
		}

		*ctx.Data = ctx.GameReader.GetData()

		allItems = ctx.Data.Inventory.ByLocation(
			item.LocationStash,
			item.LocationInventory,
			item.LocationEquipped,
			item.LocationMercenary,
		)

		mercItems := evaluateItems(allItems, item.LocationMercenary, MercScore)
		if ctx.Data.MercHPPercent() > 0 {
			if err := equipBestItems(mercItems, item.LocationMercenary); err != nil {
				ctx.Logger.Error(fmt.Sprintf("Failed to equip mercenary items: %v", err))
				continue
			}
		}

		*ctx.Data = ctx.GameReader.GetData()
		if isEquipmentStable(playerItems, mercItems) {
			ctx.Logger.Debug("All items equipped as planned, no more changes needed.")
			return nil
		}
	}

	return fmt.Errorf("failed to equip all best items after multiple retries")
}

// isEquipmentStable checks if the currently equipped items match the top-ranked items from the last evaluation.
func isEquipmentStable(playerItems, mercItems map[item.LocationType][]data.Item) bool {
	ctx := context.Get()
	isStable := true

	// Check player equipment
	for loc, items := range playerItems {
		if len(items) > 0 {
			bestItem := items[0]
			equippedItem := GetEquippedItem(ctx.Data.Inventory, loc)
			if equippedItem.UnitID == 0 || equippedItem.UnitID != bestItem.UnitID {
				ctx.Logger.Debug(fmt.Sprintf("Player equipment unstable at %s. Best item is %s, but equipped is %s", loc, bestItem.Name, equippedItem.Name))
				isStable = false
			}
		}
	}

	// Check mercenary equipment
	for loc, items := range mercItems {
		if len(items) > 0 {
			bestItem := items[0]
			equippedItem := GetMercEquippedItem(ctx.Data.Inventory, loc)
			if equippedItem.UnitID == 0 || equippedItem.UnitID != bestItem.UnitID {
				ctx.Logger.Debug(fmt.Sprintf("Mercenary equipment unstable at %s. Best item is %s, but equipped is %s", loc, bestItem.Name, equippedItem.Name))
				isStable = false
			}
		}
	}

	return isStable
}

// isEquippable checks if an item meets the requirements for the given unit (player or NPC)
func isEquippable(i data.Item, target item.LocationType) bool {
	ctx := context.Get()

	bodyLoc := i.Desc().GetType().BodyLocs
	if len(bodyLoc) == 0 {
		return false
	}

	var str, dex, lvl int
	if target == item.LocationEquipped {
		str = ctx.Data.PlayerUnit.Stats[stat.Strength].Value
		dex = ctx.Data.PlayerUnit.Stats[stat.Dexterity].Value
		lvl = ctx.Data.PlayerUnit.Stats[stat.Level].Value
	} else if target == item.LocationMercenary {
		for _, m := range ctx.Data.Monsters {
			if m.IsMerc() {
				str = m.Stats[stat.Strength]
				dex = m.Stats[stat.Dexterity]
				lvl = m.Stats[stat.Level]
			}
		}
	}

	isQuestItem := slices.Contains(questItems, i.Name)

	for class, items := range classItems {
		if ctx.Data.PlayerUnit.Class != class && slices.Contains(items, i.Desc().Type) {
			return false
		}
	}

	isBowOrXbow := i.Desc().Type == "bow" || i.Desc().Type == "xbow" || i.Desc().Type == "bowq" || i.Desc().Type == "xbowq"
	isAmazon := ctx.Data.PlayerUnit.Class == data.Amazon

	// New rule: disallow 2-handed weapons for player level > 11
	if _, isTwoHanded := i.FindStat(stat.TwoHandedMinDamage, 0); isTwoHanded {
		if target == item.LocationEquipped && ctx.Data.PlayerUnit.Stats[stat.Level].Value > 11 {
			return false
		}
	}

	if target == item.LocationEquipped && isBowOrXbow && !isAmazon {
		return false
	}

	return i.Identified &&
		str >= i.Desc().RequiredStrength &&
		dex >= i.Desc().RequiredDexterity &&
		lvl >= i.LevelReq &&
		!isQuestItem
}

func isValidLocation(i data.Item, bodyLoc item.LocationType, target item.LocationType) bool {
	ctx := context.Get()
	class := ctx.Data.PlayerUnit.Class
	itemType := i.Desc().Type
	isShield := slices.Contains(shieldTypes, string(itemType))

	if target == item.LocationMercenary {
		if slices.Contains(mercBodyLocs, bodyLoc) {
			if bodyLoc == item.LocLeftArm {
				if isAct2MercenaryPresent(npc.Guard) {
					return itemType == "spea" || itemType == "pole" || itemType == "jave"
				} else {
					return itemType == "bow"
				}
			}
			return true
		}
		return false
	}

	if target == item.LocationEquipped {
		if isShield {
			return bodyLoc == item.LocRightArm
		}

		if bodyLoc != item.LocRightArm {
			return true
		}

		switch class {
		case data.Barbarian:
			_, isOneHanded := i.FindStat(stat.MaxDamage, 0)
			_, isTwoHanded := i.FindStat(stat.TwoHandedMaxDamage, 0)
			return isOneHanded || (isTwoHanded && itemType == "swor")

		case data.Assassin:
			isClaws := itemType == "h2h" || itemType == "h2h2"

			if isClaws && bodyLoc == item.LocRightArm {
				for _, equippedItem := range ctx.Data.Inventory.ByLocation(item.LocationEquipped) {
					if equippedItem.Location.BodyLocation == item.LocLeftArm {
						return equippedItem.Desc().Type == "h2h" || equippedItem.Desc().Type == "h2h2"
					}
				}
				return false
			}
			return isClaws
		default:
			return false
		}
	}

	return false
}

// isAct2MercenaryPresent checks for the existence of an Act 2 mercenary
func isAct2MercenaryPresent(mercName npc.ID) bool {
	ctx := context.Get()
	for _, monster := range ctx.Data.Monsters {
		if monster.IsMerc() && monster.Name == mercName {
			ctx.Logger.Debug(fmt.Sprintf("Mercenary of type %v is already present.", mercName))
			return true
		}
	}
	return false
}

// evaluateItems processes items for either player or merc
func evaluateItems(items []data.Item, target item.LocationType, scoreFunc func(data.Item) map[item.LocationType]float64) map[item.LocationType][]data.Item {
	ctx := context.Get()
	itemsByLoc := make(map[item.LocationType][]data.Item)
	itemScores := make(map[data.UnitID]map[item.LocationType]float64)

	for _, itm := range items {

		if !isEquippable(itm, target) {
			continue
		}

		if itm.Desc().Name == "Bolts" || itm.Desc().Name == "Arrows" || itm.Desc().Type == "thro" || itm.Desc().Type == "thrq" || itm.Desc().Type == "tkni" || itm.Desc().Type == "taxe" || itm.Desc().Type == "tpot" {
			continue
		}

		bodyLocScores := scoreFunc(itm)

		if len(bodyLocScores) > 0 {
			if _, exists := itemScores[itm.UnitID]; !exists {
				itemScores[itm.UnitID] = make(map[item.LocationType]float64)
			}

			for bodyLoc, score := range bodyLocScores {
				isValid := isValidLocation(itm, bodyLoc, target)

				if isValid {
					itemScores[itm.UnitID][bodyLoc] = score
					itemsByLoc[bodyLoc] = append(itemsByLoc[bodyLoc], itm)
				}
			}
		}
	}

	for loc := range itemsByLoc {
		sort.Slice(itemsByLoc[loc], func(i, j int) bool {
			scoreI := itemScores[itemsByLoc[loc][i].UnitID][loc]
			scoreJ := itemScores[itemsByLoc[loc][j].UnitID][loc]
			return scoreI > scoreJ
		})

		ctx.Logger.Debug(fmt.Sprintf("*** Sorted items for %s ***", loc))
		for i, itm := range itemsByLoc[loc] {
			score := itemScores[itm.UnitID][loc]
			ctx.Logger.Debug(fmt.Sprintf("%d. %s (Score: %.1f)", i+1, itm.IdentifiedName, score))
		}
		ctx.Logger.Debug("**********************************")
	}

	if target == item.LocationEquipped {
		class := ctx.Data.PlayerUnit.Class

		if items, ok := itemsByLoc[item.LocLeftArm]; ok && len(items) > 0 {
			if _, found := items[0].FindStat(stat.TwoHandedMinDamage, 0); found {
				if class == data.Barbarian && items[0].Desc().Type == "swor" {
				} else {
					var bestComboScore float64
					for _, itm := range items {
						if _, isTwoHanded := itm.FindStat(stat.TwoHandedMinDamage, 0); !isTwoHanded {
							if score, exists := itemScores[itm.UnitID]["left_arm"]; exists {
								ctx.Logger.Debug(fmt.Sprintf("Best one-handed weapon score: %.1f", score))
								bestComboScore = score
								break
							}
						}
					}

					if rightArmItems, ok := itemsByLoc[item.LocRightArm]; ok && len(rightArmItems) > 0 {
						if score, exists := itemScores[rightArmItems[0].UnitID][item.LocRightArm]; exists {
							ctx.Logger.Debug(fmt.Sprintf("Best shield score: %.1f", score))
							bestComboScore += score
							ctx.Logger.Debug(fmt.Sprintf("Best one-hand + shield combo score: %.1f", bestComboScore))
						}
					}

					if twoHandedScore, exists := itemScores[items[0].UnitID][item.LocLeftArm]; exists && bestComboScore >= twoHandedScore {
						ctx.Logger.Debug(fmt.Sprintf("Removing two-handed weapon: %s", items[0].Name))
						itemsByLoc[item.LocLeftArm] = itemsByLoc[item.LocLeftArm][1:]
					}
				}
			}
		}
	}

	return itemsByLoc
}

// equipBestItems equips the highest scoring items for each location, with retries
func equipBestItems(itemsByLoc map[item.LocationType][]data.Item, target item.LocationType) error {
	ctx := context.Get()

	equippedItems := make(map[data.UnitID]bool)

	for loc, items := range itemsByLoc {
		if len(items) == 0 {
			continue
		}

		isBestItemEquipped := false
		currentlyEquipped := GetEquippedItem(ctx.Data.Inventory, loc)
		if currentlyEquipped.UnitID != 0 && items[0].UnitID == currentlyEquipped.UnitID {
			isBestItemEquipped = true
		}

		if isBestItemEquipped {
			ctx.Logger.Debug(fmt.Sprintf("Best item %s for %s is already equipped. Skipping.", items[0].Name, loc))
			continue
		}

		// Flag to track if at least one item was successfully equipped for this location
		itemEquippedForLoc := false

		for _, itm := range items {

			if itm.Location.LocationType == target {
				break
			}

			if equippedItems[itm.UnitID] {
				ctx.Logger.Debug(fmt.Sprintf("Skipping %s for %s as it was already equipped elsewhere", itm.Name, loc))
				continue
			}

			if (itm.Location.LocationType == item.LocationMercenary && target == item.LocationEquipped) || (itm.Location.LocationType == item.LocationEquipped && target == item.LocationMercenary) {
				continue
			}

			var equipErr error
			for i := 0; i < MaxRetries; i++ {
				ctx.Logger.Debug(fmt.Sprintf("Attempting to equip %s to %s (Attempt %d/%d)", itm.Name, loc, i+1, MaxRetries))
				equipErr = equip(itm, loc, target)
				if equipErr == nil {
					ctx.Logger.Debug(fmt.Sprintf("Successfully equipped %s to %s", itm.Name, loc))
					itemEquippedForLoc = true
					break
				}
				ctx.Logger.Warn(fmt.Sprintf("Failed to equip %s, retrying...", itm.Name))
				time.Sleep(1 * time.Second)
			}

			if equipErr != nil {
				// NEW: Handle specific error for not enough space
				if errors.Is(equipErr, ErrNotEnoughSpace) {
					ctx.Logger.Info("Not enough inventory space to equip. Trying to sell junk.")

					// Create a temporary lock config that protects the item we want to equip
					tempLock := make([][]int, len(ctx.CharacterCfg.Inventory.InventoryLock))
					for i := range ctx.CharacterCfg.Inventory.InventoryLock {
						tempLock[i] = make([]int, len(ctx.CharacterCfg.Inventory.InventoryLock[i]))
						copy(tempLock[i], ctx.CharacterCfg.Inventory.InventoryLock[i])
					}

					// Lock the new item
					if itm.Location.LocationType == item.LocationInventory {
						w, h := itm.Desc().InventoryWidth, itm.Desc().InventoryHeight
						for j := 0; j < h; j++ {
							for i := 0; i < w; i++ {
								if itm.Position.Y+j < 4 && itm.Position.X+i < 10 {
									tempLock[itm.Position.Y+j][itm.Position.X+i] = 0 // Lock this slot
								}
							}
						}
					}

					if sellErr := VendorRefill(false, true, tempLock); sellErr != nil {
						return fmt.Errorf("failed to sell junk to make space: %w", sellErr)
					}
					// Don't mark as equipped, let the main loop re-evaluate
					continue
				}

				// Original fallback for other errors
				ctx.Logger.Error(fmt.Sprintf("Failed to equip %s after %d attempts: %v. Considering it junk.", itm.Name, MaxRetries, equipErr))
				continue
			}

			// If we successfully equipped an item, we can break out of the inner loop and move to the next location
			if itemEquippedForLoc {
				equippedItems[itm.UnitID] = true
				break
			}
		}

		// If after checking all items for a location, none could be equipped, return an error
		if !itemEquippedForLoc {
			return fmt.Errorf("failed to equip any item for location %s", loc)
		}
	}

	return nil
}

// passing in bodyloc as a parameter cos rings have 2 locations
func equip(itm data.Item, bodyloc item.LocationType, target item.LocationType) error {
	ctx := context.Get()
	ctx.SetLastAction("Equip")

	// Ensure all menus are closed when the function exits
	defer step.CloseAllMenus()

	if itm.Location.LocationType == item.LocationStash || itm.Location.LocationType == item.LocationSharedStash {
		OpenStash()
		utils.Sleep(EquipDelayMS)
		tab := 1
		if itm.Location.LocationType == item.LocationSharedStash {
			tab = itm.Location.Page + 1
		}
		SwitchStashTab(tab)

		ctx.HID.ClickWithModifier(game.LeftButton, ui.GetScreenCoordsForItem(itm).X, ui.GetScreenCoordsForItem(itm).Y, game.CtrlKey)
		utils.Sleep(EquipDelayMS)

		// We need to refresh data and find the item in inventory now
		*ctx.Data = ctx.GameReader.GetData()
		var found bool
		for _, updatedItem := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
			if updatedItem.UnitID == itm.UnitID {
				itm = updatedItem
				found = true
				break
			}
		}
		if !found {
			return fmt.Errorf("item %s not found in inventory after moving from stash", itm.Name)
		}
		step.CloseAllMenus()
	}

	for !ctx.Data.OpenMenus.Inventory {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.Sleep(EquipDelayMS)
	}
	if target == item.LocationMercenary {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.MercenaryScreen)
		utils.Sleep(EquipDelayMS)
	}

	itemToEquip := FindItemByUnitID(ctx.Data.Inventory, itm.UnitID)
	if itemToEquip.UnitID == 0 {
		return fmt.Errorf("item disappeared from inventory before equipping")
	}
	itemCoords := ui.GetScreenCoordsForItem(itemToEquip)

	if target == item.LocationMercenary {
		ctx.HID.ClickWithModifier(game.LeftButton, itemCoords.X, itemCoords.Y, game.CtrlKey)
	} else {
		// NEW: Check for inventory space before unequipping anything
		currentlyEquipped := GetEquippedItem(ctx.Data.Inventory, bodyloc)
		if currentlyEquipped.UnitID != 0 {
			if _, found := findInventorySpace(currentlyEquipped); !found {
				return ErrNotEnoughSpace
			}

			// Unequip logic for specific slots (can be improved, but kept from original)
			var slotCoords data.Position
			if ctx.Data.LegacyGraphics {
				switch bodyloc {
				case item.LocRightRing:
					slotCoords = data.Position{X: ui.EquipRRinClassicX, Y: ui.EquipRRinClassicY}
				case item.LocRightArm:
					slotCoords = data.Position{X: ui.EquipRArmClassicX, Y: ui.EquipRArmClassicY}
				}
			} else {
				switch bodyloc {
				case item.LocRightRing:
					slotCoords = data.Position{X: ui.EquipRRinX, Y: ui.EquipRRinY}
				case item.LocRightArm:
					slotCoords = data.Position{X: ui.EquipRArmX, Y: ui.EquipRArmY}
				}
			}

			if slotCoords.X > 0 {
				ctx.HID.ClickWithModifier(game.LeftButton, slotCoords.X, slotCoords.Y, game.ShiftKey)
				utils.Sleep(EquipDelayMS)
			}
		}

		ctx.Logger.Debug(fmt.Sprintf("Equipping %s at %v to %s using hotkeys", itm.Name, itemCoords, bodyloc))
		ctx.HID.ClickWithModifier(game.LeftButton, itemCoords.X, itemCoords.Y, game.ShiftKey)
	}

	utils.Sleep(500)
	*ctx.Data = ctx.GameReader.GetData()

	itemEquipped := false
	for _, inPlace := range ctx.Data.Inventory.ByLocation(target) {
		if itm.UnitID == inPlace.UnitID && inPlace.Location.BodyLocation == bodyloc {
			itemEquipped = true
			break
		}
	}

	if itemEquipped {
		return nil
	}

	return fmt.Errorf("failed to equip %s to %s", itm.Name, target)
}

func FindItemByUnitID(inventory data.Inventory, unitID data.UnitID) data.Item {
	for _, itm := range inventory.AllItems {
		if itm.UnitID == unitID {
			return itm
		}
	}
	return data.Item{} // Return an empty item if not found
}

// findInventorySpace finds the top-left grid coordinates for a free spot in the inventory.
func findInventorySpace(itm data.Item) (data.Position, bool) {
	ctx := context.Get()
	inventory := ctx.Data.Inventory.ByLocation(item.LocationInventory)
	lockConfig := ctx.CharacterCfg.Inventory.InventoryLock

	// Create a grid representing the inventory, considering items and locked slots
	occupied := [4][10]bool{}

	// Mark all slots occupied by items
	for _, i := range inventory {
		for y := 0; y < i.Desc().InventoryHeight; y++ {
			for x := 0; x < i.Desc().InventoryWidth; x++ {
				if i.Position.Y+y < 4 && i.Position.X+x < 10 {
					occupied[i.Position.Y+y][i.Position.X+x] = true
				}
			}
		}
	}

	// Mark all slots that are locked in the configuration (0 = locked)
	for y, row := range lockConfig {
		if y < 4 {
			for x, cell := range row {
				if x < 10 && cell == 0 {
					occupied[y][x] = true
				}
			}
		}
	}

	// Get the item's dimensions
	w := itm.Desc().InventoryWidth
	h := itm.Desc().InventoryHeight

	// Find a free spot and return its coordinates
	for y := 0; y <= 4-h; y++ {
		for x := 0; x <= 10-w; x++ {
			fits := true
			for j := 0; j < h; j++ {
				for i := 0; i < w; i++ {
					if occupied[y+j][x+i] {
						fits = false
						break
					}
				}
				if !fits {
					break
				}
			}
			if fits {
				// Return the top-left inventory grid position
				return data.Position{X: x, Y: y}, true
			}
		}
	}

	return data.Position{}, false
}

// GetEquippedItem is a new helper function to search for the currently equipped item in a specific location
func GetEquippedItem(inventory data.Inventory, loc item.LocationType) data.Item {
	for _, itm := range inventory.ByLocation(item.LocationEquipped) {
		if itm.Location.BodyLocation == loc {
			return itm
		}
	}
	return data.Item{}
}

// GetMercEquippedItem is a new helper function for the merc
func GetMercEquippedItem(inventory data.Inventory, loc item.LocationType) data.Item {
	for _, itm := range inventory.ByLocation(item.LocationMercenary) {
		if itm.Location.BodyLocation == loc {
			return itm
		}
	}
	return data.Item{}
}