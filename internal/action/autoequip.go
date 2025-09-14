package action

import (
	"errors"
	"fmt"
	"slices"
	"sort"

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
	ErrFailedToEquip  = errors.New("failed to equip item")
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
		"KhalimsWill",
	}
)

// AutoEquip evaluates and equips items for both player and mercenary
func AutoEquip() error {
	ctx := context.Get()
	for { // Use an infinite loop that we can break from
		ctx.Logger.Debug("Evaluating items for equip...")
		locations := []item.LocationType{
			item.LocationStash,
			item.LocationInventory,
			item.LocationEquipped,
			item.LocationMercenary,
		}

		if ctx.CharacterCfg.Game.Leveling.AutoEquipFromSharedStash {
			locations = append(locations, item.LocationSharedStash)
		}

		allItems := ctx.Data.Inventory.ByLocation(locations...)

		// Player
		// Create a new list of items for the player, EXCLUDING mercenary's equipped items.
		playerEvalItems := make([]data.Item, 0)
		for _, itm := range allItems {
			if itm.Location.LocationType != item.LocationMercenary {
				playerEvalItems = append(playerEvalItems, itm)
			}
		}
		playerItems := evaluateItems(playerEvalItems, item.LocationEquipped, PlayerScore)
		playerChanged, err := equipBestItems(playerItems, item.LocationEquipped)
		if err != nil {
			ctx.Logger.Error(fmt.Sprintf("Player equip error: %v. Continuing...", err))
		}

		// Mercenary
		// We need to refresh data after player equip, as it might have changed inventory
		if playerChanged {
			*ctx.Data = ctx.GameReader.GetData()
			allItems = ctx.Data.Inventory.ByLocation(locations...)
		}

		mercChanged := false
		if ctx.Data.MercHPPercent() > 0 {
			// Create a new list of items for the merc, EXCLUDING player's equipped items.
			mercEvalItems := make([]data.Item, 0)
			for _, itm := range allItems {
				if itm.Location.LocationType != item.LocationEquipped {
					mercEvalItems = append(mercEvalItems, itm)
				}
			}

			// Use this new filtered list for the mercenary evaluation.
			mercItems := evaluateItems(mercEvalItems, item.LocationMercenary, MercScore)
			mercChanged, err = equipBestItems(mercItems, item.LocationMercenary)
			if err != nil {
				ctx.Logger.Error(fmt.Sprintf("Mercenary equip error: %v. Continuing...", err))
			}
		}

		if !playerChanged && !mercChanged {
			ctx.Logger.Debug("Equipment is stable, no changes made.")
			return nil
		}

		// If something changed, let's refresh data and loop again to ensure stability
		*ctx.Data = ctx.GameReader.GetData()
		ctx.Logger.Debug("Equipment changed, re-evaluating for stability...")
	}
}

// isEquippable checks if an item can be equipped, considering the stats of the item that would be unequipped.
// It requires the specific body location to perform an accurate stat check.
func isEquippable(newItem data.Item, bodyloc item.LocationType, target item.LocationType) bool {
	ctx := context.Get()

	// General item property checks
	if len(newItem.Desc().GetType().BodyLocs) == 0 {
		return false
	}
	if !newItem.Identified {
		return false
	}
	isQuestItem := slices.Contains(questItems, newItem.Name)
	if isQuestItem {
		return false
	}

	if _, isTwoHanded := newItem.FindStat(stat.TwoHandedMinDamage, 0); isTwoHanded {
		// We need to fetch the level stat safely.
		playerLevel := 0
		if lvl, found := ctx.Data.PlayerUnit.FindStat(stat.Level, 0); found {
			playerLevel = lvl.Value
		}

		if target == item.LocationEquipped && playerLevel > 5 {
			return false
		}
	}

	// Class specific item type checks
	for class, items := range classItems {
		if ctx.Data.PlayerUnit.Class != class && slices.Contains(items, newItem.Desc().Type) {
			return false
		}
	}
	isBowOrXbow := newItem.Desc().Type == "bow" || newItem.Desc().Type == "xbow" || newItem.Desc().Type == "bowq" || newItem.Desc().Type == "xbowq"
	isAmazon := ctx.Data.PlayerUnit.Class == data.Amazon
	if target == item.LocationEquipped && isBowOrXbow && !isAmazon {
		return false
	}

	// Main Requirement Check (Level, Strength, Dexterity)
	if target == item.LocationEquipped {
		var playerLevel int
		if lvl, found := ctx.Data.PlayerUnit.FindStat(stat.Level, 0); found {
			playerLevel = lvl.Value
		}

		itemLevelReq := 0
		if lvlReqStat, found := newItem.FindStat(stat.LevelRequire, 0); found {
			itemLevelReq = lvlReqStat.Value
		}

		// Explicitly log the level comparison
		if playerLevel < itemLevelReq {
			return false
		}

		// Now check stats, considering the item that will be unequipped
		baseStr := ctx.Data.PlayerUnit.Stats[stat.Strength].Value
		baseDex := ctx.Data.PlayerUnit.Stats[stat.Dexterity].Value

		currentlyEquipped := GetEquippedItem(ctx.Data.Inventory, bodyloc)
		if currentlyEquipped.UnitID != 0 {
			if strBonus, found := currentlyEquipped.FindStat(stat.Strength, 0); found {
				baseStr -= strBonus.Value
			}
			if dexBonus, found := currentlyEquipped.FindStat(stat.Dexterity, 0); found {
				baseDex -= dexBonus.Value
			}
		}

		if baseStr < newItem.Desc().RequiredStrength || baseDex < newItem.Desc().RequiredDexterity {
			return false
		}
	}

	if target == item.LocationMercenary {
		var mercStr, mercDex, mercLvl int
		for _, m := range ctx.Data.Monsters {
			if m.IsMerc() {
				mercStr = m.Stats[stat.Strength]
				mercDex = m.Stats[stat.Dexterity]
				mercLvl = m.Stats[stat.Level]
			}
		}

		itemLevelReq := 0
		if lvlReqStat, found := newItem.FindStat(stat.LevelRequire, 0); found {
			itemLevelReq = lvlReqStat.Value
		}
		if mercLvl < itemLevelReq {
			return false
		}

		if mercStr < newItem.Desc().RequiredStrength || mercDex < newItem.Desc().RequiredDexterity {
			return false
		}
	}

	return true
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
		// Exclude Keys from being equipped
		if itm.Name == item.Key {
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
				if !isEquippable(itm, bodyLoc, target) {
					continue
				}

				if !isValidLocation(itm, bodyLoc, target) {
					continue
				}

				itemScores[itm.UnitID][bodyLoc] = score
				itemsByLoc[bodyLoc] = append(itemsByLoc[bodyLoc], itm)
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

	// "Best Combo" logic for Two-Handed Weapons
	if target == item.LocationEquipped {
		class := ctx.Data.PlayerUnit.Class

		if items, ok := itemsByLoc[item.LocLeftArm]; ok && len(items) > 0 {
			if _, found := items[0].FindStat(stat.TwoHandedMinDamage, 0); found {
				if class != data.Barbarian || items[0].Desc().Type != "swor" {
					var bestComboScore float64
					for _, itm := range items {
						if _, isTwoHanded := itm.FindStat(stat.TwoHandedMinDamage, 0); !isTwoHanded {
							if score, exists := itemScores[itm.UnitID][item.LocLeftArm]; exists {
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
						ctx.Logger.Debug(fmt.Sprintf("Removing two-handed weapon: %s", items[0].IdentifiedName))
						itemsByLoc[item.LocLeftArm] = itemsByLoc[item.LocLeftArm][1:]
					}
				}
			}
		}
	}

	return itemsByLoc
}

// equipBestItems tries to equip the best items, returns true if any item was changed
func equipBestItems(itemsByLoc map[item.LocationType][]data.Item, target item.LocationType) (bool, error) {
	ctx := context.Get()
	equippedSomething := false

	for loc, items := range itemsByLoc {
		// Find the best item for this slot that is not already equipped in ANOTHER slot.
		var bestCandidate data.Item
		foundCandidate := false
		for _, itm := range items { // Changed "item" to "itm" here
			// A valid candidate is an item that is not equipped, OR is already equipped in the current slot we are checking.
			if itm.Location.LocationType != item.LocationEquipped || itm.Location.BodyLocation == loc { // And here
				bestCandidate = itm // And here
				foundCandidate = true
				break
			}
		}

		// If no suitable item was found (e.g., all good items are equipped in other slots)
		if !foundCandidate {
			continue
		}

		// Check if the best candidate is already equipped in the current slot
		var currentlyEquipped data.Item
		if target == item.LocationEquipped {
			currentlyEquipped = GetEquippedItem(ctx.Data.Inventory, loc)
		} else {
			currentlyEquipped = GetMercEquippedItem(ctx.Data.Inventory, loc)
		}

		if currentlyEquipped.UnitID != 0 && bestCandidate.UnitID == currentlyEquipped.UnitID {
			continue // Already equipped the best item
		}

		// Attempting to equip the best item
		ctx.Logger.Info(fmt.Sprintf("Attempting to equip %s to %s", bestCandidate.IdentifiedName, loc))
		err := equip(bestCandidate, loc, target)
		if err == nil {
			ctx.Logger.Info(fmt.Sprintf("Successfully equipped %s to %s", bestCandidate.IdentifiedName, loc))
			equippedSomething = true
			*ctx.Data = ctx.GameReader.GetData() // Refresh data after a successful equip
			continue                             // Move to the next location
		}

		// Handle specific errors
		if errors.Is(err, ErrNotEnoughSpace) {
			ctx.Logger.Info("Not enough inventory space to equip. Trying to sell junk.")
			DrinkAllPotionsInInventory()
			// Create a temporary lock config that protects the item we want to equip
			tempLock := make([][]int, len(ctx.CharacterCfg.Inventory.InventoryLock))
			for i := range ctx.CharacterCfg.Inventory.InventoryLock {
				tempLock[i] = make([]int, len(ctx.CharacterCfg.Inventory.InventoryLock[i]))
				copy(tempLock[i], ctx.CharacterCfg.Inventory.InventoryLock[i])
			}

			// Lock the new item
			if bestCandidate.Location.LocationType == item.LocationInventory {
				w, h := bestCandidate.Desc().InventoryWidth, bestCandidate.Desc().InventoryHeight
				for j := 0; j < h; j++ {
					for i := 0; i < w; i++ {
						if bestCandidate.Position.Y+j < 4 && bestCandidate.Position.X+i < 10 {
							tempLock[bestCandidate.Position.Y+j][bestCandidate.Position.X+i] = 0 // Lock this slot
						}
					}
				}
			}

			if sellErr := VendorRefill(false, true, tempLock); sellErr != nil {
				return false, fmt.Errorf("failed to sell junk to make space: %w", sellErr)
			}
			equippedSomething = true // We made a change (selling junk), so we should re-evaluate
			*ctx.Data = ctx.GameReader.GetData()
			continue
		}

		// For other errors, log it and continue to the next item slot
		ctx.Logger.Error(fmt.Sprintf("Failed to equip %s to %s: %v", bestCandidate.IdentifiedName, loc, err))
	}

	return equippedSomething, nil
}

func getBodyLocationScreenCoords(bodyloc item.LocationType) (data.Position, error) {
	ctx := context.Get()
	if ctx.Data.LegacyGraphics {
		switch bodyloc {
		case item.LocHead:
			return data.Position{X: ui.EquipHeadClassicX, Y: ui.EquipHeadClassicY}, nil
		case item.LocNeck:
			return data.Position{X: ui.EquipNeckClassicX, Y: ui.EquipNeckClassicY}, nil
		case item.LocLeftArm:
			return data.Position{X: ui.EquipLArmClassicX, Y: ui.EquipLArmClassicY}, nil
		case item.LocRightArm:
			return data.Position{X: ui.EquipRArmClassicX, Y: ui.EquipRArmClassicY}, nil
		case item.LocTorso:
			return data.Position{X: ui.EquipTorsClassicX, Y: ui.EquipTorsClassicY}, nil
		case item.LocBelt:
			return data.Position{X: ui.EquipBeltClassicX, Y: ui.EquipBeltClassicY}, nil
		case item.LocGloves:
			return data.Position{X: ui.EquipGlovClassicX, Y: ui.EquipGlovClassicY}, nil
		case item.LocFeet:
			return data.Position{X: ui.EquipFeetClassicX, Y: ui.EquipFeetClassicY}, nil
		case item.LocLeftRing:
			return data.Position{X: ui.EquipLRinClassicX, Y: ui.EquipLRinClassicY}, nil
		case item.LocRightRing:
			return data.Position{X: ui.EquipRRinClassicX, Y: ui.EquipRRinClassicY}, nil
		default:
			return data.Position{}, fmt.Errorf("legacy coordinates for %s not defined.", bodyloc)
		}
	}
	switch bodyloc {
	case item.LocHead:
		return data.Position{X: ui.EquipHeadX, Y: ui.EquipHeadY}, nil
	case item.LocNeck:
		return data.Position{X: ui.EquipNeckX, Y: ui.EquipNeckY}, nil
	case item.LocLeftArm:
		return data.Position{X: ui.EquipLArmX, Y: ui.EquipLArmY}, nil
	case item.LocRightArm:
		return data.Position{X: ui.EquipRArmX, Y: ui.EquipRArmY}, nil
	case item.LocTorso:
		return data.Position{X: ui.EquipTorsX, Y: ui.EquipTorsY}, nil
	case item.LocBelt:
		return data.Position{X: ui.EquipBeltX, Y: ui.EquipBeltY}, nil
	case item.LocGloves:
		return data.Position{X: ui.EquipGlovX, Y: ui.EquipGlovY}, nil
	case item.LocFeet:
		return data.Position{X: ui.EquipFeetX, Y: ui.EquipFeetY}, nil
	case item.LocLeftRing:
		return data.Position{X: ui.EquipLRinX, Y: ui.EquipLRinY}, nil
	case item.LocRightRing:
		return data.Position{X: ui.EquipRRinX, Y: ui.EquipRRinY}, nil
	default:
		return data.Position{}, fmt.Errorf("coordinates for slot %s not defined. ", bodyloc)
	}
}

func equipBestRings(itemsByLoc map[item.LocationType][]data.Item) (bool, error) {
	ctx := context.Get()

	allRingsMap := make(map[data.UnitID]data.Item)
	for _, ring := range itemsByLoc[item.LocLeftRing] {
		allRingsMap[ring.UnitID] = ring
	}
	for _, ring := range itemsByLoc[item.LocRightRing] {
		allRingsMap[ring.UnitID] = ring
	}

	var allRings []data.Item
	for _, ring := range allRingsMap {
		allRings = append(allRings, ring)
	}

	sort.Slice(allRings, func(i, j int) bool {

		scoreI := PlayerScore(allRings[i])[item.LocLeftRing]
		scoreJ := PlayerScore(allRings[j])[item.LocLeftRing]
		return scoreI > scoreJ
	})

	if len(allRings) == 0 {
		return false, nil
	}

	bestRing := allRings[0]
	var secondBestRing data.Item
	if len(allRings) > 1 {
		secondBestRing = allRings[1]
	}

	leftEquipped := GetEquippedItem(ctx.Data.Inventory, item.LocLeftRing)
	rightEquipped := GetEquippedItem(ctx.Data.Inventory, item.LocRightRing)

	equippedRings := []data.Item{leftEquipped, rightEquipped}
	idealIDs := map[data.UnitID]bool{
		bestRing.UnitID: true,
	}
	if secondBestRing.UnitID != 0 {
		idealIDs[secondBestRing.UnitID] = true
	}

	var ringToReplace data.Item
	for _, equipped := range equippedRings {
		if equipped.UnitID != 0 && !idealIDs[equipped.UnitID] {
			ringToReplace = equipped
			break
		}
	}

	if ringToReplace.UnitID != 0 {
		var replacementRing data.Item
		if bestRing.UnitID != leftEquipped.UnitID && bestRing.UnitID != rightEquipped.UnitID {
			replacementRing = bestRing
		} else if secondBestRing.UnitID != 0 && (secondBestRing.UnitID != leftEquipped.UnitID && secondBestRing.UnitID != rightEquipped.UnitID) {
			replacementRing = secondBestRing
		}

		if replacementRing.UnitID != 0 {
			ctx.Logger.Info(fmt.Sprintf("Replacing ring %s with %s.", ringToReplace.IdentifiedName, replacementRing.IdentifiedName))
			err := equip(replacementRing, ringToReplace.Location.BodyLocation, item.LocationEquipped)
			if err != nil {
				return false, fmt.Errorf("failed to equip ring: %w", err)
			}
			return true, nil
		}
	}

	if leftEquipped.UnitID == 0 {
		if bestRing.UnitID != rightEquipped.UnitID {
			ctx.Logger.Info(fmt.Sprintf("Equipping best ring %s in empty left slot.", bestRing.IdentifiedName))
			if err := equip(bestRing, item.LocLeftRing, item.LocationEquipped); err == nil {
				return true, nil
			}
		}
	}
	if rightEquipped.UnitID == 0 {
		if secondBestRing.UnitID != 0 && secondBestRing.UnitID != leftEquipped.UnitID {
			ctx.Logger.Info(fmt.Sprintf("Equipping second best ring %s in empty right slot.", secondBestRing.IdentifiedName))
			if err := equip(secondBestRing, item.LocRightRing, item.LocationEquipped); err == nil {
				return true, nil
			}
		}
	}

	return false, nil
}

// equip handles the physical process of equipping an item. Returns ErrNotEnoughSpace if it fails.
func equip(itm data.Item, bodyloc item.LocationType, target item.LocationType) error {
	ctx := context.Get()
	ctx.SetLastAction("Equip")
	defer step.CloseAllMenus()

	// Move item from stash to inventory if needed
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
			return fmt.Errorf("item %s not found in inventory after moving from stash", itm.IdentifiedName)
		}
		step.CloseAllMenus()
	}

	// Main retry loop
	for attempt := 0; attempt < 3; attempt++ {
		for !ctx.Data.OpenMenus.Inventory {
			ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
			utils.Sleep(EquipDelayMS)
		}

		if target == item.LocationMercenary {
			ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.MercenaryScreen)
			utils.Sleep(EquipDelayMS)
			ctx.HID.ClickWithModifier(game.LeftButton, ui.GetScreenCoordsForItem(itm).X, ui.GetScreenCoordsForItem(itm).Y, game.CtrlKey)
		} else {
			currentlyEquipped := GetEquippedItem(ctx.Data.Inventory, bodyloc)
			isRingSwap := itm.Desc().Type == "ring" && currentlyEquipped.UnitID != 0

			if isRingSwap {
				if _, found := findInventorySpace(currentlyEquipped); !found {
					return ErrNotEnoughSpace
				}

				ctx.Logger.Info(fmt.Sprintf("Unequipping old ring: %s from %s", currentlyEquipped.IdentifiedName, bodyloc))

				oldRingCoords, err := getBodyLocationScreenCoords(bodyloc)
				if err != nil {
					return err
				}

				ctx.HID.ClickWithModifier(game.LeftButton, oldRingCoords.X, oldRingCoords.Y, game.ShiftKey)
				utils.Sleep(1000)
				*ctx.Data = ctx.GameReader.GetData()

				itemAfterUnequip := GetEquippedItem(ctx.Data.Inventory, bodyloc)
				if itemAfterUnequip.UnitID != 0 {
					ctx.Logger.Warn("Failed to unequip old ring, it is still equipped. Aborting swap.")
					return fmt.Errorf("failed to unequip old ring from %s", bodyloc)
				}

				var newItemInInv data.Item
				var foundInInv bool
				for _, invItem := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
					if invItem.UnitID == itm.UnitID {
						newItemInInv = invItem
						foundInInv = true
						break
					}
				}
				if !foundInInv {
					return fmt.Errorf("new ring %s not found in inventory after unequip", itm.IdentifiedName)
				}

				ctx.Logger.Info(fmt.Sprintf("Equipping new ring: %s", newItemInInv.IdentifiedName))
				newRingCoords := ui.GetScreenCoordsForItem(newItemInInv)
				ctx.HID.ClickWithModifier(game.LeftButton, newRingCoords.X, newRingCoords.Y, game.ShiftKey)

			} else { // Standard logic for all other items
				if currentlyEquipped.UnitID != 0 {
					if _, found := findInventorySpace(currentlyEquipped); !found {
						return ErrNotEnoughSpace
					}
				}
				ctx.HID.ClickWithModifier(game.LeftButton, ui.GetScreenCoordsForItem(itm).X, ui.GetScreenCoordsForItem(itm).Y, game.ShiftKey)
			}
		}

		// Verification loop
		*ctx.Data = ctx.GameReader.GetData()
		var itemEquipped bool
		for i := 0; i < 3; i++ {
			utils.Sleep(800)
			*ctx.Data = ctx.GameReader.GetData()
			for _, inPlace := range ctx.Data.Inventory.ByLocation(target) {
				if inPlace.UnitID == itm.UnitID && inPlace.Location.BodyLocation == bodyloc {
					itemEquipped = true
					break
				}
			}
			if itemEquipped {
				break
			}
		}
		if itemEquipped {
			return nil
		}
		ctx.Logger.Debug(fmt.Sprintf("Equip attempt %d failed, retrying...", attempt+1))
		utils.Sleep(500)
	}
	return fmt.Errorf("verification failed after all attempts to equip %s", itm.IdentifiedName)
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

// UnEquipMercenary stashes all items from the player's inventory, and then unequips the mercenary's head, torso, and arm items and moves them to the player's now-empty inventory.
func UnEquipMercenary() error {
	ctx := context.Get()
	ctx.SetLastAction("UnEquip Mercenary")
	defer step.CloseAllMenus()

	// Step 1: Stash all items from the player's inventory to make space.
	ctx.Logger.Info("Stashing all items from inventory...")
	if err := OpenStash(); err != nil {
		return fmt.Errorf("could not open stash: %w", err)
	}
	if !ctx.Data.OpenMenus.Inventory {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.Sleep(EquipDelayMS)
	}

	// Loop multiple times to ensure all items are stashed.
	for i := 0; i < 3; i++ {
		*ctx.Data = ctx.GameReader.GetData()
		inventoryItems := ctx.Data.Inventory.ByLocation(item.LocationInventory)
		if len(inventoryItems) == 0 {
			break
		}

		ctx.Logger.Info(fmt.Sprintf("Stashing items from inventory, attempt %d/3...", i+1))
		for _, invItem := range inventoryItems {
			// Exclude tomes from being stashed.
			if invItem.Name == "TomeOfTownPortal" || invItem.Name == "TomeOfIdentify" {
				ctx.Logger.Debug(fmt.Sprintf("EXCLUDING: Skipping drop for %s (ID: %d) as per rule.", invItem.Name, invItem.ID))
				continue
			}

			// Find the item's coordinates and perform a ctrl+click to stash it.
			coords := ui.GetScreenCoordsForItem(invItem)
			ctx.HID.ClickWithModifier(game.LeftButton, coords.X, coords.Y, game.CtrlKey)
			utils.Sleep(EquipDelayMS)
		}
	}

	CloseStash()

	// Step 2: UnEquip the mercenary's gear.
	ctx.Logger.Info("Stashing complete. Now unequipping mercenary gear.")

	// Open both the merc screen and player inventory for the transfer to work
	ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.MercenaryScreen)
	utils.Sleep(EquipDelayMS)
	ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
	utils.Sleep(EquipDelayMS)

	// Refresh data to ensure the new menu state is recognized
	*ctx.Data = ctx.GameReader.GetData()

	// Use predefined screen coordinates for the mercenary's gear slots
	var mercGearCoords []data.Position
	if ctx.Data.LegacyGraphics {
		// D2 Classic
		mercGearCoords = []data.Position{
			{X: ui.EquipMercHeadClassicX, Y: ui.EquipMercHeadClassicY},
			{X: ui.EquipMercTorsClassicX, Y: ui.EquipMercTorsClassicY},
			{X: ui.EquipMercLArmClassicX, Y: ui.EquipMercLArmClassicY},
		}
	} else {
		// D2R
		mercGearCoords = []data.Position{
			{X: ui.EquipMercHeadX, Y: ui.EquipMercHeadY},
			{X: ui.EquipMercTorsX, Y: ui.EquipMercTorsY},
			{X: ui.EquipMercLArmX, Y: ui.EquipMercLArmY},
		}
	}

	// Perform the ctrl+click action on each item location
	for _, coords := range mercGearCoords {
		ctx.HID.ClickWithModifier(game.LeftButton, coords.X, coords.Y, game.CtrlKey)
		utils.Sleep(EquipDelayMS)
	}

	return nil
}
