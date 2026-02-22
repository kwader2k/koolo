package action

import (
	"errors"
	"fmt"
	"slices"
	"sort"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
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
		"TheGidbinn",
		"HellforgeHammer",
		"HoradricMalus",
	}
)

func isBarbLevelingCharacter() bool {
	ctx := context.Get()
	return ctx.CharacterCfg.Character.Class == "barb_leveling"
}

// AutoEquip evaluates and equips items for both player and mercenary
func AutoEquip() error {
	ctx := context.Get()

	// skip autoequip for barb leveling during boss town routines
	if ctx.IsBossEquipmentActive {
		ctx.Logger.Debug("Boss equipment is active, skipping AutoEquip to preserve boss-specific equipment")
		return nil
	}

	// Safety mechanism to prevent infinite loops
	maxIterations := 30
	currentIteration := 0

	for { // Use an infinite loop that we can break from
		currentIteration++
		if currentIteration > maxIterations {
			ctx.Logger.Warn("AutoEquip loop exceeded max iterations. Breaking to prevent freeze.")
			return nil
		}

		ctx.Logger.Debug(fmt.Sprintf("Evaluating items for equip... (Iteration %d/%d)", currentIteration, maxIterations))
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
		playerItems, playerScores := evaluateItems(playerEvalItems, item.LocationEquipped, PlayerScore)
		playerChanged, err := equipBestItems(playerItems, playerScores, item.LocationEquipped)
		if err != nil {
			ctx.Logger.Error(fmt.Sprintf("Player equip error: %v. Continuing...", err))
			if errors.Is(err, ErrNotEnoughSpace) {
				return err
			}
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
			mercItems, mercScores := evaluateItems(mercEvalItems, item.LocationMercenary, MercScore)
			mercChanged, err = equipBestItems(mercItems, mercScores, item.LocationMercenary) // Pass mercScores
			if err != nil {
				ctx.Logger.Error(fmt.Sprintf("Mercenary equip error: %v. Continuing...", err))
			}
		}

		if !playerChanged && !mercChanged {
			ctx.Logger.Debug("Equipment is stable, no changes made.")
			ctaChanged, err := equipCTAIfFound(allItems)
			if err != nil {
				ctx.Logger.Error(fmt.Sprintf("CTA equip error: %v", err))
			}
			if ctaChanged {
				*ctx.Data = ctx.GameReader.GetData()
				continue
			}

			return nil
		}

		// If something changed, let's refresh data and loop again to ensure stability
		*ctx.Data = ctx.GameReader.GetData()
		ctx.Logger.Debug("Equipment changed, re-evaluating for stability...")
	}
}

func IsBetterThanEquipped(itm data.Item, forMerc bool, scoreFunc func(data.Item) map[item.LocationType]float64) bool {
	ctx := context.Get()

	bodyLocs := itm.Desc().GetType().BodyLocs
	if len(bodyLocs) == 0 {
		return false
	}

	scores := scoreFunc(itm)

	for loc, itmScore := range scores {
		if !isEquippable(itm, loc, loc) {
			continue
		}

		if !isValidLocation(itm, loc, loc) {
			continue
		}

		var currentlyEquipped data.Item
		if !forMerc {
			currentlyEquipped = GetEquippedItem(ctx.Data.Inventory, loc)
		} else {
			currentlyEquipped = GetMercEquippedItem(ctx.Data.Inventory, loc)
		}

		equippedScore := scoreFunc(currentlyEquipped)

		if itmScore > equippedScore[loc] {
			return true
		}
	}

	return false
}

func equipCTAIfFound(allItems []data.Item) (bool, error) {
	ctx := context.Get()
	var ctaWeapon data.Item
	var spiritShield data.Item
	foundCta := false
	foundSpirit := false

	for _, itm := range allItems {
		if itm.RunewordName == item.RunewordCallToArms {
			if !isAllowedEtherealForPlayer(itm) {
				continue
			}
			ctaWeapon = itm
			foundCta = true
		}
		if itm.RunewordName == item.RunewordSpirit && slices.Contains(shieldTypes, string(itm.Desc().Type)) {
			if itm.Location.LocationType != item.LocationEquipped {
				if !isAllowedEtherealForPlayer(itm) {
					continue
				}
				spiritShield = itm
				foundSpirit = true
			}
		}
	}

	if !foundCta {
		return false, nil
	}

	// Check secondary weapon slot
	ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.SwapWeapons)
	utils.Sleep(EquipDelayMS)
	*ctx.Data = ctx.GameReader.GetData()

	equippedWeapon := GetEquippedItem(ctx.Data.Inventory, item.LocLeftArm)
	equippedShield := GetEquippedItem(ctx.Data.Inventory, item.LocRightArm)
	changed := false

	if equippedWeapon.RunewordName != item.RunewordCallToArms {
		ctx.Logger.Info("Equipping Call to Arms on secondary slot")
		err := equip(ctaWeapon, item.LocLeftArm, item.LocationEquipped)
		if err != nil {
			ctx.Logger.Error(fmt.Sprintf("Failed to equip CTA: %v", err))
		} else {
			changed = true
		}
	}

	if foundSpirit && equippedShield.RunewordName != item.RunewordSpirit {
		// Only equip spirit if CTA is one handed
		if _, isTwoHanded := ctaWeapon.FindStat(stat.TwoHandedMinDamage, 0); !isTwoHanded {
			ctx.Logger.Info("Equipping Spirit on secondary slot")
			err := equip(spiritShield, item.LocRightArm, item.LocationEquipped)
			if err != nil {
				ctx.Logger.Error(fmt.Sprintf("Failed to equip Spirit: %v", err))
			} else {
				changed = true
			}
		}
	}

	// Switch back to primary
	ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.SwapWeapons)
	utils.Sleep(EquipDelayMS)
	*ctx.Data = ctx.GameReader.GetData()

	return changed, nil
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
	if target == item.LocationEquipped && !isAllowedEtherealForPlayer(newItem) {
		return false
	}

	isSorc := ctx.CharacterCfg.Character.Class == "sorceress_leveling"
	isBarbLeveling := isBarbLevelingCharacter()

	if target == item.LocationEquipped && isBarbLeveling {
		if !barblogic(newItem, bodyloc) {
			return false
		}
	}
	// Only apply two-handed checks if this is actually a weapon
	isWeapon := bodyloc == item.LocLeftArm || bodyloc == item.LocRightArm
	if _, isTwoHanded := newItem.FindStat(stat.TwoHandedMinDamage, 0); isTwoHanded && isWeapon {
		// We need to fetch the level stat safely.
		playerLevel := 0
		if lvl, found := ctx.Data.PlayerUnit.FindStat(stat.Level, 0); found {
			playerLevel = lvl.Value
		}

		//Avoid equiping 2 handed unless it's a runeword
		if target == item.LocationEquipped && playerLevel > 5 && !newItem.IsRuneword {
			return false
		}

		if target == item.LocationEquipped && playerLevel >= 24 && isSorc {
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

func isAllowedEtherealForPlayer(itm data.Item) bool {
	if !itm.Ethereal {
		return true
	}
	if _, found := itm.FindStat(stat.Indestructible, 0); found {
		return true
	}
	if _, found := itm.FindStat(stat.ReplenishDurability, 0); found {
		return true
	}
	return false
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
			// barb_leveling shield 31+
			isBarbLeveling := isBarbLevelingCharacter()
			if isBarbLeveling {
				playerLevel := 0
				if lvl, found := ctx.Data.PlayerUnit.FindStat(stat.Level, 0); found {
					playerLevel = lvl.Value
				}
				hasWarCry := ctx.Data.PlayerUnit.Skills[skill.WarCry].Level > 0
				shieldsAllowed := playerLevel >= 31 || hasWarCry
				if shieldsAllowed {
					return false
				}
			}
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
func evaluateItems(items []data.Item, target item.LocationType, scoreFunc func(data.Item) map[item.LocationType]float64) (map[item.LocationType][]data.Item, map[data.UnitID]map[item.LocationType]float64) {
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

		if !itm.Identified && itm.Quality >= item.QualityMagic {
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
			ctx.Logger.Debug(fmt.Sprintf("%d. %s (Score: %.1f)", i+1, getItemNameForScore(itm), score))
		}
		ctx.Logger.Debug("**********************************")
	}

	// "Best Combo" logic for Two-Handed Weapons
	if target == item.LocationEquipped {
		class := ctx.Data.PlayerUnit.Class
		isBarbLeveling := isBarbLevelingCharacter()

		if items, ok := itemsByLoc[item.LocLeftArm]; ok && len(items) > 0 {
			if _, found := items[0].FindStat(stat.TwoHandedMinDamage, 0); found {
				if !isBarbLeveling && (class != data.Barbarian || items[0].Desc().Type != "swor") {
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

					if twoHandedScore, exists := itemScores[items[0].UnitID][item.LocLeftArm]; exists {
						if bestComboScore >= twoHandedScore {
							ctx.Logger.Debug(fmt.Sprintf("Removing two-handed weapon: %s", items[0].IdentifiedName))
							itemsByLoc[item.LocLeftArm] = itemsByLoc[item.LocLeftArm][1:]
						} else {
							ctx.Logger.Debug("Two-handed weapon is better, preventing shield equip.")
							delete(itemsByLoc, item.LocRightArm)
						}
					}
				}
			}
		}
	}

	return itemsByLoc, itemScores
}

// equipBestItems tries to equip the best items, returns true if any item was changed
func equipBestItems(itemsByLoc map[item.LocationType][]data.Item, itemScores map[data.UnitID]map[item.LocationType]float64, target item.LocationType) (bool, error) {
	ctx := context.Get()
	equippedSomething := false

	// special logik for barb leveling
	isBarbLeveling := isBarbLevelingCharacter()
	if isBarbLeveling && target == item.LocationEquipped {
		weaponsChanged, err := equipBestWeapons(itemsByLoc, itemScores)
		if err != nil {
			return false, err
		}
		if weaponsChanged {
			equippedSomething = true
			*ctx.Data = ctx.GameReader.GetData()
			delete(itemsByLoc, item.LocLeftArm)
			delete(itemsByLoc, item.LocRightArm)
		}
	}

	locationOrder := make([]item.LocationType, 0, len(itemsByLoc))
	for loc := range itemsByLoc {
		locationOrder = append(locationOrder, loc)
	}

	for _, loc := range locationOrder {
		items, ok := itemsByLoc[loc]
		if !ok {
			continue
		}
		// Find the best item for this slot that is not already equipped in ANOTHER slot.
		// Use fresh equipped data to avoid stale snapshots picking the same unit twice.
		equippedByID := getEquippedUnitLocations(target)
		var bestCandidate data.Item
		foundCandidate := false
		for _, itm := range items { // Changed "item" to "itm" here
			if itm.InTradeOrStoreScreen {
				continue
			}
			if equippedLoc, equipped := equippedByID[itm.UnitID]; equipped && equippedLoc != loc {
				continue
			}
			// A valid candidate is not equipped, OR is already equipped in the current slot we are checking.
			bestCandidate = itm // And here
			foundCandidate = true
			break
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

		if currentlyEquipped.UnitID != 0 {
			oldScore := itemScores[currentlyEquipped.UnitID][loc]
			newScore := itemScores[bestCandidate.UnitID][loc]

			// If the new item is NOT strictly better, we skip the equip.
			if newScore <= oldScore {
				ctx.Logger.Debug(fmt.Sprintf("Skipping equip of %s to %s: Candidate score (%.2f) is not strictly better than equipped item score (%.2f).",
					bestCandidate.IdentifiedName, loc, newScore, oldScore))
				continue
			}
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

			if sellErr := VendorRefill(VendorRefillOpts{SellJunk: true, BuyConsumables: true, LockConfig: tempLock}); sellErr != nil {
				return false, fmt.Errorf("failed to sell junk to make space: %w", sellErr)
			}
			equippedSomething = true // We made a change (selling junk), so we should re-evaluate
			*ctx.Data = ctx.GameReader.GetData()
			if _, found := findInventorySpace(currentlyEquipped); !found {
				return false, err
			}
			continue
		}

		// For other errors, log it and continue to the next item slot
		ctx.Logger.Error(fmt.Sprintf("Failed to equip %s to %s: %v", bestCandidate.IdentifiedName, loc, err))
	}

	return equippedSomething, nil
}

func getEquippedUnitLocations(target item.LocationType) map[data.UnitID]item.LocationType {
	ctx := context.Get()
	equipped := make(map[data.UnitID]item.LocationType)
	for _, itm := range ctx.Data.Inventory.ByLocation(target) {
		if itm.UnitID == 0 || itm.InTradeOrStoreScreen {
			continue
		}
		equipped[itm.UnitID] = itm.Location.BodyLocation
	}
	return equipped
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
			return data.Position{}, fmt.Errorf("legacy coordinates for %s not defined", bodyloc)
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

// special logik for barb leveling
func equipBestWeapons(itemsByLoc map[item.LocationType][]data.Item, itemScores map[data.UnitID]map[item.LocationType]float64) (bool, error) {
	ctx := context.Get()

	if ctx.Data.ActiveWeaponSlot == 1 {
		return false, nil
	}

	playerLevel := 0
	if lvl, found := ctx.Data.PlayerUnit.FindStat(stat.Level, 0); found {
		playerLevel = lvl.Value
	}
	hasWarCry := ctx.Data.PlayerUnit.Skills[skill.WarCry].Level > 0
	shieldsAllowed := playerLevel >= 31 || hasWarCry

	leftEquipped := GetEquippedItem(ctx.Data.Inventory, item.LocLeftArm)
	rightEquipped := GetEquippedItem(ctx.Data.Inventory, item.LocRightArm)

	if shieldsAllowed {

		var weapons []data.Item
		if leftArmItems, ok := itemsByLoc[item.LocLeftArm]; ok {
			for _, itm := range leftArmItems {
				itemType := itm.Desc().Type
				if !slices.Contains(shieldTypes, string(itemType)) {
					weapons = append(weapons, itm)
				}
			}
		}
		if len(weapons) > 0 {
			sort.Slice(weapons, func(i, j int) bool {
				scoreI := itemScores[weapons[i].UnitID][item.LocLeftArm]
				scoreJ := itemScores[weapons[j].UnitID][item.LocLeftArm]
				return scoreI > scoreJ
			})
			bestWeapon := weapons[0]
			if leftEquipped.UnitID != bestWeapon.UnitID {
				ctx.Logger.Info(fmt.Sprintf("Equipping best weapon %s in mainhand (left arm).", bestWeapon.IdentifiedName))
				if err := equip(bestWeapon, item.LocLeftArm, item.LocationEquipped); err == nil {
					return true, nil
				}
			}
		}
		// Wenn eine Waffe auf right_arm ist, entferne sie (damit Platz für Schild ist)
		if rightEquipped.UnitID != 0 {
			rightItemType := rightEquipped.Desc().Type
			if !slices.Contains(shieldTypes, string(rightItemType)) {
				ctx.Logger.Info(fmt.Sprintf("Removing weapon %s from right arm to make room for shield.", rightEquipped.IdentifiedName))
				if _, found := findInventorySpace(rightEquipped); !found {
					return false, nil
				}
				rightArmCoords, err := getBodyLocationScreenCoords(item.LocRightArm)
				if err != nil {
					return false, err
				}
				if !ctx.Data.OpenMenus.Inventory {
					ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
					utils.Sleep(EquipDelayMS)
				}
				ctx.HID.ClickWithModifier(game.LeftButton, rightArmCoords.X, rightArmCoords.Y, game.ShiftKey)
				utils.Sleep(1000)
				*ctx.Data = ctx.GameReader.GetData()
				itemAfterUnequip := GetEquippedItem(ctx.Data.Inventory, item.LocRightArm)
				if itemAfterUnequip.UnitID != 0 {
					ctx.Logger.Warn("Failed to unequip weapon from right arm.")
					return false, nil
				}
				return true, nil
			}
		}
		return false, nil
	}

	allWeaponsMap := make(map[data.UnitID]data.Item)
	if leftArmItems, ok := itemsByLoc[item.LocLeftArm]; ok {
		for _, weapon := range leftArmItems {
			allWeaponsMap[weapon.UnitID] = weapon
		}
	}

	if !shieldsAllowed {
		if rightArmItems, ok := itemsByLoc[item.LocRightArm]; ok {
			for _, weapon := range rightArmItems {
				allWeaponsMap[weapon.UnitID] = weapon
			}
		}
	}

	var allWeapons []data.Item
	for _, weapon := range allWeaponsMap {
		allWeapons = append(allWeapons, weapon)
	}

	sort.Slice(allWeapons, func(i, j int) bool {
		scoreI := itemScores[allWeapons[i].UnitID][item.LocLeftArm]
		scoreJ := itemScores[allWeapons[j].UnitID][item.LocLeftArm]
		return scoreI > scoreJ
	})

	if len(allWeapons) == 0 {
		return false, nil
	}

	bestWeapon := allWeapons[0]
	var secondBestWeapon data.Item
	if len(allWeapons) > 1 {
		secondBestWeapon = allWeapons[1]
	}

	equippedWeapons := []data.Item{leftEquipped, rightEquipped}
	idealIDs := map[data.UnitID]bool{
		bestWeapon.UnitID: true,
	}
	if secondBestWeapon.UnitID != 0 {
		idealIDs[secondBestWeapon.UnitID] = true
	}

	var weaponToReplace data.Item
	for _, equipped := range equippedWeapons {
		if equipped.UnitID != 0 && !idealIDs[equipped.UnitID] {
			if shieldsAllowed && equipped.Location.BodyLocation == item.LocRightArm {
				rightItemType := equipped.Desc().Type
				if !slices.Contains(shieldTypes, string(rightItemType)) {
					continue
				}
			}
			weaponToReplace = equipped
			break
		}
	}

	if weaponToReplace.UnitID != 0 {
		var replacementWeapon data.Item
		if bestWeapon.UnitID != leftEquipped.UnitID && bestWeapon.UnitID != rightEquipped.UnitID {
			replacementWeapon = bestWeapon
		} else if secondBestWeapon.UnitID != 0 && (secondBestWeapon.UnitID != leftEquipped.UnitID && secondBestWeapon.UnitID != rightEquipped.UnitID) {
			replacementWeapon = secondBestWeapon
		}

		if replacementWeapon.UnitID != 0 {
			if shieldsAllowed && weaponToReplace.Location.BodyLocation == item.LocRightArm {
				rightItemType := weaponToReplace.Desc().Type
				if !slices.Contains(shieldTypes, string(rightItemType)) {
					weaponToReplace = data.Item{}
				}
			}

			if weaponToReplace.UnitID != 0 {
				ctx.Logger.Info(fmt.Sprintf("Replacing weapon %s with %s.", weaponToReplace.IdentifiedName, replacementWeapon.IdentifiedName))
				err := equip(replacementWeapon, weaponToReplace.Location.BodyLocation, item.LocationEquipped)
				if err != nil {
					return false, fmt.Errorf("failed to equip weapon: %w", err)
				}
				return true, nil
			}
		}
	}

	if leftEquipped.UnitID == 0 {
		if bestWeapon.UnitID != rightEquipped.UnitID {
			ctx.Logger.Info(fmt.Sprintf("Equipping best weapon %s in mainhand (left arm).", bestWeapon.IdentifiedName))
			if err := equip(bestWeapon, item.LocLeftArm, item.LocationEquipped); err == nil {
				return true, nil
			}
		}
	}
	if !shieldsAllowed {
		if rightEquipped.UnitID == 0 {
			var offhandWeapon data.Item
			if secondBestWeapon.UnitID != 0 && secondBestWeapon.UnitID != leftEquipped.UnitID {
				offhandWeapon = secondBestWeapon
			} else if bestWeapon.UnitID != leftEquipped.UnitID {
				offhandWeapon = bestWeapon
			}

			if offhandWeapon.UnitID != 0 {
				ctx.Logger.Info(fmt.Sprintf("Equipping weapon %s in offhand (right arm).", offhandWeapon.IdentifiedName))
				if err := equip(offhandWeapon, item.LocRightArm, item.LocationEquipped); err == nil {
					return true, nil
				}
			}
		}
	}

	return false, nil
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

	if target == item.LocationEquipped && !isAllowedEtherealForPlayer(itm) {
		return fmt.Errorf("ethereal item %s is not allowed for player equip", itm.IdentifiedName)
	}

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
		// Also check cursor - item may have gone there if inventory was full
		if !found {
			for _, cursorItem := range ctx.Data.Inventory.ByLocation(item.LocationCursor) {
				if cursorItem.UnitID == itm.UnitID {
					itm = cursorItem
					found = true
					break
				}
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
			isBarbLeveling := isBarbLevelingCharacter()
			isBarbLocRightArm := isBarbLeveling && bodyloc == item.LocRightArm && currentlyEquipped.UnitID != 0

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

			} else if isBarbLocRightArm {
				if _, found := findInventorySpace(currentlyEquipped); !found {
					return ErrNotEnoughSpace
				}

				rightArmCoords, err := getBodyLocationScreenCoords(item.LocRightArm)
				if err != nil {
					return err
				}

				ctx.HID.ClickWithModifier(game.LeftButton, rightArmCoords.X, rightArmCoords.Y, game.ShiftKey)
				utils.Sleep(1000)
				*ctx.Data = ctx.GameReader.GetData()

				itemAfterUnequip := GetEquippedItem(ctx.Data.Inventory, item.LocRightArm)
				if itemAfterUnequip.UnitID != 0 {
					ctx.Logger.Warn("Failed to unequip old item from right arm.")
					return fmt.Errorf("failed to unequip old item from right arm")
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
					return fmt.Errorf("new item %s not found in inventory after unequip", itm.IdentifiedName)
				}

				ctx.Logger.Info(fmt.Sprintf("Equipping new item to right arm: %s", newItemInInv.IdentifiedName))
				newItemCoords := ui.GetScreenCoordsForItem(newItemInInv)
				ctx.HID.ClickWithModifier(game.LeftButton, newItemCoords.X, newItemCoords.Y, game.ShiftKey)

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

// wrapper for barb leveling boss equipment
func IsItemEquippable(newItem data.Item, bodyloc item.LocationType, target item.LocationType) bool {
	return isEquippable(newItem, bodyloc, target)
}

// wrapper for barb leveling boss equipment
func EquipItem(itm data.Item, bodyloc item.LocationType, target item.LocationType) error {
	return equip(itm, bodyloc, target)
}

func GetEquippedItem(inventory data.Inventory, loc item.LocationType) data.Item {
	for _, itm := range inventory.ByLocation(item.LocationEquipped) {
		if itm.Location.BodyLocation == loc && !itm.InTradeOrStoreScreen {
			return itm
		}
	}
	return data.Item{}
}

// GetMercEquippedItem is a new helper function for the merc
func GetMercEquippedItem(inventory data.Inventory, loc item.LocationType) data.Item {
	for _, itm := range inventory.ByLocation(item.LocationMercenary) {
		if itm.Location.BodyLocation == loc && !itm.InTradeOrStoreScreen {
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

// remove shield first run if under 31
func RemoveShield() {
	ctx := context.Get()

	if ctx.CharacterCfg.Character.Class != "barb_leveling" {
		return
	}

	lvl, found := ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
	if !found || lvl.Value >= 31 {
		return
	}

	rightEquipped := GetEquippedItem(ctx.Data.Inventory, item.LocRightArm)
	if rightEquipped.UnitID == 0 || !slices.Contains(shieldTypes, string(rightEquipped.Desc().Type)) {
		return
	}

	ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
	utils.Sleep(500)
	rightArmCoords, _ := getBodyLocationScreenCoords(item.LocRightArm)
	ctx.HID.ClickWithModifier(game.LeftButton, rightArmCoords.X, rightArmCoords.Y, game.ShiftKey)
	utils.Sleep(500)
	step.CloseAllMenus()
	*ctx.Data = ctx.GameReader.GetData()
}

// Special Barb Logic
func barblogic(newItem data.Item, bodyloc item.LocationType) bool {
	ctx := context.Get()
	itemType := newItem.Desc().Type
	isShield := slices.Contains(shieldTypes, string(itemType))

	if isShield {
		playerLevel := 0
		if lvl, found := ctx.Data.PlayerUnit.FindStat(stat.Level, 0); found {
			playerLevel = lvl.Value
		}
		hasWarCry := ctx.Data.PlayerUnit.Skills[skill.WarCry].Level > 0

		if playerLevel >= 31 || hasWarCry {
			return bodyloc == item.LocRightArm
		}
		return false
	}

	if bodyloc == item.LocLeftArm || bodyloc == item.LocRightArm {
		if newItem.Ethereal {
			return false
		}

		twoHandedTypes := []string{"pole", "spea", "staf", "bow", "xbow", "bowq", "xbowq"}
		if slices.Contains(twoHandedTypes, itemType) {
			return false
		}
		_, hasBaseTwoHandedMin := newItem.BaseStats.FindStat(stat.TwoHandedMinDamage, 0)
		_, hasBaseTwoHandedMax := newItem.BaseStats.FindStat(stat.TwoHandedMaxDamage, 0)
		isTwoHanded := hasBaseTwoHandedMin || hasBaseTwoHandedMax

		if isTwoHanded {
			return false
		}

		// Only allow specific weapon types for 1-hand weapons
		allowedOneHandTypes := []string{"mace", "club", "morningstar", "flail", "hammer", "maul", "warhammer", "scepter", "swor", "axe"}
		if !slices.Contains(allowedOneHandTypes, itemType) {
			return false
		}

		// Exclude normal/superior base items with sockets that are needed for Steel/Malice runewords
		if newItem.Quality <= item.QualitySuperior && !newItem.IsRuneword {
			sockets, hasSockets := newItem.FindStat(stat.NumSockets, 0)
			if hasSockets {
				steelMaliceBaseTypes := []string{"mace", "club"}
				if slices.Contains(steelMaliceBaseTypes, itemType) {
					if sockets.Value == 2 || sockets.Value == 3 {
						return false
					}
				}
			}
		}
	}

	return true
}
