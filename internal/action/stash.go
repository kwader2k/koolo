package action

import (
	"errors"
	"fmt"
	"log/slog"
	"slices"
	"strconv"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/nip"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

const (
	maxGoldPerStashTab    = 2500000
	maxGoldPerSharedStash = 7500000 // Combined gold cap for all shared stash pages

	// NEW CONSTANTS FOR IMPROVED GOLD STASHING
	minInventoryGoldForStashAggressiveLeveling = 1000   // Stash if inventory gold exceeds 1k during leveling when total gold is low
	maxTotalGoldForAggressiveLevelingStash     = 150000 // Trigger aggressive stashing if total gold (inventory + stashed) is below this

	// DLC-specific stash tab constants. These use high values to avoid collision
	// with shared stash page numbers (tabs 2..6).
	StashTabGems      = 100
	StashTabMaterials = 101
	StashTabRunes     = 102
)

func Stash(forceStash bool) error {
	ctx := context.Get()
	ctx.SetLastAction("Stash")

	ctx.Logger.Debug("Checking for items to stash...")
	if !isStashingRequired(forceStash) {
		return nil
	}

	ctx.Logger.Info("Stashing items...")

	switch ctx.Data.PlayerUnit.Area {
	case area.KurastDocks:
		MoveToCoords(data.Position{X: 5146, Y: 5067})
	case area.LutGholein:
		MoveToCoords(data.Position{X: 5130, Y: 5086})
	}

	bank, _ := ctx.Data.Objects.FindOne(object.Bank)
	InteractObject(bank,
		func() bool {
			return ctx.Data.OpenMenus.Stash
		},
	)
	// Clear messages like TZ change or public game spam. Prevent bot from clicking on messages
	ClearMessages()
	stashGold()
	stashInventory(forceStash)
	// Add call to dropExcessItems after stashing
	dropExcessItems()
	step.CloseAllMenus()

	return nil
}

func isStashingRequired(firstRun bool) bool {
	ctx := context.Get()
	ctx.SetLastStep("isStashingRequired")

	// Check if the character is currently leveling
	_, isLevelingChar := ctx.Char.(context.LevelingCharacter)

	for _, i := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if i.IsPotion() {
			continue
		}

		stashIt, dropIt, _, _ := shouldStashIt(i, firstRun)
		if stashIt || dropIt { // Check for dropIt as well
			return true
		}
	}

	// Check if all stash tabs are full of gold
	// Personal stash: 2.5M cap, Shared stash: 7.5M combined cap
	isStashFull := ctx.Data.Inventory.StashedGold[0] >= maxGoldPerStashTab &&
		ctx.Data.Inventory.StashedGold[1] >= maxGoldPerSharedStash

	// Calculate total gold (inventory + stashed) for the new aggressive stashing rule
	// StashedGold[0] = personal, StashedGold[1] = combined shared gold
	totalGold := ctx.Data.Inventory.Gold + ctx.Data.Inventory.StashedGold[0] + ctx.Data.Inventory.StashedGold[1]

	// 1. AGGRESSIVE STASHING for leveling characters with LOW TOTAL GOLD
	if isLevelingChar && totalGold < maxTotalGoldForAggressiveLevelingStash && ctx.Data.Inventory.Gold >= minInventoryGoldForStashAggressiveLeveling && !isStashFull {
		ctx.Logger.Debug(fmt.Sprintf("Leveling char with LOW TOTAL GOLD (%.2fk < %.2fk) and INV GOLD (%.2fk) above aggressive threshold (%.2fk). Stashing gold.",
			float64(totalGold)/1000, float64(maxTotalGoldForAggressiveLevelingStash)/1000,
			float64(ctx.Data.Inventory.Gold)/1000, float64(minInventoryGoldForStashAggressiveLeveling)/1000))
		return true
	}

	// 2. STANDARD STASHING for all other cases (non-leveling, or leveling with sufficient total gold)
	if ctx.Data.Inventory.Gold > ctx.Data.PlayerUnit.MaxGold()/3 && !isStashFull {
		ctx.Logger.Debug(fmt.Sprintf("Inventory gold (%.2fk) is above standard threshold (%.2fk). Stashing gold.",
			float64(ctx.Data.Inventory.Gold)/1000, float64(ctx.Data.PlayerUnit.MaxGold())/3/1000))
		return true
	}

	return false
}

func stashGold() {
	ctx := context.Get()
	ctx.SetLastAction("stashGold")

	if ctx.Data.Inventory.Gold == 0 {
		return
	}

	ctx.Logger.Info("Stashing gold...", slog.Int("gold", ctx.Data.Inventory.Gold))

	// Try personal stash first (tab 1, max 2.5M)
	ctx.RefreshGameData()
	if ctx.Data.Inventory.Gold > 0 && ctx.Data.Inventory.StashedGold[0] < maxGoldPerStashTab {
		SwitchStashTab(1)
		clickStashGoldBtn()
		utils.PingSleep(utils.Critical, 1000)
		ctx.RefreshGameData()
		if ctx.Data.Inventory.Gold == 0 {
			ctx.Logger.Info("All inventory gold stashed.")
			return
		}
	}

	// Try shared stash (tab 2, combined max 7.5M)
	// Gold is shared across all pages, so depositing on any page works
	if ctx.Data.Inventory.Gold > 0 && ctx.Data.Inventory.StashedGold[1] < maxGoldPerSharedStash {
		SwitchStashTab(2)
		clickStashGoldBtn()
		utils.PingSleep(utils.Critical, 1000)
		ctx.RefreshGameData()
		if ctx.Data.Inventory.Gold == 0 {
			ctx.Logger.Info("All inventory gold stashed.")
			return
		}
	}

	ctx.Logger.Info("All stash tabs are full of gold :D")
}

func stashInventory(firstRun bool) {
	ctx := context.Get()
	ctx.SetLastAction("stashInventory")

	// Determine starting tab based on configuration
	startTab := 1 // Personal stash by default (tab 1)
	if ctx.CharacterCfg.Character.StashToShared {
		startTab = 2 // Start with first shared stash tab if configured (tabs 2-4 are shared)
	}

	currentTab := startTab
	SwitchStashTab(currentTab)

	// Make a copy of inventory items to avoid issues if the slice changes during iteration
	itemsToProcess := make([]data.Item, 0)
	for _, i := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if i.IsPotion() {
			continue
		}

		itemsToProcess = append(itemsToProcess, i)
	}

	for _, i := range itemsToProcess {
		stashIt, dropIt, matchedRule, ruleFile := shouldStashIt(i, firstRun)

		if dropIt {
			ctx.Logger.Info(fmt.Sprintf("Dropping item %s [%s] due to MaxQuantity rule.", i.Desc().Name, i.Quality.ToString()))
			blacklistItem(i)
			utils.PingSleep(utils.Medium, 500) // Medium operation: Prepare for item drop
			DropItem(i)
			utils.PingSleep(utils.Medium, 500) // Medium operation: Wait for drop to complete
			step.CloseAllMenus()
			continue
		}

		if !stashIt {
			continue
		}

		stashed := stashItemAcrossTabs(i, matchedRule, ruleFile, firstRun)
		if !stashed {
			ctx.Logger.Warn(fmt.Sprintf("ERROR: Item %s [%s] could not be stashed into any tab. All stash tabs might be full.", i.Desc().Name, i.Quality.ToString()))
		}
	}
	step.CloseAllMenus()
}

// stashItemAcrossTabs attempts to stash the given item across available tabs, applying the same logic
// used by the main stash routine. It returns true if the item was stashed successfully.
func stashItemAcrossTabs(i data.Item, matchedRule string, ruleFile string, firstRun bool) bool {
	ctx := context.Get()
	displayName := formatItemName(i)

	startTab := 1
	if ctx.CharacterCfg.Character.StashToShared {
		startTab = 2
	}

	targetStartTab := startTab
	if (i.Name == "grandcharm" || i.Name == "smallcharm" || i.Name == "largecharm") && i.Quality == item.QualityUnique {
		targetStartTab = 2
	}

	itemStashed := false
	// Tab 1=Personal, Tabs 2..N=Shared stash pages.
	// Non-DLC: 3 shared pages (tabs 2-4). DLC: 5 shared pages (tabs 2-6).
	// Use SharedStashPages from memory to determine actual count.
	sharedPages := ctx.Data.Inventory.SharedStashPages
	if sharedPages == 0 {
		// Fallback: assume 3 pages if not detected
		sharedPages = 3
	}
	maxTab := 1 + sharedPages // personal (1) + all shared pages

	for tabAttempt := targetStartTab; tabAttempt <= maxTab; tabAttempt++ {
		SwitchStashTab(tabAttempt)

		if stashItemAction(i, matchedRule, ruleFile, firstRun) {
			itemStashed = true
			r, res := ctx.CharacterCfg.Runtime.Rules.EvaluateAll(i)

			if res != nip.RuleResultFullMatch && firstRun {
				ctx.Logger.Info(
					fmt.Sprintf("Item %s [%s] stashed to tab %d because it was found in the inventory during the first run.", displayName, i.Quality.ToString(), tabAttempt),
				)
			} else {
				ctx.Logger.Info(
					fmt.Sprintf("Item %s [%s] stashed to tab %d", displayName, i.Quality.ToString(), tabAttempt),
					slog.String("nipFile", fmt.Sprintf("%s:%d", r.Filename, r.LineNumber)),
					slog.String("rawRule", r.RawLine),
				)
			}
			break
		}
		ctx.Logger.Debug(fmt.Sprintf("Item %s could not be stashed on tab %d. Trying next.", displayName, tabAttempt))
	}

	if !itemStashed && targetStartTab == 2 {
		ctx.Logger.Debug(fmt.Sprintf("All shared stash tabs full for %s, trying personal stash as fallback", displayName))
		SwitchStashTab(1)
		if stashItemAction(i, matchedRule, ruleFile, firstRun) {
			itemStashed = true
			ctx.Logger.Info(fmt.Sprintf("Item %s [%s] stashed to personal stash (tab 1) as fallback", displayName, i.Quality.ToString()))
		}
	}

	return itemStashed
}

// shouldStashIt now returns stashIt, dropIt, matchedRule, ruleFile
func shouldStashIt(i data.Item, firstRun bool) (bool, bool, string, string) {
	ctx := context.Get()
	ctx.SetLastStep("shouldStashIt")

	// Don't stash items in protected slots (highest priority exclusion)
	if ctx.CharacterCfg.Inventory.InventoryLock[i.Position.Y][i.Position.X] == 0 {
		return false, false, "", ""
	}

	// These items should NEVER be stashed, regardless of quest status, pickit rules, or first run.
	fmt.Printf("DEBUG: Evaluating item '%s' for *absolute* exclusion from stash.\n", i.Name)
	if i.Name == "horadricstaff" { // This is the simplest way given your logs
		fmt.Printf("DEBUG: ABSOLUTELY PREVENTING stash for '%s' (Horadric Staff exclusion).\n", i.Name)
		return false, false, "", "" // Explicitly do NOT stash the Horadric Staff
	}

	if i.Name == "TomeOfTownPortal" || i.Name == "TomeOfIdentify" || i.Name == "Key" || i.Name == "WirtsLeg" {
		fmt.Printf("DEBUG: ABSOLUTELY PREVENTING stash for '%s' (Quest/Special item exclusion).\n", i.Name)
		return false, false, "", ""
	}

	if _, isLevelingChar := ctx.Char.(context.LevelingCharacter); isLevelingChar && i.IsFromQuest() && i.Name != "HoradricCube" || i.Name == "HoradricStaff" {
		return false, false, "", ""
	}

	if firstRun {
		fmt.Printf("DEBUG: Allowing stash for '%s' (first run).\n", i.Name)
		return true, false, "FirstRun", ""
	}

	// Stash items that are part of a recipe which are not covered by the NIP rules
	if shouldKeepRecipeItem(i) {
		return true, false, "Item is part of a enabled recipe", ""
	}

	// Location/position checks
	if i.Position.Y >= len(ctx.CharacterCfg.Inventory.InventoryLock) || i.Position.X >= len(ctx.CharacterCfg.Inventory.InventoryLock[0]) {
		return false, false, "", ""
	}

	if i.Location.LocationType == item.LocationInventory && ctx.CharacterCfg.Inventory.InventoryLock[i.Position.Y][i.Position.X] == 0 || i.IsPotion() {
		return false, false, "", ""
	}

	// NOW, evaluate pickit rules.
	tierRule, mercTierRule := ctx.CharacterCfg.Runtime.Rules.EvaluateTiers(i, ctx.CharacterCfg.Runtime.TierRules)
	if tierRule.Tier() > 0.0 && IsBetterThanEquipped(i, false, PlayerScore) {
		return true, true, tierRule.RawLine, tierRule.Filename + ":" + strconv.Itoa(tierRule.LineNumber)
	}

	if mercTierRule.Tier() > 0.0 && IsBetterThanEquipped(i, true, MercScore) {
		return true, true, mercTierRule.RawLine, mercTierRule.Filename + ":" + strconv.Itoa(mercTierRule.LineNumber)
	}

	// NOW, evaluate pickit rules.
	rule, res := ctx.CharacterCfg.Runtime.Rules.EvaluateAllIgnoreTiers(i)

	if res == nip.RuleResultFullMatch {
		if doesExceedQuantity(rule) {
			// If it matches a rule but exceeds quantity, we want to drop it, not stash.
			fmt.Printf("DEBUG: Dropping '%s' because MaxQuantity is exceeded.\n", i.Name)
			return false, true, rule.RawLine, rule.Filename + ":" + strconv.Itoa(rule.LineNumber)
		} else {
			// If it matches a rule and quantity is fine, stash it.
			fmt.Printf("DEBUG: Allowing stash for '%s' (pickit rule match: %s).\n", i.Name, rule.RawLine)
			return true, false, rule.RawLine, rule.Filename + ":" + strconv.Itoa(rule.LineNumber)
		}
	}

	if i.IsRuneword {
		return true, false, "Runeword", ""
	}

	fmt.Printf("DEBUG: Disallowing stash for '%s' (no rule match and not explicitly kept, and not exceeding quantity).\n", i.Name)
	return false, false, "", "" // Default if no other rule matches
}

// shouldKeepRecipeItem decides whether the bot should stash a low-quality item that is part of an enabled cube recipe.
// It now supports keeping multiple jewels for crafting via maxJewelsKept.
// shouldKeepRecipeItem decides whether the bot should stash a low-quality item that is part of an enabled cube recipe.
// It now supports keeping multiple jewels for crafting via JewelsToKeep.
// shouldKeepRecipeItem decides whether the bot should stash a low-quality item that is part of an enabled cube recipe.
// It now supports keeping multiple jewels (of any quality) for crafting via JewelsToKeep.
func shouldKeepRecipeItem(i data.Item) bool {
	ctx := context.Get()
	ctx.SetLastStep("shouldKeepRecipeItem")

	// For non-jewel items: only normal/magic quality can be part of recipes
	// For jewels: any quality (magic, rare, unique, etc.) can be used in crafting recipes
	if string(i.Name) != "Jewel" && i.Quality > item.QualityMagic {
		return false
	}

	itemInStashNotMatchingRule := false
	jewelCount := 0

	// Count ALL non-NIP jewels in stash (regardless of quality: magic, rare, unique, etc.)
	for _, it := range ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash) {
		if string(it.Name) == "Jewel" {
			if _, res := ctx.CharacterCfg.Runtime.Rules.EvaluateAll(it); res != nip.RuleResultFullMatch {
				jewelCount++
			}
		}
		// For OTHER recipe items (not jewels): match on base name and require magic quality
		// so only another magic item of the same base blocks us
		if string(it.Name) != "Jewel" && strings.EqualFold(string(it.Name), string(i.Name)) && it.Quality == item.QualityMagic {
			_, res := ctx.CharacterCfg.Runtime.Rules.EvaluateAll(it)
			if res != nip.RuleResultFullMatch {
				itemInStashNotMatchingRule = true
			}
		}
	}

	// CRITICAL: Also count ALL non-NIP jewels currently in inventory (any quality, excluding the one we're evaluating)
	// because they will also be stashed in the same run
	for _, it := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if string(it.Name) == "Jewel" && it.UnitID != i.UnitID {
			if _, res := ctx.CharacterCfg.Runtime.Rules.EvaluateAll(it); res != nip.RuleResultFullMatch {
				jewelCount++
			}
		}
	}

	recipeMatch := false

	// Check if the item is part of an enabled recipe
	for _, recipe := range Recipes {
		if slices.Contains(recipe.Items, string(i.Name)) &&
			slices.Contains(ctx.CharacterCfg.CubeRecipes.EnabledRecipes, recipe.Name) {
			recipeMatch = true
			break
		}
	}

	// Special-case: For jewels of ANY quality used in crafting recipes, stash up to JewelsToKeep copies.
	if string(i.Name) == "Jewel" {
		if recipeMatch && jewelCount < ctx.CharacterCfg.CubeRecipes.JewelsToKeep {
			ctx.Logger.Debug(fmt.Sprintf("Keeping jewel (quality: %s) for recipe - current count: %d, limit: %d",
				i.Quality.ToString(), jewelCount, ctx.CharacterCfg.CubeRecipes.JewelsToKeep))
			return true
		}
		ctx.Logger.Debug(fmt.Sprintf("NOT keeping jewel (quality: %s) - count: %d, limit: %d, recipeMatch: %v",
			i.Quality.ToString(), jewelCount, ctx.CharacterCfg.CubeRecipes.JewelsToKeep, recipeMatch))
		return false
	}

	// For all other recipe items, keep one copy in the stash if none exists
	if recipeMatch && !itemInStashNotMatchingRule {
		return true
	}

	return false
}

func stashItemAction(i data.Item, rule string, ruleFile string, skipLogging bool) bool {
	ctx := context.Get()
	ctx.SetLastAction("stashItemAction")
	displayName := formatItemName(i)

	screenPos := ui.GetScreenCoordsForItem(i)
	ctx.HID.MovePointer(screenPos.X, screenPos.Y)
	utils.PingSleep(utils.Medium, 170)        // Medium operation: Move pointer to item
	screenshot := ctx.GameReader.Screenshot() // Take screenshot *before* attempting stash
	utils.PingSleep(utils.Medium, 150)        // Medium operation: Wait for screenshot
	ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
	utils.PingSleep(utils.Medium, 500) // Medium operation: Give game time to process the stash

	// Verify if the item is no longer in inventory
	ctx.RefreshGameData() // Crucial: Refresh data to see if item moved
	for _, it := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if it.UnitID == i.UnitID {
			ctx.Logger.Debug(fmt.Sprintf("Failed to stash item %s (UnitID: %d), still in inventory.", i.Name, i.UnitID))
			return false // Item is still in inventory, stash failed
		}
	}

	dropLocation := "unknown"

	// log the contents of picked up items
	ctx.Logger.Debug(fmt.Sprintf("Checking PickedUpItems for %s (UnitID: %d)", displayName, i.UnitID)) // Changed to Debug as this is internal state
	if _, found := ctx.CurrentGame.PickedUpItems[int(i.UnitID)]; found {
		areaId := ctx.CurrentGame.PickedUpItems[int(i.UnitID)]
		dropLocation = area.ID(areaId).Area().Name // Corrected to use areaId variable

		if slices.Contains(ctx.Data.TerrorZones, area.ID(areaId)) {
			dropLocation += " (terrorized)"
		}
	}

	// Don't log items that we already have in inventory during first run or that we don't want to notify about (gems, low runes .. etc)
	if !skipLogging && shouldNotifyAboutStashing(i) && ruleFile != "" {
		dropItem := i
		if dropItem.IsRuneword && dropItem.IdentifiedName == "" {
			dropItem.IdentifiedName = displayName
		}
		event.Send(event.ItemStashed(
			event.WithScreenshot(ctx.Name, fmt.Sprintf("Item %s [%d] stashed", displayName, i.Quality), screenshot),
			data.Drop{Item: dropItem, Rule: rule, RuleFile: ruleFile, DropLocation: dropLocation},
		))
	}

	return true // Item successfully stashed
}

func formatItemName(i data.Item) string {
	if i.IsRuneword && i.RunewordName != item.RunewordNone {
		if rwName := string(item.Name(i.RunewordName)); rwName != "" {
			return rwName
		}
	}

	if i.IdentifiedName != "" {
		return i.IdentifiedName
	}

	if desc := i.Desc().Name; desc != "" {
		return desc
	}

	return string(i.Name)
}

// dropExcessItems iterates through inventory and drops items marked for dropping
func dropExcessItems() {
	ctx := context.Get()
	ctx.SetLastAction("dropExcessItems")

	itemsToDrop := make([]data.Item, 0)
	for _, i := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if i.IsPotion() {
			continue
		}

		_, dropIt, _, _ := shouldStashIt(i, false) // Re-evaluate if it should be dropped (not firstRun)
		if dropIt {
			itemsToDrop = append(itemsToDrop, i)
		}
	}

	if len(itemsToDrop) > 0 {
		ctx.Logger.Info(fmt.Sprintf("Dropping %d excess items from inventory.", len(itemsToDrop)))
		// Ensure we are not in a menu before dropping
		step.CloseAllMenus()

		for _, i := range itemsToDrop {
			DropItem(i)
		}
	}
}

func blacklistItem(i data.Item) {
	ctx := context.Get()
	ctx.CurrentGame.BlacklistedItems = append(ctx.CurrentGame.BlacklistedItems, i)
	ctx.Logger.Info(fmt.Sprintf("Blacklisted item %s (UnitID: %d) to prevent immediate re-pickup.", i.Name, i.UnitID))
}

// DropItem handles moving an item from inventory to the ground
func DropItem(i data.Item) {
	ctx := context.Get()
	ctx.SetLastAction("DropItem")
	utils.PingSleep(utils.Medium, 170) // Medium operation: Prepare for drop
	step.CloseAllMenus()
	utils.PingSleep(utils.Medium, 170) // Medium operation: Wait for menus to close
	ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
	utils.PingSleep(utils.Medium, 170) // Medium operation: Wait for inventory to open
	screenPos := ui.GetScreenCoordsForItem(i)
	ctx.HID.MovePointer(screenPos.X, screenPos.Y)
	utils.PingSleep(utils.Medium, 170) // Medium operation: Position pointer on item
	ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
	utils.PingSleep(utils.Medium, 500) // Medium operation: Wait for item to drop
	step.CloseAllMenus()
	utils.PingSleep(utils.Medium, 170) // Medium operation: Clean up UI
	ctx.RefreshGameData()
	for _, it := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if it.UnitID == i.UnitID {
			ctx.Logger.Warn(fmt.Sprintf("Failed to drop item %s (UnitID: %d), still in inventory. Inventory might be full or area restricted.", i.Name, i.UnitID))
			return
		}
	}
	ctx.Logger.Debug(fmt.Sprintf("Successfully dropped item %s (UnitID: %d).", i.Name, i.UnitID))

	step.CloseAllMenus()
}

func shouldNotifyAboutStashing(i data.Item) bool {
	ctx := context.Get()

	if ctx.IsBossEquipmentActive {
		return false
	}

	ctx.Logger.Debug(fmt.Sprintf("Checking if we should notify about stashing %s %v", i.Name, i.Desc()))
	// Don't notify about gems
	if strings.Contains(i.Desc().Type, "gem") {
		return false
	}

	// Skip low runes (below lem)
	lowRunes := []string{"elrune", "eldrune", "tirrune", "nefrune", "ethrune", "ithrune", "talrune", "ralrune", "ortrune", "thulrune", "amnrune", "solrune", "shaelrune", "dolrune", "helrune", "iorune", "lumrune", "korune", "falrune"}
	if i.Desc().Type == item.TypeRune {
		itemName := strings.ToLower(string(i.Name))
		for _, runeName := range lowRunes {
			if itemName == runeName {
				if !(i.Name == "tirrune" || i.Name == "talrune" || i.Name == "ralrune" || i.Name == "ortrune" || i.Name == "thulrune" || i.Name == "amnrune" || i.Name == "solrune" || i.Name == "lumrune" || i.Name == "nefrune") { // Exclude specific runes from low rune skip logic if they are part of a recipe you want to keep
					return false
				}
			}
		}
	}

	return true
}

func clickStashGoldBtn() {
	ctx := context.Get()
	ctx.SetLastStep("clickStashGoldBtn")

	utils.PingSleep(utils.Medium, 170) // Medium operation: Prepare for gold button click
	if ctx.GameReader.LegacyGraphics() {
		ctx.HID.Click(game.LeftButton, ui.StashGoldBtnXClassic, ui.StashGoldBtnYClassic)
		utils.PingSleep(utils.Critical, 1000) // Critical operation: Wait for confirm dialog
		ctx.HID.Click(game.LeftButton, ui.StashGoldBtnConfirmXClassic, ui.StashGoldBtnConfirmYClassic)
	} else {
		ctx.HID.Click(game.LeftButton, ui.StashGoldBtnX, ui.StashGoldBtnY)
		utils.PingSleep(utils.Critical, 1000) // Critical operation: Wait for confirm dialog
		ctx.HID.Click(game.LeftButton, ui.StashGoldBtnConfirmX, ui.StashGoldBtnConfirmY)
	}
}

// SwitchStashTab switches to the specified stash tab.
// Tab mapping:
//
//	Tab 1          = Personal stash
//	Tab 2..N       = Shared stash pages (non-DLC: 2-4, DLC: 2-6)
//	StashTabGems   = DLC Gems tab (100)
//	StashTabMaterials = DLC Materials tab (101)
//	StashTabRunes  = DLC Runes tab (102)
func SwitchStashTab(tab int) {
	ctx := context.Get()
	if tab == ctx.CurrentGame.CurrentStashTab {
		return // Already on this tab
	}

	// Ensure any chat messages that could prevent clicking on the tab are cleared
	ClearMessages()
	utils.Sleep(200)

	ctx.SetLastStep("switchTab")

	if ctx.GameReader.LegacyGraphics() {
		switchStashTabLegacy(ctx, tab)
	} else {
		switchStashTabHD(ctx, tab)
	}
	ctx.CurrentGame.CurrentStashTab = tab
}

func switchStashTabHD(ctx *context.Status, tab int) {
	// DLC-specific tabs: click directly, no page navigation
	switch tab {
	case StashTabGems:
		ctx.HID.Click(game.LeftButton, ui.DLCGemsTabX, ui.DLCGemsTabY)
		utils.PingSleep(utils.Medium, 500)
		return
	case StashTabMaterials:
		ctx.HID.Click(game.LeftButton, ui.DLCMaterialsTabX, ui.DLCMaterialsTabY)
		utils.PingSleep(utils.Medium, 500)
		return
	case StashTabRunes:
		ctx.HID.Click(game.LeftButton, ui.DLCRunesTabX, ui.DLCRunesTabY)
		utils.PingSleep(utils.Medium, 500)
		return
	}

	prev := ctx.CurrentGame.CurrentStashTab

	// If switching between Personal (1) and Shared (2+), or from a DLC tab,
	// we need to click the UI tab button.
	needTabClick := (prev < 2 && tab >= 2) || (prev >= 2 && tab < 2) || prev == 0 || prev >= StashTabGems

	if tab == 1 || needTabClick {
		uiTab := 1
		if tab >= 2 {
			uiTab = 2
		}
		x := ui.SwitchStashTabBtnX + ui.SwitchStashTabBtnTabSize*uiTab - ui.SwitchStashTabBtnTabSize/2
		ctx.HID.Click(game.LeftButton, x, ui.SwitchStashTabBtnY)
		utils.PingSleep(utils.Medium, 500)
	}

	// Navigate shared stash pages
	if tab >= 2 {
		// If coming from a known shared page, navigate incrementally
		if prev >= 2 && prev < StashTabGems && !needTabClick {
			delta := tab - prev
			if delta > 0 {
				for i := 0; i < delta; i++ {
					ctx.HID.Click(game.LeftButton, ui.SharedStashNextPageX, ui.SharedStashNextPageY)
					utils.PingSleep(utils.Medium, 250)
				}
			} else if delta < 0 {
				for i := 0; i < -delta; i++ {
					ctx.HID.Click(game.LeftButton, ui.SharedStashPrevPageX, ui.SharedStashPrevPageY)
					utils.PingSleep(utils.Medium, 250)
				}
			}
		} else {
			// Full reset: clicking the Shared tab lands on page 1, then navigate forward
			nextClicks := tab - 2
			for i := 0; i < nextClicks; i++ {
				ctx.HID.Click(game.LeftButton, ui.SharedStashNextPageX, ui.SharedStashNextPageY)
				utils.PingSleep(utils.Medium, 250)
			}
		}
	}
}

func switchStashTabLegacy(ctx *context.Status, tab int) {
	// DLC-specific tabs
	switch tab {
	case StashTabGems:
		ctx.HID.Click(game.LeftButton, ui.DLCGemsTabXClassic, ui.DLCGemsTabYClassic)
		utils.PingSleep(utils.Medium, 500)
		return
	case StashTabMaterials:
		ctx.HID.Click(game.LeftButton, ui.DLCMaterialsTabXClassic, ui.DLCMaterialsTabYClassic)
		utils.PingSleep(utils.Medium, 500)
		return
	case StashTabRunes:
		ctx.HID.Click(game.LeftButton, ui.DLCRunesTabXClassic, ui.DLCRunesTabYClassic)
		utils.PingSleep(utils.Medium, 500)
		return
	}

	prev := ctx.CurrentGame.CurrentStashTab
	needTabClick := (prev < 2 && tab >= 2) || (prev >= 2 && tab < 2) || prev == 0 || prev >= StashTabGems

	if tab == 1 || needTabClick {
		uiTab := 1
		if tab >= 2 {
			uiTab = 2
		}
		x := ui.SwitchStashTabBtnXClassic + ui.SwitchStashTabBtnTabSizeClassic*uiTab - ui.SwitchStashTabBtnTabSizeClassic/2
		ctx.HID.Click(game.LeftButton, x, ui.SwitchStashTabBtnYClassic)
		utils.PingSleep(utils.Medium, 500)
	}

	if tab >= 2 {
		if prev >= 2 && prev < StashTabGems && !needTabClick {
			delta := tab - prev
			if delta > 0 {
				for i := 0; i < delta; i++ {
					ctx.HID.Click(game.LeftButton, ui.SharedStashNextPageXClassic, ui.SharedStashNextPageYClassic)
					utils.PingSleep(utils.Medium, 250)
				}
			} else if delta < 0 {
				for i := 0; i < -delta; i++ {
					ctx.HID.Click(game.LeftButton, ui.SharedStashPrevPageXClassic, ui.SharedStashPrevPageYClassic)
					utils.PingSleep(utils.Medium, 250)
				}
			}
		} else {
			nextClicks := tab - 2
			for i := 0; i < nextClicks; i++ {
				ctx.HID.Click(game.LeftButton, ui.SharedStashNextPageXClassic, ui.SharedStashNextPageYClassic)
				utils.PingSleep(utils.Medium, 250)
			}
		}
	}
}

func OpenStash() error {
	ctx := context.Get()
	ctx.SetLastAction("OpenStash")

	// Reset tab tracker — stash always opens on personal tab
	ctx.CurrentGame.CurrentStashTab = 1

	bank, found := ctx.Data.Objects.FindOne(object.Bank)
	if !found {
		return errors.New("stash not found")
	}
	InteractObject(bank,
		func() bool {
			return ctx.Data.OpenMenus.Stash
		},
	)

	return nil
}

func CloseStash() error {
	ctx := context.Get()
	ctx.SetLastAction("CloseStash")

	ctx.CurrentGame.CurrentStashTab = 0 // Reset tab tracker on close

	if ctx.Data.OpenMenus.Stash {
		ctx.HID.PressKey(win.VK_ESCAPE)
	} else {
		return errors.New("stash is not open")
	}

	return nil
}

func TakeItemsFromStash(stashedItems []data.Item) error {
	ctx := context.Get()
	ctx.SetLastAction("TakeItemsFromStash")

	if !ctx.Data.OpenMenus.Stash {
		err := OpenStash()
		if err != nil {
			return err
		}
	}

	utils.PingSleep(utils.Medium, 250) // Medium operation: Wait for stash to open

	for _, i := range stashedItems {

		// Determine the tab to switch to based on location type
		var targetTab int
		switch i.Location.LocationType {
		case item.LocationStash:
			targetTab = 1 // Personal stash
		case item.LocationSharedStash:
			targetTab = i.Location.Page + 1 // Page 1=tab 2, Page 2=tab 3, etc.
		case item.LocationGemsTab:
			targetTab = StashTabGems
		case item.LocationMaterialsTab:
			targetTab = StashTabMaterials
		case item.LocationRunesTab:
			targetTab = StashTabRunes
		default:
			continue
		}

		SwitchStashTab(targetTab)

		// Move the item to the inventory
		screenPos := ui.GetScreenCoordsForItem(i)
		ctx.HID.MovePointer(screenPos.X, screenPos.Y)
		ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
		utils.PingSleep(utils.Medium, 500) // Medium operation: Wait for item to move to inventory
	}

	return nil
}
