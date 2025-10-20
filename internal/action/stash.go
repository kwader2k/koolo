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
	maxGoldPerStashTab = 2500000

	// NEW CONSTANTS FOR IMPROVED GOLD STASHING
	minInventoryGoldForStashAggressiveLeveling = 1000   // Stash if inventory gold exceeds 1k during leveling when total gold is low
	maxTotalGoldForAggressiveLevelingStash     = 150000 // Trigger aggressive stashing if total gold (inventory + stashed) is below this
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

	isStashFull := true
	for _, goldInStash := range ctx.Data.Inventory.StashedGold {
		if goldInStash < maxGoldPerStashTab {
			isStashFull = false
			break // Optimization: No need to check further tabs if one has space
		}
	}

	// Calculate total gold (inventory + stashed) for the new aggressive stashing rule
	totalGold := ctx.Data.Inventory.Gold
	for _, stashedGold := range ctx.Data.Inventory.StashedGold {
		totalGold += stashedGold
	}

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

	for tab, goldInStash := range ctx.Data.Inventory.StashedGold {
		ctx.RefreshGameData()
		if ctx.Data.Inventory.Gold == 0 {
			ctx.Logger.Info("All inventory gold stashed.") // Added log for clarity
			return
		}

		if goldInStash < maxGoldPerStashTab {
			SwitchStashTab(tab + 1) // Stash tabs are 0-indexed in data, but 1-indexed for UI interaction
			clickStashGoldBtn()
			utils.Sleep(1000) // Increased sleep after first click to ensure dialog appears
			// After clicking, refresh data again to see if gold is now 0 or less
			ctx.RefreshGameData()             // Crucial: Refresh data to see if gold has been deposited
			if ctx.Data.Inventory.Gold == 0 { // Check if all gold was stashed in this tab
				ctx.Logger.Info("All inventory gold stashed.")
				return
			}
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
			utils.Sleep(500)
			DropItem(i)
			utils.Sleep(500)
			step.CloseAllMenus()
			continue
		}

		if !stashIt {
			continue
		}

		// Determine target tab for this specific item
		targetStartTab := startTab

		// Always stash unique charms to the shared stash (override personal stash setting)
		if (i.Name == "grandcharm" || i.Name == "smallcharm" || i.Name == "largecharm") && i.Quality == item.QualityUnique {
			targetStartTab = 2 // Force shared stash for unique charms
		}

		itemStashed := false
		// Loop through tabs trying to stash the item
		// For shared stash mode: try tabs 2, 3, 4, then fall back to 1 if all shared tabs are full
		// For personal stash mode: try tab 1, then 2, 3, 4 if personal is full
		maxTab := 4 // We only have 4 tabs total

		for tabAttempt := targetStartTab; tabAttempt <= maxTab; tabAttempt++ {
			SwitchStashTab(tabAttempt)

			if stashItemAction(i, matchedRule, ruleFile, firstRun) {
				itemStashed = true
				r, res := ctx.CharacterCfg.Runtime.Rules.EvaluateAll(i)

				if res != nip.RuleResultFullMatch && firstRun {
					ctx.Logger.Info(
						fmt.Sprintf("Item %s [%s] stashed to tab %d because it was found in the inventory during the first run.", i.Desc().Name, i.Quality.ToString(), tabAttempt),
					)
				} else {
					ctx.Logger.Info(
						fmt.Sprintf("Item %s [%s] stashed to tab %d", i.Desc().Name, i.Quality.ToString(), tabAttempt),
						slog.String("nipFile", fmt.Sprintf("%s:%d", r.Filename, r.LineNumber)),
						slog.String("rawRule", r.RawLine),
					)
				}
				break
			}
			ctx.Logger.Debug(fmt.Sprintf("Item %s could not be stashed on tab %d. Trying next.", i.Name, tabAttempt))
		}

		// If we couldn't stash in shared tabs and started with shared, try personal as last resort
		if !itemStashed && targetStartTab == 2 {
			ctx.Logger.Debug(fmt.Sprintf("All shared stash tabs full for %s, trying personal stash as fallback", i.Name))
			SwitchStashTab(1)
			if stashItemAction(i, matchedRule, ruleFile, firstRun) {
				itemStashed = true
				ctx.Logger.Info(fmt.Sprintf("Item %s [%s] stashed to personal stash (tab 1) as fallback", i.Desc().Name, i.Quality.ToString()))
			}
		}

		if !itemStashed {
			ctx.Logger.Warn(fmt.Sprintf("ERROR: Item %s [%s] could not be stashed into any tab. All stash tabs might be full.", i.Desc().Name, i.Quality.ToString()))
			// TODO: Potentially stop the bot or alert the user more critically here
		}
	}
	step.CloseAllMenus()
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

	if i.Name == "tomeoftownportal" || i.Name == "tomeofidentify" || i.Name == "key" || i.Name == "wirtsleg" {
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

	if i.IsRuneword {
		return true, false, "Runeword", ""
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
	rule, res := ctx.CharacterCfg.Runtime.Rules.EvaluateAll(i)

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

	fmt.Printf("DEBUG: Disallowing stash for '%s' (no rule match and not explicitly kept, and not exceeding quantity).\n", i.Name)
	return false, false, "", "" // Default if no other rule matches
}

func shouldKeepRecipeItem(i data.Item) bool {
	ctx := context.Get()
	ctx.SetLastStep("shouldKeepRecipeItem")

	// No items with quality higher than magic can be part of a recipe
	if i.Quality > item.QualityMagic {
		return false
	}

	itemInStashNotMatchingRule := false

	// Check if we already have the item in our stash and if it doesn't match any of our pickit rules
	for _, it := range ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash) {
		if it.Name == i.Name {
			_, res := ctx.CharacterCfg.Runtime.Rules.EvaluateAll(it)
			if res != nip.RuleResultFullMatch {
				itemInStashNotMatchingRule = true
				break // Optimization: Found one, no need to check others
			}
		}
	}

	recipeMatch := false

	// Check if the item is part of a recipe and if that recipe is enabled
	// 'Recipes' variable is expected to be defined/imported from 'cube_recipes.go' or similar.
	// This function (shouldKeepRecipeItem) itself is external to this file.
	for _, recipe := range Recipes { // Assuming `Recipes` is properly defined/imported
		if slices.Contains(recipe.Items, string(i.Name)) && slices.Contains(ctx.CharacterCfg.CubeRecipes.EnabledRecipes, recipe.Name) {
			recipeMatch = true
			break
		}
	}

	if recipeMatch && !itemInStashNotMatchingRule {
		return true
	}

	return false
}

func stashItemAction(i data.Item, rule string, ruleFile string, skipLogging bool) bool {
	ctx := context.Get()
	ctx.SetLastAction("stashItemAction")

	screenPos := ui.GetScreenCoordsForItem(i)
	ctx.HID.MovePointer(screenPos.X, screenPos.Y)
	utils.Sleep(170)
	screenshot := ctx.GameReader.Screenshot() // Take screenshot *before* attempting stash
	utils.Sleep(150)
	ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
	utils.Sleep(500) // Give game time to process the stash

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
	ctx.Logger.Debug(fmt.Sprintf("Checking PickedUpItems for %s (UnitID: %d)", i.Name, i.UnitID)) // Changed to Debug as this is internal state
	if _, found := ctx.CurrentGame.PickedUpItems[int(i.UnitID)]; found {
		areaId := ctx.CurrentGame.PickedUpItems[int(i.UnitID)]
		dropLocation = area.ID(areaId).Area().Name // Corrected to use areaId variable

		if slices.Contains(ctx.Data.TerrorZones, area.ID(areaId)) {
			dropLocation += " (terrorized)"
		}
	}

	// Don't log items that we already have in inventory during first run or that we don't want to notify about (gems, low runes .. etc)
	if !skipLogging && shouldNotifyAboutStashing(i) && ruleFile != "" {
		event.Send(event.ItemStashed(event.WithScreenshot(ctx.Name, fmt.Sprintf("Item %s [%d] stashed", i.Name, i.Quality), screenshot), data.Drop{Item: i, Rule: rule, RuleFile: ruleFile, DropLocation: dropLocation}))
	}

	return true // Item successfully stashed
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
	utils.Sleep(170)
	step.CloseAllMenus()
	utils.Sleep(170)
	ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
	utils.Sleep(170)
	screenPos := ui.GetScreenCoordsForItem(i)
	ctx.HID.MovePointer(screenPos.X, screenPos.Y)
	utils.Sleep(170)
	ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
	utils.Sleep(500)
	step.CloseAllMenus()
	utils.Sleep(170)
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

	utils.Sleep(170)
	if ctx.GameReader.LegacyGraphics() {
		ctx.HID.Click(game.LeftButton, ui.StashGoldBtnXClassic, ui.StashGoldBtnYClassic)
		utils.Sleep(1000)
		ctx.HID.Click(game.LeftButton, ui.StashGoldBtnConfirmXClassic, ui.StashGoldBtnConfirmYClassic)
	} else {
		ctx.HID.Click(game.LeftButton, ui.StashGoldBtnX, ui.StashGoldBtnY)
		utils.Sleep(1000)
		ctx.HID.Click(game.LeftButton, ui.StashGoldBtnConfirmX, ui.StashGoldBtnConfirmY)
	}
}

func SwitchStashTab(tab int) {
	ctx := context.Get()
	ctx.SetLastStep("switchTab")

	if ctx.GameReader.LegacyGraphics() {
		x := ui.SwitchStashTabBtnXClassic
		y := ui.SwitchStashTabBtnYClassic

		tabSize := ui.SwitchStashTabBtnTabSizeClassic
		x = x + tabSize*tab - tabSize/2
		ctx.HID.Click(game.LeftButton, x, y)
		utils.Sleep(500)
	} else {
		x := ui.SwitchStashTabBtnX
		y := ui.SwitchStashTabBtnY

		tabSize := ui.SwitchStashTabBtnTabSize
		x = x + tabSize*tab - tabSize/2
		ctx.HID.Click(game.LeftButton, x, y)
		utils.Sleep(500)
	}

}

func OpenStash() error {
	ctx := context.Get()
	ctx.SetLastAction("OpenStash")

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

	utils.Sleep(250)

	for _, i := range stashedItems {

		if i.Location.LocationType != item.LocationStash && i.Location.LocationType != item.LocationSharedStash {
			continue
		}

		// Make sure we're on the correct tab
		SwitchStashTab(i.Location.Page + 1)

		// Move the item to the inventory
		screenPos := ui.GetScreenCoordsForItem(i)
		ctx.HID.MovePointer(screenPos.X, screenPos.Y)
		ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
		utils.Sleep(500)
	}

	return nil
}
