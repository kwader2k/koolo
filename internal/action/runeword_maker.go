package action

import (
	"fmt"
	"slices"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/pickit"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

func MakeRunewords() error {
	ctx := context.Get()
	ctx.SetLastAction("SocketAddItems")
	cfg := ctx.CharacterCfg

	if !cfg.Game.RunewordMaker.Enabled {
		return nil
	}

	// Build location list - include RunesTab for DLC characters (runes are stored there)
	insertLocations := []item.LocationType{item.LocationStash, item.LocationSharedStash, item.LocationInventory}
	if ctx.Data.IsDLC() {
		insertLocations = append(insertLocations, item.LocationRunesTab)
	}

	insertItems := FilterDLCGhostItems(ctx.Data.Inventory.ByLocation(insertLocations...))
	baseItems := ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash, item.LocationInventory)

	_, isLevelingChar := ctx.Char.(context.LevelingCharacter)

	enabledRecipes := cfg.Game.RunewordMaker.EnabledRecipes
	enabledSet := make(map[string]struct{}, len(enabledRecipes))
	for _, recipe := range enabledRecipes {
		enabledSet[recipe] = struct{}{}
	}
	if !isLevelingChar {
		for recipe := range ctx.CharacterCfg.Game.RunewordRerollRules {
			enabledSet[recipe] = struct{}{}
		}
	}

	if len(enabledSet) == 0 {
		return nil
	}

	for _, recipe := range Runewords {

		if _, enabled := enabledSet[string(recipe.Name)]; !enabled {
			continue
		}

		ctx.Logger.Debug("Runeword recipe is enabled, processing", "recipe", recipe.Name)

		continueProcessing := true
		skippedBases := make(map[data.UnitID]struct{})
		for continueProcessing {
			candidateBases := baseItems
			if len(skippedBases) > 0 {
				filteredBases := make([]data.Item, 0, len(baseItems))
				for _, base := range baseItems {
					if _, skip := skippedBases[base.UnitID]; skip {
						continue
					}
					filteredBases = append(filteredBases, base)
				}
				candidateBases = filteredBases
			}
			if baseItem, hasBase := hasBaseForRunewordRecipe(candidateBases, recipe); hasBase {
				existingTier, hasExisting := currentRunewordBaseTier(ctx, recipe, baseItem.Type().Name)

				// Check if we should skip this base due to tier upgrade logic
				// For leveling characters: always apply tier check (existing behavior)
				// For non-leveling: only apply if AutoUpgrade is enabled
				shouldCheckUpgrade := isLevelingChar || cfg.Game.RunewordMaker.AutoUpgrade
				if shouldCheckUpgrade && hasExisting && (len(recipe.BaseSortOrder) == 0 || baseItem.Desc().Tier() <= existingTier) {
					ctx.Logger.Debug("Skipping base - existing runeword has equal or better tier in same base type",
						"recipe", recipe.Name,
						"baseType", baseItem.Type().Name,
						"existingTier", existingTier,
						"newBaseTier", baseItem.Desc().Tier())
					skippedBases[baseItem.UnitID] = struct{}{}
					continue
				}

				// Check if character can wear this item (if OnlyIfWearable is enabled)
				if cfg.Game.RunewordMaker.OnlyIfWearable && !characterMeetsRequirements(ctx, baseItem) {
					ctx.Logger.Debug("Skipping base - character cannot wear this base item",
						"recipe", recipe.Name,
						"base", baseItem.Name,
						"requiredStr", baseItem.Desc().RequiredStrength,
						"requiredDex", baseItem.Desc().RequiredDexterity)
					skippedBases[baseItem.UnitID] = struct{}{}
					continue
				}

				if inserts, hasInserts := hasItemsForRunewordRecipe(insertItems, recipe); hasInserts {
					err := SocketItems(ctx, recipe, baseItem, inserts...)
					if err != nil {
						return err
					}

					// Log successful creation of the runeword for easier auditing
					ctx.Logger.Info("Runeword maker: created runeword",
						"runeword", recipe.Name,
						"base", baseItem.Name,
					)

					// Refresh game data so in-memory inventory reflects the newly created runeword
					ctx.RefreshGameData()

					// Recalculate available items from the refreshed game state so the maker
					// doesn't try to reuse the same base or inserts.
					// Rebuild location list for DLC characters
					insertLocations = []item.LocationType{item.LocationStash, item.LocationSharedStash, item.LocationInventory}
					if ctx.Data.IsDLC() {
						insertLocations = append(insertLocations, item.LocationRunesTab)
					}
					insertItems = FilterDLCGhostItems(ctx.Data.Inventory.ByLocation(insertLocations...))
					baseItems = ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash, item.LocationInventory)
				} else {
					// No inserts available for this recipe at this time
					ctx.Logger.Debug("Runeword maker: no inserts available for recipe; skipping",
						"runeword", recipe.Name,
					)
					continueProcessing = false
				}
			} else {
				// No suitable base found for this recipe
				ctx.Logger.Debug("Runeword maker: no suitable base found for recipe; skipping",
					"runeword", recipe.Name,
				)
				continueProcessing = false
			}
		}
	}
	return nil
}

func SocketItems(ctx *context.Status, recipe Runeword, base data.Item, items ...data.Item) error {

	ctx.SetLastAction("SocketItem")

	// Build location list - include RunesTab for DLC characters
	insertLocations := []item.LocationType{item.LocationStash, item.LocationSharedStash, item.LocationInventory}
	if ctx.Data.IsDLC() {
		insertLocations = append(insertLocations, item.LocationRunesTab)
	}
	ins := FilterDLCGhostItems(ctx.Data.Inventory.ByLocation(insertLocations...))

	for _, itm := range items {
		// Check if item is in any stash location (personal, shared, or DLC tabs)
		if itm.Location.LocationType == item.LocationStash ||
			itm.Location.LocationType == item.LocationSharedStash ||
			itm.Location.LocationType == item.LocationGemsTab ||
			itm.Location.LocationType == item.LocationMaterialsTab ||
			itm.Location.LocationType == item.LocationRunesTab {
			OpenStash()
			break
		}
	}
	if !ctx.Data.OpenMenus.Stash && (base.Location.LocationType == item.LocationStash || base.Location.LocationType == item.LocationSharedStash) {
		err := OpenStash()
		if err != nil {
			return err
		}
	}

	if base.Location.LocationType == item.LocationSharedStash || base.Location.LocationType == item.LocationStash {
		ctx.Logger.Debug("Base in stash - checking it fits")
		if !itemFitsInventory(base) {
			ctx.Logger.Error("Base item does not fit in inventory", "item", base.Name)
			return step.CloseAllMenus()
		}

		if base.Location.LocationType == item.LocationSharedStash {
			ctx.Logger.Debug("Base in shared stash but fits in inv, switching to correct tab")
			SwitchStashTab(base.Location.Page + 1)
		} else {
			ctx.Logger.Debug("Base in personal stash but fits in inv, switching to correct tab")
			SwitchStashTab(1)
		}
		ctx.Logger.Debug("Switched to correct tab")
		utils.Sleep(500)
		screenPos := ui.GetScreenCoordsForItem(base)
		ctx.Logger.Debug(fmt.Sprintf("Clicking after 5s at %d:%d", screenPos.X, screenPos.Y))
		moveSucceeded := false
		for attempt := 0; attempt < 2; attempt++ {
			ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
			utils.Sleep(500)
			ctx.RefreshGameData()
			moved, found := ctx.Data.Inventory.FindByID(base.UnitID)
			if found && moved.Location.LocationType == item.LocationInventory {
				base = moved
				moveSucceeded = true
				break
			}
		}
		if !moveSucceeded {
			ctx.Logger.Error("Failed to move base item from stash to inventory", "item", base.Name)
			return step.CloseAllMenus()
		}
	}

	usedItems := make(map[*data.Item]bool)
	orderedItems := make([]data.Item, 0)

	// Process each required insert in order
	for _, requiredInsert := range recipe.Runes {
		for i := range ins {
			item := &ins[i]
			if string(item.Name) == requiredInsert && !usedItems[item] {
				orderedItems = append(orderedItems, *item)
				usedItems[item] = true
				break
			}
		}
	}
	// Diagnostic log: report how many inserts we will attempt to socket
	if len(orderedItems) == 0 {
		ctx.Logger.Debug("SocketItems: no ordered inserts found for recipe",
			"runeword", recipe.Name,
		)
	} else {
		names := make([]string, 0, len(orderedItems))
		for _, it := range orderedItems {
			names = append(names, string(it.Name))
		}
		ctx.Logger.Debug("SocketItems: preparing inserts",
			"runeword", recipe.Name,
			"count", len(orderedItems),
			"items", fmt.Sprintf("%v", names),
		)
	}

	usedDLCIDs := make(map[data.UnitID]struct{})
	previousPage := -1 // Initialize to invalid page number
	for _, itm := range orderedItems {
		isDLCTab := itm.Location.LocationType == item.LocationGemsTab ||
			itm.Location.LocationType == item.LocationMaterialsTab ||
			itm.Location.LocationType == item.LocationRunesTab

		if isDLCTab {
			// DLC tab items require Ctrl+click to move to inventory first,
			// because left-click does not pick up from DLC tabs directly.
			switch itm.Location.LocationType {
			case item.LocationGemsTab:
				SwitchStashTab(StashTabGems)
			case item.LocationMaterialsTab:
				SwitchStashTab(StashTabMaterials)
			case item.LocationRunesTab:
				SwitchStashTab(StashTabRunes)
			}

			screenPos := ui.GetScreenCoordsForItem(itm)
			ctx.Logger.Debug("SocketItems: moving DLC tab item to inventory",
				"item", string(itm.Name),
				"location", string(itm.Location.LocationType),
				"screenX", screenPos.X,
				"screenY", screenPos.Y,
			)
			ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
			utils.Sleep(500)
			ctx.RefreshGameData()

			// DLC tab items get new UnitIDs when moved to inventory,
			// so find the item by name in inventory.
			// Track used UnitIDs to avoid matching the same item when
			// multiple identical runes are needed (e.g., 3x Jah).
			var invItem *data.Item
			for idx := range ctx.Data.Inventory.AllItems {
				candidate := &ctx.Data.Inventory.AllItems[idx]
				if _, used := usedDLCIDs[candidate.UnitID]; used {
					continue
				}
				if candidate.Name == itm.Name && candidate.Location.LocationType == item.LocationInventory {
					invItem = candidate
					break
				}
			}
			if invItem != nil {
				usedDLCIDs[invItem.UnitID] = struct{}{}
			} else {
				ctx.Logger.Error("SocketItems: DLC item not found in inventory after Ctrl+click",
					"item", string(itm.Name),
				)
				return fmt.Errorf("failed to move DLC item %s to inventory for socketing", itm.Name)
			}

			// Left-click from inventory to pick up to cursor
			invScreenPos := ui.GetScreenCoordsForItem(*invItem)
			ctx.Logger.Debug("SocketItems: picking up DLC item from inventory",
				"item", string(invItem.Name),
				"screenX", invScreenPos.X,
				"screenY", invScreenPos.Y,
			)
			ctx.HID.Click(game.LeftButton, invScreenPos.X, invScreenPos.Y)
			utils.Sleep(300)
		} else {
			// Regular stash or inventory items: left-click to pick up to cursor
			if itm.Location.LocationType == item.LocationSharedStash || itm.Location.LocationType == item.LocationStash {
				currentPage := itm.Location.Page + 1
				if previousPage != currentPage || currentPage != base.Location.Page {
					SwitchStashTab(currentPage)
				}
				previousPage = currentPage
			}

			screenPos := ui.GetScreenCoordsForItem(itm)
			ctx.HID.Click(game.LeftButton, screenPos.X, screenPos.Y)
			utils.Sleep(300)
		}

		// Click on base item to socket the insert
		for _, movedBase := range ctx.Data.Inventory.AllItems {
			if base.UnitID == movedBase.UnitID {
				if !isDLCTab && (base.Location.LocationType == item.LocationStash) && base.Location.Page != itm.Location.Page {
					SwitchStashTab(base.Location.Page + 1)
				}

				basescreenPos := ui.GetScreenCoordsForItem(movedBase)
				ctx.HID.Click(game.LeftButton, basescreenPos.X, basescreenPos.Y)
				utils.Sleep(300)
				if itm.Location.LocationType == item.LocationCursor {
					step.CloseAllMenus()
					DropAndRecoverCursorItem()
					return fmt.Errorf("failed to insert item %s into base %s", itm.Name, base.Name)
				}
			}
		}
		utils.Sleep(300)
	}
	return step.CloseAllMenus()
}

func currentRunewordBaseTier(ctx *context.Status, recipe Runeword, baseType string) (item.Tier, bool) {

	items := ctx.Data.Inventory.ByLocation(
		item.LocationInventory,
		item.LocationEquipped,
		item.LocationStash,
		item.LocationSharedStash,
	)

	for _, itm := range items {
		if itm.RunewordName == recipe.Name && itm.Type().Name == baseType {
			return itm.Desc().Tier(), true
		}
	}
	return 0, false
}

func hasBaseForRunewordRecipe(items []data.Item, recipe Runeword) (data.Item, bool) {
	ctx := context.Get()
	// Determine if this is a leveling character; overrides are ignored for leveling
	// to keep the existing, simpler behavior.
	_, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	isBarbLeveling := ctx.CharacterCfg.Character.Class == "barb_leveling"

	// Look up any per-runeword overrides configured for this character.
	overrides := ctx.CharacterCfg.Game.RunewordOverrides
	ov, hasOverride := overrides[string(recipe.Name)]
	useOverride := !isLevelingChar && hasOverride

	// Runeword maker uses per-runeword overrides only; reroll rules apply during reroll checks.
	effectiveEthMode := ""
	effectiveQualityMode := ""
	effectiveBaseType := ""
	effectiveBaseTier := ""
	effectiveBaseName := ""
	if useOverride && ov.EthMode != "" {
		effectiveEthMode = strings.ToLower(strings.TrimSpace(ov.EthMode))
		if effectiveEthMode == "any" {
			effectiveEthMode = ""
		}
	}
	if useOverride && ov.QualityMode != "" {
		effectiveQualityMode = strings.ToLower(strings.TrimSpace(ov.QualityMode))
		if effectiveQualityMode == "any" {
			effectiveQualityMode = ""
		}
	}
	if useOverride && ov.BaseType != "" {
		effectiveBaseType = strings.TrimSpace(ov.BaseType)
	}
	if useOverride && ov.BaseTier != "" {
		effectiveBaseTier = strings.ToLower(strings.TrimSpace(ov.BaseTier))
	}
	if useOverride && ov.BaseName != "" {
		effectiveBaseName = strings.TrimSpace(ov.BaseName)
	}

	// Auto-select tier based on difficulty if enabled and no manual tier set
	if effectiveBaseTier == "" && ctx.CharacterCfg.Game.RunewordMaker.AutoTierByDifficulty {
		switch ctx.CharacterCfg.Game.Difficulty {
		case difficulty.Normal:
			effectiveBaseTier = "normal"
		case difficulty.Nightmare:
			effectiveBaseTier = "exceptional"
		case difficulty.Hell:
			effectiveBaseTier = "elite"
		}
	}

	var validBases []data.Item
	for _, itm := range items {
		itemType := itm.Type().Code

		isValidType := false
		for _, baseType := range recipe.BaseItemTypes {
			if itemType == baseType {
				isValidType = true
				break
			}
		}
		if !isValidType {
			continue
		}

		// Apply user-specified base type restriction when not leveling.
		// Supports comma-separated list for multiple base types (e.g., "sword,shield" for Spirit)
		if effectiveBaseType != "" {
			allowedTypes := strings.Split(effectiveBaseType, ",")
			typeAllowed := false
			for _, t := range allowedTypes {
				if strings.TrimSpace(t) == itemType {
					typeAllowed = true
					break
				}
			}
			if !typeAllowed {
				continue
			}
		}

		// exception to use only 1-handed maces/clubs for steel/malice/strength for barb leveling
		if isBarbLeveling && (recipe.Name == item.RunewordSteel || recipe.Name == item.RunewordMalice || recipe.Name == item.RunewordStrength) {
			oneHandMaceTypes := []string{item.TypeMace, item.TypeClub}
			if !slices.Contains(oneHandMaceTypes, itemType) {
				continue
			}
			_, hasTwoHandedMin := itm.BaseStats.FindStat(stat.TwoHandedMinDamage, 0)
			_, hasTwoHandedMax := itm.BaseStats.FindStat(stat.TwoHandedMaxDamage, 0)
			if hasTwoHandedMin || hasTwoHandedMax {
				continue
			}
		}

		sockets, found := itm.FindStat(stat.NumSockets, 0)
		if !found || sockets.Value != len(recipe.Runes) {
			continue
		}

		// Eth handling: reroll rules beat overrides; otherwise fall back to the recipe value.
		switch effectiveEthMode {
		case "eth":
			if !itm.Ethereal {
				continue
			}
		case "noneth":
			if itm.Ethereal {
				continue
			}
		default:
			if itm.Ethereal && !recipe.AllowEth {
				continue
			}
		}

		if itm.HasSocketedItems() {
			continue
		}

		// Quality handling: reroll rules beat overrides; otherwise allow <= Superior.
		switch effectiveQualityMode {
		case "normal":
			if itm.Quality != item.QualityNormal {
				continue
			}
		case "superior":
			if itm.Quality != item.QualitySuperior {
				continue
			}
		default:
			if itm.Quality > item.QualitySuperior {
				continue
			}
		}

		// Apply base tier restriction (normal/exceptional/elite) when not leveling.
		if effectiveBaseTier != "" {
			itemTier := itm.Desc().Tier()
			switch effectiveBaseTier {
			case "normal":
				if itemTier != item.TierNormal {
					continue
				}
			case "exceptional":
				if itemTier != item.TierExceptional {
					continue
				}
			case "elite":
				if itemTier != item.TierElite {
					continue
				}
			}
		}

		// BaseName (single NIP code or comma list) only applies outside leveling.
		if effectiveBaseName != "" {
			baseCode := pickit.ToNIPName(itm.Desc().Name)
			if baseCode == "" {
				continue
			}
			allowed := false
			for _, part := range strings.Split(effectiveBaseName, ",") {
				if strings.TrimSpace(part) == baseCode {
					allowed = true
					break
				}
			}
			if !allowed {
				continue
			}
		}

		validBases = append(validBases, itm)
	}

	if len(validBases) == 0 {
		return data.Item{}, false
	}

	sortBases := func() {
		// Try stat-based sorting first if BaseSortOrder is provided
		if len(recipe.BaseSortOrder) > 0 {
			// Find which stats actually exist on at least one base
			var validSortStats []stat.ID
			for _, statID := range recipe.BaseSortOrder {
				for _, base := range validBases {
					if _, found := base.FindStat(statID, 0); found {
						validSortStats = append(validSortStats, statID)
						break
					}
				}
			}

			// If we have valid stats to sort by, use them
			if len(validSortStats) > 0 {

				slices.SortFunc(validBases, func(a, b data.Item) int {
					for _, statID := range validSortStats {
						statA, foundA := a.FindStat(statID, 0)
						statB, foundB := b.FindStat(statID, 0)

						// Skip if neither has this stat
						if !foundA && !foundB {
							continue
						}

						if !foundA {
							return 1 // b comes first
						}
						if !foundB {
							return -1 // a comes first
						}
						if statA.Value != statB.Value {
							return statB.Value - statA.Value // Higher values first
						}
					}
					return 0
				})
				return
			}
		}

		// Fall back to requirement-based sorting
		slices.SortFunc(validBases, func(a, b data.Item) int {
			aTotal := a.Desc().RequiredStrength + a.Desc().RequiredDexterity
			bTotal := b.Desc().RequiredStrength + b.Desc().RequiredDexterity
			return aTotal - bTotal // Lower requirements first
		})
	}

	// Sort the bases
	sortBases()

	// Get the best base
	bestBase := validBases[0]

	return bestBase, true
}

func hasItemsForRunewordRecipe(items []data.Item, recipe Runeword) ([]data.Item, bool) {

	RunewordRecipeItems := make(map[string]int)
	for _, item := range recipe.Runes {
		RunewordRecipeItems[item]++
	}

	itemsForRecipe := []data.Item{}

	for _, item := range items {
		if count, ok := RunewordRecipeItems[string(item.Name)]; ok {
			// DLC stacked items: one entry can satisfy multiple recipe slots
			availableQty := isDLCStackedQuantity(item)
			satisfies := min(availableQty, count)

			for i := 0; i < satisfies; i++ {
				itemsForRecipe = append(itemsForRecipe, item)
			}

			count -= satisfies
			if count == 0 {
				delete(RunewordRecipeItems, string(item.Name))
				if len(RunewordRecipeItems) == 0 {
					return itemsForRecipe, true
				}
			} else {
				RunewordRecipeItems[string(item.Name)] = count
			}
		}
	}

	return nil, false
}

// characterMeetsRequirements checks if the character has enough strength and dexterity to wear an item
func characterMeetsRequirements(ctx *context.Status, itm data.Item) bool {
	strStat, hasStr := ctx.Data.PlayerUnit.BaseStats.FindStat(stat.Strength, 0)
	dexStat, hasDex := ctx.Data.PlayerUnit.BaseStats.FindStat(stat.Dexterity, 0)

	charStr := 0
	charDex := 0
	if hasStr {
		charStr = strStat.Value
	}
	if hasDex {
		charDex = dexStat.Value
	}

	reqStr := itm.Desc().RequiredStrength
	reqDex := itm.Desc().RequiredDexterity

	return charStr >= reqStr && charDex >= reqDex
}
