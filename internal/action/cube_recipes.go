package action

import (
	"slices"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/nip"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type CubeRecipe struct {
	Name             string
	Items            []string
	PurchaseRequired bool
	PurchaseItems    []string
}

var (
	Recipes = []CubeRecipe{

		// Perfects
		{
			Name:  "Perfect Amethyst",
			Items: []string{"FlawlessAmethyst", "FlawlessAmethyst", "FlawlessAmethyst"},
		},
		{
			Name:  "Perfect Diamond",
			Items: []string{"FlawlessDiamond", "FlawlessDiamond", "FlawlessDiamond"},
		},
		{
			Name:  "Perfect Emerald",
			Items: []string{"FlawlessEmerald", "FlawlessEmerald", "FlawlessEmerald"},
		},
		{
			Name:  "Perfect Ruby",
			Items: []string{"FlawlessRuby", "FlawlessRuby", "FlawlessRuby"},
		},
		{
			Name:  "Perfect Sapphire",
			Items: []string{"FlawlessSapphire", "FlawlessSapphire", "FlawlessSapphire"},
		},
		{
			Name:  "Perfect Topaz",
			Items: []string{"FlawlessTopaz", "FlawlessTopaz", "FlawlessTopaz"},
		},
		{
			Name:  "Perfect Skull",
			Items: []string{"FlawlessSkull", "FlawlessSkull", "FlawlessSkull"},
		},

		// Token
		{
			Name:  "Token of Absolution",
			Items: []string{"TwistedEssenceOfSuffering", "ChargedEssenceOfHatred", "BurningEssenceOfTerror", "FesteringEssenceOfDestruction"},
		},

		// Runes
		{
			Name:  "Upgrade El",
			Items: []string{"ElRune", "ElRune", "ElRune"},
		},
		{
			Name:  "Upgrade Eld",
			Items: []string{"EldRune", "EldRune", "EldRune"},
		},
		{
			Name:  "Upgrade Tir",
			Items: []string{"TirRune", "TirRune", "TirRune"},
		},
		{
			Name:  "Upgrade Nef",
			Items: []string{"NefRune", "NefRune", "NefRune"},
		},
		{
			Name:  "Upgrade Eth",
			Items: []string{"EthRune", "EthRune", "EthRune"},
		},
		{
			Name:  "Upgrade Ith",
			Items: []string{"IthRune", "IthRune", "IthRune"},
		},
		{
			Name:  "Upgrade Tal",
			Items: []string{"TalRune", "TalRune", "TalRune"},
		},
		{
			Name:  "Upgrade Ral",
			Items: []string{"RalRune", "RalRune", "RalRune"},
		},
		{
			Name:  "Upgrade Ort",
			Items: []string{"OrtRune", "OrtRune", "OrtRune"},
		},
		{
			Name:  "Upgrade Thul",
			Items: []string{"ThulRune", "ThulRune", "ThulRune", "ChippedTopaz"},
		},
		{
			Name:  "Upgrade Amn",
			Items: []string{"AmnRune", "AmnRune", "AmnRune", "ChippedAmethyst"},
		},
		{
			Name:  "Upgrade Sol",
			Items: []string{"SolRune", "SolRune", "SolRune", "ChippedSapphire"},
		},
		{
			Name:  "Upgrade Shael",
			Items: []string{"ShaelRune", "ShaelRune", "ShaelRune", "ChippedRuby"},
		},
		{
			Name:  "Upgrade Dol",
			Items: []string{"DolRune", "DolRune", "DolRune", "ChippedEmerald"},
		},
		{
			Name:  "Upgrade Hel",
			Items: []string{"HelRune", "HelRune", "HelRune", "ChippedDiamond"},
		},
		{
			Name:  "Upgrade Io",
			Items: []string{"IoRune", "IoRune", "IoRune", "FlawedTopaz"},
		},
		{
			Name:  "Upgrade Lum",
			Items: []string{"LumRune", "LumRune", "LumRune", "FlawedAmethyst"},
		},
		{
			Name:  "Upgrade Ko",
			Items: []string{"KoRune", "KoRune", "KoRune", "FlawedSapphire"},
		},
		{
			Name:  "Upgrade Fal",
			Items: []string{"FalRune", "FalRune", "FalRune", "FlawedRuby"},
		},
		{
			Name:  "Upgrade Lem",
			Items: []string{"LemRune", "LemRune", "LemRune", "FlawedEmerald"},
		},
		{
			Name:  "Upgrade Pul",
			Items: []string{"PulRune", "PulRune", "FlawedDiamond"},
		},
		{
			Name:  "Upgrade Um",
			Items: []string{"UmRune", "UmRune", "Topaz"},
		},
		{
			Name:  "Upgrade Mal",
			Items: []string{"MalRune", "MalRune", "Amethyst"},
		},
		{
			Name:  "Upgrade Ist",
			Items: []string{"IstRune", "IstRune", "Sapphire"},
		},
		{
			Name:  "Upgrade Gul",
			Items: []string{"GulRune", "GulRune", "Ruby"},
		},
		{
			Name:  "Upgrade Vex",
			Items: []string{"VexRune", "VexRune", "Emerald"},
		},
		{
			Name:  "Upgrade Ohm",
			Items: []string{"OhmRune", "OhmRune", "Diamond"},
		},
		{
			Name:  "Upgrade Lo",
			Items: []string{"LoRune", "LoRune", "FlawlessTopaz"},
		},
		{
			Name:  "Upgrade Sur",
			Items: []string{"SurRune", "SurRune", "FlawlessAmethyst"},
		},
		{
			Name:  "Upgrade Ber",
			Items: []string{"BerRune", "BerRune", "FlawlessSapphire"},
		},
		{
			Name:  "Upgrade Jah",
			Items: []string{"JahRune", "JahRune", "FlawlessRuby"},
		},
		{
			Name:  "Upgrade Cham",
			Items: []string{"ChamRune", "ChamRune", "FlawlessEmerald"},
		},

		// add sockets
		{
			Name:  "Add Sockets to Weapon",
			Items: []string{"RalRune", "AmnRune", "PerfectAmethyst", "NormalWeapon"},
		},
		{
			Name:  "Add Sockets to Armor",
			Items: []string{"TalRune", "ThulRune", "PerfectTopaz", "NormalArmor"},
		},
		{
			Name:  "Add Sockets to Helm",
			Items: []string{"RalRune", "ThulRune", "PerfectSapphire", "NormalHelm"},
		},
		{
			Name:  "Add Sockets to Shield",
			Items: []string{"TalRune", "AmnRune", "PerfectRuby", "NormalShield"},
		},

		// Crafting
		{
			Name:  "Reroll GrandCharms",
			Items: []string{"GrandCharm", "Perfect", "Perfect", "Perfect"}, // Special handling in hasItemsForRecipe
		},

		// Caster Amulet
		{
			Name:             "Caster Amulet",
			Items:            []string{"RalRune", "PerfectAmethyst", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Amulet"},
		},

		// Caster Ring
		{
			Name:             "Caster Ring",
			Items:            []string{"AmnRune", "PerfectAmethyst", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Ring"},
		},

		// Caster Belt
		{
			Name:             "Caster Belt",
			Items:            []string{"IthRune", "PerfectAmethyst", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"LightBelt", "SharkskinBelt", "VampirefangBelt"},
		},

		// Caster Boots
		{
			Name:             "Caster Boots",
			Items:            []string{"ThulRune", "PerfectAmethyst", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Boots", "DemonhideBoots", "WyrmhideBoots"},
		},

		// Blood Amulet
		{
			Name:             "Blood Amulet",
			Items:            []string{"AmnRune", "PerfectRuby", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Amulet"},
		},

		// Blood Ring
		{
			Name:             "Blood Ring",
			Items:            []string{"SolRune", "PerfectRuby", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Ring"},
		},

		// Blood Gloves
		{
			Name:             "Blood Gloves",
			Items:            []string{"NefRune", "PerfectRuby", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"HeavyGloves", "SharkskinGloves", "VampireboneGloves"},
		},

		// Blood Boots
		{
			Name:             "Blood Boots",
			Items:            []string{"EthRune", "PerfectRuby", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"LightPlatedBoots", "BattleBoots", "MirroredBoots"},
		},

		// Blood Belt
		{
			Name:             "Blood Belt",
			Items:            []string{"TalRune", "PerfectRuby", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Belt", "MeshBelt", "MithrilCoil"},
		},

		// Blood Helm
		{
			Name:             "Blood Helm",
			Items:            []string{"RalRune", "PerfectRuby", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Helm", "Casque", "Armet"},
		},

		// Blood Armor
		{
			Name:             "Blood Armor",
			Items:            []string{"ThulRune", "PerfectRuby", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"PlateMail", "TemplarPlate", "HellforgePlate"},
		},

		// Blood Weapon
		{
			Name:             "Blood Weapon",
			Items:            []string{"OrtRune", "PerfectRuby", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Axe"},
		},

		// Safety Shield
		{
			Name:             "Safety Shield",
			Items:            []string{"NefRune", "PerfectEmerald", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"KiteShield", "DragonShield", "Monarch"},
		},

		// Safety Armor
		{
			Name:             "Safety Armor",
			Items:            []string{"EthRune", "PerfectEmerald", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"BreastPlate", "Curiass", "GreatHauberk"},
		},

		// Safety Boots
		{
			Name:             "Safety Boots",
			Items:            []string{"OrtRune", "PerfectEmerald", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Greaves", "WarBoots", "MyrmidonBoots"},
		},

		// Safety Gloves
		{
			Name:             "Safety Gloves",
			Items:            []string{"RalRune", "PerfectEmerald", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Gauntlets", "WarGauntlets", "OgreGauntlets"},
		},

		// Safety Belt
		{
			Name:             "Safety Belt",
			Items:            []string{"TalRune", "PerfectEmerald", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Sash", "DemonhideSash", "SpiderwebSash"},
		},

		// Safety Helm
		{
			Name:             "Safety Helm",
			Items:            []string{"IthRune", "PerfectEmerald", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"Crown", "GrandCrown", "Corona"},
		},

		// Hitpower Gloves
		{
			Name:             "Hitpower Gloves",
			Items:            []string{"OrtRune", "PerfectSapphire", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"ChainGloves", "HeavyBracers", "Vambraces"},
		},

		// Hitpower Boots
		{
			Name:             "Hitpower Boots",
			Items:            []string{"RalRune", "PerfectSapphire", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"ChainBoots", "MeshBoots", "Boneweave"},
		},

		// Hitpower Belt
		{
			Name:             "Hitpower Belt",
			Items:            []string{"TalRune", "PerfectSapphire", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"HeavyBelt", "BattleBelt", "TrollBelt"},
		},

		// Hitpower Helm
		{
			Name:             "Hitpower Helm",
			Items:            []string{"NefRune", "PerfectSapphire", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"FullHelm", "Basinet", "GiantConch"},
		},

		// Hitpower Armor
		{
			Name:             "Hitpower Armor",
			Items:            []string{"EthRune", "PerfectSapphire", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"FieldPlate", "Sharktooth", "KrakenShell"},
		},

		// Hitpower Shield
		{
			Name:             "Hitpower Shield",
			Items:            []string{"IthRune", "PerfectSapphire", "Jewel"},
			PurchaseRequired: true,
			PurchaseItems:    []string{"GothicShield", "AncientShield", "Ward"},
		},
	}
)

func CubeRecipes() error {
	ctx := context.Get()
	ctx.SetLastAction("CubeRecipes")

	// If cubing is disabled from settings just return nil
	if !ctx.CharacterCfg.CubeRecipes.Enabled {
		ctx.Logger.Debug("Cube recipes are disabled, skipping")
		return nil
	}

	// Leveling can enable cube recipes before acquiring the cube.
	// Skip recipe processing until the Horadric Cube is available.
	if _, hasCube := ctx.Data.Inventory.Find("HoradricCube", item.LocationInventory, item.LocationStash); !hasCube {
		ctx.Logger.Debug("Cube recipes enabled but Horadric Cube not found, skipping")
		return nil
	}

	// Build location list for material search - include DLC tabs if character has DLC
	locations := []item.LocationType{item.LocationStash, item.LocationSharedStash}
	if ctx.Data.IsDLC() {
		locations = append(locations, item.LocationGemsTab, item.LocationMaterialsTab, item.LocationRunesTab)
	}
	itemsInStash := FilterDLCGhostItems(ctx.Data.Inventory.ByLocation(locations...))
	for _, recipe := range Recipes {
		// Check if the current recipe is Enabled
		if !slices.Contains(ctx.CharacterCfg.CubeRecipes.EnabledRecipes, recipe.Name) {
			// is this really needed ? making huge logs
			//		ctx.Logger.Debug("Cube recipe is not enabled, skipping", "recipe", recipe.Name)
			continue
		}

		ctx.Logger.Debug("Cube recipe is enabled, processing", "recipe", recipe.Name)

		continueProcessing := true
		for continueProcessing {
			if items, hasItems := hasItemsForRecipe(ctx, recipe); hasItems {

				// TODO: Check if we have the items in our storage and if not, purchase them, else take the item from the storage
				if recipe.PurchaseRequired {
					err := GambleSingleItem(recipe.PurchaseItems, item.QualityMagic)
					if err != nil {
						ctx.Logger.Error("Error gambling item, skipping recipe", "error", err, "recipe", recipe.Name)
						break
					}

					purchasedItem := getPurchasedItem(ctx, recipe.PurchaseItems)
					if purchasedItem.Name == "" {
						ctx.Logger.Debug("Could not find purchased item. Skipping recipe", "recipe", recipe.Name)
						break
					}

					// Add the purchased item the list of items to cube
					items = append(items, purchasedItem)
				}

				// Add items to the cube and perform the transmutation
				err := CubeAddItems(items...)
				if err != nil {
					return err
				}
				if err = CubeTransmute(); err != nil {
					return err
				}

				// Get a list of items that are in our inventory
				itemsInInv := ctx.Data.Inventory.ByLocation(item.LocationInventory)

				stashingRequired := false
				stashingGrandCharm := false

				// Check if the items that are not in the protected invetory slots should be stashed
				for _, it := range itemsInInv {
					// If item is not in the protected slots, check if it should be stashed
					if ctx.CharacterCfg.Inventory.InventoryLock[it.Position.Y][it.Position.X] == 1 {
						if it.Name == "Key" || it.IsPotion() || it.Name == item.TomeOfTownPortal || it.Name == item.TomeOfIdentify {
							continue
						}

						shouldStash, _, reason, _ := shouldStashIt(it, false)

						if shouldStash {
							ctx.Logger.Debug("Stashing item after cube recipe.", "item", it.Name, "recipe", recipe.Name, "reason", reason)
							stashingRequired = true
						} else if it.Name == "GrandCharm" {
							ctx.Logger.Debug("Checking if we need to stash a GrandCharm that doesn't match any NIP rules.", "recipe", recipe.Name)
							// Check if we have a GrandCharm in stash that doesn't match any NIP rules
							hasUnmatchedGrandCharm := false
							for _, stashItem := range itemsInStash {
								// Skip non‑magic grand charms (e.g., Gheeds Fortune) when checking for a reroll candidate
								if stashItem.Name == "GrandCharm" && stashItem.Quality == item.QualityMagic {
									if _, result := ctx.CharacterCfg.Runtime.Rules.EvaluateAll(stashItem); result != nip.RuleResultFullMatch {
										hasUnmatchedGrandCharm = true
										break
									}
								}
							}
							if !hasUnmatchedGrandCharm {

								ctx.Logger.Debug("GrandCharm doesn't match any NIP rules and we don't have any in stash to be used for this recipe. Stashing it.", "recipe", recipe.Name)
								stashingRequired = true
								stashingGrandCharm = true

							} else {
								DropInventoryItem(it)
								utils.Sleep(500)
							}
						} else {
							DropInventoryItem(it)
							utils.Sleep(500)
						}
					}
				}

				// Add items to the stash if needed
				if stashingRequired && !stashingGrandCharm {
					_ = Stash(false)
				} else if stashingGrandCharm {
					// Force stashing of the invetory
					_ = Stash(true)
				}

				// Remove or decrement the used items from itemsInStash
				itemsInStash = removeUsedItems(itemsInStash, items)
			} else {
				continueProcessing = false
			}
		}
	}

	return nil
}

func hasItemsForRecipe(ctx *context.Status, recipe CubeRecipe) ([]data.Item, bool) {

	ctx.RefreshGameData()

	// Build location list for material search - include DLC tabs if character has DLC
	locations := []item.LocationType{item.LocationStash, item.LocationSharedStash}
	if ctx.Data.IsDLC() {
		locations = append(locations, item.LocationGemsTab, item.LocationMaterialsTab, item.LocationRunesTab)
	}
	items := FilterDLCGhostItems(ctx.Data.Inventory.ByLocation(locations...))

	if strings.Contains(recipe.Name, "Add Sockets to") {
		return hasItemsForSocketRecipe(ctx, recipe, items)
	}

	// Special handling for "Reroll GrandCharms" recipe
	if recipe.Name == "Reroll GrandCharms" {
		return hasItemsForGrandCharmReroll(ctx, items)
	}

	recipeItems := make(map[string]int)
	for _, item := range recipe.Items {
		recipeItems[item]++
	}

	itemsForRecipe := []data.Item{}

	// Iterate over the items in our stash to see if we have the items for the recipe.
	for _, itm := range items {
		if count, ok := recipeItems[string(itm.Name)]; ok {

			// Let's make sure we don't use an item we don't want to. Add more if needed (depending on the recipes we have)
			if itm.Name == "Jewel" {
				if _, result := ctx.CharacterCfg.Runtime.Rules.EvaluateAll(itm); result == nip.RuleResultFullMatch {
					continue
				}
			}

			// DLC stacked items: one entry can satisfy multiple recipe slots
			// (e.g., a FlawlessDiamond stack of 5 satisfies 3x FlawlessDiamond).
			// Each Ctrl+click in CubeAddItems takes one from the stack.
			availableQty := isDLCStackedQuantity(itm)
			satisfies := min(availableQty, count)

			for i := 0; i < satisfies; i++ {
				itemsForRecipe = append(itemsForRecipe, itm)
			}

			count -= satisfies
			if count == 0 {
				delete(recipeItems, string(itm.Name))
				if len(recipeItems) == 0 {
					return itemsForRecipe, true
				}
			} else {
				recipeItems[string(itm.Name)] = count
			}
		}
	}

	// We don't have all the items for the recipe.
	return nil, false
}

func hasItemsForSocketRecipe(ctx *context.Status, recipe CubeRecipe, items []data.Item) ([]data.Item, bool) {
	ctx.Logger.Debug("Processing socket recipe", "recipe", recipe.Name, "totalItems", len(items))

	recipeItems := make(map[string]int)
	for _, itemName := range recipe.Items {
		recipeItems[itemName]++
	}

	itemsForRecipe := []data.Item{}

	var targetItemTypes []string
	switch recipe.Name {
	case "Add Sockets to Weapon":
		targetItemTypes = []string{
			item.TypeWeapon, item.TypeAxe, item.TypeSword, item.TypeSpear,
			item.TypePolearm, item.TypeMace, item.TypeBow,
			item.TypeWand, item.TypeStaff, item.TypeScepter, item.TypeClub, item.TypeHammer, item.TypeKnife,
			item.TypeCrossbow, item.TypeHandtoHand, item.TypeHandtoHand2, item.TypeOrb,
			item.TypeAmazonBow, item.TypeAmazonSpear,
		}
	case "Add Sockets to Armor":
		targetItemTypes = []string{item.TypeArmor}
	case "Add Sockets to Helm":
		targetItemTypes = []string{
			item.TypeHelm,
			item.TypePrimalHelm, item.TypePelt, item.TypeCirclet,
		}
	case "Add Sockets to Shield":
		targetItemTypes = []string{
			item.TypeShield, item.TypeAuricShields, item.TypeVoodooHeads,
		}
	default:
		return nil, false
	}

	for _, itm := range items {
		itemName := string(itm.Name)

		if count, ok := recipeItems[itemName]; ok {
			// DLC stacked items can satisfy multiple recipe slots
			availableQty := isDLCStackedQuantity(itm)
			satisfies := min(availableQty, count)

			for i := 0; i < satisfies; i++ {
				itemsForRecipe = append(itemsForRecipe, itm)
			}
			count -= satisfies
			if count == 0 {
				delete(recipeItems, itemName)
			} else {
				recipeItems[itemName] = count
			}
		} else {

			specialType := ""
			switch recipe.Name {
			case "Add Sockets to Weapon":
				specialType = "NormalWeapon"
			case "Add Sockets to Armor":
				specialType = "NormalArmor"
			case "Add Sockets to Helm":
				specialType = "NormalHelm"
			case "Add Sockets to Shield":
				specialType = "NormalShield"
			}

			if count, ok := recipeItems[specialType]; ok && isSocketableItemMultiType(itm, targetItemTypes) {
				ctx.Logger.Debug("Found socketable item for recipe", "recipe", recipe.Name, "item", itm.Name, "quality", itm.Quality.ToString(), "ethereal", itm.Ethereal)
				itemsForRecipe = append(itemsForRecipe, itm)
				count--
				if count == 0 {
					delete(recipeItems, specialType)
				} else {
					recipeItems[specialType] = count
				}
			}
		}

		if len(recipeItems) == 0 {
			ctx.Logger.Debug("Socket recipe ready to execute", "recipe", recipe.Name, "itemCount", len(itemsForRecipe))
			return itemsForRecipe, true
		}
	}

	return nil, false
}

func isSocketableItemMultiType(itm data.Item, targetTypes []string) bool {

	excludedItems := []string{
		"Runic Talons",
		"War Scepter",
		"Greater Talons",
		"Caduceus",
		"Divine Scepter",
		"Cedar Staff",
		"Elder Staff",
		"Gnarled Staff",
		"Walking Stick",
	}

	for _, excluded := range excludedItems {
		if string(itm.Name) == excluded {
			return false
		}
	}

	if itm.Quality != item.QualityNormal {
		return false
	}

	if itm.HasSockets || len(itm.Sockets) > 0 {
		return false
	}

	if itm.Desc().MaxSockets == 0 {
		return false
	}

	for _, targetType := range targetTypes {
		if itm.Type().IsType(targetType) {
			return true
		}
	}

	return false
}

func hasItemsForGrandCharmReroll(ctx *context.Status, items []data.Item) ([]data.Item, bool) {
	var grandCharm data.Item
	perfectGems := make([]data.Item, 0, 3)

	for _, itm := range items {
		if itm.Name == "GrandCharm" {
			if _, result := ctx.CharacterCfg.Runtime.Rules.EvaluateAll(itm); result != nip.RuleResultFullMatch && itm.Quality == item.QualityMagic {
				grandCharm = itm
			}
		} else if isPerfectGem(itm) && len(perfectGems) < 3 {
			// Skip perfect amethysts and rubies if configured
			if (ctx.CharacterCfg.CubeRecipes.SkipPerfectAmethysts && itm.Name == "PerfectAmethyst") ||
				(ctx.CharacterCfg.CubeRecipes.SkipPerfectRubies && itm.Name == "PerfectRuby") {
				continue
			}
			// DLC stacked gems: one entry can fill multiple perfect gem slots
			needed := 3 - len(perfectGems)
			availableQty := isDLCStackedQuantity(itm)
			satisfies := min(availableQty, needed)

			for i := 0; i < satisfies; i++ {
				perfectGems = append(perfectGems, itm)
			}
		}

		if grandCharm.Name != "" && len(perfectGems) == 3 {
			return append([]data.Item{grandCharm}, perfectGems...), true
		}
	}

	return nil, false
}

// isDLCStackedQuantity returns how many recipe slots a single item entry can
// satisfy. For DLC tab items it returns StackedQuantity; for regular items it
// returns 1 (each entry is a separate physical item).
func isDLCStackedQuantity(itm data.Item) int {
	switch itm.Location.LocationType {
	case item.LocationGemsTab, item.LocationMaterialsTab, item.LocationRunesTab:
		if itm.StackedQuantity > 0 {
			return itm.StackedQuantity
		}
		return 0 // ghost entry, should have been filtered
	}
	return 1
}

func isPerfectGem(item data.Item) bool {
	perfectGems := []string{"PerfectAmethyst", "PerfectDiamond", "PerfectEmerald", "PerfectRuby", "PerfectSapphire", "PerfectTopaz", "PerfectSkull"}
	for _, gemName := range perfectGems {
		if string(item.Name) == gemName {
			return true
		}
	}
	return false
}

func removeUsedItems(stash []data.Item, usedItems []data.Item) []data.Item {
	remainingItems := make([]data.Item, 0)
	usedItemMap := make(map[string]int)

	// Populate a map with the count of used items
	for _, item := range usedItems {
		usedItemMap[string(item.Name)] += 1 // Assuming 'ID' uniquely identifies items in 'usedItems'
	}

	// Filter the stash by excluding used items based on the count in the map
	for _, item := range stash {
		if count, exists := usedItemMap[string(item.Name)]; exists && count > 0 {
			usedItemMap[string(item.Name)] -= 1
		} else {
			remainingItems = append(remainingItems, item)
		}
	}

	return remainingItems
}

func getPurchasedItem(ctx *context.Status, purchaseItems []string) data.Item {
	itemsInInv := ctx.Data.Inventory.ByLocation(item.LocationInventory)
	for _, citem := range itemsInInv {
		for _, pi := range purchaseItems {
			if string(citem.Name) == pi && citem.Quality == item.QualityMagic {
				return citem
			}
		}
	}
	return data.Item{}
}
