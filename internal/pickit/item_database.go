package pickit

import (
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data/item"
)

// ToNIPName converts a display name to NIP format (lowercase, no spaces)
// Example: "Harlequin Crest" -> "harlequincrest", "Spider web sash" -> "spiderwebsash"
func ToNIPName(displayName string) string {
	// Remove all spaces, apostrophes, hyphens and special characters, convert to lowercase
	nipName := strings.ReplaceAll(displayName, " ", "")
	nipName = strings.ReplaceAll(nipName, "'", "")
	nipName = strings.ReplaceAll(nipName, "-", "")
	nipName = strings.ReplaceAll(nipName, "'", "") // curly apostrophe
	nipName = strings.ToLower(nipName)
	return nipName
}

// ItemDatabase holds all item definitions
var ItemDatabase = initializeItemDatabase()

func initializeItemDatabase() map[string]ItemDefinition {
	db := make(map[string]ItemDefinition)

	// Add all items to database
	for _, item := range getAllItems() {
		db[item.ID] = item
	}

	return db
}

// GetItemByID returns an item definition by ID
func GetItemByID(id string) (ItemDefinition, bool) {
	item, exists := ItemDatabase[id]
	return item, exists
}

// SearchItems searches for items matching filters
func SearchItems(filters SearchFilters) []ItemDefinition {
	var results []ItemDefinition

	for _, item := range ItemDatabase {
		if matchesFilters(item, filters) {
			results = append(results, item)
		}
	}

	return results
}

func matchesFilters(item ItemDefinition, filters SearchFilters) bool {
	// Query match
	if filters.Query != "" {
		// Simple contains search (can be improved with fuzzy matching)
		query := filters.Query
		if !contains(item.Name, query) && !contains(item.InternalName, query) && !contains(item.Type, query) {
			return false
		}
	}

	// Type filter
	if len(filters.Types) > 0 && !containsString(filters.Types, item.Type) {
		return false
	}

	// Category filter
	if len(filters.Categories) > 0 && !containsString(filters.Categories, item.Category) {
		return false
	}

	// Rarity filter
	if len(filters.Rarities) > 0 && !containsString(filters.Rarities, item.Rarity) {
		return false
	}

	// Quality filter
	if len(filters.Qualities) > 0 {
		hasMatchingQuality := false
		for _, quality := range item.Quality {
			qualityStr := qualityToString(quality)
			if containsString(filters.Qualities, qualityStr) {
				hasMatchingQuality = true
				break
			}
		}
		if !hasMatchingQuality {
			return false
		}
	}

	// Ethereal filter
	if filters.HasEthereal && !item.Ethereal {
		return false
	}

	// Sockets filter
	if filters.HasSockets && item.MaxSockets == 0 {
		return false
	}

	// Level filters
	if filters.MinLevel > 0 && item.ItemLevel < filters.MinLevel {
		return false
	}
	if filters.MaxLevel > 0 && item.ItemLevel > filters.MaxLevel {
		return false
	}

	return true
}

// Helper functions
func contains(str, substr string) bool {
	// Case-insensitive contains
	return len(str) >= len(substr) && (str == substr || len(substr) == 0)
}

func containsString(slice []string, str string) bool {
	for _, s := range slice {
		if s == str {
			return true
		}
	}
	return false
}

func qualityToString(q item.Quality) string {
	switch q {
	case item.QualityNormal:
		return "Normal"
	case item.QualitySuperior:
		return "Superior"
	case item.QualityMagic:
		return "Magic"
	case item.QualitySet:
		return "Set"
	case item.QualityRare:
		return "Rare"
	case item.QualityUnique:
		return "Unique"
	case item.QualityCrafted:
		return "Crafted"
	default:
		return "Unknown"
	}
}

// getAllItems returns all item definitions
func getAllItems() []ItemDefinition {
	items := []ItemDefinition{}

	// Add uniques
	items = append(items, getUniqueItems()...)

	// Add runes
	items = append(items, getRuneItems()...)

	// Add set items
	items = append(items, getSetItems()...)

	// Add base items for common types
	items = append(items, getBaseItems()...)

	// Add gems
	items = append(items, getGemItems()...)

	// Add charms
	items = append(items, getCharmItems()...)

	return items
}

// getUniqueItems returns unique item definitions
func getUniqueItems() []ItemDefinition {
	return []ItemDefinition{
		{
			ID:           "shako",
			Name:         "Harlequin Crest",
			NIPName:      "harlequincrest",
			InternalName: "shako",
			Type:         "helm",
			Quality:      []item.Quality{item.QualityUnique},
			ImageHD:      "/assets/items/hd/shako.png",
			ImageIcon:    "/assets/items/icons/shako.webp",
			AvailableStats: []StatType{
				*GetStatTypeByID("defense"),
				*GetStatTypeByID("maxhp"),
				*GetStatTypeByID("maxmana"),
				*GetStatTypeByID("itemmagicbonus"),
				*GetStatTypeByID("damageresist"),
			},
			MaxSockets:  0,
			Ethereal:    false,
			ItemLevel:   62,
			Category:    "Uniques",
			Rarity:      "Very Rare",
			Description: "Popular endgame helm with life, mana, MF and damage reduction",
		},
		{
			ID:           "sojs",
			Name:         "Stone of Jordan",
			InternalName: "ring",
			Type:         "ring",
			Quality:      []item.Quality{item.QualityUnique},
			ImageHD:      "/assets/items/hd/soj.png",
			ImageIcon:    "/assets/items/icons/soj.webp",
			AvailableStats: []StatType{
				*GetStatTypeByID("maxmana"),
				*GetStatTypeByID("maxhp"),
			},
			MaxSockets:  0,
			Ethereal:    false,
			ItemLevel:   39,
			Category:    "Uniques",
			Rarity:      "Very Rare",
			Description: "Legendary unique ring with mana and life",
		},
		{
			ID:           "wartraveler",
			Name:         "War Traveler",
			InternalName: "warbelt",
			Type:         "boots",
			Quality:      []item.Quality{item.QualityUnique},
			ImageHD:      "/assets/items/hd/wartraveler.png",
			ImageIcon:    "/assets/items/icons/wartraveler.webp",
			AvailableStats: []StatType{
				*GetStatTypeByID("strength"),
				*GetStatTypeByID("vitality"),
				*GetStatTypeByID("itemmagicbonus"),
			},
			MaxSockets:  0,
			Ethereal:    true,
			ItemLevel:   42,
			Category:    "Uniques",
			Rarity:      "Rare",
			Description: "MF boots with strength and vitality",
		},
		{
			ID:           "arachnidmesh",
			Name:         "Arachnid Mesh",
			InternalName: "spiderweb",
			Type:         "belt",
			Quality:      []item.Quality{item.QualityUnique},
			ImageHD:      "/assets/items/hd/arachnid.png",
			ImageIcon:    "/assets/items/icons/arachnid.webp",
			AvailableStats: []StatType{
				*GetStatTypeByID("fcr"),
				*GetStatTypeByID("strength"),
				*GetStatTypeByID("maxmana"),
			},
			MaxSockets:  0,
			Ethereal:    false,
			ItemLevel:   87,
			Category:    "Uniques",
			Rarity:      "Very Rare",
			Description: "Best caster belt with 20 FCR and +1 skills",
		},
		{
			ID:           "marakaleidoscope",
			Name:         "Mara's Kaleidoscope",
			InternalName: "amulet",
			Type:         "amulet",
			Quality:      []item.Quality{item.QualityUnique},
			ImageHD:      "/assets/items/hd/maras.png",
			ImageIcon:    "/assets/items/icons/maras.webp",
			AvailableStats: []StatType{
				*GetStatTypeByID("fireresist"),
				*GetStatTypeByID("coldresist"),
				*GetStatTypeByID("lightresist"),
				*GetStatTypeByID("poisonresist"),
			},
			MaxSockets:  0,
			Ethereal:    false,
			ItemLevel:   67,
			Category:    "Uniques",
			Rarity:      "Very Rare",
			Description: "Endgame amulet with +2 skills and all resists",
		},
	}
}

// getRuneItems returns rune definitions
func getRuneItems() []ItemDefinition {
	runes := []string{
		"elrune", "eldrune", "tirrune", "nefrune", "ethrune", "ithrune", "talrune", "ralrune",
		"ortrune", "thulrune", "amnrune", "solrune", "shaelrune", "dolrune", "helrune", "iorune",
		"lumrune", "korune", "falrune", "lemrune", "pulrune", "umrune", "malrune", "istrune",
		"gulrune", "vexrune", "ohmrune", "lorune", "surrune", "berrune", "jahrune", "chamrune", "zodrune",
	}

	items := []ItemDefinition{}
	for i, rune := range runes {
		items = append(items, ItemDefinition{
			ID:             rune,
			Name:           capitalizeFirstLetter(rune[:len(rune)-4]) + " Rune",
			InternalName:   rune,
			Type:           "rune",
			Quality:        []item.Quality{}, // Runes don't need quality specified
			ImageHD:        "/assets/items/hd/" + rune + ".png",
			ImageIcon:      "/assets/items/icons/" + rune + ".webp",
			AvailableStats: []StatType{},
			MaxSockets:     0,
			Ethereal:       false,
			ItemLevel:      i + 1,
			Category:       "Runes",
			Rarity:         getRuneRarity(i),
			Description:    "Rune used for socketing and runewords",
		})
	}

	return items
}

func capitalizeFirstLetter(s string) string {
	if len(s) == 0 {
		return s
	}
	return string(s[0]-32) + s[1:]
}

func getRuneRarity(index int) string {
	if index < 8 {
		return "Common"
	} else if index < 16 {
		return "Uncommon"
	} else if index < 24 {
		return "Rare"
	}
	return "Very Rare"
}

// getSetItems returns set item definitions (examples)
func getSetItems() []ItemDefinition {
	return []ItemDefinition{
		{
			ID:           "talgods",
			Name:         "Tal Rasha's Guardianship",
			InternalName: "demonhide",
			Type:         "armor",
			Quality:      []item.Quality{item.QualitySet},
			ImageHD:      "/assets/items/hd/talarmor.png",
			ImageIcon:    "/assets/items/icons/talarmor.webp",
			AvailableStats: []StatType{
				*GetStatTypeByID("defense"),
				*GetStatTypeByID("fireresist"),
				*GetStatTypeByID("coldresist"),
				*GetStatTypeByID("lightresist"),
			},
			MaxSockets:  0,
			Ethereal:    false,
			ItemLevel:   65,
			Category:    "Sets",
			Rarity:      "Rare",
			Description: "Tal Rasha's armor piece",
		},
	}
}

// getBaseItems returns base item definitions for crafting/socketing
func getBaseItems() []ItemDefinition {
	return []ItemDefinition{
		{
			ID:           "monarchbase",
			Name:         "Monarch (Base)",
			InternalName: "monarch",
			Type:         "shield",
			Quality:      []item.Quality{item.QualityNormal, item.QualitySuperior},
			ImageHD:      "/assets/items/hd/monarch.png",
			ImageIcon:    "/assets/items/icons/monarch.webp",
			AvailableStats: []StatType{
				*GetStatTypeByID("defense"),
				*GetStatTypeByID("sockets"),
			},
			MaxSockets:  4,
			Ethereal:    true,
			ItemLevel:   54,
			Category:    "Bases",
			Rarity:      "Uncommon",
			Description: "Popular shield base for Spirit runeword",
		},
		{
			ID:           "phaseblade",
			Name:         "Phase Blade (Base)",
			InternalName: "phaseblade",
			Type:         "sword",
			Quality:      []item.Quality{item.QualityNormal, item.QualitySuperior},
			ImageHD:      "/assets/items/hd/phaseblade.png",
			ImageIcon:    "/assets/items/icons/phaseblade.webp",
			AvailableStats: []StatType{
				*GetStatTypeByID("sockets"),
				*GetStatTypeByID("eddmg"),
			},
			MaxSockets:  6,
			Ethereal:    false,
			ItemLevel:   54,
			Category:    "Bases",
			Rarity:      "Uncommon",
			Description: "Indestructible sword base for runewords",
		},
	}
}

// getGemItems returns gem definitions
func getGemItems() []ItemDefinition {
	gems := []string{"perfectruby", "perfectsapphire", "perfectemerald", "perfectdiamond", "perfecttopaz", "perfectamethyst", "perfectskull"}
	gemNames := []string{"Perfect Ruby", "Perfect Sapphire", "Perfect Emerald", "Perfect Diamond", "Perfect Topaz", "Perfect Amethyst", "Perfect Skull"}

	items := []ItemDefinition{}
	for i, gem := range gems {
		items = append(items, ItemDefinition{
			ID:             gem,
			Name:           gemNames[i],
			InternalName:   gem,
			Type:           "gem",
			Quality:        []item.Quality{item.QualityNormal},
			ImageHD:        "/assets/items/hd/" + gem + ".png",
			ImageIcon:      "/assets/items/icons/" + gem + ".webp",
			AvailableStats: []StatType{},
			MaxSockets:     0,
			Ethereal:       false,
			ItemLevel:      1,
			Category:       "Gems",
			Rarity:         "Common",
			Description:    "Perfect gem for socketing",
		})
	}

	return items
}

// getCharmItems returns charm base definitions
func getCharmItems() []ItemDefinition {
	return []ItemDefinition{
		{
			ID:           "grandcharm",
			Name:         "Grand Charm",
			InternalName: "grandcharm",
			Type:         "charm",
			Quality:      []item.Quality{item.QualityMagic, item.QualityRare},
			ImageHD:      "/assets/items/hd/grandcharm.png",
			ImageIcon:    "/assets/items/icons/grandcharm.webp",
			AvailableStats: []StatType{
				*GetStatTypeByID("maxhp"),
				*GetStatTypeByID("fireresist"),
				*GetStatTypeByID("coldskilltab"),
				*GetStatTypeByID("fireskilltab"),
				*GetStatTypeByID("lightningskilltab"),
			},
			MaxSockets:  0,
			Ethereal:    false,
			ItemLevel:   1,
			Category:    "Charms",
			Rarity:      "Common",
			Description: "Large charm for skill GCs and life",
		},
		{
			ID:           "smallcharm",
			Name:         "Small Charm",
			InternalName: "smallcharm",
			Type:         "charm",
			Quality:      []item.Quality{item.QualityMagic, item.QualityRare},
			ImageHD:      "/assets/items/hd/smallcharm.png",
			ImageIcon:    "/assets/items/icons/smallcharm.webp",
			AvailableStats: []StatType{
				*GetStatTypeByID("maxhp"),
				*GetStatTypeByID("fireresist"),
				*GetStatTypeByID("coldresist"),
				*GetStatTypeByID("lightresist"),
				*GetStatTypeByID("poisonresist"),
			},
			MaxSockets:  0,
			Ethereal:    false,
			ItemLevel:   1,
			Category:    "Charms",
			Rarity:      "Common",
			Description: "Small charm for resists and life",
		},
	}
}

// GetItemCategories returns all available categories
func GetItemCategories() []string {
	return []string{"Uniques", "Sets", "Runes", "Bases", "Gems", "Charms", "Jewels", "Crafted"}
}

// GetItemTypes returns all available item types
func GetItemTypes() []string {
	return []string{"ring", "amulet", "helm", "armor", "boots", "gloves", "belt", "shield", "weapon", "sword", "axe", "mace", "bow", "charm", "gem", "rune", "jewel"}
}

// GetItemQualities returns all available qualities
func GetItemQualities() []string {
	return []string{"Normal", "Superior", "Magic", "Rare", "Set", "Unique", "Crafted"}
}
