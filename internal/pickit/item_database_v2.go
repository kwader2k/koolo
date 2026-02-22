package pickit

import (
	"fmt"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data/item"
)

// ItemDatabaseV2 holds all item definitions from d2go
var ItemDatabaseV2 = initializeItemDatabaseV2()

func initializeItemDatabaseV2() map[string]ItemDefinition {
	db := make(map[string]ItemDefinition)

	// Add all items to database
	for _, itm := range getAllItemsV2() {
		db[itm.ID] = itm
	}

	return db
}

// GetItemByIDV2 returns an item definition by ID
func GetItemByIDV2(id string) (ItemDefinition, bool) {
	itm, exists := ItemDatabaseV2[id]
	return itm, exists
}

// GetAllItemsV2 returns all items as a slice
func GetAllItemsV2() []ItemDefinition {
	return getAllItemsV2()
}

// GetItemsByCategory returns items filtered by category
func GetItemsByCategory(category string) []ItemDefinition {
	var results []ItemDefinition
	for _, itm := range ItemDatabaseV2 {
		if itm.Category == category {
			results = append(results, itm)
		}
	}
	return results
}

// getAllItemsV2 returns all item definitions from d2go data
func getAllItemsV2() []ItemDefinition {
	items := []ItemDefinition{}

	// Add unique items from d2go
	items = append(items, getD2GOUniqueItems()...)

	// Add set items
	items = append(items, getD2GOSetItems()...)

	// Add runes
	items = append(items, getD2GORuneItems()...)

	// Add gems
	items = append(items, getD2GOGemItems()...)

	// Add charms
	items = append(items, getD2GOCharmItems()...)

	// Add popular base items
	items = append(items, getD2GOBaseItems()...)

	return items
}

// getD2GOUniqueItems returns all unique items from d2go library
func getD2GOUniqueItems() []ItemDefinition {
	// Map of unique items to their base items (from unique bases.txt)
	uniqueBases := map[string]string{
		"Harlequin Crest": "Shako", "Stone of Jordan": "Ring", "War Traveler": "Battle Boots",
		"Arachnid Mesh": "Spiderweb Sash", "Mara's Kaleidoscope": "Amulet",
		"Skin of the Vipermagi": "Serpentskin Armor", "Shako": "Shako", "Oculus": "Swirling Crystal",
		"Herald of Zakarum": "Gilded Shield", "Stormshield": "Monarch", "Arreat's Face": "Slayer Guard",
		"Titan's Revenge": "Ceremonial Javelin", "Windforce": "Hydra Bow", "Griffon's Eye": "Diadem",
		"Tal Rasha's Guardianship": "Lacquered Plate", "Homunculus": "Hierophant Trophy",
		"Andariel's Visage": "Demonhead", "Crown of Ages": "Corona", "Vampire Gaze": "Grim Helm",
		"Sandstorm Trek": "Scarabshell Boots", "Gore Rider": "War Boots", "Bloodfist": "Heavy Gloves",
		"Magefist": "Light Gauntlets", "Frostburn": "Gauntlets", "Dracul's Grasp": "Vampirebone Gloves",
		"String of Ears": "Demonhide Sash", "Verdungo's Hearty Cord": "Mithril Coil",
		"Razortail": "Sharkskin Belt", "Nightwing's Veil": "Spired Helm", "Jalal's Mane": "Totemic Mask",
		"Raven Frost": "Ring", "Bul-Kathos' Wedding Band": "Ring", "Dwarf Star": "Ring",
		"Nature's Peace": "Ring", "Carrion Wind": "Ring", "The Cat's Eye": "Amulet",
		"Highlord's Wrath": "Amulet", "Atma's Scarab": "Amulet", "The Rising Sun": "Amulet",
		"Seraph's Hymn": "Amulet", "Metalgrid": "Amulet", "Eschuta's Temper": "Eldritch Orb",
		"Death's Fathom": "Dimensional Shard", "Tomb Reaver": "Cryptic Axe", "The Reaper's Toll": "Thresher",
		"Bonehew": "Ogre Axe", "Hone Sundan": "Yari", "Wizardspike": "Bone Knife",
		"Lightsabre": "Phase Blade", "The Grandfather": "Colossus Blade", "Doombringer": "Champion Sword",
		"Azurewrath": "Phase Blade", "Hellfire Torch": "Large Charm", "Annihilus": "Small Charm",
		"Gheed's Fortune": "Grand Charm", "The Gnasher": "Hand Axe", "Deathspade": "Axe",
		"Bladebone": "Double Axe", "Bloodletter": "Gladius", "Coldsteel Eye": "Cutlass",
		"Blade of Ali Baba": "Tulwar", "Gull": "Dagger", "Spineripper": "Poignard",
		"Bartuc's Cut-Throat": "Greater Talons", "The Impaler": "War Spear", "Kelpie Snare": "Fuscina",
		"Soulfeast Tine": "War Fork", "Ribcracker": "Quarterstaff", "Skystrike": "Edge Bow",
		"Kuko Shakaku": "Cedar Bow", "Endlesshail": "Double Bow", "Magewrath": "Rune Bow",
		"Buriza-Do Kyanon": "Ballista", "Peasent Crown": "War Hat", "Rockstopper": "Sallet",
		"Stealskull": "Casque", "Valkyrie Wing": "Winged Helm", "Crown of Thieves": "Grand Crown",
		"Vampiregaze": "Grim Helm", "Blackhorn's Face": "Death Mask", "The Spirit Shroud": "Ghost Armor",
		"Skin of the Flayed One": "Demonhide Armor", "Ironpelt": "Trellised Armor",
		"Spiritforge": "Linked Mail", "Crow Caw": "Tigulated Mail", "Shaftstop": "Mesh Armor",
		"Duriel's Shell": "Cuirass", "Skullder's Ire": "Russet Armor", "Guardian Angel": "Templar Coat",
		"Toothrow": "Sharktooth Armor", "Que-Hegan's Wisdom": "Mage Plate",
		"Moser's Blessed Circle": "Round Shield", "Stormchaser": "Scutum", "Tiamat's Rebuke": "Dragon Shield",
		"Lidless Wall": "Grim Shield", "Venom Grip": "Demonhide Gloves", "Gravepalm": "Sharkskin Gloves",
		"Ghoulhide": "Heavy Bracers", "Lavagout": "Battle Gauntlets", "Hellmouth": "War Gauntlets",
		"Infernostride": "Demonhide Boots", "Waterwalk": "Sharkskin Boots", "Silkweave": "Mesh Boots",
		"Gorerider": "War Boots", "Gloomstrap": "Mesh Belt", "Snowclash": "Battle Belt",
		"Thundergod's Vigor": "War Belt", "Tarnhelm": "Skull Cap", "Nagelring": "Ring",
		"Manald Heal": "Ring", "Chance Guards": "Chain Gloves", "Goblin Toe": "Light Plated Boots",
		"Tearhaunch": "Greaves", "Lenyms Cord": "Sash", "Snakecord": "Plated Belt",
		"Nightsmoke": "Belt", "Goldwrap": "Heavy Belt", "Bladebuckle": "Plated Belt",
		"Nokozan Relic": "Amulet", "The Eye of Etlich": "Amulet", "The Mahim-Oak Curio": "Amulet",
		"Blackoak Shield": "Luna", "Steelshade": "Armet", "Death's Web": "Unearthed Wand",
		"Cerebus' Bite": "Blood Spirit", "Tyrael's Might": "Sacred Armor", "Stoneraven": "Matriarchal Spear",
		"Leviathan": "Kraken Shell", "Wisp Projector": "Ring", "Gargoyle's Bite": "Winged Harpoon",
		"Lacerator": "Winged Axe", "Mang Song's Lesson": "Archon Staff", "Ethereal Edge": "Silver-Edged Axe",
		"Demonhorn's Edge": "Destroyer Helm", "Spiritkeeper": "Earth Spirit", "Hellrack": "Colossus Crossbow",
		"Alma Negra": "Sacred Rondache", "Darkforge Spawn": "Bloodlord Skull", "Widowmaker": "Ward Bow",
		"Bloodraven's Charge": "Matriarchal Bow", "Ghostflame": "Legend Spike", "Shadowkiller": "Battle Cestus",
		"Gimmershred": "Flying Axe", "Windhammer": "Ogre Maul", "Thunderstroke": "Matriarchal Javelin",
		"Demon's Arch": "Balrog Spear", "Boneflame": "Succubus Skull", "Steelpillar": "War Pike",
		"Ormus' Robes": "Dusk Shroud", "Spike Thorn": "Blade Barrier", "Frostwind": "Cryptic Sword",
		"Templar's Might": "Sacred Armor", "Firelizard's Talons": "Feral Claws",
		"Marrowwalk": "Boneweave Boots", "Heaven's Light": "Mighty Scepter",
		"Nosferatu's Coil": "Vampirefang Belt", "Giantskull": "Bone Visage", "Ironward": "War Scepter",
		"Arioc's Needle": "Hyperion Spear", "Cranebeak": "War Axe", "Nord's Tenderizer": "Truncheon",
		"Earthshifter": "Thunder Maul", "Wraithflight": "Ghost Glaive", "Ondal's Wisdom": "Elder Staff",
		"The Redeemer": "Mighty Scepter", "Headhunter's Glory": "Troll Nest", "Steelrend": "Ogre Gauntlets",
		"Veil of Steel": "Spired Helm", "The Gladiator's Bane": "Wire Fleece", "Arkaine's Valor": "Balrog Skin",
		"Messerschmidt's Reaver": "Champion Axe", "Baranar's Star": "Devil Star",
		"Schaefer's Hammer": "Legendary Mallet", "The Cranium Basher": "Thunder Maul",
		"Eaglehorn": "Crusader Bow", "Coldkill": "Hatchet", "Butcher's Pupil": "Cleaver",
		"Islestrike": "Twin Axe", "Pompeii's Wrath": "Crowbill", "Guardian Naga": "Naga",
		"Warlord's Trust": "Military Axe", "Spellsteel": "Bearded Axe", "Stormrider": "Tabar",
		"Boneslayer Blade": "Gothic Axe", "The Minotaur": "Ancient Axe", "Suicide Branch": "Burnt Wand",
		"Carin Shard": "Petrified Wand", "Arm of King Leoric": "Tomb Wand", "Blackhand Key": "Grave Wand",
		"Dark Clan Crusher": "Cudgel", "Zakarum's Hand": "Holy Water Sprinkler",
		"The Fetid Sprinkler": "Holy Water Sprinkler", "Hand of Blessed Light": "Divine Scepter",
		"Fleshrender": "Barbed Club", "Sureshrill Frost": "Flanged Mace", "Moonfall": "Jagged Star",
		"Baezil's Vortex": "Knout", "Earthshaker": "Battle Hammer", "Bloodtree Stump": "War Club",
		"The Gavel of Pain": "Martel de Fer", "Hexfire": "Shamshir", "Ginther's Rift": "Dimensional Blade",
		"Headstriker": "Battle Sword", "Plague Bearer": "Rune Sword", "The Atlantean": "Ancient Sword",
		"Crainte Vomir": "Espandon", "Bing Sz Wang": "Dacian Falx", "The Vile Husk": "Colossus Sword",
		"Cloudcrack": "Gothic Sword", "Todesfaelle Flamme": "Zweihander", "Swordguard": "Executioner Sword",
		"Heart Carver": "Rondel", "Blackbog's Sharp": "Cinquedeas", "Stormspike": "Stiletto",
		"Spire of Honor": "Lance", "The Meat Scraper": "Lochaber Axe", "Blackleach Blade": "Bill",
		"Athena's Wrath": "Battle Scythe", "Pierre Tombale Couant": "Partisan", "Husoldal Evo": "Bec-de-Corbin",
		"Grim's Burning Dead": "Grim Scythe", "Razorswitch": "Jo Staff", "Chromatic Ire": "Cedar Staff",
		"Warpspear": "Gothic Staff", "Skullcollector": "Rune Staff", "Riphook": "Razor Bow",
		"Whichwild String": "Short Battle Bow", "Cliffkiller": "Large Siege Bow", "Godstrike Arch": "Gothic Bow",
		"Langer Briser": "Arbalest", "Pus Spitter": "Siege Crossbow", "Demon Machine": "Chu-Ko-Nu",
		"Darksight Helm": "Basinet", "Atma's Wail": "Embossed Plate", "Black Hades": "Chaos Armor",
		"Corpsemourn": "Ornate Plate", "Visceratuant": "Defender", "Kerke's Sanctuary": "Pavise",
		"Radament's Sphere": "Ancient Shield", "Lance Guard": "Barbed Shield", "Dragonscale": "Zakarum Shield",
		"Steel Carapace": "Shadow Plate", "Medusa's Gaze": "Aegis", "Ravenlore": "Sky Spirit",
		"Boneshade": "Lich Wand", "Flamebellow": "Balrog Blade", "Wolfhowl": "Fury Visor",
		"Spirit Ward": "Ward", "Kira's Guardian": "Tiara", "Stormlash": "Scourge",
		"Halaberd's Reign": "Conqueror Crown", "Fleshripper": "Fanged Knife", "Horizon's Tornado": "Scourge",
		"Stone Crusher": "Legendary Mallet", "Jadetalon": "Wrist Sword", "Shadowdancer": "Myrmidon Greaves",
		"Souldrain": "Vambraces", "Runemaster": "Ettin Axe", "Deathcleaver": "Berserker Axe",
		"Executioner's Justice": "Glorious Axe", "Viperfork": "Mancatcher", "The Scalper": "Francisca",
		"Bloodmoon": "Elegant Blade", "Djinn Slayer": "Ataghan", "Deathbit": "Battle Dart",
		"Warshrike": "Winged Knife", "Gutsiphon": "Demon Crossbow", "Razoredge": "Tomahawk",
		"Demonlimb": "Tyrant Club",
	}

	// Popular uniques from d2go UniqueItems map
	popularUniques := []string{
		"Harlequin Crest", "Stone of Jordan", "War Traveler", "Arachnid Mesh",
		"Mara's Kaleidoscope", "Skin of the Vipermagi", "Shako", "Oculus",
		"Herald of Zakarum", "Stormshield", "Arreat's Face", "Titan's Revenge",
		"Windforce", "Griffon's Eye", "Tal Rasha's Guardianship", "Homunculus",
		"Andariel's Visage", "Crown of Ages", "Vampire Gaze", "Sandstorm Trek",
		"Gore Rider", "Bloodfist", "Magefist", "Frostburn", "Dracul's Grasp",
		"String of Ears", "Verdungo's Hearty Cord", "Razortail", "Nightwing's Veil",
		"Jalal's Mane", "Raven Frost", "Bul-Kathos' Wedding Band", "Dwarf Star",
		"Nature's Peace", "Carrion Wind", "The Cat's Eye", "Highlord's Wrath",
		"Atma's Scarab", "The Rising Sun", "Seraph's Hymn", "Metalgrid",
		"Eschuta's Temper", "Death's Fathom", "Tomb Reaver", "The Reaper's Toll",
		"Bonehew", "Hone Sundan", "Wizardspike", "Lightsabre", "The Grandfather",
		"Doombringer", "Azurewrath", "Hellfire Torch", "Annihilus", "Gheed's Fortune",
		"The Gnasher", "Deathspade", "Bladebone", "Bloodletter", "Coldsteel Eye",
		"Blade of Ali Baba", "Gull", "Spineripper", "Bartuc's Cut-Throat",
		"The Impaler", "Kelpie Snare", "Soulfeast Tine", "Ribcracker", "Skystrike",
		"Kuko Shakaku", "Endlesshail", "Magewrath", "Buriza-Do Kyanon",
		"Peasent Crown", "Rockstopper", "Stealskull", "Valkyrie Wing", "Crown of Thieves",
		"Vampiregaze", "Blackhorn's Face", "The Spirit Shroud", "Skin of the Flayed One",
		"Ironpelt", "Spiritforge", "Crow Caw", "Shaftstop", "Duriel's Shell",
		"Skullder's Ire", "Guardian Angel", "Toothrow", "Que-Hegan's Wisdom",
		"Moser's Blessed Circle", "Stormchaser", "Tiamat's Rebuke", "Lidless Wall",
		"Venom Grip", "Gravepalm", "Ghoulhide", "Lavagout", "Hellmouth",
		"Infernostride", "Waterwalk", "Silkweave", "Gorerider", "Gloomstrap",
		"Snowclash", "Thundergod's Vigor", "Tarnhelm", "Nagelring", "Manald Heal",
		"Chance Guards", "Goblin Toe", "Tearhaunch", "Lenyms Cord", "Snakecord",
		"Nightsmoke", "Goldwrap", "Bladebuckle", "Nokozan Relic", "The Eye of Etlich",
		"The Mahim-Oak Curio", "Blackoak Shield", "Steelshade", "Death's Web",
		"Cerebus' Bite", "Tyrael's Might", "Stoneraven", "Leviathan", "Wisp Projector",
		"Gargoyle's Bite", "Lacerator", "Mang Song's Lesson", "Ethereal Edge",
		"Demonhorn's Edge", "Spiritkeeper", "Hellrack", "Alma Negra", "Darkforge Spawn",
		"Widowmaker", "Bloodraven's Charge", "Ghostflame", "Shadowkiller", "Gimmershred",
		"Windhammer", "Thunderstroke", "Demon's Arch", "Boneflame", "Steelpillar",
		"Ormus' Robes", "Spike Thorn", "Frostwind", "Templar's Might",
		"Firelizard's Talons", "Marrowwalk", "Heaven's Light", "Nosferatu's Coil",
		"Giantskull", "Ironward", "Arioc's Needle", "Cranebeak", "Nord's Tenderizer",
		"Earthshifter", "Wraithflight", "Ondal's Wisdom", "The Redeemer",
		"Headhunter's Glory", "Steelrend", "Veil of Steel", "The Gladiator's Bane",
		"Arkaine's Valor", "Messerschmidt's Reaver", "Baranar's Star", "Schaefer's Hammer",
		"The Cranium Basher", "Eaglehorn", "Coldkill", "Butcher's Pupil", "Islestrike",
		"Pompeii's Wrath", "Guardian Naga", "Warlord's Trust", "Spellsteel", "Stormrider",
		"Boneslayer Blade", "The Minotaur", "Suicide Branch", "Carin Shard",
		"Arm of King Leoric", "Blackhand Key", "Dark Clan Crusher", "Zakarum's Hand",
		"The Fetid Sprinkler", "Hand of Blessed Light", "Fleshrender", "Sureshrill Frost",
		"Moonfall", "Baezil's Vortex", "Earthshaker", "Bloodtree Stump", "The Gavel of Pain",
		"Hexfire", "Ginther's Rift", "Headstriker", "Plague Bearer", "The Atlantean",
		"Crainte Vomir", "Bing Sz Wang", "The Vile Husk", "Cloudcrack", "Todesfaelle Flamme",
		"Swordguard", "Heart Carver", "Blackbog's Sharp", "Stormspike", "Spire of Honor",
		"The Meat Scraper", "Blackleach Blade", "Athena's Wrath", "Pierre Tombale Couant",
		"Husoldal Evo", "Grim's Burning Dead", "Razorswitch", "Chromatic Ire", "Warpspear",
		"Skullcollector", "Riphook", "Whichwild String", "Cliffkiller", "Godstrike Arch",
		"Langer Briser", "Pus Spitter", "Demon Machine", "Darksight Helm", "Atma's Wail",
		"Black Hades", "Corpsemourn", "Visceratuant", "Kerke's Sanctuary", "Radament's Sphere",
		"Lance Guard", "Dragonscale", "Steel Carapace", "Medusa's Gaze", "Ravenlore",
		"Boneshade", "Flamebellow", "Wolfhowl", "Spirit Ward", "Kira's Guardian",
		"Stormlash", "Halaberd's Reign", "Fleshripper", "Horizon's Tornado", "Stone Crusher",
		"Jadetalon", "Shadowdancer", "Souldrain", "Runemaster", "Deathcleaver",
		"Executioner's Justice", "Viperfork", "The Scalper", "Bloodmoon", "Djinn Slayer",
		"Deathbit", "Warshrike", "Gutsiphon", "Razoredge", "Demonlimb",
	}

	var items []ItemDefinition
	for _, uniqueName := range popularUniques {
		nipName := ToNIPName(uniqueName)

		// Get basic stats that most uniques have
		stats := []StatType{}
		commonStats := []string{"defense", "maxhp", "maxmana", "fireresist", "coldresist",
			"lightresist", "poisonresist", "fcr", "fhr", "frw", "ias", "strength", "dexterity"}
		for _, statID := range commonStats {
			if st := GetStatTypeByID(statID); st != nil {
				stats = append(stats, *st)
			}
		}

		// Get base item for this unique (if exists)
		baseItem := ""
		if base, exists := uniqueBases[uniqueName]; exists {
			baseItem = ToNIPName(base)
		}

		items = append(items, ItemDefinition{
			ID:             fmt.Sprintf("unique_%s", nipName),
			Name:           uniqueName,
			NIPName:        nipName,
			InternalName:   nipName,
			Type:           "unique",
			BaseItem:       baseItem,
			Quality:        []item.Quality{item.QualityUnique},
			AvailableStats: stats,
			MaxSockets:     0,
			Ethereal:       false,
			ItemLevel:      1,
			Category:       "Uniques",
			Rarity:         "Unique",
			Description:    fmt.Sprintf("Unique item: %s", uniqueName),
		})
	}

	return items
}

// getD2GOSetItems returns popular set items
func getD2GOSetItems() []ItemDefinition {
	sets := []string{
		// Tal Rasha's Wrappings
		"Tal Rasha's Guardianship", "Tal Rasha's Adjudication", "Tal Rasha's Lidless Eye",
		"Tal Rasha's Horadric Crest", "Tal Rasha's Fine-Spun Cloth",
		// Trang-Oul's Avatar
		"Trang-Oul's Scales", "Trang-Oul's Wing", "Trang-Oul's Claws",
		"Trang-Oul's Girth", "Trang-Oul's Guise",
		// Griswold's Legacy
		"Griswold's Valor", "Griswold's Heart", "Griswold's Redemption", "Griswold's Honor",
		// Immortal King
		"Immortal King's Will", "Immortal King's Soul Cage", "Immortal King's Detail",
		"Immortal King's Forge", "Immortal King's Pillar", "Immortal King's Stone Crusher",
		// Natalya's Odium
		"Natalya's Totem", "Natalya's Shadow", "Natalya's Mark", "Natalya's Soul",
		// Aldur's Watchtower
		"Aldur's Stony Gaze", "Aldur's Deception", "Aldur's Rhythm", "Aldur's Advance",
		// M'avina's Battle Hymn
		"M'avina's True Sight", "M'avina's Embrace", "M'avina's Icy Clutch",
		"M'avina's Tenet", "M'avina's Caster",
		// Naj's Ancient Vestige
		"Naj's Circlet", "Naj's Light Plate", "Naj's Puzzler",
		// Sander's Folly
		"Sander's Paragon", "Sander's Riprap", "Sander's Taboo", "Sander's Superstition",
		// IK partial
		"IK Maul", "IK Armor", "IK Helm", "IK Boots", "IK Belt", "IK Gloves",
	}

	var items []ItemDefinition
	for _, setName := range sets {
		nipName := ToNIPName(setName)

		stats := []StatType{}
		commonStats := []string{"defense", "strength", "dexterity", "fireresist", "coldresist",
			"lightresist", "poisonresist", "maxhp", "maxmana"}
		for _, statID := range commonStats {
			if st := GetStatTypeByID(statID); st != nil {
				stats = append(stats, *st)
			}
		}

		items = append(items, ItemDefinition{
			ID:             fmt.Sprintf("set_%s", nipName),
			Name:           setName,
			NIPName:        nipName,
			InternalName:   nipName,
			Type:           "set",
			Quality:        []item.Quality{item.QualitySet},
			AvailableStats: stats,
			MaxSockets:     0,
			Ethereal:       false,
			ItemLevel:      1,
			Category:       "Sets",
			Rarity:         "Set",
			Description:    fmt.Sprintf("Set item: %s", setName),
		})
	}

	return items
}

// getD2GORuneItems returns all 33 runes
func getD2GORuneItems() []ItemDefinition {
	runes := []struct {
		name     string
		code     string
		level    int
		category string
	}{
		{"El Rune", "r01", 11, "Low Runes"},
		{"Eld Rune", "r02", 11, "Low Runes"},
		{"Tir Rune", "r03", 13, "Low Runes"},
		{"Nef Rune", "r04", 13, "Low Runes"},
		{"Eth Rune", "r05", 15, "Low Runes"},
		{"Ith Rune", "r06", 15, "Low Runes"},
		{"Tal Rune", "r07", 17, "Low Runes"},
		{"Ral Rune", "r08", 19, "Low Runes"},
		{"Ort Rune", "r09", 21, "Low Runes"},
		{"Thul Rune", "r10", 23, "Low Runes"},
		{"Amn Rune", "r11", 25, "Mid Runes"},
		{"Sol Rune", "r12", 27, "Mid Runes"},
		{"Shael Rune", "r13", 29, "Mid Runes"},
		{"Dol Rune", "r14", 31, "Mid Runes"},
		{"Hel Rune", "r15", 0, "Mid Runes"},
		{"Io Rune", "r16", 35, "Mid Runes"},
		{"Lum Rune", "r17", 37, "Mid Runes"},
		{"Ko Rune", "r18", 39, "Mid Runes"},
		{"Fal Rune", "r19", 41, "Mid Runes"},
		{"Lem Rune", "r20", 43, "Mid Runes"},
		{"Pul Rune", "r21", 45, "High Runes"},
		{"Um Rune", "r22", 47, "High Runes"},
		{"Mal Rune", "r23", 49, "High Runes"},
		{"Ist Rune", "r24", 51, "High Runes"},
		{"Gul Rune", "r25", 53, "High Runes"},
		{"Vex Rune", "r26", 55, "High Runes"},
		{"Ohm Rune", "r27", 57, "High Runes"},
		{"Lo Rune", "r28", 59, "High Runes"},
		{"Sur Rune", "r29", 61, "High Runes"},
		{"Ber Rune", "r30", 63, "High Runes"},
		{"Jah Rune", "r31", 65, "High Runes"},
		{"Cham Rune", "r32", 67, "High Runes"},
		{"Zod Rune", "r33", 69, "High Runes"},
	}

	var items []ItemDefinition
	for _, r := range runes {
		items = append(items, ItemDefinition{
			ID:             fmt.Sprintf("rune_%s", ToNIPName(r.name)),
			Name:           r.name,
			NIPName:        ToNIPName(r.name),
			InternalName:   r.code,
			Type:           "rune",
			Quality:        []item.Quality{item.QualityNormal},
			AvailableStats: []StatType{}, // Runes don't have variable stats
			MaxSockets:     0,
			Ethereal:       false,
			ItemLevel:      r.level,
			Category:       "Runes",
			Rarity:         r.category,
			Description:    fmt.Sprintf("%s for runewords and socketing", r.name),
		})
	}

	return items
}

// getD2GOGemItems returns all gem types and qualities
func getD2GOGemItems() []ItemDefinition {
	gemTypes := []string{"Amethyst", "Diamond", "Emerald", "Ruby", "Sapphire", "Topaz", "Skull"}
	gemQualities := []string{"Chipped", "Flawed", "Normal", "Flawless", "Perfect"}

	var items []ItemDefinition
	for _, gemType := range gemTypes {
		for i, quality := range gemQualities {
			name := fmt.Sprintf("%s %s", quality, gemType)
			if quality == "Normal" {
				name = gemType
			}

			items = append(items, ItemDefinition{
				ID:             fmt.Sprintf("gem_%s", ToNIPName(name)),
				Name:           name,
				NIPName:        ToNIPName(name),
				InternalName:   fmt.Sprintf("g%s%d", strings.ToLower(string(gemType[0])), i),
				Type:           "gem",
				Quality:        []item.Quality{item.QualityNormal},
				AvailableStats: []StatType{},
				MaxSockets:     0,
				Ethereal:       false,
				ItemLevel:      1,
				Category:       "Gems",
				Rarity:         quality,
				Description:    fmt.Sprintf("%s for socketing weapons/armor", name),
			})
		}
	}

	return items
}

// getD2GOCharmItems returns charm types
func getD2GOCharmItems() []ItemDefinition {
	charms := []struct {
		name  string
		code  string
		level int
	}{
		{"Small Charm", "cm1", 1},
		{"Large Charm", "cm2", 1},
		{"Grand Charm", "cm3", 1},
	}

	var items []ItemDefinition
	for _, c := range charms {
		stats := []StatType{}
		commonStats := []string{"maxhp", "maxmana", "fireresist", "coldresist",
			"lightresist", "poisonresist", "strength", "dexterity", "fcr", "fhr"}
		for _, statID := range commonStats {
			if st := GetStatTypeByID(statID); st != nil {
				stats = append(stats, *st)
			}
		}

		items = append(items, ItemDefinition{
			ID:             fmt.Sprintf("charm_%s", ToNIPName(c.name)),
			Name:           c.name,
			NIPName:        ToNIPName(c.name),
			InternalName:   c.code,
			Type:           "charm",
			Quality:        []item.Quality{item.QualityMagic, item.QualityRare},
			AvailableStats: stats,
			MaxSockets:     0,
			Ethereal:       false,
			ItemLevel:      c.level,
			Category:       "Charms",
			Rarity:         "Common",
			Description:    fmt.Sprintf("%s for inventory bonuses", c.name),
		})
	}

	return items
}

// getD2GOBaseItems returns popular base items for runewords
func getD2GOBaseItems() []ItemDefinition {
	bases := []struct {
		name        string
		itemType    string
		category    string
		level       int
		sockets     int
		description string
	}{
		// Elite Armors
		{"Dusk Shroud", "armor", "Elite Armor", 65, 4, "Low str elite armor for Enigma/Chains"},
		{"Archon Plate", "armor", "Elite Armor", 63, 4, "Elite armor with moderate str"},
		{"Wire Fleece", "armor", "Exceptional Armor", 29, 4, "Low str light armor"},
		{"Mage Plate", "armor", "Exceptional Armor", 25, 4, "Low str armor for runewords"},
		{"Scarab Husk", "armor", "Elite Armor", 65, 3, "Defense armor for melee"},
		{"Kraken Shell", "armor", "Elite Armor", 71, 4, "High defense elite armor"},
		{"Lacquered Plate", "armor", "Elite Armor", 82, 4, "Highest defense elite armor"},

		// Shields
		{"Monarch", "shield", "Exceptional Shield", 54, 4, "Lowest str shield for Spirit (156 str)"},
		{"Sacred Targe", "shield", "Elite Paladin Shield", 86, 4, "Elite paladin shield"},
		{"Kurast Shield", "shield", "Elite Paladin Shield", 77, 4, "Paladin shield for Spirit"},
		{"Vortex Shield", "shield", "Elite Shield", 82, 4, "High defense shield"},
		{"Zakarum Shield", "shield", "Elite Paladin Shield", 82, 4, "Best paladin shield"},

		// Swords
		{"Phase Blade", "sword", "Elite Sword", 73, 6, "Indestructible sword for Grief"},
		{"Colossus Sword", "sword", "Elite Sword", 63, 6, "High damage elite sword"},
		{"Colossus Blade", "sword", "Elite Sword", 63, 6, "Elite 2H sword for eBotD"},
		{"Crystal Sword", "sword", "Normal Sword", 1, 4, "Popular 4os sword for Spirit"},
		{"Broad Sword", "sword", "Normal Sword", 1, 4, "4os sword for Spirit"},
		{"Dimensional Blade", "sword", "Elite Sword", 61, 4, "Elite fast sword"},

		// Axes
		{"Berserker Axe", "axe", "Elite Axe", 63, 6, "Fast elite axe for Grief/Death"},
		{"Ettin Axe", "axe", "Elite Axe", 60, 5, "Elite axe"},
		{"War Spike", "axe", "Elite Axe", 67, 6, "Slow but strong elite axe"},

		// Polearms (for mercenary)
		{"Thresher", "polearm", "Elite Polearm", 53, 6, "Fast merc polearm for Infinity/Insight"},
		{"Giant Thresher", "polearm", "Elite Polearm", 66, 6, "Fastest elite polearm"},
		{"Colossus Voulge", "polearm", "Elite Polearm", 48, 6, "High damage merc polearm"},
		{"Cryptic Axe", "polearm", "Elite Polearm", 59, 6, "Balanced merc weapon"},
		{"Great Poleaxe", "polearm", "Elite Polearm", 63, 6, "Strong merc polearm"},

		// Spears
		{"Mancatcher", "spear", "Exceptional Spear", 37, 6, "Fast merc spear"},

		// Bows
		{"Grand Matron Bow", "bow", "Elite Bow", 78, 6, "Elite amazon bow for Faith"},
		{"Hydra Bow", "bow", "Elite Bow", 85, 6, "Highest damage bow"},
		{"Mat riarchal Bow", "bow", "Elite Bow", 87, 6, "Best amazon bow"},

		// Staves
		{"Archon Staff", "staff", "Elite Staff", 78, 6, "Elite staff for Insight/Memory"},

		// Maces/Hammers
		{"Scourge", "mace", "Elite Mace", 76, 5, "Fast elite mace for Grief"},
		{"Legendary Mallet", "hammer", "Elite Hammer", 82, 6, "Elite hammer for Dream"},

		// Helms
		{"Demonhead", "helm", "Elite Barb Helm", 63, 3, "Elite barb helm for Delirium"},
		{"Corona", "helm", "Elite Helm", 85, 3, "Highest defense helm"},
		{"Bone Visage", "helm", "Elite Necro Helm", 84, 3, "Elite necro helm"},
		{"Dream Spirit", "helm", "Elite Druid Helm", 67, 3, "Elite druid helm"},
		{"Armet", "helm", "Elite Helm", 68, 3, "High defense elite helm"},
		{"Fury Visor", "helm", "Elite Barb Helm", 79, 3, "Elite barb helm with high defense"},
		{"Destruction Helm", "helm", "Elite Barb Helm", 82, 3, "Elite barb helm"},
		{"Conqueror Crown", "helm", "Elite Barb Helm", 77, 3, "Elite barb helm for runewords"},

		// More Armors
		{"Sacred Armor", "armor", "Elite Armor", 85, 4, "Highest defense elite armor"},
		{"Balrog Skin", "armor", "Elite Armor", 76, 4, "High defense elite armor for Fort/Bramble"},
		{"Hellforge Plate", "armor", "Elite Armor", 78, 4, "High defense armor"},
		{"Shadow Plate", "armor", "Elite Armor", 83, 4, "Very high defense elite armor"},
		{"Wyrmhide", "armor", "Elite Armor", 67, 4, "Mid-tier elite armor"},
		{"Serpentskin Armor", "armor", "Exceptional Armor", 36, 3, "Exceptional light armor"},
		{"Demonhide Armor", "armor", "Exceptional Armor", 37, 3, "Exceptional light armor"},
		{"Sharktooth Armor", "armor", "Exceptional Armor", 39, 3, "Exceptional armor"},
		{"Mesh Armor", "armor", "Exceptional Armor", 45, 3, "Exceptional defense armor"},
		{"Templar Coat", "armor", "Exceptional Armor", 52, 4, "Exceptional high defense armor"},
		{"Great Hauberk", "armor", "Exceptional Armor", 50, 4, "Exceptional armor"},
		{"Boneweave", "armor", "Exceptional Armor", 41, 4, "Exceptional necro armor"},
		{"Loricated Mail", "armor", "Elite Armor", 73, 4, "Elite armor"},
		{"Cuirass", "armor", "Exceptional Armor", 47, 3, "Exceptional armor"},
		{"Russet Armor", "armor", "Exceptional Armor", 49, 4, "Exceptional armor"},

		// More Shields
		{"Ward", "shield", "Elite Necro Shield", 71, 2, "Elite necro shield"},
		{"Grim Shield", "shield", "Elite Necro Shield", 70, 4, "Elite necro shield"},
		{"Troll Nest", "shield", "Elite Barb Shield", 65, 4, "Elite barb shield"},
		{"Blade Barrier", "shield", "Elite Paladin Shield", 80, 4, "Elite paladin shield"},
		{"Sacred Rondache", "shield", "Elite Paladin Shield", 85, 4, "Elite paladin shield"},
		{"Protector Shield", "shield", "Elite Paladin Shield", 69, 4, "Elite paladin shield"},
		{"Gilded Shield", "shield", "Elite Paladin Shield", 85, 4, "Elite paladin shield"},
		{"Royal Shield", "shield", "Elite Paladin Shield", 72, 4, "Elite paladin shield"},
		{"Hyperion", "shield", "Elite Shield", 74, 4, "Elite shield"},
		{"Aegis", "shield", "Elite Shield", 84, 4, "High defense elite shield"},
		{"Heater", "shield", "Exceptional Shield", 37, 4, "Exceptional shield for Spirit"},
		{"Luna", "shield", "Exceptional Shield", 48, 4, "Exceptional shield"},
		{"Pavise", "shield", "Exceptional Shield", 50, 4, "Exceptional shield with high defense"},
		{"Dragon Shield", "shield", "Exceptional Shield", 45, 4, "Exceptional shield"},
		{"Round Shield", "shield", "Exceptional Shield", 35, 4, "Exceptional shield"},

		// More Swords
		{"Conquest Sword", "sword", "Elite Sword", 66, 6, "Elite sword"},
		{"Cryptic Sword", "sword", "Elite Sword", 82, 6, "Elite sword with high damage"},
		{"Mythical Sword", "sword", "Elite Sword", 85, 6, "Elite sword highest tier"},
		{"Legend Sword", "sword", "Elite Sword", 59, 5, "Elite sword"},
		{"Highland Blade", "sword", "Elite Sword", 66, 6, "Elite 2H sword"},
		{"Balrog Blade", "sword", "Elite Sword", 71, 6, "Elite 2H sword"},
		{"Champion Sword", "sword", "Elite Sword", 73, 5, "Elite sword"},
		{"Glorious Sword", "sword", "Elite Sword", 80, 6, "Elite sword"},
		{"Elegant Blade", "sword", "Elite Sword", 63, 4, "Fast elite sword"},
		{"Hydra Edge", "sword", "Elite Sword", 76, 6, "Elite sword"},

		// More Axes
		{"Glorious Axe", "axe", "Elite Axe", 85, 6, "Highest tier elite axe"},
		{"Decapitator", "axe", "Elite Axe", 68, 6, "Elite axe"},
		{"Champion Axe", "axe", "Elite Axe", 82, 6, "Elite axe"},
		{"Silver-Edged Axe", "axe", "Elite Axe", 70, 5, "Elite axe"},
		{"Small Crescent", "axe", "Elite Axe", 64, 3, "Elite throwing axe"},
		{"Ettin Axe", "axe", "Elite Axe", 60, 5, "Elite axe"},

		// More Polearms
		{"War Pike", "polearm", "Elite Polearm", 66, 6, "Elite polearm"},
		{"Ogre Axe", "polearm", "Elite Polearm", 60, 6, "Elite polearm"},
		{"Battle Scythe", "polearm", "Exceptional Polearm", 41, 5, "Exceptional polearm"},
		{"Partizan", "polearm", "Exceptional Polearm", 35, 5, "Exceptional polearm"},
		{"Bec-de-Corbin", "polearm", "Exceptional Polearm", 48, 6, "Exceptional polearm"},
		{"Grim Scythe", "polearm", "Exceptional Polearm", 45, 5, "Exceptional polearm"},
		{"Lochaber Axe", "polearm", "Exceptional Polearm", 38, 5, "Exceptional polearm"},
		{"Bill", "polearm", "Exceptional Polearm", 35, 5, "Exceptional polearm"},

		// Maces/Hammers
		{"Thunder Maul", "hammer", "Elite Hammer", 85, 6, "Highest tier elite maul"},
		{"Ogre Maul", "hammer", "Elite Hammer", 69, 6, "Elite maul"},
		{"Reinforced Mace", "mace", "Elite Mace", 69, 5, "Elite mace"},
		{"Devil Star", "mace", "Elite Mace", 70, 6, "Elite mace"},
		{"Caduceus", "scepter", "Elite Scepter", 85, 5, "Elite scepter"},
		{"Tyrant Club", "club", "Elite Club", 82, 6, "Elite club"},
		{"Barbed Club", "club", "Elite Club", 68, 5, "Elite club"},
		{"Martel de Fer", "hammer", "Elite Hammer", 75, 6, "Elite hammer"},
		{"War Scepter", "scepter", "Elite Scepter", 70, 5, "Elite scepter"},
		{"Mighty Scepter", "scepter", "Elite Scepter", 79, 5, "Elite scepter"},
		{"Divine Scepter", "scepter", "Elite Scepter", 82, 5, "Elite scepter"},
		{"Seraph Rod", "scepter", "Elite Scepter", 68, 5, "Elite scepter"},

		// Bows & Crossbows
		{"Diamond Bow", "bow", "Elite Bow", 72, 6, "Elite bow"},
		{"Crusader Bow", "bow", "Elite Bow", 77, 6, "Elite bow"},
		{"Ward Bow", "bow", "Elite Bow", 72, 6, "Elite bow"},
		{"Matriarchal Bow", "bow", "Elite Bow", 87, 6, "Best amazon bow"},
		{"Colossus Crossbow", "crossbow", "Elite Crossbow", 80, 6, "Elite crossbow"},
		{"Demon Crossbow", "crossbow", "Elite Crossbow", 84, 6, "Elite crossbow"},
		{"Pellet Bow", "bow", "Elite Bow", 69, 6, "Elite bow"},
		{"Gorgon Crossbow", "crossbow", "Elite Crossbow", 71, 6, "Elite crossbow"},

		// Spears & Javelins
		{"Hyperion Spear", "spear", "Elite Spear", 85, 6, "Elite spear"},
		{"Stygian Pike", "spear", "Elite Spear", 75, 6, "Elite spear"},
		{"Balrog Spear", "spear", "Elite Spear", 71, 6, "Elite spear"},
		{"Ghost Spear", "spear", "Elite Spear", 73, 6, "Elite spear"},
		{"War Pike", "spear", "Elite Spear", 66, 6, "Elite spear"},
		{"Hyperion Javelin", "javelin", "Elite Javelin", 85, 6, "Elite javelin"},
		{"Matriarchal Javelin", "javelin", "Elite Javelin", 82, 6, "Elite javelin"},
		{"Stygian Pilum", "javelin", "Elite Javelin", 71, 6, "Elite javelin"},
		{"Balrog Javelin", "javelin", "Elite Javelin", 74, 6, "Elite javelin"},
		{"Ghost Glaive", "javelin", "Elite Javelin", 79, 6, "Elite javelin"},
		{"Ceremonial Javelin", "javelin", "Elite Javelin", 77, 6, "Elite javelin"},

		// Staves & Wands
		{"Unearthed Wand", "wand", "Elite Wand", 86, 2, "Elite necro wand"},
		{"Lich Wand", "wand", "Elite Wand", 72, 2, "Elite necro wand"},
		{"Tomb Wand", "wand", "Elite Wand", 57, 2, "Elite necro wand"},
		{"Grave Wand", "wand", "Elite Wand", 52, 2, "Elite necro wand"},
		{"Bone Wand", "wand", "Exceptional Wand", 30, 2, "Exceptional necro wand"},
		{"Petrified Wand", "wand", "Exceptional Wand", 34, 2, "Exceptional necro wand"},
		{"Elder Staff", "staff", "Elite Staff", 72, 6, "Elite staff"},
		{"Matriarchal Spear", "spear", "Elite Amazon Spear", 87, 6, "Elite amazon spear"},

		// Druid Helms
		{"Earth Spirit", "druidhelm", "Elite Druid Helm", 74, 3, "Elite druid helm"},
		{"Sky Spirit", "druidhelm", "Elite Druid Helm", 78, 3, "Elite druid helm"},
		{"Blood Spirit", "druidhelm", "Elite Druid Helm", 84, 3, "Elite druid helm"},
		{"Sun Spirit", "druidhelm", "Elite Druid Helm", 75, 3, "Elite druid helm"},

		// Barbarian Helms
		{"Slayer Guard", "barbhelm", "Elite Barb Helm", 69, 3, "Elite barb helm"},
		{"Carnage Helm", "barbhelm", "Elite Barb Helm", 75, 3, "Elite barb helm"},
		{"Avenger Guard", "barbhelm", "Elite Barb Helm", 85, 3, "Elite barb helm"},

		// Claws
		{"Runic Talons", "claw", "Elite Claw", 81, 3, "Elite assassin claw"},
		{"Feral Claws", "claw", "Elite Claw", 68, 3, "Elite assassin claw"},
		{"Greater Talons", "claw", "Exceptional Claw", 34, 3, "Exceptional assassin claw"},
		{"Scissors Suwayyah", "claw", "Elite Claw", 73, 3, "Elite assassin claw"},

		// Orbs
		{"Heavenly Stone", "orb", "Elite Orb", 85, 2, "Elite sorc orb"},
		{"Eldritch Orb", "orb", "Elite Orb", 72, 2, "Elite sorc orb"},
		{"Dimensional Shard", "orb", "Elite Orb", 85, 2, "Elite sorc orb"},
		{"Swirling Crystal", "orb", "Exceptional Orb", 51, 2, "Exceptional sorc orb"},

		// Necro Shields
		{"Bloodlord Skull", "necroshield", "Elite Necro Shield", 85, 2, "Elite necro shield"},
		{"Succubus Skull", "necroshield", "Elite Necro Shield", 74, 2, "Elite necro shield"},
		{"Hierophant Trophy", "necroshield", "Elite Necro Shield", 75, 2, "Elite necro shield"},

		// Circlets
		{"Circlet", "circlet", "Circlet", 24, 0, "Rare/crafted circlet base"},
		{"Coronet", "circlet", "Circlet", 52, 0, "Better rare circlet"},
		{"Tiara", "circlet", "Circlet", 70, 0, "Best rare circlet"},
		{"Diadem", "circlet", "Circlet", 85, 0, "Elite circlet with +2 skills"},

		// Boots
		{"Battle Boots", "boots", "Elite Boots", 49, 0, "Elite boots"},
		{"War Boots", "boots", "Elite Boots", 54, 0, "Elite boots"},
		{"Mirrored Boots", "boots", "Elite Boots", 69, 0, "Elite boots"},
		{"Myrmidon Greaves", "boots", "Elite Boots", 85, 0, "Elite boots"},
		{"Scarabshell Boots", "boots", "Elite Boots", 63, 0, "Elite boots"},
		{"Boneweave Boots", "boots", "Elite Boots", 72, 0, "Elite boots"},
		{"Demonhide Boots", "boots", "Exceptional Boots", 36, 0, "Exceptional boots"},
		{"Sharkskin Boots", "boots", "Exceptional Boots", 39, 0, "Exceptional boots"},
		{"Mesh Boots", "boots", "Exceptional Boots", 43, 0, "Exceptional boots"},

		// Gloves
		{"Vambraces", "gloves", "Elite Gloves", 50, 0, "Elite gloves"},
		{"Crusader Gauntlets", "gloves", "Elite Gloves", 68, 0, "Elite gloves"},
		{"Ogre Gauntlets", "gloves", "Elite Gloves", 85, 0, "Elite gloves"},
		{"Vampirebone Gloves", "gloves", "Elite Gloves", 63, 0, "Elite gloves"},
		{"Bramble Mitts", "gloves", "Elite Gloves", 47, 0, "Elite gloves"},
		{"Battle Gauntlets", "gloves", "Elite Gloves", 54, 0, "Elite gloves"},
		{"War Gauntlets", "gloves", "Elite Gloves", 60, 0, "Elite gloves"},
		{"Demonhide Gloves", "gloves", "Exceptional Gloves", 36, 0, "Exceptional gloves"},
		{"Sharkskin Gloves", "gloves", "Exceptional Gloves", 39, 0, "Exceptional gloves"},
		{"Light Gauntlets", "gloves", "Exceptional Gloves", 45, 0, "Exceptional gloves"},
		{"Heavy Gloves", "gloves", "Exceptional Gloves", 7, 0, "Normal gloves"},
		{"Chain Gloves", "gloves", "Exceptional Gloves", 12, 0, "Normal gloves"},

		// Belts
		{"Spiderweb Sash", "belt", "Elite Belt", 50, 0, "Elite belt"},
		{"Mithril Coil", "belt", "Elite Belt", 75, 0, "Elite belt"},
		{"Vampirefang Belt", "belt", "Elite Belt", 68, 0, "Elite belt"},
		{"Troll Belt", "belt", "Elite Belt", 82, 0, "Elite belt"},
		{"Colossus Girdle", "belt", "Elite Belt", 85, 0, "Elite belt"},
		{"Demonhide Sash", "belt", "Exceptional Belt", 36, 0, "Exceptional belt"},
		{"Sharkskin Belt", "belt", "Exceptional Belt", 39, 0, "Exceptional belt"},
		{"Mesh Belt", "belt", "Exceptional Belt", 43, 0, "Exceptional belt"},
		{"Battle Belt", "belt", "Exceptional Belt", 49, 0, "Exceptional belt"},
		{"War Belt", "belt", "Exceptional Belt", 54, 0, "Exceptional belt"},
	}

	var items []ItemDefinition
	for _, b := range bases {
		stats := []StatType{}
		commonStats := []string{"defense", "enhanceddefense", "sockets", "durability"}
		for _, statID := range commonStats {
			if st := GetStatTypeByID(statID); st != nil {
				stats = append(stats, *st)
			}
		}

		items = append(items, ItemDefinition{
			ID:             fmt.Sprintf("base_%s", ToNIPName(b.name)),
			Name:           b.name,
			NIPName:        ToNIPName(b.name),
			InternalName:   ToNIPName(b.name),
			Type:           b.itemType,
			Quality:        []item.Quality{item.QualityNormal, item.QualitySuperior},
			AvailableStats: stats,
			MaxSockets:     b.sockets,
			Ethereal:       true,
			ItemLevel:      b.level,
			Category:       "Bases",
			Rarity:         b.category,
			Description:    b.description,
		})
	}

	return items
}
