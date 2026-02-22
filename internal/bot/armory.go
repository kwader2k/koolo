package bot

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/game"
)

// ArmoryItem represents an item with all its properties for display
type ArmoryItem struct {
	ID             int              `json:"id"`
	Name           string           `json:"name"`
	IdentifiedName string           `json:"identifiedName"`
	Quality        string           `json:"quality"`
	QualityInt     int              `json:"qualityInt"`
	Ethereal       bool             `json:"ethereal"`
	Identified     bool             `json:"identified"`
	IsRuneword     bool             `json:"isRuneword"`
	RunewordName   string           `json:"runewordName"`
	LevelReq       int              `json:"levelReq"`
	Position       data.Position    `json:"position"`
	Width          int              `json:"width"`
	Height         int              `json:"height"`
	Location       string           `json:"location"`
	BodyLocation   string           `json:"bodyLocation"`
	StashPage      int              `json:"stashPage"`
	Stats          []ArmoryItemStat `json:"stats"`
	BaseStats      []ArmoryItemStat `json:"baseStats"`
	Sockets        []ArmoryItem     `json:"sockets"`
	HasSockets     bool             `json:"hasSockets"`
	SocketCount    int              `json:"socketCount"`
	ImageName      string           `json:"imageName"`
	ItemType       string           `json:"itemType"`
	Defense        int              `json:"defense"`
	MinDamage      int              `json:"minDamage"`
	MaxDamage      int              `json:"maxDamage"`
	Durability     int              `json:"durability"`
	MaxDurability  int              `json:"maxDurability"`
}

// ArmoryItemStat represents a single stat on an item
type ArmoryItemStat struct {
	ID     int    `json:"id"`
	Name   string `json:"name"`
	Value  int    `json:"value"`
	Layer  int    `json:"layer"`
	String string `json:"string"`
}

// ArmoryCharacter represents the full character data snapshot
type ArmoryCharacter struct {
	CharacterName string       `json:"characterName"`
	Level         int          `json:"level"`
	Class         string       `json:"class"`
	Experience    int          `json:"experience"`
	Gold          int          `json:"gold"`
	StashedGold   [6]int       `json:"stashedGold"`
	DumpTime      time.Time    `json:"dumpTime"`
	GameName      string       `json:"gameName"`
	Equipped      []ArmoryItem `json:"equipped"`
	Inventory     []ArmoryItem `json:"inventory"`
	Stash         []ArmoryItem `json:"stash"`
	SharedStash1  []ArmoryItem `json:"sharedStash1"`
	SharedStash2  []ArmoryItem `json:"sharedStash2"`
	SharedStash3  []ArmoryItem `json:"sharedStash3"`
	SharedStash4  []ArmoryItem `json:"sharedStash4"`
	SharedStash5  []ArmoryItem `json:"sharedStash5"`
	SharedStash6  []ArmoryItem `json:"sharedStash6"` // DLC may have 6th page in memory
	GemsTab       []ArmoryItem `json:"gemsTab"`
	MaterialsTab  []ArmoryItem `json:"materialsTab"`
	RunesTab      []ArmoryItem `json:"runesTab"`
	Cube          []ArmoryItem `json:"cube"`
	Belt          []ArmoryItem `json:"belt"`
	Mercenary     []ArmoryItem `json:"mercenary"`
}

// classToString converts a data.Class to its string representation
func classToString(c data.Class) string {
	switch c {
	case data.Amazon:
		return "Amazon"
	case data.Sorceress:
		return "Sorceress"
	case data.Necromancer:
		return "Necromancer"
	case data.Paladin:
		return "Paladin"
	case data.Barbarian:
		return "Barbarian"
	case data.Druid:
		return "Druid"
	case data.Assassin:
		return "Assassin"
	default:
		return "Unknown"
	}
}

// convertArmoryItem converts a data.Item to an ArmoryItem
func convertArmoryItem(itm data.Item, assetsPath string) ArmoryItem {
	desc := itm.Desc()

	armoryItem := ArmoryItem{
		ID:             itm.ID,
		Name:           string(itm.Name),
		IdentifiedName: itm.IdentifiedName,
		Quality:        itm.Quality.ToString(),
		QualityInt:     int(itm.Quality),
		Ethereal:       itm.Ethereal,
		Identified:     itm.Identified,
		IsRuneword:     itm.IsRuneword,
		RunewordName:   string(itm.RunewordName),
		LevelReq:       itm.LevelReq,
		Position:       itm.Position,
		Width:          desc.InventoryWidth,
		Height:         desc.InventoryHeight,
		Location:       string(itm.Location.LocationType),
		BodyLocation:   string(itm.Location.BodyLocation),
		StashPage:      itm.Location.Page,
		HasSockets:     itm.HasSockets,
		SocketCount:    len(itm.Sockets),
		ImageName:      getArmoryItemImageName(itm, assetsPath),
		ItemType:       desc.Type,
		MinDamage:      desc.MinDamage,
		MaxDamage:      desc.MaxDamage,
	}

	// Convert stats
	for _, s := range itm.Stats {
		armoryItem.Stats = append(armoryItem.Stats, ArmoryItemStat{
			ID:     int(s.ID),
			Name:   getArmoryStatName(s.ID),
			Value:  s.Value,
			Layer:  s.Layer,
			String: s.String(),
		})

		// Extract specific stats
		switch s.ID {
		case stat.Defense:
			armoryItem.Defense = s.Value
		case stat.Durability:
			armoryItem.Durability = s.Value
		case stat.MaxDurability:
			armoryItem.MaxDurability = s.Value
		}
	}

	// Convert base stats
	for _, s := range itm.BaseStats {
		armoryItem.BaseStats = append(armoryItem.BaseStats, ArmoryItemStat{
			ID:     int(s.ID),
			Name:   getArmoryStatName(s.ID),
			Value:  s.Value,
			Layer:  s.Layer,
			String: s.String(),
		})
	}

	// Convert socketed items
	for _, socketedItem := range itm.Sockets {
		armoryItem.Sockets = append(armoryItem.Sockets, convertArmoryItem(socketedItem, assetsPath))
	}

	return armoryItem
}

// getArmoryStatName returns the name of a stat
func getArmoryStatName(id stat.ID) string {
	names := map[stat.ID]string{
		stat.Strength:              "Strength",
		stat.Energy:                "Energy",
		stat.Dexterity:             "Dexterity",
		stat.Vitality:              "Vitality",
		stat.Life:                  "Life",
		stat.MaxLife:               "Max Life",
		stat.Mana:                  "Mana",
		stat.MaxMana:               "Max Mana",
		stat.Defense:               "Defense",
		stat.EnhancedDefense:       "Enhanced Defense",
		stat.EnhancedDamage:        "Enhanced Damage",
		stat.AttackRating:          "Attack Rating",
		stat.FireResist:            "Fire Resist",
		stat.ColdResist:            "Cold Resist",
		stat.LightningResist:       "Lightning Resist",
		stat.PoisonResist:          "Poison Resist",
		stat.MagicFind:             "Magic Find",
		stat.GoldFind:              "Gold Find",
		stat.FasterCastRate:        "Faster Cast Rate",
		stat.FasterHitRecovery:     "Faster Hit Recovery",
		stat.FasterRunWalk:         "Faster Run/Walk",
		stat.IncreasedAttackSpeed:  "Increased Attack Speed",
		stat.LifeSteal:             "Life Steal",
		stat.ManaSteal:             "Mana Steal",
		stat.Durability:            "Durability",
		stat.MaxDurability:         "Max Durability",
		stat.AllSkills:             "All Skills",
		stat.AddClassSkills:        "Class Skills",
		stat.ReplenishLife:         "Replenish Life",
		stat.DamageReduced:         "Damage Reduced",
		stat.MagicDamageReduction:  "Magic Damage Reduction",
		stat.NormalDamageReduction: "Physical Damage Reduction",
		stat.Requirements:          "Requirements",
		stat.ChanceToBlock:         "Chance to Block",
		stat.MinDamage:             "Min Damage",
		stat.MaxDamage:             "Max Damage",
		stat.TwoHandedMinDamage:    "Two-Handed Min Damage",
		stat.TwoHandedMaxDamage:    "Two-Handed Max Damage",
	}

	if name, ok := names[id]; ok {
		return name
	}
	return fmt.Sprintf("Stat_%d", id)
}

// supportedImageExtensions lists image formats in order of preference
var supportedImageExtensions = []string{".webp", ".png", ".jpg", ".jpeg", ".gif"}

// findArmoryImageFile checks for an image file with supported extensions
// Returns the filename with extension if found, empty string otherwise
func findArmoryImageFile(baseName, assetsPath string) string {
	for _, ext := range supportedImageExtensions {
		filename := baseName + ext
		if _, err := os.Stat(filepath.Join(assetsPath, filename)); err == nil {
			return filename
		}
	}
	return ""
}

// getArmoryItemImageName returns the image filename for an item
// Tries to use identified name first, falls back to base item name
// Supports multiple image formats: .webp, .png, .jpg, .jpeg, .gif
func getArmoryItemImageName(itm data.Item, assetsPath string) string {
	// For unique/set items, try identified name first
	if (itm.Quality == item.QualityUnique || itm.Quality == item.QualitySet) && itm.IdentifiedName != "" {
		identifiedName := sanitizeArmoryImageName(itm.IdentifiedName)
		if found := findArmoryImageFile(identifiedName, assetsPath); found != "" {
			return found
		}
	}

	// For runewords, try runeword name first
	if itm.IsRuneword && itm.RunewordName != "" {
		runewordName := sanitizeArmoryImageName(string(itm.RunewordName))
		if found := findArmoryImageFile(runewordName, assetsPath); found != "" {
			return found
		}
	}

	// Fall back to base item name - try to find with any supported extension
	name := sanitizeArmoryImageName(string(itm.Name))
	if found := findArmoryImageFile(name, assetsPath); found != "" {
		return found
	}
	// Default to .webp if no file found (will show missing image)
	return name + ".webp"
}

// sanitizeArmoryImageName converts an item name to a valid image filename
func sanitizeArmoryImageName(name string) string {
	result := ""
	for _, c := range name {
		if (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') {
			result += string(c)
		}
	}
	return result
}

// dumpArmoryData creates a snapshot of the character's inventory and equipment
func dumpArmoryData(characterName string, gameData *game.Data, gameName string) error {
	if gameData == nil {
		return fmt.Errorf("game data is nil")
	}

	// Get the working directory first so we can compute paths
	cwd, err := os.Getwd()
	if err != nil {
		return fmt.Errorf("failed to get working directory: %w", err)
	}

	// Assets path relative to executable (which runs from build/ folder)
	assetsPath := filepath.Join(cwd, "..", "assets", "items")

	levelStat, _ := gameData.PlayerUnit.FindStat(stat.Level, 0)
	expStat, _ := gameData.PlayerUnit.FindStat(stat.Experience, 0)

	armory := ArmoryCharacter{
		CharacterName: characterName,
		Level:         levelStat.Value,
		Class:         classToString(gameData.PlayerUnit.Class),
		Experience:    expStat.Value,
		Gold:          gameData.Inventory.Gold,
		StashedGold:   gameData.Inventory.StashedGold,
		DumpTime:      time.Now(),
		GameName:      gameName,
	}

	// Process items from AllItems
	for _, itm := range gameData.Inventory.AllItems {
		armoryItem := convertArmoryItem(itm, assetsPath)

		switch itm.Location.LocationType {
		case item.LocationEquipped:
			armory.Equipped = append(armory.Equipped, armoryItem)
		case item.LocationInventory:
			armory.Inventory = append(armory.Inventory, armoryItem)
		case item.LocationStash:
			armory.Stash = append(armory.Stash, armoryItem)
		case item.LocationSharedStash:
			switch itm.Location.Page {
			case 1:
				armory.SharedStash1 = append(armory.SharedStash1, armoryItem)
			case 2:
				armory.SharedStash2 = append(armory.SharedStash2, armoryItem)
			case 3:
				armory.SharedStash3 = append(armory.SharedStash3, armoryItem)
			case 4:
				armory.SharedStash4 = append(armory.SharedStash4, armoryItem)
			case 5:
				armory.SharedStash5 = append(armory.SharedStash5, armoryItem)
			case 6:
				armory.SharedStash6 = append(armory.SharedStash6, armoryItem)
			default:
				armory.SharedStash1 = append(armory.SharedStash1, armoryItem)
			}
		case item.LocationGemsTab:
			armory.GemsTab = append(armory.GemsTab, armoryItem)
		case item.LocationMaterialsTab:
			armory.MaterialsTab = append(armory.MaterialsTab, armoryItem)
		case item.LocationRunesTab:
			armory.RunesTab = append(armory.RunesTab, armoryItem)
		case item.LocationCube:
			armory.Cube = append(armory.Cube, armoryItem)
		case item.LocationMercenary:
			armory.Mercenary = append(armory.Mercenary, armoryItem)
		}
	}

	// Belt items are stored separately in Inventory.Belt.Items
	// Belt positions come as linear index (0-15) in X, Y always 0
	// Convert to 4x4 grid: col = index % 4, row = index / 4
	// In-game, bottom row (index 0-3) is consumed first, so we flip Y
	// to display bottom row at the bottom visually
	for _, itm := range gameData.Inventory.Belt.Items {
		armoryItem := convertArmoryItem(itm, assetsPath)
		// Convert linear belt index to 4x4 grid position
		beltIndex := itm.Position.X
		armoryItem.Position = data.Position{
			X: beltIndex % 4,   // Column (0-3)
			Y: 3 - beltIndex/4, // Row flipped: 0->3, 1->2, 2->1, 3->0
		}
		armory.Belt = append(armory.Belt, armoryItem)
	}

	configDir := filepath.Join(cwd, "config", characterName)

	// Make sure directory exists
	if err := os.MkdirAll(configDir, 0755); err != nil {
		return fmt.Errorf("failed to create config directory: %w", err)
	}

	filePath := filepath.Join(configDir, "armory.json")
	jsonData, err := json.MarshalIndent(armory, "", "  ")
	if err != nil {
		return fmt.Errorf("failed to marshal armory data: %w", err)
	}

	if err := os.WriteFile(filePath, jsonData, 0644); err != nil {
		return fmt.Errorf("failed to write armory file: %w", err)
	}

	return nil
}

// LoadArmoryData loads the armory data for a character
func LoadArmoryData(characterName string) (*ArmoryCharacter, error) {
	cwd, err := os.Getwd()
	if err != nil {
		return nil, fmt.Errorf("failed to get working directory: %w", err)
	}

	filePath := filepath.Join(cwd, "config", characterName, "armory.json")

	jsonData, err := os.ReadFile(filePath)
	if err != nil {
		return nil, fmt.Errorf("failed to read armory file: %w", err)
	}

	var armory ArmoryCharacter
	if err := json.Unmarshal(jsonData, &armory); err != nil {
		return nil, fmt.Errorf("failed to unmarshal armory data: %w", err)
	}

	return &armory, nil
}
