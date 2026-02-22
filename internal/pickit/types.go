package pickit

import (
	"github.com/hectorgimenez/d2go/pkg/data/item"
)

// ItemDefinition represents a D2R item with metadata for the pickit editor
type ItemDefinition struct {
	ID             string         `json:"id"`             // Unique identifier (e.g., "shako", "sojs")
	Name           string         `json:"name"`           // Display name (e.g., "Harlequin Crest")
	NIPName        string         `json:"nipName"`        // NIP format name (lowercase, no spaces - e.g., "harlequincrest")
	InternalName   string         `json:"internalName"`   // D2R internal name
	Type           string         `json:"type"`           // Item type (ring, amulet, armor, etc.)
	BaseItem       string         `json:"baseItem"`       // Base item name for uniques (e.g., "battleboots" for War Traveler)
	Quality        []item.Quality `json:"quality"`        // Possible qualities
	ImageHD        string         `json:"imageHD"`        // Path to HD .png image (deprecated, use list UI)
	ImageIcon      string         `json:"imageIcon"`      // Path to icon .webp image (deprecated, use list UI)
	AvailableStats []StatType     `json:"availableStats"` // Stats this item can have
	MaxSockets     int            `json:"maxSockets"`     // Maximum sockets
	Ethereal       bool           `json:"ethereal"`       // Can be ethereal
	ItemLevel      int            `json:"itemLevel"`      // Required item level
	Category       string         `json:"category"`       // Category for filtering (Uniques, Runes, Bases, etc.)
	Rarity         string         `json:"rarity"`         // Rarity tier (Common, Uncommon, Rare, Very Rare)
	Description    string         `json:"description"`    // Item description
}

// StatType represents available stats for items
type StatType struct {
	ID          string  `json:"id"`          // Stat identifier (fcr, strength, maxhp, etc.)
	Name        string  `json:"name"`        // Display name
	NipProperty string  `json:"nipProperty"` // NIP syntax property name
	MinValue    float64 `json:"minValue"`    // Minimum possible value
	MaxValue    float64 `json:"maxValue"`    // Maximum possible value
	IsPercent   bool    `json:"isPercent"`   // Whether value is percentage
}

// PickitRule represents a complete pickit rule
type PickitRule struct {
	ID              string             `json:"id"`              // Unique rule ID
	ItemName        string             `json:"itemName"`        // Item name for display
	ItemID          string             `json:"itemId"`          // Reference to ItemDefinition
	FileName        string             `json:"fileName"`        // File this rule belongs to
	Enabled         bool               `json:"enabled"`         // Whether rule is active
	Priority        int                `json:"priority"`        // Rule priority (1-100)
	LeftConditions  []Condition        `json:"leftConditions"`  // Conditions before #
	RightConditions []Condition        `json:"rightConditions"` // Conditions after # (stats)
	MaxQuantity     int                `json:"maxQuantity"`     // Max items to keep (0 = unlimited)
	IsScored        bool               `json:"isScored"`        // Whether this is a scored rule
	ScoreThreshold  float64            `json:"scoreThreshold"`  // Minimum score required
	ScoreWeights    map[string]float64 `json:"scoreWeights"`    // Stat weights for scoring
	Comments        string             `json:"comments"`        // User comments
	GeneratedNIP    string             `json:"generatedNip"`    // Generated NIP syntax
	CreatedAt       string             `json:"createdAt"`       // Creation timestamp
	UpdatedAt       string             `json:"updatedAt"`       // Last update timestamp
}

// Condition represents a single condition in a rule
type Condition struct {
	Property  string      `json:"property"`  // Property name (name, type, quality, etc.)
	Operator  string      `json:"operator"`  // Operator (==, !=, >=, <=, >, <)
	Value     interface{} `json:"value"`     // Value to compare
	NipSyntax string      `json:"nipSyntax"` // Generated NIP syntax for this condition
}

// PickitFile represents a .nip file with metadata
type PickitFile struct {
	ID           string       `json:"id"`           // Unique file ID
	Name         string       `json:"name"`         // File name (without .nip extension)
	CharacterID  string       `json:"characterId"`  // Character this belongs to
	Category     string       `json:"category"`     // Category (general, runes, bases, leveling, etc.)
	FilePath     string       `json:"filePath"`     // Full file path
	Rules        []PickitRule `json:"rules"`        // Rules in this file
	IsLeveling   bool         `json:"isLeveling"`   // Whether this is a leveling pickit
	Description  string       `json:"description"`  // File description
	RuleCount    int          `json:"ruleCount"`    // Number of rules
	LastModified string       `json:"lastModified"` // Last modification time
}

// PickitSet represents a collection of pickit files
type PickitSet struct {
	ID          string       `json:"id"`          // Unique set ID
	Name        string       `json:"name"`        // Set name
	Description string       `json:"description"` // Set description
	Files       []PickitFile `json:"files"`       // Files in this set
	IsDefault   bool         `json:"isDefault"`   // Whether this is the default set
	Tags        []string     `json:"tags"`        // Tags for categorization
}

// RuleTemplate represents a pre-built rule template
type RuleTemplate struct {
	ID          string     `json:"id"`          // Template ID
	Name        string     `json:"name"`        // Template name
	Category    string     `json:"category"`    // Category (Rings, Amulets, Charms, etc.)
	Description string     `json:"description"` // Template description
	Rule        PickitRule `json:"rule"`        // Base rule to use
	Examples    []string   `json:"examples"`    // Example items that match
}

// ValidationResult represents the result of rule validation
type ValidationResult struct {
	Valid       bool     `json:"valid"`       // Whether rule is valid
	Errors      []string `json:"errors"`      // Validation errors
	Warnings    []string `json:"warnings"`    // Validation warnings
	Suggestions []string `json:"suggestions"` // Improvement suggestions
}

// SimulationResult represents the result of testing a rule
type SimulationResult struct {
	RuleID      string      `json:"ruleId"`      // Rule being tested
	MatchCount  int         `json:"matchCount"`  // Number of items matched
	Matches     []ItemMatch `json:"matches"`     // Matched items
	Misses      []ItemMatch `json:"misses"`      // Items that didn't match
	Performance string      `json:"performance"` // Performance assessment
	Suggestions []string    `json:"suggestions"` // Optimization suggestions
}

// ItemMatch represents an item that matched or didn't match a rule
type ItemMatch struct {
	ItemName  string                 `json:"itemName"`  // Item name
	ImageIcon string                 `json:"imageIcon"` // Icon path
	Matched   bool                   `json:"matched"`   // Whether it matched
	Score     float64                `json:"score"`     // Score if scored rule
	Stats     map[string]interface{} `json:"stats"`     // Item stats
	Reason    string                 `json:"reason"`    // Why it matched/didn't match
}

// StatPreset represents common stat combinations
type StatPreset struct {
	ID          string             `json:"id"`          // Preset ID
	Name        string             `json:"name"`        // Preset name
	ItemType    string             `json:"itemType"`    // Applicable item type
	Description string             `json:"description"` // Preset description
	Stats       map[string]float64 `json:"stats"`       // Stat requirements
	Weights     map[string]float64 `json:"weights"`     // Stat weights for scoring
	Examples    []string           `json:"examples"`    // Example items
}

// ConflictDetection represents detected conflicts between rules
type ConflictDetection struct {
	Type        string   `json:"type"`        // Type of conflict (duplicate, overlapping, etc.)
	Rules       []string `json:"rules"`       // Conflicting rule IDs
	Severity    string   `json:"severity"`    // Severity (warning, error)
	Description string   `json:"description"` // Conflict description
	Suggestion  string   `json:"suggestion"`  // How to resolve
}

// EditorPreferences represents user preferences for the editor
type EditorPreferences struct {
	ViewMode           string   `json:"viewMode"`           // Grid or List
	ShowImages         bool     `json:"showImages"`         // Show item images
	ShowStats          bool     `json:"showStats"`          // Show available stats
	DefaultQuality     []string `json:"defaultQuality"`     // Default quality filters
	FavoriteItems      []string `json:"favoriteItems"`      // Favorited item IDs
	RecentlyEdited     []string `json:"recentlyEdited"`     // Recently edited rules
	AutoSave           bool     `json:"autoSave"`           // Auto-save rules
	ShowAdvanced       bool     `json:"showAdvanced"`       // Show advanced options
	SyntaxHighlighting bool     `json:"syntaxHighlighting"` // Enable syntax highlighting
}

// ExportOptions represents options for exporting pickit files
type ExportOptions struct {
	Format          string   `json:"format"`          // Format (nip, json, yaml)
	IncludeComments bool     `json:"includeComments"` // Include rule comments
	GroupByCategory bool     `json:"groupByCategory"` // Group rules by category
	SortBy          string   `json:"sortBy"`          // Sort order (priority, name, type)
	OnlyEnabled     bool     `json:"onlyEnabled"`     // Only export enabled rules
	Files           []string `json:"files"`           // Specific files to export
}

// ImportOptions represents options for importing pickit files
type ImportOptions struct {
	ReplaceExisting bool   `json:"replaceExisting"` // Replace existing rules
	ValidateOnly    bool   `json:"validateOnly"`    // Only validate, don't import
	TargetFile      string `json:"targetFile"`      // Target file name
	MergeStrategy   string `json:"mergeStrategy"`   // How to merge (replace, append, skip)
}

// SearchFilters represents filters for item search
type SearchFilters struct {
	Query       string   `json:"query"`       // Search query
	Types       []string `json:"types"`       // Item types to include
	Qualities   []string `json:"qualities"`   // Qualities to include
	Categories  []string `json:"categories"`  // Categories to include
	Rarities    []string `json:"rarities"`    // Rarity tiers
	HasEthereal bool     `json:"hasEthereal"` // Can be ethereal
	HasSockets  bool     `json:"hasSockets"`  // Can have sockets
	MinLevel    int      `json:"minLevel"`    // Minimum item level
	MaxLevel    int      `json:"maxLevel"`    // Maximum item level
}

// AutoSuggestion represents an automatic suggestion
type AutoSuggestion struct {
	Type        string      `json:"type"`        // Suggestion type
	Title       string      `json:"title"`       // Suggestion title
	Description string      `json:"description"` // Suggestion description
	Action      string      `json:"action"`      // Suggested action
	Data        interface{} `json:"data"`        // Additional data
	Priority    int         `json:"priority"`    // Suggestion priority
}
