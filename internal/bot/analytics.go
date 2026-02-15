package bot

import (
	"time"

	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
)

// RunAnalytics contains detailed metrics for a single run
type RunAnalytics struct {
	// Identity
	ID         string    `json:"id"` // Unique run ID (timestamp-based)
	RunName    string    `json:"runName"`
	Area       area.ID   `json:"area"`
	StartedAt  time.Time `json:"startedAt"`
	FinishedAt time.Time `json:"finishedAt"`
	Duration   int64     `json:"duration"` // milliseconds

	// Outcomes
	Success       bool   `json:"success"`
	FailureReason string `json:"failureReason,omitempty"` // "death", "chicken", "timeout", "error"

	// Combat stats
	MonstersKilled  int  `json:"monstersKilled"`
	ElitesKilled    int  `json:"elitesKilled"`
	ChampionsKilled int  `json:"championsKilled"`
	BossKilled      bool `json:"bossKilled"`
	DeathCount      int  `json:"deathCount"`
	ChickenCount    int  `json:"chickenCount"`

	// Loot stats
	ItemsFound []AnalyticsItemDrop `json:"itemsFound,omitempty"`
	RunesFound []string            `json:"runesFound,omitempty"`

	// Efficiency
	PotionsUsed AnalyticsPotionStats `json:"potionsUsed"`
	TownVisits  int                  `json:"townVisits"`

	// Experience tracking
	StartExperience int64 `json:"startExperience,omitempty"`
	EndExperience   int64 `json:"endExperience,omitempty"`
	ExperienceGain  int64 `json:"experienceGain,omitempty"`

	// Death details for this run
	Deaths []DeathRecord `json:"deaths,omitempty"`

	// Character context
	CharacterName  string                `json:"characterName"`
	CharacterLevel int                   `json:"characterLevel"`
	Difficulty     difficulty.Difficulty `json:"difficulty"`
	GameName       string                `json:"gameName,omitempty"`
}

// AnalyticsItemDrop represents an item found during a run
type AnalyticsItemDrop struct {
	Name          string       `json:"name"`
	BaseName      string       `json:"baseName,omitempty"`
	Quality       item.Quality `json:"quality"`
	Ethereal      bool         `json:"ethereal,omitempty"`
	Sockets       int          `json:"sockets,omitempty"`
	Area          area.ID      `json:"area"`
	RunName       string       `json:"runName"`
	CharacterName string       `json:"characterName"`
	Timestamp     time.Time    `json:"timestamp"`
	Stashed       bool         `json:"stashed"` // Was it kept or sold
	IsRune        bool         `json:"isRune,omitempty"`
	IsUnique      bool         `json:"isUnique,omitempty"`
	IsSet         bool         `json:"isSet,omitempty"`
}

// DeathRecord captures details about a single death
type DeathRecord struct {
	Timestamp time.Time `json:"timestamp"`
	Area      area.ID   `json:"area"`
	AreaName  string    `json:"areaName"`
	RunName   string    `json:"runName"`
	Cause     string    `json:"cause"` // "monster", "environment", "unknown"
	KillerID  string    `json:"killerId,omitempty"`
	HealthPct int       `json:"healthPct,omitempty"` // Health % before death (if known)
}

// AnalyticsPotionStats tracks potion usage
type AnalyticsPotionStats struct {
	Healing      int `json:"healing"`
	Mana         int `json:"mana"`
	Rejuvenation int `json:"rejuvenation"`
}

// SessionSummary provides aggregated stats for a session
type SessionSummary struct {
	CharacterName string    `json:"characterName"`
	StartedAt     time.Time `json:"startedAt"`
	LastUpdated   time.Time `json:"lastUpdated"`
	TotalRuntime  int64     `json:"totalRuntime"` // milliseconds

	// Aggregates
	TotalRuns      int `json:"totalRuns"`
	SuccessfulRuns int `json:"successfulRuns"`
	FailedRuns     int `json:"failedRuns"`
	TotalDeaths    int `json:"totalDeaths"`
	TotalChickens  int `json:"totalChickens"`

	// Monster kills
	TotalMonstersKilled  int `json:"totalMonstersKilled"`
	TotalElitesKilled    int `json:"totalElitesKilled"`
	TotalChampionsKilled int `json:"totalChampionsKilled"`
	TotalBossesKilled    int `json:"totalBossesKilled"`

	// Loot totals
	TotalItemsFound int `json:"totalItemsFound"`
	TotalRunes      int `json:"totalRunes"`

	// Item quality breakdown
	ItemsByQuality ItemQualityBreakdown `json:"itemsByQuality"`

	// Potion usage
	TotalPotionsUsed PotionUsageStats `json:"totalPotionsUsed"`
	TotalTownVisits  int              `json:"totalTownVisits"`

	// Experience tracking
	TotalExperienceGain int64   `json:"totalExperienceGain"`
	ExperiencePerHour   float64 `json:"experiencePerHour"`
	CharacterLevel      int     `json:"characterLevel"`
	CurrentExperience   int64   `json:"currentExperience"`
	LastLevelExperience int64   `json:"lastLevelExperience"` // XP at start of current level
	NextLevelExperience int64   `json:"nextLevelExperience"` // XP needed for next level

	// Death analysis
	DeathsByArea map[string]*AreaDeathStats `json:"deathsByArea,omitempty"`
	RecentDeaths []DeathRecord              `json:"recentDeaths,omitempty"` // Last 20 deaths

	// Per-run-type breakdown
	RunTypeStats map[string]*RunTypeAggregate `json:"runTypeStats"`

	// Hourly breakdown for charts (last 24 hours)
	HourlyStats [24]HourlyBucket `json:"hourlyStats"`

	// Recent notable drops (last 50)
	NotableDrops []AnalyticsItemDrop `json:"notableDrops,omitempty"`

	// Rune tracking - counts per rune name
	RuneBreakdown map[string]int `json:"runeBreakdown,omitempty"`

	// Dry streak - runs since last notable drop
	RunsSinceLastNotable int       `json:"runsSinceLastNotable"`
	LastNotableDropTime  time.Time `json:"lastNotableDropTime,omitempty"`
	LastNotableDropName  string    `json:"lastNotableDropName,omitempty"`
}

// RunTypeAggregate contains stats for a specific run type
type RunTypeAggregate struct {
	RunName        string  `json:"runName"`
	TotalRuns      int     `json:"totalRuns"`
	SuccessfulRuns int     `json:"successfulRuns"`
	FailedRuns     int     `json:"failedRuns"`
	SuccessRate    float64 `json:"successRate"`
	TotalDuration  int64   `json:"totalDuration"` // milliseconds
	AvgDuration    int64   `json:"avgDuration"`   // milliseconds
	FastestRun     int64   `json:"fastestRun"`    // milliseconds
	SlowestRun     int64   `json:"slowestRun"`    // milliseconds
	ItemsFound     int     `json:"itemsFound"`
	ItemsPerRun    float64 `json:"itemsPerRun"`
	Deaths         int     `json:"deaths"`
	DeathsPerRun   float64 `json:"deathsPerRun"`
	Chickens       int     `json:"chickens"`
	// Monster kills per run type
	MonstersKilled  int     `json:"monstersKilled"`
	ElitesKilled    int     `json:"elitesKilled"`
	ChampionsKilled int     `json:"championsKilled"`
	MonstersPerRun  float64 `json:"monstersPerRun"`
	// Experience per run type
	TotalExperience  int64 `json:"totalExperience"`
	ExperiencePerRun int64 `json:"experiencePerRun"`
	// Items by quality for this run type (for heatmap)
	ItemsByQuality ItemQualityBreakdown `json:"itemsByQuality"`
}

// AreaDeathStats tracks death statistics for a specific area
type AreaDeathStats struct {
	AreaID      area.ID `json:"areaId"`
	AreaName    string  `json:"areaName"`
	TotalDeaths int     `json:"totalDeaths"`
	// Cause breakdown
	MonsterDeaths     int `json:"monsterDeaths"`
	EnvironmentDeaths int `json:"environmentDeaths"`
	UnknownDeaths     int `json:"unknownDeaths"`
}

// HourlyBucket contains stats for a single hour
type HourlyBucket struct {
	Hour            int   `json:"hour"` // 0-23
	RunsComplete    int   `json:"runsComplete"`
	ItemsFound      int   `json:"itemsFound"`
	Deaths          int   `json:"deaths"`
	MonstersKilled  int   `json:"monstersKilled"`
	ElitesKilled    int   `json:"elitesKilled"`
	ChampionsKilled int   `json:"championsKilled"`
	ExperienceGain  int64 `json:"experienceGain"`
}

// ItemQualityBreakdown tracks items by quality
type ItemQualityBreakdown struct {
	Normal    int `json:"normal"`
	Magic     int `json:"magic"`
	Rare      int `json:"rare"`
	Set       int `json:"set"`
	Unique    int `json:"unique"`
	Crafted   int `json:"crafted"`
	Runewords int `json:"runewords"`
	Runes     int `json:"runes"`
}

// PotionUsageStats tracks potion consumption
type PotionUsageStats struct {
	Healing      int `json:"healing"`
	Mana         int `json:"mana"`
	Rejuvenation int `json:"rejuvenation"`
	Total        int `json:"total"`
}

// GlobalAnalytics combines stats from all characters
type GlobalAnalytics struct {
	LastUpdated    time.Time                    `json:"lastUpdated"`
	TotalRuntime   int64                        `json:"totalRuntime"` // milliseconds
	TotalRuns      int                          `json:"totalRuns"`
	TotalDeaths    int                          `json:"totalDeaths"`
	TotalItems     int                          `json:"totalItems"`
	CharacterStats map[string]*CharacterSummary `json:"characterStats"`
	RunTypeStats   map[string]*RunTypeAggregate `json:"runTypeStats"`
	HourlyStats    [24]HourlyBucket             `json:"hourlyStats"`
	NotableDrops   []AnalyticsItemDrop          `json:"notableDrops,omitempty"`
}

// CharacterSummary provides a quick overview of a character's stats
type CharacterSummary struct {
	Name         string    `json:"name"`
	Class        string    `json:"class"`
	Level        int       `json:"level"`
	TotalRuns    int       `json:"totalRuns"`
	TotalDeaths  int       `json:"totalDeaths"`
	TotalItems   int       `json:"totalItems"`
	LastActive   time.Time `json:"lastActive"`
	TotalRuntime int64     `json:"totalRuntime"` // milliseconds
	FavoriteRun  string    `json:"favoriteRun,omitempty"`
}

// AnalyticsConfig holds user-configurable analytics settings
type AnalyticsConfig struct {
	HistoryDays       int  `json:"historyDays" yaml:"historyDays"`             // Days of detailed history to keep (default 30)
	MaxNotableDrops   int  `json:"maxNotableDrops" yaml:"maxNotableDrops"`     // Max notable drops to track (default 100)
	TrackAllItems     bool `json:"trackAllItems" yaml:"trackAllItems"`         // Track all items or just stashed ones
	EnablePersistence bool `json:"enablePersistence" yaml:"enablePersistence"` // Save to disk
}

// DefaultAnalyticsConfig returns sensible defaults
func DefaultAnalyticsConfig() AnalyticsConfig {
	return AnalyticsConfig{
		HistoryDays:       30,
		MaxNotableDrops:   100,
		TrackAllItems:     false, // Only track stashed items by default
		EnablePersistence: true,
	}
}

// NewSessionSummary creates a new session summary
func NewSessionSummary(characterName string) *SessionSummary {
	return &SessionSummary{
		CharacterName: characterName,
		StartedAt:     time.Now(),
		LastUpdated:   time.Now(),
		RunTypeStats:  make(map[string]*RunTypeAggregate),
		NotableDrops:  make([]AnalyticsItemDrop, 0),
		RuneBreakdown: make(map[string]int),
		DeathsByArea:  make(map[string]*AreaDeathStats),
	}
}

// NewGlobalAnalytics creates a new global analytics instance
func NewGlobalAnalytics() *GlobalAnalytics {
	return &GlobalAnalytics{
		LastUpdated:    time.Now(),
		CharacterStats: make(map[string]*CharacterSummary),
		RunTypeStats:   make(map[string]*RunTypeAggregate),
		NotableDrops:   make([]AnalyticsItemDrop, 0),
	}
}

// IsNotableDrop determines if an item is worth tracking as notable
func IsNotableDrop(drop AnalyticsItemDrop) bool {
	// Uniques and sets are always notable
	if drop.IsUnique || drop.IsSet {
		return true
	}
	// Runes are notable
	if drop.IsRune {
		return true
	}
	// High quality items with sockets
	if drop.Quality == item.QualityNormal && drop.Sockets >= 4 {
		return true
	}
	return false
}

// SessionHistoryEntry represents a past session for history/comparison
type SessionHistoryEntry struct {
	Date           string  `json:"date"`
	CharacterName  string  `json:"characterName"`
	TotalRuns      int     `json:"totalRuns"`
	SuccessfulRuns int     `json:"successfulRuns"`
	TotalDeaths    int     `json:"totalDeaths"`
	TotalChickens  int     `json:"totalChickens"`
	TotalItems     int     `json:"totalItems"`
	TotalRunes     int     `json:"totalRunes"`
	MonstersKilled int     `json:"monstersKilled"`
	ElitesKilled   int     `json:"elitesKilled"`
	ExperienceGain int64   `json:"experienceGain"`
	XPPerHour      float64 `json:"xpPerHour"`
	RuntimeMinutes float64 `json:"runtimeMinutes"`
	NotableCount   int     `json:"notableCount"`
}

// LevelProgressInfo provides data for the level progress bar
type LevelProgressInfo struct {
	Level        int     `json:"level"`
	CurrentExp   int64   `json:"currentExp"`
	LastLevelExp int64   `json:"lastLevelExp"`
	NextLevelExp int64   `json:"nextLevelExp"`
	ProgressPct  float64 `json:"progressPct"` // 0-100
	ExpToNextLvl int64   `json:"expToNextLvl"`
	ExpPerHour   float64 `json:"expPerHour"`
	EstTimeToLvl string  `json:"estTimeToLvl"` // e.g. "2h 30m"
}

// CharacterComparisonEntry provides per-character stats for side-by-side comparison
type CharacterComparisonEntry struct {
	CharacterName  string  `json:"characterName"`
	TotalRuns      int     `json:"totalRuns"`
	SuccessRate    float64 `json:"successRate"`
	AvgRunTime     float64 `json:"avgRunTime"` // seconds
	TotalDeaths    int     `json:"totalDeaths"`
	MonstersKilled int     `json:"monstersKilled"`
	ElitesKilled   int     `json:"elitesKilled"`
	TotalItems     int     `json:"totalItems"`
	TotalRunes     int     `json:"totalRunes"`
	UniquesFound   int     `json:"uniquesFound"`
	SetsFound      int     `json:"setsFound"`
	ExperienceGain int64   `json:"experienceGain"`
	XPPerHour      float64 `json:"xpPerHour"`
	RuntimeMinutes float64 `json:"runtimeMinutes"`
	BestRunType    string  `json:"bestRunType"`
	ItemsPerRun    float64 `json:"itemsPerRun"`
	DeathsPerRun   float64 `json:"deathsPerRun"`
}
