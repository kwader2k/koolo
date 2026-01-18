package server

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/koolo/internal/bot"
	"github.com/hectorgimenez/koolo/internal/config"
)

type TZGroup struct {
	Act           int
	Name          string
	PrimaryAreaID int
	Immunities    []string
	BossPacks     string
	ExpTier       string
	LootTier      string
}

// SchedulerStatusInfo contains scheduler state for UI display
type SchedulerStatusInfo struct {
	Enabled        bool             `json:"enabled"`
	Mode           string           `json:"mode"`
	Phase          string           `json:"phase"`
	PhaseStartTime string           `json:"phaseStartTime"`
	PhaseEndTime   string           `json:"phaseEndTime"`
	TodayWakeTime  string           `json:"todayWakeTime"`
	TodayRestTime  string           `json:"todayRestTime"`
	PlayedMinutes  int              `json:"playedMinutes"`
	NextBreaks     []SchedulerBreak `json:"nextBreaks"`
}

type SchedulerBreak struct {
	Type      string `json:"type"`
	StartTime string `json:"startTime"`
	Duration  int    `json:"duration"`
}

type IndexData struct {
	ErrorMessage                string
	Version                     string
	Status                      map[string]bot.Stats
	DropCount                   map[string]int
	AutoStart                   map[string]bool
	SchedulerStatus             map[string]*SchedulerStatusInfo `json:"schedulerStatus"`
	GlobalAutoStartEnabled      bool
	GlobalAutoStartDelaySeconds int
	ShowAutoStartPrompt         bool
}

type DropData struct {
	NumberOfDrops int
	Character     string
	Drops         []data.Drop
}

// AllDropsData is used by the centralized drops view.
type AllDropsData struct {
	ErrorMessage string
	Total        int
	Records      []AllDropRecord
}

// AllDropRecord flattens droplog.Record for templating.
type AllDropRecord struct {
	Time       string
	Supervisor string
	Character  string
	Profile    string
	Drop       data.Drop
}

type CharacterSettings struct {
	Version                 string
	ErrorMessage            string
	Supervisor              string
	CloneSource             string
	Config                  *config.CharacterCfg
	SkillOptions            []SkillOption
	SkillPrereqs            map[string][]string
	Saved                   bool
	DayNames                []string
	EnabledRuns             []string
	DisabledRuns            []string
	TerrorZoneGroups        []TZGroup
	RecipeList              []string
	RunewordRecipeList      []string
	RunewordFavoriteRecipes []string
	RunFavoriteRuns         []string
	RunewordRuneNames       map[string]string
	RunewordRerollable      map[string]bool
	AvailableProfiles       []string
	FarmerProfiles          []string
	LevelingSequenceFiles   []string
	Supervisors             []string
}

type SkillOption struct {
	Key  string
	Name string
}

type ConfigData struct {
	ErrorMessage string
	*config.KooloCfg
}

type AutoSettings struct {
	ErrorMessage string
}
