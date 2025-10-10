package server

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/koolo/internal/bot"
	"github.com/hectorgimenez/koolo/internal/config"
)

type IndexData struct {
	ErrorMessage string
	Version      string
	Status       map[string]bot.Stats
	DropCount    map[string]int
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
	ErrorMessage       string
	Supervisor         string
	Config             *config.CharacterCfg
	DayNames           []string
	EnabledRuns        []string
	DisabledRuns       []string
	AvailableTZs       map[int]string
	RecipeList         []string
	RunewordRecipeList []string
}

type ConfigData struct {
	ErrorMessage string
	*config.KooloCfg
}

type AutoSettings struct {
	ErrorMessage string
}
