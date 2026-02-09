package bot

import (
	"context"
	"encoding/json"
	"fmt"
	"log/slog"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/koolo/internal/event"
)

// AnalyticsManager handles collection and persistence of run analytics
type AnalyticsManager struct {
	mu sync.RWMutex

	// Current state
	characterName  string
	currentSession *SessionSummary
	currentRun     *RunAnalytics
	config         AnalyticsConfig

	// Persistence paths
	basePath      string
	characterPath string
	globalPath    string

	// Global analytics (cross-character)
	globalStats *GlobalAnalytics

	// Run history (recent runs for the current character)
	runHistory []*RunAnalytics

	// Logger
	logger *slog.Logger

	// Broadcast channel for WebSocket updates
	broadcastChan chan<- AnalyticsUpdate
}

// AnalyticsUpdate is sent via WebSocket for real-time dashboard updates
type AnalyticsUpdate struct {
	Type string      `json:"type"` // "run_start", "run_complete", "item_found", "death", "chicken", "session_update"
	Data interface{} `json:"data"`
}

// NewAnalyticsManager creates a new analytics manager
func NewAnalyticsManager(characterName string, basePath string, logger *slog.Logger) *AnalyticsManager {
	am := &AnalyticsManager{
		characterName: characterName,
		config:        DefaultAnalyticsConfig(),
		basePath:      basePath,
		logger:        logger,
		runHistory:    make([]*RunAnalytics, 0),
	}

	am.characterPath = filepath.Join(basePath, "config", characterName, "analytics")
	am.globalPath = filepath.Join(basePath, "config", "analytics")

	// Ensure directories exist
	_ = os.MkdirAll(am.characterPath, 0755)
	_ = os.MkdirAll(am.globalPath, 0755)

	// Load existing data
	am.load()

	return am
}

// SetBroadcastChannel sets the channel for WebSocket updates
func (am *AnalyticsManager) SetBroadcastChannel(ch chan<- AnalyticsUpdate) {
	am.mu.Lock()
	defer am.mu.Unlock()
	am.broadcastChan = ch
}

// broadcast sends an update if a channel is configured
func (am *AnalyticsManager) broadcast(update AnalyticsUpdate) {
	if am.broadcastChan != nil {
		select {
		case am.broadcastChan <- update:
		default:
			// Channel full, skip update to avoid blocking bot
		}
	}
}

// StartRun begins tracking a new run
func (am *AnalyticsManager) StartRun(runName string, currentArea area.ID, charLevel int, diff difficulty.Difficulty, gameName string) {
	am.StartRunWithXP(runName, currentArea, charLevel, diff, gameName, 0)
}

// StartRunWithXP begins tracking a new run with experience tracking
func (am *AnalyticsManager) StartRunWithXP(runName string, currentArea area.ID, charLevel int, diff difficulty.Difficulty, gameName string, currentXP int64) {
	am.mu.Lock()
	defer am.mu.Unlock()

	now := time.Now()
	am.currentRun = &RunAnalytics{
		ID:              fmt.Sprintf("%s-%d", runName, now.UnixMilli()),
		RunName:         runName,
		Area:            currentArea,
		StartedAt:       now,
		CharacterName:   am.characterName,
		CharacterLevel:  charLevel,
		Difficulty:      diff,
		GameName:        gameName,
		ItemsFound:      make([]AnalyticsItemDrop, 0),
		RunesFound:      make([]string, 0),
		Deaths:          make([]DeathRecord, 0),
		StartExperience: currentXP,
	}

	am.broadcast(AnalyticsUpdate{
		Type: "run_start",
		Data: map[string]interface{}{
			"runName":   runName,
			"area":      currentArea.Area().Name,
			"startedAt": now,
		},
	})

	if am.logger != nil {
		am.logger.Debug("Analytics: Run started",
			slog.String("run", runName),
			slog.Int64("startXP", currentXP))
	}
}

// EndRun completes the current run and saves stats
func (am *AnalyticsManager) EndRun(success bool, failureReason string) {
	am.EndRunWithXP(success, failureReason, 0)
}

// EndRunWithXP completes the current run with experience tracking
func (am *AnalyticsManager) EndRunWithXP(success bool, failureReason string, currentXP int64) {
	am.mu.Lock()
	defer am.mu.Unlock()

	if am.currentRun == nil {
		if am.logger != nil {
			am.logger.Debug("Analytics: EndRunWithXP called but no current run")
		}
		return
	}

	now := time.Now()
	am.currentRun.FinishedAt = now
	am.currentRun.Duration = now.Sub(am.currentRun.StartedAt).Milliseconds()
	am.currentRun.Success = success
	am.currentRun.FailureReason = failureReason

	// Calculate XP gain if we have both start and end XP
	if currentXP > 0 && am.currentRun.StartExperience > 0 {
		am.currentRun.EndExperience = currentXP
		am.currentRun.ExperienceGain = currentXP - am.currentRun.StartExperience
		if am.currentRun.ExperienceGain < 0 {
			am.currentRun.ExperienceGain = 0 // Handle level up edge case or death XP loss
		}
	}

	if am.logger != nil {
		am.logger.Debug("Analytics: Run ended",
			slog.String("run", am.currentRun.RunName),
			slog.Int64("startXP", am.currentRun.StartExperience),
			slog.Int64("endXP", currentXP),
			slog.Int64("gain", am.currentRun.ExperienceGain))
	}

	// Add to history
	am.runHistory = append(am.runHistory, am.currentRun)

	// Update session summary
	am.updateSessionFromRun(am.currentRun)

	// Update global stats
	am.updateGlobalFromRun(am.currentRun)

	// Broadcast completion
	am.broadcast(AnalyticsUpdate{
		Type: "run_complete",
		Data: map[string]interface{}{
			"runName":  am.currentRun.RunName,
			"success":  success,
			"duration": am.currentRun.Duration,
			"items":    len(am.currentRun.ItemsFound),
			"monsters": am.currentRun.MonstersKilled,
		},
	})

	if am.logger != nil {
		am.logger.Debug("Analytics: Run completed",
			slog.String("run", am.currentRun.RunName),
			slog.Bool("success", success),
			slog.Int64("duration_ms", am.currentRun.Duration),
		)
	}

	// Save periodically (every run)
	go am.Save()

	am.currentRun = nil
}

// RecordKill increments monster kill counters
func (am *AnalyticsManager) RecordKill(isElite bool, isChampion bool, isBoss bool) {
	am.mu.Lock()
	defer am.mu.Unlock()

	if am.currentRun == nil {
		return
	}

	am.currentRun.MonstersKilled++
	if isElite {
		am.currentRun.ElitesKilled++
	}
	if isChampion {
		am.currentRun.ChampionsKilled++
	}
	if isBoss {
		am.currentRun.BossKilled = true
	}
}

// RecordDeath increments death counter (legacy, no area info)
func (am *AnalyticsManager) RecordDeath() {
	am.RecordDeathWithDetails(0, "", "unknown")
}

// RecordDeathWithDetails records a death with area and cause information
func (am *AnalyticsManager) RecordDeathWithDetails(areaID area.ID, killerName string, cause string) {
	am.mu.Lock()
	defer am.mu.Unlock()

	now := time.Now()
	areaName := ""
	if areaID > 0 {
		areaName = areaID.Area().Name
	}

	// Create death record
	death := DeathRecord{
		Timestamp: now,
		Area:      areaID,
		AreaName:  areaName,
		RunName:   am.getRunName(),
		Cause:     cause,
		KillerID:  killerName,
	}

	if am.currentRun != nil {
		am.currentRun.DeathCount++
		am.currentRun.Deaths = append(am.currentRun.Deaths, death)
	}

	if am.currentSession != nil {
		am.currentSession.TotalDeaths++

		// Update deaths by area
		if am.currentSession.DeathsByArea == nil {
			am.currentSession.DeathsByArea = make(map[string]*AreaDeathStats)
		}

		areaKey := areaName
		if areaKey == "" {
			areaKey = "Unknown"
		}

		if am.currentSession.DeathsByArea[areaKey] == nil {
			am.currentSession.DeathsByArea[areaKey] = &AreaDeathStats{
				AreaID:   areaID,
				AreaName: areaKey,
			}
		}

		stats := am.currentSession.DeathsByArea[areaKey]
		stats.TotalDeaths++
		switch cause {
		case "monster":
			stats.MonsterDeaths++
		case "environment":
			stats.EnvironmentDeaths++
		default:
			stats.UnknownDeaths++
		}

		// Keep recent deaths (last 20)
		am.currentSession.RecentDeaths = append(am.currentSession.RecentDeaths, death)
		if len(am.currentSession.RecentDeaths) > 20 {
			am.currentSession.RecentDeaths = am.currentSession.RecentDeaths[1:]
		}
	}

	am.broadcast(AnalyticsUpdate{
		Type: "death",
		Data: map[string]interface{}{
			"timestamp": now,
			"runName":   am.getRunName(),
			"area":      areaName,
			"cause":     cause,
		},
	})
}

// RecordChicken increments chicken counter
func (am *AnalyticsManager) RecordChicken(reason string) {
	am.mu.Lock()
	defer am.mu.Unlock()

	if am.currentRun != nil {
		am.currentRun.ChickenCount++
	}

	if am.currentSession != nil {
		am.currentSession.TotalChickens++
	}

	am.broadcast(AnalyticsUpdate{
		Type: "chicken",
		Data: map[string]interface{}{
			"timestamp": time.Now(),
			"reason":    reason,
			"runName":   am.getRunName(),
		},
	})
}

// RecordItem tracks an item pickup
func (am *AnalyticsManager) RecordItem(itm data.Item, stashed bool) {
	am.mu.Lock()
	defer am.mu.Unlock()

	// Check if item is a rune by checking its type
	isRune := itm.Type().IsType(item.TypeRune)
	desc := itm.Desc()

	drop := AnalyticsItemDrop{
		Name:          string(itm.Name),
		Quality:       itm.Quality,
		Ethereal:      itm.Ethereal,
		Sockets:       len(itm.Sockets),
		Area:          am.getCurrentArea(),
		RunName:       am.getRunName(),
		CharacterName: am.characterName,
		Timestamp:     time.Now(),
		Stashed:       stashed,
		IsRune:        isRune,
		IsUnique:      itm.Quality == item.QualityUnique,
		IsSet:         itm.Quality == item.QualitySet,
		BaseName:      desc.Name,
	}

	// Track in current run
	if am.currentRun != nil {
		am.currentRun.ItemsFound = append(am.currentRun.ItemsFound, drop)
		if isRune {
			am.currentRun.RunesFound = append(am.currentRun.RunesFound, string(itm.Name))
		}
	}

	// Update session
	if am.currentSession != nil {
		am.currentSession.TotalItemsFound++
		if isRune {
			am.currentSession.TotalRunes++
			am.currentSession.ItemsByQuality.Runes++
			// Track rune breakdown by name
			if am.currentSession.RuneBreakdown == nil {
				am.currentSession.RuneBreakdown = make(map[string]int)
			}
			am.currentSession.RuneBreakdown[string(itm.Name)]++
		}

		// Track by quality
		switch itm.Quality {
		case item.QualityNormal, item.QualitySuperior:
			am.currentSession.ItemsByQuality.Normal++
		case item.QualityMagic:
			am.currentSession.ItemsByQuality.Magic++
		case item.QualityRare:
			am.currentSession.ItemsByQuality.Rare++
		case item.QualitySet:
			am.currentSession.ItemsByQuality.Set++
		case item.QualityUnique:
			am.currentSession.ItemsByQuality.Unique++
		case item.QualityCrafted:
			am.currentSession.ItemsByQuality.Crafted++
		}

		// Track notable drops
		if IsNotableDrop(drop) {
			am.addNotableDrop(drop)
		}
	}

	// Broadcast
	am.broadcast(AnalyticsUpdate{
		Type: "item_found",
		Data: drop,
	})
}

// RecordPotion tracks potion usage
func (am *AnalyticsManager) RecordPotion(potionType string) {
	am.mu.Lock()
	defer am.mu.Unlock()

	if am.currentRun == nil {
		return
	}

	switch potionType {
	case "healing":
		am.currentRun.PotionsUsed.Healing++
	case "mana":
		am.currentRun.PotionsUsed.Mana++
	case "rejuvenation":
		am.currentRun.PotionsUsed.Rejuvenation++
	}
}

// RecordTownVisit increments town visit counter
func (am *AnalyticsManager) RecordTownVisit() {
	am.mu.Lock()
	defer am.mu.Unlock()

	if am.currentRun != nil {
		am.currentRun.TownVisits++
	}
}

// GetSessionSummary returns the current session summary
func (am *AnalyticsManager) GetSessionSummary() *SessionSummary {
	am.mu.RLock()
	defer am.mu.RUnlock()

	if am.currentSession == nil {
		return NewSessionSummary(am.characterName)
	}
	return am.currentSession
}

// GetGlobalAnalytics returns cross-character analytics
func (am *AnalyticsManager) GetGlobalAnalytics() *GlobalAnalytics {
	am.mu.RLock()
	defer am.mu.RUnlock()

	if am.globalStats == nil {
		return NewGlobalAnalytics()
	}
	return am.globalStats
}

// GetRunHistory returns recent run history
func (am *AnalyticsManager) GetRunHistory(limit int) []*RunAnalytics {
	am.mu.RLock()
	defer am.mu.RUnlock()

	if limit <= 0 || limit > len(am.runHistory) {
		limit = len(am.runHistory)
	}

	// Return most recent runs
	start := len(am.runHistory) - limit
	if start < 0 {
		start = 0
	}

	result := make([]*RunAnalytics, limit)
	copy(result, am.runHistory[start:])
	return result
}

// GetRunTypeStats returns aggregated stats by run type
func (am *AnalyticsManager) GetRunTypeStats() map[string]*RunTypeAggregate {
	am.mu.RLock()
	defer am.mu.RUnlock()

	if am.currentSession == nil {
		return make(map[string]*RunTypeAggregate)
	}
	return am.currentSession.RunTypeStats
}

// Helper functions

func (am *AnalyticsManager) getRunName() string {
	if am.currentRun != nil {
		return am.currentRun.RunName
	}
	return ""
}

func (am *AnalyticsManager) getCurrentArea() area.ID {
	if am.currentRun != nil {
		return am.currentRun.Area
	}
	return 0
}

func (am *AnalyticsManager) updateSessionFromRun(run *RunAnalytics) {
	if am.currentSession == nil {
		am.currentSession = NewSessionSummary(am.characterName)
	}

	s := am.currentSession
	s.LastUpdated = time.Now()
	s.TotalRuns++
	s.TotalRuntime += run.Duration

	// Increment dry streak counter (reset happens in addNotableDrop if notable found)
	s.RunsSinceLastNotable++

	if run.Success {
		s.SuccessfulRuns++
	} else {
		s.FailedRuns++
	}

	s.TotalDeaths += run.DeathCount
	s.TotalChickens += run.ChickenCount
	s.TotalMonstersKilled += run.MonstersKilled
	s.TotalElitesKilled += run.ElitesKilled
	s.TotalChampionsKilled += run.ChampionsKilled
	if run.BossKilled {
		s.TotalBossesKilled++
	}

	// Track character level and current XP from the latest run
	if run.CharacterLevel > 0 {
		s.CharacterLevel = run.CharacterLevel
	}
	if run.EndExperience > 0 {
		s.CurrentExperience = run.EndExperience
	} else if run.StartExperience > 0 {
		s.CurrentExperience = run.StartExperience
	}
	// LastLevelExperience and NextLevelExperience are set via UpdateLevelProgress()

	// Track potion usage
	s.TotalPotionsUsed.Healing += run.PotionsUsed.Healing
	s.TotalPotionsUsed.Mana += run.PotionsUsed.Mana
	s.TotalPotionsUsed.Rejuvenation += run.PotionsUsed.Rejuvenation
	s.TotalPotionsUsed.Total += run.PotionsUsed.Healing + run.PotionsUsed.Mana + run.PotionsUsed.Rejuvenation
	s.TotalTownVisits += run.TownVisits

	// Note: TotalItemsFound is already incremented in RecordItem() when items are found,
	// so we don't add len(run.ItemsFound) here to avoid double counting.
	// Same for TotalRunes (incremented in RecordItem).
	// ItemsByQuality is updated in RecordItem as well.

	// Update run type stats
	if s.RunTypeStats == nil {
		s.RunTypeStats = make(map[string]*RunTypeAggregate)
	}

	rts, exists := s.RunTypeStats[run.RunName]
	if !exists {
		rts = &RunTypeAggregate{
			RunName:    run.RunName,
			FastestRun: run.Duration,
			SlowestRun: run.Duration,
		}
		s.RunTypeStats[run.RunName] = rts
	}

	rts.TotalRuns++
	rts.TotalDuration += run.Duration
	rts.ItemsFound += len(run.ItemsFound)
	rts.Deaths += run.DeathCount
	rts.Chickens += run.ChickenCount
	rts.MonstersKilled += run.MonstersKilled
	rts.ElitesKilled += run.ElitesKilled
	rts.ChampionsKilled += run.ChampionsKilled

	if run.Success {
		rts.SuccessfulRuns++
	} else {
		rts.FailedRuns++
	}

	if run.Duration < rts.FastestRun {
		rts.FastestRun = run.Duration
	}
	if run.Duration > rts.SlowestRun {
		rts.SlowestRun = run.Duration
	}

	// Calculate averages
	if rts.TotalRuns > 0 {
		rts.AvgDuration = rts.TotalDuration / int64(rts.TotalRuns)
		rts.SuccessRate = float64(rts.SuccessfulRuns) / float64(rts.TotalRuns) * 100
		rts.ItemsPerRun = float64(rts.ItemsFound) / float64(rts.TotalRuns)
		rts.DeathsPerRun = float64(rts.Deaths) / float64(rts.TotalRuns)
		totalMonsters := rts.MonstersKilled + rts.ElitesKilled + rts.ChampionsKilled
		rts.MonstersPerRun = float64(totalMonsters) / float64(rts.TotalRuns)
	}

	// Update hourly stats
	hour := time.Now().Hour()
	s.HourlyStats[hour].Hour = hour
	s.HourlyStats[hour].RunsComplete++
	s.HourlyStats[hour].ItemsFound += len(run.ItemsFound)
	s.HourlyStats[hour].Deaths += run.DeathCount
	s.HourlyStats[hour].MonstersKilled += run.MonstersKilled
	s.HourlyStats[hour].ElitesKilled += run.ElitesKilled
	s.HourlyStats[hour].ChampionsKilled += run.ChampionsKilled
	s.HourlyStats[hour].ExperienceGain += run.ExperienceGain

	// Update XP stats
	s.TotalExperienceGain += run.ExperienceGain
	rts.TotalExperience += run.ExperienceGain
	if rts.TotalRuns > 0 {
		rts.ExperiencePerRun = rts.TotalExperience / int64(rts.TotalRuns)
	}

	// Calculate XP per hour
	if s.TotalRuntime > 0 {
		hours := float64(s.TotalRuntime) / 3600000.0
		s.ExperiencePerHour = float64(s.TotalExperienceGain) / hours
	}

	// Update items by quality for run type (for heatmap)
	for _, itm := range run.ItemsFound {
		switch itm.Quality {
		case item.QualityNormal, item.QualitySuperior, item.QualityLowQuality:
			rts.ItemsByQuality.Normal++
		case item.QualityMagic:
			rts.ItemsByQuality.Magic++
		case item.QualityRare:
			rts.ItemsByQuality.Rare++
		case item.QualitySet:
			rts.ItemsByQuality.Set++
		case item.QualityUnique:
			rts.ItemsByQuality.Unique++
		case item.QualityCrafted:
			rts.ItemsByQuality.Crafted++
		}
		if itm.IsRune {
			rts.ItemsByQuality.Runes++
		}
	}
}

func (am *AnalyticsManager) updateGlobalFromRun(run *RunAnalytics) {
	if am.globalStats == nil {
		am.globalStats = NewGlobalAnalytics()
	}

	g := am.globalStats
	g.LastUpdated = time.Now()
	g.TotalRuntime += run.Duration
	g.TotalRuns++
	g.TotalDeaths += run.DeathCount
	g.TotalItems += len(run.ItemsFound)

	// Update character stats
	if g.CharacterStats == nil {
		g.CharacterStats = make(map[string]*CharacterSummary)
	}

	cs, exists := g.CharacterStats[am.characterName]
	if !exists {
		cs = &CharacterSummary{
			Name: am.characterName,
		}
		g.CharacterStats[am.characterName] = cs
	}

	cs.Level = run.CharacterLevel
	cs.TotalRuns++
	cs.TotalDeaths += run.DeathCount
	cs.TotalItems += len(run.ItemsFound)
	cs.LastActive = time.Now()
	cs.TotalRuntime += run.Duration

	// Update run type stats globally
	if g.RunTypeStats == nil {
		g.RunTypeStats = make(map[string]*RunTypeAggregate)
	}

	rts, exists := g.RunTypeStats[run.RunName]
	if !exists {
		rts = &RunTypeAggregate{
			RunName:    run.RunName,
			FastestRun: run.Duration,
			SlowestRun: run.Duration,
		}
		g.RunTypeStats[run.RunName] = rts
	}

	rts.TotalRuns++
	rts.TotalDuration += run.Duration
	rts.ItemsFound += len(run.ItemsFound)
	rts.Deaths += run.DeathCount

	if run.Success {
		rts.SuccessfulRuns++
	} else {
		rts.FailedRuns++
	}

	if run.Duration < rts.FastestRun {
		rts.FastestRun = run.Duration
	}
	if run.Duration > rts.SlowestRun {
		rts.SlowestRun = run.Duration
	}

	if rts.TotalRuns > 0 {
		rts.AvgDuration = rts.TotalDuration / int64(rts.TotalRuns)
		rts.SuccessRate = float64(rts.SuccessfulRuns) / float64(rts.TotalRuns) * 100
		rts.ItemsPerRun = float64(rts.ItemsFound) / float64(rts.TotalRuns)
		rts.DeathsPerRun = float64(rts.Deaths) / float64(rts.TotalRuns)
	}

	// Update hourly stats
	hour := time.Now().Hour()
	g.HourlyStats[hour].Hour = hour
	g.HourlyStats[hour].RunsComplete++
	g.HourlyStats[hour].ItemsFound += len(run.ItemsFound)
	g.HourlyStats[hour].Deaths += run.DeathCount
}

func (am *AnalyticsManager) addNotableDrop(drop AnalyticsItemDrop) {
	if am.currentSession == nil {
		return
	}

	am.currentSession.NotableDrops = append(am.currentSession.NotableDrops, drop)

	// Reset dry streak counter on notable drop
	am.currentSession.RunsSinceLastNotable = 0
	am.currentSession.LastNotableDropTime = drop.Timestamp
	am.currentSession.LastNotableDropName = drop.Name

	// Trim to max
	max := am.config.MaxNotableDrops
	if max <= 0 {
		max = 100
	}
	if len(am.currentSession.NotableDrops) > max {
		am.currentSession.NotableDrops = am.currentSession.NotableDrops[len(am.currentSession.NotableDrops)-max:]
	}

	// Also add to global
	if am.globalStats != nil {
		am.globalStats.NotableDrops = append(am.globalStats.NotableDrops, drop)
		if len(am.globalStats.NotableDrops) > max {
			am.globalStats.NotableDrops = am.globalStats.NotableDrops[len(am.globalStats.NotableDrops)-max:]
		}
	}
}

// Persistence functions

func (am *AnalyticsManager) Save() error {
	am.mu.RLock()
	defer am.mu.RUnlock()

	if !am.config.EnablePersistence {
		return nil
	}

	// Save character session
	if am.currentSession != nil {
		sessionFile := filepath.Join(am.characterPath, fmt.Sprintf("session_%s.json", time.Now().Format("2006-01-02")))
		if err := am.saveJSON(sessionFile, am.currentSession); err != nil {
			if am.logger != nil {
				am.logger.Error("Failed to save session analytics", slog.String("error", err.Error()))
			}
		}
	}

	// Save global stats
	if am.globalStats != nil {
		globalFile := filepath.Join(am.globalPath, "global_stats.json")
		if err := am.saveJSON(globalFile, am.globalStats); err != nil {
			if am.logger != nil {
				am.logger.Error("Failed to save global analytics", slog.String("error", err.Error()))
			}
		}
	}

	// Save run history (recent runs only)
	if len(am.runHistory) > 0 {
		historyFile := filepath.Join(am.characterPath, "run_history.json")
		// Keep only recent history based on config
		maxHistory := am.config.HistoryDays * 100 // Approximate runs per day
		if maxHistory <= 0 {
			maxHistory = 3000
		}
		history := am.runHistory
		if len(history) > maxHistory {
			history = history[len(history)-maxHistory:]
		}
		if err := am.saveJSON(historyFile, history); err != nil {
			if am.logger != nil {
				am.logger.Error("Failed to save run history", slog.String("error", err.Error()))
			}
		}
	}

	return nil
}

func (am *AnalyticsManager) load() {
	// Load global stats
	globalFile := filepath.Join(am.globalPath, "global_stats.json")
	if data, err := os.ReadFile(globalFile); err == nil {
		var global GlobalAnalytics
		if err := json.Unmarshal(data, &global); err == nil {
			am.globalStats = &global
			// Initialize nil maps after JSON unmarshal
			if am.globalStats.CharacterStats == nil {
				am.globalStats.CharacterStats = make(map[string]*CharacterSummary)
			}
			if am.globalStats.RunTypeStats == nil {
				am.globalStats.RunTypeStats = make(map[string]*RunTypeAggregate)
			}
			if am.globalStats.NotableDrops == nil {
				am.globalStats.NotableDrops = make([]AnalyticsItemDrop, 0)
			}
		} else if am.logger != nil {
			am.logger.Warn("Failed to unmarshal global stats", slog.String("error", err.Error()))
		}
	} else if am.logger != nil && !os.IsNotExist(err) {
		am.logger.Warn("Failed to read global stats file", slog.String("error", err.Error()))
	}
	if am.globalStats == nil {
		am.globalStats = NewGlobalAnalytics()
	}

	// Load today's session or create new
	sessionFile := filepath.Join(am.characterPath, fmt.Sprintf("session_%s.json", time.Now().Format("2006-01-02")))
	if data, err := os.ReadFile(sessionFile); err == nil {
		var session SessionSummary
		if err := json.Unmarshal(data, &session); err == nil {
			am.currentSession = &session
			// Initialize nil maps/slices after JSON unmarshal
			if am.currentSession.RunTypeStats == nil {
				am.currentSession.RunTypeStats = make(map[string]*RunTypeAggregate)
			}
			if am.currentSession.NotableDrops == nil {
				am.currentSession.NotableDrops = make([]AnalyticsItemDrop, 0)
			}
			if am.currentSession.RuneBreakdown == nil {
				am.currentSession.RuneBreakdown = make(map[string]int)
			}
			if am.currentSession.DeathsByArea == nil {
				am.currentSession.DeathsByArea = make(map[string]*AreaDeathStats)
			}
		} else if am.logger != nil {
			am.logger.Warn("Failed to unmarshal session", slog.String("file", sessionFile), slog.String("error", err.Error()))
		}
	} else if am.logger != nil && !os.IsNotExist(err) {
		am.logger.Warn("Failed to read session file", slog.String("file", sessionFile), slog.String("error", err.Error()))
	}
	if am.currentSession == nil {
		am.currentSession = NewSessionSummary(am.characterName)
	}

	// Load run history
	historyFile := filepath.Join(am.characterPath, "run_history.json")
	if data, err := os.ReadFile(historyFile); err == nil {
		var history []*RunAnalytics
		if err := json.Unmarshal(data, &history); err == nil {
			am.runHistory = history
		} else if am.logger != nil {
			am.logger.Warn("Failed to unmarshal run history", slog.String("error", err.Error()))
		}
	} else if am.logger != nil && !os.IsNotExist(err) {
		am.logger.Warn("Failed to read run history file", slog.String("error", err.Error()))
	}

	if am.logger != nil {
		am.logger.Debug("Analytics loaded",
			slog.String("character", am.characterName),
			slog.String("sessionFile", sessionFile),
			slog.Int("history_runs", len(am.runHistory)),
			slog.Int("session_runs", am.currentSession.TotalRuns),
			slog.Int("session_items", am.currentSession.TotalItemsFound),
		)
	}
}

func (am *AnalyticsManager) saveJSON(path string, data interface{}) error {
	jsonData, err := json.MarshalIndent(data, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, jsonData, 0644)
}

// Reset clears all in-memory analytics data and reinitializes
func (am *AnalyticsManager) Reset() {
	am.mu.Lock()
	defer am.mu.Unlock()

	am.currentSession = NewSessionSummary(am.characterName)
	am.globalStats = NewGlobalAnalytics()
	am.runHistory = make([]*RunAnalytics, 0)
	am.currentRun = nil

	if am.logger != nil {
		am.logger.Info("Analytics data reset", slog.String("character", am.characterName))
	}
}

// Cleanup removes old analytics files based on config
func (am *AnalyticsManager) Cleanup() error {
	am.mu.Lock()
	defer am.mu.Unlock()

	cutoff := time.Now().AddDate(0, 0, -am.config.HistoryDays)

	// Clean up old session files
	entries, err := os.ReadDir(am.characterPath)
	if err != nil {
		return err
	}

	for _, entry := range entries {
		if entry.IsDir() {
			continue
		}
		// Parse date from filename: session_2006-01-02.json
		name := entry.Name()
		if len(name) < 18 || name[:8] != "session_" {
			continue
		}
		dateStr := name[8:18]
		fileDate, err := time.Parse("2006-01-02", dateStr)
		if err != nil {
			continue
		}
		if fileDate.Before(cutoff) {
			filePath := filepath.Join(am.characterPath, name)
			if err := os.Remove(filePath); err != nil {
				if am.logger != nil {
					am.logger.Warn("Failed to remove old analytics file", slog.String("file", filePath))
				}
			}
		}
	}

	return nil
}

// UpdateLevelProgress stores the latest level and XP boundary values from the game
// d2rCumulativeXP holds the total cumulative XP at the start of each level (1-99).
// Source: Diablo II: LoD experience table (classic.battle.net).
var d2rCumulativeXP = [100]int64{
	0,          // level 0 (unused)
	0,          // level 1
	500,        // level 2
	1500,       // level 3
	3750,       // level 4
	7875,       // level 5
	14175,      // level 6
	22680,      // level 7
	32886,      // level 8
	44396,      // level 9
	57715,      // level 10
	72144,      // level 11
	90180,      // level 12
	112725,     // level 13
	140906,     // level 14
	176132,     // level 15
	220165,     // level 16
	275207,     // level 17
	344008,     // level 18
	430010,     // level 19
	537513,     // level 20
	671891,     // level 21
	839864,     // level 22
	1049830,    // level 23
	1312287,    // level 24
	1640359,    // level 25
	2050449,    // level 26
	2563061,    // level 27
	3203826,    // level 28
	3902260,    // level 29
	4663553,    // level 30
	5493363,    // level 31
	6397855,    // level 32
	7383752,    // level 33
	8458379,    // level 34
	9629723,    // level 35
	10906488,   // level 36
	12298162,   // level 37
	13815086,   // level 38
	15468534,   // level 39
	17270791,   // level 40
	19235252,   // level 41
	21376515,   // level 42
	23710491,   // level 43
	26254525,   // level 44
	29027522,   // level 45
	32050088,   // level 46
	35344686,   // level 47
	38935798,   // level 48
	42850109,   // level 49
	47116709,   // level 50
	51767302,   // level 51
	56836449,   // level 52
	62361819,   // level 53
	68384473,   // level 54
	74949165,   // level 55
	82104680,   // level 56
	89904191,   // level 57
	98405658,   // level 58
	107672256,  // level 59
	117772849,  // level 60
	128782495,  // level 61
	140783010,  // level 62
	153863570,  // level 63
	168121381,  // level 64
	183662396,  // level 65
	200602101,  // level 66
	219066380,  // level 67
	239192444,  // level 68
	261129853,  // level 69
	285041630,  // level 70
	311105466,  // level 71
	339515048,  // level 72
	370481492,  // level 73
	404234916,  // level 74
	441026148,  // level 75
	481128591,  // level 76
	524840254,  // level 77
	572485967,  // level 78
	624419793,  // level 79
	681027665,  // level 80
	742730244,  // level 81
	809986056,  // level 82
	883294891,  // level 83
	963201521,  // level 84
	1050299747, // level 85
	1145236814, // level 86
	1248718217, // level 87
	1361512946, // level 88
	1484459201, // level 89
	1618470619, // level 90
	1764543065, // level 91
	1923762030, // level 92
	2097310703, // level 93
	2286478756, // level 94
	2492671933, // level 95
	2717422497, // level 96
	2962400612, // level 97
	3229426756, // level 98
	3520485254, // level 99
}

func (am *AnalyticsManager) UpdateLevelProgress(level int, currentXP, lastLevelXP, nextLevelXP int64) {
	am.mu.Lock()
	defer am.mu.Unlock()

	if am.currentSession == nil {
		return
	}

	if level > 0 {
		am.currentSession.CharacterLevel = level
	}
	if currentXP > 0 {
		am.currentSession.CurrentExperience = currentXP
	}

	// Use the static XP table as the primary source (stat.LastExp/NextExp are often
	// not populated in the player stat list). Fall back to the caller-provided values
	// only if valid and the table lookup fails.
	if level >= 1 && level <= 99 {
		am.currentSession.LastLevelExperience = d2rCumulativeXP[level]
		if level < 99 {
			am.currentSession.NextLevelExperience = d2rCumulativeXP[level+1]
		} else {
			// Level 99 — no next level
			am.currentSession.NextLevelExperience = 0
		}
	} else {
		// Fallback to caller-provided values
		if lastLevelXP > 0 {
			am.currentSession.LastLevelExperience = lastLevelXP
		}
		if nextLevelXP > 0 {
			am.currentSession.NextLevelExperience = nextLevelXP
		}
	}
}

// GetSessionHistory returns a list of past session summaries by scanning session files
func (am *AnalyticsManager) GetSessionHistory(limit int) []SessionHistoryEntry {
	am.mu.RLock()
	defer am.mu.RUnlock()

	var entries []SessionHistoryEntry

	dirEntries, err := os.ReadDir(am.characterPath)
	if err != nil {
		if am.logger != nil {
			am.logger.Warn("Failed to read analytics directory for history", slog.String("error", err.Error()))
		}
		return entries
	}

	for _, entry := range dirEntries {
		if entry.IsDir() {
			continue
		}
		name := entry.Name()
		if !strings.HasPrefix(name, "session_") || !strings.HasSuffix(name, ".json") {
			continue
		}
		dateStr := strings.TrimPrefix(name, "session_")
		dateStr = strings.TrimSuffix(dateStr, ".json")

		filePath := filepath.Join(am.characterPath, name)
		fileData, err := os.ReadFile(filePath)
		if err != nil {
			continue
		}

		var session SessionSummary
		if err := json.Unmarshal(fileData, &session); err != nil {
			continue
		}

		runtimeMin := float64(session.TotalRuntime) / 60000.0
		entries = append(entries, SessionHistoryEntry{
			Date:           dateStr,
			CharacterName:  session.CharacterName,
			TotalRuns:      session.TotalRuns,
			SuccessfulRuns: session.SuccessfulRuns,
			TotalDeaths:    session.TotalDeaths,
			TotalChickens:  session.TotalChickens,
			TotalItems:     session.TotalItemsFound,
			TotalRunes:     session.TotalRunes,
			MonstersKilled: session.TotalMonstersKilled,
			ElitesKilled:   session.TotalElitesKilled,
			ExperienceGain: session.TotalExperienceGain,
			XPPerHour:      session.ExperiencePerHour,
			RuntimeMinutes: runtimeMin,
			NotableCount:   len(session.NotableDrops),
		})
	}

	// Sort by date descending (most recent first)
	for i := 0; i < len(entries); i++ {
		for j := i + 1; j < len(entries); j++ {
			if entries[j].Date > entries[i].Date {
				entries[i], entries[j] = entries[j], entries[i]
			}
		}
	}

	if limit > 0 && len(entries) > limit {
		entries = entries[:limit]
	}

	return entries
}

// GetCharacterComparison returns per-character comparison data from current sessions
func (am *AnalyticsManager) GetCharacterComparison() *CharacterComparisonEntry {
	am.mu.RLock()
	defer am.mu.RUnlock()

	if am.currentSession == nil {
		return nil
	}

	s := am.currentSession
	runtimeMin := float64(s.TotalRuntime) / 60000.0

	entry := &CharacterComparisonEntry{
		CharacterName:  s.CharacterName,
		TotalRuns:      s.TotalRuns,
		TotalDeaths:    s.TotalDeaths,
		MonstersKilled: s.TotalMonstersKilled,
		ElitesKilled:   s.TotalElitesKilled,
		TotalItems:     s.TotalItemsFound,
		TotalRunes:     s.TotalRunes,
		UniquesFound:   s.ItemsByQuality.Unique,
		SetsFound:      s.ItemsByQuality.Set,
		ExperienceGain: s.TotalExperienceGain,
		XPPerHour:      s.ExperiencePerHour,
		RuntimeMinutes: runtimeMin,
	}

	if s.TotalRuns > 0 {
		entry.SuccessRate = float64(s.SuccessfulRuns) / float64(s.TotalRuns) * 100
		entry.AvgRunTime = float64(s.TotalRuntime) / float64(s.TotalRuns) / 1000.0
		entry.ItemsPerRun = float64(s.TotalItemsFound) / float64(s.TotalRuns)
		entry.DeathsPerRun = float64(s.TotalDeaths) / float64(s.TotalRuns)
	}

	// Find best run type
	bestRuns := 0
	for name, rts := range s.RunTypeStats {
		if rts.TotalRuns > bestRuns {
			bestRuns = rts.TotalRuns
			entry.BestRunType = name
		}
	}

	return entry
}

// HandleEvent processes events from the event system and updates analytics.
// This method implements the event handler interface for integration with
// the existing event listener system.
// Note: RunStartedEvent and RunFinishedEvent are handled directly in bot.go
// with XP tracking, so we don't process them here to avoid double counting.
func (am *AnalyticsManager) HandleEvent(ctx context.Context, e event.Event) error {
	// Only handle events from our character
	if !strings.EqualFold(e.Supervisor(), am.characterName) {
		return nil
	}

	switch evt := e.(type) {
	case event.ItemStashedEvent:
		am.RecordItem(evt.Item.Item, true)

	case event.UsedPotionEvent:
		// Convert PotionType to the lowercase string format expected by RecordPotion
		var potionStr string
		switch evt.PotionType {
		case data.HealingPotion:
			potionStr = "healing"
		case data.ManaPotion:
			potionStr = "mana"
		case data.RejuvenationPotion:
			potionStr = "rejuvenation"
		}
		if potionStr != "" {
			am.RecordPotion(potionStr)
		}

	case event.GameFinishedEvent:
		if evt.Reason == event.FinishedDied {
			am.RecordDeath()
		} else if evt.Reason == event.FinishedChicken || evt.Reason == event.FinishedMercChicken {
			am.RecordChicken(string(evt.Reason))
		}

	case event.MonsterKilledEvent:
		am.RecordKill(evt.IsElite, evt.IsChampion, evt.IsBoss)
	}

	return nil
}
