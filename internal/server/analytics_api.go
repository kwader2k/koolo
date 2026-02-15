package server

import (
	"encoding/json"
	"net/http"
	"os"
	"strconv"

	"github.com/hectorgimenez/koolo/internal/bot"
	"github.com/hectorgimenez/koolo/internal/config"
)

// analyticsPage serves the analytics dashboard
func (s *HttpServer) analyticsPage(w http.ResponseWriter, r *http.Request) {
	s.templates.ExecuteTemplate(w, "analytics.gohtml", nil)
}

// getAnalyticsManagers returns analytics managers for all configured characters.
// For running supervisors it returns the live manager; for non-running ones it
// returns a lazily-cached manager loaded from disk, avoiding repeated I/O
// across concurrent API calls.
func (s *HttpServer) getAnalyticsManagers() map[string]*bot.AnalyticsManager {
	result := make(map[string]*bot.AnalyticsManager)

	// Get managers from running supervisors (always authoritative)
	runningManagers := s.manager.GetAllAnalyticsManagers()
	for name, am := range runningManagers {
		result[name] = am
	}

	// For non-running characters, use the cache to avoid repeated disk I/O.
	basePath, err := os.Getwd()
	if err != nil {
		return result
	}

	s.analyticsCacheMu.Lock()
	defer s.analyticsCacheMu.Unlock()

	for charName := range config.GetCharacters() {
		if _, running := result[charName]; running {
			continue
		}

		// Check cache first
		if cached, ok := s.cachedAnalyticsManagers[charName]; ok {
			result[charName] = cached
			continue
		}

		// Cache miss — load from disk and store
		am := bot.NewAnalyticsManager(charName, basePath, nil, bot.AnalyticsConfig{
			HistoryDays:     config.Koolo.Analytics.HistoryDays,
			MaxNotableDrops: config.Koolo.Analytics.MaxNotableDrops,
			TrackAllItems:   config.Koolo.Analytics.TrackAllItems,
		})
		if am.GetSessionSummary().TotalRuns > 0 || len(am.GetRunHistory(0)) > 0 {
			s.cachedAnalyticsManagers[charName] = am
			result[charName] = am
		}
	}

	return result
}

// invalidateAnalyticsCache removes cached analytics managers so that subsequent
// calls to getAnalyticsManagers will reload data from disk. When called with no
// arguments, the entire cache is cleared; otherwise only the named characters
// are evicted.
func (s *HttpServer) invalidateAnalyticsCache(characters ...string) {
	s.analyticsCacheMu.Lock()
	defer s.analyticsCacheMu.Unlock()

	if len(characters) == 0 {
		// Full invalidation
		s.cachedAnalyticsManagers = make(map[string]*bot.AnalyticsManager)
		return
	}

	for _, name := range characters {
		delete(s.cachedAnalyticsManagers, name)
	}
}

// analyticsSessionAPI returns session summary for a character
func (s *HttpServer) analyticsSessionAPI(w http.ResponseWriter, r *http.Request) {
	character := r.URL.Query().Get("character")
	analyticsManagers := s.getAnalyticsManagers()

	w.Header().Set("Content-Type", "application/json")

	if character == "" {
		// Return combined session from all running supervisors
		combined := bot.NewSessionSummary("all")
		for name, am := range analyticsManagers {
			session := am.GetSessionSummary()
			combined.TotalRuns += session.TotalRuns
			combined.SuccessfulRuns += session.SuccessfulRuns
			combined.FailedRuns += session.FailedRuns
			combined.TotalDeaths += session.TotalDeaths
			combined.TotalChickens += session.TotalChickens
			combined.TotalItemsFound += session.TotalItemsFound
			combined.TotalRunes += session.TotalRunes
			combined.TotalRuntime += session.TotalRuntime
			combined.TotalMonstersKilled += session.TotalMonstersKilled
			combined.TotalElitesKilled += session.TotalElitesKilled
			combined.TotalChampionsKilled += session.TotalChampionsKilled
			combined.TotalBossesKilled += session.TotalBossesKilled
			combined.TotalTownVisits += session.TotalTownVisits

			// Merge potion usage
			combined.TotalPotionsUsed.Healing += session.TotalPotionsUsed.Healing
			combined.TotalPotionsUsed.Mana += session.TotalPotionsUsed.Mana
			combined.TotalPotionsUsed.Rejuvenation += session.TotalPotionsUsed.Rejuvenation
			combined.TotalPotionsUsed.Total += session.TotalPotionsUsed.Total

			// Merge item quality breakdown
			combined.ItemsByQuality.Normal += session.ItemsByQuality.Normal
			combined.ItemsByQuality.Magic += session.ItemsByQuality.Magic
			combined.ItemsByQuality.Rare += session.ItemsByQuality.Rare
			combined.ItemsByQuality.Set += session.ItemsByQuality.Set
			combined.ItemsByQuality.Unique += session.ItemsByQuality.Unique
			combined.ItemsByQuality.Crafted += session.ItemsByQuality.Crafted
			combined.ItemsByQuality.Runes += session.ItemsByQuality.Runes

			// Merge run type stats
			for runName, rts := range session.RunTypeStats {
				if existing, ok := combined.RunTypeStats[runName]; ok {
					existing.TotalRuns += rts.TotalRuns
					existing.SuccessfulRuns += rts.SuccessfulRuns
					existing.FailedRuns += rts.FailedRuns
					existing.TotalDuration += rts.TotalDuration
					existing.ItemsFound += rts.ItemsFound
					existing.Deaths += rts.Deaths
					existing.Chickens += rts.Chickens
					existing.MonstersKilled += rts.MonstersKilled
					existing.ElitesKilled += rts.ElitesKilled
					existing.ChampionsKilled += rts.ChampionsKilled
					if rts.FastestRun < existing.FastestRun {
						existing.FastestRun = rts.FastestRun
					}
					if rts.SlowestRun > existing.SlowestRun {
						existing.SlowestRun = rts.SlowestRun
					}
				} else {
					copyRts := *rts
					combined.RunTypeStats[runName] = &copyRts
				}
			}

			// Merge hourly stats
			for i := 0; i < 24; i++ {
				combined.HourlyStats[i].Hour = i
				combined.HourlyStats[i].RunsComplete += session.HourlyStats[i].RunsComplete
				combined.HourlyStats[i].ItemsFound += session.HourlyStats[i].ItemsFound
				combined.HourlyStats[i].Deaths += session.HourlyStats[i].Deaths
				combined.HourlyStats[i].MonstersKilled += session.HourlyStats[i].MonstersKilled
				combined.HourlyStats[i].ElitesKilled += session.HourlyStats[i].ElitesKilled
				combined.HourlyStats[i].ChampionsKilled += session.HourlyStats[i].ChampionsKilled
			}

			// Merge notable drops
			combined.NotableDrops = append(combined.NotableDrops, session.NotableDrops...)

			// Merge rune breakdown
			for runeName, count := range session.RuneBreakdown {
				if combined.RuneBreakdown == nil {
					combined.RuneBreakdown = make(map[string]int)
				}
				combined.RuneBreakdown[runeName] += count
			}

			// Merge XP tracking
			combined.TotalExperienceGain += session.TotalExperienceGain

			// Track worst dry streak across characters
			if session.RunsSinceLastNotable > combined.RunsSinceLastNotable {
				combined.RunsSinceLastNotable = session.RunsSinceLastNotable
				combined.LastNotableDropTime = session.LastNotableDropTime
				combined.LastNotableDropName = session.LastNotableDropName
			}

			_ = name // suppress unused warning
		}

		// Recalculate averages for merged run type stats
		for _, rts := range combined.RunTypeStats {
			if rts.TotalRuns > 0 {
				rts.AvgDuration = rts.TotalDuration / int64(rts.TotalRuns)
				rts.SuccessRate = float64(rts.SuccessfulRuns) / float64(rts.TotalRuns) * 100
				rts.ItemsPerRun = float64(rts.ItemsFound) / float64(rts.TotalRuns)
				rts.DeathsPerRun = float64(rts.Deaths) / float64(rts.TotalRuns)
			}
		}

		// Calculate combined XP per hour
		if combined.TotalRuntime > 0 {
			hours := float64(combined.TotalRuntime) / 3600000.0
			combined.ExperiencePerHour = float64(combined.TotalExperienceGain) / hours
		}

		json.NewEncoder(w).Encode(combined)
		return
	}

	am, exists := analyticsManagers[character]
	if !exists {
		http.Error(w, "Character not found", http.StatusNotFound)
		return
	}

	json.NewEncoder(w).Encode(am.GetSessionSummary())
}

// analyticsGlobalAPI returns global cross-character analytics
func (s *HttpServer) analyticsGlobalAPI(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	analyticsManagers := s.getAnalyticsManagers()

	// Aggregate from all managers
	global := bot.NewGlobalAnalytics()

	for name, am := range analyticsManagers {
		charGlobal := am.GetGlobalAnalytics()

		global.TotalRuntime += charGlobal.TotalRuntime
		global.TotalRuns += charGlobal.TotalRuns
		global.TotalDeaths += charGlobal.TotalDeaths
		global.TotalItems += charGlobal.TotalItems

		// Merge character stats
		for charName, cs := range charGlobal.CharacterStats {
			global.CharacterStats[charName] = cs
		}

		// Merge run type stats
		for runName, rts := range charGlobal.RunTypeStats {
			if existing, ok := global.RunTypeStats[runName]; ok {
				existing.TotalRuns += rts.TotalRuns
				existing.SuccessfulRuns += rts.SuccessfulRuns
				existing.TotalDuration += rts.TotalDuration
				existing.ItemsFound += rts.ItemsFound
				existing.Deaths += rts.Deaths
			} else {
				copyRts := *rts
				global.RunTypeStats[runName] = &copyRts
			}
		}

		// Merge hourly stats
		for i := 0; i < 24; i++ {
			global.HourlyStats[i].Hour = i
			global.HourlyStats[i].RunsComplete += charGlobal.HourlyStats[i].RunsComplete
			global.HourlyStats[i].ItemsFound += charGlobal.HourlyStats[i].ItemsFound
			global.HourlyStats[i].Deaths += charGlobal.HourlyStats[i].Deaths
		}

		// Merge notable drops
		global.NotableDrops = append(global.NotableDrops, charGlobal.NotableDrops...)
		_ = name
	}

	// Recalculate averages
	for _, rts := range global.RunTypeStats {
		if rts.TotalRuns > 0 {
			rts.AvgDuration = rts.TotalDuration / int64(rts.TotalRuns)
			rts.SuccessRate = float64(rts.SuccessfulRuns) / float64(rts.TotalRuns) * 100
			rts.ItemsPerRun = float64(rts.ItemsFound) / float64(rts.TotalRuns)
			rts.DeathsPerRun = float64(rts.Deaths) / float64(rts.TotalRuns)
		}
	}

	json.NewEncoder(w).Encode(global)
}

// analyticsRunsAPI returns run history
func (s *HttpServer) analyticsRunsAPI(w http.ResponseWriter, r *http.Request) {
	character := r.URL.Query().Get("character")
	runType := r.URL.Query().Get("type")
	limitStr := r.URL.Query().Get("limit")
	analyticsManagers := s.getAnalyticsManagers()

	limit := 100
	if limitStr != "" {
		if l, err := strconv.Atoi(limitStr); err == nil && l > 0 {
			limit = l
		}
	}

	w.Header().Set("Content-Type", "application/json")

	var allRuns []*bot.RunAnalytics

	if character != "" {
		am, exists := analyticsManagers[character]
		if !exists {
			http.Error(w, "Character not found", http.StatusNotFound)
			return
		}
		allRuns = am.GetRunHistory(limit)
	} else {
		// Combine from all managers
		for _, am := range analyticsManagers {
			allRuns = append(allRuns, am.GetRunHistory(limit)...)
		}
	}

	// Filter by run type if specified
	if runType != "" {
		filtered := make([]*bot.RunAnalytics, 0)
		for _, run := range allRuns {
			if run.RunName == runType {
				filtered = append(filtered, run)
			}
		}
		allRuns = filtered
	}

	// Limit results
	if len(allRuns) > limit {
		allRuns = allRuns[len(allRuns)-limit:]
	}

	json.NewEncoder(w).Encode(allRuns)
}

// analyticsHourlyAPI returns hourly breakdown
func (s *HttpServer) analyticsHourlyAPI(w http.ResponseWriter, r *http.Request) {
	character := r.URL.Query().Get("character")
	analyticsManagers := s.getAnalyticsManagers()

	w.Header().Set("Content-Type", "application/json")

	var hourly [24]bot.HourlyBucket

	if character != "" {
		am, exists := analyticsManagers[character]
		if !exists {
			http.Error(w, "Character not found", http.StatusNotFound)
			return
		}
		session := am.GetSessionSummary()
		hourly = session.HourlyStats
	} else {
		// Combine from all managers
		for _, am := range analyticsManagers {
			session := am.GetSessionSummary()
			for i := 0; i < 24; i++ {
				hourly[i].Hour = i
				hourly[i].RunsComplete += session.HourlyStats[i].RunsComplete
				hourly[i].ItemsFound += session.HourlyStats[i].ItemsFound
				hourly[i].Deaths += session.HourlyStats[i].Deaths
			}
		}
	}

	json.NewEncoder(w).Encode(hourly)
}

// analyticsRunTypesAPI returns per-run-type statistics
func (s *HttpServer) analyticsRunTypesAPI(w http.ResponseWriter, r *http.Request) {
	character := r.URL.Query().Get("character")
	analyticsManagers := s.getAnalyticsManagers()

	w.Header().Set("Content-Type", "application/json")

	runTypes := make(map[string]*bot.RunTypeAggregate)

	if character != "" {
		am, exists := analyticsManagers[character]
		if !exists {
			http.Error(w, "Character not found", http.StatusNotFound)
			return
		}
		runTypes = am.GetRunTypeStats()
	} else {
		// Combine from all managers
		for _, am := range analyticsManagers {
			for runName, rts := range am.GetRunTypeStats() {
				if existing, ok := runTypes[runName]; ok {
					existing.TotalRuns += rts.TotalRuns
					existing.SuccessfulRuns += rts.SuccessfulRuns
					existing.FailedRuns += rts.FailedRuns
					existing.TotalDuration += rts.TotalDuration
					existing.ItemsFound += rts.ItemsFound
					existing.Deaths += rts.Deaths
					existing.Chickens += rts.Chickens
					existing.MonstersKilled += rts.MonstersKilled
					existing.ElitesKilled += rts.ElitesKilled
					existing.ChampionsKilled += rts.ChampionsKilled
					existing.TotalExperience += rts.TotalExperience
					// Merge ItemsByQuality
					existing.ItemsByQuality.Normal += rts.ItemsByQuality.Normal
					existing.ItemsByQuality.Magic += rts.ItemsByQuality.Magic
					existing.ItemsByQuality.Rare += rts.ItemsByQuality.Rare
					existing.ItemsByQuality.Set += rts.ItemsByQuality.Set
					existing.ItemsByQuality.Unique += rts.ItemsByQuality.Unique
					existing.ItemsByQuality.Crafted += rts.ItemsByQuality.Crafted
					existing.ItemsByQuality.Runes += rts.ItemsByQuality.Runes
					if rts.FastestRun < existing.FastestRun {
						existing.FastestRun = rts.FastestRun
					}
					if rts.SlowestRun > existing.SlowestRun {
						existing.SlowestRun = rts.SlowestRun
					}
				} else {
					copyRts := *rts
					runTypes[runName] = &copyRts
				}
			}
		}

		// Recalculate averages
		for _, rts := range runTypes {
			if rts.TotalRuns > 0 {
				rts.AvgDuration = rts.TotalDuration / int64(rts.TotalRuns)
				rts.SuccessRate = float64(rts.SuccessfulRuns) / float64(rts.TotalRuns) * 100
				rts.ItemsPerRun = float64(rts.ItemsFound) / float64(rts.TotalRuns)
				rts.DeathsPerRun = float64(rts.Deaths) / float64(rts.TotalRuns)
				rts.MonstersPerRun = float64(rts.MonstersKilled) / float64(rts.TotalRuns)
				rts.ExperiencePerRun = rts.TotalExperience / int64(rts.TotalRuns)
			}
		}
	}

	json.NewEncoder(w).Encode(runTypes)
}

// analyticsItemsAPI returns item drop history
func (s *HttpServer) analyticsItemsAPI(w http.ResponseWriter, r *http.Request) {
	character := r.URL.Query().Get("character")
	notableOnly := r.URL.Query().Get("notable") == "true"
	limitStr := r.URL.Query().Get("limit")
	analyticsManagers := s.getAnalyticsManagers()

	limit := 100
	if limitStr != "" {
		if l, err := strconv.Atoi(limitStr); err == nil && l > 0 {
			limit = l
		}
	}

	w.Header().Set("Content-Type", "application/json")

	var items []bot.AnalyticsItemDrop

	if character != "" {
		am, exists := analyticsManagers[character]
		if !exists {
			http.Error(w, "Character not found", http.StatusNotFound)
			return
		}
		session := am.GetSessionSummary()
		if notableOnly {
			items = session.NotableDrops
		} else {
			// Get items from run history
			for _, run := range am.GetRunHistory(limit) {
				items = append(items, run.ItemsFound...)
			}
		}
	} else {
		// Combine from all managers
		for _, am := range analyticsManagers {
			session := am.GetSessionSummary()
			if notableOnly {
				items = append(items, session.NotableDrops...)
			} else {
				managerLimit := limit
				if len(analyticsManagers) > 0 {
					managerLimit = limit / len(analyticsManagers)
				}
				for _, run := range am.GetRunHistory(managerLimit) {
					items = append(items, run.ItemsFound...)
				}
			}
		}
	}

	// Limit and reverse (most recent first)
	if len(items) > limit {
		items = items[len(items)-limit:]
	}

	// Reverse to show most recent first
	for i, j := 0, len(items)-1; i < j; i, j = i+1, j-1 {
		items[i], items[j] = items[j], items[i]
	}

	json.NewEncoder(w).Encode(items)
}

// analyticsCharactersAPI returns list of characters with analytics
func (s *HttpServer) analyticsCharactersAPI(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	analyticsManagers := s.getAnalyticsManagers()

	characters := make([]string, 0, len(analyticsManagers))
	for name := range analyticsManagers {
		characters = append(characters, name)
	}

	json.NewEncoder(w).Encode(characters)
}

// analyticsResetAPI clears all analytics data
func (s *HttpServer) analyticsResetAPI(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	w.Header().Set("Content-Type", "application/json")

	character := r.URL.Query().Get("character")

	basePath, err := os.Getwd()
	if err != nil {
		http.Error(w, "Failed to get working directory", http.StatusInternalServerError)
		return
	}

	var errors []string

	if character != "" {
		// Reset specific character
		if err := s.resetCharacterAnalytics(basePath, character); err != nil {
			errors = append(errors, err.Error())
		}
	} else {
		// Reset all characters
		for charName := range config.GetCharacters() {
			if err := s.resetCharacterAnalytics(basePath, charName); err != nil {
				errors = append(errors, err.Error())
			}
		}
		// Also reset global analytics
		globalPath := basePath + "/config/analytics"
		if err := os.RemoveAll(globalPath); err != nil && !os.IsNotExist(err) {
			errors = append(errors, "Failed to reset global analytics: "+err.Error())
		}
	}

	// Reset in-memory data for running supervisors
	for name, am := range s.manager.GetAllAnalyticsManagers() {
		if character == "" || name == character {
			am.Reset()
		}
	}

	// Invalidate cached analytics managers so stale disk-loaded data is
	// not served after a reset.
	if character != "" {
		s.invalidateAnalyticsCache(character)
	} else {
		s.invalidateAnalyticsCache()
	}

	if len(errors) > 0 {
		json.NewEncoder(w).Encode(map[string]interface{}{
			"success": false,
			"errors":  errors,
		})
		return
	}

	json.NewEncoder(w).Encode(map[string]interface{}{
		"success": true,
		"message": "Analytics data cleared successfully",
	})
}

// resetCharacterAnalytics removes analytics files for a specific character
func (s *HttpServer) resetCharacterAnalytics(basePath, character string) error {
	analyticsPath := basePath + "/config/" + character + "/analytics"
	if err := os.RemoveAll(analyticsPath); err != nil && !os.IsNotExist(err) {
		return err
	}
	return nil
}

// analyticsDeathsAPI returns death statistics by area
func (s *HttpServer) analyticsDeathsAPI(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")

	character := r.URL.Query().Get("character")
	analyticsManagers := s.getAnalyticsManagers()

	// Combined deaths by area across all/selected characters
	combinedDeaths := make(map[string]*bot.AreaDeathStats)
	recentDeaths := make([]bot.DeathRecord, 0)

	for name, am := range analyticsManagers {
		if character != "" && name != character {
			continue
		}

		session := am.GetSessionSummary()
		if session == nil {
			continue
		}

		// Merge deaths by area
		for areaName, stats := range session.DeathsByArea {
			if combinedDeaths[areaName] == nil {
				combinedDeaths[areaName] = &bot.AreaDeathStats{
					AreaID:   stats.AreaID,
					AreaName: stats.AreaName,
				}
			}
			combinedDeaths[areaName].TotalDeaths += stats.TotalDeaths
			combinedDeaths[areaName].MonsterDeaths += stats.MonsterDeaths
			combinedDeaths[areaName].EnvironmentDeaths += stats.EnvironmentDeaths
			combinedDeaths[areaName].UnknownDeaths += stats.UnknownDeaths
		}

		// Collect recent deaths
		recentDeaths = append(recentDeaths, session.RecentDeaths...)
	}

	// Sort recent deaths by timestamp (most recent first) and limit to 20
	for i := 0; i < len(recentDeaths); i++ {
		for j := i + 1; j < len(recentDeaths); j++ {
			if recentDeaths[j].Timestamp.After(recentDeaths[i].Timestamp) {
				recentDeaths[i], recentDeaths[j] = recentDeaths[j], recentDeaths[i]
			}
		}
	}
	if len(recentDeaths) > 20 {
		recentDeaths = recentDeaths[:20]
	}

	json.NewEncoder(w).Encode(map[string]interface{}{
		"byArea":       combinedDeaths,
		"recentDeaths": recentDeaths,
	})
}

// analyticsRunesAPI returns rune breakdown data
func (s *HttpServer) analyticsRunesAPI(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")

	character := r.URL.Query().Get("character")
	analyticsManagers := s.getAnalyticsManagers()

	combinedRunes := make(map[string]int)

	for name, am := range analyticsManagers {
		if character != "" && name != character {
			continue
		}

		session := am.GetSessionSummary()
		if session == nil {
			continue
		}

		for runeName, count := range session.RuneBreakdown {
			combinedRunes[runeName] += count
		}
	}

	json.NewEncoder(w).Encode(combinedRunes)
}

// analyticsSessionHistoryAPI returns list of past session summaries
func (s *HttpServer) analyticsSessionHistoryAPI(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")

	character := r.URL.Query().Get("character")
	limitStr := r.URL.Query().Get("limit")
	analyticsManagers := s.getAnalyticsManagers()

	limit := 30
	if limitStr != "" {
		if l, err := strconv.Atoi(limitStr); err == nil && l > 0 {
			limit = l
		}
	}

	var allEntries []bot.SessionHistoryEntry

	for name, am := range analyticsManagers {
		if character != "" && name != character {
			continue
		}
		entries := am.GetSessionHistory(limit)
		allEntries = append(allEntries, entries...)
	}

	// Sort by date descending
	for i := 0; i < len(allEntries); i++ {
		for j := i + 1; j < len(allEntries); j++ {
			if allEntries[j].Date > allEntries[i].Date {
				allEntries[i], allEntries[j] = allEntries[j], allEntries[i]
			}
		}
	}

	if len(allEntries) > limit {
		allEntries = allEntries[:limit]
	}

	json.NewEncoder(w).Encode(allEntries)
}

// analyticsComparisonAPI returns per-character comparison data
func (s *HttpServer) analyticsComparisonAPI(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	analyticsManagers := s.getAnalyticsManagers()

	var entries []bot.CharacterComparisonEntry

	for _, am := range analyticsManagers {
		entry := am.GetCharacterComparison()
		if entry != nil && entry.TotalRuns > 0 {
			entries = append(entries, *entry)
		}
	}

	json.NewEncoder(w).Encode(entries)
}
