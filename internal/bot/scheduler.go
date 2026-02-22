package bot

import (
	"encoding/json"
	"log/slog"
	"math/rand"
	"os"
	"path/filepath"
	"sort"
	"sync"
	"time"

	"github.com/hectorgimenez/koolo/internal/config"
)

// SchedulerPhase represents the current phase in duration mode
type SchedulerPhase string

const (
	PhaseResting  SchedulerPhase = "resting"
	PhasePlaying  SchedulerPhase = "playing"
	PhaseOnBreak  SchedulerPhase = "onBreak"
)

// ScheduledBreak represents a pre-calculated break
type ScheduledBreak struct {
	Type      string    `json:"type"`      // "meal" or "short"
	StartTime time.Time `json:"startTime"` // When break starts
	Duration  int       `json:"duration"`  // Duration in minutes
}

// DurationState tracks the current state for duration mode
type DurationState struct {
	CurrentPhase              SchedulerPhase   `json:"currentPhase"`
	PhaseStartTime            time.Time        `json:"phaseStartTime"`
	PhaseEndTime              time.Time        `json:"phaseEndTime"`
	TodayWakeTime             time.Time        `json:"todayWakeTime"`
	TodayRestTime             time.Time        `json:"todayRestTime"`
	PlayedMinutes             int              `json:"playedMinutes"`
	PlayedMinutesAtPhaseStart int              `json:"playedMinutesAtPhaseStart"` // Accumulated play time when current play session started
	ScheduledBreaks           []ScheduledBreak `json:"scheduledBreaks"`
	CurrentBreakIdx           int              `json:"currentBreakIdx"`
	LastUpdated               time.Time        `json:"lastUpdated"`
	LastSeenRunning           time.Time        `json:"lastSeenRunning"` // Last time bot was observed running (for manual stop/start detection)
}

// HistoryEntry represents one day's play session
type HistoryEntry struct {
	Date             string           `json:"date"`
	WakeTime         string           `json:"wakeTime"`
	SleepTime        string           `json:"sleepTime"`
	TotalPlayMinutes int              `json:"totalPlayMinutes"`
	TotalBreakMinutes int             `json:"totalBreakMinutes"`
	Breaks           []ScheduledBreak `json:"breaks"`
}

// SchedulerHistory stores the last 30 days of play history
type SchedulerHistory struct {
	History []HistoryEntry `json:"history"`
}

type Scheduler struct {
	manager       *SupervisorManager
	logger        *slog.Logger
	stop          chan struct{}

	// Duration mode state (per supervisor)
	durationState map[string]*DurationState
	stateMux      sync.RWMutex
}

func NewScheduler(manager *SupervisorManager, logger *slog.Logger) *Scheduler {
	s := &Scheduler{
		manager:       manager,
		logger:        logger,
		stop:          make(chan struct{}),
		durationState: make(map[string]*DurationState),
	}

	// Load persisted state for all characters
	s.loadAllStates()

	return s
}

func (s *Scheduler) Start() {
	s.logger.Info("Scheduler started")
	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			s.checkSchedules()
		case <-s.stop:
			s.logger.Info("Scheduler stopped")
			return
		}
	}
}

func (s *Scheduler) Stop() {
	close(s.stop)
}

func (s *Scheduler) checkSchedules() {
	for supervisorName, cfg := range config.GetCharacters() {
		if !cfg.Scheduler.Enabled {
			continue
		}

		// Route to appropriate scheduler based on mode
		mode := cfg.Scheduler.Mode
		if mode == "" {
			mode = "timeSlots" // Default to time slots for backward compatibility
		}

		switch mode {
		case "duration":
			s.checkDurationSchedule(supervisorName, cfg)
		default: // "timeSlots"
			s.checkTimeSlotsSchedule(supervisorName, cfg)
		}
	}
}

// checkTimeSlotsSchedule handles the original time-based scheduling
func (s *Scheduler) checkTimeSlotsSchedule(supervisorName string, cfg *config.CharacterCfg) {
	now := time.Now()
	currentDay := int(now.Weekday())

	for _, day := range cfg.Scheduler.Days {
		if day.DayOfWeek != currentDay {
			continue
		}

		actionTaken := false

		// Check if any time range is active
		for _, timeRange := range day.TimeRanges {
			// Apply variance to start and end times
			startVariance := timeRange.StartVarianceMin
			if startVariance == 0 {
				startVariance = cfg.Scheduler.GlobalVarianceMin
			}
			endVariance := timeRange.EndVarianceMin
			if endVariance == 0 {
				endVariance = cfg.Scheduler.GlobalVarianceMin
			}

			// Get deterministic variance for today (same offset all day)
			startOffset := s.getDeterministicOffset(supervisorName, now, "start", startVariance)
			endOffset := s.getDeterministicOffset(supervisorName, now, "end", endVariance)

			start := time.Date(now.Year(), now.Month(), now.Day(), timeRange.Start.Hour(), timeRange.Start.Minute(), 0, 0, now.Location())
			start = start.Add(time.Duration(startOffset) * time.Minute)

			end := time.Date(now.Year(), now.Month(), now.Day(), timeRange.End.Hour(), timeRange.End.Minute(), 0, 0, now.Location())
			end = end.Add(time.Duration(endOffset) * time.Minute)

			if now.After(start) && now.Before(end) && s.supervisorNotStarted(supervisorName) {
				s.logger.Info("Starting supervisor based on schedule",
					"supervisor", supervisorName,
					"timeRange", start.Format("15:04")+" - "+end.Format("15:04"))
				go s.startSupervisor(supervisorName)
				actionTaken = true
				break
			} else if (now.After(end) || now.Equal(end) || now.Before(start)) && !s.supervisorNotStarted(supervisorName) {
				s.logger.Info("Stopping supervisor based on schedule",
					"supervisor", supervisorName,
					"timeRange", start.Format("15:04")+" - "+end.Format("15:04"))
				s.stopSupervisor(supervisorName)
				actionTaken = true
				break
			}
		}

		if actionTaken {
			break
		}
	}
}

// checkDurationSchedule handles the duration-based scheduling
func (s *Scheduler) checkDurationSchedule(supervisorName string, cfg *config.CharacterCfg) {
	now := time.Now()

	// Get or create state for this supervisor
	state := s.getOrCreateState(supervisorName, cfg)

	// Check if we need to reset for a new day
	if s.isNewDay(state, now) {
		s.initializeNewDay(supervisorName, cfg, state, now)
	}

	switch state.CurrentPhase {
	case PhaseResting:
		// Check if bot was manually started during rest
		if !s.supervisorNotStarted(supervisorName) {
			totalPlayMinutes := cfg.Scheduler.Duration.PlayHours * 60

			if state.PlayedMinutes >= totalPlayMinutes {
				// Already completed full session today - stop the bot
				s.logger.Info("Manual start but already completed today's play time, stopping",
					"supervisor", supervisorName,
					"playedMinutes", state.PlayedMinutes)
				s.stopSupervisor(supervisorName)
			} else if state.PlayedMinutes > 0 {
				// Partially played - give remaining time
				remainingMinutes := totalPlayMinutes - state.PlayedMinutes
				s.logger.Info("Manual start during rest, giving remaining play time",
					"supervisor", supervisorName,
					"playedMinutes", state.PlayedMinutes,
					"remainingMinutes", remainingMinutes)
				s.resumeWithRemainingTime(supervisorName, cfg, state, now, remainingMinutes)
				s.transitionToPlaying(supervisorName, state, now)
			} else {
				// Never played today (forgot) - give full schedule from now
				s.logger.Info("Manual start during rest (never played), full schedule from now",
					"supervisor", supervisorName)
				s.recalculateScheduleFromNow(supervisorName, cfg, state, now)
				s.transitionToPlaying(supervisorName, state, now)
			}
			return
		}

		// Normal wake time check - also verify we're before rest time to prevent restart loop
		if (now.After(state.TodayWakeTime) || now.Equal(state.TodayWakeTime)) && now.Before(state.TodayRestTime) {
			s.transitionToPlaying(supervisorName, state, now)
		}

	case PhasePlaying:
		// Skip any breaks that are already completely in the past (e.g., manual late start)
		skippedBreaks := 0
		for state.CurrentBreakIdx < len(state.ScheduledBreaks) {
			nextBreak := state.ScheduledBreaks[state.CurrentBreakIdx]
			breakEndTime := nextBreak.StartTime.Add(time.Duration(nextBreak.Duration) * time.Minute)
			if now.After(breakEndTime) {
				// This break's end time is in the past, skip it
				state.CurrentBreakIdx++
				skippedBreaks++
			} else {
				break
			}
		}
		if skippedBreaks > 0 {
			s.logger.Info("Skipped past breaks on late start",
				"supervisor", supervisorName,
				"skippedCount", skippedBreaks,
				"nextBreakIdx", state.CurrentBreakIdx)
			s.saveState(supervisorName, state)
		}

		// Check if it's time for a break
		if state.CurrentBreakIdx < len(state.ScheduledBreaks) {
			nextBreak := state.ScheduledBreaks[state.CurrentBreakIdx]
			if now.After(nextBreak.StartTime) || now.Equal(nextBreak.StartTime) {
				s.transitionToBreak(supervisorName, state, nextBreak, now)
			}
		}

		// Detect manual stop/start and update played time
		botRunning := !s.supervisorNotStarted(supervisorName)
		if botRunning {
			// Bot is running - check if it just resumed after being stopped
			if state.LastSeenRunning.IsZero() || now.Sub(state.LastSeenRunning) > 2*time.Minute {
				// Bot was stopped and just restarted - reset phase timing
				s.logger.Info("Detected bot resume after manual stop, resetting phase timing",
					"supervisor", supervisorName,
					"lastSeen", state.LastSeenRunning,
					"accumulatedMinutes", state.PlayedMinutes)
				state.PhaseStartTime = now
				state.PlayedMinutesAtPhaseStart = state.PlayedMinutes
			}
			state.LastSeenRunning = now

			// Update played time (accumulated + current session)
			elapsed := int(now.Sub(state.PhaseStartTime).Minutes())
			state.PlayedMinutes = state.PlayedMinutesAtPhaseStart + elapsed
			s.saveState(supervisorName, state)
		}

		// Check if we've played enough for today using pre-calculated rest time
		if now.After(state.TodayRestTime) || now.Equal(state.TodayRestTime) {
			s.transitionToResting(supervisorName, state, now)
		}

	case PhaseOnBreak:
		// Check if break is over
		if now.After(state.PhaseEndTime) || now.Equal(state.PhaseEndTime) {
			state.CurrentBreakIdx++ // Increment BEFORE transition so it gets persisted
			s.transitionToPlaying(supervisorName, state, now)
		}
	}
}

// getOrCreateState gets existing state or creates new one
func (s *Scheduler) getOrCreateState(supervisorName string, cfg *config.CharacterCfg) *DurationState {
	s.stateMux.Lock()
	defer s.stateMux.Unlock()

	if state, exists := s.durationState[supervisorName]; exists {
		return state
	}

	// Try to load from file
	state := s.loadState(supervisorName)
	if state == nil {
		// Create new state
		state = &DurationState{
			CurrentPhase:    PhaseResting,
			ScheduledBreaks: []ScheduledBreak{},
		}
	}

	s.durationState[supervisorName] = state
	return state
}

// isNewDay checks if we need to reinitialize for a new day
func (s *Scheduler) isNewDay(state *DurationState, now time.Time) bool {
	if state.TodayWakeTime.IsZero() {
		return true
	}
	// Check year, month, and day to handle year boundaries correctly
	return now.Year() != state.TodayWakeTime.Year() ||
		now.Month() != state.TodayWakeTime.Month() ||
		now.Day() != state.TodayWakeTime.Day()
}

// initializeNewDay sets up the schedule for a new day
func (s *Scheduler) initializeNewDay(supervisorName string, cfg *config.CharacterCfg, state *DurationState, now time.Time) {
	duration := cfg.Scheduler.Duration

	// Save history for previous day if we have data
	if !state.TodayWakeTime.IsZero() && state.PlayedMinutes > 0 {
		s.saveHistory(supervisorName, state)
	}

	// Parse wake up time
	wakeHour, wakeMin := 8, 0
	if duration.WakeUpTime != "" {
		parsed, err := time.Parse("15:04", duration.WakeUpTime)
		if err == nil {
			wakeHour, wakeMin = parsed.Hour(), parsed.Minute()
		}
	}

	// Apply variance to wake time
	wakeVariance := s.applyJitter(duration.WakeUpVariance, duration.JitterMin, duration.JitterMax)
	wakeOffset := s.randomInRange(-wakeVariance, wakeVariance)

	wakeTime := time.Date(now.Year(), now.Month(), now.Day(), wakeHour, wakeMin, 0, 0, now.Location())
	wakeTime = wakeTime.Add(time.Duration(wakeOffset) * time.Minute)

	// Calculate actual play hours with variance
	playHours := duration.PlayHours
	if playHours < 1 {
		playHours = 1 // Minimum 1 hour
	}
	playVariance := s.applyJitter(duration.PlayHoursVariance*60, duration.JitterMin, duration.JitterMax) / 60
	playOffset := s.randomInRange(-playVariance, playVariance)
	actualPlayHours := playHours + playOffset
	if actualPlayHours < 1 {
		actualPlayHours = 1 // Ensure at least 1 hour of play time
	}

	// Generate break schedule first so we can include break time in rest calculation
	breaks := s.generateBreakSchedule(cfg, wakeTime, actualPlayHours)

	// Calculate total break duration
	totalBreakMinutes := 0
	for _, brk := range breaks {
		totalBreakMinutes += brk.Duration
	}

	// Rest time = wake + play hours + break time
	restTime := wakeTime.Add(time.Duration(actualPlayHours)*time.Hour + time.Duration(totalBreakMinutes)*time.Minute)

	// Update state
	state.TodayWakeTime = wakeTime
	state.TodayRestTime = restTime
	state.CurrentPhase = PhaseResting
	state.PlayedMinutes = 0
	state.ScheduledBreaks = breaks
	state.CurrentBreakIdx = 0
	state.LastUpdated = now

	s.logger.Info("Initialized new day schedule",
		"supervisor", supervisorName,
		"wakeTime", wakeTime.Format("15:04"),
		"restTime", restTime.Format("15:04"),
		"playHours", actualPlayHours,
		"breaks", len(breaks))

	s.saveState(supervisorName, state)
}

// generateBreakSchedule creates the break schedule for the day
func (s *Scheduler) generateBreakSchedule(cfg *config.CharacterCfg, wakeTime time.Time, playHours int) []ScheduledBreak {
	duration := cfg.Scheduler.Duration
	totalBreaks := duration.MealBreakCount + duration.ShortBreakCount

	if totalBreaks == 0 {
		return []ScheduledBreak{}
	}

	totalPlayMinutes := playHours * 60
	segmentLength := totalPlayMinutes / (totalBreaks + 1)

	breaks := make([]ScheduledBreak, 0, totalBreaks)

	// Create all break slots
	breakSlots := make([]int, totalBreaks)
	for i := 0; i < totalBreaks; i++ {
		baseTime := segmentLength * (i + 1)
		timingVariance := s.applyJitter(duration.BreakTimingVariance, duration.JitterMin, duration.JitterMax)
		offset := s.randomInRange(-timingVariance, timingVariance)
		breakSlots[i] = baseTime + offset
	}

	// Sort slots
	sort.Ints(breakSlots)

	// Assign meal breaks to slots closest to typical meal times (4h and 11h into play for lunch/dinner)
	mealSlotIndices := s.pickMealSlots(breakSlots, duration.MealBreakCount, playHours)

	for i, slotMinutes := range breakSlots {
		breakTime := wakeTime.Add(time.Duration(slotMinutes) * time.Minute)

		var breakType string
		var baseDuration, variance int

		if contains(mealSlotIndices, i) {
			breakType = "meal"
			baseDuration = duration.MealBreakDuration
			variance = s.applyJitter(duration.MealBreakVariance, duration.JitterMin, duration.JitterMax)
		} else {
			breakType = "short"
			baseDuration = duration.ShortBreakDuration
			variance = s.applyJitter(duration.ShortBreakVariance, duration.JitterMin, duration.JitterMax)
		}

		durationOffset := s.randomInRange(-variance, variance)
		actualDuration := baseDuration + durationOffset
		if actualDuration < 1 {
			actualDuration = 1
		}

		breaks = append(breaks, ScheduledBreak{
			Type:      breakType,
			StartTime: breakTime,
			Duration:  actualDuration,
		})
	}

	return breaks
}

// pickMealSlots selects which break slots should be meal breaks
func (s *Scheduler) pickMealSlots(breakSlots []int, mealCount int, playHours int) []int {
	if mealCount == 0 || len(breakSlots) == 0 {
		return []int{}
	}

	// Ideal meal times: lunch at 4h into play, dinner at 11h
	idealMealMinutes := []int{4 * 60, 11 * 60}

	selected := make([]int, 0, mealCount)
	used := make(map[int]bool)

	for _, idealMin := range idealMealMinutes {
		if len(selected) >= mealCount {
			break
		}

		// Find closest unused slot to this ideal time
		bestIdx := -1
		bestDist := int(^uint(0) >> 1) // Max int

		for i, slotMin := range breakSlots {
			if used[i] {
				continue
			}
			dist := abs(slotMin - idealMin)
			if dist < bestDist {
				bestDist = dist
				bestIdx = i
			}
		}

		if bestIdx >= 0 {
			selected = append(selected, bestIdx)
			used[bestIdx] = true
		}
	}

	return selected
}

// transitionToPlaying starts or resumes playing
func (s *Scheduler) transitionToPlaying(supervisorName string, state *DurationState, now time.Time) {
	state.CurrentPhase = PhasePlaying
	state.PhaseStartTime = now
	state.PlayedMinutesAtPhaseStart = state.PlayedMinutes // Save accumulated time before this session
	state.LastUpdated = now

	s.logger.Info("Duration scheduler: transitioning to PLAYING",
		"supervisor", supervisorName,
		"playedMinutes", state.PlayedMinutes)

	if s.supervisorNotStarted(supervisorName) {
		go s.startSupervisor(supervisorName)
	}

	s.saveState(supervisorName, state)
}

// transitionToBreak starts a break
func (s *Scheduler) transitionToBreak(supervisorName string, state *DurationState, brk ScheduledBreak, now time.Time) {
	state.CurrentPhase = PhaseOnBreak
	state.PhaseStartTime = now
	state.PhaseEndTime = now.Add(time.Duration(brk.Duration) * time.Minute)
	state.LastUpdated = now
	state.LastSeenRunning = time.Time{} // Clear so resume detection works correctly

	s.logger.Info("Duration scheduler: transitioning to BREAK",
		"supervisor", supervisorName,
		"breakType", brk.Type,
		"duration", brk.Duration,
		"resumeAt", state.PhaseEndTime.Format("15:04"))

	s.stopSupervisor(supervisorName)
	s.saveState(supervisorName, state)
}

// transitionToResting starts the rest period
func (s *Scheduler) transitionToResting(supervisorName string, state *DurationState, now time.Time) {
	state.CurrentPhase = PhaseResting
	state.PhaseStartTime = now
	state.LastUpdated = now
	state.LastSeenRunning = time.Time{} // Clear so resume detection works correctly

	s.logger.Info("Duration scheduler: transitioning to RESTING",
		"supervisor", supervisorName,
		"playedMinutes", state.PlayedMinutes,
		"nextWake", "tomorrow")

	s.stopSupervisor(supervisorName)
	s.saveState(supervisorName, state)
}

// resumeWithRemainingTime handles manual start during rest when partially played
func (s *Scheduler) resumeWithRemainingTime(supervisorName string, cfg *config.CharacterCfg, state *DurationState, now time.Time, remainingMinutes int) {
	// Set rest time to now + remaining minutes (no breaks for partial sessions)
	state.TodayRestTime = now.Add(time.Duration(remainingMinutes) * time.Minute)
	state.ScheduledBreaks = []ScheduledBreak{} // No breaks for short remaining time
	state.CurrentBreakIdx = 0
	// Keep PlayedMinutes as-is so we continue accumulating

	s.logger.Info("Resuming with remaining play time",
		"supervisor", supervisorName,
		"remainingMinutes", remainingMinutes,
		"newRestTime", state.TodayRestTime.Format("15:04"))

	s.saveState(supervisorName, state)
}

// recalculateScheduleFromNow handles manual start during rest when never played today
func (s *Scheduler) recalculateScheduleFromNow(supervisorName string, cfg *config.CharacterCfg, state *DurationState, now time.Time) {
	duration := cfg.Scheduler.Duration

	// Use current time as new wake time
	state.TodayWakeTime = now

	// Recalculate play hours and rest time
	playHours := duration.PlayHours
	if playHours < 1 {
		playHours = 1
	}

	// Generate new break schedule
	breaks := s.generateBreakSchedule(cfg, now, playHours)

	// Calculate total break time
	totalBreakMinutes := 0
	for _, brk := range breaks {
		totalBreakMinutes += brk.Duration
	}

	// New rest time = now + play hours + breaks
	state.TodayRestTime = now.Add(time.Duration(playHours)*time.Hour + time.Duration(totalBreakMinutes)*time.Minute)
	state.ScheduledBreaks = breaks
	state.CurrentBreakIdx = 0
	state.PlayedMinutes = 0

	s.logger.Info("Recalculated schedule for manual start",
		"supervisor", supervisorName,
		"newWakeTime", now.Format("15:04"),
		"newRestTime", state.TodayRestTime.Format("15:04"))

	s.saveState(supervisorName, state)
}

// applyJitter scales variance by a random jitter multiplier
func (s *Scheduler) applyJitter(baseVariance int, jitterMin, jitterMax int) int {
	if jitterMin == 0 && jitterMax == 0 {
		return baseVariance
	}

	// Default jitter range if only partial config
	if jitterMin == 0 {
		jitterMin = 50
	}
	if jitterMax == 0 {
		jitterMax = 150
	}

	minMult := float64(jitterMin) / 100.0
	maxMult := float64(jitterMax) / 100.0

	multiplier := minMult + rand.Float64()*(maxMult-minMult)
	return int(float64(baseVariance) * multiplier)
}

// randomInRange returns a random int in [min, max]
func (s *Scheduler) randomInRange(min, max int) int {
	if min >= max {
		return min
	}
	return min + rand.Intn(max-min+1)
}

// getDeterministicOffset returns the same offset for a given day/context
func (s *Scheduler) getDeterministicOffset(supervisorName string, now time.Time, context string, variance int) int {
	if variance == 0 {
		return 0
	}

	// Create deterministic seed from supervisor + date + context
	seedStr := supervisorName + now.Format("2006-01-02") + context
	seed := int64(0)
	for _, c := range seedStr {
		seed = seed*31 + int64(c)
	}

	// Use seed to generate offset
	r := rand.New(rand.NewSource(seed))
	return r.Intn(variance*2+1) - variance
}

// State persistence functions

func (s *Scheduler) getStatePath(supervisorName string) string {
	return filepath.Join("config", supervisorName, "scheduler_state.json")
}

func (s *Scheduler) getHistoryPath(supervisorName string) string {
	return filepath.Join("config", supervisorName, "scheduler_history.json")
}

func (s *Scheduler) loadState(supervisorName string) *DurationState {
	path := s.getStatePath(supervisorName)
	data, err := os.ReadFile(path)
	if err != nil {
		return nil
	}

	var state DurationState
	if err := json.Unmarshal(data, &state); err != nil {
		s.logger.Error("Failed to parse scheduler state", "supervisor", supervisorName, "error", err)
		return nil
	}

	return &state
}

func (s *Scheduler) saveState(supervisorName string, state *DurationState) {
	path := s.getStatePath(supervisorName)

	// Ensure directory exists
	dir := filepath.Dir(path)
	if err := os.MkdirAll(dir, 0755); err != nil {
		s.logger.Error("Failed to create state directory", "error", err)
		return
	}

	data, err := json.MarshalIndent(state, "", "  ")
	if err != nil {
		s.logger.Error("Failed to marshal scheduler state", "error", err)
		return
	}

	if err := os.WriteFile(path, data, 0644); err != nil {
		s.logger.Error("Failed to save scheduler state", "error", err)
	}
}

func (s *Scheduler) loadAllStates() {
	for supervisorName := range config.GetCharacters() {
		if state := s.loadState(supervisorName); state != nil {
			s.durationState[supervisorName] = state
		}
	}
}

func (s *Scheduler) saveHistory(supervisorName string, state *DurationState) {
	path := s.getHistoryPath(supervisorName)

	// Load existing history
	var history SchedulerHistory
	data, err := os.ReadFile(path)
	if err == nil {
		json.Unmarshal(data, &history)
	}

	// Calculate total break minutes
	totalBreakMinutes := 0
	for _, brk := range state.ScheduledBreaks {
		totalBreakMinutes += brk.Duration
	}

	// Add new entry
	entry := HistoryEntry{
		Date:              state.TodayWakeTime.Format("2006-01-02"),
		WakeTime:          state.TodayWakeTime.Format("15:04"),
		SleepTime:         state.TodayRestTime.Format("15:04"),
		TotalPlayMinutes:  state.PlayedMinutes,
		TotalBreakMinutes: totalBreakMinutes,
		Breaks:            state.ScheduledBreaks,
	}

	history.History = append([]HistoryEntry{entry}, history.History...)

	// Prune to 30 days
	if len(history.History) > 30 {
		history.History = history.History[:30]
	}

	// Save
	dir := filepath.Dir(path)
	os.MkdirAll(dir, 0755)

	data, _ = json.MarshalIndent(history, "", "  ")
	os.WriteFile(path, data, 0644)
}

// GetDurationState returns the current duration state for a supervisor (for UI display)
func (s *Scheduler) GetDurationState(supervisorName string) *DurationState {
	s.stateMux.RLock()
	defer s.stateMux.RUnlock()

	if state, exists := s.durationState[supervisorName]; exists {
		return state
	}
	return nil
}

// GetSchedulerHistory returns the play history for a supervisor
func (s *Scheduler) GetSchedulerHistory(supervisorName string) *SchedulerHistory {
	path := s.getHistoryPath(supervisorName)
	data, err := os.ReadFile(path)
	if err != nil {
		return &SchedulerHistory{History: []HistoryEntry{}}
	}

	var history SchedulerHistory
	if err := json.Unmarshal(data, &history); err != nil {
		return &SchedulerHistory{History: []HistoryEntry{}}
	}

	return &history
}

// Helper functions

func (s *Scheduler) supervisorNotStarted(name string) bool {
	stats := s.manager.GetSupervisorStats(name)
	return stats.SupervisorStatus == NotStarted || stats.SupervisorStatus == Crashed || stats.SupervisorStatus == ""
}

func (s *Scheduler) startSupervisor(name string) {
	if s.supervisorNotStarted(name) {
		err := s.manager.Start(name, false, false)
		if err != nil {
			s.logger.Error("Failed to start supervisor", "supervisor", name, "error", err)
		}
	}
}

func (s *Scheduler) stopSupervisor(name string) {
	if !s.supervisorNotStarted(name) {
		s.manager.Stop(name)
	}
}

func contains(slice []int, val int) bool {
	for _, item := range slice {
		if item == val {
			return true
		}
	}
	return false
}

func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}
