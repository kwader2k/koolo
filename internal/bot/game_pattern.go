package bot

import (
	"fmt"
	"sync"
)

// GamePatternTracker tracks game name patterns and auto-increments game numbers
type GamePatternTracker struct {
	mu           sync.Mutex
	patterns     map[string]*PatternState // keyed by leader name
}

// PatternState holds the state for a specific game pattern
type PatternState struct {
	Pattern     string
	Password    string
	GameNumber  int
	LastGame    string
}

// globalPatternTracker is the singleton pattern tracker
var (
	globalPatternTracker *GamePatternTracker
	patternTrackerOnce   sync.Once
)

// GetPatternTracker returns the global pattern tracker
func GetPatternTracker() *GamePatternTracker {
	patternTrackerOnce.Do(func() {
		globalPatternTracker = &GamePatternTracker{
			patterns: make(map[string]*PatternState),
		}
	})
	return globalPatternTracker
}

// RegisterPattern registers a new pattern for a leader
func (pt *GamePatternTracker) RegisterPattern(leaderName, pattern, password string) {
	pt.mu.Lock()
	defer pt.mu.Unlock()

	pt.patterns[leaderName] = &PatternState{
		Pattern:    pattern,
		Password:   password,
		GameNumber: 0,
	}
}

// GetNextGameName returns the next game name for a leader and increments the counter
func (pt *GamePatternTracker) GetNextGameName(leaderName string) (string, string, bool) {
	pt.mu.Lock()
	defer pt.mu.Unlock()

	state, exists := pt.patterns[leaderName]
	if !exists {
		return "", "", false
	}

	state.GameNumber++
	gameName := fmt.Sprintf("%s%d", state.Pattern, state.GameNumber)
	state.LastGame = gameName

	return gameName, state.Password, true
}

// GetCurrentGameName returns the current game name without incrementing
func (pt *GamePatternTracker) GetCurrentGameName(leaderName string) (string, string, bool) {
	pt.mu.Lock()
	defer pt.mu.Unlock()

	state, exists := pt.patterns[leaderName]
	if !exists {
		return "", "", false
	}

	if state.GameNumber == 0 {
		return "", state.Password, true
	}

	gameName := fmt.Sprintf("%s%d", state.Pattern, state.GameNumber)
	return gameName, state.Password, true
}

// PeekNextGameName returns what the next game name would be without incrementing
func (pt *GamePatternTracker) PeekNextGameName(leaderName string) (string, string, bool) {
	pt.mu.Lock()
	defer pt.mu.Unlock()

	state, exists := pt.patterns[leaderName]
	if !exists {
		return "", "", false
	}

	gameName := fmt.Sprintf("%s%d", state.Pattern, state.GameNumber+1)
	return gameName, state.Password, true
}

// SetGameNumber sets the game number for a leader (useful for syncing)
func (pt *GamePatternTracker) SetGameNumber(leaderName string, number int) {
	pt.mu.Lock()
	defer pt.mu.Unlock()

	if state, exists := pt.patterns[leaderName]; exists {
		state.GameNumber = number
		state.LastGame = fmt.Sprintf("%s%d", state.Pattern, number)
	}
}

// GetGameNumber returns the current game number for a leader
func (pt *GamePatternTracker) GetGameNumber(leaderName string) int {
	pt.mu.Lock()
	defer pt.mu.Unlock()

	if state, exists := pt.patterns[leaderName]; exists {
		return state.GameNumber
	}
	return 0
}

// ResetPattern resets the game counter for a leader
func (pt *GamePatternTracker) ResetPattern(leaderName string) {
	pt.mu.Lock()
	defer pt.mu.Unlock()

	if state, exists := pt.patterns[leaderName]; exists {
		state.GameNumber = 0
		state.LastGame = ""
	}
}

// RemovePattern removes a pattern for a leader
func (pt *GamePatternTracker) RemovePattern(leaderName string) {
	pt.mu.Lock()
	defer pt.mu.Unlock()
	delete(pt.patterns, leaderName)
}

// UpdatePassword updates the password for a leader's pattern
func (pt *GamePatternTracker) UpdatePassword(leaderName, password string) {
	pt.mu.Lock()
	defer pt.mu.Unlock()

	if state, exists := pt.patterns[leaderName]; exists {
		state.Password = password
	}
}

// GetAllPatterns returns all registered patterns (for debugging/UI)
func (pt *GamePatternTracker) GetAllPatterns() map[string]PatternState {
	pt.mu.Lock()
	defer pt.mu.Unlock()

	result := make(map[string]PatternState)
	for k, v := range pt.patterns {
		result[k] = *v
	}
	return result
}

