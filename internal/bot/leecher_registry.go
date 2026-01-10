package bot

import (
	"fmt"
	"log/slog"
	"math/rand"
	"sync"
)

// LeecherState represents the current state of a leecher
type LeecherState string

const (
	LeecherStateIdle         LeecherState = "idle"           // Not active
	LeecherStateWaiting      LeecherState = "waiting"        // Waiting at portal area
	LeecherStateFollowing    LeecherState = "following"      // Following leader
	LeecherStateInPortalArea LeecherState = "in_portal_area" // At portal spawn location
	LeecherStateInDungeon    LeecherState = "in_dungeon"     // In dungeon with leader
)

// LeecherInfo contains information about a leecher supervisor
type LeecherInfo struct {
	LeaderSupervisor  string       // The supervisor name of the leader coordinator
	LeaderName        string       // The character name of the leader to follow
	GamePattern       string       // Game name pattern to look for
	GamePassword      string       // Password for games
	JoinDelayMin      int          // Minimum join delay in ms
	JoinDelayMax      int          // Maximum join delay in ms
	GameCounter       int          // Current game number for auto-incrementing
	PollInterval      int          // How often to check for portals/leader (ms)
	PortalEntryDelay  int          // Seconds to wait after "come" before entering
	MaxLeaderDistance int          // Max distance from leader for XP
	UseLegacyGraphics bool         // Use legacy graphics mode
	State             LeecherState // Current leecher state
	CurrentAct        int          // Current act the leecher is in
	ComeReceived      bool         // Whether a "come" command has been received
	TPReceived        bool         // Whether a "TP back to town" command has been received
	StayReceived      bool         // Whether a "stay" command has been received (pause following)
	ExitReceived      bool         // Whether an "exit game" command has been received
	JoinRetries       int          // Number of failed join attempts for current game
}

var (
	leecherRegistry = make(map[string]*LeecherInfo)
	leecherMutex    sync.RWMutex
)

// RegisterLeecher registers a supervisor as a leecher
func RegisterLeecher(supervisorName, leaderSupervisor, leaderName, gamePattern, gamePassword string,
	joinDelayMin, joinDelayMax, pollInterval, portalEntryDelay, maxLeaderDistance int, useLegacyGraphics bool) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	leecherRegistry[supervisorName] = &LeecherInfo{
		LeaderSupervisor:  leaderSupervisor,
		LeaderName:        leaderName,
		GamePattern:       gamePattern,
		GamePassword:      gamePassword,
		JoinDelayMin:      joinDelayMin,
		JoinDelayMax:      joinDelayMax,
		GameCounter:       1, // Start at 1
		PollInterval:      pollInterval,
		PortalEntryDelay:  portalEntryDelay,
		MaxLeaderDistance: maxLeaderDistance,
		UseLegacyGraphics: useLegacyGraphics,
		State:             LeecherStateIdle,
		CurrentAct:        1,
		ComeReceived:      false,
	}
}

// UnregisterLeecher removes a supervisor from the leecher registry
func UnregisterLeecher(supervisorName string) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	delete(leecherRegistry, supervisorName)
}

// GetLeecherInfo returns leecher info for a supervisor, or nil if not a leecher
func GetLeecherInfo(supervisorName string) *LeecherInfo {
	leecherMutex.RLock()
	defer leecherMutex.RUnlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		// Return a copy to prevent data races
		infoCopy := *info
		return &infoCopy
	}
	return nil
}

// IsLeecher checks if a supervisor is registered as a leecher
func IsLeecher(supervisorName string) bool {
	leecherMutex.RLock()
	defer leecherMutex.RUnlock()

	_, ok := leecherRegistry[supervisorName]
	return ok
}

// GetLeecherGameName returns the current game name for a leecher (pattern + counter)
func GetLeecherGameName(supervisorName string) string {
	leecherMutex.RLock()
	defer leecherMutex.RUnlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		return fmt.Sprintf("%s%d", info.GamePattern, info.GameCounter)
	}
	return ""
}

// IncrementLeecherGameCounter increments the game counter for a leecher
func IncrementLeecherGameCounter(supervisorName string) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		info.GameCounter++
	}
}

// SetLeecherState updates the state of a leecher
func SetLeecherState(supervisorName string, state LeecherState) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		info.State = state
	}
}

// GetLeecherState returns the current state of a leecher
func GetLeecherState(supervisorName string) LeecherState {
	leecherMutex.RLock()
	defer leecherMutex.RUnlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		return info.State
	}
	return LeecherStateIdle
}

// SetLeecherAct updates the current act of a leecher
func SetLeecherAct(supervisorName string, act int) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		info.CurrentAct = act
	}
}

// SetLeecherComeReceived sets whether a "come" command has been received
func SetLeecherComeReceived(supervisorName string, received bool) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		info.ComeReceived = received
	}
}

// GetLeecherComeReceived returns whether a "come" command has been received
func GetLeecherComeReceived(supervisorName string) bool {
	leecherMutex.RLock()
	defer leecherMutex.RUnlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		return info.ComeReceived
	}
	return false
}

// SetLeecherTPReceived sets whether a "TP back to town" command has been received
func SetLeecherTPReceived(supervisorName string, received bool) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		info.TPReceived = received
	}
}

// GetLeecherTPReceived returns whether a "TP back to town" command has been received
func GetLeecherTPReceived(supervisorName string) bool {
	leecherMutex.RLock()
	defer leecherMutex.RUnlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		return info.TPReceived
	}
	return false
}

// NotifyAllLeechersTP sets TPReceived to true for all leechers
func NotifyAllLeechersTP(leaderName string) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	for name, info := range leecherRegistry {
		if leaderName == "" || leaderName == "human_leader" || info.LeaderName == leaderName {
			info.TPReceived = true
			slog.Debug("Notified leecher of TP command", slog.String("leecher", name))
		}
	}
}

// SetLeecherStayReceived sets whether a "stay" command has been received
func SetLeecherStayReceived(supervisorName string, received bool) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		info.StayReceived = received
	}
}

// GetLeecherStayReceived returns whether a "stay" command has been received
func GetLeecherStayReceived(supervisorName string) bool {
	leecherMutex.RLock()
	defer leecherMutex.RUnlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		return info.StayReceived
	}
	return false
}

// NotifyAllLeechersStay sets StayReceived to true for all leechers (pause following)
func NotifyAllLeechersStay(leaderName string) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	for name, info := range leecherRegistry {
		if leaderName == "" || leaderName == "human_leader" || info.LeaderName == leaderName {
			info.StayReceived = true
			info.ComeReceived = false // Cancel any pending come command
			slog.Debug("Notified leecher of STAY command", slog.String("leecher", name))
		}
	}
}

// SetLeecherExitReceived sets whether an "exit game" command has been received
func SetLeecherExitReceived(supervisorName string, received bool) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		info.ExitReceived = received
	}
}

// GetLeecherExitReceived returns whether an "exit game" command has been received
func GetLeecherExitReceived(supervisorName string) bool {
	leecherMutex.RLock()
	defer leecherMutex.RUnlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		return info.ExitReceived
	}
	return false
}

// NotifyAllLeechersExit sets ExitReceived to true for all leechers
func NotifyAllLeechersExit(leaderName string) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	for name, info := range leecherRegistry {
		if leaderName == "" || leaderName == "human_leader" || info.LeaderName == leaderName {
			info.ExitReceived = true
			slog.Debug("Notified leecher of EXIT command", slog.String("leecher", name))
		}
	}
}

// GetConnectedLeecherCount returns the number of registered leechers
func GetConnectedLeecherCount() int {
	leecherMutex.RLock()
	defer leecherMutex.RUnlock()

	return len(leecherRegistry)
}

// GetLeecherStatuses returns a map of supervisor names to their current states
func GetLeecherStatuses() map[string]LeecherState {
	leecherMutex.RLock()
	defer leecherMutex.RUnlock()

	statuses := make(map[string]LeecherState)
	for name, info := range leecherRegistry {
		statuses[name] = info.State
	}
	return statuses
}

// CalculateLeecherJoinDelay calculates a random delay between min and max for leecher joining
// If both min and max are 0, returns 0 (no delay)
func CalculateLeecherJoinDelay(minDelay, maxDelay int) int {
	// Allow 0 delay if user explicitly sets both to 0
	if minDelay == 0 && maxDelay == 0 {
		return 0
	}

	// Otherwise use sensible minimums
	if minDelay < 500 {
		minDelay = 500
	}
	if maxDelay < minDelay {
		maxDelay = minDelay
	}

	spread := maxDelay - minDelay
	if spread <= 0 {
		return minDelay
	}

	return minDelay + rand.Intn(spread)
}

// NotifyAllLeechersCome sets ComeReceived to true for all leechers
// If leaderName is empty or "human_leader", notifies ALL leechers
// Otherwise only notifies leechers following the specified leader
func NotifyAllLeechersCome(leaderName string) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	for name, info := range leecherRegistry {
		// For human leader mode, notify all leechers regardless of their registered leader name
		if leaderName == "" || leaderName == "human_leader" || info.LeaderName == leaderName {
			info.ComeReceived = true
			// Log for debugging
			slog.Debug("Notified leecher of COME command", slog.String("leecher", name))
		}
	}
}

// ResetAllLeechersCome sets ComeReceived to false for all leechers
// If leaderName is empty or "human_leader", resets ALL leechers
// Otherwise only resets leechers following the specified leader
func ResetAllLeechersCome(leaderName string) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	for name, info := range leecherRegistry {
		// For human leader mode, reset all leechers regardless of their registered leader name
		if leaderName == "" || leaderName == "human_leader" || info.LeaderName == leaderName {
			info.ComeReceived = false
			slog.Debug("Reset leecher COME flag", slog.String("leecher", name))
		}
	}
}

// GetLeecherJoinRetries returns the number of join retries for a leecher
func GetLeecherJoinRetries(supervisorName string) int {
	leecherMutex.RLock()
	defer leecherMutex.RUnlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		return info.JoinRetries
	}
	return 0
}

// IncrementLeecherJoinRetries increments the join retry counter and returns true if max retries reached
func IncrementLeecherJoinRetries(supervisorName string) bool {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	const maxRetries = 5
	if info, ok := leecherRegistry[supervisorName]; ok {
		info.JoinRetries++
		if info.JoinRetries >= maxRetries {
			info.JoinRetries = 0 // Reset for next game
			return true
		}
	}
	return false
}

// ResetLeecherJoinRetries resets the join retry counter for a leecher
func ResetLeecherJoinRetries(supervisorName string) {
	leecherMutex.Lock()
	defer leecherMutex.Unlock()

	if info, ok := leecherRegistry[supervisorName]; ok {
		info.JoinRetries = 0
	}
}
