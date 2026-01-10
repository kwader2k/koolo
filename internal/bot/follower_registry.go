package bot

import (
	"fmt"
	"math/rand"
	"sync"
)

// FollowerState represents the current state of a follower
type FollowerState string

const (
	FollowerStateIdle     FollowerState = "idle"      // Not active
	FollowerStateInTown   FollowerState = "in_town"   // Idling in town
	FollowerStateJoining  FollowerState = "joining"   // Joining game
	FollowerStateExiting  FollowerState = "exiting"   // Exiting game
)

// FollowerInfo contains information about a follower supervisor
// Followers are simpler than leechers - they just join games and idle in town for XP boost
type FollowerInfo struct {
	LeaderSupervisor  string        // The supervisor name of the leader coordinator
	LeaderName        string        // The character name of the leader
	GamePattern       string        // Game name pattern to look for
	GamePassword      string        // Password for games
	JoinDelayMin      int           // Minimum join delay in ms
	JoinDelayMax      int           // Maximum join delay in ms
	GameCounter       int           // Current game number for auto-incrementing
	PollInterval      int           // How often to check if leader left (ms)
	UseLegacyGraphics bool          // Use legacy graphics mode
	State             FollowerState // Current follower state
	ExitReceived      bool          // Whether an "exit game" command has been received
	JoinRetries       int           // Number of failed join attempts for current game
}

var (
	followerRegistry = make(map[string]*FollowerInfo)
	followerMutex    sync.RWMutex
)

// RegisterFollower registers a supervisor as a follower
func RegisterFollower(supervisorName, leaderSupervisor, leaderName, gamePattern, gamePassword string,
	joinDelayMin, joinDelayMax, pollInterval int, useLegacyGraphics bool) {
	followerMutex.Lock()
	defer followerMutex.Unlock()

	followerRegistry[supervisorName] = &FollowerInfo{
		LeaderSupervisor:  leaderSupervisor,
		LeaderName:        leaderName,
		GamePattern:       gamePattern,
		GamePassword:      gamePassword,
		JoinDelayMin:      joinDelayMin,
		JoinDelayMax:      joinDelayMax,
		GameCounter:       1, // Start at 1
		PollInterval:      pollInterval,
		UseLegacyGraphics: useLegacyGraphics,
		State:             FollowerStateIdle,
		ExitReceived:      false,
		JoinRetries:       0,
	}
}

// UnregisterFollower removes a supervisor from the follower registry
func UnregisterFollower(supervisorName string) {
	followerMutex.Lock()
	defer followerMutex.Unlock()

	delete(followerRegistry, supervisorName)
}

// GetFollowerInfo returns follower info for a supervisor, or nil if not a follower
func GetFollowerInfo(supervisorName string) *FollowerInfo {
	followerMutex.RLock()
	defer followerMutex.RUnlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		// Return a copy to prevent data races
		infoCopy := *info
		return &infoCopy
	}
	return nil
}

// IsFollower checks if a supervisor is registered as a follower
func IsFollower(supervisorName string) bool {
	followerMutex.RLock()
	defer followerMutex.RUnlock()

	_, ok := followerRegistry[supervisorName]
	return ok
}

// GetFollowerGameName returns the current game name for a follower (pattern + counter)
func GetFollowerGameName(supervisorName string) string {
	followerMutex.RLock()
	defer followerMutex.RUnlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		return fmt.Sprintf("%s%d", info.GamePattern, info.GameCounter)
	}
	return ""
}

// IncrementFollowerGameCounter increments the game counter for a follower
func IncrementFollowerGameCounter(supervisorName string) {
	followerMutex.Lock()
	defer followerMutex.Unlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		info.GameCounter++
	}
}

// SetFollowerState updates the state of a follower
func SetFollowerState(supervisorName string, state FollowerState) {
	followerMutex.Lock()
	defer followerMutex.Unlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		info.State = state
	}
}

// GetFollowerState returns the current state of a follower
func GetFollowerState(supervisorName string) FollowerState {
	followerMutex.RLock()
	defer followerMutex.RUnlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		return info.State
	}
	return FollowerStateIdle
}

// SetFollowerExitReceived sets whether an "exit game" command has been received
func SetFollowerExitReceived(supervisorName string, received bool) {
	followerMutex.Lock()
	defer followerMutex.Unlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		info.ExitReceived = received
	}
}

// GetFollowerExitReceived returns whether an "exit game" command has been received
func GetFollowerExitReceived(supervisorName string) bool {
	followerMutex.RLock()
	defer followerMutex.RUnlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		return info.ExitReceived
	}
	return false
}

// NotifyAllFollowersExit sets ExitReceived to true for all followers
func NotifyAllFollowersExit(leaderName string) {
	followerMutex.Lock()
	defer followerMutex.Unlock()

	for _, info := range followerRegistry {
		if leaderName == "" || leaderName == "human_leader" || info.LeaderName == leaderName {
			info.ExitReceived = true
		}
	}
}

// GetConnectedFollowerCount returns the number of registered followers
func GetConnectedFollowerCount() int {
	followerMutex.RLock()
	defer followerMutex.RUnlock()

	return len(followerRegistry)
}

// GetFollowerStatuses returns a map of supervisor names to their current states
func GetFollowerStatuses() map[string]FollowerState {
	followerMutex.RLock()
	defer followerMutex.RUnlock()

	statuses := make(map[string]FollowerState)
	for name, info := range followerRegistry {
		statuses[name] = info.State
	}
	return statuses
}

// GetCurrentGameName returns the current game name for a follower (pattern + counter)
// This is a package-level function that checks both follower and leecher registries
func GetCurrentGameName(supervisorName string) string {
	// First check follower registry
	followerMutex.RLock()
	if info, ok := followerRegistry[supervisorName]; ok {
		followerMutex.RUnlock()
		return fmt.Sprintf("%s%d", info.GamePattern, info.GameCounter)
	}
	followerMutex.RUnlock()

	// Then check leecher registry
	return GetLeecherGameName(supervisorName)
}

// IncrementGameCounter increments the game counter for a follower or leecher
func IncrementGameCounter(supervisorName string) {
	// First check follower registry
	followerMutex.Lock()
	if info, ok := followerRegistry[supervisorName]; ok {
		info.GameCounter++
		followerMutex.Unlock()
		return
	}
	followerMutex.Unlock()

	// Then check leecher registry
	IncrementLeecherGameCounter(supervisorName)
}

// GetJoinRetries returns the number of join retries for a follower
func GetJoinRetries(supervisorName string) int {
	followerMutex.RLock()
	defer followerMutex.RUnlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		return info.JoinRetries
	}
	return 0
}

// IncrementJoinRetries increments the join retry counter and returns true if max retries reached
func IncrementJoinRetries(supervisorName string) bool {
	followerMutex.Lock()
	defer followerMutex.Unlock()

	const maxRetries = 5
	if info, ok := followerRegistry[supervisorName]; ok {
		info.JoinRetries++
		if info.JoinRetries >= maxRetries {
			info.JoinRetries = 0 // Reset for next game
			return true
		}
	}
	return false
}

// ResetJoinRetries resets the join retry counter for a follower
func ResetJoinRetries(supervisorName string) {
	followerMutex.Lock()
	defer followerMutex.Unlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		info.JoinRetries = 0
	}
}

// CalculateRandomDelay calculates a random delay between min and max
// If both min and max are 0, returns 0 (no delay)
func CalculateRandomDelay(minDelay, maxDelay int) int {
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

// CalculateExponentialBackoff returns a delay with exponential backoff based on retry count
// Base delay is multiplied by 2^retryCount, capped at maxDelay
// Returns milliseconds
func CalculateExponentialBackoff(baseDelayMs, maxDelayMs, retryCount int) int {
	if retryCount <= 0 {
		return baseDelayMs
	}

	// Calculate exponential delay: base * 2^retryCount
	delay := baseDelayMs
	for i := 0; i < retryCount && delay < maxDelayMs; i++ {
		delay *= 2
	}

	// Add some jitter (±10%) to prevent thundering herd
	jitter := delay / 10
	if jitter > 0 {
		delay += rand.Intn(jitter*2) - jitter
	}

	if delay > maxDelayMs {
		delay = maxDelayMs
	}
	return delay
}

