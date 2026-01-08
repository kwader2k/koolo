package bot

import (
	"fmt"
	"sync"
)

// FollowerInfo contains information about a follower supervisor
type FollowerInfo struct {
	LeaderSupervisor string // The supervisor name of the leader coordinator
	LeaderName       string // The character name of the human leader to follow
	GamePattern      string // Game name pattern to look for
	GamePassword     string // Password for games
	JoinDelayMin     int    // Minimum join delay in ms
	JoinDelayMax     int    // Maximum join delay in ms
	GameCounter      int    // Current game number for auto-incrementing
}

var (
	followerRegistry = make(map[string]*FollowerInfo)
	followerMutex    sync.RWMutex
)

// RegisterFollower registers a supervisor as a follower
func RegisterFollower(supervisorName, leaderSupervisor, leaderName, gamePattern, gamePassword string, joinDelayMin, joinDelayMax int) {
	followerMutex.Lock()
	defer followerMutex.Unlock()

	followerRegistry[supervisorName] = &FollowerInfo{
		LeaderSupervisor: leaderSupervisor,
		LeaderName:       leaderName,
		GamePattern:      gamePattern,
		GamePassword:     gamePassword,
		JoinDelayMin:     joinDelayMin,
		JoinDelayMax:     joinDelayMax,
		GameCounter:      1, // Start at 1
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

// GetCurrentGameName returns the current game name for a follower (pattern + counter)
func GetCurrentGameName(supervisorName string) string {
	followerMutex.RLock()
	defer followerMutex.RUnlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		return fmt.Sprintf("%s%d", info.GamePattern, info.GameCounter)
	}
	return ""
}

// IncrementGameCounter increments the game counter for a follower
func IncrementGameCounter(supervisorName string) {
	followerMutex.Lock()
	defer followerMutex.Unlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		info.GameCounter++
	}
}

// ResetGameCounter resets the game counter for a follower to 1
func ResetGameCounter(supervisorName string) {
	followerMutex.Lock()
	defer followerMutex.Unlock()

	if info, ok := followerRegistry[supervisorName]; ok {
		info.GameCounter = 1
	}
}

// FormatGameName formats a game name from pattern and counter
func FormatGameName(pattern string, counter int) string {
	return fmt.Sprintf("%s%d", pattern, counter)
}

