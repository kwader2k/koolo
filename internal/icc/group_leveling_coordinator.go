package icc

import (
	"errors"
	"fmt"
	"log/slog"
	"sync"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/koolo/internal/context"
)

const (
	RoleLeader     = "leader"     // Creates games, sets pace, makes key decisions
	RoleFollower   = "follower"   // Follows leader, assists in combat
	RoleTeleporter = "teleporter" // Teleports team (typically Sorceress)
	RoleBoBarb     = "bo_barb"    // Provides Battle Orders buff
	RoleRusher     = "rusher"     // Rushes ahead for quests/waypoints
	RoleAuto       = "auto"       // Auto-determine role based on character class
)

// GroupLevelingCoordinator handles all ICC coordination for group leveling
type GroupLevelingCoordinator struct {
	ctx         *context.Status
	iccManager  *context.ICCManager
	logger      *slog.Logger
	groupName   string
	myCharName  string
	currentRole string

	// Quest/Boss credit tracking (Horde-inspired)
	questCredits    map[string]map[string]bool // CharName → QuestID → Completed
	bossKillCredits map[string]map[string]bool // CharName → BossName → Killed
	creditMu        sync.RWMutex

	// Group membership tracking
	groupMembers   map[string]bool // CharName → true for all known group members
	groupMembersMu sync.RWMutex

	// Synchro tracking
	synchroStates map[string]map[string]bool // SynchroType → CharName → Ready
	synchroMu     sync.RWMutex

	// Leader tracking
	currentLeader  string
	leaderLastSeen time.Time
	leaderMu       sync.RWMutex

	// Position tracking
	lastPosition   data.Position
	lastArea       area.ID
	leaderPosition data.Position
	leaderArea     area.ID
	positionMu     sync.RWMutex

	// Current run tracking
	currentRun     string
	currentRunData map[string]interface{}
	runMu          sync.RWMutex
}

// NewGroupLevelingCoordinator creates a new coordinator instance
func NewGroupLevelingCoordinator(ctx *context.Status, iccManager *context.ICCManager) *GroupLevelingCoordinator {
	coord := &GroupLevelingCoordinator{
		ctx:             ctx,
		iccManager:      iccManager,
		logger:          ctx.Logger,
		groupName:       ctx.CharacterCfg.GroupLeveling.GroupName,
		myCharName:      ctx.CharacterCfg.CharacterName,
		currentRole:     ctx.CharacterCfg.GroupLeveling.Role,
		questCredits:    make(map[string]map[string]bool),
		bossKillCredits: make(map[string]map[string]bool),
		synchroStates:   make(map[string]map[string]bool),
		groupMembers:    make(map[string]bool),
		currentRunData:  make(map[string]interface{}),
	}

	// Add self to group members
	coord.groupMembers[coord.myCharName] = true

	// Initialize group leveling state in context
	if ctx.GroupLevelingState == nil {
		ctx.GroupLevelingState = &context.GroupLevelingState{
			GroupName:       coord.groupName,
			MyRole:          coord.currentRole,
			GroupMembers:    make(map[string]string),
			QuestCredits:    make(map[string]bool),
			BossKillCredits: make(map[string]bool),
		}
	}

	// Subscribe to group leveling events
	coord.setupEventSubscriptions()

	// Announce that this character has joined the group (for dynamic member discovery)
	coord.AnnounceGroupMembership()

	return coord
}

// setupEventSubscriptions registers handlers for all group leveling events
func (c *GroupLevelingCoordinator) setupEventSubscriptions() {
	// Quest completion events
	c.iccManager.Subscribe(context.EventTypeQuestComplete, func(evt context.ICCEvent) error {
		groupName, _ := evt.Data["group_name"].(string)
		if groupName != c.groupName {
			return nil
		}
		c.handleQuestCompleteEvent(evt)
		return nil
	})

	// Boss kill events
	c.iccManager.Subscribe(context.EventTypeBossKill, func(evt context.ICCEvent) error {
		groupName, _ := evt.Data["group_name"].(string)
		if groupName != c.groupName {
			return nil
		}
		c.handleBossKillEvent(evt)
		return nil
	})

	// Leader change events
	c.iccManager.Subscribe(context.EventTypeLeaderChange, func(evt context.ICCEvent) error {
		groupName, _ := evt.Data["group_name"].(string)
		if groupName != c.groupName {
			return nil
		}
		c.handleLeaderChangeEvent(evt)
		return nil
	})

	// Synchro ready events
	c.iccManager.Subscribe(context.EventTypeSynchroReady, func(evt context.ICCEvent) error {
		groupName, _ := evt.Data["group_name"].(string)
		if groupName != c.groupName {
			return nil
		}
		c.handleSynchroReadyEvent(evt)
		return nil
	})

	// Synchro state request events (state-based synchro)
	c.iccManager.Subscribe(context.EventTypeSynchroStateRequest, func(evt context.ICCEvent) error {
		groupName, _ := evt.Data["group_name"].(string)
		if groupName != c.groupName {
			return nil
		}
		c.handleSynchroStateRequestEvent(evt)
		return nil
	})

	// Position update events
	c.iccManager.Subscribe(context.EventTypePositionUpdate, func(evt context.ICCEvent) error {
		groupName, _ := evt.Data["group_name"].(string)
		if groupName != c.groupName {
			return nil
		}
		c.handlePositionUpdateEvent(evt)
		return nil
	})

	// Group member joined events (for dynamic member discovery)
	c.iccManager.Subscribe(context.EventTypeGroupMemberJoined, func(evt context.ICCEvent) error {
		groupName, _ := evt.Data["group_name"].(string)
		if groupName != c.groupName {
			return nil
		}
		c.handleGroupMemberJoinedEvent(evt)
		return nil
	})

	// Current run events
	c.iccManager.Subscribe(context.EventTypeCurrentRun, func(evt context.ICCEvent) error {
		groupName, _ := evt.Data["group_name"].(string)
		if groupName != c.groupName {
			return nil
		}
		c.onRunEvent(evt)
		return nil
	})
}

// --- Quest/Boss Credit Tracking (Horde Approach) ---

// AnnounceQuestComplete broadcasts quest completion status to group
func (c *GroupLevelingCoordinator) AnnounceQuestComplete(questID string, completed bool) {
	c.creditMu.Lock()
	if c.questCredits[c.myCharName] == nil {
		c.questCredits[c.myCharName] = make(map[string]bool)
	}
	c.questCredits[c.myCharName][questID] = completed
	c.creditMu.Unlock()

	// Update context state
	c.ctx.GroupLevelingState.QuestCredits[questID] = completed

	// Broadcast to group via ICC
	c.iccManager.PublishEvent(
		context.EventTypeQuestComplete,
		c.myCharName,
		map[string]interface{}{
			"group_name": c.groupName,
			"quest_id":   questID,
			"completed":  completed,
		},
	)

	c.logger.Info("Announced quest completion",
		"quest", questID,
		"completed", completed,
	)
}

// AnnounceActStart broadcasts act start to group
func (c *GroupLevelingCoordinator) AnnounceActStart(actNumber int) {
	c.logger.Info("Announcing act start", "act", actNumber)
	c.iccManager.PublishEvent(
		context.EventTypeActStart,
		c.myCharName,
		map[string]interface{}{
			"group_name": c.groupName,
			"act_number": actNumber,
		},
	)
}

// CheckQuestCompletion checks if all group members got quest credit
// Returns (newLeader, needsChange) - if needsChange is true, newLeader should create new game
func (c *GroupLevelingCoordinator) CheckQuestCompletion(questID string, timeout time.Duration) (string, bool, error) {
	c.logger.Info("Checking quest completion for all group members",
		"quest", questID,
		"timeout", timeout,
	)

	// Wait for all members to announce
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		c.creditMu.RLock()
		allReported := c.allMembersReportedQuest(questID)
		c.creditMu.RUnlock()

		if allReported {
			break
		}
		time.Sleep(500 * time.Millisecond)
	}

	// Check who failed
	c.creditMu.RLock()
	defer c.creditMu.RUnlock()

	var failedMembers []string
	for charName := range c.getGroupMembers() {
		questMap, exists := c.questCredits[charName]
		if !exists || !questMap[questID] {
			failedMembers = append(failedMembers, charName)
		}
	}

	if len(failedMembers) > 0 {
		// First failed member becomes new leader
		newLeader := failedMembers[0]
		c.logger.Warn("Quest credit check failed - promoting new leader",
			"quest", questID,
			"failed_members", failedMembers,
			"new_leader", newLeader,
		)
		return newLeader, true, nil
	}

	c.logger.Info("All group members got quest credit", "quest", questID)
	return c.currentLeader, false, nil
}

// AnnounceGroupMembership broadcasts that this character is part of the group
func (c *GroupLevelingCoordinator) AnnounceGroupMembership() {
	c.iccManager.PublishEvent(
		context.EventTypeGroupMemberJoined,
		c.myCharName,
		map[string]interface{}{
			"group_name": c.groupName,
			"role":       c.currentRole,
		},
	)

	c.logger.Debug("Announced group membership", "group_name", c.groupName, "role", c.currentRole)
}

// AnnounceBossKill broadcasts boss kill status to group
func (c *GroupLevelingCoordinator) AnnounceBossKill(bossName string, killed bool) {
	c.creditMu.Lock()
	if c.bossKillCredits[c.myCharName] == nil {
		c.bossKillCredits[c.myCharName] = make(map[string]bool)
	}
	c.bossKillCredits[c.myCharName][bossName] = killed
	c.creditMu.Unlock()

	// Update context state
	c.ctx.GroupLevelingState.BossKillCredits[bossName] = killed

	// Broadcast to group via ICC
	c.iccManager.PublishEvent(
		context.EventTypeBossKill,
		c.myCharName,
		map[string]interface{}{
			"group_name": c.groupName,
			"boss_name":  bossName,
			"killed":     killed,
		},
	)

	c.logger.Info("Announced boss kill",
		"boss", bossName,
		"killed", killed,
	)
}

// CheckBossKillCredit checks if all group members got boss kill credit
func (c *GroupLevelingCoordinator) CheckBossKillCredit(bossName string, timeout time.Duration) (string, bool, error) {
	c.logger.Info("Checking boss kill credit for all group members",
		"boss", bossName,
		"timeout", timeout,
	)

	// Wait for all members to announce
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		c.creditMu.RLock()
		allReported := c.allMembersReportedBoss(bossName)
		c.creditMu.RUnlock()

		if allReported {
			break
		}
		time.Sleep(500 * time.Millisecond)
	}

	// Check who failed
	c.creditMu.RLock()
	defer c.creditMu.RUnlock()

	var failedMembers []string
	for charName := range c.getGroupMembers() {
		bossMap, exists := c.bossKillCredits[charName]
		if !exists || !bossMap[bossName] {
			failedMembers = append(failedMembers, charName)
		}
	}

	if len(failedMembers) > 0 {
		// First failed member becomes new leader
		newLeader := failedMembers[0]
		c.logger.Warn("Boss kill credit check failed - promoting new leader",
			"boss", bossName,
			"failed_members", failedMembers,
			"new_leader", newLeader,
		)
		return newLeader, true, nil
	}

	c.logger.Info("All group members got boss kill credit", "boss", bossName)
	return c.currentLeader, false, nil
}

// ResetCreditsForNewGame clears quest/boss tracking for fresh game
func (c *GroupLevelingCoordinator) ResetCreditsForNewGame() {
	c.creditMu.Lock()
	c.questCredits = make(map[string]map[string]bool)
	c.bossKillCredits = make(map[string]map[string]bool)
	c.creditMu.Unlock()

	c.logger.Info("Reset quest/boss credits for new game")
}

// --- Leader Election ---

// IsLeader returns true if this bot is currently the leader
func (c *GroupLevelingCoordinator) IsLeader() bool {
	c.leaderMu.RLock()
	defer c.leaderMu.RUnlock()

	if c.currentRole == RoleLeader {
		return true
	}

	if c.currentRole == RoleAuto {
		return c.currentLeader == c.myCharName
	}

	return false
}

// GetMyRole returns current role
func (c *GroupLevelingCoordinator) GetMyRole() string {
	c.leaderMu.RLock()
	defer c.leaderMu.RUnlock()
	return c.currentRole
}

// PromoteNewLeader broadcasts leader change event
func (c *GroupLevelingCoordinator) PromoteNewLeader(newLeader string, reason string) {
	c.leaderMu.Lock()
	oldLeader := c.currentLeader
	c.currentLeader = newLeader
	c.leaderMu.Unlock()

	// Update context state
	c.ctx.GroupLevelingState.CurrentLeader = newLeader
	c.ctx.GroupLevelingState.IsLeader = (newLeader == c.myCharName)

	// Broadcast leader change via ICC
	c.iccManager.PublishEvent(
		context.EventTypeLeaderChange,
		c.myCharName,
		map[string]interface{}{
			"group_name": c.groupName,
			"new_leader": newLeader,
			"old_leader": oldLeader,
			"reason":     reason,
		},
	)

	c.logger.Info("Leader changed",
		"old_leader", oldLeader,
		"new_leader", newLeader,
		"reason", reason,
	)
}

// --- Synchronization ---

// WaitSynchro waits for all group members to be ready
func (c *GroupLevelingCoordinator) WaitSynchro(synchroType string, timeout time.Duration) error {
	members := c.getGroupMembers()
	c.logger.Info("Waiting for group synchro",
		"synchro_type", synchroType,
		"timeout", timeout,
		"expected_members", len(members),
	)

	// Request current synchro state from all members (state-based approach)
	c.RequestSynchroState(synchroType)

	// Announce ready
	c.AnnounceReady(synchroType)

	// Wait for all members
	deadline := time.Now().Add(timeout)
	lastLogTime := time.Now()
	for time.Now().Before(deadline) {
		c.synchroMu.RLock()
		allReady := c.allMembersReady(synchroType)
		readyCount := len(c.synchroStates[synchroType])
		c.synchroMu.RUnlock()

		if allReady {
			c.logger.Info("All group members ready", "synchro_type", synchroType, "count", len(members))
			return nil
		}

		// Log progress every 10 seconds
		if time.Since(lastLogTime) > 10*time.Second {
			c.logger.Debug("Waiting for group members",
				"synchro_type", synchroType,
				"ready", readyCount,
				"expected", len(members),
			)
			lastLogTime = time.Now()
		}

		time.Sleep(500 * time.Millisecond)
	}

	return fmt.Errorf("synchro timeout: %s (expected %d members, only %d ready)", synchroType, len(members), len(c.synchroStates[synchroType]))
}

// AnnounceReady broadcasts ready status for synchronization point
func (c *GroupLevelingCoordinator) AnnounceReady(synchroType string) {
	c.synchroMu.Lock()
	if c.synchroStates[synchroType] == nil {
		c.synchroStates[synchroType] = make(map[string]bool)
	}
	c.synchroStates[synchroType][c.myCharName] = true
	c.synchroMu.Unlock()

	// Broadcast via ICC
	c.iccManager.PublishEvent(
		context.EventTypeSynchroReady,
		c.myCharName,
		map[string]interface{}{
			"group_name":   c.groupName,
			"synchro_type": synchroType,
			"ready":        true,
		},
	)

	c.logger.Debug("Announced ready", "synchro_type", synchroType)
}

// SignalReady is an alias for AnnounceReady for compatibility
func (c *GroupLevelingCoordinator) SignalReady(synchroType string, ready bool) {
	if ready {
		c.AnnounceReady(synchroType)
	}
}

// RequestSynchroState requests current synchro state from all group members
// This ensures we get current state even if we missed ephemeral events
func (c *GroupLevelingCoordinator) RequestSynchroState(synchroType string) {
	c.iccManager.PublishEvent(
		context.EventTypeSynchroStateRequest,
		c.myCharName,
		map[string]interface{}{
			"group_name":   c.groupName,
			"synchro_type": synchroType,
		},
	)

	c.logger.Debug("Requested synchro state from group", "synchro_type", synchroType)
}

// WaitForGameStart waits for all group members to join game
func (c *GroupLevelingCoordinator) WaitForGameStart(timeout time.Duration) error {
	return c.WaitSynchro("game_start", timeout)
}

// WaitForActStart waits for leader to start an act
func (c *GroupLevelingCoordinator) WaitForActStart(actNumber int, timeout time.Duration) error {
	synchroType := fmt.Sprintf("act%d_start", actNumber)
	return c.WaitSynchro(synchroType, timeout)
}

// BroadcastActStart announces leader is starting an act
func (c *GroupLevelingCoordinator) BroadcastActStart(actNumber int) {
	c.iccManager.PublishEvent(
		context.EventTypeActStart,
		c.myCharName,
		map[string]interface{}{
			"group_name": c.groupName,
			"act_number": actNumber,
		},
	)

	c.logger.Info("Broadcast act start", "act", actNumber)
}

// --- Position Tracking ---

// BroadcastPosition sends current position to group (for leader tracking)
func (c *GroupLevelingCoordinator) BroadcastPosition() {
	if !c.IsLeader() {
		return // Only leader broadcasts position
	}

	c.positionMu.Lock()
	c.lastPosition = c.ctx.Data.PlayerUnit.Position
	c.lastArea = c.ctx.Data.PlayerUnit.Area
	c.positionMu.Unlock()

	c.iccManager.PublishEvent(
		context.EventTypePositionUpdate,
		c.myCharName,
		map[string]interface{}{
			"group_name": c.groupName,
			"area_id":    int(c.ctx.Data.PlayerUnit.Area),
			"position": map[string]interface{}{
				"x": c.ctx.Data.PlayerUnit.Position.X,
				"y": c.ctx.Data.PlayerUnit.Position.Y,
			},
		},
	)
}

// WaitForLeaderAtArea waits for leader to reach target area
func (c *GroupLevelingCoordinator) WaitForLeaderAtArea(targetArea area.ID, timeout time.Duration) error {
	c.logger.Info("Waiting for leader at area",
		"area", targetArea,
		"timeout", timeout,
	)

	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		c.positionMu.RLock()
		leaderInArea := c.leaderArea == targetArea
		c.positionMu.RUnlock()

		if leaderInArea {
			c.logger.Info("Leader reached target area", "area", targetArea)
			return nil
		}

		time.Sleep(1 * time.Second)
	}

	return errors.New("timeout waiting for leader at area")
}

// --- Event Handlers ---

func (c *GroupLevelingCoordinator) handleQuestCompleteEvent(evt context.ICCEvent) {
	charName := evt.Source
	questID, _ := evt.Data["quest_id"].(string)
	completed, _ := evt.Data["completed"].(bool)

	c.creditMu.Lock()
	if c.questCredits[charName] == nil {
		c.questCredits[charName] = make(map[string]bool)
	}
	c.questCredits[charName][questID] = completed
	c.creditMu.Unlock()

	c.logger.Debug("Received quest completion",
		"char", charName,
		"quest", questID,
		"completed", completed,
	)
}

func (c *GroupLevelingCoordinator) handleBossKillEvent(evt context.ICCEvent) {
	charName := evt.Source
	bossName, _ := evt.Data["boss_name"].(string)
	killed, _ := evt.Data["killed"].(bool)

	c.creditMu.Lock()
	if c.bossKillCredits[charName] == nil {
		c.bossKillCredits[charName] = make(map[string]bool)
	}
	c.bossKillCredits[charName][bossName] = killed
	c.creditMu.Unlock()

	c.logger.Debug("Received boss kill",
		"char", charName,
		"boss", bossName,
		"killed", killed,
	)
}

func (c *GroupLevelingCoordinator) handleLeaderChangeEvent(evt context.ICCEvent) {
	newLeader, _ := evt.Data["new_leader"].(string)
	oldLeader, _ := evt.Data["old_leader"].(string)
	reason, _ := evt.Data["reason"].(string)

	c.leaderMu.Lock()
	c.currentLeader = newLeader
	c.leaderMu.Unlock()

	c.ctx.GroupLevelingState.CurrentLeader = newLeader
	c.ctx.GroupLevelingState.IsLeader = (newLeader == c.myCharName)

	c.logger.Info("Received leader change",
		"new_leader", newLeader,
		"old_leader", oldLeader,
		"reason", reason,
	)
}

func (c *GroupLevelingCoordinator) handleSynchroReadyEvent(evt context.ICCEvent) {
	charName := evt.Source
	synchroType, _ := evt.Data["synchro_type"].(string)
	ready, _ := evt.Data["ready"].(bool)

	c.synchroMu.Lock()
	if c.synchroStates[synchroType] == nil {
		c.synchroStates[synchroType] = make(map[string]bool)
	}
	c.synchroStates[synchroType][charName] = ready
	c.synchroMu.Unlock()

	c.logger.Debug("Received synchro ready",
		"char", charName,
		"synchro_type", synchroType,
		"ready", ready,
	)
}

func (c *GroupLevelingCoordinator) handleSynchroStateRequestEvent(evt context.ICCEvent) {
	requestingSynchroType, _ := evt.Data["synchro_type"].(string)
	requester := evt.Source

	// Don't respond to our own requests
	if requester == c.myCharName {
		return
	}

	c.logger.Debug("Received synchro state request",
		"from", requester,
		"synchro_type", requestingSynchroType,
	)

	// Check if we're ready for this synchro point
	c.synchroMu.RLock()
	isReady := false
	if stateMap, exists := c.synchroStates[requestingSynchroType]; exists {
		isReady = stateMap[c.myCharName]
	}
	c.synchroMu.RUnlock()

	// If we're ready, announce it (re-announcement for state query)
	if isReady {
		c.logger.Debug("Responding to synchro state request with ready status",
			"synchro_type", requestingSynchroType,
			"requester", requester,
		)
		c.iccManager.PublishEvent(
			context.EventTypeSynchroReady,
			c.myCharName,
			map[string]interface{}{
				"group_name":   c.groupName,
				"synchro_type": requestingSynchroType,
				"ready":        true,
			},
		)
	}
}

func (c *GroupLevelingCoordinator) handlePositionUpdateEvent(evt context.ICCEvent) {
	areaID, _ := evt.Data["area_id"].(float64)
	posData, _ := evt.Data["position"].(map[string]interface{})
	x, _ := posData["x"].(int)
	y, _ := posData["y"].(int)

	c.positionMu.Lock()
	c.leaderPosition = data.Position{X: x, Y: y}
	c.leaderArea = area.ID(int(areaID))
	c.positionMu.Unlock()
}

func (c *GroupLevelingCoordinator) handleGroupMemberJoinedEvent(evt context.ICCEvent) {
	charName := evt.Source
	role, _ := evt.Data["role"].(string)

	c.groupMembersMu.Lock()
	c.groupMembers[charName] = true
	totalMembers := len(c.groupMembers)
	c.groupMembersMu.Unlock()

	c.logger.Info("Group member joined",
		"char", charName,
		"role", role,
		"total_members", totalMembers,
	)

	// Re-announce all current synchro states so the new member can catch up
	// This handles late joiners who missed earlier synchro_ready announcements
	c.synchroMu.RLock()
	currentStates := make(map[string]bool)
	for synchroType, stateMap := range c.synchroStates {
		if ready, exists := stateMap[c.myCharName]; exists && ready {
			currentStates[synchroType] = true
		}
	}
	c.synchroMu.RUnlock()

	// Re-announce each active synchro state
	for synchroType := range currentStates {
		c.logger.Debug("Re-announcing synchro state for new member",
			"synchro_type", synchroType,
			"new_member", charName,
		)
		c.iccManager.PublishEvent(
			context.EventTypeSynchroReady,
			c.myCharName,
			map[string]interface{}{
				"group_name":   c.groupName,
				"synchro_type": synchroType,
				"ready":        true,
			},
		)
	}
}

// --- Current Run Tracking ---
func (c *GroupLevelingCoordinator) BroadcastRun(runName string, runData map[string]interface{}) {
	if !c.IsLeader() {
		return
	}

	eventData := map[string]interface{}{
		"group_name": c.groupName,
		"run_name":   runName,
	}

	for k, v := range runData {
		eventData[k] = v
	}

	c.iccManager.PublishEvent(
		context.EventTypeCurrentRun,
		c.myCharName,
		eventData,
	)

	c.logger.Info("Broadcast run", "run_name", runName)
}

func (c *GroupLevelingCoordinator) onRunEvent(evt context.ICCEvent) {
	runName, _ := evt.Data["run_name"].(string)
	if runName == "" {
		return
	}

	if c.IsLeader() {
		return
	}

	if c.ctx.GroupLevelingState != nil && c.ctx.GroupLevelingState.ExecuteRun != nil {
		if err := c.ctx.GroupLevelingState.ExecuteRun(runName, evt.Data); err != nil {
			c.logger.Error("Failed to execute run", "run_name", runName, "error", err)
		}
	}
}

// --- Helper Methods ---

func (c *GroupLevelingCoordinator) getGroupMembers() map[string]bool {
	// Get configured members from config
	configuredMembers := c.ctx.CharacterCfg.GroupLeveling.Members

	if len(configuredMembers) > 0 {
		// Use configured member list (explicit/robust mode)
		members := make(map[string]bool)
		for _, memberName := range configuredMembers {
			members[memberName] = true
		}
		c.logger.Debug("Using configured group members", "count", len(members), "members", configuredMembers)
		return members
	}

	// Use tracked group members (dynamic discovery via ICC announcements)
	c.groupMembersMu.RLock()
	defer c.groupMembersMu.RUnlock()

	members := make(map[string]bool)
	for charName := range c.groupMembers {
		members[charName] = true
	}

	c.logger.Debug("Using tracked group members", "count", len(members))
	return members
}

func (c *GroupLevelingCoordinator) allMembersReportedQuest(questID string) bool {
	for charName := range c.getGroupMembers() {
		questMap, exists := c.questCredits[charName]
		if !exists {
			return false
		}
		if _, reported := questMap[questID]; !reported {
			return false
		}
	}
	return true
}

func (c *GroupLevelingCoordinator) allMembersReportedBoss(bossName string) bool {
	for charName := range c.getGroupMembers() {
		bossMap, exists := c.bossKillCredits[charName]
		if !exists {
			return false
		}
		if _, reported := bossMap[bossName]; !reported {
			return false
		}
	}
	return true
}

func (c *GroupLevelingCoordinator) allMembersReady(synchroType string) bool {
	stateMap, exists := c.synchroStates[synchroType]
	if !exists {
		return false
	}

	for charName := range c.getGroupMembers() {
		if !stateMap[charName] {
			return false
		}
	}
	return true
}

// RoleCapabilities defines what a role can do in group leveling
type RoleCapabilities struct {
	CanTeleport   bool // Can teleport team forward (Sorceress)
	CanProvideBO  bool // Can provide Battle Orders buff (Barb)
	CanRush       bool // Can rush ahead for objectives (high level/geared)
	ShouldLead    bool // Should create games and lead the group
	ShouldFollow  bool // Should follow leader's position
	PriorityLevel int  // Higher = more important for leadership election (1-10)
}

// GetRoleCapabilities returns capabilities for the current role
func (c *GroupLevelingCoordinator) GetRoleCapabilities() RoleCapabilities {
	switch c.currentRole {
	case RoleLeader:
		return RoleCapabilities{
			CanTeleport:   false,
			CanProvideBO:  false,
			CanRush:       false,
			ShouldLead:    true,
			ShouldFollow:  false,
			PriorityLevel: 10, // Highest priority
		}
	case RoleFollower:
		return RoleCapabilities{
			CanTeleport:   false,
			CanProvideBO:  false,
			CanRush:       false,
			ShouldLead:    false,
			ShouldFollow:  true,
			PriorityLevel: 1, // Lowest priority
		}
	case RoleTeleporter:
		return RoleCapabilities{
			CanTeleport:   true,
			CanProvideBO:  false,
			CanRush:       true,
			ShouldLead:    false,
			ShouldFollow:  false, // Teleporter moves independently
			PriorityLevel: 7,     // High priority for leadership
		}
	case RoleBoBarb:
		return RoleCapabilities{
			CanTeleport:   false,
			CanProvideBO:  true,
			CanRush:       false,
			ShouldLead:    false,
			ShouldFollow:  true,
			PriorityLevel: 5, // Medium priority
		}
	case RoleRusher:
		return RoleCapabilities{
			CanTeleport:   false, // Unless also a teleporter class
			CanProvideBO:  false,
			CanRush:       true,
			ShouldLead:    false,
			ShouldFollow:  false,
			PriorityLevel: 8, // High priority
		}
	case RoleAuto:
		// Auto-determine based on character class
		return c.autoDetectCapabilities()
	default:
		// Default to basic follower
		return RoleCapabilities{
			CanTeleport:   false,
			CanProvideBO:  false,
			CanRush:       false,
			ShouldLead:    false,
			ShouldFollow:  true,
			PriorityLevel: 1,
		}
	}
}

// autoDetectCapabilities determines capabilities based on character class and config
func (c *GroupLevelingCoordinator) autoDetectCapabilities() RoleCapabilities {
	caps := RoleCapabilities{
		CanTeleport:   false,
		CanProvideBO:  false,
		CanRush:       false,
		ShouldLead:    false,
		ShouldFollow:  true,
		PriorityLevel: 3, // Default medium-low priority
	}

	// Check character class from config
	class := c.ctx.CharacterCfg.Character.Class

	// Sorceresses can teleport
	if class == "sorceress" || class == "blizzard_sorceress" || class == "lightning_sorceress" {
		caps.CanTeleport = true
		caps.PriorityLevel = 7
		caps.ShouldFollow = false
	}

	// Barbarians can provide BO
	if class == "barbarian" || class == "berserker" || class == "warcry_barb" {
		caps.CanProvideBO = true
		caps.PriorityLevel = 5
	}

	// Paladins are good all-rounders
	if class == "paladin" || class == "hammerdin" || class == "foh" || class == "smiter" {
		caps.PriorityLevel = 6
	}

	// Necromancers with summons
	if class == "necromancer" || class == "summoner" {
		caps.PriorityLevel = 4
	}

	// Check if character can use teleport from config
	if c.ctx.CharacterCfg.Character.UseTeleport {
		caps.CanTeleport = true
		if caps.PriorityLevel < 6 {
			caps.PriorityLevel = 6
		}
	}

	return caps
}

// CanTeleport returns true if this character can teleport the team
func (c *GroupLevelingCoordinator) CanTeleport() bool {
	return c.GetRoleCapabilities().CanTeleport
}

// CanProvideBO returns true if this character can provide Battle Orders
func (c *GroupLevelingCoordinator) CanProvideBO() bool {
	return c.GetRoleCapabilities().CanProvideBO
}

// CanRush returns true if this character can rush ahead for objectives
func (c *GroupLevelingCoordinator) CanRush() bool {
	return c.GetRoleCapabilities().CanRush
}

// ShouldLead returns true if this character should create games and lead
func (c *GroupLevelingCoordinator) ShouldLead() bool {
	return c.GetRoleCapabilities().ShouldLead
}

// ShouldFollow returns true if this character should follow leader's position
func (c *GroupLevelingCoordinator) ShouldFollow() bool {
	return c.GetRoleCapabilities().ShouldFollow
}

// GetPriorityLevel returns leadership election priority (1-10, higher = more important)
func (c *GroupLevelingCoordinator) GetPriorityLevel() int {
	return c.GetRoleCapabilities().PriorityLevel
}
