package bot

import (
	"context"
	"log/slog"
	"math/rand"
	"sync"
	"time"

	"github.com/hectorgimenez/koolo/internal/bot/messagebus"
	"github.com/hectorgimenez/koolo/internal/config"
)

// FollowerHandler handles follower-related events from the message bus
// It manages the state of a supervisor that is acting as a follower
type FollowerHandler struct {
	supervisorName string
	log            *slog.Logger
	cfg            *config.CharacterCfg
	bus            *messagebus.Bus

	// Follower state
	mu             sync.Mutex
	isFollowing    bool
	leaderName     string
	gamePattern    string
	gamePassword   string
	joinDelayMin   int
	joinDelayMax   int
	currentGame    string
	gameCounter    int
	isScout        bool
	scoutClaimed   bool

	// Channels for coordination
	gameAvailableCh chan *messagebus.GameAvailableMessage
	leaderLeftCh    chan struct{}
}

// NewFollowerHandler creates a new follower handler
func NewFollowerHandler(supervisorName string, log *slog.Logger, cfg *config.CharacterCfg) *FollowerHandler {
	return &FollowerHandler{
		supervisorName:  supervisorName,
		log:             log,
		cfg:             cfg,
		bus:             messagebus.GetBus(),
		gameAvailableCh: make(chan *messagebus.GameAvailableMessage, 1),
		leaderLeftCh:    make(chan struct{}, 1),
	}
}

// Handle processes follower-related events from the message bus
func (h *FollowerHandler) Handle(ctx context.Context, e interface{}) error {
	// Handle message bus messages
	switch msg := e.(type) {
	case messagebus.FollowerStartMessage:
		// Check if this message is for us
		if msg.FollowerName == h.supervisorName {
			h.handleFollowerStart(msg)
		}

	case messagebus.FollowerStopMessage:
		if msg.FollowerName == h.supervisorName {
			h.handleFollowerStop(msg)
		}

	case messagebus.GameAvailableMessage:
		h.mu.Lock()
		if h.isFollowing && msg.LeaderName == h.leaderName {
			h.mu.Unlock()
			h.handleGameAvailable(msg)
		} else {
			h.mu.Unlock()
		}

	case messagebus.LeaderLeftGameMessage:
		h.mu.Lock()
		if h.isFollowing && msg.LeaderName == h.leaderName {
			h.mu.Unlock()
			h.handleLeaderLeft(msg)
		} else {
			h.mu.Unlock()
		}

	case messagebus.ScoutClaimedMessage:
		h.mu.Lock()
		if h.isFollowing && msg.LeaderName == h.leaderName {
			h.scoutClaimed = true
		}
		h.mu.Unlock()
	}

	return nil
}

// handleFollowerStart handles the start following command from a leader
func (h *FollowerHandler) handleFollowerStart(msg messagebus.FollowerStartMessage) {
	h.mu.Lock()
	defer h.mu.Unlock()

	h.isFollowing = true
	h.leaderName = msg.LeaderName
	h.gamePattern = msg.GamePattern
	h.gamePassword = msg.GamePassword
	h.joinDelayMin = msg.JoinDelayMin
	h.joinDelayMax = msg.JoinDelayMax
	h.gameCounter = 0

	h.log.Info("Follower mode activated",
		slog.String("supervisor", h.supervisorName),
		slog.String("leader", msg.LeaderName),
		slog.String("pattern", msg.GamePattern))
}

// handleFollowerStop handles the stop following command
func (h *FollowerHandler) handleFollowerStop(msg messagebus.FollowerStopMessage) {
	h.mu.Lock()
	defer h.mu.Unlock()

	h.isFollowing = false
	h.leaderName = ""

	h.log.Info("Follower mode deactivated",
		slog.String("supervisor", h.supervisorName))
}

// handleGameAvailable handles when a new game is found
func (h *FollowerHandler) handleGameAvailable(msg messagebus.GameAvailableMessage) {
	h.log.Info("Game available for follower",
		slog.String("supervisor", h.supervisorName),
		slog.String("game", msg.GameName))

	// Send to channel for the follower run to pick up
	select {
	case h.gameAvailableCh <- &msg:
	default:
		// Channel full, replace with new message
		select {
		case <-h.gameAvailableCh:
		default:
		}
		h.gameAvailableCh <- &msg
	}
}

// handleLeaderLeft handles when the leader leaves the game
func (h *FollowerHandler) handleLeaderLeft(msg messagebus.LeaderLeftGameMessage) {
	h.log.Info("Leader left game",
		slog.String("supervisor", h.supervisorName),
		slog.String("leader", msg.LeaderName),
		slog.String("game", msg.GameName))

	// Signal that leader left
	select {
	case h.leaderLeftCh <- struct{}{}:
	default:
	}
}

// IsFollowing returns whether this handler is in follower mode
func (h *FollowerHandler) IsFollowing() bool {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.isFollowing
}

// GetLeaderName returns the current leader name
func (h *FollowerHandler) GetLeaderName() string {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.leaderName
}

// GetNextGameName returns the next game name based on the pattern
func (h *FollowerHandler) GetNextGameName() string {
	h.mu.Lock()
	defer h.mu.Unlock()

	h.gameCounter++
	return h.gamePattern + string(rune('0'+h.gameCounter))
}

// GetGamePassword returns the configured game password
func (h *FollowerHandler) GetGamePassword() string {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.gamePassword
}

// WaitForGameAvailable waits for a game to become available
func (h *FollowerHandler) WaitForGameAvailable(timeout time.Duration) *messagebus.GameAvailableMessage {
	select {
	case msg := <-h.gameAvailableCh:
		return msg
	case <-time.After(timeout):
		return nil
	}
}

// WaitForLeaderLeft waits for the leader to leave
func (h *FollowerHandler) WaitForLeaderLeft(timeout time.Duration) bool {
	select {
	case <-h.leaderLeftCh:
		return true
	case <-time.After(timeout):
		return false
	}
}

// TryClaimScout attempts to claim the scout role
func (h *FollowerHandler) TryClaimScout() bool {
	h.mu.Lock()
	defer h.mu.Unlock()

	if h.scoutClaimed {
		return false
	}

	h.isScout = true
	h.scoutClaimed = true

	// Broadcast scout claim
	h.bus.Publish(messagebus.NewScoutClaimedMessage(
		h.supervisorName,
		h.supervisorName,
		h.leaderName,
	))

	return true
}

// IsScout returns whether this follower is the scout
func (h *FollowerHandler) IsScout() bool {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.isScout
}

// ResetScoutState resets the scout state for a new game cycle
func (h *FollowerHandler) ResetScoutState() {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.isScout = false
	h.scoutClaimed = false
}

// CalculateJoinDelay calculates a random join delay
func (h *FollowerHandler) CalculateJoinDelay() time.Duration {
	h.mu.Lock()
	minDelay := h.joinDelayMin
	maxDelay := h.joinDelayMax
	h.mu.Unlock()

	if minDelay == 0 {
		minDelay = 2000
	}
	if maxDelay == 0 {
		maxDelay = 8000
	}
	if minDelay > maxDelay {
		minDelay, maxDelay = maxDelay, minDelay
	}

	delayMs := minDelay
	if maxDelay > minDelay {
		delayMs += rand.Intn(maxDelay - minDelay)
	}

	return time.Duration(delayMs) * time.Millisecond
}

// SetGameInfo updates the companion config with game info for joining
func (h *FollowerHandler) SetGameInfo(gameName, gamePassword string) {
	h.mu.Lock()
	h.currentGame = gameName
	h.mu.Unlock()

	// Update the companion config so the menu flow uses it
	h.cfg.Companion.CompanionGameName = gameName
	h.cfg.Companion.CompanionGamePassword = gamePassword
}

// GetCurrentGame returns the current game name
func (h *FollowerHandler) GetCurrentGame() string {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.currentGame
}

// BroadcastGameFound broadcasts that a game was found (called by scout)
func (h *FollowerHandler) BroadcastGameFound(gameName, gamePassword string) {
	h.mu.Lock()
	leaderName := h.leaderName
	h.mu.Unlock()

	h.log.Info("Scout broadcasting game found",
		slog.String("supervisor", h.supervisorName),
		slog.String("game", gameName))

	h.bus.Publish(messagebus.NewGameAvailableMessage(
		h.supervisorName,
		leaderName,
		gameName,
		gamePassword,
	))
}

// BroadcastJoinedGame broadcasts that we joined a game
func (h *FollowerHandler) BroadcastJoinedGame(gameName string) {
	h.mu.Lock()
	leaderName := h.leaderName
	h.mu.Unlock()

	h.bus.Publish(messagebus.NewBotJoinedGameMessage(
		h.supervisorName,
		h.supervisorName,
		gameName,
		leaderName,
	))
}

// BroadcastLeftGame broadcasts that we left a game
func (h *FollowerHandler) BroadcastLeftGame(gameName, reason string) {
	h.bus.Publish(messagebus.NewBotLeftGameMessage(
		h.supervisorName,
		h.supervisorName,
		gameName,
		reason,
	))
}

// Subscribe registers this handler with the message bus
func (h *FollowerHandler) Subscribe() func() {
	// Subscribe to game coordination topic
	unsub1 := h.bus.Subscribe(messagebus.TopicGameCoordination, h.supervisorName, func(msg messagebus.Message) {
		h.Handle(context.Background(), msg)
	})

	// Subscribe to leader status topic
	unsub2 := h.bus.Subscribe(messagebus.TopicLeaderStatus, h.supervisorName, func(msg messagebus.Message) {
		h.Handle(context.Background(), msg)
	})

	return func() {
		unsub1()
		unsub2()
	}
}

