package run

import (
	"fmt"
	"log/slog"
	"math/rand"
	"sync"
	"time"

	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/bot/messagebus"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	// LeaderModeHuman indicates the leader is a human player (no D2R instance for leader supervisor)
	LeaderModeHuman = "human"
	// LeaderModeBot indicates the leader is a bot that creates games
	LeaderModeBot = "bot"
)

// LeaderFollower is a run that coordinates followers to join games with a leader
// In Human mode: No D2R spawned, coordinates follower bots to join human's games
// In Bot mode: Leader bot creates games and instructs followers to join
type LeaderFollower struct {
	ctx *context.Status
	bus *messagebus.Bus

	// State tracking
	mu               sync.Mutex
	isScout          bool
	scoutClaimed     bool
	gameAvailable    *messagebus.GameAvailableMessage
	leaderLeftSignal chan struct{}

	// Unsubscribe functions
	unsubscribers []func()
}

// NewLeaderFollower creates a new LeaderFollower run
func NewLeaderFollower() *LeaderFollower {
	lf := &LeaderFollower{
		ctx:              context.Get(),
		bus:              messagebus.GetBus(),
		leaderLeftSignal: make(chan struct{}, 1),
	}
	return lf
}

// Name returns the name of this run
func (lf *LeaderFollower) Name() string {
	return string(config.LeaderFollowerRun)
}

// SkipTownRoutines indicates this run doesn't need town preparation
func (lf *LeaderFollower) SkipTownRoutines() bool {
	return true
}

// CheckConditions verifies if the run can execute
func (lf *LeaderFollower) CheckConditions(parameters *RunParameters) SequencerResult {
	if !lf.ctx.CharacterCfg.LeaderFollower.Enabled {
		return SequencerSkip
	}

	mode := lf.ctx.CharacterCfg.LeaderFollower.Mode
	if mode != LeaderModeHuman && mode != LeaderModeBot {
		lf.ctx.Logger.Warn("LeaderFollower enabled but invalid mode configured",
			slog.String("mode", mode))
		return SequencerSkip
	}

	// For human mode, must have leader name and game pattern
	if mode == LeaderModeHuman {
		if lf.ctx.CharacterCfg.LeaderFollower.LeaderName == "" {
			lf.ctx.Logger.Warn("Human leader mode requires leaderName to be configured")
			return SequencerSkip
		}
		if lf.ctx.CharacterCfg.LeaderFollower.GameNamePattern == "" {
			lf.ctx.Logger.Warn("Human leader mode requires gameNamePattern to be configured")
			return SequencerSkip
		}
	}

	// Must have followers configured
	if len(lf.ctx.CharacterCfg.LeaderFollower.Followers) == 0 {
		lf.ctx.Logger.Warn("LeaderFollower enabled but no followers configured")
		return SequencerSkip
	}

	return SequencerOk
}

// Run executes the leader-follower coordination logic
func (lf *LeaderFollower) Run(parameters *RunParameters) error {
	mode := lf.ctx.CharacterCfg.LeaderFollower.Mode

	lf.ctx.Logger.Info("LeaderFollower run started",
		slog.String("mode", mode),
		slog.Int("followers", len(lf.ctx.CharacterCfg.LeaderFollower.Followers)))

	// Subscribe to message bus events
	lf.subscribeToEvents()
	defer lf.unsubscribeAll()

	if mode == LeaderModeHuman {
		return lf.runHumanLeaderMode()
	}
	return lf.runBotLeaderMode()
}

// runHumanLeaderMode runs the coordination for human leader mode
// In this mode, no D2R is spawned - we just coordinate followers
func (lf *LeaderFollower) runHumanLeaderMode() error {
	leaderName := lf.ctx.CharacterCfg.LeaderFollower.LeaderName
	pollInterval := lf.ctx.CharacterCfg.LeaderFollower.PollInterval
	if pollInterval == 0 {
		pollInterval = 30000 // Default 30 seconds
	}

	lf.ctx.Logger.Info("Human leader mode: coordinating followers",
		slog.String("leader", leaderName),
		slog.Int("pollIntervalMs", pollInterval))

	// Notify all followers to start in follower mode
	lf.notifyFollowersToStart()

	// In human leader mode, the leader supervisor doesn't run D2R
	// It just coordinates followers via the message bus
	// The actual game joining is handled by the followers themselves

	// Main coordination loop - monitors when followers should transition
	for {
		if lf.ctx.ExecutionPriority == context.PriorityStop {
			lf.ctx.Logger.Info("LeaderFollower run stopping")
			return nil
		}

		// Wait for poll interval
		time.Sleep(time.Duration(pollInterval) * time.Millisecond)

		// In human mode, the leader supervisor just keeps running
		// Followers handle their own game detection and joining
	}
}

// runBotLeaderMode runs the coordination for bot leader mode
func (lf *LeaderFollower) runBotLeaderMode() error {
	lf.ctx.Logger.Info("Bot leader mode: creating games and coordinating followers")

	// Notify followers to start
	lf.notifyFollowersToStart()

	// Main loop - create games and coordinate followers
	for {
		if lf.ctx.ExecutionPriority == context.PriorityStop {
			lf.ctx.Logger.Info("LeaderFollower run stopping")
			return nil
		}

		// Wait for game to be created (handled by normal game creation flow)
		// Then notify followers

		gameName := lf.ctx.Data.Game.LastGameName
		gamePassword := lf.ctx.Data.Game.LastGamePassword

		if gameName != "" {
			// Broadcast game available to followers
			lf.bus.Publish(messagebus.NewGameAvailableMessage(
				lf.ctx.Name,
				lf.ctx.CharacterCfg.CharacterName,
				gameName,
				gamePassword,
			))

			lf.ctx.Logger.Info("Notified followers of new game",
				slog.String("game", gameName))
		}

		// Wait for leader to finish this game
		// This will be triggered when the game ends naturally
		return nil
	}
}

// notifyFollowersToStart sends a message to all configured followers to start following
func (lf *LeaderFollower) notifyFollowersToStart() {
	leaderName := lf.ctx.CharacterCfg.LeaderFollower.LeaderName
	if lf.ctx.CharacterCfg.LeaderFollower.Mode == LeaderModeBot {
		leaderName = lf.ctx.CharacterCfg.CharacterName
	}

	for _, follower := range lf.ctx.CharacterCfg.LeaderFollower.Followers {
		lf.ctx.Logger.Debug("Notifying follower to start",
			slog.String("follower", follower))

		// Send start following message
		lf.bus.Publish(messagebus.NewFollowerStartMessage(
			lf.ctx.Name,
			follower,
			leaderName,
			lf.ctx.CharacterCfg.LeaderFollower.GameNamePattern,
			lf.ctx.CharacterCfg.LeaderFollower.GamePassword,
			lf.ctx.CharacterCfg.LeaderFollower.JoinDelayMin,
			lf.ctx.CharacterCfg.LeaderFollower.JoinDelayMax,
		))
	}
}

// subscribeToEvents sets up message bus subscriptions
func (lf *LeaderFollower) subscribeToEvents() {
	// Subscribe to follower status updates
	unsub := lf.bus.Subscribe(messagebus.TopicBotStatus, lf.ctx.Name, func(msg messagebus.Message) {
		switch m := msg.(type) {
		case messagebus.BotJoinedGameMessage:
			lf.ctx.Logger.Info("Follower joined game",
				slog.String("follower", m.BotName),
				slog.String("game", m.GameName))
		case messagebus.BotLeftGameMessage:
			lf.ctx.Logger.Info("Follower left game",
				slog.String("follower", m.BotName),
				slog.String("game", m.GameName))
		}
	})
	lf.unsubscribers = append(lf.unsubscribers, unsub)
}

// unsubscribeAll removes all message bus subscriptions
func (lf *LeaderFollower) unsubscribeAll() {
	for _, unsub := range lf.unsubscribers {
		unsub()
	}
	lf.unsubscribers = nil
}

// GetNextGameName returns the next game name in the pattern sequence
func (lf *LeaderFollower) GetNextGameName() string {
	lf.mu.Lock()
	defer lf.mu.Unlock()

	lf.ctx.CharacterCfg.LeaderFollower.CurrentGameNumber++
	pattern := lf.ctx.CharacterCfg.LeaderFollower.GameNamePattern
	return fmt.Sprintf("%s%d", pattern, lf.ctx.CharacterCfg.LeaderFollower.CurrentGameNumber)
}

// ResetGameCounter resets the game counter (useful for new sessions)
func (lf *LeaderFollower) ResetGameCounter() {
	lf.mu.Lock()
	defer lf.mu.Unlock()
	lf.ctx.CharacterCfg.LeaderFollower.CurrentGameNumber = 0
}

// FollowerRun handles the follower side of the leader-follower relationship
// This is used by supervisors that are listed as followers in another supervisor's config
type FollowerRun struct {
	ctx *context.Status
	bus *messagebus.Bus

	// Configuration received from leader
	mu             sync.Mutex
	leaderName     string
	gamePattern    string
	gamePassword   string
	joinDelayMin   int
	joinDelayMax   int
	currentGame    string
	isActive       bool
	isScout        bool
	scoutClaimed   bool
	gameAvailable  *messagebus.GameAvailableMessage

	// Channels
	startSignal      chan *messagebus.FollowerStartMessage
	gameFoundSignal  chan *messagebus.GameAvailableMessage
	leaderLeftSignal chan struct{}

	// Unsubscribe functions
	unsubscribers []func()
}

// NewFollowerRun creates a new follower run
func NewFollowerRun() *FollowerRun {
	fr := &FollowerRun{
		ctx:              context.Get(),
		bus:              messagebus.GetBus(),
		startSignal:      make(chan *messagebus.FollowerStartMessage, 1),
		gameFoundSignal:  make(chan *messagebus.GameAvailableMessage, 1),
		leaderLeftSignal: make(chan struct{}, 1),
	}
	return fr
}

// Name returns the name of this run
func (fr *FollowerRun) Name() string {
	return "follower"
}

// SkipTownRoutines indicates this run doesn't need town preparation
func (fr *FollowerRun) SkipTownRoutines() bool {
	return true
}

// CheckConditions - followers are activated by message, not config
func (fr *FollowerRun) CheckConditions(parameters *RunParameters) SequencerResult {
	// Follower runs are triggered via message bus, not by direct config
	// Return skip - the follower handler will inject this run when needed
	return SequencerSkip
}

// Run executes the follower logic
func (fr *FollowerRun) Run(parameters *RunParameters) error {
	fr.ctx.Logger.Info("Follower mode active",
		slog.String("leader", fr.leaderName))

	// Subscribe to events
	fr.subscribeToEvents()
	defer fr.unsubscribeAll()

	for {
		if fr.ctx.ExecutionPriority == context.PriorityStop {
			return nil
		}

		// Phase 1: Wait for game info and join
		if err := fr.waitForGameAndJoin(); err != nil {
			return err
		}

		// Phase 2: Wait for leader in game and join party
		if err := fr.waitForLeaderAndJoinParty(); err != nil {
			fr.ctx.Logger.Warn("Failed to join party", slog.String("error", err.Error()))
		}

		// Phase 3: Idle while leader is present
		if err := fr.idleWhileLeaderPresent(); err != nil {
			fr.ctx.Logger.Info("Leader left, preparing to rejoin")
			return err // Trigger game exit and rejoin
		}
	}
}

// waitForGameAndJoin waits for game available message and joins with staggered delay
func (fr *FollowerRun) waitForGameAndJoin() error {
	fr.ctx.Logger.Info("Waiting for game to be available")

	timeout := time.Duration(30) * time.Second
	startTime := time.Now()

	for {
		select {
		case gameMsg := <-fr.gameFoundSignal:
			// Game found - calculate delay
			delay := fr.calculateJoinDelay()
			fr.ctx.Logger.Info("Game available, waiting before joining",
				slog.String("game", gameMsg.GameName),
				slog.Duration("delay", delay))

			time.Sleep(delay)

			// Set game info for menu flow
			fr.ctx.CharacterCfg.Companion.CompanionGameName = gameMsg.GameName
			fr.ctx.CharacterCfg.Companion.CompanionGamePassword = gameMsg.GamePassword
			fr.currentGame = gameMsg.GameName

			// Notify that we're joining
			fr.bus.Publish(messagebus.NewBotJoinedGameMessage(
				fr.ctx.Name,
				fr.ctx.Name,
				gameMsg.GameName,
				fr.leaderName,
			))

			return nil

		default:
			if time.Since(startTime) > timeout {
				return fmt.Errorf("timeout waiting for game")
			}

			// Check if we should try to be scout
			fr.mu.Lock()
			if !fr.scoutClaimed {
				fr.isScout = true
				fr.scoutClaimed = true
				fr.mu.Unlock()

				// Broadcast scout claim
				fr.bus.Publish(messagebus.NewScoutClaimedMessage(
					fr.ctx.Name,
					fr.ctx.Name,
					fr.leaderName,
				))

				// Scout searches for game
				return fr.scoutForGame()
			}
			fr.mu.Unlock()

			utils.Sleep(500)
		}
	}
}

// scoutForGame searches for the next game in the pattern
func (fr *FollowerRun) scoutForGame() error {
	fr.ctx.Logger.Info("Scout mode: searching for next game")

	// Increment game counter and try to find game
	// This would involve lobby scanning or pattern matching
	// For now, use the companion game name from config
	gameName := fr.ctx.CharacterCfg.Companion.CompanionGameName
	gamePassword := fr.ctx.CharacterCfg.Companion.CompanionGamePassword

	if gameName == "" {
		return fmt.Errorf("no game name available for scout")
	}

	// Broadcast game found
	fr.bus.Publish(messagebus.NewGameAvailableMessage(
		fr.ctx.Name,
		fr.leaderName,
		gameName,
		gamePassword,
	))

	// Set our own game info
	fr.currentGame = gameName

	return nil
}

// waitForLeaderAndJoinParty waits for the leader to appear and joins their party
func (fr *FollowerRun) waitForLeaderAndJoinParty() error {
	timeout := 60 * time.Second

	fr.ctx.Logger.Info("Waiting for leader to appear",
		slog.String("leader", fr.leaderName))

	if !action.WaitForPlayerInRoster(fr.leaderName, timeout) {
		return fmt.Errorf("leader %s did not appear in game", fr.leaderName)
	}

	// Try to join party
	return action.JoinPlayerParty(fr.leaderName, 5)
}

// idleWhileLeaderPresent idles in town while monitoring leader
func (fr *FollowerRun) idleWhileLeaderPresent() error {
	fr.ctx.Logger.Info("Idling while leader is present",
		slog.String("leader", fr.leaderName))

	checkTicker := time.NewTicker(2 * time.Second)
	defer checkTicker.Stop()

	for {
		select {
		case <-fr.leaderLeftSignal:
			return fmt.Errorf("leader left game")

		case <-checkTicker.C:
			if fr.ctx.ExecutionPriority == context.PriorityStop {
				return fmt.Errorf("stopped")
			}

			fr.ctx.RefreshGameData()

			if !action.FindPlayerInRoster(fr.leaderName) {
				// Broadcast leader left
				fr.bus.Publish(messagebus.NewLeaderLeftGameMessage(
					fr.ctx.Name,
					fr.leaderName,
					fr.currentGame,
				))
				return fmt.Errorf("leader left game")
			}
		}
	}
}

// subscribeToEvents sets up message bus subscriptions for follower
func (fr *FollowerRun) subscribeToEvents() {
	// Subscribe to game coordination
	unsub1 := fr.bus.Subscribe(messagebus.TopicGameCoordination, fr.ctx.Name, func(msg messagebus.Message) {
		switch m := msg.(type) {
		case messagebus.GameAvailableMessage:
			if m.LeaderName == fr.leaderName {
				select {
				case fr.gameFoundSignal <- &m:
				default:
				}
			}
		case messagebus.ScoutClaimedMessage:
			if m.LeaderName == fr.leaderName {
				fr.mu.Lock()
				fr.scoutClaimed = true
				fr.mu.Unlock()
			}
		}
	})
	fr.unsubscribers = append(fr.unsubscribers, unsub1)

	// Subscribe to leader status
	unsub2 := fr.bus.Subscribe(messagebus.TopicLeaderStatus, fr.ctx.Name, func(msg messagebus.Message) {
		switch m := msg.(type) {
		case messagebus.LeaderLeftGameMessage:
			if m.LeaderName == fr.leaderName {
				select {
				case fr.leaderLeftSignal <- struct{}{}:
				default:
				}
			}
		}
	})
	fr.unsubscribers = append(fr.unsubscribers, unsub2)
}

// unsubscribeAll removes all subscriptions
func (fr *FollowerRun) unsubscribeAll() {
	for _, unsub := range fr.unsubscribers {
		unsub()
	}
	fr.unsubscribers = nil
}

// calculateJoinDelay calculates random delay for joining
func (fr *FollowerRun) calculateJoinDelay() time.Duration {
	minDelay := fr.joinDelayMin
	maxDelay := fr.joinDelayMax

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

// SetLeaderConfig sets the follower's leader configuration (called from message handler)
func (fr *FollowerRun) SetLeaderConfig(leaderName, gamePattern, gamePassword string, joinDelayMin, joinDelayMax int) {
	fr.mu.Lock()
	defer fr.mu.Unlock()

	fr.leaderName = leaderName
	fr.gamePattern = gamePattern
	fr.gamePassword = gamePassword
	fr.joinDelayMin = joinDelayMin
	fr.joinDelayMax = joinDelayMax
	fr.isActive = true
}

// ResetState resets follower state for a new game cycle
func (fr *FollowerRun) ResetState() {
	fr.mu.Lock()
	defer fr.mu.Unlock()

	fr.isScout = false
	fr.scoutClaimed = false
	fr.gameAvailable = nil
	fr.currentGame = ""

	// Drain channels
	select {
	case <-fr.startSignal:
	default:
	}
	select {
	case <-fr.gameFoundSignal:
	default:
	}
	select {
	case <-fr.leaderLeftSignal:
	default:
	}
}

