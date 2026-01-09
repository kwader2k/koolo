package run

import (
	"log/slog"
	"sync"
	"time"

	"github.com/hectorgimenez/koolo/internal/bot/messagebus"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

// LeaderLeecher is a run that coordinates leechers to follow a leader through portals
// In Human mode: No D2R spawned for leader, coordinates leecher bots via control panel
// In Bot mode: Leader bot signals leechers when to enter portals
type LeaderLeecher struct {
	ctx *context.Status
	bus *messagebus.Bus

	// State tracking
	mu sync.Mutex

	// Unsubscribe functions
	unsubscribers []func()
}

// NewLeaderLeecher creates a new LeaderLeecher run
func NewLeaderLeecher() *LeaderLeecher {
	ll := &LeaderLeecher{
		ctx: context.Get(),
		bus: messagebus.GetBus(),
	}
	return ll
}

// Name returns the name of this run
func (ll *LeaderLeecher) Name() string {
	return string(config.LeaderLeecherRun)
}

// SkipTownRoutines indicates this run doesn't need town preparation
func (ll *LeaderLeecher) SkipTownRoutines() bool {
	return true
}

// CheckConditions verifies if the run can execute
func (ll *LeaderLeecher) CheckConditions(parameters *RunParameters) SequencerResult {
	// Must have leader name configured
	if ll.ctx.CharacterCfg.LeaderLeecher.LeaderName == "" {
		ll.ctx.Logger.Warn("LeaderLeecher requires leaderName to be configured")
		return SequencerSkip
	}

	// Must have game pattern configured
	if ll.ctx.CharacterCfg.LeaderLeecher.GameNamePattern == "" {
		ll.ctx.Logger.Warn("LeaderLeecher requires gameNamePattern to be configured")
		return SequencerSkip
	}

	// Must have leechers configured
	if len(ll.ctx.CharacterCfg.LeaderLeecher.Leechers) == 0 {
		ll.ctx.Logger.Warn("LeaderLeecher requires at least one leecher configured")
		return SequencerSkip
	}

	return SequencerOk
}

// Run executes the leader-leecher coordination logic
func (ll *LeaderLeecher) Run(parameters *RunParameters) error {
	ll.ctx.Logger.Info("LeaderLeecher run started",
		slog.Int("leechers", len(ll.ctx.CharacterCfg.LeaderLeecher.Leechers)))

	// Subscribe to message bus events
	ll.subscribeToEvents()
	defer ll.unsubscribeAll()

	return ll.runHumanLeaderMode()
}

// runHumanLeaderMode runs the coordination for human leader mode
// In this mode, no D2R is spawned - we just coordinate leechers via the control panel
func (ll *LeaderLeecher) runHumanLeaderMode() error {
	leaderName := ll.ctx.CharacterCfg.LeaderLeecher.LeaderName
	pollInterval := ll.ctx.CharacterCfg.LeaderLeecher.PollInterval
	if pollInterval == 0 {
		pollInterval = 5000 // Default 5 seconds
	}

	ll.ctx.Logger.Info("Human leader mode (leecher): coordinating leechers",
		slog.String("leader", leaderName),
		slog.Int("pollIntervalMs", pollInterval))

	// Notify all leechers to start in leecher mode
	ll.notifyLeechersToStart()

	// Main coordination loop - in human mode, commands come from the control panel
	// This loop just keeps the run alive and handles cleanup
	for {
		if ll.ctx.ExecutionPriority == context.PriorityStop {
			ll.ctx.Logger.Info("LeaderLeecher run stopping")
			return nil
		}

		// Wait for poll interval
		time.Sleep(time.Duration(pollInterval) * time.Millisecond)

		// In human mode, the leader supervisor just keeps running
		// Leechers handle their own game detection and joining
		// Commands are sent via the web control panel
	}
}

// notifyLeechersToStart sends a message to all configured leechers to start
func (ll *LeaderLeecher) notifyLeechersToStart() {
	leaderName := ll.ctx.CharacterCfg.LeaderLeecher.LeaderName
	cfg := ll.ctx.CharacterCfg.LeaderLeecher

	for _, leecher := range cfg.Leechers {
		ll.ctx.Logger.Debug("Notifying leecher to start",
			slog.String("leecher", leecher))

		// Send leecher start message via game coordination topic
		ll.bus.Publish(messagebus.NewLeecherStartMessage(
			ll.ctx.Name,
			leecher,
			leaderName,
			cfg.GameNamePattern,
			cfg.GamePassword,
			cfg.JoinDelayMin,
			cfg.JoinDelayMax,
			cfg.PollInterval,
			cfg.PortalEntryDelay,
			cfg.MaxLeaderDistance,
			cfg.UseLegacyGraphics,
		))
	}
}

// subscribeToEvents sets up message bus subscriptions
func (ll *LeaderLeecher) subscribeToEvents() {
	// Subscribe to leecher status updates
	unsub := ll.bus.Subscribe(messagebus.TopicBotStatus, ll.ctx.Name, func(msg messagebus.Message) {
		switch m := msg.(type) {
		case messagebus.BotJoinedGameMessage:
			ll.ctx.Logger.Info("Leecher joined game",
				slog.String("leecher", m.BotName),
				slog.String("game", m.GameName))
		case messagebus.BotLeftGameMessage:
			ll.ctx.Logger.Info("Leecher left game",
				slog.String("leecher", m.BotName),
				slog.String("game", m.GameName))
		}
	})
	ll.unsubscribers = append(ll.unsubscribers, unsub)
}

// unsubscribeAll removes all message bus subscriptions
func (ll *LeaderLeecher) unsubscribeAll() {
	for _, unsub := range ll.unsubscribers {
		unsub()
	}
	ll.unsubscribers = nil
}
