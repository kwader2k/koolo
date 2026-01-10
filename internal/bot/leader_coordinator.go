package bot

import (
	"context"
	"fmt"
	"log/slog"
	"time"

	"github.com/hectorgimenez/koolo/internal/config"
	ct "github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

// FollowerStartFunc is a function type for starting follower supervisors
type FollowerStartFunc func(supervisorName string) error

// FollowerStopFunc is a function type for stopping follower supervisors
type FollowerStopFunc func(supervisorName string)

// LeaderCoordinator is a special supervisor for human leader mode
// It doesn't spawn D2R - it just coordinates follower bots
type LeaderCoordinator struct {
	name            string
	logger          *slog.Logger
	cfg             *config.CharacterCfg
	statsHandler    *StatsHandler
	cancelFn        context.CancelFunc
	ctx             *ct.Context
	startFollower   FollowerStartFunc
	stopFollower    FollowerStopFunc
	startedFollowers []string
}

// NewLeaderCoordinator creates a new leader coordinator for human leader mode
func NewLeaderCoordinator(name string, logger *slog.Logger, cfg *config.CharacterCfg, statsHandler *StatsHandler, startFn FollowerStartFunc, stopFn FollowerStopFunc) (*LeaderCoordinator, error) {
	// Create a minimal context for the coordinator (no game reader, etc.)
	ctxStatus := ct.NewContext(name)
	ctxStatus.CharacterCfg = cfg
	ctxStatus.Logger = logger

	return &LeaderCoordinator{
		name:            name,
		logger:          logger,
		cfg:             cfg,
		statsHandler:    statsHandler,
		ctx:             ctxStatus.Context,
		startFollower:   startFn,
		stopFollower:    stopFn,
		startedFollowers: make([]string, 0),
	}, nil
}

// Start begins the leader coordinator
func (lc *LeaderCoordinator) Start() error {
	// Validate configuration first
	if err := lc.validateConfig(); err != nil {
		return fmt.Errorf("invalid leader-follower configuration: %w", err)
	}

	ctx, cancel := context.WithCancel(context.Background())
	lc.cancelFn = cancel

	lc.logger.Info("Leader coordinator starting (human leader mode - no D2R instance)",
		slog.String("supervisor", lc.name),
		slog.String("leader", lc.cfg.LeaderFollower.LeaderName),
		slog.Int("followers", len(lc.cfg.LeaderFollower.Followers)))

	// Actually start all follower supervisors with delays (pass context for cancellation)
	if err := lc.startFollowerSupervisors(ctx); err != nil {
		if ctx.Err() != nil {
			// Context was cancelled during startup
			lc.logger.Info("Leader coordinator cancelled during follower startup")
			lc.stopFollowerSupervisors()
			return nil
		}
		lc.logger.Error("Failed to start some followers", slog.String("error", err.Error()))
	}

	// Wait for shutdown signal - followers handle their own game detection
	<-ctx.Done()
	lc.logger.Info("Leader coordinator stopping")
	lc.stopFollowerSupervisors()
	return nil
}

// validateConfig validates the leader-follower configuration
func (lc *LeaderCoordinator) validateConfig() error {
	if len(lc.cfg.LeaderFollower.Followers) == 0 {
		return fmt.Errorf("no followers configured")
	}
	if lc.cfg.LeaderFollower.LeaderName == "" {
		return fmt.Errorf("leader character name is required")
	}
	if lc.cfg.LeaderFollower.GameNamePattern == "" {
		return fmt.Errorf("game name pattern is required")
	}
	if lc.cfg.LeaderFollower.JoinDelayMin < 0 {
		return fmt.Errorf("join delay min cannot be negative")
	}
	if lc.cfg.LeaderFollower.JoinDelayMax < lc.cfg.LeaderFollower.JoinDelayMin {
		return fmt.Errorf("join delay max must be >= join delay min")
	}
	return nil
}

// startFollowerSupervisors starts each follower supervisor with a delay between them
func (lc *LeaderCoordinator) startFollowerSupervisors(ctx context.Context) error {
	// Use configured delay or default to 10 seconds
	delayBetweenStarts := 10 * time.Second
	if lc.cfg.LeaderFollower.JoinDelayMin > 0 {
		// Use half of min join delay for start staggering (sensible default)
		delayBetweenStarts = time.Duration(lc.cfg.LeaderFollower.JoinDelayMin/2) * time.Millisecond
		if delayBetweenStarts < 5*time.Second {
			delayBetweenStarts = 5 * time.Second
		}
	}
	
	var lastErr error

	for i, follower := range lc.cfg.LeaderFollower.Followers {
		// Check for context cancellation before each follower
		select {
		case <-ctx.Done():
			lc.logger.Info("Follower startup cancelled")
			return ctx.Err()
		default:
		}

		// Add delay before starting (except for first follower)
		if i > 0 {
			lc.logger.Info("Waiting before starting next follower",
				slog.String("nextFollower", follower),
				slog.Duration("delay", delayBetweenStarts))
			
			// Respect context cancellation during delay
			select {
			case <-ctx.Done():
				lc.logger.Info("Follower startup cancelled during delay")
				return ctx.Err()
			case <-time.After(delayBetweenStarts):
			}
		}

		lc.logger.Info("Starting follower supervisor",
			slog.String("follower", follower),
			slog.String("leader", lc.cfg.LeaderFollower.LeaderName),
			slog.Int("index", i+1),
			slog.Int("total", len(lc.cfg.LeaderFollower.Followers)))

		// Mark this follower as being started by this leader (in global registry)
		// Calculate poll interval with sensible default
		pollInterval := lc.cfg.LeaderFollower.PollInterval
		if pollInterval <= 0 {
			pollInterval = 5000 // Default 5 seconds
		}
		
		RegisterFollower(follower, lc.name, lc.cfg.LeaderFollower.LeaderName,
			lc.cfg.LeaderFollower.GameNamePattern, lc.cfg.LeaderFollower.GamePassword,
			lc.cfg.LeaderFollower.JoinDelayMin, lc.cfg.LeaderFollower.JoinDelayMax,
			pollInterval, lc.cfg.LeaderFollower.UseLegacyGraphics)

		if err := lc.startFollower(follower); err != nil {
			lc.logger.Error("Failed to start follower",
				slog.String("follower", follower),
				slog.String("error", err.Error()))
			lastErr = err
			UnregisterFollower(follower)
			continue
		}

		lc.startedFollowers = append(lc.startedFollowers, follower)
		lc.logger.Info("Follower supervisor started successfully",
			slog.String("follower", follower))
	}

	if lastErr != nil {
		return fmt.Errorf("one or more followers failed to start: %w", lastErr)
	}
	return nil
}

// stopFollowerSupervisors stops all started follower supervisors
func (lc *LeaderCoordinator) stopFollowerSupervisors() {
	for _, follower := range lc.startedFollowers {
		func(f string) {
			defer func() {
				if r := recover(); r != nil {
					lc.logger.Error("Panic while stopping follower",
						slog.String("follower", f),
						slog.Any("panic", r))
				}
			}()
			
			lc.logger.Info("Stopping follower supervisor",
				slog.String("follower", f))
			UnregisterFollower(f)
			if lc.stopFollower != nil {
				lc.stopFollower(f)
			}
		}(follower)
	}
	lc.startedFollowers = nil
}

// Stop stops the leader coordinator
func (lc *LeaderCoordinator) Stop() {
	lc.logger.Info("Stopping leader coordinator", slog.String("supervisor", lc.name))
	if lc.cancelFn != nil {
		lc.cancelFn()
	}
}

// Name returns the supervisor name
func (lc *LeaderCoordinator) Name() string {
	return lc.name
}

// Stats returns the current stats
func (lc *LeaderCoordinator) Stats() Stats {
	if lc.statsHandler != nil {
		return lc.statsHandler.Stats()
	}
	return Stats{
		SupervisorStatus: NotStarted,
	}
}

// TogglePause toggles pause state (no-op for coordinator)
func (lc *LeaderCoordinator) TogglePause() {
	lc.logger.Info("Pause not applicable for leader coordinator")
}

// SetWindowPosition sets window position (no-op for coordinator - no window)
func (lc *LeaderCoordinator) SetWindowPosition(x, y int) {
	// No window to position
}

// GetData returns game data (nil for coordinator - no game)
func (lc *LeaderCoordinator) GetData() *game.Data {
	return nil
}

// GetContext returns the context
func (lc *LeaderCoordinator) GetContext() *ct.Context {
	return lc.ctx
}

// IsLeaderFollowerMode checks if a config has the leader-follower run configured
func IsLeaderFollowerMode(cfg *config.CharacterCfg) bool {
	// Check if leader_follower is in the runs list
	for _, run := range cfg.Game.Runs {
		if run == config.LeaderFollowerRun {
			return true
		}
	}
	return false
}
