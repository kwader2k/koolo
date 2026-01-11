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

// LeecherStartFunc is a function type for starting leecher supervisors
type LeecherStartFunc func(supervisorName string) error

// LeecherStopFunc is a function type for stopping leecher supervisors
type LeecherStopFunc func(supervisorName string)

// LeecherCoordinator is a special supervisor for human leader mode (leecher)
// It doesn't spawn D2R - it just coordinates leecher bots via the control panel
// It also supports "followers" - simpler bots that just join and idle for XP boost
type LeecherCoordinator struct {
	name             string
	logger           *slog.Logger
	cfg              *config.CharacterCfg
	statsHandler     *StatsHandler
	cancelFn         context.CancelFunc
	ctx              *ct.Context
	startLeecher     LeecherStartFunc
	stopLeecher      LeecherStopFunc
	startedLeechers  []string
	startedFollowers []string
}

// NewLeecherCoordinator creates a new leecher coordinator for human leader mode
func NewLeecherCoordinator(name string, logger *slog.Logger, cfg *config.CharacterCfg, statsHandler *StatsHandler, startFn LeecherStartFunc, stopFn LeecherStopFunc) (*LeecherCoordinator, error) {
	// Create a minimal context for the coordinator (no game reader, etc.)
	ctxStatus := ct.NewContext(name)
	ctxStatus.CharacterCfg = cfg
	ctxStatus.Logger = logger

	return &LeecherCoordinator{
		name:             name,
		logger:           logger,
		cfg:              cfg,
		statsHandler:     statsHandler,
		ctx:              ctxStatus.Context,
		startLeecher:     startFn,
		stopLeecher:      stopFn,
		startedLeechers:  make([]string, 0),
		startedFollowers: make([]string, 0),
	}, nil
}

// Start begins the leecher coordinator
func (lc *LeecherCoordinator) Start() error {
	// Validate configuration first
	if err := lc.validateConfig(); err != nil {
		return fmt.Errorf("invalid leader-leecher configuration: %w", err)
	}

	ctx, cancel := context.WithCancel(context.Background())
	lc.cancelFn = cancel

	lc.logger.Info("Leecher coordinator starting (human leader mode - no D2R instance)",
		slog.String("supervisor", lc.name),
		slog.String("leader", lc.cfg.LeaderLeecher.LeaderName),
		slog.Int("leechers", len(lc.cfg.LeaderLeecher.Leechers)),
		slog.Int("followers", len(lc.cfg.LeaderLeecher.Followers)))

	// Actually start all leecher supervisors with delays (pass context for cancellation)
	if err := lc.startLeecherSupervisors(ctx); err != nil {
		if ctx.Err() != nil {
			// Context was cancelled during startup
			lc.logger.Info("Leecher coordinator cancelled during leecher startup")
			lc.stopAllSupervisors()
			return nil
		}
		lc.logger.Error("Failed to start some leechers", slog.String("error", err.Error()))
	}

	// Start followers (simpler bots that just idle for XP boost)
	if len(lc.cfg.LeaderLeecher.Followers) > 0 {
		if err := lc.startFollowerSupervisors(ctx); err != nil {
			if ctx.Err() != nil {
				lc.logger.Info("Leecher coordinator cancelled during follower startup")
				lc.stopAllSupervisors()
				return nil
			}
			lc.logger.Error("Failed to start some followers", slog.String("error", err.Error()))
		}
	}

	// Wait for shutdown signal - leechers handle their own game detection
	// Commands are sent via the web control panel
	<-ctx.Done()
	lc.logger.Info("Leecher coordinator stopping")
	lc.stopAllSupervisors()
	return nil
}

// validateConfig validates the leader-leecher configuration
func (lc *LeecherCoordinator) validateConfig() error {
	// Must have at least one leecher or follower
	if len(lc.cfg.LeaderLeecher.Leechers) == 0 && len(lc.cfg.LeaderLeecher.Followers) == 0 {
		return fmt.Errorf("no leechers or followers configured")
	}
	if lc.cfg.LeaderLeecher.LeaderName == "" {
		return fmt.Errorf("leader character name is required")
	}
	if lc.cfg.LeaderLeecher.GameNamePattern == "" {
		return fmt.Errorf("game name pattern is required")
	}
	
	// Check for duplicates between leechers and followers
	leecherSet := make(map[string]bool)
	for _, leecher := range lc.cfg.LeaderLeecher.Leechers {
		leecherSet[leecher] = true
	}
	for _, follower := range lc.cfg.LeaderLeecher.Followers {
		if leecherSet[follower] {
			return fmt.Errorf("supervisor '%s' is configured as both a leecher and follower - each supervisor can only be one", follower)
		}
	}
	if lc.cfg.LeaderLeecher.JoinDelayMin < 0 {
		return fmt.Errorf("join delay min cannot be negative")
	}
	if lc.cfg.LeaderLeecher.JoinDelayMax < lc.cfg.LeaderLeecher.JoinDelayMin {
		return fmt.Errorf("join delay max must be >= join delay min")
	}
	return nil
}

// startLeecherSupervisors starts each leecher supervisor with a delay between them
func (lc *LeecherCoordinator) startLeecherSupervisors(ctx context.Context) error {
	// Use configured start instance delay, with sensible defaults
	delayBetweenStarts := time.Duration(lc.cfg.LeaderLeecher.StartInstanceDelay) * time.Millisecond
	if delayBetweenStarts <= 0 {
		// Default to 500ms if not configured
		delayBetweenStarts = 500 * time.Millisecond
	}

	var lastErr error

	for i, leecher := range lc.cfg.LeaderLeecher.Leechers {
		// Check for context cancellation before each leecher
		select {
		case <-ctx.Done():
			lc.logger.Info("Leecher startup cancelled")
			return ctx.Err()
		default:
		}

		// Add delay before starting (except for first leecher)
		if i > 0 {
			lc.logger.Info("Waiting before starting next leecher",
				slog.String("nextLeecher", leecher),
				slog.Duration("delay", delayBetweenStarts))

			// Respect context cancellation during delay
			select {
			case <-ctx.Done():
				lc.logger.Info("Leecher startup cancelled during delay")
				return ctx.Err()
			case <-time.After(delayBetweenStarts):
			}
		}

		lc.logger.Info("Starting leecher supervisor",
			slog.String("leecher", leecher),
			slog.String("leader", lc.cfg.LeaderLeecher.LeaderName),
			slog.Int("index", i+1),
			slog.Int("total", len(lc.cfg.LeaderLeecher.Leechers)))

		// Mark this leecher as being started by this leader (in global registry)
		// Calculate poll interval with sensible default
		pollInterval := lc.cfg.LeaderLeecher.PollInterval
		if pollInterval <= 0 {
			pollInterval = 2000 // Default 2 seconds for leechers (more responsive)
		}

		maxDistance := lc.cfg.LeaderLeecher.MaxLeaderDistance
		if maxDistance <= 0 {
			maxDistance = 35 // Default XP range
		}

		portalDelay := lc.cfg.LeaderLeecher.PortalEntryDelay
		if portalDelay < 0 {
			portalDelay = 2
		}

		RegisterLeecher(leecher, lc.name, lc.cfg.LeaderLeecher.LeaderName,
			lc.cfg.LeaderLeecher.GameNamePattern, lc.cfg.LeaderLeecher.GamePassword,
			lc.cfg.LeaderLeecher.JoinDelayMin, lc.cfg.LeaderLeecher.JoinDelayMax,
			pollInterval, portalDelay, maxDistance, lc.cfg.LeaderLeecher.UseLegacyGraphics)

		if err := lc.startLeecher(leecher); err != nil {
			lc.logger.Error("Failed to start leecher",
				slog.String("leecher", leecher),
				slog.String("error", err.Error()))
			lastErr = err
			UnregisterLeecher(leecher)
			continue
		}

		lc.startedLeechers = append(lc.startedLeechers, leecher)
		lc.logger.Info("Leecher supervisor started successfully",
			slog.String("leecher", leecher))
	}

	if lastErr != nil {
		return fmt.Errorf("one or more leechers failed to start: %w", lastErr)
	}
	return nil
}

// startFollowerSupervisors starts each follower supervisor (idle XP bots)
func (lc *LeecherCoordinator) startFollowerSupervisors(ctx context.Context) error {
	// Use configured start instance delay, with sensible defaults
	delayBetweenStarts := time.Duration(lc.cfg.LeaderLeecher.StartInstanceDelay) * time.Millisecond
	if delayBetweenStarts <= 0 {
		// Default to 500ms if not configured
		delayBetweenStarts = 500 * time.Millisecond
	}

	var lastErr error

	for i, follower := range lc.cfg.LeaderLeecher.Followers {
		// Check for context cancellation before each follower
		select {
		case <-ctx.Done():
			lc.logger.Info("Follower startup cancelled")
			return ctx.Err()
		default:
		}

		// Add delay before starting (except for first follower, and after leechers)
		if i > 0 || len(lc.startedLeechers) > 0 {
			lc.logger.Info("Waiting before starting next follower",
				slog.String("nextFollower", follower),
				slog.Duration("delay", delayBetweenStarts))

			select {
			case <-ctx.Done():
				lc.logger.Info("Follower startup cancelled during delay")
				return ctx.Err()
			case <-time.After(delayBetweenStarts):
			}
		}

		lc.logger.Info("Starting follower supervisor (idle XP bot)",
			slog.String("follower", follower),
			slog.String("leader", lc.cfg.LeaderLeecher.LeaderName),
			slog.Int("index", i+1),
			slog.Int("total", len(lc.cfg.LeaderLeecher.Followers)))

		// Calculate poll interval with sensible default
		pollInterval := lc.cfg.LeaderLeecher.PollInterval
		if pollInterval <= 0 {
			pollInterval = 30000 // Default 30 seconds for followers (less responsive needed)
		}

		// Register as follower (simpler than leecher - just idles in town)
		RegisterFollower(follower, lc.name, lc.cfg.LeaderLeecher.LeaderName,
			lc.cfg.LeaderLeecher.GameNamePattern, lc.cfg.LeaderLeecher.GamePassword,
			lc.cfg.LeaderLeecher.JoinDelayMin, lc.cfg.LeaderLeecher.JoinDelayMax,
			pollInterval, lc.cfg.LeaderLeecher.UseLegacyGraphics)

		if err := lc.startLeecher(follower); err != nil {
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

// stopAllSupervisors stops all started leecher and follower supervisors
func (lc *LeecherCoordinator) stopAllSupervisors() {
	// Stop leechers
	for _, leecher := range lc.startedLeechers {
		func(l string) {
			defer func() {
				if r := recover(); r != nil {
					lc.logger.Error("Panic while stopping leecher",
						slog.String("leecher", l),
						slog.Any("panic", r))
				}
			}()

			lc.logger.Info("Stopping leecher supervisor",
				slog.String("leecher", l))
			UnregisterLeecher(l)
			if lc.stopLeecher != nil {
				lc.stopLeecher(l)
			}
		}(leecher)
	}
	lc.startedLeechers = nil

	// Stop followers
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
			if lc.stopLeecher != nil {
				lc.stopLeecher(f)
			}
		}(follower)
	}
	lc.startedFollowers = nil
}

// Stop stops the leecher coordinator
func (lc *LeecherCoordinator) Stop() {
	lc.logger.Info("Stopping leecher coordinator", slog.String("supervisor", lc.name))
	if lc.cancelFn != nil {
		lc.cancelFn()
	}
}

// Name returns the supervisor name
func (lc *LeecherCoordinator) Name() string {
	return lc.name
}

// Stats returns the current stats
func (lc *LeecherCoordinator) Stats() Stats {
	if lc.statsHandler != nil {
		return lc.statsHandler.Stats()
	}
	return Stats{
		SupervisorStatus: NotStarted,
	}
}

// TogglePause toggles pause state (no-op for coordinator)
func (lc *LeecherCoordinator) TogglePause() {
	lc.logger.Info("Pause not applicable for leecher coordinator")
}

// SetWindowPosition sets window position (no-op for coordinator - no window)
func (lc *LeecherCoordinator) SetWindowPosition(x, y int) {
	// No window to position
}

// GetData returns game data (nil for coordinator - no game)
func (lc *LeecherCoordinator) GetData() *game.Data {
	return nil
}

// GetContext returns the context
func (lc *LeecherCoordinator) GetContext() *ct.Context {
	return lc.ctx
}

// IsLeaderLeecherMode checks if a config has the leader-leecher run configured
func IsLeaderLeecherMode(cfg *config.CharacterCfg) bool {
	// Check if leader_leecher is in the runs list
	for _, run := range cfg.Game.Runs {
		if run == config.LeaderLeecherRun {
			return true
		}
	}
	return false
}
