package bot

import (
	"context"
	"log/slog"
	"sync"
	"time"

	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/event"
)

// ShrineLocation stores information about a found experience shrine
type ShrineLocation struct {
	CompanionName string
	AreaName      string
	AreaID        int
	X             int
	Y             int
	Timestamp     time.Time
}

// CompanionEventHandler handles events related to companion functionality
type CompanionEventHandler struct {
	supervisor          string
	log                 *slog.Logger
	cfg                 *config.CharacterCfg
	lastLeaderHeartbeat time.Time
	leaderInGame        bool
	currentGameName     string
	exitGameChan        chan struct{}
	shrineLocations     []ShrineLocation // Stores shrine locations found by companions
	mu                  sync.RWMutex     // Protects heartbeat fields and shrine locations
}

// NewCompanionEventHandler creates a new instance of CompanionEventHandler
func NewCompanionEventHandler(supervisor string, log *slog.Logger, cfg *config.CharacterCfg) *CompanionEventHandler {
	return &CompanionEventHandler{
		supervisor:          supervisor,
		log:                 log,
		cfg:                 cfg,
		exitGameChan:        make(chan struct{}, 1),
		lastLeaderHeartbeat: time.Time{},
		leaderInGame:        false,
		currentGameName:     "",
		shrineLocations:     make([]ShrineLocation, 0),
	}
}

// GetExitGameChan returns the exit game signal channel
func (h *CompanionEventHandler) GetExitGameChan() <-chan struct{} {
	return h.exitGameChan
}

// IsLeaderInGame returns whether the leader is currently in-game (thread-safe)
func (h *CompanionEventHandler) IsLeaderInGame() bool {
	h.mu.RLock()
	defer h.mu.RUnlock()
	return h.leaderInGame
}

// GetCurrentGameName returns the current game name from heartbeat (thread-safe)
func (h *CompanionEventHandler) GetCurrentGameName() string {
	h.mu.RLock()
	defer h.mu.RUnlock()
	return h.currentGameName
}

// GetShrineLocations returns all shrine locations found by companions (thread-safe)
func (h *CompanionEventHandler) GetShrineLocations() []ShrineLocation {
	h.mu.RLock()
	defer h.mu.RUnlock()
	// Return a copy to prevent external modifications
	locations := make([]ShrineLocation, len(h.shrineLocations))
	copy(locations, h.shrineLocations)
	return locations
}

// ClearShrineLocations clears all stored shrine locations (thread-safe)
func (h *CompanionEventHandler) ClearShrineLocations() {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.shrineLocations = make([]ShrineLocation, 0)
}

// Handle processes companion-related events
func (h *CompanionEventHandler) Handle(ctx context.Context, e event.Event) error {

	switch evt := e.(type) {

	case event.LeaderGameHeartbeatEvent:
		// Only process if we're a companion and this is our leader
		if h.cfg.Companion.Enabled && !h.cfg.Companion.Leader {
			if h.cfg.Companion.LeaderName == "" || evt.Leader == h.cfg.Companion.LeaderName {
				h.mu.Lock()
				h.lastLeaderHeartbeat = time.Now()
				previousInGameState := h.leaderInGame
				h.leaderInGame = evt.InGame
				h.currentGameName = evt.GameName

				// Update companion game credentials from heartbeat (supports both bot-created and manual games)
				if evt.InGame && evt.GameName != "" {
					h.cfg.Companion.CompanionGameName = evt.GameName
					h.cfg.Companion.CompanionGamePassword = evt.GamePassword
				}
				h.mu.Unlock()

				// Log heartbeat reception with state changes
				if previousInGameState != evt.InGame {
					h.log.Info("Leader game state changed",
						slog.String("supervisor", h.supervisor),
						slog.String("leader", evt.Leader),
						slog.Bool("previousInGame", previousInGameState),
						slog.Bool("currentInGame", evt.InGame),
						slog.String("gameName", evt.GameName))
				} else {
					h.log.Debug("Leader heartbeat received",
						slog.String("supervisor", h.supervisor),
						slog.String("leader", evt.Leader),
						slog.Bool("inGame", evt.InGame),
						slog.String("gameName", evt.GameName))
				}

				// If leader exited, signal companion to exit too
				if !evt.InGame && h.currentGameName == h.cfg.Companion.CompanionGameName {
					h.log.Info("Leader exited game signal received",
						slog.String("supervisor", h.supervisor),
						slog.String("leader", evt.Leader),
						slog.String("game", evt.GameName),
						slog.String("companionGameName", h.cfg.Companion.CompanionGameName))

					// Signal to exit game (non-blocking)
					select {
					case h.exitGameChan <- struct{}{}:
						h.log.Debug("Exit signal sent to channel")
					default:
						h.log.Debug("Exit signal already in channel")
					}
				}
			}
		}

	case event.RequestCompanionJoinGameEvent:

		if h.cfg.Companion.Enabled && !h.cfg.Companion.Leader {

			// Check if the leader matches the one in our config or no leader set
			if h.cfg.Companion.LeaderName == "" || evt.Leader == h.cfg.Companion.LeaderName {
				h.log.Info("Companion join game event received", slog.String("supervisor", h.supervisor), slog.String("leader", evt.Leader), slog.String("name", evt.Name), slog.String("password", evt.Password))
				h.cfg.Companion.CompanionGameName = evt.Name
				h.cfg.Companion.CompanionGamePassword = evt.Password
			}
		}

	case event.ResetCompanionGameInfoEvent:

		// If this character is a companion (not a leader), clear game info
		if h.cfg.Companion.Enabled && !h.cfg.Companion.Leader {

			// Check if the leader matches the one in our config or no leader set.
			// Additional check for if LeaderName is the same as the character name for Manual join triggers
			if h.cfg.Companion.LeaderName == "" || evt.Leader == h.cfg.Companion.LeaderName || h.cfg.CharacterName == evt.Leader {
				h.log.Info("Companion reset game info event received", slog.String("supervisor", h.supervisor), slog.String("leader", evt.Leader))
				h.cfg.Companion.CompanionGameName = ""
				h.cfg.Companion.CompanionGamePassword = ""
			}
		}

	case event.CompanionFoundExpShrineEvent:
		// If this character is a leader, store the shrine location from companion
		if h.cfg.Companion.Enabled && h.cfg.Companion.Leader {
			h.mu.Lock()
			defer h.mu.Unlock()

			// Add the shrine location to the list
			shrineLocation := ShrineLocation{
				CompanionName: evt.CompanionName,
				AreaName:      evt.AreaName,
				AreaID:        evt.AreaID,
				X:             evt.X,
				Y:             evt.Y,
				Timestamp:     time.Now(),
			}
			h.shrineLocations = append(h.shrineLocations, shrineLocation)

			h.log.Info("Leader received Experience Shrine location from companion",
				slog.String("supervisor", h.supervisor),
				slog.String("companion", evt.CompanionName),
				slog.String("area", evt.AreaName),
				slog.Int("x", evt.X),
				slog.Int("y", evt.Y))
		}
	}

	return nil
}

// StartHeartbeatMonitor starts monitoring for leader heartbeat timeouts
func (h *CompanionEventHandler) StartHeartbeatMonitor(ctx context.Context) {
	if !h.cfg.Companion.Enabled || h.cfg.Companion.Leader {
		return // Only companions need to monitor heartbeats
	}

	go func() {
		ticker := time.NewTicker(10 * time.Second)
		defer ticker.Stop()

		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				h.mu.RLock()
				lastHeartbeat := h.lastLeaderHeartbeat
				leaderInGame := h.leaderInGame
				h.mu.RUnlock()

				// If we haven't received a heartbeat in 30 seconds and leader was in-game, assume crash
				if !lastHeartbeat.IsZero() && time.Since(lastHeartbeat) > 30*time.Second && leaderInGame {
					h.log.Warn("Leader heartbeat timeout - leader may have crashed",
						slog.String("supervisor", h.supervisor),
						slog.Duration("since_last_heartbeat", time.Since(lastHeartbeat)))

					h.mu.Lock()
					h.leaderInGame = false
					h.mu.Unlock()

					// Signal exit (non-blocking)
					select {
					case h.exitGameChan <- struct{}{}:
					default:
					}
				}
			}
		}
	}()
}
