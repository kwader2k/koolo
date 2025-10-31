package health

import (
	"log/slog"
	"time"
)

// PingMonitor tracks sustained high ping conditions
// Monitors ping over time and triggers actions when thresholds are exceeded
type PingMonitor struct {
	HighPingStart      time.Time
	HighPingThreshold  int           // Ping threshold in ms (typically 500-1000ms)
	HighPingSustained  time.Duration // How long high ping must persist before action (typically 10-30 seconds)
	LastPingCheck      time.Time
	CheckInterval      time.Duration // How often to check ping
	Enabled            bool
	Logger             *slog.Logger
	OnHighPingDetected func() // Callback when sustained high ping detected
}

// NewPingMonitor creates a new ping monitor with default settings
// Default: Quit after 10-30 seconds of ping > 500-1000ms
func NewPingMonitor(logger *slog.Logger, threshold int, sustainedDuration time.Duration) *PingMonitor {
	return &PingMonitor{
		HighPingThreshold: threshold,
		HighPingSustained: sustainedDuration,
		CheckInterval:     time.Second * 2, // Check every 2 seconds
		Enabled:           true,
		Logger:            logger,
	}
}

// CheckPing monitors for sustained high ping conditions
// Returns true if sustained high ping detected (should quit)
// Logic: Track start time when ping exceeds threshold, reset when it drops below
// If ping stays high for longer than sustained duration, trigger action
func (pm *PingMonitor) CheckPing(currentPing int) bool {
	if !pm.Enabled {
		return false
	}

	now := time.Now()

	// Only check at intervals to avoid spam
	if now.Sub(pm.LastPingCheck) < pm.CheckInterval {
		return false
	}
	pm.LastPingCheck = now

	// Sanity check - if ping reads as < 10ms, it's likely measurement error
	if currentPing < 10 {
		currentPing = 50
	}

	// Check if ping exceeds threshold
	if currentPing > pm.HighPingThreshold {
		// If this is first detection, start timer
		if pm.HighPingStart.IsZero() {
			pm.HighPingStart = now
			pm.Logger.Warn("High ping detected, starting monitor",
				slog.Int("ping", currentPing),
				slog.Int("threshold", pm.HighPingThreshold),
				slog.Duration("sustainedDuration", pm.HighPingSustained))
		} else {
			// Check if high ping has been sustained
			elapsed := now.Sub(pm.HighPingStart)
			if elapsed >= pm.HighPingSustained {
				pm.Logger.Error("Sustained high ping detected, triggering action",
					slog.Int("ping", currentPing),
					slog.Duration("duration", elapsed))

				// Call callback if set (typically supervisor stop)
				if pm.OnHighPingDetected != nil {
					pm.OnHighPingDetected()
				}

				return true
			} else {
				pm.Logger.Debug("High ping still sustained",
					slog.Int("ping", currentPing),
					slog.Duration("elapsed", elapsed),
					slog.Duration("remaining", pm.HighPingSustained-elapsed))
			}
		}
	} else {
		// Ping is normal, reset timer
		if !pm.HighPingStart.IsZero() {
			pm.Logger.Info("Ping returned to normal",
				slog.Int("ping", currentPing),
				slog.Duration("highPingDuration", now.Sub(pm.HighPingStart)))
			pm.HighPingStart = time.Time{} // Reset to zero value
		}
	}

	return false
}

// Reset clears the high ping tracking state
func (pm *PingMonitor) Reset() {
	pm.HighPingStart = time.Time{}
	pm.LastPingCheck = time.Time{}
}

// SetCallback sets the function to call when sustained high ping is detected
func (pm *PingMonitor) SetCallback(callback func()) {
	pm.OnHighPingDetected = callback
}

// Enable enables ping monitoring
func (pm *PingMonitor) Enable() {
	pm.Enabled = true
	pm.Logger.Info("Ping monitoring enabled",
		slog.Int("threshold", pm.HighPingThreshold),
		slog.Duration("sustainedDuration", pm.HighPingSustained))
}

// Disable disables ping monitoring
func (pm *PingMonitor) Disable() {
	pm.Enabled = false
	pm.Reset()
	pm.Logger.Info("Ping monitoring disabled")
}
