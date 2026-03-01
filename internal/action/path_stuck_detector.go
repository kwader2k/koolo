package action

import (
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	// PathStuckTimeout is the duration after which a player is considered stuck on a path
	PathStuckTimeout = 5 * time.Second
	// BlacklistRadius is the radius around stuck positions that should be blacklisted
	BlacklistRadius = 5
)

// BlacklistedPoint represents a point that should be avoided during pathfinding
type BlacklistedPoint struct {
	Position data.Position
	Radius   int
}

// PathStuckDetector tracks whether a player is stuck while following a path
type PathStuckDetector struct {
	lastPosition      data.Position
	lastArea          area.ID
	stuckSince        time.Time
	blacklistedPoints []BlacklistedPoint
	enabled           bool
}

// NewPathStuckDetector creates a new path stuck detector instance
func NewPathStuckDetector() *PathStuckDetector {
	return &PathStuckDetector{
		enabled: true,
	}
}

// Update checks if the player has moved and updates the stuck timer
// Returns true if the player is stuck and path should be recalculated
func (psd *PathStuckDetector) Update(currentPos data.Position, currentArea area.ID) bool {
	if !psd.enabled {
		return false
	}

	// Check if we've entered a new area
	if psd.lastArea != currentArea {
		psd.Reset()
		psd.lastArea = currentArea
		psd.lastPosition = currentPos
		return false
	}

	// Check if player has moved
	if currentPos.X == psd.lastPosition.X && currentPos.Y == psd.lastPosition.Y {
		// Player hasn't moved
		if psd.stuckSince.IsZero() {
			// Start the stuck timer
			psd.stuckSince = time.Now()
		} else if time.Since(psd.stuckSince) >= PathStuckTimeout+
			(time.Duration(utils.GetCurrentPing())*time.Millisecond) {
			// Player has been stuck for too long
			return true
		}
	} else {
		// Player has moved, reset the stuck timer
		psd.stuckSince = time.Time{}
		psd.lastPosition = currentPos
	}

	return false
}

// OnStuckDetected is called when a player is detected as stuck
// It blacklists the current position and the next step along the path
func (psd *PathStuckDetector) OnStuckDetected(currentPos data.Position, nextPathStep data.Position) {
	// Blacklist current position
	psd.blacklistedPoints = append(psd.blacklistedPoints, BlacklistedPoint{
		Position: currentPos,
		Radius:   BlacklistRadius,
	})

	// Blacklist next step along the path if different from current position
	if nextPathStep.X != currentPos.X || nextPathStep.Y != currentPos.Y {
		psd.blacklistedPoints = append(psd.blacklistedPoints, BlacklistedPoint{
			Position: nextPathStep,
			Radius:   BlacklistRadius,
		})
	}

	// Reset the stuck timer
	psd.stuckSince = time.Time{}
}

// Reset clears all stuck detection state and blacklisted points
func (psd *PathStuckDetector) Reset() {
	psd.stuckSince = time.Time{}
	psd.blacklistedPoints = nil
	psd.lastPosition = data.Position{}
}

// IsPointBlacklisted checks if a given point is within any blacklisted radius
func (psd *PathStuckDetector) IsPointBlacklisted(pos data.Position) bool {
	for _, bp := range psd.blacklistedPoints {
		if pather.DistanceFromPoint(pos, bp.Position) <= bp.Radius {
			return true
		}
	}
	return false
}

// GetBlacklistedPoints returns a copy of all blacklisted points for use in pathfinding
func (psd *PathStuckDetector) GetBlacklistedPoints() []BlacklistedPoint {
	result := make([]BlacklistedPoint, len(psd.blacklistedPoints))
	copy(result, psd.blacklistedPoints)
	return result
}

// HasBlacklistedPoints returns true if there are any blacklisted points
func (psd *PathStuckDetector) HasBlacklistedPoints() bool {
	return len(psd.blacklistedPoints) > 0
}

// Enable turns on stuck detection
func (psd *PathStuckDetector) Enable() {
	psd.enabled = true
}

// Disable turns off stuck detection (useful for specific scenarios where stuck detection shouldn't trigger)
func (psd *PathStuckDetector) Disable() {
	psd.enabled = false
}

// IsEnabled returns whether stuck detection is currently enabled
func (psd *PathStuckDetector) IsEnabled() bool {
	return psd.enabled
}
