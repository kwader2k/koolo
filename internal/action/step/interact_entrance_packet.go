package step

import (
	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/pather"
)

const (
	// Maximum attempts for packet-based entrance interaction
	maxPacketEntranceAttempts = 3

	// Timeout waiting for area transition after packet send
	entranceTransitionTimeout = 3 * time.Second

	// Distance threshold for packet interaction
	packetEntranceDistance = 7

	// Fuzzy position matching threshold to handle map data vs memory position variance
	positionMatchThreshold = 10
)

// InteractEntrancePacket attempts to interact with an entrance using D2GS packets
// instead of mouse simulation. This is faster and more reliable.
//
// Packet structure (5 bytes total):
// [0x40] [UnitID byte 1] [UnitID byte 2] [UnitID byte 3] [UnitID byte 4]
//
//	^-- Packet ID          ^-- UnitID as uint32 little-endian
//
// Returns nil on success, error if packet interaction fails completely.
func InteractEntrancePacket(targetArea area.ID) error {
	ctx := context.Get()
	ctx.SetLastStep("InteractEntrancePacket")

	// Find the entrance from adjacent levels that matches our target area
	var targetEntrance data.Entrance
	var targetLevel data.Level
	var found bool

	// First, find the level information from AdjacentLevels
	for _, level := range ctx.Data.AdjacentLevels {
		if level.Area == targetArea && level.IsEntrance {
			targetLevel = level
			break
		}
	}

	if targetLevel.Area == 0 {
		return fmt.Errorf("target area %s not found in adjacent levels", targetArea.Area().Name)
	}

	// Find the corresponding entrance from the Entrances list using fuzzy matching
	// Map data positions may differ from memory object positions by several units
	var nearestEntrance data.Entrance
	minDistance := positionMatchThreshold + 1

	for _, ent := range ctx.Data.Entrances {
		distance := pather.DistanceFromPoint(targetLevel.Position, ent.Position)
		if distance < minDistance && distance <= positionMatchThreshold {
			nearestEntrance = ent
			minDistance = distance
		}
	}

	if minDistance > positionMatchThreshold {
		return fmt.Errorf("entrance to %s not found within %d units (nearest: %d units)",
			targetArea.Area().Name, positionMatchThreshold, minDistance)
	}

	targetEntrance = nearestEntrance
	found = true

	ctx.Logger.Debug("Found entrance via fuzzy matching",
		"positionOffset", minDistance,
		"mapX", targetLevel.Position.X,
		"mapY", targetLevel.Position.Y,
		"memoryX", targetEntrance.Position.X,
		"memoryY", targetEntrance.Position.Y)

	// Check distance first - must be within range
	distance := ctx.PathFinder.DistanceFromMe(targetEntrance.Position)
	if distance > packetEntranceDistance {
		ctx.Logger.Debug("Entrance too far, moving closer",
			"currentDistance", distance,
			"maxDistance", packetEntranceDistance)

		// Move closer to entrance - stop 2 units away to ensure we're within interaction range
		if err := MoveTo(targetEntrance.Position, WithDistanceToFinish(2)); err != nil {
			return fmt.Errorf("failed to move to entrance: %w", err)
		}

		// Refresh and re-check distance
		ctx.RefreshGameData()
		distance = ctx.PathFinder.DistanceFromMe(targetEntrance.Position)
		if distance > packetEntranceDistance {
			return fmt.Errorf("still too far from entrance after move (distance: %d)", distance)
		}

		// Re-find the entrance after moving using fuzzy matching
		found = false
		minDistance = positionMatchThreshold + 1

		for _, ent := range ctx.Data.Entrances {
			distance := pather.DistanceFromPoint(targetLevel.Position, ent.Position)
			if distance < minDistance && distance <= positionMatchThreshold {
				targetEntrance = ent
				minDistance = distance
				found = true
			}
		}

		if !found {
			return fmt.Errorf("entrance disappeared after moving closer")
		}
	}

	// Final distance validation before packet send
	finalDistance := ctx.PathFinder.DistanceFromMe(targetEntrance.Position)
	if finalDistance > packetEntranceDistance {
		return fmt.Errorf("entrance out of range before packet send (distance: %d, max: %d)",
			finalDistance, packetEntranceDistance)
	}

	// Log entrance details for debugging
	ctx.Logger.Debug("Found entrance for packet interaction",
		"targetArea", targetArea.Area().Name,
		"entranceID", targetEntrance.ID,
		"entranceName", targetEntrance.Name,
		"position", fmt.Sprintf("X:%d Y:%d", targetEntrance.Position.X, targetEntrance.Position.Y),
		"distance", finalDistance)
	ctx.Logger.Info("Sending entrance interaction packet",
		"targetArea", targetArea.Area().Name,
		"entranceID", targetEntrance.ID,
		"distance", distance)

	// Attempt packet send with retries
	var lastErr error
	for attempt := 1; attempt <= maxPacketEntranceAttempts; attempt++ {
		// Send the packet using PacketSender
		if err := ctx.PacketSender.InteractWithEntrance(targetEntrance); err != nil {
			ctx.Logger.Warn("Entrance packet send failed",
				"attempt", attempt,
				"error", err)
			lastErr = err
			time.Sleep(200 * time.Millisecond)
			continue
		}

		ctx.Logger.Debug("Entrance packet sent successfully", "attempt", attempt)

		// Wait for area transition
		if waitForAreaTransition(ctx, targetArea, entranceTransitionTimeout) {
			ctx.Logger.Info("Entrance interaction successful via packet",
				"targetArea", targetArea.Area().Name,
				"attempts", attempt)
			return nil
		}

		ctx.Logger.Debug("Area transition not detected after packet send", "attempt", attempt)
		lastErr = fmt.Errorf("area transition timeout")

		// Refresh game data and retry
		time.Sleep(300 * time.Millisecond)
		ctx.RefreshGameData()

		// Re-check if we're somehow already in the target area
		if ctx.Data.AreaData.Area == targetArea {
			ctx.Logger.Info("Successfully transitioned to target area", "targetArea", targetArea.Area().Name)
			return nil
		}
	}

	return fmt.Errorf("entrance packet interaction failed after %d attempts: %w", maxPacketEntranceAttempts, lastErr)
}

// waitForAreaTransition polls the game data waiting for area transition to complete
// Returns true if transition succeeded within timeout, false otherwise
func waitForAreaTransition(ctx *context.Status, targetArea area.ID, timeout time.Duration) bool {
	// Wait 300ms before checking to allow server to process the transition
	time.Sleep(300 * time.Millisecond)

	deadline := time.Now().Add(timeout)
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

	for time.Now().Before(deadline) {
		<-ticker.C
		ctx.RefreshGameData()

		// Check if we're in the target area
		if ctx.Data.AreaData.Area == targetArea {
			// Additional verification - ensure we're inside the area bounds
			if ctx.Data.AreaData.IsInside(ctx.Data.PlayerUnit.Position) {
				return true
			}
		}
	}

	return false
}

// TryInteractEntrancePacket is a safe wrapper that attempts packet interaction
// but returns a specific error if packet method should be skipped in favor of mouse method.
// This allows for graceful fallback in the main InteractEntrance function.
