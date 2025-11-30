package step

import (
	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	maxEntranceDistance         = 8 // Increased from 6 to reduce false "too far" errors
	maxMoveRetries              = 3
	mousePositionMatchThreshold = 10 // Handle map data vs memory position variance (same as packet method)
)

// This defines areas with tricky entrances that may require alternative interaction positions
// For example, the A2 Sewers Level 3 entrance faces away from the direction the bot usually approaches from
// so we try different sides of the entrance if interaction fails repeatedly.
var alternativeEntranceAreas = map[area.ID]bool{
	area.SewersLevel3Act2: true,
}

func InteractEntrance(targetArea area.ID) error {
	ctx := context.Get()
	ctx.SetLastStep("InteractEntrance")

	// Force mouse-only for Act 3 Sewers (due to lever mechanism complexity)
	currentArea := ctx.Data.PlayerUnit.Area
	isSewerEntrance := (currentArea == area.SewersLevel1Act3 && targetArea == area.SewersLevel2Act3) ||
		(currentArea == area.KurastBazaar && targetArea == area.SewersLevel1Act3)

	if isSewerEntrance {
		ctx.Logger.Debug("Act 3 Sewers entrance detected, forcing mouse interaction for reliability")
		return InteractEntranceMouse(targetArea)
	}

	// Check if packet casting is enabled for entrance interaction
	if ctx.CharacterCfg.PacketCasting.UseForEntranceInteraction {
		ctx.Logger.Debug("Attempting entrance interaction via packet method")
		err := InteractEntrancePacket(targetArea)
		if err != nil {
			// Fallback to mouse interaction if packet method fails
			ctx.Logger.Warn("Packet entrance interaction failed, falling back to mouse method",
				"error", err.Error(),
				"targetArea", targetArea.Area().Name)
			return InteractEntranceMouse(targetArea)
		}
		return nil
	}

	// Use mouse-based interaction (original implementation)
	return InteractEntranceMouse(targetArea)
}

func InteractEntranceMouse(targetArea area.ID) error {
	maxInteractionAttempts := 21
	interactionAttempts := 1
	waitingForInteraction := false
	currentMouseCoords := data.Position{}
	lastRun := time.Time{}
	alternativeMoves := []data.Position{}
	alternativeIndex := 0

	// If we move the mouse to interact with an entrance, we will set this variable.
	var lastEntranceLevel data.Level

	ctx := context.Get()
	attempts := 0

	for {
		ctx.PauseIfNotPriority()

		if ctx.Data.AreaData.Area == targetArea && time.Since(lastRun) > time.Millisecond*500 && ctx.Data.AreaData.IsInside(ctx.Data.PlayerUnit.Position) {
			return nil
		}

		if interactionAttempts > maxInteractionAttempts {
			return fmt.Errorf("area %s [%d] could not be interacted", targetArea.Area().Name, targetArea)
		}

		if waitingForInteraction && time.Since(lastRun) < time.Millisecond*500 {
			continue
		}

		lastRun = time.Now()

		areaCombinations := func(a, b area.ID) bool {
			return (a == targetArea && ctx.Data.PlayerUnit.Area == b) || (a == ctx.Data.PlayerUnit.Area && targetArea == b)
		}

		isOverride := areaCombinations(area.Abaddon, area.FrigidHighlands) || areaCombinations(area.PitOfAcheron, area.ArreatPlateau) || areaCombinations(area.FrozenTundra, area.InfernalPit)

		// Find target level in adjacent levels
		var targetLevel data.Level
		if isOverride {
			// it's not mapped correctly, so we hardcode it here
			ctx.RefreshGameData()
			utils.Sleep(500)
			portal, found := ctx.Data.Objects.FindOne(object.PermanentTownPortal)
			if found {
				targetLevel = data.Level{
					Area:       targetArea,
					Position:   portal.Position,
					IsEntrance: true,
				}
			}
		} else {
			for _, l := range ctx.Data.AdjacentLevels {
				if l.Area == targetArea {
					targetLevel = l
					break
				}
			}
		}

		if targetLevel.Area == 0 {
			if attempts++; attempts > maxInteractionAttempts {
				return fmt.Errorf("area %s [%d] not found in adjacent levels", targetArea.Area().Name, targetArea)
			}
			continue // Area not found in adjacent levels, try again
		}

		// Find the corresponding entrance using fuzzy matching
		// Map data positions may differ from memory object positions by several units
		var nearestEntrance data.Level
		var found bool
		minDistance := mousePositionMatchThreshold + 1

		entranceLevels := ctx.Data.AdjacentLevels

		if isOverride {
			entranceLevels = append(entranceLevels, targetLevel)
		}

		for _, l := range entranceLevels {
			// It is possible to have multiple entrances to the same area (A2 sewers, A2 palace, etc)
			// Once we "select" an area and start to move the mouse to hover with it, we don't want
			// to change the area to the 2nd entrance in the same area on the next iteration.
			if l.Area == targetArea && (lastEntranceLevel == (data.Level{}) || lastEntranceLevel.Position == l.Position) {
				distance := pather.DistanceFromPoint(targetLevel.Position, l.Position)
				if distance <= mousePositionMatchThreshold {
					if !found || distance < minDistance {
						nearestEntrance = l
						minDistance = distance
						found = true
					}
				}
			}
		}

		if !found {
			continue // No entrance found within threshold, try again
		}

		l := nearestEntrance

		// Log when fuzzy matching helps (offset > 0)
		if minDistance > 0 {
			ctx.Logger.Debug("Found entrance via fuzzy matching",
				"positionOffset", minDistance,
				"targetArea", targetArea.Area().Name)
		}

		// Prepare alternative approach positions to try different sides of the entrance if interaction keeps failing.
		if len(alternativeMoves) == 0 && alternativeEntranceAreas[targetArea] {
			sign := func(n int) int {
				switch {
				case n > 0:
					return 1
				case n < 0:
					return -1
				default:
					return 0
				}
			}
			dir := data.Position{
				X: sign(l.Position.X - ctx.Data.PlayerUnit.Position.X),
				Y: sign(l.Position.Y - ctx.Data.PlayerUnit.Position.Y),
			}
			if dir.X == 0 && dir.Y == 0 {
				dir.Y = 1 // default up if we're exactly on the entrance
			}
			// Opposite side, then rotate around the entrance to sample different angles
			alternativeMoves = []data.Position{
				{X: l.Position.X + dir.X*3, Y: l.Position.Y + dir.Y*3}, // opposite side
				{X: l.Position.X - dir.Y*3, Y: l.Position.Y + dir.X*3}, // 90 deg from new position
				{X: l.Position.X - dir.X*3, Y: l.Position.Y - dir.Y*3}, // opposite of starting side
				{X: l.Position.X + dir.Y*3, Y: l.Position.Y - dir.X*3}, // opposite of the 90 deg side
			}
		}

		// Every few failed attempts, reposition to an alternative side of the entrance.
		if alternativeEntranceAreas[targetArea] && alternativeIndex < len(alternativeMoves) && interactionAttempts%5 == 0 {
			targetPos := alternativeMoves[alternativeIndex]
			ctx.Logger.Debug("Repositioning for entrance interaction",
				"targetArea", targetArea.Area().Name,
				"attempt", interactionAttempts,
				"targetPos", targetPos)
			if err := MoveTo(targetPos, WithDistanceToFinish(2)); err != nil {
				ctx.Logger.Warn("Failed repositioning for entrance interaction", "error", err)
			}
			alternativeIndex++

		}

		distance := ctx.PathFinder.DistanceFromMe(l.Position)
		if distance > maxEntranceDistance {
			// Try to move closer with retries - stop 2 units away for better interaction range
			// Use escalating retry delays
			for retry := 0; retry < maxMoveRetries; retry++ {
				if err := MoveTo(l.Position, WithDistanceToFinish(2)); err != nil {
					// If MoveTo fails, try direct movement
					screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(
						l.Position.X-2,
						l.Position.Y-2,
					)
					ctx.HID.Click(game.LeftButton, screenX, screenY)
					// Escalating retry delay: increases with each attempt
					utils.RetrySleep(retry, float64(ctx.Data.Game.Ping), 800)
					ctx.RefreshGameData()
				}

				// Check if we're close enough now
				newDistance := ctx.PathFinder.DistanceFromMe(l.Position)
				if newDistance <= maxEntranceDistance {
					break
				}

				if retry == maxMoveRetries-1 {
					return fmt.Errorf("entrance too far away (distance: %d)", distance)
				}
			}
		}

		if l.IsEntrance {
			lx, ly := ctx.PathFinder.GameCoordsToScreenCords(l.Position.X-1, l.Position.Y-1)
			if ctx.Data.HoverData.UnitType == 5 || ctx.Data.HoverData.UnitType == 2 && ctx.Data.HoverData.IsHovered {
				ctx.HID.Click(game.LeftButton, currentMouseCoords.X, currentMouseCoords.Y)
				waitingForInteraction = true
				utils.PingSleep(utils.Light, 200) // Light operation: Wait for click registration
			}

			x, y := utils.Spiral(interactionAttempts)
			if ctx.Data.AreaData.Area == area.CanyonOfTheMagi {
				x = x * 5
				y = y * 5
			}
			currentMouseCoords = data.Position{X: lx + x, Y: ly + y}
			ctx.HID.MovePointer(lx+x, ly+y)
			interactionAttempts++
			utils.PingSleep(utils.Light, 100) // Light operation: Mouse movement delay

			lastEntranceLevel = l

			continue
		}

		return fmt.Errorf("area %s [%d] is not an entrance", targetArea.Area().Name, targetArea)
	}
}
