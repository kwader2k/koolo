package action

import (
	"fmt"
	"math"
	"sort"
	"time"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/pather"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
)

func GetDistanceFromClosestEnemy(pos data.Position, monsters data.Monsters) float64 {
	minDistance := math.MaxFloat64
	for _, monster := range monsters.Enemies() {
		if monster.Stats[stat.Life] <= 0 {
			continue
		}

		distance := pather.DistanceFromPoint(pos, monster.Position)
		if float64(distance) < minDistance {
			minDistance = float64(distance)
		}
	}
	return minDistance
}

func IsAnyEnemyAroundPlayer(radius int) (bool, data.Monster) {
	ctx := context.Get()
	for _, monster := range ctx.Data.Monsters.Enemies() {
		if monster.Stats[stat.Life] <= 0 {
			continue
		}

		distance := pather.DistanceFromPoint(ctx.Data.PlayerUnit.Position, monster.Position)
		if distance <= radius {
			return true, monster
		}
	}

	return false, data.Monster{}
}

// ShouldSwitchTarget checks if we've lost line of sight to the target for too long,
// or if there are no more alive monsters in the area.
// Returns true if we should abandon the current target and find a new one
// lastLineOfSight map tracks when we last had line of sight to each target
//
// How to Use in Other Characters:
//
//  1. Add to your character struct:
//     type YourCharacter struct {
//     BaseCharacter
//     lastLineOfSight map[data.UnitID]time.Time
//     }
//
//  2. Initialize the map in your KillMonsterSequence function:
//     if c.lastLineOfSight == nil {
//     c.lastLineOfSight = make(map[data.UnitID]time.Time)
//     }
//
//  3. Call this function after finding your target monster:
//     targetMonster, found := c.Data.Monsters.FindByID(id)
//     if !found {
//     return nil
//     }
//
//     // Check if we should switch targets due to lost line of sight
//     if action.ShouldSwitchTarget(id, targetMonster, c.lastLineOfSight) {
//     completedAttackLoops = 0
//     continue  // Skip to next loop iteration to find a new target
//     }
//
// The function will automatically track line of sight and return true when
// the target has been out of sight for more than 1 second, prompting a target switch.
// It will also return true if the target is dead or no alive monsters remain.
func ShouldSwitchTarget(targetID data.UnitID, targetMonster data.Monster, lastLineOfSight map[data.UnitID]time.Time) bool {
	const lostLineOfSightTimeout = time.Second * 1

	ctx := context.Get()

	// Check if the target monster is dead
	if targetMonster.Stats[stat.Life] <= 0 {
		ctx.Logger.Debug("Target monster is dead, switching targets", "targetID", targetID)
		delete(lastLineOfSight, targetID)
		return true
	}

	// Check if there are any alive monsters in the area
	hasAliveMonsters := false
	for _, monster := range ctx.Data.Monsters.Enemies() {
		if monster.Stats[stat.Life] > 0 {
			hasAliveMonsters = true
			break
		}
	}

	if !hasAliveMonsters {
		ctx.Logger.Debug("No alive monsters in area, ending combat")
		// Clear the entire map since combat is ending
		for k := range lastLineOfSight {
			delete(lastLineOfSight, k)
		}
		return true
	}

	// Check if we have line of sight to the target
	hasLineOfSight := ctx.PathFinder.LineOfSight(ctx.Data.PlayerUnit.Position, targetMonster.Position)

	if hasLineOfSight {
		// Update the last time we had line of sight
		lastLineOfSight[targetID] = time.Now()
		return false
	}

	// We don't have line of sight - check how long it's been
	lastSeen, exists := lastLineOfSight[targetID]
	if !exists {
		// First time checking this monster, record current time
		lastLineOfSight[targetID] = time.Now()
		return false
	}

	// If we've lost line of sight for too long, switch targets
	if time.Since(lastSeen) > lostLineOfSightTimeout {
		ctx.Logger.Debug("Lost line of sight to target for too long, switching targets",
			"targetID", targetID,
			"timeSinceLastSeen", time.Since(lastSeen))
		// Clean up old entry
		delete(lastLineOfSight, targetID)
		return true
	}

	return false
}

func FindSafePosition(targetMonster data.Monster, dangerDistance int, safeDistance int, minAttackDistance int, maxAttackDistance int) (data.Position, bool) {
	ctx := context.Get()
	playerPos := ctx.Data.PlayerUnit.Position

	// Define a stricter minimum safe distance from monsters
	minSafeMonsterDistance := int(math.Floor((float64(safeDistance) + float64(dangerDistance)) / 2))

	// Generate candidate positions in a circle around the player
	candidatePositions := []data.Position{}

	// First try positions in the opposite direction from the dangerous monster
	vectorX := playerPos.X - targetMonster.Position.X
	vectorY := playerPos.Y - targetMonster.Position.Y

	// Normalize the vector
	length := math.Sqrt(float64(vectorX*vectorX + vectorY*vectorY))
	if length > 0 {
		normalizedX := int(float64(vectorX) / length * float64(safeDistance))
		normalizedY := int(float64(vectorY) / length * float64(safeDistance))

		// Add positions in the opposite direction with some variation
		for offsetX := -3; offsetX <= 3; offsetX++ {
			for offsetY := -3; offsetY <= 3; offsetY++ {
				candidatePos := data.Position{
					X: playerPos.X + normalizedX + offsetX,
					Y: playerPos.Y + normalizedY + offsetY,
				}

				if ctx.Data.AreaData.IsWalkable(candidatePos) {
					candidatePositions = append(candidatePositions, candidatePos)
				}
			}
		}
	}

	// Generate positions in a circle with smaller angle increments for more candidates
	// Try positions in different directions around the player
	for angle := 0; angle < 360; angle += 5 {
		radians := float64(angle) * math.Pi / 180

		// Try multiple distances from the player
		for distance := minSafeMonsterDistance; distance <= safeDistance+5; distance += 2 {
			dx := int(math.Cos(radians) * float64(distance))
			dy := int(math.Sin(radians) * float64(distance))

			basePos := data.Position{
				X: playerPos.X + dx,
				Y: playerPos.Y + dy,
			}

			// Check a small area around the base position
			for offsetX := -1; offsetX <= 1; offsetX++ {
				for offsetY := -1; offsetY <= 1; offsetY++ {
					candidatePos := data.Position{
						X: basePos.X + offsetX,
						Y: basePos.Y + offsetY,
					}

					if ctx.Data.AreaData.IsWalkable(candidatePos) {
						candidatePositions = append(candidatePositions, candidatePos)
					}
				}
			}
		}
	}

	// No walkable positions found
	if len(candidatePositions) == 0 {
		return data.Position{}, false
	}

	// Evaluate all candidate positions
	type scoredPosition struct {
		pos   data.Position
		score float64
	}

	scoredPositions := []scoredPosition{}

	for _, pos := range candidatePositions {
		// Check if this position has line of sight to target
		if !ctx.PathFinder.LineOfSight(pos, targetMonster.Position) {
			continue
		}

		// Calculate minimum distance to any monster
		minMonsterDist := GetDistanceFromClosestEnemy(pos, ctx.Data.Monsters)

		// Strictly skip positions that are too close to monsters
		if minMonsterDist < float64(minSafeMonsterDistance) {
			continue
		}

		// Calculate distance to target monster
		targetDistance := pather.DistanceFromPoint(pos, targetMonster.Position)

		distanceFromPlayer := pather.DistanceFromPoint(pos, playerPos)

		// Calculate attack range score (highest when in optimal attack range)
		attackRangeScore := 0.0
		if targetDistance >= minAttackDistance && targetDistance <= maxAttackDistance {
			attackRangeScore = 10.0
		} else {
			// Penalize positions outside attack range
			attackRangeScore = -math.Abs(float64(targetDistance) - float64(minAttackDistance+maxAttackDistance)/2.0)
		}

		// Final score calculation - heavily weight monster distance for safety
		score := minMonsterDist*3.0 + attackRangeScore*2.0 - float64(distanceFromPlayer)*0.5

		// Extra bonus for positions that are very safe (far from monsters)
		if minMonsterDist > float64(dangerDistance) {
			score += 5.0
		}

		scoredPositions = append(scoredPositions, scoredPosition{
			pos:   pos,
			score: score,
		})
	}

	// Sort positions by score (highest first)
	sort.Slice(scoredPositions, func(i, j int) bool {
		return scoredPositions[i].score > scoredPositions[j].score
	})

	// Return the best position if we found any
	if len(scoredPositions) > 0 {
		ctx.Logger.Info(fmt.Sprintf("Found safe position with score %.2f at distance %.2f from nearest monster",
			scoredPositions[0].score, GetDistanceFromClosestEnemy(scoredPositions[0].pos, ctx.Data.Monsters)))
		return scoredPositions[0].pos, true
	}

	return data.Position{}, false
}
