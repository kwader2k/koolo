package pather

import (
	"fmt"
	"math"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/pather/astar"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// MaxWaypointDistance is the maximum Chebyshev distance (in tiles) allowed between consecutive
// waypoints in a path. This ensures that every point along the path is reachable.
const MaxWaypointDistance = 7

type PathFinder struct {
	gr           *game.MemoryReader
	data         *game.Data
	hid          *game.HID
	cfg          *config.CharacterCfg
	packetSender *game.PacketSender
	// astarBuffers are reusable A* buffers to avoid allocations. Thread-safe without
	// synchronization because pathfinding is only called from the PriorityNormal goroutine
	// (main bot loop). Background goroutines (data refresh, health check) do not perform pathfinding.
	astarBuffers *astar.AStarBuffers
}

func NewPathFinder(gr *game.MemoryReader, data *game.Data, hid *game.HID, cfg *config.CharacterCfg) *PathFinder {
	return &PathFinder{
		gr:           gr,
		data:         data,
		hid:          hid,
		cfg:          cfg,
		astarBuffers: &astar.AStarBuffers{},
	}
}

func (pf *PathFinder) SetPacketSender(ps *game.PacketSender) {
	pf.packetSender = ps
}

// DetectGapAndGetTeleportPositions detects if there's a gap of unwalkable area between from and to,
// and returns the optimal crossing point where the gap width is <= 7 units.
// Searches both along the direct path and perpendicular to find the narrowest crossing.
// Returns (beforeGap, afterGap, foundGap)
func (pf *PathFinder) DetectGapAndGetTeleportPositions(from, to data.Position) (data.Position, data.Position, bool) {
	a := pf.data.AreaData
	maxTeleportGap := 5 // Maximum gap width that can be efficiently teleported
	maxGapWidth := 8    // Hard limit on gap width to consider for teleporting

	dx := to.X - from.X
	dy := to.Y - from.Y
	steps := int(math.Max(math.Abs(float64(dx)), math.Abs(float64(dy))))
	if steps == 0 {
		return data.Position{}, data.Position{}, false
	}

	stepX := float64(dx) / float64(steps)
	stepY := float64(dy) / float64(steps)

	// Track state as we move along the line and find all gaps
	type gapInfo struct {
		startIdx int
		endIdx   int
		midIdx   int
		width    int
		startPos data.Position
		endPos   data.Position
	}

	var gaps []gapInfo
	inWalkable := a.IsWalkable(from)
	gapStart := -1

	for i := 0; i <= steps; i++ {
		x := from.X + int(float64(i)*stepX)
		y := from.Y + int(float64(i)*stepY)
		pos := data.Position{X: x, Y: y}

		isWalkable := a.IsWalkable(pos)
		isTeleportable := a.CanTeleportTo(pos)

		// If we're in a walkable area and hit unwalkable (but teleportable), mark gap start
		if inWalkable && !isWalkable && isTeleportable {
			gapStart = i
			inWalkable = false
		}

		// If we were in a gap and now we're back to walkable, record the gap
		if !inWalkable && isWalkable && gapStart >= 0 {
			gapWidth := i - gapStart
			midIdx := (gapStart + i) / 2
			gaps = append(gaps, gapInfo{
				startIdx: gapStart,
				endIdx:   i,
				midIdx:   midIdx,
				width:    gapWidth,
				startPos: data.Position{
					X: from.X + int(float64(gapStart)*stepX),
					Y: from.Y + int(float64(gapStart)*stepY),
				},
				endPos: data.Position{
					X: from.X + int(float64(i)*stepX),
					Y: from.Y + int(float64(i)*stepY),
				},
			})
			gapStart = -1
			inWalkable = true
		}

		// If we hit a non-teleportable obstacle, no valid gap
		if !isWalkable && !isTeleportable {
			return data.Position{}, data.Position{}, false
		}
	}

	if len(gaps) == 0 {
		return data.Position{}, data.Position{}, false
	}

	// For each gap, search perpendicular to find narrower crossing points
	type crossingPoint struct {
		beforePos data.Position
		afterPos  data.Position
		width     int
	}

	var bestCrossing *crossingPoint

	for _, gap := range gaps {
		// Skip gaps that are too large to teleport across
		if gap.width > maxGapWidth {
			continue
		}

		// Start with the direct crossing
		if bestCrossing == nil || gap.width < bestCrossing.width {
			beforeIdx := gap.startIdx - 1
			if beforeIdx < 0 {
				beforeIdx = 0
			}
			bestCrossing = &crossingPoint{
				beforePos: data.Position{
					X: from.X + int(float64(beforeIdx)*stepX),
					Y: from.Y + int(float64(beforeIdx)*stepY),
				},
				afterPos: gap.endPos,
				width:    gap.width,
			}
		}

		// If already within acceptable range, no need to search further
		if gap.width <= maxTeleportGap {
			continue
		}

		// Search perpendicular to the main direction to find narrower crossings
		// Calculate perpendicular direction (rotate 90 degrees) from the integer gap vector
		dx := gap.endPos.X - gap.startPos.X
		dy := gap.endPos.Y - gap.startPos.Y
		perpX := -dy
		perpY := dx
		// Normalize to unit steps to avoid large jumps
		if perpX > 0 {
			perpX = 1
		} else if perpX < 0 {
			perpX = -1
		}
		if perpY > 0 {
			perpY = 1
		} else if perpY < 0 {
			perpY = -1
		}

		// Search up to 20 units perpendicular in both directions
		for offset := 1; offset <= 20; offset++ {
			for _, dir := range []int{-1, 1} {
				offsetX := perpX * offset * dir
				offsetY := perpY * offset * dir

				// Check crossing at this perpendicular offset
				checkStart := data.Position{
					X: gap.startPos.X + offsetX,
					Y: gap.startPos.Y + offsetY,
				}
				checkEnd := data.Position{
					X: gap.endPos.X + offsetX,
					Y: gap.endPos.Y + offsetY,
				}

				// Verify both positions are walkable and inside area
				if !a.IsInside(checkStart) || !a.IsInside(checkEnd) {
					continue
				}
				if !a.IsWalkable(checkStart) || !a.IsWalkable(checkEnd) {
					continue
				}

				// Measure the gap width at this position
				gapWidthHere := int(math.Sqrt(float64((checkEnd.X-checkStart.X)*(checkEnd.X-checkStart.X) +
					(checkEnd.Y-checkStart.Y)*(checkEnd.Y-checkStart.Y))))

				// Skip if this crossing exceeds max gap width
				if gapWidthHere > maxGapWidth {
					continue
				}

				// If this crossing is better (narrower), use it
				if gapWidthHere < bestCrossing.width {
					bestCrossing = &crossingPoint{
						beforePos: checkStart,
						afterPos:  checkEnd,
						width:     gapWidthHere,
					}

					// If we found one within acceptable range, stop searching
					if gapWidthHere <= maxTeleportGap {
						goto foundOptimal
					}
				}
			}
		}
	}

foundOptimal:
	if bestCrossing != nil {
		// Final sanity check: ensure the gap width doesn't exceed max
		if bestCrossing.width > maxGapWidth {
			return data.Position{}, data.Position{}, false
		}
		return bestCrossing.beforePos, bestCrossing.afterPos, true
	}

	return data.Position{}, data.Position{}, false
}

func (pf *PathFinder) GetPathFrom(from, to data.Position) (Path, int, bool) {
	a := pf.data.AreaData
	canTeleport := pf.data.CanTeleport()

	// We don't want to modify the original grid
	grid := a.Grid.Copy()

	// Special handling for Arcane Sanctuary (to allow pathing with platforms)
	if pf.data.PlayerUnit.Area == area.ArcaneSanctuary && pf.data.CanTeleport() {
		// Make all non-walkable tiles into low priority tiles for teleport pathing
		for y := 0; y < grid.Height; y++ {
			for x := 0; x < grid.Width; x++ {
				if grid.Get(x, y) == game.CollisionTypeNonWalkable {
					grid.Set(x, y, game.CollisionTypeLowPriority)
				}
			}
		}
	}
	// Lut Gholein map is a bit bugged, we should close this fake path to avoid pathing issues
	if a.Area == area.LutGholein {
		if 210 < a.Grid.Width && 13 < a.Grid.Height {
			a.Grid.Set(210, 13, game.CollisionTypeNonWalkable)
		}
	}

	if !a.IsInside(to) {
		expandedGrid, err := pf.mergeGrids(to, canTeleport)
		if err != nil {
			return nil, 0, false
		}
		grid = expandedGrid
	}

	if !grid.IsWalkable(to) {
		if walkableTo, found := pf.findNearbyWalkablePositionInGrid(grid, to); found {
			to = walkableTo
		}
	}
	from = grid.RelativePosition(from)
	to = grid.RelativePosition(to)

	// Add objects to the collision grid as obstacles
	for _, o := range pf.data.AreaData.Objects {
		if !grid.IsWalkable(o.Position) {
			continue
		}
		relativePos := grid.RelativePosition(o.Position)
		if relativePos.X < 0 || relativePos.X >= grid.Width || relativePos.Y < 0 || relativePos.Y >= grid.Height {
			continue
		}
		grid.Set(relativePos.X, relativePos.Y, game.CollisionTypeObject)
		for i := -2; i <= 2; i++ {
			for j := -2; j <= 2; j++ {
				if i == 0 && j == 0 {
					continue
				}
				ny, nx := relativePos.Y+i, relativePos.X+j
				if ny < 0 || ny >= grid.Height || nx < 0 || nx >= grid.Width {
					continue
				}
				if grid.Get(nx, ny) == game.CollisionTypeWalkable {
					grid.Set(nx, ny, game.CollisionTypeLowPriority)
				}
			}
		}
	}

	// Add monsters to the collision grid as obstacles
	for _, m := range pf.data.Monsters {
		if !grid.IsWalkable(m.Position) {
			continue
		}
		relativePos := grid.RelativePosition(m.Position)
		if relativePos.X < 0 || relativePos.X >= grid.Width || relativePos.Y < 0 || relativePos.Y >= grid.Height {
			continue
		}
		grid.Set(relativePos.X, relativePos.Y, game.CollisionTypeMonster)
	}

	// set barricade tower as non walkable in act 5
	if a.Area == area.FrigidHighlands || a.Area == area.FrozenTundra || a.Area == area.ArreatPlateau {
		towerCount := 0
		for _, n := range pf.data.NPCs {
			if n.ID != npc.BarricadeTower {
				continue
			}
			if len(n.Positions) == 0 {
				continue
			}
			npcPos := n.Positions[0]
			relativePos := grid.RelativePosition(npcPos)
			towerCount++

			// Set a 5x5 area around the barricade tower as non-walkable
			blockedCells := 0
			for dy := -2; dy <= 2; dy++ {
				for dx := -2; dx <= 2; dx++ {
					towerY := relativePos.Y + dy
					towerX := relativePos.X + dx

					// Bounds checking to prevent array index out of bounds
					if towerY >= 0 && towerY < grid.Height &&
						towerX >= 0 && towerX < grid.Width {
						grid.Set(towerX, towerY, game.CollisionTypeNonWalkable)
						blockedCells++
					}
				}
			}
		}
	}

	path, distance, found := astar.CalculatePath(grid, from, to, canTeleport, pf.astarBuffers, a.Area, false)

	if config.Koolo.Debug.RenderMap {
		pf.renderMap(grid, from, to, path)
	}

	if pf.data.PlayerUnit.Area != area.ArcaneSanctuary && pf.data.CanTeleport() && found && len(path) > 1 {
		// Remove or replace short teleports (<= 3) that try to teleport into invalid map terrain (not accessible)
		newPath := make(Path, 0, len(path))
		newPath = append(newPath, path[0])

		isValidTeleportDest := func(pos data.Position) bool {
			if pos.X < 0 || pos.Y < 0 || pos.X >= grid.Width || pos.Y >= grid.Height {
				return false
			}
			ct := grid.Get(pos.X, pos.Y)
			return ct == game.CollisionTypeWalkable || ct == game.CollisionTypeTeleportOver
		}

		// Helper: Bresenham's line algorithm to find last valid tile before invalid
		findLastValid := func(from, to data.Position) data.Position {
			x0, y0 := from.X, from.Y
			x1, y1 := to.X, to.Y
			dx := utils.Abs(x1 - x0)
			dy := utils.Abs(y1 - y0)
			sx := -1
			if x0 < x1 {
				sx = 1
			}
			sy := -1
			if y0 < y1 {
				sy = 1
			}
			err := dx - dy
			lastValid := from
			for {
				pos := data.Position{X: x0, Y: y0}
				if !isValidTeleportDest(pos) {
					break
				}
				lastValid = pos
				if x0 == x1 && y0 == y1 {
					break
				}
				e2 := 2 * err
				if e2 > -dy {
					err -= dy
					x0 += sx
				}
				if e2 < dx {
					err += dx
					y0 += sy
				}
			}
			return lastValid
		}

		for i := 1; i < len(path); i++ {
			p1 := path[i-1]
			p2 := path[i]

			dx := p2.X - p1.X
			if dx < 0 {
				dx = -dx
			}
			dy := p2.Y - p1.Y
			if dy < 0 {
				dy = -dy
			}
			cheb := dx
			if dy > cheb {
				cheb = dy
			}

			if cheb <= 3 {
				if !isValidTeleportDest(p2) {
					lastValid := findLastValid(p1, p2)
					// Only add if not the same as p1 and not already in path
					if (lastValid.X != p1.X || lastValid.Y != p1.Y) && (len(newPath) == 0 || lastValid != newPath[len(newPath)-1]) {
						// Ensure lastValid is in the correct direction (between p1 and p2)
						d1 := (p2.X-p1.X)*(lastValid.X-p1.X) + (p2.Y-p1.Y)*(lastValid.Y-p1.Y)
						d2 := (p2.X-p1.X)*(p2.X-p1.X) + (p2.Y-p1.Y)*(p2.Y-p1.Y)
						// d1 >= 0 ensures lastValid is not behind p1, d1 <= d2 ensures it's not past p2
						if d1 >= 0 && d1 <= d2 {
							// Also ensure lastValid is closer to p2 than p1 is
							distLastValidP2 := (p2.X-lastValid.X)*(p2.X-lastValid.X) + (p2.Y-lastValid.Y)*(p2.Y-lastValid.Y)
							if distLastValidP2 < d2 {
								newPath = append(newPath, lastValid)
							}
						}
					}
					continue // skip original invalid step
				}
			}

			newPath = append(newPath, p2)
		}

		path = newPath
	}

	// Ensure every point along the path is reachable (max 7 points away)
	path = ensurePathReachability(path)

	return path, distance, found
}

// ensurePathReachability validates that every consecutive pair of points in the path
// is at most MaxWaypointDistance tiles away (using Chebyshev distance). If any points exceed this distance,
// intermediate waypoints are inserted.
func ensurePathReachability(path Path) Path {
	if len(path) <= 1 {
		return path
	}

	validatedPath := make(Path, 0, len(path)*2)
	validatedPath = append(validatedPath, path[0])

	for i := 1; i < len(path); i++ {
		prev := validatedPath[len(validatedPath)-1]
		curr := path[i]

		// Calculate raw distances
		rawDx := curr.X - prev.X
		rawDy := curr.Y - prev.Y

		// Get absolute values and direction
		dx := rawDx
		if dx < 0 {
			dx = -dx
		}
		dy := rawDy
		if dy < 0 {
			dy = -dy
		}

		// Use Chebyshev distance (maximum of absolute differences)
		dist := dx
		if dy > dist {
			dist = dy
		}

		// If distance exceeds max, insert intermediate waypoints
		if dist > MaxWaypointDistance {
			// Calculate step direction (normalized to -1, 0, or 1)
			stepX := 0
			stepY := 0
			if rawDx > 0 {
				stepX = 1
			} else if rawDx < 0 {
				stepX = -1
			}
			if rawDy > 0 {
				stepY = 1
			} else if rawDy < 0 {
				stepY = -1
			}

			// Add waypoints moving in MaxWaypointDistance steps until we can reach the destination
			current := prev
			for current != curr {
				// Calculate next waypoint (up to MaxWaypointDistance tiles away)
				nextX := current.X + stepX*MaxWaypointDistance
				nextY := current.Y + stepY*MaxWaypointDistance
				nextWaypoint := data.Position{X: nextX, Y: nextY}

				// Check distance from next waypoint to target
				nextToTargetDx := curr.X - nextWaypoint.X
				if nextToTargetDx < 0 {
					nextToTargetDx = -nextToTargetDx
				}
				nextToTargetDy := curr.Y - nextWaypoint.Y
				if nextToTargetDy < 0 {
					nextToTargetDy = -nextToTargetDy
				}
				nextToTargetDist := nextToTargetDx
				if nextToTargetDy > nextToTargetDist {
					nextToTargetDist = nextToTargetDy
				}

				if nextToTargetDist <= MaxWaypointDistance {
					// From nextWaypoint, we can reach the target directly
					// Add the nextWaypoint but DON'T set current=curr yet,
					// we'll add curr after exiting the loop
					validatedPath = append(validatedPath, nextWaypoint)
					break
				} else {
					// nextWaypoint is still too far, add it as intermediate and continue
					validatedPath = append(validatedPath, nextWaypoint)
					current = nextWaypoint
				}
			}
			// Always add the final target point
			validatedPath = append(validatedPath, curr)
		} else {
			// Distance is acceptable, add the point directly
			validatedPath = append(validatedPath, curr)
		}
	}

	return validatedPath
}

func (pf *PathFinder) mergeGrids(to data.Position, canTeleport bool) (*game.Grid, error) {
	for _, a := range pf.data.AreaData.AdjacentLevels {
		destination, exists := pf.data.Areas[a.Area]
		if !exists || destination.Grid == nil || destination.Grid.CollisionGrid == nil {
			continue
		}
		if destination.IsInside(to) {
			origin := pf.data.AreaData

			endX1 := origin.OffsetX + origin.Grid.Width
			endY1 := origin.OffsetY + origin.Grid.Height
			endX2 := destination.OffsetX + destination.Grid.Width
			endY2 := destination.OffsetY + destination.Grid.Height

			minX := min(origin.OffsetX, destination.OffsetX)
			minY := min(origin.OffsetY, destination.OffsetY)
			maxX := max(endX1, endX2)
			maxY := max(endY1, endY2)

			width := maxX - minX
			height := maxY - minY

			// Use 2D array for NewGrid compatibility (it converts to flat internally)
			resultGrid := make([][]game.CollisionType, height)
			for i := range resultGrid {
				resultGrid[i] = make([]game.CollisionType, width)
			}

			// Let's copy both grids into the result grid
			copyGridFlat(resultGrid, origin.Grid, origin.OffsetX-minX, origin.OffsetY-minY)
			copyGridFlat(resultGrid, destination.Grid, destination.OffsetX-minX, destination.OffsetY-minY)

			grid := game.NewGrid(resultGrid, minX, minY, canTeleport)

			return grid, nil
		}
	}

	return nil, fmt.Errorf("destination grid not found")
}

// copyGridFlat copies from a flat Grid to a 2D destination array
func copyGridFlat(dest [][]game.CollisionType, src *game.Grid, offsetX, offsetY int) {
	for y := 0; y < src.Height; y++ {
		for x := 0; x < src.Width; x++ {
			dest[offsetY+y][offsetX+x] = src.Get(x, y)
		}
	}
}

func (pf *PathFinder) GetClosestWalkablePath(dest data.Position) (Path, int, bool) {
	return pf.GetClosestWalkablePathFrom(pf.data.PlayerUnit.Position, dest)
}

func (pf *PathFinder) GetClosestWalkablePathFrom(from, dest data.Position) (Path, int, bool) {
	a := pf.data.AreaData
	if a.IsWalkable(dest) || !a.IsInside(dest) {
		path, distance, found := pf.GetPath(dest)
		if found {
			return path, distance, found
		}
	}

	maxRange := 20
	step := 4
	dst := 1

	for dst < maxRange {
		for i := -dst; i < dst; i += 1 {
			for j := -dst; j < dst; j += 1 {
				if math.Abs(float64(i)) >= math.Abs(float64(dst)) || math.Abs(float64(j)) >= math.Abs(float64(dst)) {
					cgY := dest.Y - pf.data.AreaOrigin.Y + j
					cgX := dest.X - pf.data.AreaOrigin.X + i
					if cgX >= 0 && cgY >= 0 && a.Height > cgY && a.Width > cgX && a.Grid.Get(cgX, cgY) == game.CollisionTypeWalkable {
						return pf.GetPathFrom(from, data.Position{
							X: dest.X + i,
							Y: dest.Y + j,
						})
					}
				}
			}
		}
		dst += step
	}

	return nil, 0, false
}

func (pf *PathFinder) findNearbyWalkablePositionInGrid(grid *game.Grid, target data.Position) (data.Position, bool) {
	// Search in expanding squares around the target position
	// Increased radius from 3 to 7 to better handle wall-stuck scenarios
	for radius := 1; radius <= 7; radius++ {
		for x := -radius; x <= radius; x++ {
			for y := -radius; y <= radius; y++ {
				if x == 0 && y == 0 {
					continue
				}
				pos := data.Position{X: target.X + x, Y: target.Y + y}
				if (*grid).IsWalkable(pos) {
					return pos, true
				}
			}
		}
	}
	return data.Position{}, false
}

func (pf *PathFinder) findNearbyWalkablePosition(target data.Position) (data.Position, bool) {
	return pf.findNearbyWalkablePositionInGrid(pf.data.AreaData.Grid, target)
}

// RequiresWallHuggingToReach checks if the destination requires wall-hugging to reach it
// (i.e., moving close to a wall/gap that needs to be teleported through).
// Returns the wall-hug position to move to before teleporting, or empty Position if no special handling needed.
func (pf *PathFinder) RequiresWallHuggingToReach(to data.Position) (data.Position, bool) {
	from := pf.data.PlayerUnit.Position

	// Only relevant if player can teleport
	if !pf.data.CanTeleport() {
		return data.Position{}, false
	}

	// Use existing gap detection to find if path crosses wall/gap
	beforeGap, _, foundGap := pf.DetectGapAndGetTeleportPositions(from, to)
	if !foundGap {
		return data.Position{}, false
	}

	// There's a gap - return the position just before the gap to move to
	// Player should wall-hug to reach this position, then teleport through
	return beforeGap, true
}

func (pf *PathFinder) GetPath(to data.Position) (Path, int, bool) {
	from := pf.data.PlayerUnit.Position
	a := pf.data.AreaData
	canTeleport := pf.data.CanTeleport()

	// We don't want to modify the original grid
	grid := a.Grid.Copy()

	// Special handling for Arcane Sanctuary (to allow pathing with platforms)
	if pf.data.PlayerUnit.Area == area.ArcaneSanctuary && pf.data.CanTeleport() {
		// Make all non-walkable tiles into low priority tiles for teleport pathing
		for y := 0; y < grid.Height; y++ {
			for x := 0; x < grid.Width; x++ {
				if grid.Get(x, y) == game.CollisionTypeNonWalkable {
					grid.Set(x, y, game.CollisionTypeLowPriority)
				}
			}
		}
	}
	// Lut Gholein map is a bit bugged, we should close this fake path to avoid pathing issues
	if a.Area == area.LutGholein {
		if 210 < a.Grid.Width && 13 < a.Grid.Height {
			a.Grid.Set(210, 13, game.CollisionTypeNonWalkable)
		}
	}

	if !a.IsInside(to) {
		expandedGrid, err := pf.mergeGrids(to, canTeleport)
		if err != nil {
			return nil, 0, false
		}
		grid = expandedGrid
	}

	if !grid.IsWalkable(to) {
		if walkableTo, found := pf.findNearbyWalkablePositionInGrid(grid, to); found {
			to = walkableTo
		}
	}
	from = grid.RelativePosition(from)
	to = grid.RelativePosition(to)

	// Add objects to the collision grid as obstacles
	for _, o := range pf.data.AreaData.Objects {
		if !grid.IsWalkable(o.Position) {
			continue
		}
		relativePos := grid.RelativePosition(o.Position)
		if relativePos.X < 0 || relativePos.X >= grid.Width || relativePos.Y < 0 || relativePos.Y >= grid.Height {
			continue
		}
		grid.Set(relativePos.X, relativePos.Y, game.CollisionTypeObject)
		for i := -2; i <= 2; i++ {
			for j := -2; j <= 2; j++ {
				if i == 0 && j == 0 {
					continue
				}
				ny, nx := relativePos.Y+i, relativePos.X+j
				if ny < 0 || ny >= grid.Height || nx < 0 || nx >= grid.Width {
					continue
				}
				if grid.Get(nx, ny) == game.CollisionTypeWalkable {
					grid.Set(nx, ny, game.CollisionTypeLowPriority)
				}
			}
		}
	}

	// Add monsters to the collision grid as obstacles
	for _, m := range pf.data.Monsters {
		if !grid.IsWalkable(m.Position) {
			continue
		}
		relativePos := grid.RelativePosition(m.Position)
		if relativePos.X < 0 || relativePos.X >= grid.Width || relativePos.Y < 0 || relativePos.Y >= grid.Height {
			continue
		}
		grid.Set(relativePos.X, relativePos.Y, game.CollisionTypeMonster)
	}

	// set barricade tower as non walkable in act 5
	if a.Area == area.FrigidHighlands || a.Area == area.FrozenTundra || a.Area == area.ArreatPlateau {
		towerCount := 0
		for _, n := range pf.data.NPCs {
			if n.ID != npc.BarricadeTower {
				continue
			}
			if len(n.Positions) == 0 {
				continue
			}
			npcPos := n.Positions[0]
			relativePos := grid.RelativePosition(npcPos)
			towerCount++

			// Set a 5x5 area around the barricade tower as non-walkable
			blockedCells := 0
			for dy := -2; dy <= 2; dy++ {
				for dx := -2; dx <= 2; dx++ {
					towerY := relativePos.Y + dy
					towerX := relativePos.X + dx

					// Bounds checking to prevent array index out of bounds
					if towerY >= 0 && towerY < grid.Height &&
						towerX >= 0 && towerX < grid.Width {
						grid.Set(towerX, towerY, game.CollisionTypeNonWalkable)
						blockedCells++
					}
				}
			}
		}
	}

	path, distance, found := astar.CalculatePath(grid, from, to, canTeleport, pf.astarBuffers, a.Area, false)

	if config.Koolo.Debug.RenderMap {
		pf.renderMap(grid, from, to, path)
	}

	if pf.data.PlayerUnit.Area != area.ArcaneSanctuary && pf.data.CanTeleport() && found && len(path) > 1 {
		// Remove or replace short teleports (<= 3) that try to teleport into invalid map terrain (not accessible)
		newPath := make(Path, 0, len(path))
		newPath = append(newPath, path[0])

		isValidTeleportDest := func(pos data.Position) bool {
			if pos.X < 0 || pos.Y < 0 || pos.X >= grid.Width || pos.Y >= grid.Height {
				return false
			}
			ct := grid.Get(pos.X, pos.Y)
			return ct == game.CollisionTypeWalkable || ct == game.CollisionTypeTeleportOver
		}

		// Helper: Bresenham's line algorithm to find last valid tile before invalid
		findLastValid := func(from, to data.Position) data.Position {
			x0, y0 := from.X, from.Y
			x1, y1 := to.X, to.Y
			dx := utils.Abs(x1 - x0)
			dy := utils.Abs(y1 - y0)
			sx := -1
			if x0 < x1 {
				sx = 1
			}
			sy := -1
			if y0 < y1 {
				sy = 1
			}
			err := dx - dy
			lastValid := from
			for {
				pos := data.Position{X: x0, Y: y0}
				if !isValidTeleportDest(pos) {
					break
				}
				lastValid = pos
				if x0 == x1 && y0 == y1 {
					break
				}
				e2 := 2 * err
				if e2 > -dy {
					err -= dy
					x0 += sx
				}
				if e2 < dx {
					err += dx
					y0 += sy
				}
			}
			return lastValid
		}

		for i := 1; i < len(path); i++ {
			p1 := path[i-1]
			p2 := path[i]

			dx := p2.X - p1.X
			if dx < 0 {
				dx = -dx
			}
			dy := p2.Y - p1.Y
			if dy < 0 {
				dy = -dy
			}
			cheb := dx
			if dy > cheb {
				cheb = dy
			}

			if cheb <= 3 {
				if !isValidTeleportDest(p2) {
					lastValid := findLastValid(p1, p2)
					// Only add if not the same as p1 and not already in path
					if (lastValid.X != p1.X || lastValid.Y != p1.Y) && (len(newPath) == 0 || lastValid != newPath[len(newPath)-1]) {
						// Ensure lastValid is in the correct direction (between p1 and p2)
						d1 := (p2.X-p1.X)*(lastValid.X-p1.X) + (p2.Y-p1.Y)*(lastValid.Y-p1.Y)
						d2 := (p2.X-p1.X)*(p2.X-p1.X) + (p2.Y-p1.Y)*(p2.Y-p1.Y)
						// d1 >= 0 ensures lastValid is not behind p1, d1 <= d2 ensures it's not past p2
						if d1 >= 0 && d1 <= d2 {
							// Also ensure lastValid is closer to p2 than p1 is
							distLastValidP2 := (p2.X-lastValid.X)*(p2.X-lastValid.X) + (p2.Y-lastValid.Y)*(p2.Y-lastValid.Y)
							if distLastValidP2 < d2 {
								newPath = append(newPath, lastValid)
							}
						}
					}
					continue // skip original invalid step
				}
			}

			newPath = append(newPath, p2)
		}

		path = newPath
	}

	return path, distance, found
}

// isAdjacentToWall returns true if the given grid tile is next to a NonWalkable tile.
// Chebyshev distance 1 (8-neighborhood).
func isAdjacentToWall(grid *game.Grid, pos data.Position) bool {
	if pos.X < 0 || pos.Y < 0 || pos.X >= grid.Width || pos.Y >= grid.Height {
		return false
	}
	for dy := -1; dy <= 1; dy++ {
		for dx := -1; dx <= 1; dx++ {
			if dx == 0 && dy == 0 {
				continue
			}
			nx := pos.X + dx
			ny := pos.Y + dy
			if nx < 0 || ny < 0 || nx >= grid.Width || ny >= grid.Height {
				continue
			}
			ct := grid.Get(nx, ny)
			if ct == game.CollisionTypeNonWalkable {
				return true
			}
		}
	}
	return false
}

// isTeleportCrossingSegment returns true if segment from p1 to p2 crosses NonWalkable/TeleportOver terrain
// implying the intent to cross a wall via teleport when teleport is enabled.
func isTeleportCrossingSegment(grid *game.Grid, p1, p2 data.Position) bool {
	// Bresenham along the line, if any tile is NonWalkable or TeleportOver, it is a crossing.
	x0, y0 := p1.X, p1.Y
	x1, y1 := p2.X, p2.Y
	dx := utils.Abs(x1 - x0)
	dy := utils.Abs(y1 - y0)
	sx := -1
	if x0 < x1 {
		sx = 1
	}
	sy := -1
	if y0 < y1 {
		sy = 1
	}
	err := dx - dy
	for {
		if x0 < 0 || y0 < 0 || x0 >= grid.Width || y0 >= grid.Height {
			break
		}
		ct := grid.Get(x0, y0)
		if ct == game.CollisionTypeNonWalkable || ct == game.CollisionTypeTeleportOver {
			return true
		}
		if x0 == x1 && y0 == y1 {
			break
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x0 += sx
		}
		if e2 < dx {
			err += dx
			y0 += sy
		}
	}
	return false
}

// findLastValidWalkableOrTeleportDest finds the last tile along the segment that is valid to stand/teleport on.
func findLastValidWalkableOrTeleportDest(grid *game.Grid, from, to data.Position) data.Position {
	isValid := func(pos data.Position) bool {
		if pos.X < 0 || pos.Y < 0 || pos.X >= grid.Width || pos.Y >= grid.Height {
			return false
		}
		ct := grid.Get(pos.X, pos.Y)
		return ct == game.CollisionTypeWalkable || ct == game.CollisionTypeTeleportOver
	}
	x0, y0 := from.X, from.Y
	x1, y1 := to.X, to.Y
	dx := utils.Abs(x1 - x0)
	dy := utils.Abs(y1 - y0)
	sx := -1
	if x0 < x1 {
		sx = 1
	}
	sy := -1
	if y0 < y1 {
		sy = 1
	}
	err := dx - dy
	lastValid := from
	for {
		pos := data.Position{X: x0, Y: y0}
		if !isValid(pos) {
			break
		}
		lastValid = pos
		if x0 == x1 && y0 == y1 {
			break
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x0 += sx
		}
		if e2 < dx {
			err += dx
			y0 += sy
		}
	}
	return lastValid
}

// advanceAlongDirection returns the point moved "step" units along vector p1->p2.
func advanceAlongDirection(p1, p2 data.Position, step int) data.Position {
	dx := p2.X - p1.X
	dy := p2.Y - p1.Y
	// Normalize to unit Chebyshev step
	stepx := 0
	stepy := 0
	if dx > 0 {
		stepx = 1
	} else if dx < 0 {
		stepx = -1
	}
	if dy > 0 {
		stepy = 1
	} else if dy < 0 {
		stepy = -1
	}
	return data.Position{X: p1.X + stepx*step, Y: p1.Y + stepy*step}
}
