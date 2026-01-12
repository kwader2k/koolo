package astar

import (
	"container/heap"
	"math"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/koolo/internal/game"
)

var directions = []data.Position{
	{0, 1},   // Down
	{1, 0},   // Right
	{0, -1},  // Up
	{-1, 0},  // Left
	{1, 1},   // Down-Right (Southeast)
	{-1, 1},  // Down-Left (Southwest)
	{1, -1},  // Up-Right (Northeast)
	{-1, -1}, // Up-Left (Northwest)
}

type Node struct {
	data.Position
	Cost     int
	Priority int
	Index    int
	TpStreak int
}

func direction(from, to data.Position) (dx, dy int) {
	dx = to.X - from.X
	dy = to.Y - from.Y
	return
}

const MaxConsecutiveTeleportOver = 12

// AStarBuffers holds reusable buffers for A* pathfinding to avoid allocations
type AStarBuffers struct {
	costSoFar []int           // flat 1D array: index = x*height + y
	cameFrom  []data.Position // flat 1D array: index = x*height + y
	width     int
	height    int
}

// ensureBuffers ensures the buffers are large enough for the given dimensions
func (b *AStarBuffers) ensureBuffers(width, height int) {
	size := width * height
	if len(b.costSoFar) < size || b.width != width || b.height != height {
		b.costSoFar = make([]int, size)
		b.cameFrom = make([]data.Position, size)
		b.width = width
		b.height = height
	}
	// Reset cost values to MaxInt32 (only reset what we'll use)
	for i := 0; i < size; i++ {
		b.costSoFar[i] = math.MaxInt32
	}
}

func (b *AStarBuffers) index(x, y int) int {
	return x*b.height + y
}

// CalculatePath finds a path using A* algorithm. If buffers is nil, allocates new buffers.
// For optimal performance, reuse buffers across calls by creating AStarBuffers once per PathFinder.
func CalculatePath(g *game.Grid, start, goal data.Position, canTeleport bool, buffers *AStarBuffers) ([]data.Position, int, bool) {
	inBounds := func(p data.Position) bool {
		return p.X >= 0 && p.Y >= 0 && p.X < g.Width && p.Y < g.Height
	}

	if g == nil || g.Width == 0 || g.Height == 0 {
		return nil, 0, false
	}

	// Bail out early if start or goal is outside the grid to prevent panics
	if !inBounds(start) || !inBounds(goal) {
		return nil, 0, false
	}

	// Use provided buffers or create temporary ones
	var costSoFar []int
	var cameFrom []data.Position
	var idx func(x, y int) int

	if buffers != nil {
		buffers.ensureBuffers(g.Width, g.Height)
		costSoFar = buffers.costSoFar
		cameFrom = buffers.cameFrom
		idx = buffers.index
	} else {
		// Fallback: allocate flat arrays (still better than 2D)
		size := g.Width * g.Height
		costSoFar = make([]int, size)
		cameFrom = make([]data.Position, size)
		for i := range costSoFar {
			costSoFar[i] = math.MaxInt32
		}
		height := g.Height
		idx = func(x, y int) int { return x*height + y }
	}

	pq := make(PriorityQueue, 0, 256)
	heap.Init(&pq)

	startNode := &Node{Position: start, Cost: 0, Priority: heuristic(start, goal)}
	heap.Push(&pq, startNode)
	costSoFar[idx(start.X, start.Y)] = 0

	neighbors := make([]data.Position, 0, 8)

	for pq.Len() > 0 {
		current := heap.Pop(&pq).(*Node)

		// Build the path if we reached the goal - O(n) instead of O(n²)
		if current.Position == goal {
			// First pass: count path length (excluding teleport-over tiles)
			pathLen := 1 // start position
			for p := goal; p != start; p = cameFrom[idx(p.X, p.Y)] {
				if g.Get(p.X, p.Y) != game.CollisionTypeTeleportOver {
					pathLen++
				}
			}
			// Build path in reverse order, then reverse in-place
			path := make([]data.Position, pathLen)
			i := pathLen - 1
			for p := goal; p != start; p = cameFrom[idx(p.X, p.Y)] {
				if g.Get(p.X, p.Y) != game.CollisionTypeTeleportOver {
					path[i] = p
					i--
				}
			}
			path[0] = start
			return path, len(path), true
		}

		updateNeighbors(g, current, &neighbors, canTeleport)

		for _, neighbor := range neighbors {
			tileType := g.Get(neighbor.X, neighbor.Y)

			// Determine teleport streak
			teleportStreak := 0
			if tileType == game.CollisionTypeTeleportOver {
				teleportStreak = current.TpStreak + 1
			} else {
				teleportStreak = 0
			}

			// Skip if exceeds allowed consecutive teleport tiles
			if teleportStreak > MaxConsecutiveTeleportOver {
				continue
			}

			currentIdx := idx(current.X, current.Y)
			neighborIdx := idx(neighbor.X, neighbor.Y)
			newCost := costSoFar[currentIdx] + getCost(tileType, canTeleport)

			// Handicap for changing direction, this prevents zig-zagging around obstacles
			//curDirX, curDirY := direction(cameFrom[currentIdx], current.Position)
			//newDirX, newDirY := direction(current.Position, neighbor)
			//if curDirX != newDirX || curDirY != newDirY {
			//	newCost++
			//}

			if newCost < costSoFar[neighborIdx] {
				costSoFar[neighborIdx] = newCost
				priority := newCost + int(0.5*float64(heuristic(neighbor, goal)))
				heap.Push(&pq, &Node{Position: neighbor, Cost: newCost, Priority: priority, TpStreak: teleportStreak})
				cameFrom[neighborIdx] = current.Position
			}
		}
	}

	return nil, 0, false
}

// Get walkable neighbors of a given node
func updateNeighbors(grid *game.Grid, node *Node, neighbors *[]data.Position, canTeleport bool) {
	*neighbors = (*neighbors)[:0]

	x, y := node.X, node.Y
	gridWidth, gridHeight := grid.Width, grid.Height

	isBlocked := func(px, py int) bool {
		if px < 0 || px >= gridWidth || py < 0 || py >= gridHeight {
			return true
		}
		collisionType := grid.Get(px, py)
		switch collisionType {
		case game.CollisionTypeNonWalkable:
			return true
		case game.CollisionTypeTeleportOver:
			return !canTeleport
		case game.CollisionTypeThickened:
			return !canTeleport
		default:
			return false
		}
	}

	for _, d := range directions {
		newX, newY := x+d.X, y+d.Y

		if isBlocked(newX, newY) {
			continue
		}

		if d.X != 0 && d.Y != 0 {
			adj1X, adj1Y := x+d.X, y
			adj2X, adj2Y := x, y+d.Y

			if isBlocked(adj1X, adj1Y) || isBlocked(adj2X, adj2Y) {
				continue
			}
		}

		*neighbors = append(*neighbors, data.Position{X: newX, Y: newY})
	}
}

func getCost(tileType game.CollisionType, canTeleport bool) int {
	switch tileType {
	case game.CollisionTypeWalkable:
		return 1 // Walkable
	case game.CollisionTypeMonster:
		return 16
	case game.CollisionTypeObject:
		return 4 // Soft blocker
	case game.CollisionTypeLowPriority:
		return 20
	case game.CollisionTypeTeleportOver:
		if canTeleport {
			return 1
		}
		return math.MaxInt32
	case game.CollisionTypeThickened:
		if canTeleport {
			return 1
		}
		return math.MaxInt32
	default:
		return math.MaxInt32
	}
}

func heuristic(a, b data.Position) int {
	dx := math.Abs(float64(a.X - b.X))
	dy := math.Abs(float64(a.Y - b.Y))
	return int(dx + dy + (math.Sqrt(2)-2)*math.Min(dx, dy))
}
