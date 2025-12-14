package game

import "github.com/hectorgimenez/d2go/pkg/data"

const (
	CollisionTypeNonWalkable CollisionType = iota
	CollisionTypeWalkable
	CollisionTypeLowPriority
	CollisionTypeMonster
	CollisionTypeObject
	CollisionTypeTeleportOver
	CollisionTypeThickened
)

type CollisionType uint8

type Grid struct {
	OffsetX       int
	OffsetY       int
	Width         int
	Height        int
	CollisionGrid [][]CollisionType
}

func NewGrid(rawCollisionGrid [][]CollisionType, offsetX, offsetY int, canTeleport bool) *Grid {
	grid := &Grid{
		OffsetX:       offsetX,
		OffsetY:       offsetY,
		Width:         len(rawCollisionGrid[0]),
		Height:        len(rawCollisionGrid),
		CollisionGrid: rawCollisionGrid,
	}

	// Let's lower the priority for the walkable tiles that are close to non-walkable tiles, so we can avoid walking too close to walls and obstacles
	for y := 0; y < len(rawCollisionGrid); y++ {
		for x := 0; x < len(rawCollisionGrid[y]); x++ {
			collisionType := rawCollisionGrid[y][x]
			if collisionType == CollisionTypeNonWalkable || (!canTeleport && collisionType == CollisionTypeTeleportOver) {
				for i := -2; i <= 2; i++ {
					for j := -2; j <= 2; j++ {
						if i == 0 && j == 0 {
							continue
						}
						if y+i < 0 || y+i >= len(rawCollisionGrid) || x+j < 0 || x+j >= len(rawCollisionGrid[y]) {
							continue
						}
						if rawCollisionGrid[y+i][x+j] == CollisionTypeWalkable {
							rawCollisionGrid[y+i][x+j] = CollisionTypeLowPriority
						}
					}
				}
			}
		}
	}

	return grid
}

// thickenCollisions marks narrow gaps and single-tile openings as TeleportOver
// to prevent walkers from pathing through problematic 1-2 tile wide passages.
// Applied to all areas to improve pathfinding stability and reduce stuck issues.
func thickenCollisions(grid *Grid) {
	if grid == nil || grid.CollisionGrid == nil {
		return
	}

	height := len(grid.CollisionGrid)
	if height == 0 {
		return
	}
	width := len(grid.CollisionGrid[0])

	// First pass: identify and mark narrow passages
	for y := 1; y < height-1; y++ {
		for x := 1; x < width-1; x++ {
			if grid.CollisionGrid[y][x] != CollisionTypeWalkable {
				continue
			}

			// Check if this walkable tile creates a narrow passage
			nonWalkableNeighbors := 0

			// Check 4 cardinal directions
			if grid.CollisionGrid[y-1][x] == CollisionTypeNonWalkable {
				nonWalkableNeighbors++
			}
			if grid.CollisionGrid[y+1][x] == CollisionTypeNonWalkable {
				nonWalkableNeighbors++
			}
			if grid.CollisionGrid[y][x-1] == CollisionTypeNonWalkable {
				nonWalkableNeighbors++
			}
			if grid.CollisionGrid[y][x+1] == CollisionTypeNonWalkable {
				nonWalkableNeighbors++
			}

			// If surrounded by 3+ non-walkable neighbors, it's a narrow passage
			if nonWalkableNeighbors >= 3 {
				grid.CollisionGrid[y][x] = CollisionTypeTeleportOver
			}
		}
	}

	// Second pass: fill diagonal gaps
	fillGaps(grid)
}

// fillGaps closes diagonal gaps in collision map to prevent corner-cutting through walls
func fillGaps(grid *Grid) {
	if grid == nil || grid.CollisionGrid == nil {
		return
	}

	height := len(grid.CollisionGrid)
	if height == 0 {
		return
	}
	width := len(grid.CollisionGrid[0])

	for y := 1; y < height-1; y++ {
		for x := 1; x < width-1; x++ {
			// Check for diagonal gaps: opposite corners are both non-walkable
			// but the connecting diagonal tiles are walkable

			// Top-left to bottom-right diagonal gap
			if (grid.CollisionGrid[y-1][x-1] == CollisionTypeNonWalkable ||
				grid.CollisionGrid[y-1][x-1] == CollisionTypeTeleportOver) &&
				(grid.CollisionGrid[y+1][x+1] == CollisionTypeNonWalkable ||
					grid.CollisionGrid[y+1][x+1] == CollisionTypeTeleportOver) {
				if grid.CollisionGrid[y][x] == CollisionTypeWalkable {
					// Check if adjacent tiles allow passage
					if grid.CollisionGrid[y-1][x] == CollisionTypeNonWalkable &&
						grid.CollisionGrid[y][x-1] == CollisionTypeNonWalkable {
						grid.CollisionGrid[y][x] = CollisionTypeTeleportOver
					}
				}
			}

			// Top-right to bottom-left diagonal gap
			if (grid.CollisionGrid[y-1][x+1] == CollisionTypeNonWalkable ||
				grid.CollisionGrid[y-1][x+1] == CollisionTypeTeleportOver) &&
				(grid.CollisionGrid[y+1][x-1] == CollisionTypeNonWalkable ||
					grid.CollisionGrid[y+1][x-1] == CollisionTypeTeleportOver) {
				if grid.CollisionGrid[y][x] == CollisionTypeWalkable {
					// Check if adjacent tiles allow passage
					if grid.CollisionGrid[y-1][x] == CollisionTypeNonWalkable &&
						grid.CollisionGrid[y][x+1] == CollisionTypeNonWalkable {
						grid.CollisionGrid[y][x] = CollisionTypeTeleportOver
					}
				}
			}
		}
	}
}

// drillExits re-opens known entrance/exit points that may have been thickened
// This ensures valid entrances remain accessible for walkers
func drillExits(grid *Grid, exitPositions []data.Position) {
	if grid == nil || grid.CollisionGrid == nil || len(exitPositions) == 0 {
		return
	}

	for _, exit := range exitPositions {
		relPos := grid.RelativePosition(exit)

		// Bounds check
		if relPos.X < 0 || relPos.X >= grid.Width || relPos.Y < 0 || relPos.Y >= grid.Height {
			continue
		}

		// Re-open exit position and immediate neighbors
		for dy := -1; dy <= 1; dy++ {
			for dx := -1; dx <= 1; dx++ {
				y := relPos.Y + dy
				x := relPos.X + dx

				if y >= 0 && y < len(grid.CollisionGrid) && x >= 0 && x < len(grid.CollisionGrid[0]) {
					if grid.CollisionGrid[y][x] == CollisionTypeTeleportOver {
						grid.CollisionGrid[y][x] = CollisionTypeWalkable
					}
				}
			}
		}
	}
}

func (g *Grid) RelativePosition(p data.Position) data.Position {
	return data.Position{
		X: p.X - g.OffsetX,
		Y: p.Y - g.OffsetY,
	}
}

func (g *Grid) IsWalkable(p data.Position) bool {
	p = g.RelativePosition(p)
	if p.X < 0 || p.X >= g.Width || p.Y < 0 || p.Y >= g.Height {
		return false
	}
	positionType := g.CollisionGrid[p.Y][p.X]
	return positionType != CollisionTypeNonWalkable && positionType != CollisionTypeTeleportOver
}

func (g *Grid) Copy() *Grid {
	cg := make([][]CollisionType, g.Height)
	for y := 0; y < g.Height; y++ {
		cg[y] = make([]CollisionType, g.Width)
		copy(cg[y], g.CollisionGrid[y])
	}

	return &Grid{
		OffsetX:       g.OffsetX,
		OffsetY:       g.OffsetY,
		Width:         g.Width,
		Height:        g.Height,
		CollisionGrid: cg,
	}
}
