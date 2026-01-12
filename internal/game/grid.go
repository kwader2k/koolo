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

// Grid uses a flat 1D slice for collision data to minimize allocations.
// Access via Get(x,y) and Set(x,y,v) methods, or directly via CollisionGrid[y*Width+x].
type Grid struct {
	OffsetX       int
	OffsetY       int
	Width         int
	Height        int
	CollisionGrid []CollisionType // flat 1D array: index = y*Width + x
}

// Get returns the collision type at (x, y). No bounds checking.
func (g *Grid) Get(x, y int) CollisionType {
	return g.CollisionGrid[y*g.Width+x]
}

// Set sets the collision type at (x, y). No bounds checking.
func (g *Grid) Set(x, y int, v CollisionType) {
	g.CollisionGrid[y*g.Width+x] = v
}

// NewGrid creates a Grid from a 2D collision grid, converting to flat storage.
func NewGrid(rawCollisionGrid [][]CollisionType, offsetX, offsetY int, canTeleport bool) *Grid {
	height := len(rawCollisionGrid)
	width := len(rawCollisionGrid[0])

	// Convert 2D to flat 1D (single allocation instead of height allocations)
	flat := make([]CollisionType, width*height)
	for y := 0; y < height; y++ {
		copy(flat[y*width:(y+1)*width], rawCollisionGrid[y])
	}

	grid := &Grid{
		OffsetX:       offsetX,
		OffsetY:       offsetY,
		Width:         width,
		Height:        height,
		CollisionGrid: flat,
	}

	// Let's lower the priority for the walkable tiles that are close to non-walkable tiles, so we can avoid walking too close to walls and obstacles
	for y := 0; y < height; y++ {
		for x := 0; x < width; x++ {
			collisionType := grid.Get(x, y)
			if collisionType == CollisionTypeNonWalkable || (!canTeleport && collisionType == CollisionTypeTeleportOver) {
				for i := -2; i <= 2; i++ {
					for j := -2; j <= 2; j++ {
						if i == 0 && j == 0 {
							continue
						}
						if y+i < 0 || y+i >= height || x+j < 0 || x+j >= width {
							continue
						}
						if grid.Get(x+j, y+i) == CollisionTypeWalkable {
							grid.Set(x+j, y+i, CollisionTypeLowPriority)
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

	height := grid.Height
	width := grid.Width
	if height == 0 || width == 0 {
		return
	}

	// First pass: identify and mark narrow passages
	for y := 1; y < height-1; y++ {
		for x := 1; x < width-1; x++ {
			if grid.Get(x, y) != CollisionTypeWalkable {
				continue
			}

			// Check if this walkable tile creates a narrow passage
			nonWalkableNeighbors := 0

			// Check 4 cardinal directions
			if grid.Get(x, y-1) == CollisionTypeNonWalkable {
				nonWalkableNeighbors++
			}
			if grid.Get(x, y+1) == CollisionTypeNonWalkable {
				nonWalkableNeighbors++
			}
			if grid.Get(x-1, y) == CollisionTypeNonWalkable {
				nonWalkableNeighbors++
			}
			if grid.Get(x+1, y) == CollisionTypeNonWalkable {
				nonWalkableNeighbors++
			}

			// If surrounded by 3+ non-walkable neighbors, it's a narrow passage
			if nonWalkableNeighbors >= 3 {
				grid.Set(x, y, CollisionTypeTeleportOver)
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

	height := grid.Height
	width := grid.Width
	if height == 0 || width == 0 {
		return
	}

	for y := 1; y < height-1; y++ {
		for x := 1; x < width-1; x++ {
			// Check for diagonal gaps: opposite corners are both non-walkable
			// but the connecting diagonal tiles are walkable

			// Top-left to bottom-right diagonal gap
			topLeft := grid.Get(x-1, y-1)
			bottomRight := grid.Get(x+1, y+1)
			if (topLeft == CollisionTypeNonWalkable || topLeft == CollisionTypeTeleportOver) &&
				(bottomRight == CollisionTypeNonWalkable || bottomRight == CollisionTypeTeleportOver) {
				if grid.Get(x, y) == CollisionTypeWalkable {
					// Check if adjacent tiles allow passage
					if grid.Get(x, y-1) == CollisionTypeNonWalkable &&
						grid.Get(x-1, y) == CollisionTypeNonWalkable {
						grid.Set(x, y, CollisionTypeTeleportOver)
					}
				}
			}

			// Top-right to bottom-left diagonal gap
			topRight := grid.Get(x+1, y-1)
			bottomLeft := grid.Get(x-1, y+1)
			if (topRight == CollisionTypeNonWalkable || topRight == CollisionTypeTeleportOver) &&
				(bottomLeft == CollisionTypeNonWalkable || bottomLeft == CollisionTypeTeleportOver) {
				if grid.Get(x, y) == CollisionTypeWalkable {
					// Check if adjacent tiles allow passage
					if grid.Get(x, y-1) == CollisionTypeNonWalkable &&
						grid.Get(x+1, y) == CollisionTypeNonWalkable {
						grid.Set(x, y, CollisionTypeTeleportOver)
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

				if y >= 0 && y < grid.Height && x >= 0 && x < grid.Width {
					if grid.Get(x, y) == CollisionTypeTeleportOver {
						grid.Set(x, y, CollisionTypeWalkable)
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
	positionType := g.Get(p.X, p.Y)
	return positionType != CollisionTypeNonWalkable && positionType != CollisionTypeTeleportOver
}

// Copy returns a deep copy of the Grid with single allocation for flat array
func (g *Grid) Copy() *Grid {
	cg := make([]CollisionType, len(g.CollisionGrid))
	copy(cg, g.CollisionGrid)

	return &Grid{
		OffsetX:       g.OffsetX,
		OffsetY:       g.OffsetY,
		Width:         g.Width,
		Height:        g.Height,
		CollisionGrid: cg,
	}
}
