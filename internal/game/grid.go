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
	CollisionTypeDiagonalTile
)

type CollisionType uint8

type Grid struct {
	OffsetX       int
	OffsetY       int
	Width         int
	Height        int
	CollisionGrid [][]CollisionType
}

func NewGrid(rawCollisionGrid [][]CollisionType, offsetX, offsetY int, canTeleport bool, exits []data.Level) *Grid {
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
			if isBlockingTile(collisionType, canTeleport) {
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

	fillGaps(rawCollisionGrid, canTeleport)
	thickenCollisions(rawCollisionGrid, canTeleport)
	markDiagonalTiles(rawCollisionGrid)
	drillExits(rawCollisionGrid, offsetX, offsetY, exits)

	return grid
}

// NewGridFromProcessed returns a grid without applying any additional collision processing.
// Use it when the supplied collision matrix has already been thickened/drilled.
func NewGridFromProcessed(processedCollisionGrid [][]CollisionType, offsetX, offsetY int) *Grid {
	return &Grid{
		OffsetX:       offsetX,
		OffsetY:       offsetY,
		Width:         len(processedCollisionGrid[0]),
		Height:        len(processedCollisionGrid),
		CollisionGrid: processedCollisionGrid,
	}
}

func isBlockingTile(tile CollisionType, canTeleport bool) bool {
	if tile == CollisionTypeNonWalkable || tile == CollisionTypeObject || tile == CollisionTypeMonster {
		return true
	}
	if !canTeleport && tile == CollisionTypeTeleportOver {
		return true
	}
	return tile == CollisionTypeThickened
}

func baseWalkable(ct CollisionType) bool {
	return ct == CollisionTypeWalkable || ct == CollisionTypeLowPriority
}

func isWalkableType(ct CollisionType) bool {
	return ct == CollisionTypeWalkable || ct == CollisionTypeLowPriority || ct == CollisionTypeDiagonalTile
}

func IsWalkableType(ct CollisionType) bool {
	return isWalkableType(ct)
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
	return isWalkableType(positionType)
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

func thickenCollisions(grid [][]CollisionType, canTeleport bool) {
	buffer := make([][]bool, len(grid))
	for y := range grid {
		buffer[y] = make([]bool, len(grid[y]))
	}

	for y := range grid {
		for x := range grid[y] {
			if !isBlockingTile(grid[y][x], canTeleport) {
				continue
			}
			for _, delta := range [][2]int{{1, 0}, {-1, 0}, {0, 1}, {0, -1}} {
				ny := y + delta[1]
				nx := x + delta[0]
				if ny < 0 || ny >= len(grid) {
					continue
				}

				row := grid[ny]
				if len(row) == 0 || nx < 0 || nx >= len(row) {
					continue
				}

				if row[nx] == CollisionTypeWalkable || row[nx] == CollisionTypeLowPriority {
					buffer[ny][nx] = true
				}
			}
		}
	}

	for y := range grid {
		for x := range grid[y] {
			if buffer[y][x] {
				grid[y][x] = CollisionTypeThickened
			}
		}
	}
}

func markDiagonalTiles(grid [][]CollisionType) {
	// Mark tiles that only touch walkable space diagonally
	// so pathing can slide through tight corners created by thickened tiles
	if len(grid) == 0 {
		return
	}

	toDiagonal := make([][]bool, len(grid))
	for y := range grid {
		toDiagonal[y] = make([]bool, len(grid[y]))
	}

	for y := 0; y+1 < len(grid); y++ {
		row := grid[y]
		next := grid[y+1]

		w := len(row)
		if len(next) < w {
			w = len(next)
		}

		for x := 0; x+1 < w; x++ {
			a := row[x]    // (x,   y)
			b := row[x+1]  // (x+1, y)
			c := next[x]   // (x,   y+1)
			d := next[x+1] // (x+1, y+1)

			// Case 1: a & d walkable, b & c thickened
			if baseWalkable(a) && baseWalkable(d) &&
				b == CollisionTypeThickened && c == CollisionTypeThickened {
				toDiagonal[y][x+1] = true // b
				toDiagonal[y+1][x] = true // c
			}

			// Case 2: b & c walkable, a & d thickened
			if baseWalkable(b) && baseWalkable(c) &&
				a == CollisionTypeThickened && d == CollisionTypeThickened {
				toDiagonal[y][x] = true     // a
				toDiagonal[y+1][x+1] = true // d
			}
		}
	}

	for y := range grid {
		for x := range grid[y] {
			if toDiagonal[y][x] {
				grid[y][x] = CollisionTypeDiagonalTile
			}
		}
	}
}

func setWalkable(grid [][]CollisionType, x, y int) {
	if y < 0 || y >= len(grid) {
		return
	}
	if x < 0 || x >= len(grid[y]) {
		return
	}
	grid[y][x] = CollisionTypeWalkable
}

func hasWalkableNeighbor(grid [][]CollisionType, x, y int) bool {
	for dx := -1; dx <= 1; dx++ {
		for dy := -1; dy <= 1; dy++ {
			if dx == 0 && dy == 0 {
				continue
			}
			nx := x + dx
			ny := y + dy
			if ny < 0 || ny >= len(grid) {
				continue
			}
			if nx < 0 || nx >= len(grid[ny]) {
				continue
			}
			if isWalkableType(grid[ny][nx]) {
				return true
			}
		}
	}
	return false
}

func drillExit(grid [][]CollisionType, x, y int) {
	setWalkable(grid, x, y)

	if hasWalkableNeighbor(grid, x, y) {
		return
	}

	for _, delta := range [][2]int{{1, 0}, {-1, 0}, {0, 1}, {0, -1}} {
		setWalkable(grid, x+delta[0], y+delta[1])
	}

	for dx := -2; dx <= 2; dx++ {
		for dy := -2; dy <= 2; dy++ {
			if ((dx < -1 || dx > 1) && (dy < -1 || dy > 1)) || (dx == 0 && dy == 0) {
				continue
			}
			nx := x + dx
			ny := y + dy
			if ny < 0 || ny >= len(grid) {
				continue
			}
			if nx < 0 || nx >= len(grid[ny]) {
				continue
			}
			if grid[ny][nx] == CollisionTypeThickened {
				grid[ny][nx] = CollisionTypeWalkable
			}
		}
	}
}

func drillExits(grid [][]CollisionType, offsetX, offsetY int, exits []data.Level) {
	for _, exit := range exits {
		relativeX := exit.Position.X - offsetX
		relativeY := exit.Position.Y - offsetY
		if relativeY < 0 || relativeY >= len(grid) {
			continue
		}
		if relativeX < 0 || relativeX >= len(grid[relativeY]) {
			continue
		}
		drillExit(grid, relativeX, relativeY)
	}
}

func isGap(grid [][]CollisionType, x, y int, canTeleport bool) bool {
	if !isBlockingTile(grid[y][x], canTeleport) {
		return false
	}

	const gapSize = 3

	spaces := 0
	for i := x - gapSize - 1; i <= x+gapSize+1 && spaces < gapSize; i++ {
		if i < 0 || i >= len(grid[y]) {
			continue
		}
		if isBlockingTile(grid[y][i], canTeleport) {
			spaces++
		} else {
			spaces = 0
		}
	}
	if spaces < gapSize {
		return true
	}

	spaces = 0
	for j := y - gapSize - 1; j <= y+gapSize+1 && spaces < gapSize; j++ {
		if j < 0 || j >= len(grid) {
			continue
		}
		if isBlockingTile(grid[j][x], canTeleport) {
			spaces++
		} else {
			spaces = 0
		}
	}
	return spaces < gapSize
}

func fillGaps(grid [][]CollisionType, canTeleport bool) {
	for y := 0; y < len(grid); y++ {
		row := grid[y]
		for x := 0; x < len(row); x++ {
			if !isGap(grid, x, y, canTeleport) {
				continue
			}
			row[x] = CollisionTypeThickened
		}
	}
}
