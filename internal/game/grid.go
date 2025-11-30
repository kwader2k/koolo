package game

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/object"
)

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

func NewGrid(rawCollisionGrid [][]CollisionType, offsetX, offsetY int, exits []data.Level, areaID area.ID, objects []data.Object) *Grid {
	grid := &Grid{
		OffsetX:       offsetX,
		OffsetY:       offsetY,
		Width:         len(rawCollisionGrid[0]),
		Height:        len(rawCollisionGrid),
		CollisionGrid: rawCollisionGrid,
	}

	if areaID.Area().Act() == 1 || areaID.IsTown() {
		// Act 1 areas have some problematic areas with tiles that are marked walkable but are not.
		// To avoid pathing issues, we thicken all non-walkable tiles in Act 1 areas.
		thickenCollisions(rawCollisionGrid, objects, offsetX, offsetY)
	}

	fillGaps(rawCollisionGrid)
	drillExits(rawCollisionGrid, offsetX, offsetY, exits)

	// Lower the priority for the walkable tiles that are close to non-walkable tiles, so we can avoid walking too close to walls and obstacles
	lowPriorityRadius := 2
	for y := 0; y < len(rawCollisionGrid); y++ {
		for x := 0; x < len(rawCollisionGrid[y]); x++ {
			collisionType := rawCollisionGrid[y][x]
			if isBlockingTile(collisionType, false) {
				for i := -lowPriorityRadius; i <= lowPriorityRadius; i++ {
					for j := -lowPriorityRadius; j <= lowPriorityRadius; j++ {
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
	switch tile {
	case CollisionTypeNonWalkable:
		return true
	case CollisionTypeTeleportOver:
		return !canTeleport
	case CollisionTypeThickened:
		return !canTeleport
	default:
		return false
	}
}

func isWalkableType(ct CollisionType) bool {
	return ct == CollisionTypeWalkable || ct == CollisionTypeLowPriority
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

func thickenCollisions(grid [][]CollisionType, objects []data.Object, offsetX, offsetY int) {
	buffer := make([][]bool, len(grid))
	for y := range grid {
		buffer[y] = make([]bool, len(grid[y]))
	}

	var treeObj *data.Object
	for _, obj := range objects {
		if obj.Name == object.InifussTree {
			treeObj = &obj
			break
		}
	}

	for y := range grid {
		for x := range grid[y] {
			if grid[y][x] != CollisionTypeNonWalkable {
				continue
			}
			// if we're close the Inifuss tree, skip thickening
			if treeObj != nil {
				treeX := treeObj.Position.X - offsetX
				treeY := treeObj.Position.Y - offsetY
				if absInt(x-treeX) <= 10 && absInt(y-treeY) <= 10 {
					continue
				}
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

				if row[nx] == CollisionTypeWalkable {
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

func isGap(grid [][]CollisionType, x, y int) bool {
	if !isBlockingTile(grid[y][x], false) {
		return false
	}

	const gapSize = 3

	spaces := 0
	for i := x - gapSize - 1; i <= x+gapSize+1 && spaces < gapSize; i++ {
		if i < 0 || i >= len(grid[y]) {
			continue
		}
		if isBlockingTile(grid[y][i], false) {
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
		if isBlockingTile(grid[j][x], false) {
			spaces++
		} else {
			spaces = 0
		}
	}
	return spaces < gapSize
}

func fillGaps(grid [][]CollisionType) {
	for y := 0; y < len(grid); y++ {
		row := grid[y]
		for x := 0; x < len(row); x++ {
			if !isGap(grid, x, y) {
				continue
			}
			row[x] = CollisionTypeThickened
		}
	}
}

// absInt returns the absolute value of an integer.
func absInt(n int) int {
	if n < 0 {
		return -n
	}
	return n
}
