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
)

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

func (pf *PathFinder) GetPath(to data.Position) (Path, int, bool) {
	// First try direct path
	if path, distance, found := pf.GetPathFrom(pf.data.PlayerUnit.Position, to); found {
		return path, distance, true
	}

	walkableTo, foundTo := pf.findNearbyWalkablePosition(to)
	// If direct path fails, try to find nearby to walkable position
	if foundTo {
		path, distance, found := pf.GetPathFrom(pf.data.PlayerUnit.Position, walkableTo)
		if found {
			return path, distance, true
		}
	}

	//We definitively tried our best
	return nil, 0, false
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

	path, distance, found := astar.CalculatePath(grid, from, to, canTeleport, pf.astarBuffers)

	if config.Koolo.Debug.RenderMap {
		pf.renderMap(grid, from, to, path)
	}

	return path, distance, found
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
	for radius := 1; radius <= 3; radius++ {
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
