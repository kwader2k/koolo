package pather

import (
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/pather/astar"
)

// Smart path cache with LRU and area awareness
type pathCacheEntry struct {
	path         Path
	distance     int
	timestamp    time.Time
	lastAccessed time.Time
	accessCount  int
	forArea      area.ID
}

type pathCache struct {
	entries     map[string]*pathCacheEntry
	currentArea area.ID
	mu          sync.RWMutex
}

type gridCache struct {
	preparedGrid *game.Grid
	forArea      area.ID
	timestamp    time.Time
	mu           sync.RWMutex
}

type PathFinder struct {
	gr   *game.MemoryReader
	data *game.Data
	hid  *game.HID
	cfg  *config.CharacterCfg
	// Add smart path cache
	cache *pathCache
	// Add grid cache
	gridCache *gridCache
}

func NewPathFinder(gr *game.MemoryReader, data *game.Data, hid *game.HID, cfg *config.CharacterCfg) *PathFinder {
	return &PathFinder{
		gr:   gr,
		data: data,
		hid:  hid,
		cfg:  cfg,
		cache: &pathCache{
			entries:     make(map[string]*pathCacheEntry),
			currentArea: 0,
		},
		gridCache: &gridCache{
			preparedGrid: nil,
			forArea:      0,
		},
	}
}

// Generate cache key from area + quantized positions
func (pf *PathFinder) getCacheKey(from, to data.Position, forArea area.ID) string {
	// Quantize to 5-unit grid
	fromX := (from.X / 5) * 5
	fromY := (from.Y / 5) * 5
	toX := (to.X / 5) * 5
	toY := (to.Y / 5) * 5

	return fmt.Sprintf("%d_%d_%d_%d_%d", forArea, fromX, fromY, toX, toY)
}

// Smart cache cleanup with LRU strategy
func (pf *PathFinder) cleanCache() {
	pf.cache.mu.Lock()
	defer pf.cache.mu.Unlock()

	now := time.Now()

	// Remove entries from old areas
	for key, entry := range pf.cache.entries {
		if entry.forArea != pf.cache.currentArea {
			delete(pf.cache.entries, key)
		}
	}

	// LRU eviction when cache is full
	if len(pf.cache.entries) > 300 {
		type evictionCandidate struct {
			key   string
			score float64
		}

		candidates := make([]evictionCandidate, 0, len(pf.cache.entries))

		for key, entry := range pf.cache.entries {
			timeSinceAccess := now.Sub(entry.lastAccessed).Seconds()
			timeSinceCreation := now.Sub(entry.timestamp).Seconds()

			accessRecencyScore := 1000.0 / (timeSinceAccess + 1)
			frequencyMultiplier := float64(entry.accessCount)
			ageScore := math.Max(0, 100.0-timeSinceCreation/10.0)

			score := (accessRecencyScore * frequencyMultiplier) + ageScore

			candidates = append(candidates, evictionCandidate{
				key:   key,
				score: score,
			})
		}

		evictCount := len(candidates) / 4
		if evictCount < 50 {
			evictCount = 50
		}

		for i := 0; i < evictCount && len(candidates) > 0; i++ {
			lowestIdx := 0
			lowestScore := candidates[0].score

			for j := 1; j < len(candidates); j++ {
				if candidates[j].score < lowestScore {
					lowestScore = candidates[j].score
					lowestIdx = j
				}
			}

			delete(pf.cache.entries, candidates[lowestIdx].key)

			candidates[lowestIdx] = candidates[len(candidates)-1]
			candidates = candidates[:len(candidates)-1]
		}
	}
}

// Check and handle area changes
func (pf *PathFinder) checkAreaChange() {
	currentArea := pf.data.PlayerUnit.Area

	pf.cache.mu.RLock()
	needsUpdate := currentArea != pf.cache.currentArea
	pf.cache.mu.RUnlock()

	if needsUpdate {
		pf.cache.mu.Lock()
		if currentArea != pf.cache.currentArea {
			oldArea := pf.cache.currentArea
			pf.cache.currentArea = currentArea

			for key, entry := range pf.cache.entries {
				if entry.forArea == oldArea {
					delete(pf.cache.entries, key)
				}
			}
		}
		pf.cache.mu.Unlock()

		// Invalidate grid cache on area change
		pf.gridCache.mu.Lock()
		pf.gridCache.preparedGrid = nil
		pf.gridCache.forArea = currentArea
		pf.gridCache.mu.Unlock()
	}
}

// Get or create prepared grid with objects/monsters added to collision
// This caches the expensive grid preparation (copying + adding obstacles) for 500ms
// Dramatically reduces CPU when path cache misses but grid is still valid
func (pf *PathFinder) getPreparedGrid() *game.Grid {
	currentArea := pf.data.PlayerUnit.Area

	// Check if we have a valid cached grid
	pf.gridCache.mu.RLock()
	if pf.gridCache.preparedGrid != nil &&
		pf.gridCache.forArea == currentArea &&
		time.Since(pf.gridCache.timestamp) < 500*time.Millisecond {
		// Use cached grid
		cached := pf.gridCache.preparedGrid
		pf.gridCache.mu.RUnlock()
		return cached
	}
	pf.gridCache.mu.RUnlock()

	// Need to prepare new grid
	a := pf.data.AreaData
	grid := a.Grid.Copy()

	// Special handling for Arcane Sanctuary
	if pf.data.PlayerUnit.Area == area.ArcaneSanctuary && pf.data.CanTeleport() {
		for y := 0; y < len(grid.CollisionGrid); y++ {
			for x := 0; x < len(grid.CollisionGrid[y]); x++ {
				if grid.CollisionGrid[y][x] == game.CollisionTypeNonWalkable {
					grid.CollisionGrid[y][x] = game.CollisionTypeLowPriority
				}
			}
		}
	}

	// Lut Gholein fix
	if a.Area == area.LutGholein {
		a.CollisionGrid[13][210] = game.CollisionTypeNonWalkable
	}

	// Add objects to collision grid
	for _, o := range pf.data.AreaData.Objects {
		if !grid.IsWalkable(o.Position) {
			continue
		}
		relativePos := grid.RelativePosition(o.Position)
		grid.CollisionGrid[relativePos.Y][relativePos.X] = game.CollisionTypeObject

		// Simplified loop - only mark immediate surroundings
		for i := -2; i <= 2; i++ {
			for j := -2; j <= 2; j++ {
				if i == 0 && j == 0 {
					continue
				}
				ny, nx := relativePos.Y+i, relativePos.X+j
				if ny >= 0 && ny < len(grid.CollisionGrid) && nx >= 0 && nx < len(grid.CollisionGrid[ny]) {
					if grid.CollisionGrid[ny][nx] == game.CollisionTypeWalkable {
						grid.CollisionGrid[ny][nx] = game.CollisionTypeLowPriority
					}
				}
			}
		}
	}

	// Add monsters to collision grid
	for _, m := range pf.data.Monsters {
		if !grid.IsWalkable(m.Position) {
			continue
		}
		relativePos := grid.RelativePosition(m.Position)
		grid.CollisionGrid[relativePos.Y][relativePos.X] = game.CollisionTypeMonster
	}

	// Barricade towers in Act 5
	if a.Area == area.FrigidHighlands || a.Area == area.FrozenTundra || a.Area == area.ArreatPlateau {
		for _, n := range pf.data.NPCs {
			if n.ID != npc.BarricadeTower || len(n.Positions) == 0 {
				continue
			}
			relativePos := grid.RelativePosition(n.Positions[0])

			for dy := -2; dy <= 2; dy++ {
				for dx := -2; dx <= 2; dx++ {
					ty, tx := relativePos.Y+dy, relativePos.X+dx
					if ty >= 0 && ty < len(grid.CollisionGrid) && tx >= 0 && tx < len(grid.CollisionGrid[ty]) {
						grid.CollisionGrid[ty][tx] = game.CollisionTypeNonWalkable
					}
				}
			}
		}
	}

	// Cache the prepared grid
	pf.gridCache.mu.Lock()
	pf.gridCache.preparedGrid = grid
	pf.gridCache.forArea = currentArea
	pf.gridCache.timestamp = time.Now()
	pf.gridCache.mu.Unlock()

	return grid
}

// GetPath calculates path from current player position to target
// Uses path cache first, then grid cache, then full calculation
func (pf *PathFinder) GetPath(to data.Position) (Path, int, bool) {
	pf.checkAreaChange()

	if path, distance, found := pf.GetPathFrom(pf.data.PlayerUnit.Position, to); found {
		return path, distance, true
	}

	if walkableTo, found := pf.findNearbyWalkablePosition(to); found {
		return pf.GetPathFrom(pf.data.PlayerUnit.Position, walkableTo)
	}

	return nil, 0, false
}

// GetPathFrom calculates path from specific position to target
// Implements two-level caching: path cache + grid cache
func (pf *PathFinder) GetPathFrom(from, to data.Position) (Path, int, bool) {
	currentArea := pf.data.PlayerUnit.Area

	// Check cache first
	cacheKey := pf.getCacheKey(from, to, currentArea)

	pf.cache.mu.Lock()
	if cached, exists := pf.cache.entries[cacheKey]; exists {
		cached.lastAccessed = time.Now()
		cached.accessCount++
		path := cached.path
		distance := cached.distance
		pf.cache.mu.Unlock()
		return path, distance, true
	}
	pf.cache.mu.Unlock()

	// Cache miss - use prepared grid
	a := pf.data.AreaData

	// Use cached prepared grid instead of rebuilding
	var grid *game.Grid
	if a.IsInside(to) {
		grid = pf.getPreparedGrid()
	} else {
		// Destination outside current area - need merged grid
		expandedGrid, err := pf.mergeGrids(to)
		if err != nil {
			return nil, 0, false
		}
		grid = expandedGrid
	}

	from = grid.RelativePosition(from)
	to = grid.RelativePosition(to)

	// Calculate path using A*
	path, distance, found := astar.CalculatePath(grid, from, to)

	// Store in cache if found
	if found {
		now := time.Now()
		pf.cache.mu.Lock()
		pf.cache.entries[cacheKey] = &pathCacheEntry{
			path:         path,
			distance:     distance,
			timestamp:    now,
			lastAccessed: now,
			accessCount:  1,
			forArea:      currentArea,
		}

		shouldClean := len(pf.cache.entries) > 300
		pf.cache.mu.Unlock()

		if shouldClean {
			go pf.cleanCache()
		}
	}

	if config.Koolo.Debug.RenderMap {
		pf.renderMap(grid, from, to, path)
	}

	return path, distance, found
}

// mergeGrids combines current area grid with adjacent area grid
// Used when pathfinding to destination in adjacent area
func (pf *PathFinder) mergeGrids(to data.Position) (*game.Grid, error) {
	for _, a := range pf.data.AreaData.AdjacentLevels {
		destination := pf.data.Areas[a.Area]
		if destination.IsInside(to) {
			origin := pf.data.AreaData

			endX1 := origin.OffsetX + len(origin.Grid.CollisionGrid[0])
			endY1 := origin.OffsetY + len(origin.Grid.CollisionGrid)
			endX2 := destination.OffsetX + len(destination.Grid.CollisionGrid[0])
			endY2 := destination.OffsetY + len(destination.Grid.CollisionGrid)

			minX := min(origin.OffsetX, destination.OffsetX)
			minY := min(origin.OffsetY, destination.OffsetY)
			maxX := max(endX1, endX2)
			maxY := max(endY1, endY2)

			width := maxX - minX
			height := maxY - minY

			resultGrid := make([][]game.CollisionType, height)
			for i := range resultGrid {
				resultGrid[i] = make([]game.CollisionType, width)
			}

			copyGrid(resultGrid, origin.CollisionGrid, origin.OffsetX-minX, origin.OffsetY-minY)
			copyGrid(resultGrid, destination.CollisionGrid, destination.OffsetX-minX, destination.OffsetY-minY)

			grid := game.NewGrid(resultGrid, minX, minY)

			return grid, nil
		}
	}

	return nil, fmt.Errorf("destination grid not found")
}

// copyGrid copies source collision grid into destination at specified offset
// Helper function for grid merging operations
func copyGrid(dest [][]game.CollisionType, src [][]game.CollisionType, offsetX, offsetY int) {
	for y := 0; y < len(src); y++ {
		for x := 0; x < len(src[0]); x++ {
			dest[offsetY+y][offsetX+x] = src[y][x]
		}
	}
}

// GetClosestWalkablePath finds path to nearest walkable position near destination
// Useful when destination itself is not walkable
func (pf *PathFinder) GetClosestWalkablePath(dest data.Position) (Path, int, bool) {
	return pf.GetClosestWalkablePathFrom(pf.data.PlayerUnit.Position, dest)
}

// GetClosestWalkablePathFrom finds path from specific position to nearest walkable position near destination
// Searches in expanding squares around destination
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
					if cgX > 0 && cgY > 0 && a.Height > cgY && a.Width > cgX && a.CollisionGrid[cgY][cgX] == game.CollisionTypeWalkable {
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

// findNearbyWalkablePosition searches for nearest walkable tile around target
// Used as fallback when direct path calculation fails
func (pf *PathFinder) findNearbyWalkablePosition(target data.Position) (data.Position, bool) {
	for radius := 1; radius <= 3; radius++ {
		for x := -radius; x <= radius; x++ {
			for y := -radius; y <= radius; y++ {
				if x == 0 && y == 0 {
					continue
				}
				pos := data.Position{X: target.X + x, Y: target.Y + y}
				if pf.data.AreaData.IsWalkable(pos) {
					return pos, true
				}
			}
		}
	}
	return data.Position{}, false
}
