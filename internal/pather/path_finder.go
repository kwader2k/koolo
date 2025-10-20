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
	forArea      area.ID // Track which area this path is for
}

type pathCache struct {
	entries     map[string]*pathCacheEntry
	currentArea area.ID
	mu          sync.RWMutex
}

type PathFinder struct {
	gr   *game.MemoryReader
	data *game.Data
	hid  *game.HID
	cfg  *config.CharacterCfg
	// Add smart path cache
	cache *pathCache
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
	}
}

// Generate cache key from area + quantized positions
// Quantization reduces cache size by grouping nearby positions
func (pf *PathFinder) getCacheKey(from, to data.Position, forArea area.ID) string {
	// Quantize positions to 5-unit grid to increase cache hits
	// This means positions within 5 units share the same cache key
	fromX := (from.X / 5) * 5
	fromY := (from.Y / 5) * 5
	toX := (to.X / 5) * 5
	toY := (to.Y / 5) * 5

	return fmt.Sprintf("%d_%d_%d_%d_%d", forArea, fromX, fromY, toX, toY)
}

// Smart cache cleanup with LRU strategy
// This is the KEY improvement - we don't expire by time, we expire by usage
func (pf *PathFinder) cleanCache() {
	pf.cache.mu.Lock()
	defer pf.cache.mu.Unlock()

	now := time.Now()

	// STRATEGY 1: Remove entries from old areas immediately
	// When area changes, old paths are useless
	for key, entry := range pf.cache.entries {
		if entry.forArea != pf.cache.currentArea {
			delete(pf.cache.entries, key)
		}
	}

	// STRATEGY 2: LRU eviction when cache is full
	// Keep frequently accessed paths, remove rarely used ones
	if len(pf.cache.entries) > 300 {
		// Build list of candidates for eviction
		type evictionCandidate struct {
			key   string
			score float64 // Lower score = more likely to evict
		}

		candidates := make([]evictionCandidate, 0, len(pf.cache.entries))

		for key, entry := range pf.cache.entries {
			// Calculate eviction score based on:
			// 1. How recently accessed (more recent = higher score)
			// 2. How frequently accessed (more frequent = higher score)
			// 3. How old the entry is (older = slightly lower score)

			timeSinceAccess := now.Sub(entry.lastAccessed).Seconds()
			timeSinceCreation := now.Sub(entry.timestamp).Seconds()

			// Score formula (higher = keep, lower = evict):
			// - Recent access gives high score
			// - Multiple accesses multiply score
			// - Very old entries get penalty
			accessRecencyScore := 1000.0 / (timeSinceAccess + 1)
			frequencyMultiplier := float64(entry.accessCount)
			ageScore := math.Max(0, 100.0-timeSinceCreation/10.0)

			score := (accessRecencyScore * frequencyMultiplier) + ageScore

			candidates = append(candidates, evictionCandidate{
				key:   key,
				score: score,
			})
		}

		// Sort candidates by score (lowest first)
		// Simple selection to find bottom 25% for eviction
		evictCount := len(candidates) / 4 // Remove 25% of cache
		if evictCount < 50 {
			evictCount = 50 // Minimum 50 entries removed
		}

		// Find entries with lowest scores
		for i := 0; i < evictCount && len(candidates) > 0; i++ {
			lowestIdx := 0
			lowestScore := candidates[0].score

			for j := 1; j < len(candidates); j++ {
				if candidates[j].score < lowestScore {
					lowestScore = candidates[j].score
					lowestIdx = j
				}
			}

			// Evict the lowest score entry
			delete(pf.cache.entries, candidates[lowestIdx].key)

			// Remove from candidates list
			candidates[lowestIdx] = candidates[len(candidates)-1]
			candidates = candidates[:len(candidates)-1]
		}
	}
}

// Check and handle area changes
func (pf *PathFinder) checkAreaChange() {
	currentArea := pf.data.PlayerUnit.Area

	pf.cache.mu.RLock()
	if currentArea != pf.cache.currentArea {
		pf.cache.mu.RUnlock()

		// Area changed - clear cache for old area
		pf.cache.mu.Lock()
		if currentArea != pf.cache.currentArea {
			// Double-check after acquiring write lock
			oldArea := pf.cache.currentArea
			pf.cache.currentArea = currentArea

			// Remove all entries from old area
			for key, entry := range pf.cache.entries {
				if entry.forArea == oldArea {
					delete(pf.cache.entries, key)
				}
			}
		}
		pf.cache.mu.Unlock()
	} else {
		pf.cache.mu.RUnlock()
	}
}

func (pf *PathFinder) GetPath(to data.Position) (Path, int, bool) {
	// Check for area changes first
	pf.checkAreaChange()

	// First try direct path
	if path, distance, found := pf.GetPathFrom(pf.data.PlayerUnit.Position, to); found {
		return path, distance, true
	}

	// If direct path fails, try to find nearby walkable position
	if walkableTo, found := pf.findNearbyWalkablePosition(to); found {
		return pf.GetPathFrom(pf.data.PlayerUnit.Position, walkableTo)
	}

	return nil, 0, false
}

func (pf *PathFinder) GetPathFrom(from, to data.Position) (Path, int, bool) {
	currentArea := pf.data.PlayerUnit.Area

	// Check cache first
	cacheKey := pf.getCacheKey(from, to, currentArea)

	pf.cache.mu.Lock()
	if cached, exists := pf.cache.entries[cacheKey]; exists {
		// Cache hit - update access statistics
		cached.lastAccessed = time.Now()
		cached.accessCount++
		path := cached.path
		distance := cached.distance
		pf.cache.mu.Unlock()
		return path, distance, true
	}
	pf.cache.mu.Unlock()

	// Cache miss - calculate path
	a := pf.data.AreaData

	// We don't want to modify the original grid
	grid := a.Grid.Copy()

	// Special handling for Arcane Sanctuary (to allow pathing with platforms)
	if pf.data.PlayerUnit.Area == area.ArcaneSanctuary && pf.data.CanTeleport() {
		// Make all non-walkable tiles into low priority tiles for teleport pathing
		for y := 0; y < len(grid.CollisionGrid); y++ {
			for x := 0; x < len(grid.CollisionGrid[y]); x++ {
				if grid.CollisionGrid[y][x] == game.CollisionTypeNonWalkable {
					grid.CollisionGrid[y][x] = game.CollisionTypeLowPriority
				}
			}
		}
	}
	// Lut Gholein map is a bit bugged, we should close this fake path to avoid pathing issues
	if a.Area == area.LutGholein {
		a.CollisionGrid[13][210] = game.CollisionTypeNonWalkable
	}

	if !a.IsInside(to) {
		expandedGrid, err := pf.mergeGrids(to)
		if err != nil {
			return nil, 0, false
		}
		grid = expandedGrid
	}

	from = grid.RelativePosition(from)
	to = grid.RelativePosition(to)

	// Add objects to the collision grid as obstacles
	for _, o := range pf.data.AreaData.Objects {
		if !grid.IsWalkable(o.Position) {
			continue
		}
		relativePos := grid.RelativePosition(o.Position)
		grid.CollisionGrid[relativePos.Y][relativePos.X] = game.CollisionTypeObject
		for i := -2; i <= 2; i++ {
			for j := -2; j <= 2; j++ {
				if i == 0 && j == 0 {
					continue
				}
				if relativePos.Y+i < 0 || relativePos.Y+i >= len(grid.CollisionGrid) || relativePos.X+j < 0 || relativePos.X+j >= len(grid.CollisionGrid[relativePos.Y]) {
					continue
				}
				if grid.CollisionGrid[relativePos.Y+i][relativePos.X+j] == game.CollisionTypeWalkable {
					grid.CollisionGrid[relativePos.Y+i][relativePos.X+j] = game.CollisionTypeLowPriority

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
		grid.CollisionGrid[relativePos.Y][relativePos.X] = game.CollisionTypeMonster
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
					if towerY >= 0 && towerY < len(grid.CollisionGrid) &&
						towerX >= 0 && towerX < len(grid.CollisionGrid[towerY]) {
						grid.CollisionGrid[towerY][towerX] = game.CollisionTypeNonWalkable
						blockedCells++
					}
				}
			}
		}
	}

	// Calculate path using A*
	path, distance, found := astar.CalculatePath(grid, from, to)

	// Store result in cache if found
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

		// Clean cache if getting full
		shouldClean := len(pf.cache.entries) > 300
		pf.cache.mu.Unlock()

		if shouldClean {
			// Run cleanup in background to avoid blocking
			go pf.cleanCache()
		}
	}

	if config.Koolo.Debug.RenderMap {
		pf.renderMap(grid, from, to, path)
	}

	return path, distance, found
}

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

			// Let's copy both grids into the result grid
			copyGrid(resultGrid, origin.CollisionGrid, origin.OffsetX-minX, origin.OffsetY-minY)
			copyGrid(resultGrid, destination.CollisionGrid, destination.OffsetX-minX, destination.OffsetY-minY)

			grid := game.NewGrid(resultGrid, minX, minY)

			return grid, nil
		}
	}

	return nil, fmt.Errorf("destination grid not found")
}

func copyGrid(dest [][]game.CollisionType, src [][]game.CollisionType, offsetX, offsetY int) {
	for y := 0; y < len(src); y++ {
		for x := 0; x < len(src[0]); x++ {
			dest[offsetY+y][offsetX+x] = src[y][x]
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

func (pf *PathFinder) findNearbyWalkablePosition(target data.Position) (data.Position, bool) {
	// Search in expanding squares around the target position
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
