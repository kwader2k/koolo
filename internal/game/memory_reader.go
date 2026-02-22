package game

import (
	"errors"
	"fmt"
	"log/slog"
	"strconv"
	"sync"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/memory"
	"github.com/hectorgimenez/d2go/pkg/utils"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/game/map_client"
	"github.com/lxn/win"
	"golang.org/x/sync/errgroup"
)

type MemoryReader struct {
	cfg *config.CharacterCfg
	*memory.GameReader
	mapSeed        uint
	HWND           win.HWND
	WindowLeftX    int
	WindowTopY     int
	GameAreaSizeX  int
	GameAreaSizeY  int
	supervisorName string
	cachedMapData  map[area.ID]AreaData
	mapDataMu      sync.RWMutex // Protects cachedMapData from concurrent access
	logger         *slog.Logger
}

func NewGameReader(cfg *config.CharacterCfg, supervisorName string, pid uint32, window win.HWND, logger *slog.Logger) (*MemoryReader, error) {
	process, err := memory.NewProcessForPID(pid)
	if err != nil {
		return nil, err
	}

	gr := &MemoryReader{
		GameReader:     memory.NewGameReader(process),
		HWND:           window,
		supervisorName: supervisorName,
		cfg:            cfg,
		logger:         logger,
	}

	gr.updateWindowPositionData()

	return gr, nil
}

func (gd *MemoryReader) MapSeed() uint {
	return gd.mapSeed
}

// ClearMapData releases cached map data to free memory when not in game
func (gd *MemoryReader) ClearMapData() {
	gd.mapDataMu.Lock()
	defer gd.mapDataMu.Unlock()
	gd.cachedMapData = nil
}

func (gd *MemoryReader) FetchMapData() error {
	// Clear old map data before fetching new data to allow GC to reclaim memory
	gd.mapDataMu.Lock()
	gd.cachedMapData = nil
	gd.mapDataMu.Unlock()

	d := gd.GameReader.GetData()
	gd.mapSeed, _ = gd.getMapSeed(d.PlayerUnit.Address)
	t := time.Now()
	cfg, _ := config.GetCharacter(gd.supervisorName)
	gd.logger.Debug("Fetching map data...", slog.Uint64("seed", uint64(gd.mapSeed)), slog.String("difficulty", string(cfg.Game.Difficulty)))

	mapData, err := map_client.GetMapData(strconv.Itoa(int(gd.mapSeed)), cfg.Game.Difficulty)
	if err != nil {
		return fmt.Errorf("error fetching map data: %w", err)
	}

	areas := make(map[area.ID]AreaData)
	var mu sync.Mutex
	g := errgroup.Group{}
	for _, lvl := range mapData {
		g.Go(func() error {
			cg := lvl.CollisionGrid()
			resultGrid := make([][]CollisionType, lvl.Size.Height)
			for i := range resultGrid {
				resultGrid[i] = make([]CollisionType, lvl.Size.Width)
			}

			for y := 0; y < lvl.Size.Height; y++ {
				for x := 0; x < lvl.Size.Width; x++ {
					if cg[y][x] {
						resultGrid[y][x] = CollisionTypeWalkable
					} else {
						resultGrid[y][x] = CollisionTypeNonWalkable
					}
				}
			}

			npcs, exits, objects, rooms := lvl.NPCsExitsAndObjects()
			areaID := area.ID(lvl.ID)
			// Process teleportable tiles on 2D array before flattening
			if !areaID.IsTown() {
				gd.TeleportPostProcess(&resultGrid, lvl.Size.Width, lvl.Size.Height)
			}
			grid := NewGrid(resultGrid, lvl.Offset.X, lvl.Offset.Y, false)

			// Apply collision thickening to all non-town areas
			if !areaID.IsTown() {
				thickenCollisions(grid)
				// Re-open exits after thickening
				drillExits(grid, extractExitPositions(exits))
			} else {
				// Apply thickening to towns but don't drill exits
				thickenCollisions(grid)
			}

			mu.Lock()
			areas[areaID] = AreaData{
				Area:           area.ID(lvl.ID),
				Name:           lvl.Name,
				NPCs:           npcs,
				AdjacentLevels: exits,
				Objects:        objects,
				Rooms:          rooms,
				Grid:           grid,
			}
			mu.Unlock()

			return nil
		})
	}

	_ = g.Wait()

	gd.mapDataMu.Lock()
	gd.cachedMapData = areas
	gd.mapDataMu.Unlock()
	gd.logger.Debug("Fetch completed", slog.Int64("ms", time.Since(t).Milliseconds()))

	return nil
}

func (gd *MemoryReader) TeleportPostProcess(grid *[][]CollisionType, width int, height int) {
	for y := 1; y < height-1; y++ {
		for x := 1; x < width-1; x++ {
			gridValue := (*grid)[y][x]
			if gridValue == CollisionTypeNonWalkable || gridValue == CollisionTypeTeleportOver {
				gd.CheckAndAdjustForTeleport(grid, x, y, width, height, 10)
			}
		}
	}
}

func (gd *MemoryReader) CheckAndAdjustForTeleport(grid *[][]CollisionType, xPos int, yPos int, width int, height int, dist int) {
	minX := max(xPos-dist, 0)
	maxX := min(xPos+dist, width-1)
	minY := max(yPos-dist, 0)
	maxY := min(yPos+dist, height-1)

	if gd.CheckForWalkable(grid, minX, yPos, xPos-1, yPos) && gd.CheckForWalkable(grid, xPos+1, yPos, maxX, yPos) {
		gd.SetTeleportable(grid, minX, yPos, maxX, yPos)
	}

	if gd.CheckForWalkable(grid, xPos, minY, xPos, yPos-1) && gd.CheckForWalkable(grid, xPos, yPos+1, xPos, maxY) {
		gd.SetTeleportable(grid, xPos, minY, xPos, maxY)
	}

	diagDist := dist / 2
	minX = max(xPos-diagDist, 0)
	maxX = min(xPos+diagDist, width-1)
	minY = max(yPos-diagDist, 0)
	maxY = min(yPos+diagDist, height-1)

	if gd.CheckForWalkableDiagonal(grid, minX, minY, xPos-1, yPos-1, 1) && gd.CheckForWalkableDiagonal(grid, xPos+1, yPos+1, maxX, maxY, 1) {
		gd.SetTeleportableDiagonal(grid, minX, minY, maxX, maxY, 1, 1)
	}

	if gd.CheckForWalkableDiagonal(grid, minX, maxY, xPos-1, yPos+1, -1) && gd.CheckForWalkableDiagonal(grid, xPos+1, yPos-1, maxX, minY, -1) {
		gd.SetTeleportableDiagonal(grid, minX, maxY, maxX, minY, 1, -1)
	}
}

func (gd *MemoryReader) CheckForWalkable(grid *[][]CollisionType, xStart int, yStart int, xEnd int, yEnd int) bool {
	for x := xStart; x <= xEnd; x++ {
		for y := yStart; y <= yEnd; y++ {
			if (*grid)[y][x] == CollisionTypeWalkable {
				return true
			}
		}
	}
	return false
}

func (gd *MemoryReader) CheckForWalkableDiagonal(grid *[][]CollisionType, xStart int, yStart int, xEnd int, yEnd int, yStep int) bool {
	y := yStart
	minY := min(yStart, yEnd)
	maxY := max(yStart, yEnd)
	for x := xStart; x <= xEnd && y >= minY && y <= maxY; {
		if (*grid)[y][x] == CollisionTypeWalkable {
			return true
		}
		x += 1
		y += yStep
	}
	return false
}

func (gd *MemoryReader) SetTeleportable(grid *[][]CollisionType, xStart int, yStart int, xEnd int, yEnd int) {
	for x := xStart; x <= xEnd; x++ {
		for y := yStart; y <= yEnd; y++ {
			if (*grid)[y][x] == CollisionTypeNonWalkable {
				(*grid)[y][x] = CollisionTypeTeleportOver
			}
		}
	}
}

func (gd *MemoryReader) SetTeleportableDiagonal(grid *[][]CollisionType, xStart int, yStart int, xEnd int, yEnd int, xStep int, yStep int) {
	y := yStart
	minY := min(yStart, yEnd)
	maxY := max(yStart, yEnd)
	for x := xStart; x <= xEnd && y >= minY && y <= maxY; {
		if (*grid)[y][x] == CollisionTypeNonWalkable {
			(*grid)[y][x] = CollisionTypeTeleportOver
		}
		x += 1
		y += yStep
	}
}

func (gd *MemoryReader) updateWindowPositionData() {
	pos := win.WINDOWPLACEMENT{}
	point := win.POINT{}
	win.ClientToScreen(gd.HWND, &point)
	win.GetWindowPlacement(gd.HWND, &pos)

	gd.WindowLeftX = int(point.X)
	gd.WindowTopY = int(point.Y)
	gd.GameAreaSizeX = int(pos.RcNormalPosition.Right) - gd.WindowLeftX - 9
	gd.GameAreaSizeY = int(pos.RcNormalPosition.Bottom) - gd.WindowTopY - 9
}

func (gd *MemoryReader) GetData() Data {
	d := gd.GameReader.GetData()

	// Take a snapshot of cachedMapData under lock to avoid race with ClearMapData
	gd.mapDataMu.RLock()
	cachedData := gd.cachedMapData
	gd.mapDataMu.RUnlock()

	currentArea, ok := cachedData[d.PlayerUnit.Area]
	if ok {
		// This hacky thing is because sometimes if the objects are far away we can not fetch them, basically WP.
		memObjects := gd.Objects(d.PlayerUnit.Position, d.HoverData)
		for _, clientObject := range currentArea.Objects {
			found := false
			for _, obj := range memObjects {
				// Only consider it a duplicate if same name AND same position
				if obj.Name == clientObject.Name && obj.Position.X == clientObject.Position.X && obj.Position.Y == clientObject.Position.Y {
					found = true
					break
				}
			}
			if !found {
				memObjects = append(memObjects, clientObject)
			}
		}

		d.AreaOrigin = data.Position{X: currentArea.OffsetX, Y: currentArea.OffsetY}
		d.NPCs = currentArea.NPCs
		d.AdjacentLevels = currentArea.AdjacentLevels
		d.Rooms = currentArea.Rooms
		d.Objects = memObjects
	}

	var cfgCopy config.CharacterCfg
	if gd.cfg != nil {
		cfgCopy = *gd.cfg
	}

	return Data{
		Areas:        cachedData,
		AreaData:     currentArea,
		Data:         d,
		CharacterCfg: cfgCopy,
		ExpChar:      gd.GetExpChar(),
	}
}

func (gd *MemoryReader) getMapSeed(playerUnit uintptr) (uint, error) {
	actPtr := uintptr(gd.Process.ReadUInt(playerUnit+0x20, memory.Uint64))
	//actMiscPtr := uintptr(gd.Process.ReadUInt(actPtr+0x78, memory.Uint64))
	actMiscPtr := uintptr(gd.Process.ReadUInt(actPtr+0x70, memory.Uint64))

	dwInitSeedHash1 := gd.Process.ReadUInt(actMiscPtr+0x840, memory.Uint32)
	//dwInitSeedHash2 := uintptr(gd.Process.ReadUInt(actMiscPtr+0x844, memory.Uint32))
	//dwEndSeedHash1 := gd.Process.ReadUInt(actMiscPtr+0x868, memory.Uint32)
	dwEndSeedHash1 := gd.Process.ReadUInt(actMiscPtr+0x860, memory.Uint32)

	mapSeed, found := utils.GetMapSeed(dwInitSeedHash1, dwEndSeedHash1)
	if !found {
		return 0, errors.New("error calculating map seed")
	}

	return mapSeed, nil
}

// extractExitPositions extracts positions from exit level data for drill exit purposes
func extractExitPositions(exits []data.Level) []data.Position {
	positions := make([]data.Position, 0, len(exits))
	for _, exit := range exits {
		if exit.Position.X != 0 || exit.Position.Y != 0 {
			positions = append(positions, exit.Position)
		}
	}
	return positions
}
