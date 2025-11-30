package DebugOverlay

import (
	stdctx "context"
	"embed"
	"encoding/json"
	"errors"
	"fmt"
	"io/fs"
	"log/slog"
	"math"
	"net"
	"net/http"
	"sync"
	"sync/atomic"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	botctx "github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/inkeliz/gowebview"
)

//go:embed assets/*
var overlayAssets embed.FS

const (
	overlayScale        = 2.5
	overlayRange        = 120.0
	overlayWindowWidth  = int64(760)
	overlayWindowHeight = int64(720)
)

// DebugOverlay opens a lightweight window that polls the bot context and renders
// nearby rooms/objects for debugging purposes. Mainly useful for pathfinding and
// navigation issues.
type DebugOverlay struct {
	ctx    *botctx.Status
	logger *slog.Logger

	running atomic.Bool

	mu     sync.Mutex
	server *overlayServer
	window *overlayWindow
}

type overlayServer struct {
	overlay  *DebugOverlay
	listener net.Listener
	server   *http.Server
}

type overlayWindow struct {
	view gowebview.WebView
}

func newOverlayServer(po *DebugOverlay) (*overlayServer, error) {
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		return nil, fmt.Errorf("listen overlay: %w", err)
	}

	assetsFS, err := fs.Sub(overlayAssets, "assets")
	if err != nil {
		return nil, fmt.Errorf("load overlay assets: %w", err)
	}
	fileServer := http.FileServer(http.FS(assetsFS))

	mux := http.NewServeMux()
	srv := &overlayServer{
		overlay:  po,
		listener: listener,
		server: &http.Server{
			Handler:      mux,
			ReadTimeout:  5 * time.Second,
			WriteTimeout: 5 * time.Second,
		},
	}

	mux.Handle("/", fileServer)
	mux.HandleFunc("/favicon.ico", func(w http.ResponseWriter, _ *http.Request) {
		w.WriteHeader(http.StatusNoContent)
	})
	mux.HandleFunc("/data", srv.handleData)

	return srv, nil
}

func (s *overlayServer) start() error {
	if s == nil {
		return errors.New("nil overlay server")
	}

	go func() {
		if err := s.server.Serve(s.listener); err != nil && !errors.Is(err, http.ErrServerClosed) {
			s.overlay.logger.Error("Overlay server stopped", slog.Any("error", err))
		}
	}()

	return nil
}

func (s *overlayServer) stop() {
	if s == nil {
		return
	}

	ctx, cancel := stdctx.WithTimeout(stdctx.Background(), time.Second)
	defer cancel()
	_ = s.server.Shutdown(ctx)
	_ = s.listener.Close()
}

func (s *overlayServer) url() string {
	if s == nil || s.listener == nil {
		return ""
	}
	return fmt.Sprintf("http://%s/", s.listener.Addr().String())
}

func (s *overlayServer) handleData(w http.ResponseWriter, _ *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	w.Header().Set("Cache-Control", "no-store")

	payload := s.overlay.collectData()
	if err := json.NewEncoder(w).Encode(payload); err != nil {
		s.overlay.logger.Debug("Failed to encode overlay payload", slog.Any("error", err))
	}
}

func newOverlayWindow(characterName, url string) (*overlayWindow, error) {
	windowSize := &gowebview.Point{X: overlayWindowWidth, Y: overlayWindowHeight}
	w, err := gowebview.New(&gowebview.Config{
		URL: url,
		WindowConfig: &gowebview.WindowConfig{
			Title: fmt.Sprintf("Debug Overlay - %s", characterName),
			Size:  windowSize,
		},
	})
	if err != nil {
		return nil, fmt.Errorf("create overlay window: %w", err)
	}

	w.SetSize(windowSize, gowebview.HintFixed)

	return &overlayWindow{view: w}, nil
}

func (w *overlayWindow) run(onClosed func()) {
	if w == nil || w.view == nil {
		return
	}

	go func() {
		defer func() {
			w.view.Destroy()
		}()

		w.view.Run()
		if onClosed != nil {
			onClosed()
		}
	}()
}

func (w *overlayWindow) close() {
	if w == nil || w.view == nil {
		return
	}
	w.view.Terminate()
}

var overlayInstances sync.Map

func Instance(status *botctx.Status) *DebugOverlay {
	if status == nil || status.Context == nil {
		return nil
	}

	key := status.Context
	if existing, ok := overlayInstances.Load(key); ok {
		po := existing.(*DebugOverlay)
		po.UpdateStatus(status)
		return po
	}

	po := NewDebugOverlay(status)
	overlayInstances.Store(key, po)
	return po
}

type overlayPoint struct {
	X    float64 `json:"x"`
	Y    float64 `json:"y"`
	Size float64 `json:"size"`
	Kind string  `json:"kind"`
}

type overlayPayload struct {
	Scale    float64        `json:"scale"`
	Tiles    []overlayTile  `json:"tiles"`
	Path     []overlayPoint `json:"path"`
	Objects  []overlayPoint `json:"objects"`
	Monsters []overlayPoint `json:"monsters"`
	Meta     string         `json:"meta"`
}

type overlayTile struct {
	X    float64 `json:"x"`
	Y    float64 `json:"y"`
	Type int     `json:"type"`
}

func NewDebugOverlay(ctx *botctx.Status) *DebugOverlay {
	return &DebugOverlay{
		ctx: ctx,
		logger: ctx.Logger.With(
			slog.String("component", "DebugOverlay"),
			slog.String("supervisor", ctx.Name),
		),
	}
}

func (po *DebugOverlay) UpdateStatus(ctx *botctx.Status) {
	if ctx == nil {
		return
	}

	po.ctx = ctx
	po.logger = ctx.Logger.With(
		slog.String("component", "DebugOverlay"),
		slog.String("supervisor", ctx.Name),
	)
}

func (po *DebugOverlay) Toggle() error {
	if po.running.Load() {
		po.Stop()
		return nil
	}
	return po.Start()
}

func (po *DebugOverlay) IsRunning() bool {
	if po == nil {
		return false
	}
	return po.running.Load()
}

func (po *DebugOverlay) Start() error {
	if !po.running.CompareAndSwap(false, true) {
		return nil
	}

	server, err := newOverlayServer(po)
	if err != nil {
		po.running.Store(false)
		return err
	}

	po.mu.Lock()
	po.server = server
	po.mu.Unlock()

	if err := server.start(); err != nil {
		po.running.Store(false)
		return err
	}

	window, err := newOverlayWindow(po.ctx.Name, server.url())
	if err != nil {
		po.stopServer()
		po.running.Store(false)
		return err
	}

	po.setWindow(window)
	window.run(po.windowClosed)

	po.logger.Info("Overlay window opened", slog.String("url", server.url()))
	return nil
}

func (po *DebugOverlay) Stop() {
	if !po.running.CompareAndSwap(true, false) {
		return
	}

	po.logger.Info("Stopping Overlay")
	po.stopServer()
	po.stopWindow()
}

func (po *DebugOverlay) setWindow(w *overlayWindow) {
	po.mu.Lock()
	defer po.mu.Unlock()
	po.window = w
}

func (po *DebugOverlay) stopWindow() {
	po.mu.Lock()
	w := po.window
	po.window = nil
	po.mu.Unlock()

	if w != nil {
		w.close()
	}
}

func (po *DebugOverlay) windowClosed() {
	if po.running.CompareAndSwap(true, false) {
		po.stopServer()
	}
}

func (po *DebugOverlay) stopServer() {
	po.mu.Lock()
	server := po.server
	po.server = nil
	po.mu.Unlock()

	if server != nil {
		server.stop()
	}
}

func (po *DebugOverlay) collectData() overlayPayload {
	dataSnapshot := po.ctx.Data
	payload := overlayPayload{
		Scale: overlayScale,
		Meta:  "Waiting for game state...",
	}

	if dataSnapshot == nil || !dataSnapshot.IsIngame {
		return payload
	}

	player := dataSnapshot.PlayerUnit.Position

	objects := make([]overlayPoint, 0, len(dataSnapshot.Objects))
	for _, obj := range dataSnapshot.Objects {
		dx := obj.Position.X - player.X
		dy := obj.Position.Y - player.Y
		if !withinRange(dx, dy) {
			continue
		}

		objects = append(objects, overlayPoint{
			X:    float64(dx),
			Y:    float64(dy),
			Size: 2,
			Kind: fmt.Sprintf("%v", obj.Name),
		})

		if len(objects) >= 80 {
			break
		}
	}

	monsters := make([]overlayPoint, 0, len(dataSnapshot.Monsters))
	for _, monster := range dataSnapshot.Monsters.Enemies() {
		dx := monster.Position.X - player.X
		dy := monster.Position.Y - player.Y
		if !withinRange(dx, dy) {
			continue
		}

		size := 2.5
		if monster.Type == data.MonsterTypeChampion || monster.Type == data.MonsterTypeUnique || monster.Type == data.MonsterTypeSuperUnique {
			size = 3.5
		}

		monsters = append(monsters, overlayPoint{
			X:    float64(dx),
			Y:    float64(dy),
			Size: size,
			Kind: string(monster.Type),
		})

		if len(monsters) >= 80 {
			break
		}
	}

	tiles := po.collectTiles(dataSnapshot, player)
	payload.Tiles = tiles
	payload.Path = po.collectPath(player)
	payload.Objects = objects
	payload.Monsters = monsters
	payload.Meta = fmt.Sprintf("%s | tiles:%d objects:%d monsters:%d", dataSnapshot.PlayerUnit.Area.Area().Name, len(tiles), len(objects), len(monsters))

	return payload
}

func (po *DebugOverlay) collectTiles(dataSnapshot *game.Data, player data.Position) []overlayTile {
	grid := dataSnapshot.AreaData.Grid
	if grid == nil {
		return nil
	}

	tiles := make([]overlayTile, 0, 5000)
	rangeSize := int(overlayRange)

	startX := max(player.X-rangeSize, grid.OffsetX)
	endX := min(player.X+rangeSize, grid.OffsetX+grid.Width-1)
	startY := max(player.Y-rangeSize, grid.OffsetY)
	endY := min(player.Y+rangeSize, grid.OffsetY+grid.Height-1)

	for worldY := startY; worldY <= endY; worldY++ {
		gridY := worldY - grid.OffsetY
		row := grid.CollisionGrid[gridY]
		for worldX := startX; worldX <= endX; worldX++ {
			gridX := worldX - grid.OffsetX
			cell := row[gridX]
			switch cell {
			case game.CollisionTypeWalkable,
				game.CollisionTypeLowPriority,
				game.CollisionTypeNonWalkable,
				game.CollisionTypeTeleportOver,
				game.CollisionTypeObject,
				game.CollisionTypeThickened:
			default:
				continue
			}
			tiles = append(tiles, overlayTile{
				X:    float64(worldX - player.X),
				Y:    float64(worldY - player.Y),
				Type: int(cell),
			})
		}
	}

	return tiles
}

func (po *DebugOverlay) collectPath(player data.Position) []overlayPoint {
	if po.ctx.PathFinder == nil {
		return nil
	}

	lastPath, ok := po.ctx.PathFinder.LastPathDebug()
	if !ok || len(lastPath.Path) == 0 {
		return nil
	}

	points := make([]overlayPoint, 0, len(lastPath.Path))
	for _, node := range lastPath.Path {
		dx := node.X - player.X
		dy := node.Y - player.Y
		if !withinRange(dx, dy) {
			continue
		}
		points = append(points, overlayPoint{
			X: float64(dx),
			Y: float64(dy),
		})
	}

	return points
}

func withinRange(dx, dy int) bool {
	return math.Abs(float64(dx)) <= overlayRange && math.Abs(float64(dy)) <= overlayRange
}
