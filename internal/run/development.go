package run

import (
	"log/slog"
	"syscall"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/lxn/win"
)

// This is a "helper/development" run that keeps the reader/injector alive while skipping all automation.
// It simplifies debugging and inspecting the game state and new functionality without interference from automation routines.

// Hotkey handling using Windows API
var (
	user32DLL        = syscall.NewLazyDLL("user32.dll")
	GetAsyncKeyState = user32DLL.NewProc("GetAsyncKeyState")
)

// Virtual key codes for a few useful keys not defined in the win package
var (
	ZERO  = 0x30
	ONE   = 0x31
	TWO   = 0x32
	THREE = 0x33
	FOUR  = 0x34
	FIVE  = 0x35
	SIX   = 0x36
	SEVEN = 0x37
	EIGHT = 0x38
	NINE  = 0x39
	A     = 0x41
	B     = 0x42
	C     = 0x43
	D     = 0x44
	E     = 0x45
	F     = 0x46
	G     = 0x47
	H     = 0x48
	I     = 0x49
)

func isKeyPressed(vk int) bool {
	ret, _, _ := GetAsyncKeyState.Call(uintptr(vk))
	return ret&0x8000 != 0
}

func ctrlDown() bool {
	return isKeyPressed(win.VK_CONTROL) || isKeyPressed(win.VK_LCONTROL) || isKeyPressed(win.VK_RCONTROL)
}

func (t *DevRun) captureCursor() (screenX, screenY, gameX, gameY int) {
	var pt win.POINT
	win.GetCursorPos(&pt)
	screenX = int(pt.X)
	screenY = int(pt.Y)
	gameX = screenX - t.ctx.GameReader.WindowLeftX
	gameY = screenY - t.ctx.GameReader.WindowTopY
	return
}

func (t *DevRun) logCursor(reason string) {
	screenX, screenY, gameX, gameY := t.captureCursor()
	t.ctx.Logger.Info(
		"Cursor position",
		slog.String("reason", reason),
		slog.Int("screenX", screenX),
		slog.Int("screenY", screenY),
		slog.Int("gameX", gameX),
		slog.Int("gameY", gameY),
	)
}

func (t *DevRun) logSnapshot(reason string) {
	data := t.ctx.Data
	screenX, screenY, gameX, gameY := t.captureCursor()
	t.ctx.Logger.Debug(
		"Development snapshot",
		slog.String("reason", reason),
		slog.String("area", data.PlayerUnit.Area.Area().Name),
		slog.Int("hpPct", data.PlayerUnit.HPPercent()),
		slog.Int("mpPct", data.PlayerUnit.MPPercent()),
		slog.Int("gold", data.PlayerUnit.TotalPlayerGold()),
		slog.Int("ping", data.Game.Ping),
		slog.Int("monsters", len(data.Monsters)),
		slog.Int("cursorScreenX", screenX),
		slog.Int("cursorScreenY", screenY),
		slog.Int("cursorGameX", gameX),
		slog.Int("cursorGameY", gameY),
	)
}

func (t *DevRun) logPlayerState(reason string) {
	data := t.ctx.Data
	position := data.PlayerUnit.Position
	lvl, _ := data.PlayerUnit.FindStat(stat.Level, 0)
	t.ctx.Logger.Info(
		"Player state",
		slog.String("reason", reason),
		slog.String("area", data.PlayerUnit.Area.Area().Name),
		slog.Int("level", lvl.Value),
		slog.Int("hpPct", data.PlayerUnit.HPPercent()),
		slog.Int("mpPct", data.PlayerUnit.MPPercent()),
		slog.Int("gold", data.PlayerUnit.TotalPlayerGold()),
		slog.Int("posX", position.X),
		slog.Int("posY", position.Y),
	)
}

func (t *DevRun) fixBeltFromHotkey() {
	t.ctx.Logger.Info("Manual belt fix triggered")

	restoreCursorOverride := false
	if !t.ctx.MemoryInjector.CursorOverrideActive() {
		if err := t.ctx.MemoryInjector.EnableCursorOverride(); err != nil {
			t.ctx.Logger.Error("Failed to re-enable cursor override for manual belt fix", slog.Any("error", err))
			return
		}
		restoreCursorOverride = true
	}

	if restoreCursorOverride {
		defer func() {
			if err := t.ctx.MemoryInjector.DisableCursorOverride(); err != nil {
				t.ctx.Logger.Warn("Failed to disable cursor override after manual belt fix", slog.Any("error", err))
			} else {
				t.ctx.Logger.Info("Cursor override disabled after manual belt fix to restore manual control")
			}
		}()
	}

	if err := action.ManageBelt(); err != nil {
		t.ctx.Logger.Error("Manual belt fix failed", slog.Any("error", err))
		return
	}
	t.ctx.Logger.Info("Manual belt fix completed")
}

type DevRun struct {
	ctx *context.Status
}

type Hotkey struct {
	name    string
	combo   func() bool
	pressed bool
	action  func()
}

func NewDevRun() *DevRun {
	return &DevRun{
		ctx: context.Get(),
	}
}

func (t *DevRun) Name() string {
	return string(config.DevelopmentRun)
}

func (t *DevRun) SkipTownRoutines() bool {
	return true
}

func (t *DevRun) CheckConditions(parameters *RunParameters) SequencerResult {
	return SequencerOk
}

func (t *DevRun) Run(parameters *RunParameters) error {
	t.ctx.Logger.Info(
		"Development mode enabled: No automation will run.",
	)

	if err := t.ctx.MemoryInjector.DisableCursorOverride(); err != nil {
		t.ctx.Logger.Warn("Failed to disable cursor override for development run", slog.Any("error", err))
	} else {
		t.ctx.Logger.Info("Cursor override disabled for manual control")
	}
	defer func() {
		if err := t.ctx.MemoryInjector.EnableCursorOverride(); err != nil {
			t.ctx.Logger.Warn("Failed to re-enable cursor override after development run", slog.Any("error", err))
		}
	}()

	refreshTicker := time.NewTicker(250 * time.Millisecond)
	defer refreshTicker.Stop()

	lastArea := t.ctx.Data.PlayerUnit.Area

	hotkeys := []*Hotkey{
		{
			name: "Ctrl+1 Cursor",
			combo: func() bool {
				return ctrlDown() && isKeyPressed(ONE)
			},
			action: func() {
				t.logCursor("hotkey ctrl+1")
			},
		},
		{
			name: "Ctrl+2 Snapshot",
			combo: func() bool {
				return ctrlDown() && isKeyPressed(TWO)
			},
			action: func() {
				t.logSnapshot("hotkey ctrl+2")
			},
		},
		{
			name: "Ctrl+3 Player",
			combo: func() bool {
				return ctrlDown() && isKeyPressed(THREE)
			},
			action: func() {
				t.logPlayerState("hotkey ctrl+3")
			},
		},
		{
			name: "Ctrl+D Fix Belt",
			combo: func() bool {
				return ctrlDown() && isKeyPressed(D)
			},
			action: func() {
				t.fixBeltFromHotkey()
			},
		},
	}

	for {
		t.processHotkeys(hotkeys)

		if t.ctx.ExecutionPriority == context.PriorityStop {
			t.ctx.Logger.Info("Development mode stopped by supervisor")
			return nil
		}

		select {
		case <-refreshTicker.C:
			t.ctx.RefreshGameData()
			if currentArea := t.ctx.Data.PlayerUnit.Area; currentArea != lastArea {
				t.ctx.Logger.Info(
					"Area changed while in development run",
					slog.String("area", currentArea.Area().Name),
				)
				lastArea = currentArea
			}
		default:
			t.processHotkeys(hotkeys)
			time.Sleep(100 * time.Millisecond)
		}
	}
}

func (t *DevRun) processHotkeys(keys []*Hotkey) {
	for _, hk := range keys {
		if hk.combo() {
			// this is to prevent multiple triggers while holding the key down, if you remove this, funny things will happen :)
			if !hk.pressed {
				hk.action()
			}
			hk.pressed = true
		} else {
			hk.pressed = false
		}
	}
}
