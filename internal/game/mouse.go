package game

import (
	"log/slog"
	"math"
	"math/rand"
	"time"

	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

const (
	RightButton MouseButton = win.MK_RBUTTON
	LeftButton  MouseButton = win.MK_LBUTTON

	ShiftKey ModifierKey = win.VK_SHIFT
	CtrlKey  ModifierKey = win.VK_CONTROL
)

type MouseButton uint
type ModifierKey byte

const pointerReleaseDelay = 150 * time.Millisecond

// MovePointer moves the mouse to (x, y) relative to the game window using a bio-realistic trajectory.
func (hid *HID) MovePointer(x, y int) {
	hid.gr.updateWindowPositionData()
	absX := hid.gr.WindowLeftX + x
	absY := hid.gr.WindowTopY + y
	minX := hid.gr.WindowLeftX
	minY := hid.gr.WindowTopY
	maxX := hid.gr.WindowLeftX + hid.gr.GameAreaSizeX
	maxY := hid.gr.WindowTopY + hid.gr.GameAreaSizeY

	if absX < minX {
		absX = minX
	} else if absX > maxX {
		absX = maxX
	}
	if absY < minY {
		absY = minY
	} else if absY > maxY {
		absY = maxY
	}

	if !hid.gi.CursorOverrideActive() {
		slog.Default().Debug("MovePointer: cursor override inactive, skipping move")
		return
	}

	startX, startY, ok := hid.gi.LastCursorPos()
	if !ok {
		// No prior cursor position known; skip animation on the first move.
		if err := hid.gi.CursorPos(absX, absY); err != nil {
			return
		}
		lParam := calculateLparam(absX, absY)
		win.SendMessage(hid.gr.HWND, win.WM_NCHITTEST, 0, lParam)
		win.SendMessage(hid.gr.HWND, win.WM_SETCURSOR, 0x000105A8, 0x2010001)
		win.PostMessage(hid.gr.HWND, win.WM_MOUSEMOVE, 0, lParam)
		return
	}

	// Generate a SigmaDrift trajectory in absolute screen coordinates.
	path := bioMotionPath(float64(startX), float64(startY), float64(absX), float64(absY), defaultSDConfig)

	// Play back intermediate points: update injected cursor + send WM_MOUSEMOVE,
	// sleeping between samples as specified by the gamma-distributed timestamps.
	for i := 0; i+1 < len(path); i++ {
		// Abort if injection was disabled mid-trajectory to avoid chasing the real cursor.
		if !hid.gi.CursorOverrideActive() {
			return
		}
		pt := path[i]
		px := int(math.Round(pt.x))
		py := int(math.Round(pt.y))
		if px < minX {
			px = minX
		} else if px > maxX {
			px = maxX
		}
		if py < minY {
			py = minY
		} else if py > maxY {
			py = maxY
		}
		if err := hid.gi.CursorPos(px, py); err != nil {
			return
		}
		win.PostMessage(hid.gr.HWND, win.WM_MOUSEMOVE, 0, calculateLparam(px, py))
		if dt := path[i+1].t - pt.t; dt > 0 {
			time.Sleep(time.Duration(dt) * time.Millisecond)
		}
	}

	// Stop if injection was disabled during playback.
	if !hid.gi.CursorOverrideActive() {
		return
	}

	// Micro-correction: briefly re-aim near the target, scaled by movement distance.
	dist := math.Hypot(float64(absX-startX), float64(absY-startY))
	var microCorrProb float64
	switch {
	case dist < 30:
		microCorrProb = 0.04
	case dist < 200:
		microCorrProb = 0.12
	default:
		microCorrProb = 0.25
	}
	if rand.Float64() < microCorrProb {
		ox := rand.Intn(11) - 5 // -5 to +5 px
		oy := rand.Intn(11) - 5
		mx := absX + ox
		my := absY + oy
		if mx < minX {
			mx = minX
		} else if mx > maxX {
			mx = maxX
		}
		if my < minY {
			my = minY
		} else if my > maxY {
			my = maxY
		}
		if err := hid.gi.CursorPos(mx, my); err != nil {
			return
		}
		win.PostMessage(hid.gr.HWND, win.WM_MOUSEMOVE, 0, calculateLparam(mx, my))
		time.Sleep(time.Duration(rand.Intn(40)+15) * time.Millisecond)
	}

	// Finalise at the exact target using the original full message sequence.
	if err := hid.gi.CursorPos(absX, absY); err != nil {
		return
	}
	lParam := calculateLparam(absX, absY)
	win.SendMessage(hid.gr.HWND, win.WM_NCHITTEST, 0, lParam)
	win.SendMessage(hid.gr.HWND, win.WM_SETCURSOR, 0x000105A8, 0x2010001)
	win.PostMessage(hid.gr.HWND, win.WM_MOUSEMOVE, 0, lParam)
}

// Click just does a single mouse click at current pointer position
func (hid *HID) Click(btn MouseButton, x, y int) {
	hid.MovePointer(x, y)
	if !hid.InjectorActive() {
		slog.Default().Debug("Click: injector inactive after MovePointer, skipping click")
		return
	}
	x = hid.gr.WindowLeftX + x
	y = hid.gr.WindowTopY + y

	lParam := calculateLparam(x, y)
	buttonDown := uint32(win.WM_LBUTTONDOWN)
	buttonUp := uint32(win.WM_LBUTTONUP)
	if btn == RightButton {
		buttonDown = win.WM_RBUTTONDOWN
		buttonUp = win.WM_RBUTTONUP
	}

	win.SendMessage(hid.gr.HWND, buttonDown, 1, lParam)
	// Gamma(3, mean=60ms) for right-skewed click duration, matching key-press profile.
	sleepTime := utils.GammaDurationMs(60.0, 3.0)
	if sleepTime < keyPressMinTime*time.Millisecond {
		sleepTime = keyPressMinTime * time.Millisecond
	} else if sleepTime > keyPressMaxTime*time.Millisecond {
		sleepTime = keyPressMaxTime * time.Millisecond
	}
	time.Sleep(sleepTime)
	win.SendMessage(hid.gr.HWND, buttonUp, 1, lParam)
}

func (hid *HID) ClickWithModifier(btn MouseButton, x, y int, modifier ModifierKey) {
	hid.gi.KeyStateLock()
	defer hid.gi.KeyStateUnlock()
	hid.gi.OverrideGetKeyState(byte(modifier))
	hid.Click(btn, x, y)
	hid.gi.RestoreGetKeyState()
}

func calculateLparam(x, y int) uintptr {
	return uintptr((y&0xFFFF)<<16 | (x & 0xFFFF))
}
