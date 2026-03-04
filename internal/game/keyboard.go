package game

import (
	"strings"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

const (
	keyPressMinTime = 30  // ms — lower clamp for Gamma-sampled press duration
	keyPressMaxTime = 150 // ms — upper clamp for Gamma-sampled press duration
)

// PressKey receives an ASCII code and sends a key press event to the game window
func (hid *HID) PressKey(key byte) {
	// Polymorphic dispatch — randomly uses PostMessageW or SendMessageW.
	dispatchKeyMsg(hid.gr.HWND, win.WM_KEYDOWN, uintptr(key), hid.calculatelParam(key, true))
	// Gamma(3, mean=60ms) produces a right-skewed duration matching human key-press data.
	sleepTime := utils.GammaDurationMs(60.0, 3.0)
	if sleepTime < keyPressMinTime*time.Millisecond {
		sleepTime = keyPressMinTime * time.Millisecond
	} else if sleepTime > keyPressMaxTime*time.Millisecond {
		sleepTime = keyPressMaxTime * time.Millisecond
	}
	time.Sleep(sleepTime)
	dispatchKeyMsg(hid.gr.HWND, win.WM_KEYUP, uintptr(key), hid.calculatelParam(key, false))
}

func (hid *HID) KeySequence(keysToPress ...byte) {
	for _, key := range keysToPress {
		hid.PressKey(key)
		utils.Sleep(200)
	}
}

// TypeText sends each character in text to the game window as a WM_CHAR message.
// Use this for free-form text entry (e.g., chat commands) rather than key bindings.
func (hid *HID) TypeText(text string) {
	for _, ch := range text {
		dispatchKeyMsg(hid.gr.HWND, win.WM_CHAR, uintptr(ch), 1)
		utils.Sleep(50)
	}
}

// PressKeyWithModifier works the same as PressKey but with a modifier key (shift, ctrl, alt)
func (hid *HID) PressKeyWithModifier(key byte, modifier ModifierKey) {
	hid.gi.KeyStateLock()
	defer hid.gi.KeyStateUnlock()
	hid.gi.OverrideGetKeyState(byte(modifier))
	hid.PressKey(key)
	hid.gi.RestoreGetKeyState()
}

func (hid *HID) PressKeyBinding(kb data.KeyBinding) {
	keys := getKeysForKB(kb)
	if keys[0] == 0 {
		return // No key bound, skip
	}
	if keys[1] == 0 || keys[1] == 255 {
		hid.PressKey(keys[0])
		return
	}

	hid.PressKeyWithModifier(keys[0], ModifierKey(keys[1]))
}

// KeyDown sends a key down event to the game window
func (hid *HID) KeyDown(kb data.KeyBinding) {
	keys := getKeysForKB(kb)
	dispatchKeyMsg(hid.gr.HWND, win.WM_KEYDOWN, uintptr(keys[0]), hid.calculatelParam(keys[0], true))
}

// KeyUp sends a key up event to the game window
func (hid *HID) KeyUp(kb data.KeyBinding) {
	keys := getKeysForKB(kb)
	dispatchKeyMsg(hid.gr.HWND, win.WM_KEYUP, uintptr(keys[0]), hid.calculatelParam(keys[0], false))
}

func getKeysForKB(kb data.KeyBinding) [2]byte {
	if kb.Key1[0] == 0 || kb.Key1[0] == 255 {
		return [2]byte{kb.Key2[0], kb.Key2[1]}
	}

	return [2]byte{kb.Key1[0], kb.Key1[1]}
}

func (hid *HID) GetASCIICode(key string) byte {
	char, found := specialChars[strings.ToLower(key)]
	if found {
		return char
	}

	return strings.ToUpper(key)[0]
}

var specialChars = map[string]byte{
	"esc":       win.VK_ESCAPE,
	"enter":     win.VK_RETURN,
	"f1":        win.VK_F1,
	"f2":        win.VK_F2,
	"f3":        win.VK_F3,
	"f4":        win.VK_F4,
	"f5":        win.VK_F5,
	"f6":        win.VK_F6,
	"f7":        win.VK_F7,
	"f8":        win.VK_F8,
	"f9":        win.VK_F9,
	"f10":       win.VK_F10,
	"f11":       win.VK_F11,
	"f12":       win.VK_F12,
	"lctrl":     win.VK_LCONTROL,
	"home":      win.VK_HOME,
	"down":      win.VK_DOWN,
	"up":        win.VK_UP,
	"left":      win.VK_LEFT,
	"right":     win.VK_RIGHT,
	"tab":       win.VK_TAB,
	"space":     win.VK_SPACE,
	"alt":       win.VK_MENU,
	"lalt":      win.VK_LMENU,
	"ralt":      win.VK_RMENU,
	"shift":     win.VK_LSHIFT,
	"backspace": win.VK_BACK,
	"lwin":      win.VK_LWIN,
	"rwin":      win.VK_RWIN,
	"end":       win.VK_END,
	"-":         win.VK_OEM_MINUS,
}

func (hid *HID) calculatelParam(keyCode byte, down bool) uintptr {
	scanCode := int(mapVKey(uintptr(keyCode), 0))
	repeatCount := 1
	extendedKeyFlag := 0
	// Set extended-key flag for keys that produce it on real hardware
	// (arrows, insert, delete, home, end, page up/down, right-side modifiers).
	// Omitting this flag is a telltale sign of synthetic input.
	if isExtendedKey(keyCode) {
		extendedKeyFlag = 1
	}
	contextCode := 0
	previousKeyState := 0
	transitionState := 0
	if !down {
		transitionState = 1
		// On a real keyboard the key was down before going up, so
		// previousKeyState is always 1 for WM_KEYUP. The original code
		// left this at 0 which is unrealistic.
		previousKeyState = 1
	}

	lParam := uintptr((repeatCount & 0xFFFF) | (scanCode << 16) | (extendedKeyFlag << 24) | (contextCode << 29) | (previousKeyState << 30) | (transitionState << 31))
	return lParam
}
