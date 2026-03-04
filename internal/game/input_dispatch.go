package game

import (
	"crypto/rand"

	"github.com/lxn/win"
	"golang.org/x/sys/windows"
)

var (
	user32Direct       = windows.NewLazySystemDLL("user32.dll")
	procPostMessageW   = user32Direct.NewProc("PostMessageW")
	procSendMessageW   = user32Direct.NewProc("SendMessageW")
	procMapVirtualKeyW = user32Direct.NewProc("MapVirtualKeyW")
)

// postMsg sends a window message asynchronously via PostMessageW.
func postMsg(hwnd win.HWND, msg uint32, wParam, lParam uintptr) {
	procPostMessageW.Call(uintptr(hwnd), uintptr(msg), wParam, lParam)
}

// sendMsg sends a window message synchronously via SendMessageW.
func sendMsg(hwnd win.HWND, msg uint32, wParam, lParam uintptr) uintptr {
	ret, _, _ := procSendMessageW.Call(uintptr(hwnd), uintptr(msg), wParam, lParam)
	return ret
}

// mapVKey wraps MapVirtualKeyW (scan-code lookup).
func mapVKey(code, mapType uintptr) uintptr {
	ret, _, _ := procMapVirtualKeyW.Call(code, mapType)
	return ret
}

func dispatchKeyMsg(hwnd win.HWND, msg uint32, wParam, lParam uintptr) {
	b := make([]byte, 1)
	if _, err := rand.Read(b); err == nil && b[0]&1 == 0 {
		sendMsg(hwnd, msg, wParam, lParam)
	} else {
		postMsg(hwnd, msg, wParam, lParam)
	}
}

func isExtendedKey(key byte) bool {
	switch key {
	case
		win.VK_INSERT, win.VK_DELETE,
		win.VK_HOME, win.VK_END,
		win.VK_PRIOR, win.VK_NEXT, // Page Up, Page Down
		win.VK_LEFT, win.VK_RIGHT, win.VK_UP, win.VK_DOWN,
		win.VK_RCONTROL, win.VK_RMENU,
		win.VK_LWIN, win.VK_RWIN,
		win.VK_APPS, win.VK_SNAPSHOT,
		win.VK_NUMLOCK:
		return true
	}
	return false
}
