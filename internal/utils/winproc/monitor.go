package winproc

import (
	"fmt"
	"log/slog"
	"syscall"
	"unsafe"

	"golang.org/x/sys/windows"
)

// MonitorInfo describes a display monitor's index, name, and bounding rectangle.
type MonitorInfo struct {
	Index  int    // 0-based index
	Name   string // e.g. "\\.\DISPLAY1"
	Left   int32
	Top    int32
	Right  int32
	Bottom int32
}

var (
	enumDisplayMonitors = USER32.NewProc("EnumDisplayMonitors")
	getMonitorInfoW     = USER32.NewProc("GetMonitorInfoW")
)

// MONITORINFOEXW matches the Windows MONITORINFOEXW structure.
type monitorInfoExW struct {
	CbSize    uint32
	RcMonitor struct{ Left, Top, Right, Bottom int32 }
	RcWork    struct{ Left, Top, Right, Bottom int32 }
	DwFlags   uint32
	SzDevice  [32]uint16
}

// EnumMonitors returns information about all connected display monitors.
func EnumMonitors() []MonitorInfo {
	var monitors []MonitorInfo

	cb := syscall.NewCallback(func(hMonitor uintptr, hdcMonitor uintptr, lprcMonitor uintptr, dwData uintptr) uintptr {
		var info monitorInfoExW
		info.CbSize = uint32(unsafe.Sizeof(info))
		ret, _, _ := getMonitorInfoW.Call(hMonitor, uintptr(unsafe.Pointer(&info)))
		if ret == 0 {
			return 1 // continue enumeration
		}
		monitors = append(monitors, MonitorInfo{
			Index:  len(monitors),
			Name:   windows.UTF16ToString(info.SzDevice[:]),
			Left:   info.RcMonitor.Left,
			Top:    info.RcMonitor.Top,
			Right:  info.RcMonitor.Right,
			Bottom: info.RcMonitor.Bottom,
		})
		return 1 // continue enumeration
	})

	ret, _, callErr := enumDisplayMonitors.Call(0, 0, cb, 0)
	if ret == 0 {
		if callErr == syscall.Errno(0) {
			callErr = syscall.GetLastError()
		}
		slog.Error("EnumDisplayMonitors failed", slog.Any("error", callErr))
		return nil
	}
	return monitors
}

// DisplayString returns a user-friendly label like "Monitor 1 (1920x1080)".
func (m MonitorInfo) DisplayString() string {
	w := m.Right - m.Left
	h := m.Bottom - m.Top
	return fmt.Sprintf("Monitor %d (%dx%d)", m.Index+1, w, h)
}
