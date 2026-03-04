package game

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"log/slog"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/hectorgimenez/d2go/pkg/memory"
	"golang.org/x/sys/windows"
)

const fullAccess = windows.PROCESS_VM_OPERATION | windows.PROCESS_VM_WRITE | windows.PROCESS_VM_READ

type MemoryInjector struct {
	isLoaded              bool
	pid                   uint32
	handle                windows.Handle
	getCursorPosAddr      uintptr
	getCursorPosOrigBytes [32]byte
	trackMouseEventAddr   uintptr
	trackMouseEventBytes  [32]byte
	getKeyStateAddr       uintptr
	getKeyStateOrigBytes  [18]byte
	setCursorPosAddr      uintptr
	setCursorPosOrigBytes [6]byte
	logger                *slog.Logger
	cursorOverrideActive  bool
	lastCursorX           int
	lastCursorY           int
	cursorPosKnown        bool // true once CursorPos has been called at least once
	lastHookTime          time.Time
	stateMu               sync.RWMutex
	keyStateMu            sync.Mutex // serializes OverrideGetKeyState / RestoreGetKeyState pairs
}

func InjectorInit(logger *slog.Logger, pid uint32) (*MemoryInjector, error) {
	i := &MemoryInjector{pid: pid, logger: logger, lastHookTime: time.Now()}
	pHandle, err := ntOpenProcess(pid, fullAccess)
	if err != nil {
		return nil, fmt.Errorf("error opening process: %w", err)
	}
	i.handle = pHandle

	return i, nil
}

func (i *MemoryInjector) Load() error {
	i.stateMu.RLock()
	loaded := i.isLoaded
	i.stateMu.RUnlock()
	if loaded {
		return nil
	}

	modules, err := memory.GetProcessModules(i.pid)
	if err != nil {
		return fmt.Errorf("error getting process modules: %w", err)
	}

	if _, err = syscall.LoadDLL("USER32.dll"); err != nil {
		return fmt.Errorf("error loading USER32.dll: %w", err)
	}

	for _, module := range modules {
		if strings.Contains(strings.ToLower(module.ModuleName), "user32.dll") {
			i.getCursorPosAddr, err = syscall.GetProcAddress(module.ModuleHandle, "GetCursorPos")
			if err != nil {
				return fmt.Errorf("error getting GetCursorPos address: %w", err)
			}
			i.getKeyStateAddr, err = syscall.GetProcAddress(module.ModuleHandle, "GetKeyState")
			if err != nil {
				return fmt.Errorf("error getting GetKeyState address: %w", err)
			}
			i.trackMouseEventAddr, err = syscall.GetProcAddress(module.ModuleHandle, "TrackMouseEvent")
			if err != nil {
				return fmt.Errorf("error getting TrackMouseEvent address: %w", err)
			}
			i.setCursorPosAddr, err = syscall.GetProcAddress(module.ModuleHandle, "SetCursorPos")
			if err != nil {
				return fmt.Errorf("error getting SetCursorPos address: %w", err)
			}

			err = ntReadMemory(i.handle, i.getCursorPosAddr, &i.getCursorPosOrigBytes[0], uintptr(len(i.getCursorPosOrigBytes)))
			if err != nil {
				return fmt.Errorf("error reading memory: %w", err)
			}

			err = i.stopTrackingMouseLeaveEvents()
			if err != nil {
				return err
			}

			err = ntReadMemory(i.handle, i.setCursorPosAddr, &i.setCursorPosOrigBytes[0], uintptr(len(i.setCursorPosOrigBytes)))
			if err != nil {
				return fmt.Errorf("error reading setcursor memory: %w", err)
			}

			err = i.OverrideSetCursorPos()
			if err != nil {
				return err
			}

			err = ntReadMemory(i.handle, i.getKeyStateAddr, &i.getKeyStateOrigBytes[0], uintptr(len(i.getKeyStateOrigBytes)))
			if err != nil {
				return fmt.Errorf("error reading memory: %w", err)
			}
		}
	}
	if i.getCursorPosAddr == 0 || i.getKeyStateAddr == 0 {
		return errors.New("could not find GetCursorPos address")
	}

	i.stateMu.Lock()
	i.isLoaded = true
	i.stateMu.Unlock()
	return nil
}

func (i *MemoryInjector) Unload() error {
	if err := i.RestoreMemory(); err != nil {
		i.logger.Error(fmt.Sprintf("error restoring memory: %v", err))
	}

	return ntClose(i.handle)
}

// Close releases the process handle without restoring memory hooks.
func (i *MemoryInjector) Close() error {
	return ntClose(i.handle)
}

func (i *MemoryInjector) RestoreMemory() error {
	i.stateMu.Lock()
	if !i.isLoaded {
		i.stateMu.Unlock()
		return nil
	}
	// Clear flags before restoring bytes.
	i.isLoaded = false
	i.cursorOverrideActive = false
	i.stateMu.Unlock()

	if err := i.RestoreGetCursorPosAddr(); err != nil {
		return fmt.Errorf("error restoring memory: %v", err)
	}
	if err := i.RestoreSetCursorPosAddr(); err != nil {
		return fmt.Errorf("error restoring cursor memory: %v", err)
	}
	return i.RestoreGetKeyState()
}

func (i *MemoryInjector) DisableCursorOverride() error {
	i.stateMu.RLock()
	loaded := i.isLoaded
	overrideActive := i.cursorOverrideActive
	i.stateMu.RUnlock()
	if !loaded || !overrideActive {
		return nil
	}
	if err := i.RestoreGetCursorPosAddr(); err != nil {
		return err
	}
	if err := i.RestoreSetCursorPosAddr(); err != nil {
		return err
	}
	i.stateMu.Lock()
	i.cursorOverrideActive = false
	i.stateMu.Unlock()
	return nil
}

func (i *MemoryInjector) EnableCursorOverride() error {
	i.stateMu.RLock()
	loaded := i.isLoaded
	overrideActive := i.cursorOverrideActive
	lastX := i.lastCursorX
	lastY := i.lastCursorY
	i.stateMu.RUnlock()
	if !loaded || overrideActive {
		return nil
	}
	if err := i.OverrideSetCursorPos(); err != nil {
		return err
	}
	// Reapply GetCursorPos hook using the last known coordinates
	return i.CursorPos(lastX, lastY)
}

func (i *MemoryInjector) CursorPos(x, y int) error {
	i.stateMu.RLock()
	loaded := i.isLoaded
	i.stateMu.RUnlock()
	if !loaded {
		return nil
	}

	i.stateMu.Lock()
	i.lastHookTime = time.Now()
	i.lastCursorX = x
	i.lastCursorY = y
	i.cursorPosKnown = true
	i.stateMu.Unlock()

	code := polyGetCursorPos(x, y)
	return writeAndClean(i.handle, i.getCursorPosAddr, code)
}

// OverrideGetKeyState hooks GetKeyState to report the given key as pressed.
func (i *MemoryInjector) OverrideGetKeyState(key byte) error {
	i.stateMu.RLock()
	loaded := i.isLoaded
	i.stateMu.RUnlock()
	if !loaded {
		return nil
	}

	i.stateMu.Lock()
	i.lastHookTime = time.Now()
	i.stateMu.Unlock()

	code := polyGetKeyStateHook(key)
	return writeAndClean(i.handle, i.getKeyStateAddr, code)
}

// KeyStateLock serializes modifier key override sequences.
func (i *MemoryInjector) KeyStateLock()   { i.keyStateMu.Lock() }
func (i *MemoryInjector) KeyStateUnlock() { i.keyStateMu.Unlock() }

func (i *MemoryInjector) OverrideSetCursorPos() error {
	// Prevents the game from repositioning our cursor (inventory, waypoints, etc.).
	i.stateMu.Lock()
	i.lastHookTime = time.Now()
	i.stateMu.Unlock()

	blob := polyReturn1()
	err := writeAndClean(i.handle, i.setCursorPosAddr, blob)
	if err == nil {
		i.stateMu.Lock()
		i.cursorOverrideActive = true
		i.stateMu.Unlock()
	}
	return err
}

func (i *MemoryInjector) RestoreGetKeyState() error {
	return ntWriteMemory(i.handle, i.getKeyStateAddr, &i.getKeyStateOrigBytes[0], uintptr(len(i.getKeyStateOrigBytes)))
}

func (i *MemoryInjector) RestoreGetCursorPosAddr() error {
	return ntWriteMemory(i.handle, i.getCursorPosAddr, &i.getCursorPosOrigBytes[0], uintptr(len(i.getCursorPosOrigBytes)))
}

func (i *MemoryInjector) RestoreSetCursorPosAddr() error {
	return ntWriteMemory(i.handle, i.setCursorPosAddr, &i.setCursorPosOrigBytes[0], uintptr(len(i.setCursorPosOrigBytes)))
}

func (i *MemoryInjector) CursorOverrideActive() bool {
	if i == nil {
		return false
	}
	i.stateMu.RLock()
	defer i.stateMu.RUnlock()
	return i.isLoaded && i.cursorOverrideActive
}

// LastCursorPos returns the last injected cursor position; ok is false before the first move.
func (i *MemoryInjector) LastCursorPos() (x, y int, ok bool) {
	i.stateMu.RLock()
	defer i.stateMu.RUnlock()
	return i.lastCursorX, i.lastCursorY, i.cursorPosKnown
}

// This is needed in order to let the game keep processing mouse events even if the mouse is not over the window
func (i *MemoryInjector) stopTrackingMouseLeaveEvents() error {
	err := ntReadMemory(i.handle, i.trackMouseEventAddr, &i.trackMouseEventBytes[0], uintptr(len(i.trackMouseEventBytes)))
	if err != nil {
		return err
	}

	// and dword ptr [rcx+4], 0xFFFFFFFD
	// Modify TRACKMOUSEEVENT struct to disable mouse leave events, since we are injecting our events even if the mouse is not over the window
	disableMouseLeaveRequest := []byte{0x81, 0x61, 0x04, 0xFD, 0xFF, 0xFF, 0xFF}

	// Already hooked
	if bytes.Contains(i.trackMouseEventBytes[:], disableMouseLeaveRequest) {
		return nil
	}

	// We need to move back the pointer 7 bytes to get the correct position, since we are injecting 7 bytes in front of it
	num := int32(binary.LittleEndian.Uint32(i.trackMouseEventBytes[2:6]))
	num -= 7
	numberBytes := make([]byte, 4)
	binary.LittleEndian.PutUint32(numberBytes, uint32(num))
	// Copy the prefix to avoid aliasing the trackMouseEventBytes backing array.
	prefix := make([]byte, 2)
	copy(prefix, i.trackMouseEventBytes[0:2])
	injectBytes := append(prefix, numberBytes...)

	hook := append(disableMouseLeaveRequest, injectBytes...)

	i.stateMu.Lock()
	i.lastHookTime = time.Now()
	i.stateMu.Unlock()

	return writeAndClean(i.handle, i.trackMouseEventAddr, hook)
}
