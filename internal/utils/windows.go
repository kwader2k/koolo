package utils

import (
	"os"
	"syscall"
	"unsafe"

	"golang.org/x/sys/windows"
)

var (
	shell32                 = windows.NewLazySystemDLL("shell32.dll")
	ole32                   = windows.NewLazySystemDLL("ole32.dll")
	user32                  = windows.NewLazySystemDLL("user32.dll")
	procSHBrowseForFolder   = shell32.NewProc("SHBrowseForFolderW")
	procSHGetPathFromIDList = shell32.NewProc("SHGetPathFromIDListW")
	procCoTaskMemFree       = ole32.NewProc("CoTaskMemFree")
	procGetForegroundWindow = user32.NewProc("GetForegroundWindow")
	procSetForegroundWindow = user32.NewProc("SetForegroundWindow")
)

type browseInfo struct {
	hwndOwner      uintptr
	pidlRoot       uintptr
	pszDisplayName *uint16
	lpszTitle      *uint16
	ulFlags        uint32
	lpfn           uintptr
	lParam         uintptr
	iImage         int32
}

const (
	BIF_RETURNONLYFSDIRS = 0x00000001
	BIF_NEWDIALOGSTYLE   = 0x00000040
	BIF_EDITBOX          = 0x00000010
)

func HasAdminPermission() bool {
	_, err := os.Open("\\\\.\\PHYSICALDRIVE0")

	return err == nil
}

func ShowDialog(title, message string) {
	t, _ := syscall.UTF16PtrFromString(title)
	txt, _ := syscall.UTF16PtrFromString(message)

	windows.MessageBox(0, txt, t, 0)
}

// BrowseForFolder opens a native Windows folder selection dialog
func BrowseForFolder(title string) (string, error) {
	// Get the current foreground window to use as parent
	hwnd, _, _ := procGetForegroundWindow.Call()

	displayName := make([]uint16, windows.MAX_PATH)
	titlePtr, _ := syscall.UTF16PtrFromString(title)

	bi := browseInfo{
		hwndOwner:      hwnd, // Use the active window as parent
		pidlRoot:       0,
		pszDisplayName: &displayName[0],
		lpszTitle:      titlePtr,
		ulFlags:        BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE | BIF_EDITBOX,
		lpfn:           0,
		lParam:         0,
		iImage:         0,
	}

	// Show the browse dialog
	ret, _, _ := procSHBrowseForFolder.Call(uintptr(unsafe.Pointer(&bi)))
	if ret == 0 {
		return "", nil // User cancelled
	}

	// Get the path from the returned PIDL
	pathBuffer := make([]uint16, windows.MAX_PATH)
	procSHGetPathFromIDList.Call(ret, uintptr(unsafe.Pointer(&pathBuffer[0])))

	// Free the PIDL memory
	procCoTaskMemFree.Call(ret)

	// Convert UTF16 to string
	path := syscall.UTF16ToString(pathBuffer)

	// Restore focus to the original window
	if hwnd != 0 {
		procSetForegroundWindow.Call(hwnd)
	}

	return path, nil
}
