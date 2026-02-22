package utils

import (
	"fmt"
	"os"
	"syscall"
	"unicode/utf16"
	"unsafe"

	"golang.org/x/sys/windows"
)

var (
	shell32                 = windows.NewLazySystemDLL("shell32.dll")
	ole32                   = windows.NewLazySystemDLL("ole32.dll")
	user32                  = windows.NewLazySystemDLL("user32.dll")
	comdlg32                = windows.NewLazySystemDLL("comdlg32.dll")
	procSHBrowseForFolder   = shell32.NewProc("SHBrowseForFolderW")
	procSHGetPathFromIDList = shell32.NewProc("SHGetPathFromIDListW")
	procCoTaskMemFree       = ole32.NewProc("CoTaskMemFree")
	procGetForegroundWindow = user32.NewProc("GetForegroundWindow")
	procSetForegroundWindow = user32.NewProc("SetForegroundWindow")
	procGetOpenFileName     = comdlg32.NewProc("GetOpenFileNameW")
	procCommDlgExtendedErr  = comdlg32.NewProc("CommDlgExtendedError")
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

	ofnFileMustExist = 0x00001000
	ofnPathMustExist = 0x00000800
	ofnExplorer      = 0x00080000
	ofnNoChangeDir   = 0x00000008
)

type openfilename struct {
	lStructSize       uint32
	hwndOwner         uintptr
	hInstance         uintptr
	lpstrFilter       *uint16
	lpstrCustomFilter *uint16
	nMaxCustFilter    uint32
	nFilterIndex      uint32
	lpstrFile         *uint16
	nMaxFile          uint32
	lpstrFileTitle    *uint16
	nMaxFileTitle     uint32
	lpstrInitialDir   *uint16
	lpstrTitle        *uint16
	Flags             uint32
	nFileOffset       uint16
	nFileExtension    uint16
	lpstrDefExt       *uint16
	lCustData         uintptr
	lpfnHook          uintptr
	lpTemplateName    *uint16
	pvReserved        unsafe.Pointer
	dwReserved        uint32
	FlagsEx           uint32
}

// FileDialogFilter defines a filter entry for Open File dialogs.
type FileDialogFilter struct {
	Name    string
	Pattern string
}

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

// BrowseForFile opens a native Windows open-file dialog.
// Returns the selected file path or an empty string if the user cancels.
func BrowseForFile(title string, filters []FileDialogFilter, initialDir string, defaultExt string) (string, error) {
	hwnd, _, _ := procGetForegroundWindow.Call()

	filterSlice := buildFileDialogFilter(filters)
	if len(filterSlice) == 0 {
		filterSlice = []uint16{0, 0}
	}

	fileBuffer := make([]uint16, windows.MAX_PATH)
	fileTitleBuffer := make([]uint16, windows.MAX_PATH)

	titlePtr, _ := syscall.UTF16PtrFromString(title)
	var initialDirPtr *uint16
	if initialDir != "" {
		initialDirPtr, _ = syscall.UTF16PtrFromString(initialDir)
	}

	var defExtPtr *uint16
	if defaultExt != "" {
		defExtPtr, _ = syscall.UTF16PtrFromString(defaultExt)
	}

	ofn := openfilename{
		lStructSize:     uint32(unsafe.Sizeof(openfilename{})),
		hwndOwner:       hwnd,
		lpstrFilter:     &filterSlice[0],
		nFilterIndex:    1,
		lpstrFile:       &fileBuffer[0],
		nMaxFile:        uint32(len(fileBuffer)),
		lpstrFileTitle:  &fileTitleBuffer[0],
		nMaxFileTitle:   uint32(len(fileTitleBuffer)),
		lpstrInitialDir: initialDirPtr,
		lpstrTitle:      titlePtr,
		Flags:           ofnExplorer | ofnFileMustExist | ofnPathMustExist | ofnNoChangeDir,
		lpstrDefExt:     defExtPtr,
	}

	ret, _, _ := procGetOpenFileName.Call(uintptr(unsafe.Pointer(&ofn)))
	if ret == 0 {
		errCode, _, _ := procCommDlgExtendedErr.Call()
		if errCode == 0 {
			return "", nil // user cancelled dialog
		}
		return "", fmt.Errorf("open file dialog failed with code 0x%X", uint32(errCode))
	}

	path := windows.UTF16PtrToString(ofn.lpstrFile)

	if hwnd != 0 {
		procSetForegroundWindow.Call(hwnd)
	}

	return path, nil
}

func buildFileDialogFilter(filters []FileDialogFilter) []uint16 {
	if len(filters) == 0 {
		filters = []FileDialogFilter{{Name: "All Files (*.*)", Pattern: "*.*"}}
	}

	var data []uint16
	for _, filter := range filters {
		if filter.Name == "" || filter.Pattern == "" {
			continue
		}
		data = append(data, utf16.Encode([]rune(filter.Name))...)
		data = append(data, 0)
		data = append(data, utf16.Encode([]rune(filter.Pattern))...)
		data = append(data, 0)
	}

	if len(data) == 0 {
		data = append(data, utf16.Encode([]rune("All Files (*.*)"))...)
		data = append(data, 0)
		data = append(data, utf16.Encode([]rune("*.*"))...)
		data = append(data, 0)
	}

	data = append(data, 0)
	return data
}
