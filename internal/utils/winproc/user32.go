package winproc

import "golang.org/x/sys/windows"

var (
    USER32                  = windows.NewLazySystemDLL("user32.dll")
    PrintWindow             = USER32.NewProc("PrintWindow")
    GetWindowDC             = USER32.NewProc("GetWindowDC")
    GetDC                   = USER32.NewProc("GetDC")
    ReleaseDC               = USER32.NewProc("ReleaseDC")
    IsIconic                = USER32.NewProc("IsIconic")
    SetProcessDpiAware      = USER32.NewProc("SetProcessDPIAware")
    GetClientRect           = USER32.NewProc("GetClientRect")
    GetWindowRect           = USER32.NewProc("GetWindowRect")
    ClientToScreen          = USER32.NewProc("ClientToScreen")
    MapVirtualKey           = USER32.NewProc("MapVirtualKeyW")
    GetKeyState             = USER32.NewProc("GetKeyState")
    SetWindowText           = USER32.NewProc("SetWindowTextW")
    GetWindowText           = USER32.NewProc("GetWindowTextW")
    GetWindowTextLength     = USER32.NewProc("GetWindowTextLengthW")
    RedrawWindow            = USER32.NewProc("RedrawWindow")
    UpdateWindow            = USER32.NewProc("UpdateWindow")
    EnumChildWindows        = USER32.NewProc("EnumChildWindows")
)
