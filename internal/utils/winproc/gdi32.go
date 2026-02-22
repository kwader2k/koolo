package winproc

import "golang.org/x/sys/windows"

var (
    GDI32                  = windows.NewLazySystemDLL("gdi32.dll")
    CreateCompatibleDC     = GDI32.NewProc("CreateCompatibleDC")
    CreateCompatibleBitmap = GDI32.NewProc("CreateCompatibleBitmap")
    CreateDIBSection       = GDI32.NewProc("CreateDIBSection")
    SelectObject           = GDI32.NewProc("SelectObject")
    DeleteObject           = GDI32.NewProc("DeleteObject")
    DeleteDC               = GDI32.NewProc("DeleteDC")
    BitBlt                 = GDI32.NewProc("BitBlt")
    GdiFlush               = GDI32.NewProc("GdiFlush")
)

const SRCCOPY = 0x00CC0020
