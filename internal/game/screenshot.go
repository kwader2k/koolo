package game

import (
    "image"
    "reflect"
    "unsafe"

    "github.com/hectorgimenez/koolo/internal/utils/winproc"
)

type bmpInfoHeader struct {
    BiSize          uint32
    BiWidth         int32
    BiHeight        int32
    BiPlanes        uint16
    BiBitCount      uint16
    BiCompression   uint32
    BiSizeImage     uint32
    BiXPelsPerMeter int32
    BiYPelsPerMeter int32
    BiClrUsed       uint32
    BiClrImportant  uint32
}

type bitmapInfo struct{ Header bmpInfoHeader }

type rect struct{ Left, Top, Right, Bottom int32 }

func clientSize(hwnd uintptr) (width, height int) {
    var rc rect
    winproc.GetClientRect.Call(hwnd, uintptr(unsafe.Pointer(&rc)))
    return int(rc.Right - rc.Left), int(rc.Bottom - rc.Top)
}

func (gd *MemoryReader) Screenshot() image.Image {
    gd.updateWindowPositionData()

    width, height := clientSize(uintptr(gd.HWND))
    if width <= 0 || height <= 0 {
        return nil
    }

    // Screen DC (for compatibility); no blit from it, only to create DIB/DC
    hdcScreen, _, _ := winproc.GetDC.Call(0)
    if hdcScreen == 0 {
        return nil
    }
    defer winproc.ReleaseDC.Call(0, hdcScreen)

    // Memory DC
    hdcMem, _, _ := winproc.CreateCompatibleDC.Call(hdcScreen)
    if hdcMem == 0 {
        return nil
    }
    defer winproc.DeleteDC.Call(hdcMem)

    // Top‑down 32‑bpp DIB
    bi := bitmapInfo{Header: bmpInfoHeader{
        BiSize:     40,
        BiWidth:    int32(width),
        BiHeight:   -int32(height),
        BiPlanes:   1,
        BiBitCount: 32,
    }}
    var bitsPtr uintptr
    hbm, _, _ := winproc.CreateDIBSection.Call(hdcScreen, uintptr(unsafe.Pointer(&bi)), 0, uintptr(unsafe.Pointer(&bitsPtr)), 0, 0)
    if hbm == 0 || bitsPtr == 0 {
        return nil
    }
    defer winproc.DeleteObject.Call(hbm)
    winproc.SelectObject.Call(hdcMem, hbm)

    // Single capture method: PrintWindow with PW_CLIENTONLY|PW_RENDERFULLCONTENT (flags=3)
    _, _, _ = winproc.PrintWindow.Call(uintptr(gd.HWND), hdcMem, 3)
    winproc.GdiFlush.Call()

    // Wrap the DIB memory into an RGBA and swap B<->R (BGRA->RGBA)
    n := width * height * 4
    var src []byte
    hdr := (*reflect.SliceHeader)(unsafe.Pointer(&src))
    hdr.Data = bitsPtr
    hdr.Len = n
    hdr.Cap = n

    img := image.NewRGBA(image.Rect(0, 0, width, height))
    copy(img.Pix, src)
    for y := 0; y < img.Bounds().Dy(); y++ {
        for x := 0; x < img.Bounds().Dx(); x++ {
            idx := y*img.Stride + x*4
            img.Pix[idx], img.Pix[idx+2] = img.Pix[idx+2], img.Pix[idx]
        }
    }
    return img
}
