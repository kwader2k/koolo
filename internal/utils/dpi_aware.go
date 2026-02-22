package utils

import "github.com/hectorgimenez/koolo/internal/utils/winproc"

func init() {
    winproc.SetProcessDpiAware.Call()
}
