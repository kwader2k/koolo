package bot

import (
	"errors"
	"fmt"
	"image"
	"log/slog"
	"strings"
	"syscall"
	"unsafe"

	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

var classCoords = map[string][2]int{
	"amazon": {ui.CharAmazonX, ui.CharAmazonY}, "assassin": {ui.CharAssassinX, ui.CharAssassinY},
	"necro": {ui.CharNecroX, ui.CharNecroY}, "barb": {ui.CharBarbX, ui.CharBarbY},
	"pala": {ui.CharPallyX, ui.CharPallyY}, "sorc": {ui.CharSorcX, ui.CharSorcY},
	"druid": {ui.CharDruidX, ui.CharDruidY},
}

const (
	INPUT_KEYBOARD    = 1
	KEYEVENTF_UNICODE = 0x0004
	KEYEVENTF_KEYUP   = 0x0002

	gameVersionMenuOpenDelay = 400
	gameVersionSampleDelay   = 220
	gameVersionPostCheckWait = 300
)

type KEYBDINPUT struct {
	wVk, wScan  uint16
	dwFlags     uint32
	time        uint32
	dwExtraInfo uintptr
}

type INPUT struct {
	inputType uint32
	ki        KEYBDINPUT
	padding   [8]byte
}

var (
	user32        = syscall.NewLazyDLL("user32.dll")
	procSendInput = user32.NewProc("SendInput")
)

func AutoCreateCharacter(class, name string) error {
	ctx := context.Get()
	ctx.Logger.Info("[AutoCreate] Processing", slog.String("class", class), slog.String("name", name))
	authMethod := strings.TrimSpace(ctx.CharacterCfg.AuthMethod)
	isOfflineAuth := authMethod == "" || strings.EqualFold(authMethod, "None")

	// 1. Enter character creation screen
	if !ctx.GameReader.IsInCharacterCreationScreen() {
		if err := enterCreationScreen(ctx); err != nil {
			return err
		}
	}

	ctx.SetLastAction("CreateCharacter")

	// 2. Select Game Version
	selectGameVersion(ctx)

	// 3. Select Class
	classPos, err := getClassPosition(class)
	if err != nil {
		return err
	}
	ctx.HID.Click(game.LeftButton, classPos[0], classPos[1])
	utils.Sleep(500)

	// 4. Toggle Ladder
	if !isOfflineAuth && !ctx.CharacterCfg.Game.IsNonLadderChar {
		ctx.HID.Click(game.LeftButton, ui.CharLadderBtnX, ui.CharLadderBtnY)
		utils.Sleep(300)
	}

	// 5. Toggle Hardcore
	if ctx.CharacterCfg.Game.IsHardCoreChar {
		hardcoreX, hardcoreY := ui.CharHardcoreBtnX, ui.CharHardcoreBtnY
		if isOfflineAuth {
			// Offline creation screen omits ladder, shifting toggle positions left.
			hardcoreX, hardcoreY = ui.CharOfflineHardcoreBtnX, ui.CharHardcoreBtnY
		}
		ctx.HID.Click(game.LeftButton, hardcoreX, hardcoreY)
		utils.Sleep(300)
	}

	// 6. Input Name
	ensureForegroundWindow(ctx)
	if err := inputCharacterName(ctx, name); err != nil {
		return err
	}

	// 7. Click Create Button
	ctx.HID.Click(game.LeftButton, ui.CharCreateBtnX, ui.CharCreateBtnY)
	utils.Sleep(1500)

	// 8. Confirm hardcore warning dialog
	if ctx.CharacterCfg.Game.IsHardCoreChar {
		ctx.HID.PressKey(win.VK_RETURN)
		utils.Sleep(500)
	}

	// Wait for character selection screen and confirm the new character is visible/selected
	for i := 0; i < 5; i++ {
		if ctx.GameReader.IsInCharacterSelectionScreen() {
			// Give it a moment to update selection state
			utils.Sleep(500)
			selected := ctx.GameReader.GameReader.GetSelectedCharacterName()
			ctx.Logger.Info("[AutoCreate] Back at selection screen",
				slog.String("selected", selected),
				slog.String("expected", name))

			if strings.EqualFold(selected, name) {
				ctx.Logger.Info("[AutoCreate] Character successfully created and selected")
				return nil
			}
		}
		utils.Sleep(500)
	}

	return errors.New("creation timeout or character not found after creation")
}

func selectGameVersion(ctx *context.Status) {
	if ctx == nil || ctx.CharacterCfg == nil {
		return
	}

	version, ok := normalizeGameVersion(ctx.CharacterCfg.Game.GameVersion)
	if !ok {
		ctx.Logger.Warn("[AutoCreate] Unknown game version, defaulting to warlock",
			slog.String("gameVersion", ctx.CharacterCfg.Game.GameVersion))
	}
	ctx.Logger.Info("[AutoCreate] Selecting game version", slog.String("gameVersion", version))

	ctx.HID.Click(game.LeftButton, ui.GameVersionBtnX, ui.GameVersionBtnY)
	utils.Sleep(gameVersionMenuOpenDelay)

	hasDLC, confident := detectWarlockDLC(ctx)
	utils.Sleep(gameVersionPostCheckWait)
	if !confident {
		if ctx.CharacterCfg.Game.DLCEnabled {
			hasDLC = true
			confident = true
		}
	}

	if confident {
		cacheDLCEnabled(ctx, hasDLC)
		if hasDLC {
			if version == "expansion" {
				// DLC present: default is ROTW. Expansion is an explicit non-default pick.
				ctx.HID.Click(game.LeftButton, ui.GameversionExpansionX, ui.GameversionExpansionY)
			} else {
				// DLC present + ROTW selected: keep default via one extra toggle click.
				ctx.HID.Click(game.LeftButton, ui.GameVersionBtnX, ui.GameVersionBtnY)
			}
		} else {
			// DLC absent: expansion is always default.
			// Keep default via one extra toggle click regardless of requested version.
			ctx.HID.Click(game.LeftButton, ui.GameVersionBtnX, ui.GameVersionBtnY)
		}
	} else {
		switch version {
		case "expansion":
			ctx.HID.Click(game.LeftButton, ui.GameversionExpansionX, ui.GameversionExpansionY)
		default:
			ctx.HID.Click(game.LeftButton, ui.GameversionWarlockX, ui.GameversionWarlockY)
		}
	}
	utils.Sleep(250)
}

func cacheDLCEnabled(ctx *context.Status, hasDLC bool) {
	if ctx == nil || ctx.CharacterCfg == nil {
		return
	}

	updated := false
	if ctx.CharacterCfg.Game.DLCEnabled != hasDLC {
		ctx.CharacterCfg.Game.DLCEnabled = hasDLC
		updated = true
	}
	currentVersion, _ := normalizeGameVersion(ctx.CharacterCfg.Game.GameVersion)
	if !hasDLC && currentVersion != "expansion" {
		ctx.CharacterCfg.Game.GameVersion = config.GameVersionExpansion
		updated = true
	}
	if !updated {
		return
	}

	if cfg, ok := config.GetCharacter(ctx.Name); ok && cfg != nil {
		cfg.Game.DLCEnabled = hasDLC
		if !hasDLC {
			cfg.Game.GameVersion = config.GameVersionExpansion
		}
		if err := config.SaveSupervisorConfig(ctx.Name, cfg); err != nil {
			ctx.Logger.Warn("[AutoCreate] Failed to persist DLC cache", slog.Any("error", err))
		}
	}
}

func detectWarlockDLC(ctx *context.Status) (bool, bool) {
	if ctx == nil || ctx.GameReader == nil {
		return false, false
	}

	hoverX := ui.GameVersionDLCHoverX
	hoverY := ui.GameVersionDLCHoverY
	refX := ui.GameVersionDLCCheckX
	refY := ui.GameVersionDLCCheckY
	sampleRadius := 2 // 5x5 area average

	ctx.HID.MovePointer(hoverX, hoverY)
	utils.Sleep(gameVersionSampleDelay)
	imgHover := ctx.GameReader.Screenshot()
	if imgHover == nil {
		return false, false
	}
	hoverR, hoverG, hoverB, ok := averageRGBAt(imgHover, hoverX, hoverY, sampleRadius)
	if !ok {
		ctx.Logger.Warn("[AutoCreate] DLC hover sample out of bounds",
			slog.Int("x", hoverX),
			slog.Int("y", hoverY))
		return false, false
	}

	ctx.HID.MovePointer(refX, refY)
	utils.Sleep(gameVersionSampleDelay)
	imgRef := ctx.GameReader.Screenshot()
	if imgRef == nil {
		return false, false
	}
	refR, refG, refB, ok := averageRGBAt(imgRef, refX, refY, sampleRadius)
	if !ok {
		ctx.Logger.Warn("[AutoCreate] DLC reference sample out of bounds",
			slog.Int("x", refX),
			slog.Int("y", refY))
		return false, false
	}

	dr := absInt(hoverR - refR)
	dg := absInt(hoverG - refG)
	db := absInt(hoverB - refB)
	sumDelta := dr + dg + db
	maxDelta := maxInt(dr, maxInt(dg, db))

	// DLC present signal: two hover areas should look very similar.
	hasDLC := sumDelta <= 42 && maxDelta <= 16

	ctx.Logger.Info("[AutoCreate] DLC hover compare",
		slog.Int("hoverX", hoverX),
		slog.Int("hoverY", hoverY),
		slog.Int("hoverR", hoverR),
		slog.Int("hoverG", hoverG),
		slog.Int("hoverB", hoverB),
		slog.Int("refX", refX),
		slog.Int("refY", refY),
		slog.Int("refR", refR),
		slog.Int("refG", refG),
		slog.Int("refB", refB),
		slog.Int("sumDelta", sumDelta),
		slog.Int("maxDelta", maxDelta),
		slog.Bool("hasDLC", hasDLC))

	return hasDLC, true
}

func normalizeGameVersion(version string) (string, bool) {
	switch strings.ToLower(strings.TrimSpace(version)) {
	case "", config.GameVersionReignOfTheWarlock, "reignofthewarlock", "reign of the warlock", "warlock":
		return "warlock", true
	case config.GameVersionExpansion:
		return "expansion", true
	default:
		return "warlock", false
	}
}

func averageRGBAt(img image.Image, x, y, radius int) (int, int, int, bool) {
	if img == nil {
		return 0, 0, 0, false
	}
	bounds := img.Bounds()
	if x-radius < bounds.Min.X || x+radius >= bounds.Max.X || y-radius < bounds.Min.Y || y+radius >= bounds.Max.Y {
		return 0, 0, 0, false
	}

	sumR := 0
	sumG := 0
	sumB := 0
	count := 0
	for py := y - radius; py <= y+radius; py++ {
		for px := x - radius; px <= x+radius; px++ {
			r16, g16, b16, _ := img.At(px, py).RGBA()
			sumR += int(r16 >> 8)
			sumG += int(g16 >> 8)
			sumB += int(b16 >> 8)
			count++
		}
	}
	if count == 0 {
		return 0, 0, 0, false
	}
	return sumR / count, sumG / count, sumB / count, true
}

func absInt(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func ensureForegroundWindow(ctx *context.Status) {
	if ctx == nil || ctx.GameReader == nil {
		return
	}
	hwnd := ctx.GameReader.HWND
	if hwnd == 0 {
		return
	}

	for i := 0; i < 3; i++ {
		win.ShowWindow(hwnd, win.SW_RESTORE)
		win.SetForegroundWindow(hwnd)
		win.BringWindowToTop(hwnd)
		win.SetActiveWindow(hwnd)
		win.SetFocus(hwnd)
		utils.Sleep(150)
		if win.GetForegroundWindow() == hwnd {
			return
		}
		utils.Sleep(150)
	}

	ctx.Logger.Warn("[AutoCreate] Failed to set foreground window before name input")
}

func enterCreationScreen(ctx *context.Status) error {
	for i := 0; i < 5; i++ {
		ctx.HID.Click(game.LeftButton, ui.CharCreateNewBtnX, ui.CharCreateNewBtnY)
		utils.Sleep(300)
		ctx.HID.Click(game.LeftButton, ui.CharCreateNewBtnX, ui.CharCreateNewBtnY)
		utils.Sleep(1000)
		if ctx.GameReader.IsInCharacterCreationScreen() {
			return nil
		}
	}
	return errors.New("failed to enter creation screen")
}

func getClassPosition(class string) ([2]int, error) {
	lowerClass := strings.ToLower(class)
	for k, pos := range classCoords {
		if strings.Contains(lowerClass, k) {
			return pos, nil
		}
	}
	return [2]int{}, fmt.Errorf("unknown class: %s", class)
}

func inputCharacterName(ctx *context.Status, name string) error {
	ctx.HID.Click(game.LeftButton, ui.CharNameInputX, ui.CharNameInputY)
	utils.Sleep(300)

	// Clear existing text
	for i := 0; i < 16; i++ {
		ctx.HID.PressKey(win.VK_BACK)
		utils.Sleep(20)
	}
	utils.Sleep(200)

	// Check for non-ASCII
	hasNonASCII := false
	for _, r := range name {
		if r > 127 {
			hasNonASCII = true
			break
		}
	}

	if hasNonASCII {
		return inputNonASCIIName(ctx, name)
	}
	return inputASCIIName(ctx, name)
}

func inputASCIIName(ctx *context.Status, name string) error {
	for _, r := range name {
		if err := sendUnicodeChar(r); err != nil {
			ctx.Logger.Error("Failed to send char", slog.String("char", string(r)), slog.Any("error", err))
			return err
		}
		utils.Sleep(60)
	}
	utils.Sleep(500)
	return nil
}

func inputNonASCIIName(ctx *context.Status, name string) error {
	ctx.Logger.Info("[AutoCreate] Using SendInput for non-ASCII name", slog.String("name", name))

	for _, r := range name {
		if err := sendUnicodeChar(r); err != nil {
			ctx.Logger.Error("Failed to send unicode char", slog.String("char", string(r)), slog.Any("error", err))
			return err
		}
		utils.Sleep(100)
	}
	utils.Sleep(500)
	return nil
}

func sendUnicodeChar(char rune) error {
	inputs := []INPUT{
		{INPUT_KEYBOARD, KEYBDINPUT{0, uint16(char), KEYEVENTF_UNICODE, 0, 0}, [8]byte{}},
		{INPUT_KEYBOARD, KEYBDINPUT{0, uint16(char), KEYEVENTF_UNICODE | KEYEVENTF_KEYUP, 0, 0}, [8]byte{}},
	}

	ret, _, err := procSendInput.Call(
		uintptr(len(inputs)),
		uintptr(unsafe.Pointer(&inputs[0])),
		unsafe.Sizeof(inputs[0]),
	)

	if ret == 0 {
		return fmt.Errorf("SendInput failed: %v", err)
	}
	return nil
}
