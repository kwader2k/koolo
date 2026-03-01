package bot

import (
	"errors"
	"fmt"
	"log/slog"
	"sort"
	"strings"
	"syscall"
	"unsafe"

	d2data "github.com/hectorgimenez/d2go/pkg/data"
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
	"druid": {ui.CharDruidX, ui.CharDruidY}, "warlock": {ui.CharWarlockX, ui.CharWarlockY},
}

const (
	INPUT_KEYBOARD    = 1
	KEYEVENTF_UNICODE = 0x0004
	KEYEVENTF_KEYUP   = 0x0002

	gameVersionMenuOpenDelay = 400
	expansionUpPresses       = 3
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

	if !isPanelVisible(ctx, "DropdownListContents") {
		ctx.Logger.Info("[AutoCreate] Opening game version dropdown for option read")
		ctx.HID.Click(game.LeftButton, ui.GameVersionBtnX, ui.GameVersionBtnY)
		utils.Sleep(gameVersionMenuOpenDelay)
	}

	options := getGameVersionOptionsWithRetry(ctx)
	hasWarlock := containsGameVersionOption(options, "warlock")
	hasExpansion := containsGameVersionOption(options, "expansion")

	ctx.Logger.Info("[AutoCreate] Game version options from panel",
		slog.Any("options", options),
		slog.Bool("hasWarlock", hasWarlock),
		slog.Bool("hasExpansion", hasExpansion))

	// Panel detection is fallback-only metadata; primary behavior uses requested version.
	if len(options) > 0 {
		cacheDLCEnabled(ctx, hasWarlock)
	}

	// Requested ROTW/warlock: do not change game version selection.
	if version == "warlock" {
		// Keep current selection; close dropdown to avoid intercepting later class clicks.
		if isPanelVisible(ctx, "DropdownListContents") {
			ctx.HID.Click(game.LeftButton, ui.GameVersionBtnX, ui.GameVersionBtnY)
			utils.Sleep(180)
		}
		ctx.Logger.Info("[AutoCreate] Warlock requested, skipping game version selection input")
		return
	}

	// Expansion selection strategy (works for both DLC and non-DLC accounts):
	// move to top with UP presses, then one DOWN to land on Expansion.
	if version == "expansion" {
		if !isPanelVisible(ctx, "DropdownListContents") {
			ctx.HID.Click(game.LeftButton, ui.GameVersionBtnX, ui.GameVersionBtnY)
			utils.Sleep(gameVersionMenuOpenDelay)
		} else {
			ctx.Logger.Info("[AutoCreate] Game version dropdown already open, selecting expansion via keyboard sequence")
		}

		for i := 0; i < expansionUpPresses; i++ {
			ctx.HID.PressKey(win.VK_UP)
			utils.Sleep(90)
		}
		ctx.HID.PressKey(win.VK_DOWN)
		utils.Sleep(120)
		ctx.HID.PressKey(win.VK_RETURN)
		utils.Sleep(250)
		return
	}

	ctx.Logger.Info("[AutoCreate] Unsupported normalized game version, leaving game version unchanged",
		slog.String("gameVersion", version))
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

func sanitizePanelText(text string) string {
	if text == "" {
		return ""
	}

	text = strings.ReplaceAll(text, "\r", " ")
	text = strings.ReplaceAll(text, "\n", " ")
	text = strings.TrimSpace(text)
	if len(text) > 160 {
		return text[:160]
	}

	return text
}

func isPanelVisible(ctx *context.Status, panelName string) bool {
	if ctx == nil || ctx.GameReader == nil || panelName == "" {
		return false
	}

	panel := ctx.GameReader.GetPanel(panelName)
	return panel.PanelName != "" && panel.PanelVisible
}

func getGameVersionDropdownOptions(ctx *context.Status) []string {
	if ctx == nil || ctx.GameReader == nil {
		return nil
	}

	container := ctx.GameReader.GetPanel("DropdownListContents", "Items", "View", "Container")
	if container.PanelName == "" || len(container.PanelChildren) == 0 {
		return nil
	}

	childNames := make([]string, 0, len(container.PanelChildren))
	for name := range container.PanelChildren {
		childNames = append(childNames, name)
	}
	sort.Strings(childNames)

	options := make([]string, 0, len(childNames))
	for _, childName := range childNames {
		row := container.PanelChildren[childName]
		text := panelTextValue(row.PanelChildren["TextBox"])
		text = sanitizePanelText(text)
		if text == "" {
			continue
		}
		options = append(options, text)
	}

	return options
}

func getGameVersionOptionsWithRetry(ctx *context.Status) []string {
	options := getGameVersionDropdownOptions(ctx)
	if len(options) > 0 {
		return options
	}

	if ctx != nil {
		ctx.Logger.Warn("[AutoCreate] options read failed/empty, retrying after delay")
	}

	utils.Sleep(1000)
	options = getGameVersionDropdownOptions(ctx)
	if len(options) == 0 && ctx != nil {
		ctx.Logger.Warn("[AutoCreate] options read failed/empty after retry")
	}

	return options
}

func panelTextValue(panel d2data.Panel) string {
	// DropdownListContents rows can intermittently expose garbled data in ExtraText2.
	// Keep ExtraText ahead of ExtraText2 so we prefer stable, readable text when ExtraText3 is empty.
	for _, candidate := range []string{panel.ExtraText3, panel.ExtraText, panel.ExtraText2} {
		if strings.TrimSpace(candidate) != "" {
			return candidate
		}
	}
	return ""
}

func containsGameVersionOption(options []string, version string) bool {
	version = strings.ToLower(strings.TrimSpace(version))
	if version == "" {
		return false
	}

	for _, option := range options {
		normalized := strings.ToLower(strings.TrimSpace(option))
		switch version {
		case "warlock":
			if strings.Contains(normalized, "warlock") {
				return true
			}
		case "expansion":
			if normalized == "expansion" {
				return true
			}
		default:
			if normalized == version {
				return true
			}
		}
	}

	return false
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

	return inputName(ctx, name)
}

func inputName(ctx *context.Status, name string) error {
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
