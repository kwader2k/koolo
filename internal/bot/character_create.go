package bot

import (
	"errors"
	"fmt"
	"log/slog"
	"strings"
	"syscall"
	"unsafe"

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

	// 1. Enter character creation screen
	if !ctx.GameReader.IsInCharacterCreationScreen() {
		if err := enterCreationScreen(ctx); err != nil {
			return err
		}
	}

	ctx.SetLastAction("CreateCharacter")

	// 2. Select Class
	classPos, err := getClassPosition(class)
	if err != nil {
		return err
	}
	ctx.HID.Click(game.LeftButton, classPos[0], classPos[1])
	utils.Sleep(500)

	// 3. Toggle Ladder
	if !ctx.CharacterCfg.Game.IsNonLadderChar {
		ctx.HID.Click(game.LeftButton, ui.CharLadderBtnX, ui.CharLadderBtnY)
		utils.Sleep(300)
	}

	// 4. Toggle Hardcore
	if ctx.CharacterCfg.Game.IsHardCoreChar {
		ctx.HID.Click(game.LeftButton, ui.CharHardcoreBtnX, ui.CharHardcoreBtnY)
		utils.Sleep(300)
	}

	// 5. Input Name
	if err := inputCharacterName(ctx, name); err != nil {
		return err
	}

	// 6. Click Create Button
	ctx.HID.Click(game.LeftButton, ui.CharCreateBtnX, ui.CharCreateBtnY)
	utils.Sleep(1500)

	// 7. Confirm hardcore warning dialog
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

func enterCreationScreen(ctx *context.Status) error {
	for i := 0; i < 5; i++ {
		ctx.HID.Click(game.LeftButton, ui.CharCreateNewBtnX, ui.CharCreateNewBtnY)
		utils.Sleep(180)
		ctx.HID.Click(game.LeftButton, ui.CharCreateNewBtnX, ui.CharCreateNewBtnY)
		utils.Sleep(1500)
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