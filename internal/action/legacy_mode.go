package action

import (
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	legacySwitchPollAttempts = 4
	legacySwitchPollDelayMs  = 300
)

func SwitchToLegacyMode() {
	ctx := context.Get()
	ctx.SetLastAction("SwitchToLegacyMode")

	if !ctx.CharacterCfg.ClassicMode {
		return
	}

	enableLegacyMode(ctx, true)
}

func EnableLegacyMode() bool {
	ctx := context.Get()
	ctx.SetLastAction("EnableLegacyMode")

	return enableLegacyMode(ctx, false)
}

func enableLegacyMode(ctx *context.Status, closeMiniPanel bool) bool {
	if ctx == nil || ctx.GameReader == nil {
		return false
	}

	if ctx.Data.LegacyGraphics {
		return true
	}

	// Prevent toggling legacy mode while in lobby or character selection
	// so lobby-game joins are not affected by unintended legacy input.
	if ctx.GameReader.IsInLobby() || ctx.GameReader.IsInCharacterSelectionScreen() {
		return false
	}

	// Prevent toggeling legacy mode for DLC character (without logging)
	if ctx.Data.IsDLC() {
		return false
	}

	if len(ctx.Data.KeyBindings.LegacyToggle.Key1) == 0 {
		ctx.Logger.Warn("Legacy toggle key binding not configured, skipping legacy mode switch")
		return false
	}

	ctx.Logger.Debug("Switching to legacy mode...")
	ctx.HID.PressKey(ctx.Data.KeyBindings.LegacyToggle.Key1[0])
	if !waitForLegacyGraphicsState(ctx, true) {
		ctx.Logger.Debug("Legacy graphics did not activate after toggle input")
		return false
	}

	if closeMiniPanel {
		ctx.Logger.Debug("Closing mini panel...")
		ctx.HID.Click(game.LeftButton, ui.CloseMiniPanelClassicX, ui.CloseMiniPanelClassicY)
		utils.Sleep(100)
	}

	return true
}

func waitForLegacyGraphicsState(ctx *context.Status, expected bool) bool {
	for i := 0; i < legacySwitchPollAttempts; i++ {
		utils.Sleep(legacySwitchPollDelayMs)
		ctx.RefreshGameData()
		if ctx.Data.LegacyGraphics == expected {
			return true
		}
	}

	return false
}
