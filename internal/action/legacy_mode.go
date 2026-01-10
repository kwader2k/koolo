package action

import (
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

func SwitchToLegacyMode() {
	ctx := context.Get()
	ctx.SetLastAction("SwitchToLegacyMode")

	// Prevent toggling legacy mode while in lobby or character selection
	// so lobby-game joins are not affected by unintended legacy input.

	if ctx.CharacterCfg.ClassicMode && !ctx.Data.LegacyGraphics {
		if ctx.GameReader.IsInLobby() || ctx.GameReader.IsInCharacterSelectionScreen() {
			return
		}

		ctx.Logger.Debug("Switching to legacy mode...")
		ctx.HID.PressKey(ctx.Data.KeyBindings.LegacyToggle.Key1[0])
		utils.Sleep(500) // Add small delay to allow the game to switch

		// Close the mini panel if option is enabled
		if ctx.CharacterCfg.CloseMiniPanel {
			utils.Sleep(100)
			ctx.HID.Click(game.LeftButton, ui.CloseMiniPanelClassicX, ui.CloseMiniPanelClassicY)
			utils.Sleep(100)
		}
	}
}

// ForceSwitchToLegacyMode switches to legacy graphics regardless of ClassicMode setting
// Used by follower bots when UseLegacyGraphics is enabled
func ForceSwitchToLegacyMode() {
	ctx := context.Get()
	ctx.SetLastAction("ForceSwitchToLegacyMode")

	// Check if already in legacy mode using direct memory read
	if ctx.GameReader.LegacyGraphics() {
		ctx.Logger.Debug("Already in legacy mode, skipping switch")
		return
	}

	// Prevent toggling while in lobby or character selection
	if ctx.GameReader.IsInLobby() || ctx.GameReader.IsInCharacterSelectionScreen() {
		ctx.Logger.Debug("In lobby/character selection, skipping legacy mode switch")
		return
	}

	// Wait for game to be fully loaded and character to spawn
	ctx.RefreshGameData()
	for i := 0; i < 50; i++ { // Up to 5 seconds
		if ctx.Data.PlayerUnit.Area != 0 {
			break
		}
		utils.Sleep(100)
		ctx.RefreshGameData()
	}

	// Short delay for UI to be ready (map data is now fetched separately)
	utils.Sleep(1000)

	// Get the legacy toggle key - use 'G' (0x47) as fallback if not bound
	legacyKey := ctx.Data.KeyBindings.LegacyToggle.Key1[0]
	if legacyKey == 0 {
		legacyKey = 0x47 // Default 'G' key
		ctx.Logger.Debug("LegacyToggle key not bound, using default 'G' key")
	}

	// Retry loop - press key and verify with memory read, retry if needed
	maxRetries := 3
	for attempt := 1; attempt <= maxRetries; attempt++ {
		// Check again before pressing (memory read is instant)
		if ctx.GameReader.LegacyGraphics() {
			ctx.Logger.Debug("Legacy mode now active, no need to press key")
			return
		}

		ctx.Logger.Info("Switching to legacy graphics mode", "key", legacyKey, "attempt", attempt)
		ctx.HID.PressKey(legacyKey)

		// Wait and verify the switch with multiple checks
		for i := 0; i < 20; i++ { // Check for up to 2 seconds
			utils.Sleep(100)
			if ctx.GameReader.LegacyGraphics() {
				ctx.Logger.Info("Successfully switched to legacy graphics mode")
				return
			}
		}

		if attempt < maxRetries {
			ctx.Logger.Warn("Legacy mode switch not detected, retrying...", "attempt", attempt)
			utils.Sleep(500) // Wait before retry
		}
	}

	ctx.Logger.Warn("Failed to verify legacy mode switch after all retries")
}
