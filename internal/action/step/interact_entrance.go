package step

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/koolo/internal/context"
)

func InteractEntrance(area area.ID) error {
	ctx := context.Get()
	ctx.SetLastStep("InteractEntrance")

	// TESTING MODE: Only use packet-based interaction (no fallback)
	// This forces the bot to fail if packets don't work, so we can verify packet functionality
	success, packetErr := TryInteractEntrancePacket(area)
	if success {
		ctx.Logger.Debug("Entrance interaction succeeded via packet method")
		return nil
	}

	// If packet method failed, return the error (no fallback during testing)
	if packetErr != nil {
		ctx.Logger.Error("Packet entrance interaction failed - NO FALLBACK (testing mode)",
			"error", packetErr)
		return fmt.Errorf("packet entrance interaction failed (testing mode): %w", packetErr)
	}

	return fmt.Errorf("packet entrance interaction failed for unknown reason (testing mode)")

	// === FALLBACK DISABLED FOR TESTING ===
	// To re-enable mouse fallback after testing, restore the original mouse-based code
	// that was here (check git history or backup)
}
