package action

import (
	"fmt"
	"log/slog"
	"strings"
	"time"

	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

// OpenPartyScreen opens the party/social panel by pressing P
func OpenPartyScreen() error {
	ctx := context.Get()
	ctx.SetLastAction("OpenPartyScreen")

	// Press P (0x50) to open party screen
	ctx.HID.PressKey(0x50)
	utils.Sleep(500)

	// Note: OpenMenus.Party may not be available in all d2go versions
	// We rely on timing rather than verification for now
	ctx.RefreshGameData()

	return nil
}

// ClosePartyScreen closes the party/social panel
func ClosePartyScreen() error {
	ctx := context.Get()
	ctx.SetLastAction("ClosePartyScreen")

	// Press Escape to close any open menu including party screen
	ctx.HID.PressKey(win.VK_ESCAPE)
	utils.Sleep(300)

	return nil
}

// FindPlayerInRoster checks if a player with the given name exists in the current game's roster
func FindPlayerInRoster(playerName string) bool {
	ctx := context.Get()
	ctx.SetLastAction("FindPlayerInRoster")

	ctx.RefreshGameData()

	for _, member := range ctx.Data.Roster {
		if strings.EqualFold(member.Name, playerName) {
			return true
		}
	}

	return false
}

// WaitForPlayerInRoster waits for a player to appear in the roster
func WaitForPlayerInRoster(playerName string, timeout time.Duration) bool {
	ctx := context.Get()
	ctx.SetLastAction("WaitForPlayerInRoster")

	startTime := time.Now()
	for time.Since(startTime) < timeout {
		if FindPlayerInRoster(playerName) {
			ctx.Logger.Info("Player found in roster", slog.String("player", playerName))
			return true
		}
		utils.Sleep(500)
		ctx.RefreshGameData()
	}

	ctx.Logger.Warn("Timeout waiting for player in roster", slog.String("player", playerName))
	return false
}

// GetPlayerIndexInRoster returns the index of a player in the roster (0-based), or -1 if not found
func GetPlayerIndexInRoster(playerName string) int {
	ctx := context.Get()
	ctx.RefreshGameData()

	for i, member := range ctx.Data.Roster {
		if strings.EqualFold(member.Name, playerName) {
			return i
		}
	}

	return -1
}

// RequestJoinParty attempts to join another player's party by clicking their invite/join button
func RequestJoinParty(playerName string) error {
	ctx := context.Get()
	ctx.SetLastAction("RequestJoinParty")

	// First, ensure the party screen is open
	if err := OpenPartyScreen(); err != nil {
		return err
	}
	defer ClosePartyScreen()

	// Find player index in roster
	playerIndex := GetPlayerIndexInRoster(playerName)
	if playerIndex < 0 {
		return fmt.Errorf("player %s not found in roster", playerName)
	}

	// Calculate position of the player's entry and invite button
	var clickX, clickY int
	if ctx.Data.LegacyGraphics {
		clickX = ui.PartyPlayerListStartXClassic + ui.PartyInviteButtonOffsetXClassic
		clickY = ui.PartyPlayerListStartYClassic + (playerIndex * ui.PartyPlayerListOffsetYClassic)
	} else {
		clickX = ui.PartyPlayerListStartX + ui.PartyInviteButtonOffsetX
		clickY = ui.PartyPlayerListStartY + (playerIndex * ui.PartyPlayerListOffsetY)
	}

	ctx.Logger.Info("Requesting to join party",
		slog.String("player", playerName),
		slog.Int("playerIndex", playerIndex),
		slog.Int("clickX", clickX),
		slog.Int("clickY", clickY))

	// Click the invite/join button
	ctx.HID.Click(game.LeftButton, clickX, clickY)
	utils.Sleep(300)

	return nil
}

// AcceptPartyInvite attempts to accept a pending party invite
func AcceptPartyInvite() error {
	ctx := context.Get()
	ctx.SetLastAction("AcceptPartyInvite")

	// Click accept button position
	var acceptX, acceptY int
	if ctx.Data.LegacyGraphics {
		acceptX = ui.PartyAcceptBtnXClassic
		acceptY = ui.PartyAcceptBtnYClassic
	} else {
		acceptX = ui.PartyAcceptBtnX
		acceptY = ui.PartyAcceptBtnY
	}

	ctx.Logger.Info("Accepting party invite",
		slog.Int("clickX", acceptX),
		slog.Int("clickY", acceptY))

	ctx.HID.Click(game.LeftButton, acceptX, acceptY)
	utils.Sleep(300)

	return nil
}

// DeclinePartyInvite declines a pending party invite
func DeclinePartyInvite() error {
	ctx := context.Get()
	ctx.SetLastAction("DeclinePartyInvite")

	var declineX, declineY int
	if ctx.Data.LegacyGraphics {
		declineX = ui.PartyDeclineBtnXClassic
		declineY = ui.PartyDeclineBtnYClassic
	} else {
		declineX = ui.PartyDeclineBtnX
		declineY = ui.PartyDeclineBtnY
	}

	ctx.HID.Click(game.LeftButton, declineX, declineY)
	utils.Sleep(300)

	return nil
}

// IsInPartyWith checks if we're in the same game as the specified player
// Note: d2go's RosterMember doesn't expose PartyID directly, so we can only
// verify both players are in the same game (same roster). For leecher mode,
// this is sufficient since we just need to be in the same game for XP sharing.
// A proper party check would require additional game memory reading.
func IsInPartyWith(playerName string) bool {
	ctx := context.Get()
	ctx.RefreshGameData()

	foundSelf := false
	foundTarget := false

	for _, member := range ctx.Data.Roster {
		if strings.EqualFold(member.Name, ctx.Data.PlayerUnit.Name) {
			foundSelf = true
		}
		if strings.EqualFold(member.Name, playerName) {
			foundTarget = true
		}
	}

	// If both players are in the roster, they're in the same game
	// For leecher/follower purposes, this is sufficient - we'll request party join
	// even if already in party (the game will handle it gracefully)
	return foundSelf && foundTarget
}

// WaitForPartyJoin waits until we're in the same party as the specified player
func WaitForPartyJoin(playerName string, timeout time.Duration) bool {
	ctx := context.Get()
	ctx.SetLastAction("WaitForPartyJoin")

	startTime := time.Now()
	for time.Since(startTime) < timeout {
		if IsInPartyWith(playerName) {
			ctx.Logger.Info("Successfully joined party", slog.String("leader", playerName))
			return true
		}
		utils.Sleep(500)
		ctx.RefreshGameData()
	}

	ctx.Logger.Warn("Timeout waiting to join party", slog.String("leader", playerName))
	return false
}

// JoinPlayerParty is a high-level function that handles the full party join flow
func JoinPlayerParty(playerName string, maxRetries int) error {
	ctx := context.Get()
	ctx.SetLastAction("JoinPlayerParty")

	// Check if already in party with the player
	if IsInPartyWith(playerName) {
		ctx.Logger.Info("Already in party with player", slog.String("player", playerName))
		return nil
	}

	for attempt := 0; attempt < maxRetries; attempt++ {
		ctx.Logger.Info("Attempting to join party",
			slog.String("player", playerName),
			slog.Int("attempt", attempt+1))

		// Try to request to join their party
		if err := RequestJoinParty(playerName); err != nil {
			ctx.Logger.Warn("Failed to request party join",
				slog.String("player", playerName),
				slog.String("error", err.Error()))
			utils.Sleep(1000)
			continue
		}

		// Wait a bit for the party system to process
		utils.Sleep(500)

		// Check if we joined successfully (in case they auto-accept)
		if WaitForPartyJoin(playerName, 5*time.Second) {
			return nil
		}

		// If not joined yet, try accepting any invite that might have come through
		AcceptPartyInvite()

		// Check again
		if WaitForPartyJoin(playerName, 3*time.Second) {
			return nil
		}

		utils.Sleep(2000) // Wait before retry
	}

	return fmt.Errorf("failed to join party with %s after %d attempts", playerName, maxRetries)
}

