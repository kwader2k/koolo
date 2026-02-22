package mule

import (
	"log/slog"

	"github.com/hectorgimenez/koolo/internal/config"
)

// Manager is responsible for managing the muling process.
type Manager struct {
	logger *slog.Logger
}

// NewManager creates a new MuleManager.
func NewManager(logger *slog.Logger) *Manager {
	return &Manager{
		logger: logger,
	}
}

// ShouldMule checks if the stash is full and muling is required.
func (m *Manager) ShouldMule(stashFull bool, characterName string) (bool, string) {
	for _, char := range config.GetCharacters() {
		// Updated logic: Check if MuleProfiles list is not empty instead of SwitchToMule
		if char.CharacterName == characterName && char.Muling.Enabled && len(char.Muling.MuleProfiles) > 0 {
			if stashFull {
				firstMule := char.Muling.MuleProfiles[0]
				m.logger.Info("Stash is full, muling is required.", "switchToMule", firstMule)
				return true, firstMule
			}
		}
	}

	return false, ""
}

// IsMuleCharacter checks if the character is configured as a mule.
// (ReturnTo).
func IsMuleCharacter(characterName string) bool {
	for name, char := range config.GetCharacters() {
		if name == characterName {
			return char.Muling.Enabled && char.Muling.ReturnTo != ""
		}
	}
	return false
}
