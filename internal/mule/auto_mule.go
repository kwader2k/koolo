package mule

import (
	"crypto/rand"
	"fmt"
	"log/slog"
	"math/big"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/koolo/internal/config"
)

const (
	// muleNamePrefix prefixed to every auto-generated mule name.
	muleNamePrefix = "Mule"
	// muleNameSuffixLen is the number of random letters appended.
	muleNameSuffixLen = 6
	// defaultMuleGameClass is the D2R class used when auto-creating mule characters in-game.
	defaultMuleGameClass = "sorc"
)

// GenerateMuleName creates a random, D2R-valid character name (letters only, 2-15 chars).
// Format: "Mule" + 6 random lowercase letters, e.g. "Mulexvkqbr".
func GenerateMuleName() (string, error) {
	const letters = "abcdefghijklmnopqrstuvwxyz"
	suffix := make([]byte, muleNameSuffixLen)
	for i := range suffix {
		idx, err := rand.Int(rand.Reader, big.NewInt(int64(len(letters))))
		if err != nil {
			return "", fmt.Errorf("failed to generate random mule name: %w", err)
		}
		suffix[i] = letters[idx.Int64()]
	}
	return muleNamePrefix + string(suffix), nil
}

// CreateAutoMuleProfile generates a new mule profile from the farmer's config.
// It creates the config directory, sets up the mule character config, adds the
// mule to the farmer's MuleProfiles, and saves both configs.
// Returns the generated mule profile name.
func CreateAutoMuleProfile(logger *slog.Logger, farmerName string, farmerCfg *config.CharacterCfg) (string, error) {
	muleName, err := GenerateMuleName()
	if err != nil {
		return "", err
	}

	// Ensure generated name doesn't collide with existing profiles
	existingChars := config.GetCharacters()
	for attempts := 0; attempts < 10; attempts++ {
		if _, exists := existingChars[muleName]; !exists {
			break
		}
		muleName, err = GenerateMuleName()
		if err != nil {
			return "", err
		}
	}
	if _, exists := existingChars[muleName]; exists {
		return "", fmt.Errorf("failed to generate unique mule name after retries")
	}

	logger.Info("Creating auto-mule profile",
		slog.String("farmer", farmerName),
		slog.String("muleName", muleName))

	// Create config directory from template
	if err := config.CreateFromTemplate(muleName); err != nil {
		return "", fmt.Errorf("failed to create mule config from template: %w", err)
	}

	// Load the newly created config
	muleCfg, found := config.GetCharacter(muleName)
	if !found || muleCfg == nil {
		return "", fmt.Errorf("failed to load auto-mule config after creation")
	}

	// Configure as a mule character
	muleCfg.CharacterName = muleName
	muleCfg.Character.Class = "mule"
	muleCfg.AutoCreateCharacter = true

	// Copy auth credentials from farmer
	muleCfg.Username = farmerCfg.Username
	muleCfg.Password = farmerCfg.Password
	muleCfg.AuthMethod = farmerCfg.AuthMethod
	muleCfg.AuthToken = farmerCfg.AuthToken
	muleCfg.Realm = farmerCfg.Realm
	muleCfg.CommandLineArgs = farmerCfg.CommandLineArgs

	// Copy game version settings from farmer
	muleCfg.Game.GameVersion = farmerCfg.Game.GameVersion
	muleCfg.Game.IsNonLadderChar = farmerCfg.Game.IsNonLadderChar
	muleCfg.Game.IsHardCoreChar = false // Mules should never be hardcore
	muleCfg.Game.DLCEnabled = farmerCfg.Game.DLCEnabled

	// Set runs to only the mule run and difficulty to Normal (new level 1 character)
	muleCfg.Game.Runs = []config.Run{config.MuleRun}
	muleCfg.Game.Difficulty = difficulty.Normal

	// Unlock all inventory slots for maximum item transfer capacity
	muleCfg.Inventory.InventoryLock = [][]int{
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	}

	// Set up muling config on the mule side
	muleCfg.Muling.Enabled = true
	muleCfg.Muling.ReturnTo = farmerName

	// Mules must kill D2R on stop so the crash detector fires and triggers the
	// character switch flow (restartFunc). Without this, the client would stay
	// alive at the character selection screen and the next supervisor never starts.
	muleCfg.KillD2OnStop = true
	muleCfg.ClassicMode = farmerCfg.ClassicMode

	// Save the mule config
	if err := config.SaveSupervisorConfig(muleName, muleCfg); err != nil {
		return "", fmt.Errorf("failed to save auto-mule config: %w", err)
	}

	// Add the mule to the farmer's MuleProfiles and save
	farmerCfg.Muling.MuleProfiles = append(farmerCfg.Muling.MuleProfiles, muleName)
	if err := config.SaveSupervisorConfig(farmerName, farmerCfg); err != nil {
		return "", fmt.Errorf("failed to save farmer config with new mule profile: %w", err)
	}

	logger.Info("Auto-mule profile created successfully",
		slog.String("muleName", muleName),
		slog.String("farmer", farmerName))

	return muleName, nil
}

// MuleGameClass returns the D2R class to use when auto-creating a mule character in-game.
// The "mule" class is a koolo-internal concept; we need a real D2R class for character creation.
func MuleGameClass() string {
	return defaultMuleGameClass
}

// IsMuleClass returns true if the given class string represents a mule (koolo-internal class).
func IsMuleClass(class string) bool {
	return strings.EqualFold(class, "mule")
}
