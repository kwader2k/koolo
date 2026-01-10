package config

import (
	"fmt"
	"os"
	"strings"

	"github.com/lxn/win"
	cp "github.com/otiai10/copy"
)

var userProfile = os.Getenv("USERPROFILE")
var settingsPath = userProfile + "\\Saved Games\\Diablo II Resurrected"

func ReplaceGameSettings(modName string) error {
	// All koolo mods use savepath "koolo/", so settings go to the shared koolo folder
	// This ensures consistent settings across all supervisor-specific mods
	targetModDir := "koolo"
	if !strings.HasPrefix(modName, "koolo") {
		// For non-koolo mods, use the mod-specific folder
		targetModDir = modName
	}

	modDirPath := settingsPath + "\\mods\\" + targetModDir
	modSettingsPath := modDirPath + "\\Settings.json"

	if _, err := os.Stat(settingsPath); os.IsNotExist(err) {
		return fmt.Errorf("game settings not found at %s", settingsPath)
	}

	if _, err := os.Stat(modDirPath); os.IsNotExist(err) {
		err = os.MkdirAll(modDirPath, os.ModePerm)
		if err != nil {
			return fmt.Errorf("error creating mod folder to store settings: %w", err)
		}
	}

	if _, err := os.Stat(modSettingsPath + ".bkp"); os.IsExist(err) {
		err = os.Rename(modSettingsPath, modSettingsPath+".bkp")
		// File does not exist, no need to back up
		if err != nil && !os.IsNotExist(err) {
			return err
		}
	}

	return cp.Copy("config/Settings.json", modSettingsPath)
}

func InstallMod(modName string) error {
	if modName == "" {
		modName = "koolo"
	}
	
	if _, err := os.Stat(Koolo.D2RPath + "\\d2r.exe"); os.IsNotExist(err) {
		return fmt.Errorf("game not found at %s", Koolo.D2RPath)
	}

	modPath := Koolo.D2RPath + "\\mods\\" + modName + "\\" + modName + ".mpq"
	modInfoPath := modPath + "\\modinfo.json"

	if _, err := os.Stat(modInfoPath); err == nil {
		return nil
	}

	if err := os.MkdirAll(modPath, os.ModePerm); err != nil {
		return fmt.Errorf("error creating mod folder: %w", err)
	}

	// Use "koolo" as savepath base to share save files across all koolo instances
	modFileContent := []byte(fmt.Sprintf(`{"name":"%s","savepath":"koolo/"}`, modName))

	return os.WriteFile(modInfoPath, modFileContent, 0644)
}

func GetCurrentDisplayScale() float64 {
	hDC := win.GetDC(0)
	defer win.ReleaseDC(0, hDC)
	dpiX := win.GetDeviceCaps(hDC, win.LOGPIXELSX)

	return float64(dpiX) / 96.0
}

