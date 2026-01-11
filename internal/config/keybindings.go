package config

import (
	"encoding/binary"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/lxn/win"
)

type KeyBindingEnsureResult struct {
	Updated bool
	Missing bool
	SaveDir string
	Files   []string
}

var skillActionIDs = []uint16{
	0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
	0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
}

var skillKeyPriority = []uint16{
	uint16(win.VK_F1), uint16(win.VK_F2), uint16(win.VK_F3), uint16(win.VK_F4),
	uint16(win.VK_F5), uint16(win.VK_F6), uint16(win.VK_F7), uint16(win.VK_F8),
	uint16(win.VK_F9), uint16(win.VK_F10), uint16(win.VK_F11), uint16(win.VK_F12),
	uint16('1'), uint16('2'), uint16('3'), uint16('4'),
	uint16('5'), uint16('6'), uint16('7'), uint16('8'), uint16('9'), uint16('0'),
	uint16(win.VK_OEM_MINUS), uint16(win.VK_OEM_PLUS),
}

func EnsureSkillKeyBindings(cfg *CharacterCfg, useCustomSettings bool) (KeyBindingEnsureResult, error) {
	result := KeyBindingEnsureResult{
		SaveDir: resolveSaveDir(cfg.CommandLineArgs, useCustomSettings),
	}
	saveDir := result.SaveDir
	if saveDir == "" {
		return result, nil
	}

	if _, err := os.Stat(saveDir); os.IsNotExist(err) {
		result.Missing = true
		return result, nil
	}

	characterName := strings.TrimSpace(cfg.CharacterName)
	if characterName == "" {
		result.Missing = true
		return result, nil
	}

	path := filepath.Join(saveDir, characterName+".key")
	result.Files = []string{path}

	if _, err := os.Stat(path); err != nil {
		if os.IsNotExist(err) {
			result.Missing = true
			return result, nil
		}
		return result, err
	}

	updated, err := ensureSkillKeyBindingsInFile(path)
	if err != nil {
		return result, err
	}
	result.Updated = updated

	return result, nil
}

func resolveSaveDir(commandLineArgs string, useCustomSettings bool) string {
	modName := ""
	args := strings.Fields(commandLineArgs)
	for i, arg := range args {
		if strings.EqualFold(arg, "-mod") && i+1 < len(args) {
			modName = args[i+1]
			break
		}
	}
	if useCustomSettings && modName == "" {
		modName = "koolo"
	}
	if modName == "" {
		return settingsPath
	}
	return filepath.Join(settingsPath, "mods", modName)
}

func WaitForKeyBindings(saveDir, characterName string, timeout time.Duration) error {
	if saveDir == "" {
		return fmt.Errorf("save dir is empty")
	}
	characterName = strings.TrimSpace(characterName)
	if characterName == "" {
		return fmt.Errorf("character name is empty")
	}

	deadline := time.Now().Add(timeout)
	path := filepath.Join(saveDir, characterName+".key")
	for time.Now().Before(deadline) {
		if _, err := os.Stat(path); err == nil {
			return nil
		}
		time.Sleep(500 * time.Millisecond)
	}

	return fmt.Errorf("timed out waiting for key binding file in %s", saveDir)
}

func ensureSkillKeyBindingsInFile(path string) (bool, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return false, err
	}
	if len(data)%2 != 0 {
		return false, fmt.Errorf("unexpected key file length for %s", path)
	}

	pairs := make([]uint16, len(data)/2)
	for i := range pairs {
		pairs[i] = binary.LittleEndian.Uint16(data[i*2:])
	}

	changed := applyMissingSkillKeyBindings(pairs)

	if !changed {
		return false, nil
	}

	for i, val := range pairs {
		binary.LittleEndian.PutUint16(data[i*2:], val)
	}
	return true, os.WriteFile(path, data, 0644)
}

func applyMissingSkillKeyBindings(pairs []uint16) bool {
	usedKeys := collectUsedKeys(pairs)
	indicesByAction, ok := primaryKeyIndicesByRecord(pairs)
	changed := false

	for _, actionID := range skillActionIDs {
		var indices []int
		if ok {
			indices = indicesByAction[actionID]
		} else {
			indices = findPrimaryKeyIndices(pairs, actionID)
		}
		if len(indices) == 0 {
			continue
		}

		targetKey := uint16(0)
		for _, idx := range indices {
			if !isKeyEmpty(pairs[idx]) {
				targetKey = pairs[idx]
				break
			}
		}
		if targetKey == 0 {
			targetKey = nextAvailableKey(usedKeys)
			if targetKey == 0 {
				continue
			}
			usedKeys[targetKey] = struct{}{}
		} else {
			usedKeys[targetKey] = struct{}{}
		}

		for _, idx := range indices {
			if isKeyEmpty(pairs[idx]) {
				pairs[idx] = targetKey
				changed = true
			}
		}
	}

	return changed
}

func collectUsedKeys(pairs []uint16) map[uint16]struct{} {
	used := make(map[uint16]struct{})
	if keys, ok := collectUsedKeysByRecord(pairs); ok {
		return keys
	}

	for i := 0; i <= len(pairs)-4; i++ {
		if pairs[i+1] != 0 {
			continue
		}
		flag := pairs[i+3]
		if flag != 0 && flag != 1 {
			continue
		}
		key := pairs[i+2]
		if isKeyEmpty(key) {
			continue
		}
		used[key] = struct{}{}
	}

	return used
}

func collectUsedKeysByRecord(pairs []uint16) (map[uint16]struct{}, bool) {
	if len(pairs) < 11 || (len(pairs)-1)%10 != 0 {
		return nil, false
	}

	used := make(map[uint16]struct{})
	for i := 1; i+9 < len(pairs); i += 10 {
		if pairs[i+3] != 1 || pairs[i+5] != pairs[i] || pairs[i+6] != 0 {
			return nil, false
		}
		key1 := pairs[i+2]
		key2 := pairs[i+7]
		if !isKeyEmpty(key1) {
			used[key1] = struct{}{}
		}
		if !isKeyEmpty(key2) {
			used[key2] = struct{}{}
		}
	}

	return used, true
}

func primaryKeyIndicesByRecord(pairs []uint16) (map[uint16][]int, bool) {
	if len(pairs) < 11 || (len(pairs)-1)%10 != 0 {
		return nil, false
	}

	indices := make(map[uint16][]int)
	for i := 1; i+9 < len(pairs); i += 10 {
		if pairs[i+3] != 1 || pairs[i+5] != pairs[i] || pairs[i+6] != 0 {
			return nil, false
		}
		actionID := pairs[i]
		indices[actionID] = append(indices[actionID], i+2)
	}

	return indices, true
}

func findPrimaryKeyIndices(pairs []uint16, actionID uint16) []int {
	indices := make([]int, 0)
	for i := 0; i <= len(pairs)-4; i++ {
		if pairs[i] == actionID && pairs[i+1] == 0 && pairs[i+3] == 1 {
			indices = append(indices, i+2)
		}
	}
	return indices
}

func nextAvailableKey(used map[uint16]struct{}) uint16 {
	for _, key := range skillKeyPriority {
		if isKeyEmpty(key) {
			continue
		}
		if _, exists := used[key]; exists {
			continue
		}
		return key
	}
	return 0
}

func isKeyEmpty(key uint16) bool {
	return key == 0 || key == 0xFFFF
}
