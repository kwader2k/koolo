package obs

import (
	"encoding/json"
	"fmt"
	"log/slog"
	"math/rand"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"time"

	"github.com/gorilla/websocket"
	"github.com/hectorgimenez/koolo/internal/config"
)

var obsProfileNames = []string{
	"d2r-replay-capture",
	"game-session-record",
	"desktop-capture-d2",
	"window-record-live",
	"capture-session-01",
	"display-record-main",
	"screen-capture-live",
	"gaming-session-rec",
	"window-session-cap",
	"local-record-buffer",
}

var obsSceneNames = []string{
	"d2r-instance",
	"game-window-main",
	"desktop-scene-01",
	"capture-scene-live",
	"window-scene-main",
	"gaming-display-cap",
	"local-game-scene",
	"screen-scene-main",
	"display-scene-live",
	"window-capture-main",
}

var obsProcess *exec.Cmd

func getOBSAppDataPath() string {
	return filepath.Join(os.Getenv("APPDATA"), "obs-studio")
}

func generateUUID() string {
	b := make([]byte, 16)
	rand.Read(b)
	return fmt.Sprintf("%x-%x-%x-%x-%x", b[0:4], b[4:6], b[6:8], b[8:10], b[10:])
}

// FindOBSPath looks for obs-studio in the same directory as obfuscated koolo.exe.
// Expected structure:
//
//	build/
//	  koolo.exe (obfuscated)
//	  obs-studio/     <- OBS install (download from obsproject.com)
func FindOBSPath() (string, error) {
	if config.Koolo.OBS.OBSPath != "" {
		p := strings.TrimRight(config.Koolo.OBS.OBSPath, `\/`)

		if strings.HasSuffix(strings.ToLower(p), ".exe") {
			// Full path to exe provided
		} else if strings.HasSuffix(strings.ToLower(p), "64bit") {
			// Path to 64bit folder provided
			p = filepath.Join(p, "obs64.exe")
		} else if strings.HasSuffix(strings.ToLower(p), "bin") {
			// Path to bin folder provided
			p = filepath.Join(p, "64bit", "obs64.exe")
		} else {
			// Assume root obs-studio folder provided
			p = filepath.Join(p, "bin", "64bit", "obs64.exe")
		}

		if _, err := os.Stat(p); err != nil {
			return "", fmt.Errorf("OBS not found at configured path %s", p)
		}
		return p, nil
	}

	exePath, err := os.Executable()
	if err != nil {
		return "", err
	}
	obsPath := filepath.Clean(filepath.Join(filepath.Dir(exePath), "obs-studio", "bin", "64bit", "obs64.exe"))
	if _, err := os.Stat(obsPath); err != nil {
		return "", fmt.Errorf("OBS not found at %s", obsPath)
	}
	return obsPath, nil
}

func writeWebSocketConfig() {
	appData := getOBSAppDataPath()
	wsConfigDir := filepath.Join(appData, "plugin_config", "obs-websocket")
	os.MkdirAll(wsConfigDir, 0755)
	wsConfig := fmt.Sprintf(`{"alerts_enabled":false,"auth_required":false,"first_load":false,"server_enabled":true,"server_password":"","server_port":%d}`,
		config.Koolo.OBS.WebSocketPort)
	os.WriteFile(filepath.Join(wsConfigDir, "config.json"), []byte(wsConfig), 0644)
}

func killOBS() {
	cmd := exec.Command("taskkill", "/F", "/IM", "obs64.exe")
	cmd.Run()
	time.Sleep(2 * time.Second)
}

func getOrGenerateNames() (profileName, sceneName string) {
	profileName = config.Koolo.OBS.ProfileName
	sceneName = config.Koolo.OBS.SceneName

	changed := false
	if config.Koolo.OBS.WebSocketPort == 0 {
		config.Koolo.OBS.WebSocketPort = 4455
		changed = true
	}
	if profileName == "" {
		profileName = obsProfileNames[rand.Intn(len(obsProfileNames))]
		config.Koolo.OBS.ProfileName = profileName
		changed = true
	}
	if sceneName == "" {
		sceneName = obsSceneNames[rand.Intn(len(obsSceneNames))]
		config.Koolo.OBS.SceneName = sceneName
		changed = true
	}
	if config.Koolo.OBS.SceneUUID == "" {
		config.Koolo.OBS.SceneUUID = generateUUID()
		changed = true
	}
	if config.Koolo.OBS.SourceUUID == "" {
		config.Koolo.OBS.SourceUUID = generateUUID()
		changed = true
	}
	if changed {
		config.SaveKooloConfig(config.Koolo)
	}
	return profileName, sceneName
}

func getReplayPath() string {
	if config.Koolo.OBS.ReplayPath != "" {
		return config.Koolo.OBS.ReplayPath
	}
	return filepath.Join(os.Getenv("USERPROFILE"), "Videos", "Replays")
}

func ensureOBSProfile(logger *slog.Logger, profileName, sceneName string) error {
	appData := getOBSAppDataPath()
	profileDir := filepath.Join(appData, "basic", "profiles", profileName)
	scenesDir := filepath.Join(appData, "basic", "scenes")
	sceneFile := filepath.Join(scenesDir, sceneName+".json")
	profileIni := filepath.Join(profileDir, "basic.ini")

	if _, err := os.Stat(profileIni); err == nil {
		return nil
	}

	logger.Info("Setting up OBS profile for the first time", slog.String("profile", profileName))

	if err := os.MkdirAll(profileDir, 0755); err != nil {
		return fmt.Errorf("failed to create OBS profile dir: %w", err)
	}
	if err := os.MkdirAll(scenesDir, 0755); err != nil {
		return fmt.Errorf("failed to create OBS scenes dir: %w", err)
	}

	videosSavePath := getReplayPath()
	if err := os.MkdirAll(videosSavePath, 0755); err != nil {
		logger.Warn("Could not create Replays folder", slog.Any("error", err))
	}
	obsVideosSavePath := strings.ReplaceAll(videosSavePath, `\`, `\\`)

	encoder := "obs_x264"
	streamEncoder := "obs_x264"
	if isNVENCAvailable() {
		encoder = "nvenc"
		streamEncoder = "nvenc"
		logger.Info("OBS: NVENC detected, using hardware encoding")
	} else {
		logger.Info("OBS: NVENC not detected, using software encoding (x264)")
	}

	ini := fmt.Sprintf(`[General]
Name=%s

[Output]
Mode=Simple
FilenameFormatting=%%CCYY-%%MM-%%DD %%hh-%%mm-%%ss
DelayEnable=false

[SimpleOutput]
FilePath=%s
RecFormat2=hybrid_mp4
VBitrate=3000
ABitrate=160
UseAdvanced=false
Preset=veryfast
NVENCPreset2=p5
RecQuality=Stream
RecRB=true
RecRBTime=26
RecRBSize=512
RecRBPrefix=Replay
StreamAudioEncoder=aac
RecAudioEncoder=aac
RecTracks=1
StreamEncoder=%s
RecEncoder=%s

[Video]
BaseCX=1920
BaseCY=1080
OutputCX=640
OutputCY=360
FPSType=0
FPSCommon=30
FPSInt=30
FPSNum=30
FPSDen=1
ScaleType=bicubic
ColorFormat=NV12
ColorSpace=709
ColorRange=Partial

[Audio]
SampleRate=48000
ChannelSetup=Stereo
`, profileName, obsVideosSavePath, streamEncoder, encoder)

	if err := os.WriteFile(profileIni, []byte(ini), 0644); err != nil {
		return fmt.Errorf("failed to write OBS profile ini: %w", err)
	}

	scenes := buildScenesJSON(sceneName)
	scenesData, err := json.MarshalIndent(scenes, "", "    ")
	if err != nil {
		return fmt.Errorf("failed to marshal scenes JSON: %w", err)
	}
	if err := os.WriteFile(sceneFile, scenesData, 0644); err != nil {
		return fmt.Errorf("failed to write OBS scenes JSON: %w", err)
	}

	logger.Info("OBS profile created", slog.String("profile", profileDir), slog.String("videos", videosSavePath))
	return nil
}

func buildScenesJSON(sceneName string) map[string]interface{} {
	return map[string]interface{}{
		"current_scene":         "Scene",
		"current_program_scene": "Scene",
		"scene_order":           []map[string]interface{}{{"name": "Scene"}},
		"name":                  sceneName,
		"sources": []map[string]interface{}{
			{
				"name":         "Scene",
				"uuid":         config.Koolo.OBS.SceneUUID,
				"id":           "scene",
				"versioned_id": "scene",
				"settings": map[string]interface{}{
					"id_counter":  1,
					"custom_size": false,
					"items": []map[string]interface{}{
						{
							"name":        "D2R",
							"source_uuid": config.Koolo.OBS.SourceUUID,
							"visible":     true,
							"locked":      false,
							"rot":         0.0,
							"align":       5,
							"id":          1,
							"pos":         map[string]float64{"x": 0.0, "y": 0.0},
							"scale":       map[string]float64{"x": 1.0, "y": 1.0},
							"bounds_type": 2,
							"bounds":      map[string]float64{"x": 1920.0, "y": 1080.0},
						},
					},
				},
				"mixers":  0,
				"enabled": true,
				"muted":   false,
			},
			{
				"name":         "D2R",
				"uuid":         config.Koolo.OBS.SourceUUID,
				"id":           "window_capture",
				"versioned_id": "window_capture",
				"settings": map[string]interface{}{
					"window":        ":OsWindow:D2R.exe",
					"method":        2,
					"priority":      2,
					"cursor":        false,
					"compatibility": false,
				},
				"mixers":  0,
				"enabled": true,
				"muted":   true,
			},
		},
		"groups":              []interface{}{},
		"transitions":         []interface{}{},
		"current_transition":  "Fade",
		"transition_duration": 300,
		"version":             2,
	}
}

func isNVENCAvailable() bool {
	_, err := exec.LookPath("nvidia-smi")
	return err == nil
}

func launchOBS(obsPath, profileName, sceneName string) {
	obsProcess = exec.Command(obsPath,
		"--minimize-to-tray",
		"--startreplaybuffer",
		"--disable-updater",
		"--profile", profileName,
		"--collection", sceneName,
	)
	obsProcess.Dir = filepath.Dir(obsPath)
	obsProcess.Start()
}

// StartOBS launches OBS with replay buffer enabled. OBS must be installed in
// the obs-studio folder inside the koolo directory. On first run, a profile and
// scene collection are created automatically. The OBS WebSocket server is
// configured on the port specified in koolo.yaml (default 4455).
func StartOBS(logger *slog.Logger) {
    if !config.Koolo.OBS.Enabled {
        return
    }
	obsPath, err := FindOBSPath()
	if err != nil {
		logger.Warn("OBS not found - place obs-studio folder inside your build folder to enable replay recording", slog.Any("error", err))
		return
	}

	profileName, sceneName := getOrGenerateNames()

	if err := ensureOBSProfile(logger, profileName, sceneName); err != nil {
		logger.Warn("Failed to set up OBS profile", slog.Any("error", err))
		return
	}

	// Write WebSocket config before launching so OBS picks it up on start.
	// This sets auth_required=false and server_enabled=true.
	writeWebSocketConfig()

	if isOBSReady() {
		logger.Info("OBS already running and WebSocket ready")
		return
	}

	// Kill any existing OBS instance that may have stale or auth-locked config
	killOBS()
	// Write again after kill — the previous instance may have overwritten config on exit
	writeWebSocketConfig()

	launchOBS(obsPath, profileName, sceneName)

	// OBS rewrites its WebSocket plugin config on first launch, resetting
	// auth_required=true and server_enabled=false. We overwrite it again after
	// startup to ensure our settings are preserved.
	time.Sleep(3 * time.Second)
	writeWebSocketConfig()

	for i := 0; i < 20; i++ {
		time.Sleep(time.Second)
		if isOBSReady() {
			logger.Info("OBS started and WebSocket ready")
			return
		}
	}
	logger.Warn("OBS started but WebSocket not responding after 20s")
}

func isOBSReady() bool {
	url := fmt.Sprintf("ws://localhost:%d", config.Koolo.OBS.WebSocketPort)
	conn, _, err := websocket.DefaultDialer.Dial(url, nil)
	if err != nil {
		return false
	}
	conn.Close()
	return true
}

// SaveReplay triggers OBS to save the current replay buffer and returns the
// path of the saved file. Returns an empty string if the save fails.
func SaveReplay(logger *slog.Logger, supervisor string) string {
	url := fmt.Sprintf("ws://localhost:%d", config.Koolo.OBS.WebSocketPort)
	conn, _, err := websocket.DefaultDialer.Dial(url, nil)
	if err != nil {
		logger.Warn("OBS WebSocket unavailable", slog.Any("error", err))
		return ""
	}
	defer conn.Close()

	_, _, err = conn.ReadMessage()
	if err != nil {
		logger.Warn("OBS WebSocket handshake failed", slog.Any("error", err))
		return ""
	}

	if err := conn.WriteJSON(map[string]interface{}{
		"op": 1,
		"d":  map[string]interface{}{"rpcVersion": 1},
	}); err != nil {
		logger.Warn("OBS identify failed", slog.Any("error", err))
		return ""
	}

	_, _, err = conn.ReadMessage()
	if err != nil {
		logger.Warn("OBS identified read failed", slog.Any("error", err))
		return ""
	}

	if err := conn.WriteJSON(map[string]interface{}{
		"op": 6,
		"d": map[string]interface{}{
			"requestType": "SaveReplayBuffer",
			"requestId":   "save",
			"requestData": map[string]interface{}{},
		},
	}); err != nil {
		logger.Warn("OBS SaveReplayBuffer failed", slog.Any("error", err))
		return ""
	}

	var response map[string]interface{}
	conn.ReadJSON(&response)
	logger.Info("OBS replay saved", slog.String("supervisor", supervisor))

	// Wait for file to finish writing before returning the path
	time.Sleep(1500 * time.Millisecond)
	return findNewestReplay(logger)
}

func findNewestReplay(logger *slog.Logger) string {
	replayDir := getReplayPath()
	entries, err := os.ReadDir(replayDir)
	if err != nil {
		logger.Warn("OBS replay dir not found", slog.Any("error", err))
		return ""
	}
	var newestName string
	var newestTime time.Time
	for _, e := range entries {
		info, err := e.Info()
		if err != nil {
			continue
		}
		if info.ModTime().After(newestTime) {
			newestTime = info.ModTime()
			newestName = e.Name()
		}
	}
	if newestName == "" {
		return ""
	}
	return filepath.Join(replayDir, newestName)
}


// StopOBS gracefully shuts down the OBS process that was launched by koolo.
// A graceful shutdown allows OBS to write its sentinel file, preventing the
// "OBS did not shut down properly" safe mode dialog on next launch.
func StopOBS(logger *slog.Logger) {
	if !config.Koolo.OBS.Enabled {
		return
	}
	if obsProcess == nil || obsProcess.Process == nil {
		return
	}
	logger.Info("Stopping OBS")
	// Use graceful shutdown without /F to allow OBS to clean up properly
	cmd := exec.Command("taskkill", "/IM", "obs64.exe")
	cmd.Run()
	// Give OBS time to write its shutdown sentinel before koolo exits
	time.Sleep(2 * time.Second)
}