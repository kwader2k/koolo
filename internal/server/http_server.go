package server

import (
	"bytes"
	"cmp"
	"context"
	"embed"
	"encoding/json"
	"errors"
	"fmt"
	"html/template"
	"io/fs"
	"log/slog"
	"math"
	"net/http"
	"net/url"
	"reflect"
	"slices"
	"sort"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"
	"unsafe"

	"os"
	"os/exec"
	"path/filepath"

	"github.com/gorilla/websocket"
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/bot"
	"github.com/hectorgimenez/koolo/internal/character/paladin"
	"github.com/hectorgimenez/koolo/internal/config"
	ctx "github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/drop"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/remote/droplog"
	terrorzones "github.com/hectorgimenez/koolo/internal/terrorzone"
	"github.com/hectorgimenez/koolo/internal/updater"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/hectorgimenez/koolo/internal/utils/winproc"
	"github.com/lxn/win"
	"golang.org/x/sys/windows"
	"gopkg.in/yaml.v3"
)

type HttpServer struct {
	logger              *slog.Logger
	server              *http.Server
	manager             *bot.SupervisorManager
	scheduler           *bot.Scheduler
	templates           *template.Template
	wsServer            *WebSocketServer
	pickitAPI           *PickitAPI
	sequenceAPI         *SequenceAPI
	updater             *updater.Updater
	DropHistory         []DropHistoryEntry
	RunewordHistory     []RunewordHistoryEntry
	DropFilters         map[string]drop.Filters
	DropCardInfo        map[string]dropCardInfo
	DropMux             sync.Mutex
	RunewordMux         sync.Mutex
	autoStartPromptOnce sync.Once
}

var (
	//go:embed all:assets
	assetsFS embed.FS
	//go:embed all:templates
	templatesFS embed.FS

	upgrader = websocket.Upgrader{
		CheckOrigin: func(r *http.Request) bool {
			return true
		},
	}
)

type Client struct {
	conn *websocket.Conn
	send chan []byte
}

type WebSocketServer struct {
	clients    map[*Client]bool
	broadcast  chan []byte
	register   chan *Client
	unregister chan *Client
}

func NewWebSocketServer() *WebSocketServer {
	return &WebSocketServer{
		clients:    make(map[*Client]bool),
		broadcast:  make(chan []byte),
		register:   make(chan *Client),
		unregister: make(chan *Client),
	}
}

type Process struct {
	WindowTitle string `json:"windowTitle"`
	ProcessName string `json:"processName"`
	PID         uint32 `json:"pid"`
}

type dropCardInfo struct {
	ID   int
	Name string
}

func (s *WebSocketServer) Run() {
	for {
		select {
		case client := <-s.register:
			s.clients[client] = true
		case client := <-s.unregister:
			if _, ok := s.clients[client]; ok {
				delete(s.clients, client)
				close(client.send)
			}
		case message := <-s.broadcast:
			for client := range s.clients {
				select {
				case client.send <- message:
				default:
					close(client.send)
					delete(s.clients, client)
				}
			}
		}
	}
}

func (s *WebSocketServer) HandleWebSocket(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		slog.Error("Failed to upgrade connection to WebSocket", "error", err)
		return
	}

	client := &Client{conn: conn, send: make(chan []byte, 256)}
	s.register <- client

	go s.writePump(client)
	go s.readPump(client)
}

func (s *WebSocketServer) writePump(client *Client) {
	defer func() {
		client.conn.Close()
	}()

	for {
		select {
		case message, ok := <-client.send:
			if !ok {
				client.conn.WriteMessage(websocket.CloseMessage, []byte{})
				return
			}

			w, err := client.conn.NextWriter(websocket.TextMessage)
			if err != nil {
				return
			}
			w.Write(message)

			if err := w.Close(); err != nil {
				return
			}
		}
	}
}

func (s *WebSocketServer) readPump(client *Client) {
	defer func() {
		s.unregister <- client
		client.conn.Close()
	}()

	for {
		_, _, err := client.conn.ReadMessage()
		if err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				slog.Error("WebSocket read error", "error", err)
			}
			break
		}
	}
}

func (s *HttpServer) BroadcastStatus() {
	for {
		data := s.getStatusData()
		jsonData, err := json.Marshal(data)
		if err != nil {
			slog.Error("Failed to marshal status data", "error", err)
			continue
		}

		s.wsServer.broadcast <- jsonData
		time.Sleep(1 * time.Second)
	}
}

func New(logger *slog.Logger, manager *bot.SupervisorManager, scheduler *bot.Scheduler) (*HttpServer, error) {
	var templates *template.Template
	helperFuncs := template.FuncMap{
		"isInSlice": func(slice []stat.Resist, value string) bool {
			return slices.Contains(slice, stat.Resist(value))
		},
		"isTZSelected": func(slice []area.ID, value int) bool {
			return slices.Contains(slice, area.ID(value))
		},
		"executeTemplateByName": func(name string, data interface{}) template.HTML {
			tmpl := templates.Lookup(name)
			var buf bytes.Buffer
			if tmpl == nil {
				return "This run is not configurable."
			}

			tmpl.Execute(&buf, data)
			return template.HTML(buf.String())
		},
		"runDisplayName": func(run string) string {
			switch run {
			case string(config.OrgansRun):
				return "Uber (Organs)"
			case string(config.PandemoniumRun):
				return "Uber (Torch)"
			default:
				return run
			}
		},
		"qualityClass": qualityClass,
		"statIDToText": statIDToText,
		"contains":     containss,
		"seq": func(start, end int) []int {
			var result []int
			for i := start; i <= end; i++ {
				result = append(result, i)
			}
			return result
		},
		"allImmunities": func() []string {
			return []string{"f", "c", "l", "p", "ph", "m"}
		},
		"upper": strings.ToUpper,
		"trim":  strings.TrimSpace,
		"isLevelingBuild": func(build string) bool {
			if strings.HasSuffix(build, "_leveling") {
				return true
			}
			switch build {
			case "necromancer", "assassin", "barb_leveling":
				return true
			default:
				return false
			}
		},
		"toJSON": func(v interface{}) template.JS {
			b, err := json.Marshal(v)
			if err != nil {
				return template.JS("{}")
			}
			return template.JS(b)
		},
		// Armory template helpers
		"iterate": func(count int) []int {
			result := make([]int, count)
			for i := range result {
				result[i] = i
			}
			return result
		},
		"mul": func(a, b int) int {
			return a * b
		},
		"json": func(v interface{}) template.JS {
			b, err := json.Marshal(v)
			if err != nil {
				return template.JS("{}")
			}
			return template.JS(b)
		},
		"lower": strings.ToLower,
	}
	templates, err := template.New("").Funcs(helperFuncs).ParseFS(templatesFS, "templates/*.gohtml")
	if err != nil {
		return nil, err
	}

	// Debug: List all loaded templates
	logger.Info("Loaded templates:")
	for _, t := range templates.Templates() {
		logger.Info("  - " + t.Name())
	}

	server := &HttpServer{
		logger:       logger,
		manager:      manager,
		scheduler:    scheduler,
		templates:    templates,
		pickitAPI:    NewPickitAPI(),
		sequenceAPI:  NewSequenceAPI(logger),
		updater:      updater.NewUpdater(logger),
		DropFilters:  make(map[string]drop.Filters),
		DropCardInfo: make(map[string]dropCardInfo),
	}

	server.updater.SetPreRestartCallback(func() error {
		server.logger.Info("Stopping HTTP server before restart")
		return server.Stop()
	})

	server.initDropCallbacks()
	return server, nil
}

func (s *HttpServer) getProcessList(w http.ResponseWriter, r *http.Request) {
	processes, err := getRunningProcesses()
	if err != nil {
		http.Error(w, "Failed to get process list", http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(processes)
}

func (s *HttpServer) attachProcess(w http.ResponseWriter, r *http.Request) {
	characterName := r.URL.Query().Get("characterName")
	pidStr := r.URL.Query().Get("pid")

	pid, err := strconv.ParseUint(pidStr, 10, 32)
	if err != nil {
		s.logger.Error("Invalid PID", "error", err)
		return
	}

	// Find the main window handle (HWND) for the process
	var hwnd win.HWND
	enumWindowsCallback := func(h win.HWND, param uintptr) uintptr {
		var processID uint32
		win.GetWindowThreadProcessId(h, &processID)
		if processID == uint32(pid) {
			hwnd = h
			return 0 // Stop enumeration
		}
		return 1 // Continue enumeration
	}

	windows.EnumWindows(syscall.NewCallback(enumWindowsCallback), nil)

	if hwnd == 0 {
		s.logger.Error("Failed to find window handle for process", "pid", pid)
		return
	}

	// Call manager.Start with the correct arguments, including the HWND
	go s.manager.Start(characterName, true, false, uint32(pid), uint32(hwnd))

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]bool{"success": true})
}

// Add this helper function
func getRunningProcesses() ([]Process, error) {
	var processes []Process

	snapshot, err := windows.CreateToolhelp32Snapshot(windows.TH32CS_SNAPPROCESS, 0)
	if err != nil {
		return nil, err
	}
	defer windows.CloseHandle(snapshot)

	var entry windows.ProcessEntry32
	entry.Size = uint32(unsafe.Sizeof(entry))

	err = windows.Process32First(snapshot, &entry)
	if err != nil {
		return nil, err
	}

	for {
		windowTitle, _ := getWindowTitle(entry.ProcessID)

		if strings.ToLower(syscall.UTF16ToString(entry.ExeFile[:])) == "d2r.exe" {
			processes = append(processes, Process{
				WindowTitle: windowTitle,
				ProcessName: syscall.UTF16ToString(entry.ExeFile[:]),
				PID:         entry.ProcessID,
			})
		}

		err = windows.Process32Next(snapshot, &entry)
		if err != nil {
			if err == windows.ERROR_NO_MORE_FILES {
				break
			}
			return nil, err
		}
	}

	return processes, nil
}

func getWindowTitle(pid uint32) (string, error) {
	var windowTitle string
	var hwnd windows.HWND

	cb := syscall.NewCallback(func(h win.HWND, param uintptr) uintptr {
		var currentPID uint32
		_ = win.GetWindowThreadProcessId(h, &currentPID)

		if currentPID == pid {
			hwnd = windows.HWND(h)
			return 0 // stop enumeration
		}
		return 1 // continue enumeration
	})

	// Enumerate all windows
	windows.EnumWindows(cb, nil)

	if hwnd == 0 {
		return "", fmt.Errorf("no window found for process ID %d", pid)
	}

	// Get window title
	var title [256]uint16
	_, _, _ = winproc.GetWindowText.Call(
		uintptr(hwnd),
		uintptr(unsafe.Pointer(&title[0])),
		uintptr(len(title)),
	)

	windowTitle = syscall.UTF16ToString(title[:])
	return windowTitle, nil

}

func qualityClass(quality string) string {
	switch quality {
	case "LowQuality":
		return "low-quality"
	case "Normal":
		return "normal-quality"
	case "Superior":
		return "superior-quality"
	case "Magic":
		return "magic-quality"
	case "Set":
		return "set-quality"
	case "Rare":
		return "rare-quality"
	case "Unique":
		return "unique-quality"
	case "Crafted":
		return "crafted-quality"
	default:
		return "unknown-quality"
	}
}

func statIDToText(id stat.ID) string {
	return stat.StringStats[id]
}

func containss(slice []string, item string) bool {
	for _, v := range slice {
		if v == item {
			return true
		}
	}
	return false
}

func formatCommitDate(t time.Time) string {
	if t.IsZero() {
		return "unknown"
	}
	return t.Format("2006-01-02 15:04:05")
}

func resolveSkillClassFromBuild(build string) string {
	switch build {
	case "amazon_leveling", "javazon":
		return "ama"
	case "sorceress", "nova", "hydraorb", "lightsorc", "fireballsorc", "sorceress_leveling":
		return "sor"
	case "necromancer":
		return "nec"
	case "paladin_leveling", "paladin_default", "paladin_dragon":
		return "pal"
	case "barb_leveling", "berserker", "warcry_barb":
		return "bar"
	case "druid_leveling", "winddruid":
		return "dru"
	case "assassin", "trapsin", "mosaic":
		return "ass"
	default:
		return ""
	}
}

func buildSkillOptionsForBuild(build string) []SkillOption {
	classKey := resolveSkillClassFromBuild(build)
	options := make([]SkillOption, 0)
	for id, sk := range skill.Skills {
		if sk.Class == "" {
			continue
		}
		if classKey != "" && sk.Class != classKey {
			continue
		}
		key := skill.SkillNames[id]
		name := sk.Name
		if name == "" {
			name = key
		}
		options = append(options, SkillOption{Key: key, Name: name})
	}
	sort.Slice(options, func(i, j int) bool {
		return options[i].Name < options[j].Name
	})
	return options
}

func buildSkillPrereqsForBuild(build string) map[string][]string {
	classKey := resolveSkillClassFromBuild(build)
	nameToKey := make(map[string]string)
	for id, sk := range skill.Skills {
		key := skill.SkillNames[id]
		if key == "" {
			continue
		}
		nameToKey[strings.ToLower(key)] = key
		if sk.Name != "" {
			nameToKey[strings.ToLower(sk.Name)] = key
		}
	}

	prereqs := make(map[string][]string)
	for id, sk := range skill.Skills {
		if sk.Class == "" {
			continue
		}
		if classKey != "" && sk.Class != classKey {
			continue
		}
		key := skill.SkillNames[id]
		if key == "" {
			continue
		}
		reqs := make([]string, 0, 2)
		for _, reqName := range []string{sk.ReqSkill1, sk.ReqSkill2} {
			if reqName == "" {
				continue
			}
			if reqKey, ok := nameToKey[strings.ToLower(reqName)]; ok {
				reqs = append(reqs, reqKey)
			}
		}
		if len(reqs) > 0 {
			prereqs[key] = reqs
		}
	}

	return prereqs
}

func (s *HttpServer) updateAutoStatSkillFromForm(values url.Values, cfg *config.CharacterCfg) {
	oldRespec := cfg.Character.AutoStatSkill.Respec

	cfg.Character.AutoStatSkill.Enabled = values.Has("autoStatSkillEnabled")
	cfg.Character.AutoStatSkill.ExcludeQuestStats = values.Has("autoStatSkillExcludeQuestStats")
	cfg.Character.AutoStatSkill.ExcludeQuestSkills = values.Has("autoStatSkillExcludeQuestSkills")

	statKeys := values["autoStatSkillStat[]"]
	statTargets := values["autoStatSkillStatTarget[]"]
	stats := make([]config.AutoStatSkillStat, 0, len(statKeys))
	for i, statKey := range statKeys {
		if i >= len(statTargets) {
			break
		}
		statKey = strings.TrimSpace(statKey)
		if statKey == "" {
			continue
		}
		target, err := strconv.Atoi(strings.TrimSpace(statTargets[i]))
		if err != nil || target <= 0 {
			continue
		}
		stats = append(stats, config.AutoStatSkillStat{Stat: statKey, Target: target})
	}
	cfg.Character.AutoStatSkill.Stats = stats

	skillKeys := values["autoStatSkillSkill[]"]
	skillTargets := values["autoStatSkillSkillTarget[]"]
	skills := make([]config.AutoStatSkillSkill, 0, len(skillKeys))
	for i, skillKey := range skillKeys {
		if i >= len(skillTargets) {
			break
		}
		skillKey = strings.TrimSpace(skillKey)
		if skillKey == "" {
			continue
		}
		target, err := strconv.Atoi(strings.TrimSpace(skillTargets[i]))
		if err != nil || target <= 0 {
			continue
		}
		skills = append(skills, config.AutoStatSkillSkill{Skill: skillKey, Target: target})
	}
	cfg.Character.AutoStatSkill.Skills = skills

	respecEnabled := values.Has("autoRespecEnabled") && cfg.Character.AutoStatSkill.Enabled
	targetLevel := 0
	if raw := strings.TrimSpace(values.Get("autoRespecTargetLevel")); raw != "" {
		if n, err := strconv.Atoi(raw); err == nil {
			if n < 0 {
				n = 0
			} else if n > 0 && n < 2 {
				n = 2
			} else if n > 99 {
				n = 99
			}
			targetLevel = n
		}
	}
	cfg.Character.AutoStatSkill.Respec.Enabled = respecEnabled
	cfg.Character.AutoStatSkill.Respec.TokenFirst = values.Has("autoRespecTokenFirst") && respecEnabled
	cfg.Character.AutoStatSkill.Respec.TargetLevel = targetLevel

	if !respecEnabled {
		cfg.Character.AutoStatSkill.Respec.Applied = false
	} else if !oldRespec.Enabled || oldRespec.TargetLevel != targetLevel {
		cfg.Character.AutoStatSkill.Respec.Applied = false
	}
}

func (s *HttpServer) initialData(w http.ResponseWriter, r *http.Request) {
	data := s.getStatusData()

	skipPrompt := r.URL.Query().Get("skipAutoStartPrompt") == "true"

	// Decide whether to show the auto-start confirmation prompt.
	// This should only happen once per program run, on the first
	// dashboard load where global auto-start is enabled and at
	// least one character is marked for auto-start.
	showPrompt := false
	if !skipPrompt && data.GlobalAutoStartEnabled {
		s.autoStartPromptOnce.Do(func() {
			for _, enabled := range data.AutoStart {
				if enabled {
					showPrompt = true
					break
				}
			}
		})
	}
	data.ShowAutoStartPrompt = showPrompt

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(data)
}

func (s *HttpServer) getStatusData() IndexData {
	status := make(map[string]bot.Stats)
	drops := make(map[string]int)
	autoStart := make(map[string]bool)

	for _, supervisorName := range s.manager.AvailableSupervisors() {
		stats := s.manager.Status(supervisorName)

		// Enrich with lightweight live character overview for UI
		if data := s.manager.GetData(supervisorName); data != nil {
			// Defaults
			var lvl, life, maxLife, mana, maxMana, mf, gold, gf int
			var exp, lastExp, nextExp uint64
			var fr, cr, lr, pr int
			var mfr, mcr, mlr, mpr int

			if v, ok := data.PlayerUnit.FindStat(stat.Level, 0); ok {
				lvl = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.Experience, 0); ok {
				// Treat as unsigned to handle values > 2^31-1
				exp = uint64(uint32(v.Value))
			}
			if v, ok := data.PlayerUnit.FindStat(stat.LastExp, 0); ok {
				// Treat as unsigned to handle values > 2^31-1
				lastExp = uint64(uint32(v.Value))
			}
			if v, ok := data.PlayerUnit.FindStat(stat.NextExp, 0); ok {
				// Treat as unsigned to handle values > 2^31-1
				nextExp = uint64(uint32(v.Value))
			}
			if v, ok := data.PlayerUnit.FindStat(stat.Life, 0); ok {
				life = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.MaxLife, 0); ok {
				maxLife = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.Mana, 0); ok {
				mana = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.MaxMana, 0); ok {
				maxMana = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.MagicFind, 0); ok {
				mf = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.GoldFind, 0); ok {
				gf = v.Value
			}

			gold = data.PlayerUnit.TotalPlayerGold()

			if v, ok := data.PlayerUnit.FindStat(stat.FireResist, 0); ok {
				fr = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.ColdResist, 0); ok {
				cr = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.LightningResist, 0); ok {
				lr = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.PoisonResist, 0); ok {
				pr = v.Value
			}
			// Max resists (increase cap)
			if v, ok := data.PlayerUnit.FindStat(stat.MaxFireResist, 0); ok {
				mfr = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.MaxColdResist, 0); ok {
				mcr = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.MaxLightningResist, 0); ok {
				mlr = v.Value
			}
			if v, ok := data.PlayerUnit.FindStat(stat.MaxPoisonResist, 0); ok {
				mpr = v.Value
			}

			// Apply difficulty penalty and cap to compute current/effective resists
			penalty := 0
			switch data.CharacterCfg.Game.Difficulty {
			case difficulty.Nightmare:
				penalty = 40
			case difficulty.Hell:
				penalty = 100
			}
			capFR := 75 + mfr
			capCR := 75 + mcr
			capLR := 75 + mlr
			capPR := 75 + mpr
			if fr-penalty > capFR {
				fr = capFR
			} else {
				fr = fr - penalty
			}
			if cr-penalty > capCR {
				cr = capCR
			} else {
				cr = cr - penalty
			}
			if lr-penalty > capLR {
				lr = capLR
			} else {
				lr = lr - penalty
			}
			if pr-penalty > capPR {
				pr = capPR
			} else {
				pr = pr - penalty
			}

			// Resolve difficulty and area names
			diffStr := fmt.Sprint(data.CharacterCfg.Game.Difficulty)
			areaStr := ""
			// Prefer human-readable area name if available
			if lvl := data.PlayerUnit.Area.Area(); lvl.Name != "" {
				areaStr = lvl.Name
			} else {
				areaStr = fmt.Sprint(data.PlayerUnit.Area)
			}

			stats.UI = bot.CharacterOverview{
				Class:           data.CharacterCfg.Character.Class,
				Level:           lvl,
				Experience:      exp,
				LastExp:         lastExp,
				NextExp:         nextExp,
				Difficulty:      diffStr,
				Area:            areaStr,
				Ping:            data.Game.Ping,
				Life:            life,
				MaxLife:         maxLife,
				Mana:            mana,
				MaxMana:         maxMana,
				MagicFind:       mf,
				Gold:            gold,
				GoldFind:        gf,
				FireResist:      fr,
				ColdResist:      cr,
				LightningResist: lr,
				PoisonResist:    pr,
			}
		}

		// Check if this is a companion follower & ensure we always expose class
		cfg, found := config.GetCharacter(supervisorName)
		if found {
			if stats.UI.Class == "" {
				stats.UI.Class = cfg.Character.Class
			}
			// Add companion information to the stats
			if cfg.Companion.Enabled && !cfg.Companion.Leader {
				// This is a companion follower
				stats.IsCompanionFollower = true
				stats.MuleEnabled = cfg.Muling.Enabled
			}

			// Per-character Auto Start flag
			autoStart[supervisorName] = cfg.AutoStart
		}

		status[supervisorName] = stats

		if s.manager.GetSupervisorStats(supervisorName).Drops != nil {
			drops[supervisorName] = len(s.manager.GetSupervisorStats(supervisorName).Drops)
		} else {
			drops[supervisorName] = 0
		}
	}

	// Collect scheduler status for each supervisor
	schedulerStatus := make(map[string]*SchedulerStatusInfo)
	if s.scheduler != nil {
		for _, supervisorName := range s.manager.AvailableSupervisors() {
			cfg := config.GetCharacters()[supervisorName]
			if cfg == nil {
				continue
			}

			info := &SchedulerStatusInfo{
				Enabled: cfg.Scheduler.Enabled,
				Mode:    cfg.Scheduler.Mode,
			}

			// For duration mode, get live state from scheduler
			if cfg.Scheduler.Mode == "duration" && cfg.Scheduler.Enabled {
				state := s.scheduler.GetDurationState(supervisorName)
				if state != nil {
					info.Phase = string(state.CurrentPhase)
					info.PhaseStartTime = state.PhaseStartTime.Format(time.RFC3339)
					info.PhaseEndTime = state.PhaseEndTime.Format(time.RFC3339)
					info.TodayWakeTime = state.TodayWakeTime.Format(time.RFC3339)
					info.TodayRestTime = state.TodayRestTime.Format(time.RFC3339)
					info.PlayedMinutes = state.PlayedMinutes

					// Get next 3 breaks
					nextBreaks := []SchedulerBreak{}
					now := time.Now()
					for i := state.CurrentBreakIdx; i < len(state.ScheduledBreaks) && len(nextBreaks) < 3; i++ {
						brk := state.ScheduledBreaks[i]
						if brk.StartTime.After(now) {
							nextBreaks = append(nextBreaks, SchedulerBreak{
								Type:      brk.Type,
								StartTime: brk.StartTime.Format(time.RFC3339),
								Duration:  brk.Duration,
							})
						}
					}
					info.NextBreaks = nextBreaks
				}
			}

			schedulerStatus[supervisorName] = info
		}
	}

	return IndexData{
		Version:                     config.Version,
		Status:                      status,
		DropCount:                   drops,
		AutoStart:                   autoStart,
		SchedulerStatus:             schedulerStatus,
		GlobalAutoStartEnabled:      config.Koolo.AutoStart.Enabled,
		GlobalAutoStartDelaySeconds: config.Koolo.AutoStart.DelaySeconds,
	}
}

func (s *HttpServer) Listen(port int) error {
	s.wsServer = NewWebSocketServer()
	go s.wsServer.Run()
	go s.BroadcastStatus()

	http.HandleFunc("/", s.getRoot)
	http.HandleFunc("/config", s.config)
	http.HandleFunc("/supervisorSettings", s.characterSettings)
	http.HandleFunc("/runewords", s.runewordSettings)
	http.HandleFunc("/api/runewords/rolls", s.runewordRolls)
	http.HandleFunc("/api/runewords/base-types", s.runewordBaseTypes)
	http.HandleFunc("/api/runewords/bases", s.runewordBases)
	http.HandleFunc("/api/runewords/history", s.runewordHistory)
	http.HandleFunc("/start", s.startSupervisor)
	http.HandleFunc("/stop", s.stopSupervisor)
	http.HandleFunc("/togglePause", s.togglePause)
	http.HandleFunc("/autostart/toggle", s.toggleAutoStart)
	http.HandleFunc("/autostart/run-once", s.runAutoStartOnce)
	http.HandleFunc("/debug", s.debugHandler)
	http.HandleFunc("/debug-data", s.debugData)
	http.HandleFunc("/drops", s.drops)
	http.HandleFunc("/all-drops", s.allDrops)
	http.HandleFunc("/export-drops", s.exportDrops)
	http.HandleFunc("/open-droplogs", s.openDroplogs)
	http.HandleFunc("/reset-droplogs", s.resetDroplogs)
	http.HandleFunc("/process-list", s.getProcessList)
	http.HandleFunc("/attach-process", s.attachProcess)
	http.HandleFunc("/ws", s.wsServer.HandleWebSocket)                         // Web socket
	http.HandleFunc("/initial-data", s.initialData)                            // Web socket data
	http.HandleFunc("/api/reload-config", s.reloadConfig)                      // New handler
	http.HandleFunc("/api/companion-join", s.companionJoin)                    // Companion join handler
	http.HandleFunc("/api/generate-battlenet-token", s.generateBattleNetToken) // Battle.net token generation
	http.HandleFunc("/reset-muling", s.resetMuling)

	// Updater routes
	http.HandleFunc("/api/updater/version", s.getVersion)
	http.HandleFunc("/api/updater/check", s.checkUpdates)
	http.HandleFunc("/api/updater/current-commits", s.getCurrentCommits)
	http.HandleFunc("/api/updater/update", s.performUpdate)
	http.HandleFunc("/api/updater/status", s.getUpdaterStatus)
	http.HandleFunc("/api/updater/backups", s.getBackups)
	http.HandleFunc("/api/updater/rollback", s.performRollback)
	http.HandleFunc("/api/updater/prs", s.getUpstreamPRs)
	http.HandleFunc("/api/updater/cherry-pick", s.cherryPickPRs)
	http.HandleFunc("/api/updater/prs/revert", s.revertPR)

	// Pickit Editor routes
	http.HandleFunc("/pickit-editor", s.pickitEditorPage)
	http.HandleFunc("/sequence-editor", s.sequenceEditorPage)
	http.HandleFunc("/api/pickit/items", s.pickitAPI.handleGetItems)
	http.HandleFunc("/api/pickit/items/search", s.pickitAPI.handleSearchItems)
	http.HandleFunc("/api/pickit/items/categories", s.pickitAPI.handleGetCategories)
	http.HandleFunc("/api/pickit/stats", s.pickitAPI.handleGetStats)
	http.HandleFunc("/api/pickit/templates", s.pickitAPI.handleGetTemplates)
	http.HandleFunc("/api/pickit/presets", s.pickitAPI.handleGetPresets)
	http.HandleFunc("/api/pickit/rules", s.pickitAPI.handleGetRules)
	http.HandleFunc("/api/pickit/rules/create", s.pickitAPI.handleCreateRule)
	http.HandleFunc("/api/pickit/rules/update", s.pickitAPI.handleUpdateRule)
	http.HandleFunc("/api/pickit/rules/delete", s.pickitAPI.handleDeleteRule)
	http.HandleFunc("/api/pickit/rules/validate", s.pickitAPI.handleValidateRule)
	http.HandleFunc("/api/pickit/rules/validate-nip", s.pickitAPI.handleValidateNIPLine)
	http.HandleFunc("/api/pickit/files", s.pickitAPI.handleGetFiles)
	http.HandleFunc("/api/pickit/files/import", s.pickitAPI.handleImportFile)
	http.HandleFunc("/api/pickit/files/export", s.pickitAPI.handleExportFile)
	http.HandleFunc("/api/pickit/files/rules/delete", s.pickitAPI.handleDeleteFileRule)
	http.HandleFunc("/api/pickit/files/rules/update", s.pickitAPI.handleUpdateFileRule)
	http.HandleFunc("/api/pickit/files/rules/append", s.pickitAPI.handleAppendNIPLine)
	http.HandleFunc("/api/pickit/browse-folder", s.pickitAPI.handleBrowseFolder)
	http.HandleFunc("/api/pickit/simulate", s.pickitAPI.handleSimulate)
	http.HandleFunc("/api/sequence-editor/runs", s.sequenceAPI.handleListRuns)
	http.HandleFunc("/api/sequence-editor/file", s.sequenceAPI.handleGetSequence)
	http.HandleFunc("/api/sequence-editor/open", s.sequenceAPI.handleBrowseSequence)
	http.HandleFunc("/api/sequence-editor/save", s.sequenceAPI.handleSaveSequence)
	http.HandleFunc("/api/sequence-editor/delete", s.sequenceAPI.handleDeleteSequence)
	http.HandleFunc("/api/sequence-editor/files", s.sequenceAPI.handleListSequenceFiles)
	http.HandleFunc("/api/skill-options", s.skillOptionsAPI)

	http.HandleFunc("/api/supervisors/bulk-apply", s.bulkApplyCharacterSettings)
	http.HandleFunc("/api/scheduler-history", s.schedulerHistory)
	http.HandleFunc("/Drop-manager", s.DropManagerPage)

	// Armory routes
	http.HandleFunc("/armory", s.armoryPage)
	http.HandleFunc("/api/armory", s.armoryAPI)
	http.HandleFunc("/api/armory/characters", s.armoryCharactersAPI)
	http.HandleFunc("/api/armory/all", s.armoryAllAPI)

	s.registerDropRoutes()

	assets, _ := fs.Sub(assetsFS, "assets")
	http.Handle("/assets/", http.StripPrefix("/assets/", http.FileServer(http.FS(assets))))

	// Serve item images from the filesystem (assets/items folder relative to executable)
	http.Handle("/items/", http.StripPrefix("/items/", http.FileServer(http.Dir("../assets/items"))))

	s.server = &http.Server{
		Addr: fmt.Sprintf(":%d", port),
	}

	if err := s.server.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
		return err
	}

	return nil
}

func (s *HttpServer) reloadConfig(w http.ResponseWriter, r *http.Request) {
	result := s.manager.ReloadConfig()
	if result != nil {
		http.Error(w, result.Error(), http.StatusInternalServerError)
		return
	}

	s.logger.Info("Config reloaded")
	w.WriteHeader(http.StatusOK)
}

// SchedulerHistoryEntry matches bot.HistoryEntry for JSON serialization
type SchedulerHistoryEntry struct {
	Date              string                `json:"date"`
	WakeTime          string                `json:"wakeTime"`
	SleepTime         string                `json:"sleepTime"`
	TotalPlayMinutes  int                   `json:"totalPlayMinutes"`
	TotalBreakMinutes int                   `json:"totalBreakMinutes"`
	Breaks            []SchedulerBreakEntry `json:"breaks"`
}

type SchedulerBreakEntry struct {
	Type      string `json:"type"`
	StartTime string `json:"startTime"`
	Duration  int    `json:"duration"`
}

type SchedulerHistoryResponse struct {
	History []SchedulerHistoryEntry `json:"history"`
}

func (s *HttpServer) schedulerHistory(w http.ResponseWriter, r *http.Request) {
	supervisor := r.URL.Query().Get("supervisor")
	if supervisor == "" {
		http.Error(w, "supervisor parameter required", http.StatusBadRequest)
		return
	}

	// Read history file directly (same path as scheduler uses)
	historyPath := filepath.Join("config", supervisor, "scheduler_history.json")
	data, err := os.ReadFile(historyPath)
	if err != nil {
		// No history yet - return empty array
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(SchedulerHistoryResponse{History: []SchedulerHistoryEntry{}})
		return
	}

	// Parse and return
	var history SchedulerHistoryResponse
	if err := json.Unmarshal(data, &history); err != nil {
		s.logger.Error("Failed to parse scheduler history", "error", err)
		w.Header().Set("Content-Type", "application/json")
		json.NewEncoder(w).Encode(SchedulerHistoryResponse{History: []SchedulerHistoryEntry{}})
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(history)
}

func (s *HttpServer) Stop() error {
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	return s.server.Shutdown(ctx)
}

func (s *HttpServer) getRoot(w http.ResponseWriter, r *http.Request) {
	if !utils.HasAdminPermission() {
		s.templates.ExecuteTemplate(w, "templates/admin_required.gohtml", nil)
		return
	}

	if config.Koolo.FirstRun {
		http.Redirect(w, r, "/config", http.StatusSeeOther)
		return
	}

	s.index(w)
}

func (s *HttpServer) debugData(w http.ResponseWriter, r *http.Request) {
	characterName := r.URL.Query().Get("characterName")
	if characterName == "" {
		http.Error(w, "Character name is required", http.StatusBadRequest)
		return
	}

	type DebugData struct {
		DebugData map[ctx.Priority]*ctx.Debug
		GameData  *game.Data
	}

	context := s.manager.GetContext(characterName)

	debugData := DebugData{
		DebugData: context.ContextDebug,
		GameData:  context.Data,
	}

	jsonData, err := json.Marshal(debugData)
	if err != nil {
		http.Error(w, "Failed to serialize game data", http.StatusInternalServerError)
		return
	}
	w.Header().Set("Content-Type", "application/json")
	w.Write(jsonData)
}

func (s *HttpServer) debugHandler(w http.ResponseWriter, r *http.Request) {
	s.templates.ExecuteTemplate(w, "debug.gohtml", nil)
}

func (s *HttpServer) pickitEditorPage(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/html; charset=utf-8")

	// Try without templates/ prefix first (like debug.gohtml)
	err := s.templates.ExecuteTemplate(w, "pickit_editor.gohtml", nil)
	if err != nil {
		// If that fails, log what templates we have
		s.logger.Error("Failed to execute pickit_editor template", "error", err)
		s.logger.Info("Available templates:")
		for _, t := range s.templates.Templates() {
			s.logger.Info("  - " + t.Name())
		}
		http.Error(w, fmt.Sprintf("Template error: %v", err), http.StatusInternalServerError)
		return
	}
}

func (s *HttpServer) sequenceEditorPage(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/html; charset=utf-8")

	if err := s.templates.ExecuteTemplate(w, "sequence_editor.gohtml", nil); err != nil {
		s.logger.Error("Failed to execute sequence_editor template", "error", err)
		http.Error(w, fmt.Sprintf("Template error: %v", err), http.StatusInternalServerError)
		return
	}
}

func (s *HttpServer) startSupervisor(w http.ResponseWriter, r *http.Request) {
	supervisorList := s.manager.AvailableSupervisors()
	supervisor := r.URL.Query().Get("characterName")
	manualMode := r.URL.Query().Get("manualMode") == "true"

	if supervisor == "" {
		http.Error(w, "missing characterName", http.StatusBadRequest)
		return
	}

	// Get the current auth method for the supervisor we wanna start
	supCfg, currFound := config.GetCharacter(supervisor)
	if !currFound || supCfg == nil {
		http.Error(w, "character configuration not found", http.StatusNotFound)
		return
	}

	if err := s.canStartSupervisor(supervisor, supervisorList, supCfg); err != nil {
		http.Error(w, err.Error(), http.StatusConflict)
		return
	}

	go func(name string, manual bool) {
		if err := s.manager.Start(name, false, manual); err != nil {
			s.logger.Error("Failed to start supervisor", slog.String("supervisor", name), slog.Any("error", err))
		}
	}(supervisor, manualMode)

	s.initialData(w, r)
}

// canStartSupervisor enforces TokenAuth concurrency rules before starting a supervisor.
func (s *HttpServer) canStartSupervisor(target string, supervisorList []string, targetCfg *config.CharacterCfg) error {
	// Prevent launching of other clients while there's a client with TokenAuth still starting
	for _, sup := range supervisorList {
		// Skip the target itself
		if sup == target {
			continue
		}

		if s.manager.GetSupervisorStats(sup).SupervisorStatus == bot.Starting {
			// Prevent launching if we're using token auth & another client is starting (no matter what auth method)
			if targetCfg.AuthMethod == "TokenAuth" {
				return fmt.Errorf("waiting to start %s: another client (%s) is still starting", target, sup)
			}

			// Prevent launching if another client that is using token auth is starting
			sCfg, found := config.GetCharacter(sup)
			if found && sCfg.AuthMethod == "TokenAuth" {
				return fmt.Errorf("waiting to start %s: token-auth client %s is still starting", target, sup)
			}
		}
	}

	return nil
}

func (s *HttpServer) toggleAutoStart(w http.ResponseWriter, r *http.Request) {
	name := r.URL.Query().Get("characterName")
	enabled := r.URL.Query().Get("enabled") == "true"

	if name == "" {
		http.Error(w, "missing characterName", http.StatusBadRequest)
		return
	}

	cfg, found := config.GetCharacter(name)
	if !found || cfg == nil {
		http.Error(w, "character not found", http.StatusNotFound)
		return
	}

	cfg.AutoStart = enabled
	if err := config.SaveSupervisorConfig(name, cfg); err != nil {
		http.Error(w, "failed to save supervisor config", http.StatusInternalServerError)
		return
	}

	s.initialData(w, r)
}

func (s *HttpServer) runAutoStartOnce(w http.ResponseWriter, r *http.Request) {
	if err := s.autoStartOnceInternal(); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	w.WriteHeader(http.StatusOK)
}

// autoStartOnceInternal contains the core logic for starting all supervisors
// marked with AutoStart, using the global delay setting. It is used both by
// the HTTP handler and by application startup.
func (s *HttpServer) autoStartOnceInternal() error {
	supervisorList := s.manager.AvailableSupervisors()
	if len(supervisorList) == 0 {
		return fmt.Errorf("no supervisors available")
	}

	// Collect supervisors marked for AutoStart
	var targets []string
	for _, name := range supervisorList {
		cfg, found := config.GetCharacter(name)
		if !found || cfg == nil {
			continue
		}
		if cfg.AutoStart {
			targets = append(targets, name)
		}
	}

	if len(targets) == 0 {
		return fmt.Errorf("no supervisors marked for auto start")
	}

	// Fallback to a sensible default if not configured
	delaySeconds := config.Koolo.AutoStart.DelaySeconds
	if delaySeconds <= 0 {
		delaySeconds = 60
	}
	delay := time.Duration(delaySeconds) * time.Second

	go func() {
		s.logger.Info("Auto-start sequence begin",
			"characters", targets,
			"delay_seconds", delaySeconds)

		const concurrencyRetryDelay = 5 * time.Second

		for i, name := range targets {
			if i > 0 && delay > 0 {
				s.logger.Info("Waiting before next auto-start",
					"next_name", name,
					"delay", delay)
				time.Sleep(delay)
			}

			cfg, found := config.GetCharacter(name)
			if !found || cfg == nil {
				s.logger.Warn("Skipping auto-start because configuration was not found",
					slog.String("name", name))
				continue
			}

			for {
				if err := s.canStartSupervisor(name, supervisorList, cfg); err != nil {
					s.logger.Info("Auto-start waiting for available slot",
						"name", name,
						"reason", err.Error())
					time.Sleep(concurrencyRetryDelay)
					continue
				}
				break
			}

			s.logger.Info("Auto-starting character",
				"name", name,
				"position", fmt.Sprintf("%d/%d", i+1, len(targets)))

			// Run each supervisor start in its own goroutine so that
			// a long-running Start call for one character does not block
			// the scheduling of subsequent characters.
			go func(supervisorName string) {
				if err := s.manager.Start(supervisorName, false, false); err != nil {
					s.logger.Error("Auto-start failed",
						"name", supervisorName,
						"error", err)
				}
			}(name)
		}

		s.logger.Info("Auto-start sequence completed",
			"total", len(targets))
	}()

	return nil
}

// AutoStartOnStartup triggers a one-off Auto Start sequence if it is enabled
// in the global configuration. This is intended to be called when Koolo starts.
func (s *HttpServer) AutoStartOnStartup() {
	if !config.Koolo.AutoStart.Enabled {
		return
	}

	if err := s.autoStartOnceInternal(); err != nil {
		s.logger.Error("Auto start on startup failed", slog.Any("error", err))
	}
}

func (s *HttpServer) stopSupervisor(w http.ResponseWriter, r *http.Request) {
	s.manager.Stop(r.URL.Query().Get("characterName"))
	s.initialData(w, r)
}

func (s *HttpServer) togglePause(w http.ResponseWriter, r *http.Request) {
	s.manager.TogglePause(r.URL.Query().Get("characterName"))
	s.initialData(w, r)
}

func (s *HttpServer) index(w http.ResponseWriter) {
	status := make(map[string]bot.Stats)
	drops := make(map[string]int)

	for _, supervisorName := range s.manager.AvailableSupervisors() {
		status[supervisorName] = bot.Stats{
			SupervisorStatus: bot.NotStarted,
		}

		status[supervisorName] = s.manager.Status(supervisorName)

		if s.manager.GetSupervisorStats(supervisorName).Drops != nil {
			drops[supervisorName] = len(s.manager.GetSupervisorStats(supervisorName).Drops)
		} else {
			drops[supervisorName] = 0
		}

	}

	s.templates.ExecuteTemplate(w, "index.gohtml", IndexData{
		Version:   config.Version,
		Status:    status,
		DropCount: drops,
	})
}

func (s *HttpServer) drops(w http.ResponseWriter, r *http.Request) {
	sup := r.URL.Query().Get("supervisor")
	cfg, found := config.GetCharacter(sup)
	if !found {
		http.Error(w, "Can't fetch drop data because the configuration "+sup+" wasn't found", http.StatusNotFound)
		return
	}

	var Drops []data.Drop

	if s.manager.GetSupervisorStats(sup).Drops == nil {
		Drops = make([]data.Drop, 0)
	} else {
		Drops = s.manager.GetSupervisorStats(sup).Drops
	}

	s.templates.ExecuteTemplate(w, "drops.gohtml", DropData{
		NumberOfDrops: len(Drops),
		Character:     cfg.CharacterName,
		Drops:         Drops,
	})
}

// allDrops renders a centralized droplog view across all characters.
func (s *HttpServer) allDrops(w http.ResponseWriter, r *http.Request) {
	// Determine droplog directory
	base := config.Koolo.LogSaveDirectory
	if base == "" {
		base = "logs"
	}
	dir := filepath.Join(base, "droplogs")

	records, err := droplog.ReadAll(dir)
	if err != nil {
		s.templates.ExecuteTemplate(w, "all_drops.gohtml", AllDropsData{ErrorMessage: err.Error()})
		return
	}

	// Optional filters via query:
	qSup := strings.TrimSpace(r.URL.Query().Get("supervisor"))
	qChar := strings.TrimSpace(r.URL.Query().Get("character"))
	qText := strings.ToLower(strings.TrimSpace(r.URL.Query().Get("q")))

	var rows []AllDropRecord
	for _, rec := range records {
		if qSup != "" && !strings.EqualFold(qSup, rec.Supervisor) {
			continue
		}
		if qChar != "" && !strings.EqualFold(qChar, rec.Character) {
			continue
		}
		// text filter on name or stats string
		if qText != "" {
			name := rec.Drop.Item.IdentifiedName
			if name == "" {
				name = fmt.Sprint(rec.Drop.Item.Name)
			}
			blob := strings.ToLower(name + " " + strings.Join(statsToStrings(rec.Drop.Item.Stats), " "))
			if !strings.Contains(blob, qText) {
				continue
			}
		}
		rows = append(rows, AllDropRecord{
			Time:       rec.Time.Format("2006-01-02 15:04:05"),
			Supervisor: rec.Supervisor,
			Character:  rec.Character,
			Profile:    rec.Profile,
			Drop:       rec.Drop,
		})
	}

	// Sort newest first
	sort.SliceStable(rows, func(i, j int) bool { return rows[i].Time > rows[j].Time })

	s.templates.ExecuteTemplate(w, "all_drops.gohtml", AllDropsData{
		Total:   len(rows),
		Records: rows,
	})
}

// exportDrops renders a static HTML of the centralized drops and returns it as a file download.
func (s *HttpServer) exportDrops(w http.ResponseWriter, r *http.Request) {
	// Reuse allDrops data generation
	base := config.Koolo.LogSaveDirectory
	if base == "" {
		base = "logs"
	}
	dir := filepath.Join(base, "droplogs")

	records, err := droplog.ReadAll(dir)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	var rows []AllDropRecord
	for _, rec := range records {
		rows = append(rows, AllDropRecord{
			Time:       rec.Time.Format("2006-01-02 15:04:05"),
			Supervisor: rec.Supervisor,
			Character:  rec.Character,
			Profile:    rec.Profile,
			Drop:       rec.Drop,
		})
	}

	var buf bytes.Buffer
	if err := s.templates.ExecuteTemplate(&buf, "all_drops.gohtml", AllDropsData{Total: len(rows), Records: rows}); err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	// Ensure directory exists
	if err := os.MkdirAll(dir, 0o755); err != nil {
		http.Error(w, fmt.Sprintf("failed to create export directory: %v", err), http.StatusInternalServerError)
		return
	}

	// Write to a timestamped HTML file under droplogs
	outName := fmt.Sprintf("all-drops-%s.html", time.Now().Format("2006-01-02-15-04-05"))
	outPath := filepath.Join(dir, outName)
	if err := os.WriteFile(outPath, buf.Bytes(), 0o644); err != nil {
		http.Error(w, fmt.Sprintf("failed to write export: %v", err), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "ok", "file": outPath})
}

// helper: convert stats to strings for filtering
func statsToStrings(stats any) []string {
	v := reflect.ValueOf(stats)
	if v.Kind() != reflect.Slice && v.Kind() != reflect.Array {
		return nil
	}
	out := make([]string, 0, v.Len())
	for i := 0; i < v.Len(); i++ {
		sv := v.Index(i)
		if sv.Kind() == reflect.Pointer {
			sv = sv.Elem()
		}
		if sv.Kind() == reflect.Struct {
			f := sv.FieldByName("String")
			if f.IsValid() && f.Kind() == reflect.String {
				s := f.String()
				if s != "" {
					out = append(out, s)
				}
			}
		}
	}
	return out
}

func validateSchedulerData(cfg *config.CharacterCfg) error {
	for day := 0; day < 7; day++ {

		cfg.Scheduler.Days[day].DayOfWeek = day

		// Sort time ranges
		sort.Slice(cfg.Scheduler.Days[day].TimeRanges, func(i, j int) bool {
			return cfg.Scheduler.Days[day].TimeRanges[i].Start.Before(cfg.Scheduler.Days[day].TimeRanges[j].Start)
		})

		daysOfWeek := []string{"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"}

		// Check for overlapping time ranges
		for i := 0; i < len(cfg.Scheduler.Days[day].TimeRanges); i++ {
			if !cfg.Scheduler.Days[day].TimeRanges[i].End.After(cfg.Scheduler.Days[day].TimeRanges[i].Start) {
				return fmt.Errorf("end time must be after start time for day %s", daysOfWeek[day])
			}

			if i > 0 {
				if !cfg.Scheduler.Days[day].TimeRanges[i].Start.After(cfg.Scheduler.Days[day].TimeRanges[i-1].End) {
					return fmt.Errorf("overlapping time ranges for day %s", daysOfWeek[day])
				}
			}
		}
	}

	return nil
}

func (s *HttpServer) getVersionData() *VersionData {
	versionInfo, _ := updater.GetCurrentVersionNoClone()
	if versionInfo != nil {
		return &VersionData{
			CommitHash: versionInfo.CommitHash,
			CommitDate: formatCommitDate(versionInfo.CommitDate),
			CommitMsg:  versionInfo.CommitMsg,
			Branch:     versionInfo.Branch,
		}
	}
	return nil
}

func (s *HttpServer) config(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodPost {
		err := r.ParseForm()
		if err != nil {
			s.templates.ExecuteTemplate(w, "config.gohtml", ConfigData{
				KooloCfg:       config.Koolo,
				ErrorMessage:   "Error parsing form",
				CurrentVersion: s.getVersionData(),
			})
			return
		}

		newConfig := *config.Koolo
		newConfig.FirstRun = false // Disable the welcome assistant
		newConfig.D2RPath = r.Form.Get("d2rpath")
		newConfig.D2LoDPath = r.Form.Get("d2lodpath")
		newConfig.CentralizedPickitPath = r.Form.Get("centralized_pickit_path")
		newConfig.UseCustomSettings = r.Form.Get("use_custom_settings") == "true"
		newConfig.GameWindowArrangement = r.Form.Get("game_window_arrangement") == "true"
		// Debug
		newConfig.Debug.Log = r.Form.Get("debug_log") == "true"
		newConfig.Debug.Screenshots = r.Form.Get("debug_screenshots") == "true"
		newConfig.Debug.OpenOverlayMapOnGameStart = r.Form.Get("debug_open_overlay_map") == "true"
		// Discord
		newConfig.Discord.Enabled = r.Form.Get("discord_enabled") == "true"
		newConfig.Discord.EnableGameCreatedMessages = r.Form.Has("enable_game_created_messages")
		newConfig.Discord.EnableNewRunMessages = r.Form.Has("enable_new_run_messages")
		newConfig.Discord.EnableRunFinishMessages = r.Form.Has("enable_run_finish_messages")
		newConfig.Discord.EnableDiscordChickenMessages = r.Form.Has("enable_discord_chicken_messages")
		newConfig.Discord.EnableDiscordErrorMessages = r.Form.Has("enable_discord_error_messages")
		newConfig.Discord.DisableItemStashScreenshots = r.Form.Has("discord_disable_item_stash_screenshots")
		newConfig.Discord.IncludePickitInfoInItemText = r.Form.Has("discord_include_pickit_info_in_item_text")
		newConfig.Discord.Token = r.Form.Get("discord_token")
		newConfig.Discord.ChannelID = r.Form.Get("discord_channel_id")
		newConfig.Discord.ItemChannelID = r.Form.Get("discord_item_channel_id")
		newConfig.Discord.UseWebhook = r.Form.Get("discord_use_webhook") == "true"
		newConfig.Discord.WebhookURL = strings.TrimSpace(r.Form.Get("discord_webhook_url"))
		newConfig.Discord.ItemWebhookURL = strings.TrimSpace(r.Form.Get("discord_item_webhook_url"))

		// Discord admins who can use bot commands
		discordAdmins := r.Form.Get("discord_admins")
		cleanedAdmins := strings.Map(func(r rune) rune {
			if (r >= '0' && r <= '9') || r == ',' {
				return r
			}
			return -1
		}, discordAdmins)
		newConfig.Discord.BotAdmins = strings.Split(cleanedAdmins, ",")
		newConfig.Telegram.Enabled = r.Form.Get("telegram_enabled") == "true"
		newConfig.Telegram.Token = r.Form.Get("telegram_token")
		telegramChatId, err := strconv.ParseInt(r.Form.Get("telegram_chat_id"), 10, 64)
		if err != nil {
			s.templates.ExecuteTemplate(w, "config.gohtml", ConfigData{
				KooloCfg:       &newConfig,
				ErrorMessage:   "Invalid Telegram Chat ID",
				CurrentVersion: s.getVersionData(),
			})
			return
		}
		newConfig.Telegram.ChatID = telegramChatId

		newConfig.Ngrok.Enabled = r.Form.Get("ngrok_enabled") == "true"
		newConfig.Ngrok.SendURL = r.Form.Get("ngrok_send_url") == "true"
		newConfig.Ngrok.Authtoken = strings.TrimSpace(r.Form.Get("ngrok_authtoken"))
		newConfig.Ngrok.Region = strings.TrimSpace(r.Form.Get("ngrok_region"))
		newConfig.Ngrok.Domain = strings.TrimSpace(r.Form.Get("ngrok_domain"))
		newConfig.Ngrok.BasicAuthUser = strings.TrimSpace(r.Form.Get("ngrok_basic_auth_user"))
		newConfig.Ngrok.BasicAuthPass = strings.TrimSpace(r.Form.Get("ngrok_basic_auth_pass"))
		if newConfig.Ngrok.BasicAuthUser != "" && newConfig.Ngrok.BasicAuthPass == "" {
			s.templates.ExecuteTemplate(w, "config.gohtml", ConfigData{
				KooloCfg:       &newConfig,
				ErrorMessage:   "ngrok basic auth password is required when a username is set",
				CurrentVersion: s.getVersionData(),
			})
			return
		}
		if newConfig.Ngrok.BasicAuthPass != "" && newConfig.Ngrok.BasicAuthUser == "" {
			s.templates.ExecuteTemplate(w, "config.gohtml", ConfigData{
				KooloCfg:       &newConfig,
				ErrorMessage:   "ngrok basic auth username is required when a password is set",
				CurrentVersion: s.getVersionData(),
			})
			return
		}
		if newConfig.Ngrok.BasicAuthPass != "" && len(newConfig.Ngrok.BasicAuthPass) < 8 {
			s.templates.ExecuteTemplate(w, "config.gohtml", ConfigData{
				KooloCfg:       &newConfig,
				ErrorMessage:   "ngrok basic auth password must be at least 8 characters",
				CurrentVersion: s.getVersionData(),
			})
			return
		}

		// Ping Monitor
		newConfig.PingMonitor.Enabled = r.Form.Get("ping_monitor_enabled") == "true"
		pingThreshold, err := strconv.Atoi(r.Form.Get("ping_monitor_threshold"))
		if err != nil || pingThreshold < 100 {
			pingThreshold = 500 // Default to 500ms
		}
		newConfig.PingMonitor.HighPingThreshold = pingThreshold

		pingDuration, err := strconv.Atoi(r.Form.Get("ping_monitor_duration"))
		if err != nil || pingDuration < 5 {
			pingDuration = 30 // Default to 30 seconds
		}
		newConfig.PingMonitor.SustainedDuration = pingDuration

		// Auto Start
		newConfig.AutoStart.Enabled = r.Form.Get("autostart_enabled") == "true"
		autoStartDelay, err := strconv.Atoi(r.Form.Get("autostart_delay"))
		if err != nil || autoStartDelay < 0 {
			autoStartDelay = 0
		}
		newConfig.AutoStart.DelaySeconds = autoStartDelay

		err = config.ValidateAndSaveConfig(newConfig)
		if err != nil {
			s.templates.ExecuteTemplate(w, "config.gohtml", ConfigData{
				KooloCfg:       &newConfig,
				ErrorMessage:   err.Error(),
				CurrentVersion: s.getVersionData(),
			})
			return
		}

		http.Redirect(w, r, "/", http.StatusSeeOther)
		return
	}

	// Get current version info
	versionInfo, _ := updater.GetCurrentVersionNoClone()
	var versionData *VersionData
	if versionInfo != nil {
		versionData = &VersionData{
			CommitHash: versionInfo.CommitHash,
			CommitDate: formatCommitDate(versionInfo.CommitDate),
			CommitMsg:  versionInfo.CommitMsg,
			Branch:     versionInfo.Branch,
		}
	}

	s.templates.ExecuteTemplate(w, "config.gohtml", ConfigData{
		KooloCfg:       config.Koolo,
		ErrorMessage:   "",
		CurrentVersion: versionData,
	})
}

// ConfigUpdateOptions defines which sections of the configuration should be updated
// from the provided form data.
type ConfigUpdateOptions struct {
	Identity            bool `json:"identity"` // Name, Auth, etc.
	Health              bool `json:"health"`
	Runs                bool `json:"runs"`
	PacketCasting       bool `json:"packetCasting"`
	CubeRecipes         bool `json:"cubeRecipes"`
	RunewordMaker       bool `json:"runewordMaker"`
	Merc                bool `json:"merc"`
	General             bool `json:"general"` // Includes class specific options too
	GeneralExtras       bool `json:"generalExtras"`
	Client              bool `json:"client"`
	Scheduler           bool `json:"scheduler"`
	Muling              bool `json:"muling"`
	Shopping            bool `json:"shopping"`
	CharacterCreation   bool `json:"characterCreation"` // Auto-create character setting
	UpdateAllRunDetails bool `json:"updateAllRunDetails"`
}

func (s *HttpServer) updateConfigFromForm(values url.Values, cfg *config.CharacterCfg, sections ConfigUpdateOptions, runDetailTargets []string) error {
	// Identity / Basic Settings
	if sections.Identity {
		if v := values.Get("maxGameLength"); v != "" {
			cfg.MaxGameLength, _ = strconv.Atoi(v)
		}
		cfg.CharacterName = values.Get("characterName")
		cfg.Character.Class = values.Get("characterClass")
		cfg.CommandLineArgs = values.Get("commandLineArgs")
		cfg.AutoCreateCharacter = values.Has("autoCreateCharacter")
		cfg.Username = values.Get("username")
		cfg.Password = values.Get("password")
		cfg.Realm = values.Get("realm")
		cfg.AuthMethod = values.Get("authmethod")
		cfg.AuthToken = values.Get("AuthToken")
	}

	// Character Creation Settings
	if sections.CharacterCreation {
		cfg.AutoCreateCharacter = values.Has("autoCreateCharacter")
	}

	// Client Settings
	if sections.Client {
		if !sections.Identity { // If Identity was skipped, handle these here if Client is checked
			cfg.CommandLineArgs = values.Get("commandLineArgs")
		}
		cfg.KillD2OnStop = values.Has("kill_d2_process")
		cfg.ClassicMode = values.Has("classic_mode")
		cfg.HidePortraits = values.Has("hide_portraits")
	}

	// Scheduler
	if sections.Scheduler {
		cfg.Scheduler.Enabled = values.Has("schedulerEnabled")
		cfg.Scheduler.Mode = values.Get("schedulerMode")
		if cfg.Scheduler.Mode == "" {
			cfg.Scheduler.Mode = "timeSlots"
		}

		// Global variance for time slots mode
		if v := values.Get("globalVarianceMin"); v != "" {
			cfg.Scheduler.GlobalVarianceMin, _ = strconv.Atoi(v)
		}

		// Reset scheduler days if we are updating them
		if len(cfg.Scheduler.Days) != 7 {
			cfg.Scheduler.Days = make([]config.Day, 7)
		}

		// Parse time slots mode data
		for day := 0; day < 7; day++ {
			starts := values[fmt.Sprintf("scheduler[%d][start][]", day)]
			ends := values[fmt.Sprintf("scheduler[%d][end][]", day)]
			startVars := values[fmt.Sprintf("scheduler[%d][startVar][]", day)]
			endVars := values[fmt.Sprintf("scheduler[%d][endVar][]", day)]

			cfg.Scheduler.Days[day].DayOfWeek = day
			cfg.Scheduler.Days[day].TimeRanges = make([]config.TimeRange, 0)

			for i := 0; i < len(starts); i++ {
				start, err := time.Parse("15:04", starts[i])
				if err != nil {
					continue
				}
				end, err := time.Parse("15:04", ends[i])
				if err != nil {
					continue
				}

				var startVar, endVar int
				if i < len(startVars) {
					startVar, _ = strconv.Atoi(startVars[i])
				}
				if i < len(endVars) {
					endVar, _ = strconv.Atoi(endVars[i])
				}

				cfg.Scheduler.Days[day].TimeRanges = append(cfg.Scheduler.Days[day].TimeRanges, config.TimeRange{
					Start:            start,
					End:              end,
					StartVarianceMin: startVar,
					EndVarianceMin:   endVar,
				})
			}
		}

		// Parse duration mode data
		cfg.Scheduler.Duration.WakeUpTime = values.Get("durationWakeUpTime")
		if v := values.Get("durationWakeUpVariance"); v != "" {
			cfg.Scheduler.Duration.WakeUpVariance, _ = strconv.Atoi(v)
		}
		if v := values.Get("durationPlayHours"); v != "" {
			cfg.Scheduler.Duration.PlayHours, _ = strconv.Atoi(v)
		}
		if v := values.Get("durationPlayHoursVariance"); v != "" {
			cfg.Scheduler.Duration.PlayHoursVariance, _ = strconv.Atoi(v)
		}
		if v := values.Get("durationMealBreakCount"); v != "" {
			cfg.Scheduler.Duration.MealBreakCount, _ = strconv.Atoi(v)
		}
		if v := values.Get("durationMealBreakDuration"); v != "" {
			cfg.Scheduler.Duration.MealBreakDuration, _ = strconv.Atoi(v)
		}
		if v := values.Get("durationMealBreakVariance"); v != "" {
			cfg.Scheduler.Duration.MealBreakVariance, _ = strconv.Atoi(v)
		}
		if v := values.Get("durationShortBreakCount"); v != "" {
			cfg.Scheduler.Duration.ShortBreakCount, _ = strconv.Atoi(v)
		}
		if v := values.Get("durationShortBreakDuration"); v != "" {
			cfg.Scheduler.Duration.ShortBreakDuration, _ = strconv.Atoi(v)
		}
		if v := values.Get("durationShortBreakVariance"); v != "" {
			cfg.Scheduler.Duration.ShortBreakVariance, _ = strconv.Atoi(v)
		}
		if v := values.Get("durationBreakTimingVariance"); v != "" {
			cfg.Scheduler.Duration.BreakTimingVariance, _ = strconv.Atoi(v)
		}
		if v := values.Get("durationJitterMin"); v != "" {
			cfg.Scheduler.Duration.JitterMin, _ = strconv.Atoi(v)
		}
		if v := values.Get("durationJitterMax"); v != "" {
			cfg.Scheduler.Duration.JitterMax, _ = strconv.Atoi(v)
		}

		if err := validateSchedulerData(cfg); err != nil {
			return err
		}
	}

	// Health
	if sections.Health {
		if v := values.Get("healingPotionAt"); v != "" {
			cfg.Health.HealingPotionAt, _ = strconv.Atoi(v)
		}
		if v := values.Get("manaPotionAt"); v != "" {
			cfg.Health.ManaPotionAt, _ = strconv.Atoi(v)
		}
		if v := values.Get("rejuvPotionAtLife"); v != "" {
			cfg.Health.RejuvPotionAtLife, _ = strconv.Atoi(v)
		}
		if v := values.Get("rejuvPotionAtMana"); v != "" {
			cfg.Health.RejuvPotionAtMana, _ = strconv.Atoi(v)
		}
		if v := values.Get("chickenAt"); v != "" {
			cfg.Health.ChickenAt, _ = strconv.Atoi(v)
		}
		if v := values.Get("townChickenAt"); v != "" {
			cfg.Health.TownChickenAt, _ = strconv.Atoi(v)
		}
		cfg.ChickenOnCurses.AmplifyDamage = values.Has("chickenAmplifyDamage")
		cfg.ChickenOnCurses.Decrepify = values.Has("chickenDecrepify")
		cfg.ChickenOnCurses.LowerResist = values.Has("chickenLowerResist")
		cfg.ChickenOnCurses.BloodMana = values.Has("chickenBloodMana")
		cfg.ChickenOnAuras.Fanaticism = values.Has("chickenFanaticism")
		cfg.ChickenOnAuras.Might = values.Has("chickenMight")
		cfg.ChickenOnAuras.Conviction = values.Has("chickenConviction")
		cfg.ChickenOnAuras.HolyFire = values.Has("chickenHolyFire")
		cfg.ChickenOnAuras.BlessedAim = values.Has("chickenBlessedAim")
		cfg.ChickenOnAuras.HolyFreeze = values.Has("chickenHolyFreeze")
		cfg.ChickenOnAuras.HolyShock = values.Has("chickenHolyShock")
		// Back to town config handled with Health or General?
		// It was in General in bulkApply but logic is closer to Health/Safety.
		// Let's allow updating it if either General or Health is selected, or stick to General.
		// For now, let's keep it under General to match previous bulk logic, or move it if needed.
	}

	// Mercenary
	if sections.Merc {
		cfg.Character.UseMerc = values.Has("useMerc")
		if v := values.Get("mercHealingPotionAt"); v != "" {
			cfg.Health.MercHealingPotionAt, _ = strconv.Atoi(v)
		}
		if v := values.Get("mercRejuvPotionAt"); v != "" {
			cfg.Health.MercRejuvPotionAt, _ = strconv.Atoi(v)
		}
		if v := values.Get("mercChickenAt"); v != "" {
			cfg.Health.MercChickenAt, _ = strconv.Atoi(v)
		}
	}

	// General (Character & Game)
	if sections.General {
		cfg.Character.StashToShared = values.Has("characterStashToShared")
		cfg.Character.UseTeleport = values.Has("characterUseTeleport")
		cfg.Character.UseExtraBuffs = values.Has("characterUseExtraBuffs")
		s.updateAutoStatSkillFromForm(values, cfg)

		// Game Settings (General)
		if v := values.Get("gameMinGoldPickupThreshold"); v != "" {
			cfg.Game.MinGoldPickupThreshold, _ = strconv.Atoi(v)
		}
		cfg.UseCentralizedPickit = values.Has("useCentralizedPickit")
		cfg.Game.UseCainIdentify = values.Has("useCainIdentify")
		cfg.Game.DisableIdentifyTome = values.Get("game.disableIdentifyTome") == "on"
		cfg.Game.InteractWithShrines = values.Has("interactWithShrines")
		cfg.Game.InteractWithChests = values.Has("interactWithChests")
		cfg.Game.InteractWithSuperChests = values.Has("interactWithSuperChests")

		// Ensure the two chest options are mutually exclusive. If both are enabled
		// (e.g. due to manual edits), keep the legacy behavior (all chests).
		if cfg.Game.InteractWithChests {
			cfg.Game.InteractWithSuperChests = false
		}
		if v := values.Get("stopLevelingAt"); v != "" {
			cfg.Game.StopLevelingAt, _ = strconv.Atoi(v)
		}

		if sections.GeneralExtras {
			cfg.Character.UseSwapForBuffs = values.Has("useSwapForBuffs")
			cfg.Character.BuffOnNewArea = values.Has("characterBuffOnNewArea")
			cfg.Character.BuffAfterWP = values.Has("characterBuffAfterWP")

			// Process ClearPathDist - only relevant when teleport is disabled
			if !cfg.Character.UseTeleport {
				clearPathDist, err := strconv.Atoi(values.Get("clearPathDist"))
				if err == nil && clearPathDist >= 0 && clearPathDist <= 30 {
					cfg.Character.ClearPathDist = clearPathDist
				} else {
					// Set default value if invalid
					cfg.Character.ClearPathDist = 7
				}
			} else {
				cfg.Character.ClearPathDist = 7
			}

			// Inventory Lock
			for y, row := range cfg.Inventory.InventoryLock {
				for x := range row {
					if values.Has(fmt.Sprintf("inventoryLock[%d][%d]", y, x)) {
						cfg.Inventory.InventoryLock[y][x] = 0
					} else {
						cfg.Inventory.InventoryLock[y][x] = 1
					}
				}
			}

			// Belt Columns
			if cols, ok := values["inventoryBeltColumns[]"]; ok {
				copy(cfg.Inventory.BeltColumns[:], cols)
			}

			if v := values.Get("healingPotionCount"); v != "" {
				cfg.Inventory.HealingPotionCount, _ = strconv.Atoi(v)
			}
			if v := values.Get("manaPotionCount"); v != "" {
				cfg.Inventory.ManaPotionCount, _ = strconv.Atoi(v)
			}
			if v := values.Get("rejuvPotionCount"); v != "" {
				cfg.Inventory.RejuvPotionCount, _ = strconv.Atoi(v)
			}

			cfg.Game.CreateLobbyGames = values.Has("createLobbyGames")
			cfg.Game.IsNonLadderChar = values.Has("isNonLadderChar")
			cfg.Game.IsHardCoreChar = values.Has("isHardCoreChar")
			cfg.Game.Difficulty = difficulty.Difficulty(values.Get("gameDifficulty"))
			cfg.Game.RandomizeRuns = values.Has("gameRandomizeRuns")

			// Back To Town Settings
			cfg.BackToTown.NoHpPotions = values.Has("noHpPotions")
			cfg.BackToTown.NoMpPotions = values.Has("noMpPotions")
			cfg.BackToTown.MercDied = values.Has("mercDied")
			cfg.BackToTown.EquipmentBroken = values.Has("equipmentBroken")

			// Companion
			cfg.Companion.Enabled = values.Has("companionEnabled")
			cfg.Companion.Leader = values.Has("companionLeader")
			cfg.Companion.LeaderName = values.Get("companionLeaderName")
			cfg.Companion.GameNameTemplate = values.Get("companionGameNameTemplate")
			cfg.Companion.GamePassword = values.Get("companionGamePassword")

			// Gambling
			cfg.Gambling.Enabled = values.Has("gamblingEnabled")
			if raw := strings.TrimSpace(values.Get("gamblingItems")); raw != "" {
				parts := strings.Split(raw, ",")
				items := make([]string, 0, len(parts))
				for _, p := range parts {
					if p = strings.TrimSpace(p); p != "" {
						items = append(items, p)
					}
				}
				cfg.Gambling.Items = items
			} else {
				cfg.Gambling.Items = []string{}
			}
		}

		// Class-specific options are only updated when identity is explicitly updated.
		if sections.Identity {
			s.updateClassSpecificConfig(values, cfg)
		}
	}

	// Packet Casting
	if sections.PacketCasting {
		cfg.PacketCasting.UseForEntranceInteraction = values.Has("packetCastingUseForEntranceInteraction")
		cfg.PacketCasting.UseForItemPickup = values.Has("packetCastingUseForItemPickup")
		cfg.PacketCasting.UseForTpInteraction = values.Has("packetCastingUseForTpInteraction")
		cfg.PacketCasting.UseForTeleport = values.Has("packetCastingUseForTeleport")
		cfg.PacketCasting.UseForEntitySkills = values.Has("packetCastingUseForEntitySkills")
		cfg.PacketCasting.UseForSkillSelection = values.Has("packetCastingUseForSkillSelection")
	}

	// Cube Recipes
	if sections.CubeRecipes {
		cfg.CubeRecipes.Enabled = values.Has("enableCubeRecipes")
		if recipes, ok := values["enabledRecipes"]; ok {
			cfg.CubeRecipes.EnabledRecipes = recipes
		}
		cfg.CubeRecipes.SkipPerfectAmethysts = values.Has("skipPerfectAmethysts")
		cfg.CubeRecipes.SkipPerfectRubies = values.Has("skipPerfectRubies")
		if v := values.Get("jewelsToKeep"); v != "" {
			if n, err := strconv.Atoi(v); err == nil && n >= 0 {
				cfg.CubeRecipes.JewelsToKeep = n
			} else {
				cfg.CubeRecipes.JewelsToKeep = 1
			}
		}
	}

	// Muling
	if sections.Muling {
		cfg.Muling.Enabled = values.Get("mulingEnabled") == "on"
		cfg.Muling.ReturnTo = values.Get("mulingReturnTo")

		// Validate mule profiles
		requestedMuleProfiles := values["mulingMuleProfiles[]"]
		validMuleProfiles := []string{}
		allCharacters := config.GetCharacters()
		for _, muleName := range requestedMuleProfiles {
			if muleCfg, exists := allCharacters[muleName]; exists && strings.ToLower(muleCfg.Character.Class) == "mule" {
				validMuleProfiles = append(validMuleProfiles, muleName)
			}
		}
		cfg.Muling.MuleProfiles = validMuleProfiles
	}

	// Shopping
	if sections.Shopping {
		s.applyShoppingFromForm(values, cfg)
	}

	// Runs
	if sections.Runs {
		if raw := values.Get("gameRuns"); raw != "" {
			var enabledRuns []config.Run
			if err := json.Unmarshal([]byte(raw), &enabledRuns); err == nil {
				cfg.Game.Runs = enabledRuns
			}
		}

		// Run Details
		if sections.UpdateAllRunDetails {
			// Update ALL run details if UpdateAllRunDetails is true
			s.applyRunDetails(values, cfg, getAllRunIDs())
		} else if len(runDetailTargets) > 0 {
			// Update only specific run details
			s.applyRunDetails(values, cfg, runDetailTargets)
		}
	}

	return nil
}

func (s *HttpServer) updateClassSpecificConfig(values url.Values, cfg *config.CharacterCfg) {
	if paladin.IsClass(cfg.Character.Class) {
		paladin.ApplyUI(values, cfg)
	}

	// Berserker Barb specific options
	if cfg.Character.Class == "berserker" {
		cfg.Character.BerserkerBarb.SkipPotionPickupInTravincal = values.Has("barbSkipPotionPickupInTravincal")
		cfg.Character.BerserkerBarb.FindItemSwitch = values.Has("characterFindItemSwitch")
		cfg.Character.BerserkerBarb.UseHowl = values.Has("barbUseHowl")
		if cfg.Character.BerserkerBarb.UseHowl {
			howlCooldown, err := strconv.Atoi(values.Get("barbHowlCooldown"))
			if err == nil && howlCooldown >= 1 && howlCooldown <= 60 {
				cfg.Character.BerserkerBarb.HowlCooldown = howlCooldown
			} else {
				cfg.Character.BerserkerBarb.HowlCooldown = 6
			}
			howlMinMonsters, err := strconv.Atoi(values.Get("barbHowlMinMonsters"))
			if err == nil && howlMinMonsters >= 1 && howlMinMonsters <= 20 {
				cfg.Character.BerserkerBarb.HowlMinMonsters = howlMinMonsters
			} else {
				cfg.Character.BerserkerBarb.HowlMinMonsters = 4
			}
		}
		cfg.Character.BerserkerBarb.UseBattleCry = values.Has("barbUseBattleCry")
		if cfg.Character.BerserkerBarb.UseBattleCry {
			battleCryCooldown, err := strconv.Atoi(values.Get("barbBattleCryCooldown"))
			if err == nil && battleCryCooldown >= 1 && battleCryCooldown <= 60 {
				cfg.Character.BerserkerBarb.BattleCryCooldown = battleCryCooldown
			} else {
				cfg.Character.BerserkerBarb.BattleCryCooldown = 6
			}
			battleCryMinMonsters, err := strconv.Atoi(values.Get("barbBattleCryMinMonsters"))
			if err == nil && battleCryMinMonsters >= 1 && battleCryMinMonsters <= 20 {
				cfg.Character.BerserkerBarb.BattleCryMinMonsters = battleCryMinMonsters
			} else {
				cfg.Character.BerserkerBarb.BattleCryMinMonsters = 4
			}
		}
		cfg.Character.BerserkerBarb.HorkNormalMonsters = values.Has("berserkerBarbHorkNormalMonsters")
		horkRange, err := strconv.Atoi(values.Get("berserkerBarbHorkMonsterCheckRange"))
		if err == nil && horkRange > 0 {
			cfg.Character.BerserkerBarb.HorkMonsterCheckRange = horkRange
		} else {
			cfg.Character.BerserkerBarb.HorkMonsterCheckRange = 7
		}
	}

	// Barb Leveling specific options
	if cfg.Character.Class == "barb_leveling" {
		cfg.Character.BarbLeveling.UseHowl = values.Has("barbLevelingUseHowl")
		if cfg.Character.BarbLeveling.UseHowl {
			howlCooldown, err := strconv.Atoi(values.Get("barbLevelingHowlCooldown"))
			if err == nil && howlCooldown >= 1 && howlCooldown <= 60 {
				cfg.Character.BarbLeveling.HowlCooldown = howlCooldown
			} else {
				cfg.Character.BarbLeveling.HowlCooldown = 8
			}
			howlMinMonsters, err := strconv.Atoi(values.Get("barbLevelingHowlMinMonsters"))
			if err == nil && howlMinMonsters >= 1 && howlMinMonsters <= 20 {
				cfg.Character.BarbLeveling.HowlMinMonsters = howlMinMonsters
			} else {
				cfg.Character.BarbLeveling.HowlMinMonsters = 4
			}
		}
		cfg.Character.BarbLeveling.UseBattleCry = values.Has("barbLevelingUseBattleCry")
		if cfg.Character.BarbLeveling.UseBattleCry {
			battleCryCooldown, err := strconv.Atoi(values.Get("barbLevelingBattleCryCooldown"))
			if err == nil && battleCryCooldown >= 1 && battleCryCooldown <= 60 {
				cfg.Character.BarbLeveling.BattleCryCooldown = battleCryCooldown
			} else {
				cfg.Character.BarbLeveling.BattleCryCooldown = 6
			}
			battleCryMinMonsters, err := strconv.Atoi(values.Get("barbLevelingBattleCryMinMonsters"))
			if err == nil && battleCryMinMonsters >= 1 && battleCryMinMonsters <= 20 {
				cfg.Character.BarbLeveling.BattleCryMinMonsters = battleCryMinMonsters
			} else {
				cfg.Character.BarbLeveling.BattleCryMinMonsters = 1
			}
			cfg.Character.BarbLeveling.UsePacketLearning = values.Has("usePacketLearning")
		}
	}

	// Warcry Barb specific options
	if cfg.Character.Class == "warcry_barb" {
		cfg.Character.WarcryBarb.FindItemSwitch = values.Has("warcryBarbFindItemSwitch")
		cfg.Character.WarcryBarb.SkipPotionPickupInTravincal = values.Has("warcryBarbSkipPotionPickupInTravincal")
		cfg.Character.WarcryBarb.UseHowl = values.Has("warcryBarbUseHowl")
		if cfg.Character.WarcryBarb.UseHowl {
			howlCooldown, err := strconv.Atoi(values.Get("warcryBarbHowlCooldown"))
			if err == nil && howlCooldown >= 1 && howlCooldown <= 60 {
				cfg.Character.WarcryBarb.HowlCooldown = howlCooldown
			} else {
				cfg.Character.WarcryBarb.HowlCooldown = 8
			}
			howlMinMonsters, err := strconv.Atoi(values.Get("warcryBarbHowlMinMonsters"))
			if err == nil && howlMinMonsters >= 1 && howlMinMonsters <= 20 {
				cfg.Character.WarcryBarb.HowlMinMonsters = howlMinMonsters
			} else {
				cfg.Character.WarcryBarb.HowlMinMonsters = 4
			}
		}
		cfg.Character.WarcryBarb.UseBattleCry = values.Has("warcryBarbUseBattleCry")
		if cfg.Character.WarcryBarb.UseBattleCry {
			battleCryCooldown, err := strconv.Atoi(values.Get("warcryBarbBattleCryCooldown"))
			if err == nil && battleCryCooldown >= 1 && battleCryCooldown <= 60 {
				cfg.Character.WarcryBarb.BattleCryCooldown = battleCryCooldown
			} else {
				cfg.Character.WarcryBarb.BattleCryCooldown = 6
			}
			battleCryMinMonsters, err := strconv.Atoi(values.Get("warcryBarbBattleCryMinMonsters"))
			if err == nil && battleCryMinMonsters >= 1 && battleCryMinMonsters <= 20 {
				cfg.Character.WarcryBarb.BattleCryMinMonsters = battleCryMinMonsters
			} else {
				cfg.Character.WarcryBarb.BattleCryMinMonsters = 1
			}
		}
		cfg.Character.WarcryBarb.UseGrimWard = values.Has("warcryBarbUseGrimWard")
		cfg.Character.WarcryBarb.HorkNormalMonsters = values.Has("warcryBarbHorkNormalMonsters")
		horkRange, err := strconv.Atoi(values.Get("warcryBarbHorkMonsterCheckRange"))
		if err == nil && horkRange > 0 {
			cfg.Character.WarcryBarb.HorkMonsterCheckRange = horkRange
		} else {
			cfg.Character.WarcryBarb.HorkMonsterCheckRange = 7
		}
	}

	// Nova Sorceress specific options
	if cfg.Character.Class == "nova" || cfg.Character.Class == "lightsorc" {
		bossStaticThreshold, err := strconv.Atoi(values.Get("novaBossStaticThreshold"))
		if err == nil {
			minThreshold := 65
			switch cfg.Game.Difficulty {
			case difficulty.Normal:
				minThreshold = 1
			case difficulty.Nightmare:
				minThreshold = 33
			case difficulty.Hell:
				minThreshold = 50
			}
			if bossStaticThreshold >= minThreshold && bossStaticThreshold <= 100 {
				cfg.Character.NovaSorceress.BossStaticThreshold = bossStaticThreshold
			} else {
				cfg.Character.NovaSorceress.BossStaticThreshold = minThreshold
			}
		} else {
			cfg.Character.NovaSorceress.BossStaticThreshold = 65
		}
	}

	// Mosaic specific options
	if cfg.Character.Class == "mosaic" {
		cfg.Character.MosaicSin.UseTigerStrike = values.Has("mosaicUseTigerStrike")
		cfg.Character.MosaicSin.UseCobraStrike = values.Has("mosaicUseCobraStrike")
		cfg.Character.MosaicSin.UseClawsOfThunder = values.Has("mosaicUseClawsOfThunder")
		cfg.Character.MosaicSin.UseBladesOfIce = values.Has("mosaicUseBladesOfIce")
		cfg.Character.MosaicSin.UseFistsOfFire = values.Has("mosaicUseFistsOfFire")
	}

	// Blizzard Sorc specific options
	if cfg.Character.Class == "sorceress" {
		cfg.Character.BlizzardSorceress.UseMoatTrick = values.Has("blizzardUseMoatTrick")
		cfg.Character.BlizzardSorceress.UseStaticOnMephisto = values.Has("blizzardUseStaticOnMephisto")
		cfg.Character.BlizzardSorceress.UseBlizzardPackets = values.Has("blizzardUseBlizzardPackets")
	}

	// Sorceress Leveling specific options
	if cfg.Character.Class == "sorceress_leveling" {
		cfg.Character.SorceressLeveling.UseMoatTrick = values.Has("levelingUseMoatTrick")
		cfg.Character.SorceressLeveling.UseStaticOnMephisto = values.Has("levelingUseStaticOnMephisto")
		cfg.Character.SorceressLeveling.UseBlizzardPackets = values.Has("levelingUseBlizzardPackets")
		cfg.Character.SorceressLeveling.UsePacketLearning = values.Has("levelingUsePacketLearning")
	}

	// Assassin Leveling specific options
	if cfg.Character.Class == "assassin" {
		cfg.Character.AssassinLeveling.UsePacketLearning = values.Has("usePacketLearning")
	}

	// Amazon Leveling specific options
	if cfg.Character.Class == "amazon_leveling" {
		cfg.Character.AmazonLeveling.UsePacketLearning = values.Has("usePacketLearning")
	}

	// Druid Leveling specific options
	if cfg.Character.Class == "druid_leveling" {
		cfg.Character.DruidLeveling.UsePacketLearning = values.Has("usePacketLearning")
	}

	// Necromancer Leveling specific options
	if cfg.Character.Class == "necromancer" {
		cfg.Character.NecromancerLeveling.UsePacketLearning = values.Has("usePacketLearning")
	}

	// Paladin Leveling specific options
	if cfg.Character.Class == "paladin_leveling" {
		cfg.Character.PaladinLeveling.UsePacketLearning = values.Has("usePacketLearning")
		levelingBuild := strings.ToLower(strings.TrimSpace(values.Get("paladinLevelingBuild")))
		if levelingBuild != "foh" && levelingBuild != "hammer" {
			levelingBuild = "hammer"
		}
		cfg.Character.PaladinLeveling.LevelingBuild = levelingBuild
	}

	// Nova Sorceress specific options (Extra)
	if cfg.Character.Class == "nova" {
		cfg.Character.NovaSorceress.AggressiveNovaPositioning = values.Has("aggressiveNovaPositioning")
	}

	// Javazon specific options
	if cfg.Character.Class == "javazon" {
		cfg.Character.Javazon.DensityKillerEnabled = values.Has("javazonDensityKillerEnabled")
		if v := values.Get("javazonDensityKillerIgnoreWhitesBelow"); v != "" {
			if i, err := strconv.Atoi(v); err == nil {
				cfg.Character.Javazon.DensityKillerIgnoreWhitesBelow = i
			} else {
				cfg.Character.Javazon.DensityKillerIgnoreWhitesBelow = 4
			}
		} else if cfg.Character.Javazon.DensityKillerIgnoreWhitesBelow == 0 {
			cfg.Character.Javazon.DensityKillerIgnoreWhitesBelow = 4
		}
		if v := values.Get("javazonDensityKillerForceRefillBelowPercent"); v != "" {
			if i, err := strconv.Atoi(v); err == nil {
				if i < 1 {
					i = 1
				}
				if i > 100 {
					i = 100
				}
				cfg.Character.Javazon.DensityKillerForceRefillBelowPercent = i
			} else {
				cfg.Character.Javazon.DensityKillerForceRefillBelowPercent = 50
			}
		} else if cfg.Character.Javazon.DensityKillerForceRefillBelowPercent == 0 {
			cfg.Character.Javazon.DensityKillerForceRefillBelowPercent = 50
		}
	}

	// Lightning Sorceress specific options
	if cfg.Character.Class == "lightsorc" {
	}

	// Hydra Orb Sorceress specific options
	if cfg.Character.Class == "hydraorb" {
	}

	// Fireball Sorceress specific options
	if cfg.Character.Class == "fireballsorc" {
	}
}

func getAllRunIDs() []string {
	// A helper to get all possible run keys if we want to apply everything
	// This list should ideally match all case statements in applyRunDetails
	return []string{
		"andariel", "countess", "duriel", "pit", "cows", "pindleskin",
		"stony_tomb", "mausoleum", "ancient_tunnels", "drifter_cavern",
		"spider_cavern", "arachnid_lair", "mephisto", "tristram",
		"nihlathak", "summoner", "baal", "eldritch", "lower_kurast_chest",
		"diablo", "leveling", "leveling_sequence", "quests", "terror_zone",
		"utility", "shopping",
	}
}

func sanitizeFavoriteRunSelection(selected []string) []string {
	seen := make(map[string]struct{}, len(selected))
	result := make([]string, 0, len(selected))
	for _, name := range selected {
		trimmed := strings.TrimSpace(name)
		if trimmed == "" {
			continue
		}
		if _, ok := config.AvailableRuns[config.Run(trimmed)]; !ok {
			continue
		}
		if _, ok := seen[trimmed]; ok {
			continue
		}
		seen[trimmed] = struct{}{}
		result = append(result, trimmed)
	}
	return result
}

func cloneCharacterCfg(cfg *config.CharacterCfg) (*config.CharacterCfg, error) {
	if cfg == nil {
		return nil, errors.New("nil source config")
	}
	raw, err := yaml.Marshal(cfg)
	if err != nil {
		return nil, err
	}
	var out config.CharacterCfg
	if err := yaml.Unmarshal(raw, &out); err != nil {
		return nil, err
	}
	return &out, nil
}

func applyRunewordSettings(dst *config.CharacterCfg, src *config.CharacterCfg) {
	if dst == nil || src == nil {
		return
	}
	dst.Game.RunewordMaker = src.Game.RunewordMaker
	dst.Game.RunewordOverrides = src.Game.RunewordOverrides
	dst.Game.RunewordRerollRules = src.Game.RunewordRerollRules
	dst.CubeRecipes.PrioritizeRunewords = src.CubeRecipes.PrioritizeRunewords
}

func (s *HttpServer) characterSettings(w http.ResponseWriter, r *http.Request) {
	sequenceFiles := s.listLevelingSequenceFiles()
	defaultSkillOptions := buildSkillOptionsForBuild("")
	var err error
	if r.Method == http.MethodPost {
		err = r.ParseForm()
		if err != nil {
			s.templates.ExecuteTemplate(w, "character_settings.gohtml", CharacterSettings{
				Version:               config.Version,
				ErrorMessage:          err.Error(),
				SkillOptions:          defaultSkillOptions,
				LevelingSequenceFiles: sequenceFiles,
				RunFavoriteRuns:       config.Koolo.RunFavoriteRuns,
			})
			return
		}

		supervisorName := r.Form.Get("name")
		cloneSource := strings.TrimSpace(r.Form.Get("cloneSource"))
		cfg, found := config.GetCharacter(supervisorName)
		if !found {
			err = config.CreateFromTemplate(supervisorName)
			if err != nil {
				s.templates.ExecuteTemplate(w, "character_settings.gohtml", CharacterSettings{
					Version:               config.Version,
					ErrorMessage:          err.Error(),
					Supervisor:            supervisorName,
					SkillOptions:          defaultSkillOptions,
					LevelingSequenceFiles: sequenceFiles,
					RunFavoriteRuns:       config.Koolo.RunFavoriteRuns,
				})
				return
			}
			cfg, found = config.GetCharacter(supervisorName)
			if !found || cfg == nil {
				s.templates.ExecuteTemplate(w, "character_settings.gohtml", CharacterSettings{
					Version:               config.Version,
					ErrorMessage:          "failed to load newly created configuration",
					Supervisor:            supervisorName,
					SkillOptions:          defaultSkillOptions,
					LevelingSequenceFiles: sequenceFiles,
					RunFavoriteRuns:       config.Koolo.RunFavoriteRuns,
				})
				return
			}

			if cloneSource != "" {
				if cloneCfg, ok := config.GetCharacter(cloneSource); ok && cloneCfg != nil {
					cloned, err := cloneCharacterCfg(cloneCfg)
					if err != nil {
						s.logger.Warn("failed to clone character config", slog.String("source", cloneSource), slog.Any("error", err))
					} else {
						cloned.ConfigFolderName = cfg.ConfigFolderName
						*cfg = *cloned
					}
				}
			}
		}

		if v := strings.TrimSpace(r.Form.Get("characterName")); v != "" {
			cfg.CharacterName = v
		}
		cfg.Game.RunewordMaker.Enabled = r.Form.Has("runewordMakerEnabled")
		cfg.AutoCreateCharacter = r.Form.Has("autoCreateCharacter")
		cfg.Username = r.Form.Get("username")
		cfg.Password = r.Form.Get("password")
		cfg.Realm = r.Form.Get("realm")
		cfg.AuthMethod = r.Form.Get("authmethod")
		cfg.AuthToken = r.Form.Get("AuthToken")
		cfg.CommandLineArgs = r.Form.Get("commandLineArgs")
		cfg.KillD2OnStop = r.Form.Has("kill_d2_process")
		cfg.ClassicMode = r.Form.Has("classic_mode")
		cfg.HidePortraits = r.Form.Has("hide_portraits")

		// Health config
		cfg.Health.HealingPotionAt, _ = strconv.Atoi(r.Form.Get("healingPotionAt"))
		cfg.Health.ManaPotionAt, _ = strconv.Atoi(r.Form.Get("manaPotionAt"))
		cfg.Health.RejuvPotionAtLife, _ = strconv.Atoi(r.Form.Get("rejuvPotionAtLife"))
		cfg.Health.RejuvPotionAtMana, _ = strconv.Atoi(r.Form.Get("rejuvPotionAtMana"))
		cfg.Health.ChickenAt, _ = strconv.Atoi(r.Form.Get("chickenAt"))
		cfg.Health.TownChickenAt, _ = strconv.Atoi(r.Form.Get("townChickenAt"))
		cfg.Character.UseMerc = r.Form.Has("useMerc")
		cfg.Health.MercHealingPotionAt, _ = strconv.Atoi(r.Form.Get("mercHealingPotionAt"))
		cfg.Health.MercRejuvPotionAt, _ = strconv.Atoi(r.Form.Get("mercRejuvPotionAt"))
		cfg.Health.MercChickenAt, _ = strconv.Atoi(r.Form.Get("mercChickenAt"))

		// Chicken on Curses/Auras
		cfg.ChickenOnCurses.AmplifyDamage = r.Form.Has("chickenAmplifyDamage")
		cfg.ChickenOnCurses.Decrepify = r.Form.Has("chickenDecrepify")
		cfg.ChickenOnCurses.LowerResist = r.Form.Has("chickenLowerResist")
		cfg.ChickenOnCurses.BloodMana = r.Form.Has("chickenBloodMana")
		cfg.ChickenOnAuras.Fanaticism = r.Form.Has("chickenFanaticism")
		cfg.ChickenOnAuras.Might = r.Form.Has("chickenMight")
		cfg.ChickenOnAuras.Conviction = r.Form.Has("chickenConviction")
		cfg.ChickenOnAuras.HolyFire = r.Form.Has("chickenHolyFire")
		cfg.ChickenOnAuras.BlessedAim = r.Form.Has("chickenBlessedAim")
		cfg.ChickenOnAuras.HolyFreeze = r.Form.Has("chickenHolyFreeze")
		cfg.ChickenOnAuras.HolyShock = r.Form.Has("chickenHolyShock")

		// Character config section
		cfg.Character.Class = r.Form.Get("characterClass")
		if strings.HasSuffix(cfg.Character.Class, "_leveling") {
			// Leveling characters should start without cloned runeword reroll/override rules.
			cfg.Game.RunewordOverrides = nil
			cfg.Game.RunewordRerollRules = nil
		}
		cfg.Character.StashToShared = r.Form.Has("characterStashToShared")
		cfg.Character.UseTeleport = r.Form.Has("characterUseTeleport")
		cfg.Character.UseExtraBuffs = r.Form.Has("characterUseExtraBuffs")
		cfg.Character.UseSwapForBuffs = r.Form.Has("useSwapForBuffs")
		cfg.Character.BuffOnNewArea = r.Form.Has("characterBuffOnNewArea")
		cfg.Character.BuffAfterWP = r.Form.Has("characterBuffAfterWP")
		s.updateAutoStatSkillFromForm(r.Form, cfg)

		// Process ClearPathDist - only relevant when teleport is disabled
		if !cfg.Character.UseTeleport {
			clearPathDist, err := strconv.Atoi(r.Form.Get("clearPathDist"))
			if err == nil && clearPathDist >= 0 && clearPathDist <= 30 {
				cfg.Character.ClearPathDist = clearPathDist
			} else {
				// Set default value if invalid
				cfg.Character.ClearPathDist = 7
				s.logger.Debug("Using default ClearPathDist value",
					slog.Int("default", 7),
					slog.String("input", r.Form.Get("clearPathDist")))
			}
		} else {
			cfg.Character.ClearPathDist = 7
		}

		if paladin.IsClass(cfg.Character.Class) {
			paladin.ApplyUI(r.Form, cfg)
		}

		// Berserker Barb specific options
		if cfg.Character.Class == "berserker" {
			cfg.Character.BerserkerBarb.SkipPotionPickupInTravincal = r.Form.Has("barbSkipPotionPickupInTravincal")
			cfg.Character.BerserkerBarb.FindItemSwitch = r.Form.Has("characterFindItemSwitch")
			cfg.Character.BerserkerBarb.UseHowl = r.Form.Has("barbUseHowl")
			if cfg.Character.BerserkerBarb.UseHowl {
				howlCooldown, err := strconv.Atoi(r.Form.Get("barbHowlCooldown"))
				if err == nil && howlCooldown >= 1 && howlCooldown <= 60 {
					cfg.Character.BerserkerBarb.HowlCooldown = howlCooldown
				} else {
					cfg.Character.BerserkerBarb.HowlCooldown = 6
				}
				howlMinMonsters, err := strconv.Atoi(r.Form.Get("barbHowlMinMonsters"))
				if err == nil && howlMinMonsters >= 1 && howlMinMonsters <= 20 {
					cfg.Character.BerserkerBarb.HowlMinMonsters = howlMinMonsters
				} else {
					cfg.Character.BerserkerBarb.HowlMinMonsters = 4
				}
			}
			cfg.Character.BerserkerBarb.UseBattleCry = r.Form.Has("barbUseBattleCry")
			if cfg.Character.BerserkerBarb.UseBattleCry {
				battleCryCooldown, err := strconv.Atoi(r.Form.Get("barbBattleCryCooldown"))
				if err == nil && battleCryCooldown >= 1 && battleCryCooldown <= 60 {
					cfg.Character.BerserkerBarb.BattleCryCooldown = battleCryCooldown
				} else {
					cfg.Character.BerserkerBarb.BattleCryCooldown = 6
				}
				battleCryMinMonsters, err := strconv.Atoi(r.Form.Get("barbBattleCryMinMonsters"))
				if err == nil && battleCryMinMonsters >= 1 && battleCryMinMonsters <= 20 {
					cfg.Character.BerserkerBarb.BattleCryMinMonsters = battleCryMinMonsters
				} else {
					cfg.Character.BerserkerBarb.BattleCryMinMonsters = 4
				}
			}
			cfg.Character.BerserkerBarb.HorkNormalMonsters = r.Form.Has("berserkerBarbHorkNormalMonsters")
			horkRange, err := strconv.Atoi(r.Form.Get("berserkerBarbHorkMonsterCheckRange"))
			if err == nil && horkRange > 0 {
				cfg.Character.BerserkerBarb.HorkMonsterCheckRange = horkRange
			} else {
				cfg.Character.BerserkerBarb.HorkMonsterCheckRange = 7
			}
		}

		// Barb Leveling specific options
		if cfg.Character.Class == "barb_leveling" {
			cfg.Character.BarbLeveling.UseHowl = r.Form.Has("barbLevelingUseHowl")
			if cfg.Character.BarbLeveling.UseHowl {
				howlCooldown, err := strconv.Atoi(r.Form.Get("barbLevelingHowlCooldown"))
				if err == nil && howlCooldown >= 1 && howlCooldown <= 60 {
					cfg.Character.BarbLeveling.HowlCooldown = howlCooldown
				} else {
					cfg.Character.BarbLeveling.HowlCooldown = 8
				}
				howlMinMonsters, err := strconv.Atoi(r.Form.Get("barbLevelingHowlMinMonsters"))
				if err == nil && howlMinMonsters >= 1 && howlMinMonsters <= 20 {
					cfg.Character.BarbLeveling.HowlMinMonsters = howlMinMonsters
				} else {
					cfg.Character.BarbLeveling.HowlMinMonsters = 4
				}
			}
			cfg.Character.BarbLeveling.UseBattleCry = r.Form.Has("barbLevelingUseBattleCry")
			if cfg.Character.BarbLeveling.UseBattleCry {
				battleCryCooldown, err := strconv.Atoi(r.Form.Get("barbLevelingBattleCryCooldown"))
				if err == nil && battleCryCooldown >= 1 && battleCryCooldown <= 60 {
					cfg.Character.BarbLeveling.BattleCryCooldown = battleCryCooldown
				} else {
					cfg.Character.BarbLeveling.BattleCryCooldown = 6
				}
				battleCryMinMonsters, err := strconv.Atoi(r.Form.Get("barbLevelingBattleCryMinMonsters"))
				if err == nil && battleCryMinMonsters >= 1 && battleCryMinMonsters <= 20 {
					cfg.Character.BarbLeveling.BattleCryMinMonsters = battleCryMinMonsters
				} else {
					cfg.Character.BarbLeveling.BattleCryMinMonsters = 1
				}
				cfg.Character.BarbLeveling.UsePacketLearning = r.Form.Has("usePacketLearning")
			}
		}

		// Warcry Barb specific options
		if cfg.Character.Class == "warcry_barb" {
			cfg.Character.WarcryBarb.FindItemSwitch = r.Form.Has("warcryBarbFindItemSwitch")
			cfg.Character.WarcryBarb.SkipPotionPickupInTravincal = r.Form.Has("warcryBarbSkipPotionPickupInTravincal")
			cfg.Character.WarcryBarb.UseHowl = r.Form.Has("warcryBarbUseHowl")
			if cfg.Character.WarcryBarb.UseHowl {
				howlCooldown, err := strconv.Atoi(r.Form.Get("warcryBarbHowlCooldown"))
				if err == nil && howlCooldown >= 1 && howlCooldown <= 60 {
					cfg.Character.WarcryBarb.HowlCooldown = howlCooldown
				} else {
					cfg.Character.WarcryBarb.HowlCooldown = 8
				}
				howlMinMonsters, err := strconv.Atoi(r.Form.Get("warcryBarbHowlMinMonsters"))
				if err == nil && howlMinMonsters >= 1 && howlMinMonsters <= 20 {
					cfg.Character.WarcryBarb.HowlMinMonsters = howlMinMonsters
				} else {
					cfg.Character.WarcryBarb.HowlMinMonsters = 4
				}
			}
			cfg.Character.WarcryBarb.UseBattleCry = r.Form.Has("warcryBarbUseBattleCry")
			if cfg.Character.WarcryBarb.UseBattleCry {
				battleCryCooldown, err := strconv.Atoi(r.Form.Get("warcryBarbBattleCryCooldown"))
				if err == nil && battleCryCooldown >= 1 && battleCryCooldown <= 60 {
					cfg.Character.WarcryBarb.BattleCryCooldown = battleCryCooldown
				} else {
					cfg.Character.WarcryBarb.BattleCryCooldown = 6
				}
				battleCryMinMonsters, err := strconv.Atoi(r.Form.Get("warcryBarbBattleCryMinMonsters"))
				if err == nil && battleCryMinMonsters >= 1 && battleCryMinMonsters <= 20 {
					cfg.Character.WarcryBarb.BattleCryMinMonsters = battleCryMinMonsters
				} else {
					cfg.Character.WarcryBarb.BattleCryMinMonsters = 1
				}
			}
			cfg.Character.WarcryBarb.UseGrimWard = r.Form.Has("warcryBarbUseGrimWard")
			cfg.Character.WarcryBarb.HorkNormalMonsters = r.Form.Has("warcryBarbHorkNormalMonsters")
			horkRange, err := strconv.Atoi(r.Form.Get("warcryBarbHorkMonsterCheckRange"))
			if err == nil && horkRange > 0 {
				cfg.Character.WarcryBarb.HorkMonsterCheckRange = horkRange
			} else {
				cfg.Character.WarcryBarb.HorkMonsterCheckRange = 7
			}
		}

		// Nova Sorceress specific options
		if cfg.Character.Class == "nova" || cfg.Character.Class == "lightsorc" {
			bossStaticThreshold, err := strconv.Atoi(r.Form.Get("novaBossStaticThreshold"))
			if err == nil {
				minThreshold := 65 // Default
				switch cfg.Game.Difficulty {
				case difficulty.Normal:
					minThreshold = 1
				case difficulty.Nightmare:
					minThreshold = 33
				case difficulty.Hell:
					minThreshold = 50
				}
				if bossStaticThreshold >= minThreshold && bossStaticThreshold <= 100 {
					cfg.Character.NovaSorceress.BossStaticThreshold = bossStaticThreshold
				} else {
					cfg.Character.NovaSorceress.BossStaticThreshold = minThreshold
					s.logger.Warn("Invalid Boss Static Threshold, setting to minimum for difficulty",
						slog.Int("min", minThreshold),
						slog.String("difficulty", string(cfg.Game.Difficulty)))
				}
			} else {
				cfg.Character.NovaSorceress.BossStaticThreshold = 65 // Default value
				s.logger.Warn("Invalid Boss Static Threshold input, setting to default", slog.Int("default", 65))
			}
		}

		// Mosaic specific options
		if cfg.Character.Class == "mosaic" {
			cfg.Character.MosaicSin.UseTigerStrike = r.Form.Has("mosaicUseTigerStrike")
			cfg.Character.MosaicSin.UseCobraStrike = r.Form.Has("mosaicUseCobraStrike")
			cfg.Character.MosaicSin.UseClawsOfThunder = r.Form.Has("mosaicUseClawsOfThunder")
			cfg.Character.MosaicSin.UseBladesOfIce = r.Form.Has("mosaicUseBladesOfIce")
			cfg.Character.MosaicSin.UseFistsOfFire = r.Form.Has("mosaicUseFistsOfFire")
		}

		// Blizzard Sorc specific options
		if cfg.Character.Class == "sorceress" {
			cfg.Character.BlizzardSorceress.UseMoatTrick = r.Form.Has("blizzardUseMoatTrick")
			cfg.Character.BlizzardSorceress.UseStaticOnMephisto = r.Form.Has("blizzardUseStaticOnMephisto")
			cfg.Character.BlizzardSorceress.UseBlizzardPackets = r.Form.Has("blizzardUseBlizzardPackets")
		}

		// Sorceress Leveling specific options
		if cfg.Character.Class == "sorceress_leveling" {
			cfg.Character.SorceressLeveling.UseMoatTrick = r.Form.Has("levelingUseMoatTrick")
			cfg.Character.SorceressLeveling.UseStaticOnMephisto = r.Form.Has("levelingUseStaticOnMephisto")
			cfg.Character.SorceressLeveling.UseBlizzardPackets = r.Form.Has("levelingUseBlizzardPackets")
			cfg.Character.SorceressLeveling.UsePacketLearning = r.Form.Has("levelingUsePacketLearning")
		}

		// Assassin Leveling specific options
		if cfg.Character.Class == "assassin" {
			cfg.Character.AssassinLeveling.UsePacketLearning = r.Form.Has("usePacketLearning")
		}

		// Amazon Leveling specific options
		if cfg.Character.Class == "amazon_leveling" {
			cfg.Character.AmazonLeveling.UsePacketLearning = r.Form.Has("usePacketLearning")
		}

		// Druid Leveling specific options
		if cfg.Character.Class == "druid_leveling" {
			cfg.Character.DruidLeveling.UsePacketLearning = r.Form.Has("usePacketLearning")
		}

		// Necromancer Leveling specific options
		if cfg.Character.Class == "necromancer" {
			cfg.Character.NecromancerLeveling.UsePacketLearning = r.Form.Has("usePacketLearning")
		}

		// Paladin Leveling specific options
		if cfg.Character.Class == "paladin_leveling" {
			cfg.Character.PaladinLeveling.UsePacketLearning = r.Form.Has("usePacketLearning")
			levelingBuild := strings.ToLower(strings.TrimSpace(r.Form.Get("paladinLevelingBuild")))
			if levelingBuild != "foh" && levelingBuild != "hammer" {
				levelingBuild = "hammer"
			}
			cfg.Character.PaladinLeveling.LevelingBuild = levelingBuild
		}

		// Nova Sorceress specific options
		if cfg.Character.Class == "nova" {
			cfg.Character.NovaSorceress.AggressiveNovaPositioning = r.Form.Has("aggressiveNovaPositioning")
		}

		// Javazon specific options
		if cfg.Character.Class == "javazon" {
			cfg.Character.Javazon.DensityKillerEnabled = r.Form.Has("javazonDensityKillerEnabled")
			if v := r.Form.Get("javazonDensityKillerIgnoreWhitesBelow"); v != "" {
				if i, err := strconv.Atoi(v); err == nil {
					cfg.Character.Javazon.DensityKillerIgnoreWhitesBelow = i
				} else {
					cfg.Character.Javazon.DensityKillerIgnoreWhitesBelow = 4
				}
			} else if cfg.Character.Javazon.DensityKillerIgnoreWhitesBelow == 0 {
				cfg.Character.Javazon.DensityKillerIgnoreWhitesBelow = 4
			}
			if v := r.Form.Get("javazonDensityKillerForceRefillBelowPercent"); v != "" {
				if i, err := strconv.Atoi(v); err == nil {
					if i < 1 {
						i = 1
					}
					if i > 100 {
						i = 100
					}
					cfg.Character.Javazon.DensityKillerForceRefillBelowPercent = i
				} else {
					cfg.Character.Javazon.DensityKillerForceRefillBelowPercent = 50
				}
			} else if cfg.Character.Javazon.DensityKillerForceRefillBelowPercent == 0 {
				cfg.Character.Javazon.DensityKillerForceRefillBelowPercent = 50
			}
		}

		for y, row := range cfg.Inventory.InventoryLock {
			for x := range row {
				if r.Form.Has(fmt.Sprintf("inventoryLock[%d][%d]", y, x)) {
					cfg.Inventory.InventoryLock[y][x] = 0
				} else {
					cfg.Inventory.InventoryLock[y][x] = 1
				}
			}
		}

		copy(cfg.Inventory.BeltColumns[:], r.Form["inventoryBeltColumns[]"])

		cfg.Inventory.HealingPotionCount, _ = strconv.Atoi(r.Form.Get("healingPotionCount"))
		cfg.Inventory.ManaPotionCount, _ = strconv.Atoi(r.Form.Get("manaPotionCount"))
		cfg.Inventory.RejuvPotionCount, _ = strconv.Atoi(r.Form.Get("rejuvPotionCount"))

		// Game
		cfg.Game.CreateLobbyGames = r.Form.Has("createLobbyGames")
		cfg.Game.MinGoldPickupThreshold, _ = strconv.Atoi(r.Form.Get("gameMinGoldPickupThreshold"))
		cfg.UseCentralizedPickit = r.Form.Has("useCentralizedPickit")
		cfg.Game.UseCainIdentify = r.Form.Has("useCainIdentify")
		cfg.Game.DisableIdentifyTome = r.PostFormValue("game.disableIdentifyTome") == "on"
		cfg.Game.InteractWithShrines = r.Form.Has("interactWithShrines")
		cfg.Game.InteractWithChests = r.Form.Has("interactWithChests")
		cfg.Game.InteractWithSuperChests = r.Form.Has("interactWithSuperChests")
		cfg.Game.StopLevelingAt, _ = strconv.Atoi(r.Form.Get("stopLevelingAt"))
		cfg.Game.GameVersion = config.NormalizeGameVersion(r.Form.Get("gameVersion"))
		cfg.Game.IsNonLadderChar = r.Form.Has("isNonLadderChar")
		cfg.Game.IsHardCoreChar = r.Form.Has("isHardCoreChar")

		if v := r.Form.Get("maxGameLength"); v != "" {
			cfg.MaxGameLength, _ = strconv.Atoi(v)
		}

		// Packet Casting
		cfg.PacketCasting.UseForEntranceInteraction = r.Form.Has("packetCastingUseForEntranceInteraction")
		cfg.PacketCasting.UseForItemPickup = r.Form.Has("packetCastingUseForItemPickup")
		cfg.PacketCasting.UseForTpInteraction = r.Form.Has("packetCastingUseForTpInteraction")
		cfg.PacketCasting.UseForTeleport = r.Form.Has("packetCastingUseForTeleport")
		cfg.PacketCasting.UseForEntitySkills = r.Form.Has("packetCastingUseForEntitySkills")
		cfg.PacketCasting.UseForSkillSelection = r.Form.Has("packetCastingUseForSkillSelection")
		cfg.Game.Difficulty = difficulty.Difficulty(r.Form.Get("gameDifficulty"))
		cfg.Game.RandomizeRuns = r.Form.Has("gameRandomizeRuns")

		// Runs specific config
		enabledRuns := make([]config.Run, 0)

		// we don't like errors, so we ignore them
		json.Unmarshal([]byte(r.FormValue("gameRuns")), &enabledRuns)
		cfg.Game.Runs = enabledRuns

		s.applyShoppingFromForm(r.Form, cfg)

		cfg.Game.Cows.OpenChests = r.Form.Has("gameCowsOpenChests")

		cfg.Game.Pit.MoveThroughBlackMarsh = r.Form.Has("gamePitMoveThroughBlackMarsh")
		cfg.Game.Pit.OpenChests = r.Form.Has("gamePitOpenChests")
		cfg.Game.Pit.FocusOnElitePacks = r.Form.Has("gamePitFocusOnElitePacks")
		cfg.Game.Pit.OnlyClearLevel2 = r.Form.Has("gamePitOnlyClearLevel2")

		cfg.Game.Andariel.ClearRoom = r.Form.Has("gameAndarielClearRoom")
		cfg.Game.Andariel.UseAntidotes = r.Form.Has("gameAndarielUseAntidotes")

		cfg.Game.Countess.ClearFloors = r.Form.Has("gameCountessClearFloors")

		cfg.Game.Pindleskin.SkipOnImmunities = []stat.Resist{}
		for _, i := range r.Form["gamePindleskinSkipOnImmunities[]"] {
			cfg.Game.Pindleskin.SkipOnImmunities = append(cfg.Game.Pindleskin.SkipOnImmunities, stat.Resist(i))
		}

		cfg.Game.StonyTomb.OpenChests = r.Form.Has("gameStonytombOpenChests")
		cfg.Game.StonyTomb.FocusOnElitePacks = r.Form.Has("gameStonytombFocusOnElitePacks")

		cfg.Game.AncientTunnels.OpenChests = r.Form.Has("gameAncientTunnelsOpenChests")
		cfg.Game.AncientTunnels.FocusOnElitePacks = r.Form.Has("gameAncientTunnelsFocusOnElitePacks")

		cfg.Game.Duriel.UseThawing = r.Form.Has("gameDurielUseThawing")

		cfg.Game.Mausoleum.OpenChests = r.Form.Has("gameMausoleumOpenChests")
		cfg.Game.Mausoleum.FocusOnElitePacks = r.Form.Has("gameMausoleumFocusOnElitePacks")

		cfg.Game.DrifterCavern.OpenChests = r.Form.Has("gameDrifterCavernOpenChests")
		cfg.Game.DrifterCavern.FocusOnElitePacks = r.Form.Has("gameDrifterCavernFocusOnElitePacks")

		cfg.Game.SpiderCavern.OpenChests = r.Form.Has("gameSpiderCavernOpenChests")
		cfg.Game.SpiderCavern.FocusOnElitePacks = r.Form.Has("gameSpiderCavernFocusOnElitePacks")

		cfg.Game.ArachnidLair.OpenChests = r.Form.Has("gameArachnidLairOpenChests")
		cfg.Game.ArachnidLair.FocusOnElitePacks = r.Form.Has("gameArachnidLairFocusOnElitePacks")

		cfg.Game.Mephisto.KillCouncilMembers = r.Form.Has("gameMephistoKillCouncilMembers")
		cfg.Game.Mephisto.OpenChests = r.Form.Has("gameMephistoOpenChests")
		cfg.Game.Mephisto.ExitToA4 = r.Form.Has("gameMephistoExitToA4")

		cfg.Game.Tristram.ClearPortal = r.Form.Has("gameTristramClearPortal")
		cfg.Game.Tristram.FocusOnElitePacks = r.Form.Has("gameTristramFocusOnElitePacks")
		cfg.Game.Tristram.OnlyFarmRejuvs = r.Form.Has("gameTristramOnlyFarmRejuvs")

		cfg.Game.Nihlathak.ClearArea = r.Form.Has("gameNihlathakClearArea")
		cfg.Game.Summoner.KillFireEye = r.Form.Has("gameSummonerKillFireEye")

		cfg.Game.Baal.KillBaal = r.Form.Has("gameBaalKillBaal")
		cfg.Game.Baal.DollQuit = r.Form.Has("gameBaalDollQuit")
		cfg.Game.Baal.SoulQuit = r.Form.Has("gameBaalSoulQuit")
		cfg.Game.Baal.ClearFloors = r.Form.Has("gameBaalClearFloors")
		cfg.Game.Baal.OnlyElites = r.Form.Has("gameBaalOnlyElites")

		cfg.Game.Eldritch.KillShenk = r.Form.Has("gameEldritchKillShenk")

		cfg.Game.LowerKurastChest.OpenRacks = r.Form.Has("gameLowerKurastChestOpenRacks")

		cfg.Game.Diablo.StartFromStar = r.Form.Has("gameDiabloStartFromStar")
		cfg.Game.Diablo.KillDiablo = r.Form.Has("gameDiabloKillDiablo")
		cfg.Game.Diablo.FocusOnElitePacks = r.Form.Has("gameDiabloFocusOnElitePacks")
		cfg.Game.Diablo.DisableItemPickupDuringBosses = r.Form.Has("gameDiabloDisableItemPickupDuringBosses")
		cfg.Game.Diablo.AttackFromDistance = s.getIntFromForm(r, "gameLevelingHellRequiredFireRes", 0, 25, 0)
		cfg.Game.Leveling.EnsurePointsAllocation = r.Form.Has("gameLevelingEnsurePointsAllocation")
		cfg.Game.Leveling.EnsureKeyBinding = r.Form.Has("gameLevelingEnsureKeyBinding")
		cfg.Game.Leveling.AutoEquip = r.Form.Has("gameLevelingAutoEquip")
		cfg.Game.Leveling.AutoEquipFromSharedStash = r.Form.Has("gameLevelingAutoEquipFromSharedStash")
		cfg.Game.Leveling.NightmareRequiredLevel = s.getIntFromForm(r, "gameLevelingNightmareRequiredLevel", 1, 99, 41)
		cfg.Game.Leveling.HellRequiredLevel = s.getIntFromForm(r, "gameLevelingHellRequiredLevel", 1, 99, 70)
		cfg.Game.Leveling.HellRequiredFireRes = s.getIntFromForm(r, "gameLevelingHellRequiredFireRes", -100, 75, 15)
		cfg.Game.Leveling.HellRequiredLightRes = s.getIntFromForm(r, "gameLevelingHellRequiredLightRes", -100, 75, -10)

		cfg.Game.LevelingSequence.SequenceFile = r.Form.Get("gameLevelingSequenceFile")

		// Quests options for Act 1
		cfg.Game.Quests.ClearDen = r.Form.Has("gameQuestsClearDen")
		cfg.Game.Quests.RescueCain = r.Form.Has("gameQuestsRescueCain")
		cfg.Game.Quests.RetrieveHammer = r.Form.Has("gameQuestsRetrieveHammer")
		// Quests options for Act 2
		cfg.Game.Quests.KillRadament = r.Form.Has("gameQuestsKillRadament")
		cfg.Game.Quests.GetCube = r.Form.Has("gameQuestsGetCube")
		// Quests options for Act 3
		cfg.Game.Quests.RetrieveBook = r.Form.Has("gameQuestsRetrieveBook")
		// Quests options for Act 4
		cfg.Game.Quests.KillIzual = r.Form.Has("gameQuestsKillIzual")
		// Quests options for Act 5
		cfg.Game.Quests.KillShenk = r.Form.Has("gameQuestsKillShenk")
		cfg.Game.Quests.RescueAnya = r.Form.Has("gameQuestsRescueAnya")
		cfg.Game.Quests.KillAncients = r.Form.Has("gameQuestsKillAncients")

		cfg.Game.TerrorZone.FocusOnElitePacks = r.Form.Has("gameTerrorZoneFocusOnElitePacks")
		cfg.Game.TerrorZone.SkipOtherRuns = r.Form.Has("gameTerrorZoneSkipOtherRuns")
		cfg.Game.TerrorZone.OpenChests = r.Form.Has("gameTerrorZoneOpenChests")

		cfg.Game.TerrorZone.SkipOnImmunities = []stat.Resist{}
		for _, i := range r.Form["gameTerrorZoneSkipOnImmunities[]"] {
			cfg.Game.TerrorZone.SkipOnImmunities = append(cfg.Game.TerrorZone.SkipOnImmunities, stat.Resist(i))
		}

		tzAreas := make([]area.ID, 0)
		for _, a := range r.Form["gameTerrorZoneAreas[]"] {
			ID, _ := strconv.Atoi(a)
			tzAreas = append(tzAreas, area.ID(ID))
		}
		cfg.Game.TerrorZone.Areas = tzAreas

		// Utility
		if parkingActStr := r.Form.Get("gameUtilityParkingAct"); parkingActStr != "" {
			if parkingAct, err := strconv.Atoi(parkingActStr); err == nil {
				cfg.Game.Utility.ParkingAct = parkingAct
			}
		}

		// Gambling
		cfg.Gambling.Enabled = r.Form.Has("gamblingEnabled")
		if raw := strings.TrimSpace(r.Form.Get("gamblingItems")); raw != "" {
			parts := strings.Split(raw, ",")
			items := make([]string, 0, len(parts))
			for _, p := range parts {
				if p = strings.TrimSpace(p); p != "" {
					items = append(items, p)
				}
			}
			cfg.Gambling.Items = items
		} else {
			cfg.Gambling.Items = []string{}
		}

		// Cube Recipes
		cfg.CubeRecipes.Enabled = r.Form.Has("enableCubeRecipes")
		enabledRecipes := r.Form["enabledRecipes"]
		cfg.CubeRecipes.EnabledRecipes = enabledRecipes
		cfg.CubeRecipes.SkipPerfectAmethysts = r.Form.Has("skipPerfectAmethysts")
		cfg.CubeRecipes.SkipPerfectRubies = r.Form.Has("skipPerfectRubies")
		// New: parse jewelsToKeep
		if v := r.Form.Get("jewelsToKeep"); v != "" {
			if n, err := strconv.Atoi(v); err == nil && n >= 0 {
				cfg.CubeRecipes.JewelsToKeep = n
			} else {
				cfg.CubeRecipes.JewelsToKeep = 1 // sensible default
			}
		}
		// Companion config
		cfg.Companion.Enabled = r.Form.Has("companionEnabled")
		cfg.Companion.Leader = r.Form.Has("companionLeader")
		cfg.Companion.LeaderName = r.Form.Get("companionLeaderName")
		cfg.Companion.GameNameTemplate = r.Form.Get("companionGameNameTemplate")
		cfg.Companion.GamePassword = r.Form.Get("companionGamePassword")

		// Back to town config
		cfg.BackToTown.NoHpPotions = r.Form.Has("noHpPotions")
		cfg.BackToTown.NoMpPotions = r.Form.Has("noMpPotions")
		cfg.BackToTown.MercDied = r.Form.Has("mercDied")
		cfg.BackToTown.EquipmentBroken = r.Form.Has("equipmentBroken")

		// Scheduler
		cfg.Scheduler.Enabled = r.Form.Has("schedulerEnabled")
		cfg.Scheduler.Mode = r.Form.Get("schedulerMode")
		if cfg.Scheduler.Mode == "" {
			cfg.Scheduler.Mode = "timeSlots"
		}

		// Global variance for time slots mode
		if v := r.Form.Get("globalVarianceMin"); v != "" {
			cfg.Scheduler.GlobalVarianceMin, _ = strconv.Atoi(v)
		}

		// Reset scheduler days if we are updating them
		if len(cfg.Scheduler.Days) != 7 {
			cfg.Scheduler.Days = make([]config.Day, 7)
		}

		// Parse time slots mode data
		for day := 0; day < 7; day++ {
			starts := r.Form[fmt.Sprintf("scheduler[%d][start][]", day)]
			ends := r.Form[fmt.Sprintf("scheduler[%d][end][]", day)]
			startVars := r.Form[fmt.Sprintf("scheduler[%d][startVar][]", day)]
			endVars := r.Form[fmt.Sprintf("scheduler[%d][endVar][]", day)]

			cfg.Scheduler.Days[day].DayOfWeek = day
			cfg.Scheduler.Days[day].TimeRanges = make([]config.TimeRange, 0)

			for i := 0; i < len(starts); i++ {
				start, err := time.Parse("15:04", starts[i])
				if err != nil {
					continue
				}
				end, err := time.Parse("15:04", ends[i])
				if err != nil {
					continue
				}

				var startVar, endVar int
				if i < len(startVars) {
					startVar, _ = strconv.Atoi(startVars[i])
				}
				if i < len(endVars) {
					endVar, _ = strconv.Atoi(endVars[i])
				}

				cfg.Scheduler.Days[day].TimeRanges = append(cfg.Scheduler.Days[day].TimeRanges, config.TimeRange{
					Start:            start,
					End:              end,
					StartVarianceMin: startVar,
					EndVarianceMin:   endVar,
				})
			}
		}

		// Parse duration mode data
		cfg.Scheduler.Duration.WakeUpTime = r.Form.Get("durationWakeUpTime")
		if v := r.Form.Get("durationWakeUpVariance"); v != "" {
			cfg.Scheduler.Duration.WakeUpVariance, _ = strconv.Atoi(v)
		}
		if v := r.Form.Get("durationPlayHours"); v != "" {
			cfg.Scheduler.Duration.PlayHours, _ = strconv.Atoi(v)
		}
		if v := r.Form.Get("durationPlayHoursVariance"); v != "" {
			cfg.Scheduler.Duration.PlayHoursVariance, _ = strconv.Atoi(v)
		}
		if v := r.Form.Get("durationMealBreakCount"); v != "" {
			cfg.Scheduler.Duration.MealBreakCount, _ = strconv.Atoi(v)
		}
		if v := r.Form.Get("durationMealBreakDuration"); v != "" {
			cfg.Scheduler.Duration.MealBreakDuration, _ = strconv.Atoi(v)
		}
		if v := r.Form.Get("durationMealBreakVariance"); v != "" {
			cfg.Scheduler.Duration.MealBreakVariance, _ = strconv.Atoi(v)
		}
		if v := r.Form.Get("durationShortBreakCount"); v != "" {
			cfg.Scheduler.Duration.ShortBreakCount, _ = strconv.Atoi(v)
		}
		if v := r.Form.Get("durationShortBreakDuration"); v != "" {
			cfg.Scheduler.Duration.ShortBreakDuration, _ = strconv.Atoi(v)
		}
		if v := r.Form.Get("durationShortBreakVariance"); v != "" {
			cfg.Scheduler.Duration.ShortBreakVariance, _ = strconv.Atoi(v)
		}
		if v := r.Form.Get("durationBreakTimingVariance"); v != "" {
			cfg.Scheduler.Duration.BreakTimingVariance, _ = strconv.Atoi(v)
		}
		if v := r.Form.Get("durationJitterMin"); v != "" {
			cfg.Scheduler.Duration.JitterMin, _ = strconv.Atoi(v)
		}
		if v := r.Form.Get("durationJitterMax"); v != "" {
			cfg.Scheduler.Duration.JitterMax, _ = strconv.Atoi(v)
		}

		// Muling
		cfg.Muling.Enabled = r.FormValue("mulingEnabled") == "on"

		// Validate mule profiles - filter out any deleted mule profiles
		requestedMuleProfiles := r.Form["mulingMuleProfiles[]"]
		validMuleProfiles := []string{}
		allCharacters := config.GetCharacters()
		for _, muleName := range requestedMuleProfiles {
			if muleCfg, exists := allCharacters[muleName]; exists && strings.ToLower(muleCfg.Character.Class) == "mule" {
				validMuleProfiles = append(validMuleProfiles, muleName)
			}
		}
		cfg.Muling.MuleProfiles = validMuleProfiles

		cfg.Muling.ReturnTo = r.FormValue("mulingReturnTo")
		favoriteRuns := sanitizeFavoriteRunSelection(r.Form["runFavoriteRuns"])
		config.Koolo.RunFavoriteRuns = favoriteRuns
		if err := config.SaveKooloConfig(config.Koolo); err != nil {
			s.logger.Error("Failed to save run favorites", slog.Any("error", err))
		}
		config.SaveSupervisorConfig(supervisorName, cfg)
		http.Redirect(w, r, "/", http.StatusSeeOther)
		return
	}

	supervisor := r.URL.Query().Get("supervisor")
	cloneSource := ""
	cloneParam := r.URL.Query().Get("clone")
	cfg, _ := config.GetCharacter("template")
	if supervisor != "" {
		if cfgLoaded, ok := config.GetCharacter(supervisor); ok && cfgLoaded != nil {
			cfg = cfgLoaded
		}
		cloneParam = ""
	} else if cloneParam != "" {
		if cfgLoaded, ok := config.GetCharacter(cloneParam); ok && cfgLoaded != nil {
			tmp := *cfgLoaded
			cfg = &tmp
			cloneSource = cloneParam
		}
	}
	skillOptions := buildSkillOptionsForBuild(cfg.Character.Class)

	enabledRuns := make([]string, 0)
	for _, run := range cfg.Game.Runs {
		if run == config.UberIzualRun || run == config.UberDurielRun || run == config.LilithRun {
			continue
		}
		enabledRuns = append(enabledRuns, string(run))
	}
	disabledRuns := make([]string, 0)
	for run := range config.AvailableRuns {
		if run == config.UberIzualRun || run == config.UberDurielRun || run == config.LilithRun {
			continue
		}
		if !slices.Contains(cfg.Game.Runs, run) {
			disabledRuns = append(disabledRuns, string(run))
		}
	}
	sort.Strings(disabledRuns)

	if len(cfg.Scheduler.Days) == 0 {
		cfg.Scheduler.Days = make([]config.Day, 7)
		for i := 0; i < 7; i++ {
			cfg.Scheduler.Days[i] = config.Day{DayOfWeek: i}
		}
	}

	dayNames := []string{"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"}

	// Get list of mule profiles (for farmer's mule dropdown)
	// and farmer profiles (for mule's return character dropdown)
	muleProfiles := []string{}
	farmerProfiles := []string{}
	allCharacters := config.GetCharacters()
	supervisors := make([]string, 0, len(allCharacters))
	for profileName, profileCfg := range allCharacters {
		if profileName == "template" {
			continue
		}
		supervisors = append(supervisors, profileName)
		if strings.ToLower(profileCfg.Character.Class) == "mule" {
			muleProfiles = append(muleProfiles, profileName)
		} else {
			farmerProfiles = append(farmerProfiles, profileName)
		}
	}
	sort.Strings(supervisors)
	sort.Strings(muleProfiles)
	sort.Strings(farmerProfiles)

	// Filter out any invalid mule profiles from the config before rendering
	// This prevents form validation errors when deleted mules are still referenced
	validConfigMuleProfiles := []string{}
	for _, muleName := range cfg.Muling.MuleProfiles {
		if muleCfg, exists := allCharacters[muleName]; exists && strings.ToLower(muleCfg.Character.Class) == "mule" {
			validConfigMuleProfiles = append(validConfigMuleProfiles, muleName)
		}
	}
	cfg.Muling.MuleProfiles = validConfigMuleProfiles

	s.templates.ExecuteTemplate(w, "character_settings.gohtml", CharacterSettings{
		Version:               config.Version,
		Supervisor:            supervisor,
		CloneSource:           cloneSource,
		Config:                cfg,
		SkillOptions:          skillOptions,
		SkillPrereqs:          buildSkillPrereqsForBuild(cfg.Character.Class),
		DayNames:              dayNames,
		EnabledRuns:           enabledRuns,
		DisabledRuns:          disabledRuns,
		TerrorZoneGroups:      buildTZGroups(),
		RecipeList:            config.AvailableRecipes,
		RunewordRecipeList:    availableRunewordRecipesForCharacter(cfg),
		RunFavoriteRuns:       config.Koolo.RunFavoriteRuns,
		AvailableProfiles:     muleProfiles,
		FarmerProfiles:        farmerProfiles,
		LevelingSequenceFiles: sequenceFiles,
		Supervisors:           supervisors,
	})
}

func (s *HttpServer) listLevelingSequenceFiles() []string {
	if s.sequenceAPI == nil {
		return nil
	}
	files, err := s.sequenceAPI.ListSequenceFiles()
	if err != nil {
		s.logger.Error("failed to list leveling sequences", slog.Any("error", err))
		return nil
	}
	return files
}

// companionJoin handles requests to force a companion to join a game
func (s *HttpServer) companionJoin(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var requestData struct {
		Supervisor string `json:"supervisor"`
		GameName   string `json:"gameName"`
		Password   string `json:"password"`
	}

	err := json.NewDecoder(r.Body).Decode(&requestData)
	if err != nil {
		http.Error(w, "Invalid request data", http.StatusBadRequest)
		return
	}

	cfg, found := config.GetCharacter(requestData.Supervisor)
	if !found {
		http.Error(w, "Supervisor not found", http.StatusNotFound)
		return
	}

	if !cfg.Companion.Enabled || cfg.Companion.Leader {
		http.Error(w, "Supervisor is not a companion follower", http.StatusBadRequest)
		return
	}

	baseEvent := event.Text(requestData.Supervisor, fmt.Sprintf("Manual request to join game %s", requestData.GameName))
	joinEvent := event.RequestCompanionJoinGame(baseEvent, cfg.CharacterName, requestData.GameName, requestData.Password)

	event.Send(joinEvent)

	s.logger.Info("Manual companion join request sent",
		slog.String("supervisor", requestData.Supervisor),
		slog.String("game", requestData.GameName))

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]bool{"success": true})
}

// applyShoppingFromForm parses shopping-specific fields (used in updateConfigFromForm)
func (s *HttpServer) applyShoppingFromForm(values url.Values, cfg *config.CharacterCfg) {
	cfg.Shopping.Enabled = values.Has("shoppingEnabled")

	if v, err := strconv.Atoi(values.Get("shoppingMaxGoldToSpend")); err == nil {
		cfg.Shopping.MaxGoldToSpend = v
	}
	if v, err := strconv.Atoi(values.Get("shoppingMinGoldReserve")); err == nil {
		cfg.Shopping.MinGoldReserve = v
	}
	if v, err := strconv.Atoi(values.Get("shoppingRefreshesPerRun")); err == nil {
		cfg.Shopping.RefreshesPerRun = v
	}

	cfg.Shopping.ShoppingRulesFile = values.Get("shoppingRulesFile")

	if raw := strings.TrimSpace(values.Get("shoppingItemTypes")); raw != "" {
		parts := strings.Split(raw, ",")
		items := make([]string, 0, len(parts))
		for _, p := range parts {
			if p = strings.TrimSpace(p); p != "" {
				items = append(items, p)
			}
		}
		cfg.Shopping.ItemTypes = items
	} else {
		cfg.Shopping.ItemTypes = []string{}
	}

	cfg.Shopping.VendorAkara = values.Has("shoppingVendorAkara")
	cfg.Shopping.VendorCharsi = values.Has("shoppingVendorCharsi")
	cfg.Shopping.VendorGheed = values.Has("shoppingVendorGheed")
	cfg.Shopping.VendorFara = values.Has("shoppingVendorFara")
	cfg.Shopping.VendorDrognan = values.Has("shoppingVendorDrognan")
	cfg.Shopping.VendorElzix = values.Has("shoppingVendorElzix")
	cfg.Shopping.VendorOrmus = values.Has("shoppingVendorOrmus")
	cfg.Shopping.VendorMalah = values.Has("shoppingVendorMalah")
	cfg.Shopping.VendorAnya = values.Has("shoppingVendorAnya")
}

func (s *HttpServer) applyRunDetails(values url.Values, cfg *config.CharacterCfg, runs []string) {
	for _, runID := range runs {
		switch runID {
		case "andariel":
			cfg.Game.Andariel.ClearRoom = values.Has("gameAndarielClearRoom")
			cfg.Game.Andariel.UseAntidotes = values.Has("gameAndarielUseAntidotes")
		case "countess":
			cfg.Game.Countess.ClearFloors = values.Has("gameCountessClearFloors")
		case "duriel":
			cfg.Game.Duriel.UseThawing = values.Has("gameDurielUseThawing")
		case "pit":
			cfg.Game.Pit.MoveThroughBlackMarsh = values.Has("gamePitMoveThroughBlackMarsh")
			cfg.Game.Pit.OpenChests = values.Has("gamePitOpenChests")
			cfg.Game.Pit.FocusOnElitePacks = values.Has("gamePitFocusOnElitePacks")
			cfg.Game.Pit.OnlyClearLevel2 = values.Has("gamePitOnlyClearLevel2")
		case "cows":
			cfg.Game.Cows.OpenChests = values.Has("gameCowsOpenChests")
		case "pindleskin":
			if raw, ok := values["gamePindleskinSkipOnImmunities[]"]; ok {
				skips := make([]stat.Resist, 0, len(raw))
				for _, v := range raw {
					if v == "" {
						continue
					}
					skips = append(skips, stat.Resist(v))
				}
				cfg.Game.Pindleskin.SkipOnImmunities = skips
			} else {
				cfg.Game.Pindleskin.SkipOnImmunities = nil
			}
		case "stony_tomb":
			cfg.Game.StonyTomb.OpenChests = values.Has("gameStonytombOpenChests")
			cfg.Game.StonyTomb.FocusOnElitePacks = values.Has("gameStonytombFocusOnElitePacks")
		case "mausoleum":
			cfg.Game.Mausoleum.OpenChests = values.Has("gameMausoleumOpenChests")
			cfg.Game.Mausoleum.FocusOnElitePacks = values.Has("gameMausoleumFocusOnElitePacks")
		case "ancient_tunnels":
			cfg.Game.AncientTunnels.OpenChests = values.Has("gameAncientTunnelsOpenChests")
			cfg.Game.AncientTunnels.FocusOnElitePacks = values.Has("gameAncientTunnelsFocusOnElitePacks")
		case "drifter_cavern":
			cfg.Game.DrifterCavern.OpenChests = values.Has("gameDrifterCavernOpenChests")
			cfg.Game.DrifterCavern.FocusOnElitePacks = values.Has("gameDrifterCavernFocusOnElitePacks")
		case "spider_cavern":
			cfg.Game.SpiderCavern.OpenChests = values.Has("gameSpiderCavernOpenChests")
			cfg.Game.SpiderCavern.FocusOnElitePacks = values.Has("gameSpiderCavernFocusOnElitePacks")
		case "arachnid_lair":
			cfg.Game.ArachnidLair.OpenChests = values.Has("gameArachnidLairOpenChests")
			cfg.Game.ArachnidLair.FocusOnElitePacks = values.Has("gameArachnidLairFocusOnElitePacks")
		case "mephisto":
			cfg.Game.Mephisto.KillCouncilMembers = values.Has("gameMephistoKillCouncilMembers")
			cfg.Game.Mephisto.OpenChests = values.Has("gameMephistoOpenChests")
			cfg.Game.Mephisto.ExitToA4 = values.Has("gameMephistoExitToA4")
		case "tristram":
			cfg.Game.Tristram.ClearPortal = values.Has("gameTristramClearPortal")
			cfg.Game.Tristram.FocusOnElitePacks = values.Has("gameTristramFocusOnElitePacks")
			cfg.Game.Tristram.OnlyFarmRejuvs = values.Has("gameTristramOnlyFarmRejuvs")
		case "nihlathak":
			cfg.Game.Nihlathak.ClearArea = values.Has("gameNihlathakClearArea")
		case "summoner":
			cfg.Game.Summoner.KillFireEye = values.Has("gameSummonerKillFireEye")
		case "baal":
			cfg.Game.Baal.KillBaal = values.Has("gameBaalKillBaal")
			cfg.Game.Baal.DollQuit = values.Has("gameBaalDollQuit")
			cfg.Game.Baal.SoulQuit = values.Has("gameBaalSoulQuit")
			cfg.Game.Baal.ClearFloors = values.Has("gameBaalClearFloors")
			cfg.Game.Baal.OnlyElites = values.Has("gameBaalOnlyElites")
		case "eldritch":
			cfg.Game.Eldritch.KillShenk = values.Has("gameEldritchKillShenk")
		case "lower_kurast_chest":
			cfg.Game.LowerKurastChest.OpenRacks = values.Has("gameLowerKurastChestOpenRacks")
		case "diablo":
			cfg.Game.Diablo.KillDiablo = values.Has("gameDiabloKillDiablo")
			cfg.Game.Diablo.DisableItemPickupDuringBosses = values.Has("gameDiabloDisableItemPickupDuringBosses")
			cfg.Game.Diablo.StartFromStar = values.Has("gameDiabloStartFromStar")
			cfg.Game.Diablo.FocusOnElitePacks = values.Has("gameDiabloFocusOnElitePacks")
			if v := values.Get("gameDiabloAttackFromDistance"); v != "" {
				if n, err := strconv.Atoi(v); err == nil {
					if n < 0 {
						n = 0
					} else if n > 25 {
						n = 25
					}
					cfg.Game.Diablo.AttackFromDistance = n
				}
			}
		case "leveling":
			cfg.Game.Leveling.EnsurePointsAllocation = values.Has("gameLevelingEnsurePointsAllocation")
			cfg.Game.Leveling.EnsureKeyBinding = values.Has("gameLevelingEnsureKeyBinding")
			cfg.Game.Leveling.AutoEquip = values.Has("gameLevelingAutoEquip")
			cfg.Game.Leveling.AutoEquipFromSharedStash = values.Has("gameLevelingAutoEquipFromSharedStash")
			if v := values.Get("gameLevelingNightmareRequiredLevel"); v != "" {
				if n, err := strconv.Atoi(v); err == nil {
					if n < 0 {
						n = 0
					} else if n > 99 {
						n = 99
					}
					cfg.Game.Leveling.NightmareRequiredLevel = n
				}
			}
			if v := values.Get("gameLevelingHellRequiredLevel"); v != "" {
				if n, err := strconv.Atoi(v); err == nil {
					if n < 0 {
						n = 0
					} else if n > 99 {
						n = 99
					}
					cfg.Game.Leveling.HellRequiredLevel = n
				}
			}
			if v := values.Get("gameLevelingHellRequiredFireRes"); v != "" {
				if n, err := strconv.Atoi(v); err == nil {
					if n < -100 {
						n = -100
					} else if n > 75 {
						n = 75
					}
					cfg.Game.Leveling.HellRequiredFireRes = n
				}
			}
			if v := values.Get("gameLevelingHellRequiredLightRes"); v != "" {
				if n, err := strconv.Atoi(v); err == nil {
					if n < -100 {
						n = -100
					} else if n > 75 {
						n = 75
					}
					cfg.Game.Leveling.HellRequiredLightRes = n
				}
			}
		case "leveling_sequence":
			cfg.Game.LevelingSequence.SequenceFile = values.Get("gameLevelingSequenceFile")
		case "quests":
			cfg.Game.Quests.ClearDen = values.Has("gameQuestsClearDen")
			cfg.Game.Quests.RescueCain = values.Has("gameQuestsRescueCain")
			cfg.Game.Quests.RetrieveHammer = values.Has("gameQuestsRetrieveHammer")
			cfg.Game.Quests.KillRadament = values.Has("gameQuestsKillRadament")
			cfg.Game.Quests.GetCube = values.Has("gameQuestsGetCube")
			cfg.Game.Quests.RetrieveBook = values.Has("gameQuestsRetrieveBook")
			cfg.Game.Quests.KillIzual = values.Has("gameQuestsKillIzual")
			cfg.Game.Quests.KillShenk = values.Has("gameQuestsKillShenk")
			cfg.Game.Quests.RescueAnya = values.Has("gameQuestsRescueAnya")
			cfg.Game.Quests.KillAncients = values.Has("gameQuestsKillAncients")
		case "terror_zone":
			cfg.Game.TerrorZone.FocusOnElitePacks = values.Has("gameTerrorZoneFocusOnElitePacks")
			cfg.Game.TerrorZone.SkipOtherRuns = values.Has("gameTerrorZoneSkipOtherRuns")
			cfg.Game.TerrorZone.OpenChests = values.Has("gameTerrorZoneOpenChests")

			if raw, ok := values["gameTerrorZoneSkipOnImmunities[]"]; ok {
				skips := make([]stat.Resist, 0, len(raw))
				for _, v := range raw {
					if v == "" {
						continue
					}
					skips = append(skips, stat.Resist(v))
				}
				cfg.Game.TerrorZone.SkipOnImmunities = skips
			} else {
				cfg.Game.TerrorZone.SkipOnImmunities = nil
			}

			if raw, ok := values["gameTerrorZoneAreas[]"]; ok {
				areas := make([]area.ID, 0, len(raw))
				for _, v := range raw {
					if v == "" {
						continue
					}
					if id, err := strconv.Atoi(v); err == nil {
						areas = append(areas, area.ID(id))
					}
				}
				cfg.Game.TerrorZone.Areas = areas
			} else {
				cfg.Game.TerrorZone.Areas = nil
			}
		case "utility":
			if v := values.Get("gameUtilityParkingAct"); v != "" {
				if n, err := strconv.Atoi(v); err == nil {
					cfg.Game.Utility.ParkingAct = n
				}
			}
		case "shopping":
			// Handled in applyShoppingFromForm, repeated here just for run detail completeness if needed
		}
	}
}

func (s *HttpServer) bulkApplyCharacterSettings(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req struct {
		SourceSupervisor  string              `json:"sourceSupervisor"`
		TargetSupervisors []string            `json:"targetSupervisors"`
		Sections          ConfigUpdateOptions `json:"sections"` // Use the shared struct
		RunDetailTargets  []string            `json:"runDetailTargets"`
		Form              map[string][]string `json:"form"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "invalid payload", http.StatusBadRequest)
		return
	}

	targets := map[string]struct{}{}
	if req.SourceSupervisor != "" {
		targets[req.SourceSupervisor] = struct{}{}
	}
	for _, t := range req.TargetSupervisors {
		if t == "" {
			continue
		}
		targets[t] = struct{}{}
	}

	if len(targets) == 0 {
		http.Error(w, "no supervisors specified", http.StatusBadRequest)
		return
	}

	// Convert JSON Form map to url.Values
	values := url.Values{}
	for k, arr := range req.Form {
		for _, v := range arr {
			values.Add(k, v)
		}
	}

	// Save source supervisor's form data first so bulk apply uses current form state (not saved state)
	// This fixes UX issue where bulk apply button is next to save, but would apply old saved values
	if req.SourceSupervisor != "" {
		if sourceCfg, found := config.GetCharacter(req.SourceSupervisor); found && sourceCfg != nil {
			if err := s.updateConfigFromForm(values, sourceCfg, req.Sections, req.RunDetailTargets); err == nil {
				if err := config.SaveSupervisorConfig(req.SourceSupervisor, sourceCfg); err != nil {
					s.logger.Warn("failed to save source supervisor config", slog.String("supervisor", req.SourceSupervisor), slog.Any("error", err))
				}
			}
		}
	}

	// Now get the runeword source from the freshly saved config
	var runewordSource *config.CharacterCfg
	if req.Sections.RunewordMaker && req.SourceSupervisor != "" {
		if src, ok := config.GetCharacter(req.SourceSupervisor); ok && src != nil {
			if cloned, err := cloneCharacterCfg(src); err == nil {
				runewordSource = cloned
			} else {
				s.logger.Warn("failed to clone runeword settings", slog.String("supervisor", req.SourceSupervisor), slog.Any("error", err))
			}
		}
	}

	for name := range targets {
		cfg, found := config.GetCharacter(name)
		if !found || cfg == nil {
			continue
		}

		if req.Sections.RunewordMaker && runewordSource != nil {
			applyRunewordSettings(cfg, runewordSource)
		}

		// Ensure Identity, Muling, Shopping are NOT applied by default in Bulk Apply unless specified
		// The client currently sends `ConfigUpdateOptions` which matches the JS struct.
		// Muling/Shopping keys might be missing in JS struct, defaulting to false (which is safe).
		// UpdateAllRunDetails is defaulted to false for Bulk Apply (desired).

		if err := s.updateConfigFromForm(values, cfg, req.Sections, req.RunDetailTargets); err != nil {
			s.logger.Error("failed to apply config", slog.String("supervisor", name), slog.Any("error", err))
			continue
		}

		if err := config.SaveSupervisorConfig(name, cfg); err != nil {
			s.logger.Error("failed to save bulk-applied config", slog.String("supervisor", name), slog.Any("error", err))
			continue
		}
	}

	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(map[string]any{"success": true})
}

func (s *HttpServer) resetMuling(w http.ResponseWriter, r *http.Request) {
	characterName := r.URL.Query().Get("characterName")
	if characterName == "" {
		http.Error(w, "Character name is required", http.StatusBadRequest)
		return
	}

	cfg, found := config.GetCharacter(characterName)
	if !found {
		http.Error(w, "Character config not found", http.StatusNotFound)
		return
	}

	s.logger.Info("Resetting muling index for character", "character", characterName)
	cfg.MulingState.CurrentMuleIndex = 0

	err := config.SaveSupervisorConfig(characterName, cfg)
	if err != nil {
		http.Error(w, "Failed to save updated config", http.StatusInternalServerError)
		return
	}

	w.WriteHeader(http.StatusOK)
}

func (s *HttpServer) skillOptionsAPI(w http.ResponseWriter, r *http.Request) {
	build := r.URL.Query().Get("build")
	payload := struct {
		Options  []SkillOption       `json:"options"`
		Prereqs  map[string][]string `json:"prereqs"`
		Resolved string              `json:"resolvedBuild"`
	}{
		Options:  buildSkillOptionsForBuild(build),
		Prereqs:  buildSkillPrereqsForBuild(build),
		Resolved: build,
	}
	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(payload)
}

// openDroplogs opens the droplogs directory in Windows Explorer.
func (s *HttpServer) openDroplogs(w http.ResponseWriter, r *http.Request) {
	base := config.Koolo.LogSaveDirectory
	if base == "" {
		base = "logs"
	}
	dir := filepath.Join(base, "droplogs")

	if err := os.MkdirAll(dir, 0o755); err != nil {
		http.Error(w, fmt.Sprintf("failed to create directory: %v", err), http.StatusInternalServerError)
		return
	}

	// Open folder using Windows Explorer
	cmd := exec.Command("explorer.exe", dir)
	if err := cmd.Start(); err != nil {
		http.Error(w, fmt.Sprintf("failed to open folder: %v", err), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "ok", "dir": dir})
}

// resetDroplogs removes droplog JSONL/HTML files from the droplogs directory.
func (s *HttpServer) resetDroplogs(w http.ResponseWriter, r *http.Request) {
	base := config.Koolo.LogSaveDirectory
	if base == "" {
		base = "logs"
	}
	dir := filepath.Join(base, "droplogs")

	if err := os.MkdirAll(dir, 0o755); err != nil {
		http.Error(w, fmt.Sprintf("failed to create directory: %v", err), http.StatusInternalServerError)
		return
	}

	entries, err := os.ReadDir(dir)
	if err != nil {
		http.Error(w, fmt.Sprintf("failed to list directory: %v", err), http.StatusInternalServerError)
		return
	}

	removed := 0
	for _, e := range entries {
		if e.IsDir() {
			continue
		}
		name := strings.ToLower(e.Name())
		if strings.HasSuffix(name, ".jsonl") || strings.HasSuffix(name, ".html") {
			_ = os.Remove(filepath.Join(dir, e.Name()))
			removed++
		}
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]any{"status": "ok", "dir": dir, "removed": removed})
}

func (s *HttpServer) getIntFromForm(r *http.Request, param string, min int, max int, defaultValue int) int {
	result := defaultValue
	paramValue, err := strconv.Atoi(r.Form.Get(param))
	if err != nil {
		s.logger.Warn("Invalid form value, setting to default",
			slog.String("parameter", param),
			slog.String("error", err.Error()),
			slog.Int("default", 0))
	} else {
		result = int(math.Max(math.Min(float64(paramValue), float64(max)), float64(min)))
	}
	return result
}

func buildTZGroups() []TZGroup {
	groups := make(map[string][]area.ID)
	for id, info := range terrorzones.Zones() {
		groupName := info.Group
		if groupName == "" {
			groupName = id.Area().Name
		}
		groups[groupName] = append(groups[groupName], id)
	}

	var result []TZGroup
	for name, ids := range groups {
		zone := terrorzones.Zones()[ids[0]]

		result = append(result, TZGroup{
			Act:           zone.Act,
			Name:          name,
			PrimaryAreaID: int(ids[0]),
			Immunities:    zone.Immunities,
			BossPacks:     zone.BossPack,
			ExpTier:       string(zone.ExpTier),
			LootTier:      string(zone.LootTier),
		})
	}

	slices.SortStableFunc(result, func(a, b TZGroup) int {
		if a.Act != b.Act {
			return cmp.Compare(a.Act, b.Act)
		}
		return cmp.Compare(a.Name, b.Name)
	})

	return result
}

// Updater handlers

func (s *HttpServer) getVersion(w http.ResponseWriter, r *http.Request) {
	version, err := updater.GetCurrentVersionNoClone()
	if err != nil {
		http.Error(w, fmt.Sprintf("Failed to get version: %v", err), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"commitHash": version.CommitHash,
		"commitDate": formatCommitDate(version.CommitDate),
		"commitMsg":  version.CommitMsg,
		"branch":     version.Branch,
	})
}

func (s *HttpServer) checkUpdates(w http.ResponseWriter, r *http.Request) {
	result, err := updater.CheckForUpdates()
	if err != nil {
		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(http.StatusInternalServerError)
		json.NewEncoder(w).Encode(map[string]interface{}{
			"error": fmt.Sprintf("Failed to check for updates: %v", err),
		})
		return
	}

	commits := make([]map[string]string, 0)
	for _, c := range result.NewCommits {
		commits = append(commits, map[string]string{
			"hash":    c.Hash,
			"date":    c.Date.Format("2006-01-02 15:04:05"),
			"message": c.Message,
		})
	}

	aheadCommits := make([]map[string]string, 0)
	for _, c := range result.AheadCommits {
		aheadCommits = append(aheadCommits, map[string]string{
			"hash":    c.Hash,
			"date":    c.Date.Format("2006-01-02 15:04:05"),
			"message": c.Message,
		})
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"hasUpdates":    result.HasUpdates,
		"commitsAhead":  result.CommitsAhead,
		"commitsBehind": result.CommitsBehind,
		"aheadCommits":  aheadCommits,
		"newCommits":    commits,
		"currentVersion": map[string]interface{}{
			"commitHash": result.CurrentVersion.CommitHash,
			"commitDate": formatCommitDate(result.CurrentVersion.CommitDate),
			"commitMsg":  result.CurrentVersion.CommitMsg,
			"branch":     result.CurrentVersion.Branch,
		},
	})
}

func (s *HttpServer) getCurrentCommits(w http.ResponseWriter, r *http.Request) {
	limit := 10
	if limitStr := r.URL.Query().Get("limit"); limitStr != "" {
		if l, err := strconv.Atoi(limitStr); err == nil && l > 0 && l <= 50 {
			limit = l
		}
	}

	commits, err := updater.GetCurrentCommits(limit)
	if err != nil {
		http.Error(w, fmt.Sprintf("Failed to get current commits: %v", err), http.StatusInternalServerError)
		return
	}

	payload := make([]map[string]string, 0, len(commits))
	for _, c := range commits {
		payload = append(payload, map[string]string{
			"hash":    c.Hash,
			"date":    c.Date.Format("2006-01-02 15:04:05"),
			"message": c.Message,
		})
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(payload)
}

func (s *HttpServer) performUpdate(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	// Check if any bots are running
	runningCount := 0
	for _, supervisorName := range s.manager.AvailableSupervisors() {
		stats := s.manager.Status(supervisorName)
		// Consider bot running if in Starting, InGame, or Paused state
		if stats.SupervisorStatus == bot.Starting ||
			stats.SupervisorStatus == bot.InGame ||
			stats.SupervisorStatus == bot.Paused {
			runningCount++
		}
	}

	if runningCount > 0 {
		http.Error(w, fmt.Sprintf("Cannot update while %d bot(s) are running. Please stop all bots first.", runningCount), http.StatusConflict)
		return
	}

	if !s.updater.TryStartOperation("update") {
		http.Error(w, "Updater is already running another operation", http.StatusConflict)
		return
	}

	// Parse auto-restart flag
	autoRestart := r.URL.Query().Get("restart") == "true"
	mode := strings.ToLower(strings.TrimSpace(r.URL.Query().Get("mode")))
	source := strings.ToLower(strings.TrimSpace(r.URL.Query().Get("source")))

	// Set log callback to broadcast via WebSocket
	s.updater.SetLogCallback(func(message string) {
		s.wsServer.broadcast <- []byte(fmt.Sprintf(`{"type":"updater_log","message":%q}`, message))
	})

	// Start update in background
	go func() {
		defer s.updater.EndOperation()
		var err error
		if mode == "build" {
			backupTag := "build"
			if source == "pr" {
				backupTag = "pr"
			}
			err = s.updater.ExecuteBuild(autoRestart, backupTag)
		} else {
			err = s.updater.ExecuteUpdate(autoRestart)
		}
		if err != nil {
			s.logger.Error("Update failed", slog.Any("error", err))
			s.wsServer.broadcast <- []byte(fmt.Sprintf(`{"type":"updater_error","error":%q}`, err.Error()))
		} else {
			s.wsServer.broadcast <- []byte(`{"type":"updater_complete"}`)
		}
	}()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{
		"status": "update started",
	})
}

func (s *HttpServer) getUpdaterStatus(w http.ResponseWriter, r *http.Request) {
	status := s.updater.GetStatus()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(status)
}

func (s *HttpServer) getBackups(w http.ResponseWriter, r *http.Request) {
	// Get backup versions (limit to 5 most recent)
	backups, err := updater.GetBackupVersions(5)
	if err != nil {
		http.Error(w, fmt.Sprintf("Failed to get backup versions: %v", err), http.StatusInternalServerError)
		return
	}

	// Get current executable info
	currentExe, _ := updater.GetCurrentExecutable()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"backups": backups,
		"current": currentExe,
	})
}

func (s *HttpServer) performRollback(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	// Check if any bots are running
	runningCount := 0
	for _, supervisorName := range s.manager.AvailableSupervisors() {
		stats := s.manager.Status(supervisorName)
		if stats.SupervisorStatus == bot.Starting ||
			stats.SupervisorStatus == bot.InGame ||
			stats.SupervisorStatus == bot.Paused {
			runningCount++
		}
	}

	if runningCount > 0 {
		http.Error(w, fmt.Sprintf("Cannot rollback while %d bot(s) are running. Please stop all bots first.", runningCount), http.StatusConflict)
		return
	}

	// Get backup file path from request
	var request struct {
		BackupPath string `json:"backupPath"`
	}
	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	if request.BackupPath == "" {
		http.Error(w, "backupPath is required", http.StatusBadRequest)
		return
	}

	// Confirm the file exists
	if _, err := os.Stat(request.BackupPath); os.IsNotExist(err) {
		http.Error(w, "Backup file not found", http.StatusNotFound)
		return
	}

	exePath, err := os.Executable()
	if err != nil {
		http.Error(w, "Failed to resolve install directory", http.StatusInternalServerError)
		return
	}
	absExe, err := filepath.Abs(exePath)
	if err != nil {
		http.Error(w, "Failed to resolve install directory", http.StatusInternalServerError)
		return
	}
	installDir := filepath.Dir(absExe)
	oldVersionsDir := filepath.Join(installDir, "old_versions")
	absOldVersions, err := filepath.Abs(oldVersionsDir)
	if err != nil {
		http.Error(w, "Failed to resolve backup directory", http.StatusInternalServerError)
		return
	}
	absBackup, err := filepath.Abs(request.BackupPath)
	if err != nil {
		http.Error(w, "Invalid backupPath", http.StatusBadRequest)
		return
	}
	base := strings.TrimRight(absOldVersions, string(os.PathSeparator)) + string(os.PathSeparator)
	if !strings.HasPrefix(strings.ToLower(absBackup), strings.ToLower(base)) {
		http.Error(w, "backupPath must be inside old_versions", http.StatusBadRequest)
		return
	}

	if !s.updater.TryStartOperation("rollback") {
		http.Error(w, "Updater is already running another operation", http.StatusConflict)
		return
	}

	// Set log callback to broadcast via WebSocket
	s.updater.SetLogCallback(func(message string) {
		s.wsServer.broadcast <- []byte(fmt.Sprintf(`{"type":"rollback_log","message":%q}`, message))
	})

	// Perform rollback in background
	go func() {
		defer s.updater.EndOperation()
		err := s.updater.RollbackToVersion(request.BackupPath)
		if err != nil {
			s.logger.Error("Rollback failed", slog.Any("error", err))
			s.wsServer.broadcast <- []byte(fmt.Sprintf(`{"type":"rollback_error","error":%q}`, err.Error()))
		}
		// Note: If successful, the application will restart, so no completion message is sent
	}()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{
		"status": "rollback started",
	})
}

func (s *HttpServer) getUpstreamPRs(w http.ResponseWriter, r *http.Request) {
	// Get query parameters
	state := r.URL.Query().Get("state")
	if state == "" {
		state = "open"
	}

	limit := 30
	if limitStr := r.URL.Query().Get("limit"); limitStr != "" {
		if l, err := strconv.Atoi(limitStr); err == nil && l > 0 {
			limit = l
		}
	}

	prs, err := updater.GetUpstreamPRs(state, limit)
	if err != nil {
		http.Error(w, fmt.Sprintf("Failed to fetch PRs: %v", err), http.StatusInternalServerError)
		return
	}

	applied, err := updater.LoadAppliedPRs()
	if err != nil {
		s.logger.Warn("Failed to load applied PRs", slog.Any("error", err))
	} else {
		for i := range prs {
			if info, ok := applied[prs[i].Number]; ok {
				prs[i].Applied = true
				prs[i].CanRevert = len(info.Commits) > 0
			}
		}
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(prs)
}

func (s *HttpServer) cherryPickPRs(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	// Check if any bots are running
	runningCount := 0
	for _, supervisorName := range s.manager.AvailableSupervisors() {
		stats := s.manager.Status(supervisorName)
		if stats.SupervisorStatus == bot.Starting ||
			stats.SupervisorStatus == bot.InGame ||
			stats.SupervisorStatus == bot.Paused {
			runningCount++
		}
	}

	if runningCount > 0 {
		http.Error(w, fmt.Sprintf("Cannot cherry-pick while %d bot(s) are running. Please stop all bots first.", runningCount), http.StatusConflict)
		return
	}

	// Parse request body
	var request struct {
		PRNumbers []int `json:"prNumbers"`
	}
	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	if len(request.PRNumbers) == 0 {
		http.Error(w, "prNumbers is required", http.StatusBadRequest)
		return
	}

	if !s.updater.TryStartOperation("cherry-pick") {
		http.Error(w, "Updater is already running another operation", http.StatusConflict)
		return
	}

	// Set log callback to broadcast via WebSocket
	s.updater.SetLogCallback(func(message string) {
		s.wsServer.broadcast <- []byte(fmt.Sprintf(`{"type":"cherrypick_log","message":%q}`, message))
	})

	// Perform cherry-pick in background
	go func() {
		defer s.updater.EndOperation()
		results, err := s.updater.CherryPickMultiplePRs(request.PRNumbers, func(message string) {
			s.wsServer.broadcast <- []byte(fmt.Sprintf(`{"type":"cherrypick_log","message":%q}`, message))
		})

		if err != nil {
			s.logger.Error("Cherry-pick failed", slog.Any("error", err))
			s.wsServer.broadcast <- []byte(fmt.Sprintf(`{"type":"cherrypick_error","error":%q}`, err.Error()))
			return
		}

		// Send results
		resultsJSON, _ := json.Marshal(results)
		s.wsServer.broadcast <- []byte(fmt.Sprintf(`{"type":"cherrypick_complete","results":%s}`, resultsJSON))
	}()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{
		"status": "cherry-pick started",
	})
}

func (s *HttpServer) revertPR(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	// Check if any bots are running
	runningCount := 0
	for _, supervisorName := range s.manager.AvailableSupervisors() {
		stats := s.manager.Status(supervisorName)
		if stats.SupervisorStatus == bot.Starting ||
			stats.SupervisorStatus == bot.InGame ||
			stats.SupervisorStatus == bot.Paused {
			runningCount++
		}
	}

	if runningCount > 0 {
		http.Error(w, fmt.Sprintf("Cannot revert while %d bot(s) are running. Please stop all bots first.", runningCount), http.StatusConflict)
		return
	}

	var request struct {
		PRNumber int `json:"prNumber"`
	}
	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}
	if request.PRNumber <= 0 {
		http.Error(w, "prNumber is required", http.StatusBadRequest)
		return
	}

	if !s.updater.TryStartOperation("revert") {
		http.Error(w, "Updater is already running another operation", http.StatusConflict)
		return
	}

	progressCallback := func(message string) {
		s.wsServer.broadcast <- []byte(fmt.Sprintf(`{"type":"revert_log","message":%q}`, message))
	}

	go func(prNumber int) {
		defer s.updater.EndOperation()
		result, err := s.updater.RevertPR(prNumber, progressCallback)
		if err != nil {
			s.logger.Error("Revert failed", slog.Any("error", err))
			s.wsServer.broadcast <- []byte(fmt.Sprintf(`{"type":"revert_error","error":%q}`, err.Error()))
			return
		}
		if result != nil && !result.Success {
			errMsg := result.Error
			if errMsg == "" {
				errMsg = fmt.Sprintf("Revert failed for PR #%d", prNumber)
			}
			s.logger.Error("Revert failed", slog.String("reason", errMsg))
			s.wsServer.broadcast <- []byte(fmt.Sprintf(`{"type":"revert_error","error":%q}`, errMsg))
			return
		}
		s.wsServer.broadcast <- []byte(`{"type":"revert_complete"}`)
	}(request.PRNumber)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{
		"status": "revert started",
	})
}

func (s *HttpServer) generateBattleNetToken(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req struct {
		Username string `json:"username"`
		Password string `json:"password"`
		Realm    string `json:"realm"`
	}

	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		s.logger.Error("Failed to decode request", slog.Any("error", err))
		http.Error(w, "Invalid request format", http.StatusBadRequest)
		return
	}

	// Validate input
	if req.Username == "" || req.Password == "" {
		http.Error(w, "Username and password are required", http.StatusBadRequest)
		return
	}

	s.logger.Info("Generating Battle.net token",
		slog.String("username", req.Username),
		slog.String("realm", req.Realm))

	flusher, ok := w.(http.Flusher)
	if !ok {
		http.Error(w, "Streaming not supported", http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "text/plain; charset=utf-8")
	w.Header().Set("Cache-Control", "no-cache")

	sendLine := func(line string) {
		if line == "" {
			return
		}
		fmt.Fprintln(w, line)
		flusher.Flush()
	}

	token, err := game.GetBattleNetTokenWithDebugContext(r.Context(), req.Username, req.Password, req.Realm, sendLine)
	if err != nil {
		s.logger.Error("Failed to generate Battle.net token",
			slog.String("username", req.Username),
			slog.Any("error", err))
		sendLine("ERROR: " + err.Error())
		return
	}

	s.logger.Info("Battle.net token generated successfully",
		slog.String("username", req.Username))

	sendLine("TOKEN: " + token)
}
