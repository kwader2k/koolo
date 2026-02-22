package server

import (
	"encoding/json"
	"errors"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strings"

	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/run"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type SequenceAPI struct {
	logger       *slog.Logger
	commentRegex *regexp.Regexp
}

type sequenceLoadResponse struct {
	Name     string                       `json:"name"`
	Settings run.LevelingSequenceSettings `json:"settings"`
}

type sequenceSaveRequest struct {
	Name     string                       `json:"name"`
	Settings run.LevelingSequenceSettings `json:"settings"`
}

type sequenceSaveResponse struct {
	Status string `json:"status"`
	Path   string `json:"path"`
}

type sequenceDeleteRequest struct {
	Name string `json:"name"`
}

type sequenceBrowseResponse struct {
	Name      string                        `json:"name,omitempty"`
	Settings  *run.LevelingSequenceSettings `json:"settings,omitempty"`
	Cancelled bool                          `json:"cancelled,omitempty"`
}

type questMetadata struct {
	Run         string `json:"run"`
	Act         int    `json:"act"`
	IsMandatory bool   `json:"isMandatory"`
}

type sequenceRunsResponse struct {
	Runs          []string        `json:"runs"`
	SequencerRuns []string        `json:"sequencerRuns"`
	QuestCatalog  []questMetadata `json:"questCatalog"`
}

type sequenceFilesResponse struct {
	Files []string `json:"files"`
}

func NewSequenceAPI(logger *slog.Logger) *SequenceAPI {
	return &SequenceAPI{
		logger:       logger,
		commentRegex: regexp.MustCompile(`(?s)//.*?\n|/\*.*?\*/`),
	}
}

func (api *SequenceAPI) handleListRuns(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	runNames := make([]string, 0, len(config.AvailableRuns))
	for runName := range config.AvailableRuns {
		runNames = append(runNames, string(runName))
	}
	sort.Strings(runNames)

	sequencerRuns := make([]string, len(config.SequencerRuns))
	for i, runName := range config.SequencerRuns {
		sequencerRuns[i] = string(runName)
	}

	questCatalog := make([]questMetadata, 0, len(config.SequencerQuests))
	for _, quest := range config.SequencerQuests {
		questCatalog = append(questCatalog, questMetadata{
			Run:         string(quest.Run),
			Act:         quest.Act,
			IsMandatory: quest.IsMandatory,
		})
	}

	api.writeJSON(w, http.StatusOK, sequenceRunsResponse{
		Runs:          runNames,
		SequencerRuns: sequencerRuns,
		QuestCatalog:  questCatalog,
	})
}

func (api *SequenceAPI) handleListSequenceFiles(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	files, err := api.ListSequenceFiles()
	if err != nil {
		api.internalError(w, fmt.Errorf("failed to list sequence files: %w", err))
		return
	}

	api.writeJSON(w, http.StatusOK, sequenceFilesResponse{Files: files})
}

func (api *SequenceAPI) handleGetSequence(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	name, err := api.extractSequenceName(r.URL.Query().Get("name"))
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	dir, err := api.baseDir()
	if err != nil {
		api.internalError(w, fmt.Errorf("failed to resolve sequence directory: %w", err))
		return
	}

	filePath := filepath.Join(dir, name+".json")
	settings, err := api.readSequenceFile(filePath)
	if err != nil {
		if errors.Is(err, os.ErrNotExist) {
			http.Error(w, "sequence file not found", http.StatusNotFound)
			return
		}
		var syntaxErr *json.SyntaxError
		if errors.As(err, &syntaxErr) {
			http.Error(w, "invalid sequence file", http.StatusBadRequest)
			return
		}
		api.internalError(w, fmt.Errorf("failed to read sequence file: %w", err))
		return
	}

	api.writeJSON(w, http.StatusOK, sequenceLoadResponse{
		Name:     name,
		Settings: settings,
	})
}

func (api *SequenceAPI) handleBrowseSequence(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	baseDir, err := api.baseDir()
	if err != nil {
		api.internalError(w, fmt.Errorf("failed to resolve sequence directory: %w", err))
		return
	}

	filters := []utils.FileDialogFilter{
		{Name: "Sequence Files (*.json)", Pattern: "*.json"},
		{Name: "All Files (*.*)", Pattern: "*.*"},
	}

	path, err := utils.BrowseForFile("Open Leveling Sequence", filters, baseDir, "json")
	if err != nil {
		api.internalError(w, fmt.Errorf("open sequence dialog failed: %w", err))
		return
	}

	if path == "" {
		api.writeJSON(w, http.StatusOK, sequenceBrowseResponse{Cancelled: true})
		return
	}

	if !strings.EqualFold(filepath.Ext(path), ".json") {
		http.Error(w, "selected file must be a .json sequence", http.StatusBadRequest)
		return
	}

	absPath, err := filepath.Abs(path)
	if err != nil {
		api.internalError(w, fmt.Errorf("failed to resolve sequence path: %w", err))
		return
	}

	settings, err := api.readSequenceFile(absPath)
	if err != nil {
		var syntaxErr *json.SyntaxError
		if errors.As(err, &syntaxErr) {
			http.Error(w, "selected file is not a valid sequence", http.StatusBadRequest)
			return
		}
		api.internalError(w, fmt.Errorf("failed to read selected sequence: %w", err))
		return
	}

	base := strings.TrimSuffix(filepath.Base(absPath), filepath.Ext(absPath))
	name, err := api.extractSequenceName(base)
	if err != nil {
		http.Error(w, "sequence file name contains unsupported characters", http.StatusBadRequest)
		return
	}

	api.writeJSON(w, http.StatusOK, sequenceBrowseResponse{
		Name:     name,
		Settings: &settings,
	})
}

func (api *SequenceAPI) handleSaveSequence(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	defer r.Body.Close()

	var req sequenceSaveRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "invalid request payload", http.StatusBadRequest)
		return
	}

	name, err := api.extractSequenceName(req.Name)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	dir, err := api.baseDir()
	if err != nil {
		api.internalError(w, fmt.Errorf("failed to resolve sequence directory: %w", err))
		return
	}

	api.normalizeSettings(&req.Settings)

	if err := api.writeSequenceFile(filepath.Join(dir, name+".json"), req.Settings, true); err != nil {
		api.internalError(w, fmt.Errorf("failed to save sequence: %w", err))
		return
	}

	api.writeJSON(w, http.StatusOK, sequenceSaveResponse{Status: "ok", Path: filepath.Join(dir, name+".json")})
}

func (api *SequenceAPI) handleDeleteSequence(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	defer r.Body.Close()

	var req sequenceDeleteRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "invalid request payload", http.StatusBadRequest)
		return
	}

	name, err := api.extractSequenceName(req.Name)
	if err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	dir, err := api.baseDir()
	if err != nil {
		api.internalError(w, fmt.Errorf("failed to resolve sequence directory: %w", err))
		return
	}

	path := filepath.Join(dir, name+".json")
	if err := os.Remove(path); err != nil {
		if errors.Is(err, os.ErrNotExist) {
			http.Error(w, "sequence file not found", http.StatusNotFound)
			return
		}
		api.internalError(w, fmt.Errorf("failed to delete sequence: %w", err))
		return
	}

	api.writeJSON(w, http.StatusOK, map[string]string{"status": "deleted"})
}

func (api *SequenceAPI) baseDir() (string, error) {
	cwd, err := os.Getwd()
	if err != nil {
		return "", err
	}

	dir := filepath.Join(cwd, "config", "template", "sequences_leveling")
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return "", err
	}

	return dir, nil
}

func (api *SequenceAPI) ListSequenceFiles() ([]string, error) {
	dir, err := api.baseDir()
	if err != nil {
		return nil, err
	}

	entries, err := os.ReadDir(dir)
	if err != nil {
		if errors.Is(err, os.ErrNotExist) {
			return []string{}, nil
		}
		return nil, err
	}

	files := make([]string, 0, len(entries))
	seen := make(map[string]struct{}, len(entries))
	for _, entry := range entries {
		if entry.IsDir() {
			continue
		}
		name := entry.Name()
		ext := filepath.Ext(name)
		if !strings.EqualFold(ext, ".json") {
			continue
		}
		trimmed := strings.TrimSuffix(name, ext)
		if trimmed == "" {
			continue
		}
		if _, exists := seen[trimmed]; exists {
			continue
		}
		seen[trimmed] = struct{}{}
		files = append(files, trimmed)
	}

	sort.Strings(files)
	return files, nil
}

func (api *SequenceAPI) extractSequenceName(raw string) (string, error) {
	raw = strings.TrimSpace(raw)
	if raw == "" {
		return "", errors.New("sequence name is required")
	}

	if strings.ContainsAny(raw, `/\\`) {
		return "", errors.New("sequence name cannot contain path separators")
	}

	valid, err := regexp.MatchString(`^[A-Za-z0-9_-]+$`, raw)
	if err != nil {
		return "", err
	}
	if !valid {
		return "", errors.New("sequence name may only contain letters, numbers, underscores, or hyphens")
	}

	return raw, nil
}

func (api *SequenceAPI) writeSequenceFile(path string, settings run.LevelingSequenceSettings, overwrite bool) error {
	if !overwrite {
		if _, err := os.Stat(path); err == nil {
			return fmt.Errorf("file already exists: %s", path)
		}
	}

	api.normalizeSettings(&settings)

	payload, err := json.MarshalIndent(settings, "", "    ")
	if err != nil {
		return err
	}

	payload = append(payload, '\n')

	if err := os.WriteFile(path, payload, 0o644); err != nil {
		return err
	}

	return nil
}

func (api *SequenceAPI) readSequenceFile(path string) (run.LevelingSequenceSettings, error) {
	var settings run.LevelingSequenceSettings

	content, err := os.ReadFile(path)
	if err != nil {
		return settings, err
	}

	cleaned := api.commentRegex.ReplaceAll(content, nil)
	if err := json.Unmarshal(cleaned, &settings); err != nil {
		return settings, err
	}

	api.normalizeSettings(&settings)
	return settings, nil
}

func (api *SequenceAPI) normalizeSettings(settings *run.LevelingSequenceSettings) {
	normalizeDifficulty := func(dst *run.DifficultyLevelingSettings) {
		if dst == nil {
			return
		}
		if dst.Quests == nil {
			dst.Quests = make([]run.SequenceSettings, 0)
		}
		if dst.BeforeQuests == nil {
			dst.BeforeQuests = make([]run.SequenceSettings, 0)
		}
		if dst.AfterQuests == nil {
			dst.AfterQuests = make([]run.SequenceSettings, 0)
		}
		if dst.ConfigSettings == nil {
			dst.ConfigSettings = make([]run.ConfigLevelingSettings, 0)
		}
	}

	normalizeDifficulty(&settings.Normal)
	normalizeDifficulty(&settings.Nightmare)
	normalizeDifficulty(&settings.Hell)
}

func (api *SequenceAPI) writeJSON(w http.ResponseWriter, status int, payload any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	if err := json.NewEncoder(w).Encode(payload); err != nil {
		api.logger.Error("failed to write JSON response", slog.Any("error", err))
	}
}

func (api *SequenceAPI) internalError(w http.ResponseWriter, err error) {
	api.logger.Error("sequence api error", slog.Any("error", err))
	http.Error(w, "internal server error", http.StatusInternalServerError)
}
