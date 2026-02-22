package server

import (
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"path/filepath"
	"strings"

	"github.com/hectorgimenez/koolo/internal/pickit"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// PickitAPI handles all pickit editor endpoints
type PickitAPI struct {
	builder *pickit.NIPBuilder
}

// NewPickitAPI creates a new pickit API handler
func NewPickitAPI() *PickitAPI {
	return &PickitAPI{
		builder: pickit.NewNIPBuilder(),
	}
}

// RegisterRoutes registers all pickit API routes
func (api *PickitAPI) RegisterRoutes(mux *http.ServeMux) {
	// Item database endpoints
	mux.HandleFunc("/api/pickit/items", api.handleGetItems)
	mux.HandleFunc("/api/pickit/items/search", api.handleSearchItems)
	mux.HandleFunc("/api/pickit/items/categories", api.handleGetCategories)

	// Rule endpoints
	mux.HandleFunc("/api/pickit/rules", api.handleGetRules)
	mux.HandleFunc("/api/pickit/rules/create", api.handleCreateRule)
	mux.HandleFunc("/api/pickit/rules/update", api.handleUpdateRule)
	mux.HandleFunc("/api/pickit/rules/delete", api.handleDeleteRule)
	mux.HandleFunc("/api/pickit/rules/validate", api.handleValidateRule)
	mux.HandleFunc("/api/pickit/rules/validate-nip", api.handleValidateNIPLine)

	// File endpoints
	mux.HandleFunc("/api/pickit/files", api.handleGetFiles)
	mux.HandleFunc("/api/pickit/files/import", api.handleImportFile)
	mux.HandleFunc("/api/pickit/files/export", api.handleExportFile)
	mux.HandleFunc("/api/pickit/files/rules/delete", api.handleDeleteFileRule)
	mux.HandleFunc("/api/pickit/files/rules/update", api.handleUpdateFileRule)
	mux.HandleFunc("/api/pickit/files/rules/append", api.handleAppendNIPLine)
	mux.HandleFunc("/api/pickit/browse-folder", api.handleBrowseFolder)

	// Template endpoints
	mux.HandleFunc("/api/pickit/templates", api.handleGetTemplates)
	mux.HandleFunc("/api/pickit/presets", api.handleGetPresets)

	// Utility endpoints
	mux.HandleFunc("/api/pickit/stats", api.handleGetStats)
	mux.HandleFunc("/api/pickit/simulate", api.handleSimulate)
	mux.HandleFunc("/api/pickit/suggestions", api.handleGetSuggestions)
	mux.HandleFunc("/api/pickit/conflicts", api.handleDetectConflicts)
}

// handleGetItems returns all items from the database
func (api *PickitAPI) handleGetItems(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	// Get query parameters for filtering
	category := r.URL.Query().Get("category")

	var items []pickit.ItemDefinition
	if category != "" {
		// Filter by category
		items = pickit.GetItemsByCategory(category)
	} else {
		// Return all items from V2 database
		items = pickit.GetAllItemsV2()
	}

	api.sendJSON(w, items)
}

// handleSearchItems searches for items with filters
func (api *PickitAPI) handleSearchItems(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var filters pickit.SearchFilters
	if err := json.NewDecoder(r.Body).Decode(&filters); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	results := pickit.SearchItems(filters)
	api.sendJSON(w, results)
}

// handleGetCategories returns all item categories
func (api *PickitAPI) handleGetCategories(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	response := map[string]interface{}{
		"categories": pickit.GetItemCategories(),
		"types":      pickit.GetItemTypes(),
		"qualities":  pickit.GetItemQualities(),
	}

	api.sendJSON(w, response)
}

// handleGetRules returns all rules for a character
func (api *PickitAPI) handleGetRules(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	characterID := r.URL.Query().Get("character")
	if characterID == "" {
		http.Error(w, "character parameter required", http.StatusBadRequest)
		return
	}

	// Load rules from character's pickit files
	rules, err := api.loadCharacterRules(characterID)
	if err != nil {
		http.Error(w, fmt.Sprintf("Error loading rules: %v", err), http.StatusInternalServerError)
		return
	}

	api.sendJSON(w, rules)
}

// handleCreateRule creates a new pickit rule
func (api *PickitAPI) handleCreateRule(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var rule pickit.PickitRule
	if err := json.NewDecoder(r.Body).Decode(&rule); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	// Validate rule
	validation := api.builder.ValidateRule(&rule)
	if !validation.Valid {
		api.sendJSON(w, map[string]interface{}{
			"success":    false,
			"validation": validation,
		})
		return
	}

	// Generate NIP syntax
	nipLine, err := api.builder.GenerateNIP(&rule)
	if err != nil {
		http.Error(w, fmt.Sprintf("Error generating NIP: %v", err), http.StatusInternalServerError)
		return
	}
	rule.GeneratedNIP = nipLine

	// Save rule (implementation needed)
	characterID := r.URL.Query().Get("character")
	if err := api.saveRule(characterID, &rule); err != nil {
		http.Error(w, fmt.Sprintf("Error saving rule: %v", err), http.StatusInternalServerError)
		return
	}

	api.sendJSON(w, map[string]interface{}{
		"success":    true,
		"rule":       rule,
		"validation": validation,
	})
}

// handleUpdateRule updates an existing rule
func (api *PickitAPI) handleUpdateRule(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var rule pickit.PickitRule
	if err := json.NewDecoder(r.Body).Decode(&rule); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	// Validate and update
	validation := api.builder.ValidateRule(&rule)
	if !validation.Valid {
		api.sendJSON(w, map[string]interface{}{
			"success":    false,
			"validation": validation,
		})
		return
	}

	// Regenerate NIP syntax
	nipLine, err := api.builder.GenerateNIP(&rule)
	if err != nil {
		http.Error(w, fmt.Sprintf("Error generating NIP: %v", err), http.StatusInternalServerError)
		return
	}
	rule.GeneratedNIP = nipLine

	characterID := r.URL.Query().Get("character")
	if err := api.updateRule(characterID, &rule); err != nil {
		http.Error(w, fmt.Sprintf("Error updating rule: %v", err), http.StatusInternalServerError)
		return
	}

	api.sendJSON(w, map[string]interface{}{
		"success": true,
		"rule":    rule,
	})
}

// handleDeleteRule deletes a rule
func (api *PickitAPI) handleDeleteRule(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	ruleID := r.URL.Query().Get("id")
	characterID := r.URL.Query().Get("character")

	if err := api.deleteRule(characterID, ruleID); err != nil {
		http.Error(w, fmt.Sprintf("Error deleting rule: %v", err), http.StatusInternalServerError)
		return
	}

	api.sendJSON(w, map[string]interface{}{
		"success": true,
	})
}

// handleValidateRule validates a rule without saving
func (api *PickitAPI) handleValidateRule(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var rule pickit.PickitRule
	if err := json.NewDecoder(r.Body).Decode(&rule); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	validation := api.builder.ValidateRule(&rule)

	// Try to generate NIP
	nipLine, err := api.builder.GenerateNIP(&rule)
	if err == nil {
		rule.GeneratedNIP = nipLine
	}

	api.sendJSON(w, map[string]interface{}{
		"validation": validation,
		"nipSyntax":  rule.GeneratedNIP,
	})
}

// handleGetFiles returns all pickit files for a character
func (api *PickitAPI) handleGetFiles(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		api.sendError(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	// Support both 'path' (new) and 'character' (legacy) parameters
	pickitPath := r.URL.Query().Get("path")
	characterID := r.URL.Query().Get("character")
	fileName := r.URL.Query().Get("file")

	// Determine the directory to use
	var directory string
	if pickitPath != "" {
		directory = pickitPath
	} else if characterID != "" {
		directory = filepath.Join("config", characterID, "pickit")
	} else {
		api.sendError(w, "Either 'path' or 'character' parameter required", http.StatusBadRequest)
		return
	}

	// If file parameter is provided, return the content of that specific file
	if fileName != "" {
		rules, err := api.loadPickitFileRulesFromPath(directory, fileName)
		if err != nil {
			api.sendError(w, fmt.Sprintf("Error loading file: %v", err), http.StatusInternalServerError)
			return
		}
		api.sendJSON(w, rules)
		return
	}

	// Otherwise return list of all files
	files, err := api.getPickitFilesFromPath(directory)
	if err != nil {
		api.sendError(w, fmt.Sprintf("Error loading files: %v", err), http.StatusInternalServerError)
		return
	}

	api.sendJSON(w, files)
}

// handleImportFile imports a .nip file
func (api *PickitAPI) handleImportFile(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	// Parse multipart form
	if err := r.ParseMultipartForm(10 << 20); err != nil { // 10 MB max
		http.Error(w, "Error parsing form", http.StatusBadRequest)
		return
	}

	file, _, err := r.FormFile("file")
	if err != nil {
		http.Error(w, "Error reading file", http.StatusBadRequest)
		return
	}
	defer file.Close()

	// Read file content
	content, err := io.ReadAll(file)
	if err != nil {
		http.Error(w, "Error reading file content", http.StatusInternalServerError)
		return
	}

	// Parse NIP lines
	lines := strings.Split(string(content), "\n")
	rules := []pickit.PickitRule{}

	for _, line := range lines {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, "//") {
			continue
		}

		rule, err := api.builder.ParseNIP(line)
		if err != nil {
			// Skip invalid lines with warning
			continue
		}
		rules = append(rules, *rule)
	}

	api.sendJSON(w, map[string]interface{}{
		"success":    true,
		"rulesCount": len(rules),
		"rules":      rules,
	})
}

// handleExportFile exports rules to .nip format
func (api *PickitAPI) handleExportFile(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var request struct {
		Rules   []pickit.PickitRule  `json:"rules"`
		Options pickit.ExportOptions `json:"options"`
	}

	if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	// Export to NIP format
	nipContent, err := api.builder.ExportToNIP(request.Rules, request.Options)
	if err != nil {
		http.Error(w, fmt.Sprintf("Error exporting: %v", err), http.StatusInternalServerError)
		return
	}

	// Set headers for file download
	w.Header().Set("Content-Type", "text/plain")
	w.Header().Set("Content-Disposition", "attachment; filename=pickit.nip")
	w.Write([]byte(nipContent))
}

// handleGetTemplates returns rule templates
func (api *PickitAPI) handleGetTemplates(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	templates := pickit.GetRuleTemplates()
	api.sendJSON(w, templates)
}

// handleGetPresets returns stat presets
func (api *PickitAPI) handleGetPresets(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	presets := pickit.GetStatPresets()
	api.sendJSON(w, presets)
}

// handleGetStats returns all available stats
func (api *PickitAPI) handleGetStats(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	response := map[string]interface{}{
		"all":        pickit.GetAllStatTypes(),
		"byCategory": pickit.GetStatTypesByCategory(),
	}

	api.sendJSON(w, response)
}

// handleSimulate simulates a rule against test data
func (api *PickitAPI) handleSimulate(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var rule pickit.PickitRule
	if err := json.NewDecoder(r.Body).Decode(&rule); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	// Generate simulation based on rule
	result := pickit.SimulationResult{
		RuleID:      rule.ID,
		MatchCount:  0,
		Matches:     []pickit.ItemMatch{},
		Performance: "Good",
		Suggestions: []string{},
	}

	// Simulate by finding matching items in database
	_, err := api.builder.GenerateNIP(&rule)
	if err != nil {
		http.Error(w, fmt.Sprintf("Failed to generate NIP: %v", err), http.StatusBadRequest)
		return
	}

	// Find items that match the rule criteria
	for _, item := range pickit.ItemDatabase {
		// Check name match
		nameMatch := false
		for _, cond := range rule.LeftConditions {
			if cond.Property == "name" && cond.Operator == "==" {
				if item.ID == cond.Value || item.InternalName == cond.Value {
					nameMatch = true
					break
				}
			}
		}

		if nameMatch {
			result.MatchCount++
			result.Matches = append(result.Matches, pickit.ItemMatch{
				ItemName:  item.Name,
				ImageIcon: item.ImageIcon,
				Matched:   true,
				Reason:    "Name matches rule criteria",
				Stats:     make(map[string]interface{}),
			})
		}
	}

	// Add suggestions based on match count
	if result.MatchCount == 0 {
		result.Suggestions = append(result.Suggestions, "No items matched. Check your item name and conditions.")
	} else if result.MatchCount > 10 {
		result.Suggestions = append(result.Suggestions, "Rule matches many items. Consider adding quality or stat filters.")
	}

	api.sendJSON(w, result)

	api.sendJSON(w, result)
}

// handleGetSuggestions returns auto-suggestions for a rule
func (api *PickitAPI) handleGetSuggestions(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var rule pickit.PickitRule
	if err := json.NewDecoder(r.Body).Decode(&rule); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	suggestions := pickit.GetAutoSuggestions(&rule)
	api.sendJSON(w, suggestions)
}

// handleDetectConflicts detects conflicts between rules
func (api *PickitAPI) handleDetectConflicts(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var rules []pickit.PickitRule
	if err := json.NewDecoder(r.Body).Decode(&rules); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	conflicts := pickit.DetectConflicts(rules)
	api.sendJSON(w, conflicts)
}

// Helper functions

func (api *PickitAPI) sendJSON(w http.ResponseWriter, data interface{}) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(data)
}

func (api *PickitAPI) sendError(w http.ResponseWriter, message string, statusCode int) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(statusCode)
	json.NewEncoder(w).Encode(map[string]string{"error": message})
}

func (api *PickitAPI) loadPickitFileRules(characterID, fileName string) ([]pickit.PickitRule, error) {
	rules := []pickit.PickitRule{}

	// Get character's pickit directory
	pickitDir := filepath.Join("config", characterID, "pickit")
	filePath := filepath.Join(pickitDir, fileName)

	// Check if file exists
	fileInfo, err := os.Stat(filePath)
	if os.IsNotExist(err) {
		return rules, fmt.Errorf("file does not exist: %s", filePath)
	}
	if err != nil {
		return rules, fmt.Errorf("failed to stat file: %w", err)
	}

	// Log file size for debugging
	log.Printf("Loading pickit file: %s (size: %d bytes)", filePath, fileInfo.Size())

	// Read file content
	content, err := os.ReadFile(filePath)
	if err != nil {
		return rules, fmt.Errorf("failed to read file: %w", err)
	}

	// Parse NIP rules (basic parsing for now)
	lines := strings.Split(string(content), "\n")
	validLineCount := 0
	for i, line := range lines {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, "//") {
			continue
		}

		validLineCount++
		rule := pickit.PickitRule{
			ID:           fmt.Sprintf("%s_%d", fileName, i),
			FileName:     fileName,
			GeneratedNIP: line,
			Enabled:      true,
		}
		rules = append(rules, rule)
	}

	log.Printf("Parsed %d valid rules from %d total lines", validLineCount, len(lines))

	return rules, nil
}

func (api *PickitAPI) loadCharacterRules(characterID string) ([]pickit.PickitRule, error) {
	rules := []pickit.PickitRule{}

	// Get character's pickit directory
	pickitDir := filepath.Join("config", characterID, "pickit")

	// Check if directory exists
	if _, err := os.Stat(pickitDir); os.IsNotExist(err) {
		// Create directory if it doesn't exist
		if err := os.MkdirAll(pickitDir, 0755); err != nil {
			return rules, fmt.Errorf("failed to create pickit directory: %w", err)
		}
		return rules, nil
	}

	// Read all .nip files
	entries, err := os.ReadDir(pickitDir)
	if err != nil {
		return rules, fmt.Errorf("failed to read pickit directory: %w", err)
	}

	for _, entry := range entries {
		if !entry.IsDir() && strings.HasSuffix(entry.Name(), ".nip") {
			filePath := filepath.Join(pickitDir, entry.Name())
			content, err := os.ReadFile(filePath)
			if err != nil {
				continue
			}

			// Parse each line as a rule
			lines := strings.Split(string(content), "\n")
			for i, line := range lines {
				line = strings.TrimSpace(line)
				if line == "" || strings.HasPrefix(line, "//") {
					continue
				}

				rule, err := api.builder.ParseNIP(line)
				if err != nil {
					// Skip invalid rules
					continue
				}

				rule.ID = fmt.Sprintf("%s:%d", entry.Name(), i+1)
				rule.FileName = entry.Name()
				rules = append(rules, *rule)
			}
		}
	}

	return rules, nil
}

func (api *PickitAPI) saveRule(characterID string, rule *pickit.PickitRule) error {
	// Get character's pickit directory
	pickitDir := filepath.Join("config", characterID, "pickit")

	// Ensure directory exists
	if err := os.MkdirAll(pickitDir, 0755); err != nil {
		return fmt.Errorf("failed to create pickit directory: %w", err)
	}

	// Default file is general.nip
	fileName := "general.nip"
	if rule.FileName != "" {
		fileName = rule.FileName
	}

	filePath := filepath.Join(pickitDir, fileName)

	// Generate NIP syntax
	nipLine, err := api.builder.GenerateNIP(rule)
	if err != nil {
		return fmt.Errorf("failed to generate NIP syntax: %w", err)
	}

	// Read existing file or create new
	var content string
	if data, err := os.ReadFile(filePath); err == nil {
		content = string(data)
	}

	// Append new rule
	if content != "" && !strings.HasSuffix(content, "\n") {
		content += "\n"
	}
	content += nipLine + "\n"

	// Write back to file
	if err := os.WriteFile(filePath, []byte(content), 0644); err != nil {
		return fmt.Errorf("failed to write pickit file: %w", err)
	}

	return nil
}

func (api *PickitAPI) updateRule(characterID string, rule *pickit.PickitRule) error {
	// Parse rule ID to get file and line number
	parts := strings.Split(rule.ID, ":")
	if len(parts) != 2 {
		return fmt.Errorf("invalid rule ID format")
	}

	fileName := parts[0]
	filePath := filepath.Join("config", characterID, "pickit", fileName)

	// Read file
	content, err := os.ReadFile(filePath)
	if err != nil {
		return fmt.Errorf("failed to read pickit file: %w", err)
	}

	// Generate new NIP syntax
	newNipLine, err := api.builder.GenerateNIP(rule)
	if err != nil {
		return fmt.Errorf("failed to generate NIP syntax: %w", err)
	}

	// Split into lines and replace the specific line
	lines := strings.Split(string(content), "\n")
	lineNum, _ := fmt.Sscanf(parts[1], "%d")
	if lineNum > 0 && lineNum <= len(lines) {
		lines[lineNum-1] = newNipLine
	}

	// Write back
	if err := os.WriteFile(filePath, []byte(strings.Join(lines, "\n")), 0644); err != nil {
		return fmt.Errorf("failed to update pickit file: %w", err)
	}

	return nil
}

func (api *PickitAPI) deleteRule(characterID, ruleID string) error {
	// Parse rule ID to get file and line number
	parts := strings.Split(ruleID, ":")
	if len(parts) != 2 {
		return fmt.Errorf("invalid rule ID format")
	}

	fileName := parts[0]
	filePath := filepath.Join("config", characterID, "pickit", fileName)

	// Read file
	content, err := os.ReadFile(filePath)
	if err != nil {
		return fmt.Errorf("failed to read pickit file: %w", err)
	}

	// Split into lines and remove the specific line
	lines := strings.Split(string(content), "\n")
	lineNum, _ := fmt.Sscanf(parts[1], "%d")
	if lineNum > 0 && lineNum <= len(lines) {
		lines = append(lines[:lineNum-1], lines[lineNum:]...)
	}

	// Write back
	if err := os.WriteFile(filePath, []byte(strings.Join(lines, "\n")), 0644); err != nil {
		return fmt.Errorf("failed to delete from pickit file: %w", err)
	}

	return nil
}

func (api *PickitAPI) getCharacterFiles(characterID string) ([]pickit.PickitFile, error) {
	files := []pickit.PickitFile{}

	// Get character's pickit directory
	pickitDir := filepath.Join("config", characterID, "pickit")

	entries, err := os.ReadDir(pickitDir)
	if err != nil {
		return files, err
	}

	for _, entry := range entries {
		if !entry.IsDir() && strings.HasSuffix(entry.Name(), ".nip") {
			info, _ := entry.Info()
			files = append(files, pickit.PickitFile{
				ID:           entry.Name(),
				Name:         entry.Name(), // Keep full filename including .nip extension
				CharacterID:  characterID,
				FilePath:     filepath.Join(pickitDir, entry.Name()),
				LastModified: info.ModTime().Format("2006-01-02 15:04:05"),
			})
		}
	}

	return files, nil
}

// getPickitFilesFromPath returns all .nip files from a custom directory path
func (api *PickitAPI) getPickitFilesFromPath(pickitDir string) ([]pickit.PickitFile, error) {
	files := []pickit.PickitFile{}

	entries, err := os.ReadDir(pickitDir)
	if err != nil {
		return files, err
	}

	for _, entry := range entries {
		if !entry.IsDir() && strings.HasSuffix(entry.Name(), ".nip") {
			info, _ := entry.Info()
			files = append(files, pickit.PickitFile{
				ID:           entry.Name(),
				Name:         entry.Name(),
				CharacterID:  "", // Not applicable when using custom path
				FilePath:     filepath.Join(pickitDir, entry.Name()),
				LastModified: info.ModTime().Format("2006-01-02 15:04:05"),
			})
		}
	}

	return files, nil
}

// loadPickitFileRulesFromPath loads rules from a specific file in a custom directory
func (api *PickitAPI) loadPickitFileRulesFromPath(pickitDir, fileName string) ([]pickit.PickitRule, error) {
	rules := []pickit.PickitRule{}

	filePath := filepath.Join(pickitDir, fileName)

	// Check if file exists
	fileInfo, err := os.Stat(filePath)
	if os.IsNotExist(err) {
		return rules, fmt.Errorf("file does not exist: %s", filePath)
	}
	if err != nil {
		return rules, fmt.Errorf("failed to stat file: %w", err)
	}

	// Log file size for debugging
	log.Printf("Loading pickit file: %s (size: %d bytes)", filePath, fileInfo.Size())

	// Read file content
	content, err := os.ReadFile(filePath)
	if err != nil {
		return rules, fmt.Errorf("failed to read file: %w", err)
	}

	// Parse NIP rules (basic parsing for now)
	lines := strings.Split(string(content), "\n")
	validLineCount := 0
	for i, line := range lines {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, "//") {
			continue
		}

		validLineCount++
		rule := pickit.PickitRule{
			ID:           fmt.Sprintf("%s_%d", fileName, i),
			FileName:     fileName,
			GeneratedNIP: line,
			Enabled:      true,
		}
		rules = append(rules, rule)
	}

	log.Printf("Loaded %d valid rules from %s", validLineCount, filePath)
	return rules, nil
}

// handleDeleteFileRule deletes a specific rule from a loaded .nip file
func (api *PickitAPI) handleDeleteFileRule(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		api.sendError(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	pickitPath := r.URL.Query().Get("path")
	characterID := r.URL.Query().Get("character")
	fileName := r.URL.Query().Get("file")
	ruleID := r.URL.Query().Get("id")

	if fileName == "" || ruleID == "" {
		api.sendError(w, "Missing required parameters: file, id", http.StatusBadRequest)
		return
	}

	// Parse rule ID (format: filename_lineNumber)
	parts := strings.Split(ruleID, "_")
	if len(parts) < 2 {
		api.sendError(w, "Invalid rule ID format", http.StatusBadRequest)
		return
	}

	lineNum := 0
	fmt.Sscanf(parts[len(parts)-1], "%d", &lineNum)
	if lineNum == 0 {
		api.sendError(w, "Invalid line number in rule ID", http.StatusBadRequest)
		return
	}

	// Determine file path
	var filePath string
	if pickitPath != "" {
		filePath = filepath.Join(pickitPath, fileName)
	} else if characterID != "" {
		filePath = filepath.Join("config", characterID, "pickit", fileName)
	} else {
		api.sendError(w, "Either 'path' or 'character' parameter required", http.StatusBadRequest)
		return
	}

	// Read the file
	content, err := os.ReadFile(filePath)
	if err != nil {
		api.sendError(w, fmt.Sprintf("Failed to read file: %v", err), http.StatusInternalServerError)
		return
	}

	// Split into lines
	lines := strings.Split(string(content), "\n")

	// Check if line number is valid
	if lineNum < 0 || lineNum >= len(lines) {
		api.sendError(w, "Line number out of range", http.StatusBadRequest)
		return
	}

	// Remove the line
	newLines := append(lines[:lineNum], lines[lineNum+1:]...)

	// Write back to file
	newContent := strings.Join(newLines, "\n")
	if err := os.WriteFile(filePath, []byte(newContent), 0644); err != nil {
		api.sendError(w, fmt.Sprintf("Failed to write file: %v", err), http.StatusInternalServerError)
		return
	}

	log.Printf("Deleted rule at line %d from file %s", lineNum, fileName)

	api.sendJSON(w, map[string]interface{}{
		"success": true,
		"message": "Rule deleted successfully",
	})
}

// handleUpdateFileRule updates a specific rule in a loaded .nip file
func (api *PickitAPI) handleUpdateFileRule(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		api.sendError(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	pickitPath := r.URL.Query().Get("path")
	characterID := r.URL.Query().Get("character")
	fileName := r.URL.Query().Get("file")
	ruleID := r.URL.Query().Get("id")

	if fileName == "" || ruleID == "" {
		api.sendError(w, "Missing required parameters: file, id", http.StatusBadRequest)
		return
	}

	// Read the new rule content from request body
	body, err := io.ReadAll(r.Body)
	if err != nil {
		api.sendError(w, "Failed to read request body", http.StatusBadRequest)
		return
	}

	var updateData struct {
		NewNIPLine string `json:"newNipLine"`
	}
	if err := json.Unmarshal(body, &updateData); err != nil {
		api.sendError(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	if updateData.NewNIPLine == "" {
		api.sendError(w, "New NIP line cannot be empty", http.StatusBadRequest)
		return
	}

	// Parse rule ID (format: filename_lineNumber)
	parts := strings.Split(ruleID, "_")
	if len(parts) < 2 {
		api.sendError(w, "Invalid rule ID format", http.StatusBadRequest)
		return
	}

	lineNum := 0
	fmt.Sscanf(parts[len(parts)-1], "%d", &lineNum)
	if lineNum == 0 {
		api.sendError(w, "Invalid line number in rule ID", http.StatusBadRequest)
		return
	}

	// Determine file path
	var filePath string
	if pickitPath != "" {
		filePath = filepath.Join(pickitPath, fileName)
	} else if characterID != "" {
		filePath = filepath.Join("config", characterID, "pickit", fileName)
	} else {
		api.sendError(w, "Either 'path' or 'character' parameter required", http.StatusBadRequest)
		return
	}

	// Read the file
	content, err := os.ReadFile(filePath)
	if err != nil {
		api.sendError(w, fmt.Sprintf("Failed to read file: %v", err), http.StatusInternalServerError)
		return
	}

	// Split into lines
	lines := strings.Split(string(content), "\n")

	// Check if line number is valid
	if lineNum < 0 || lineNum >= len(lines) {
		api.sendError(w, "Line number out of range", http.StatusBadRequest)
		return
	}

	// Update the line
	lines[lineNum] = updateData.NewNIPLine

	// Write back to file
	newContent := strings.Join(lines, "\n")
	if err := os.WriteFile(filePath, []byte(newContent), 0644); err != nil {
		api.sendError(w, fmt.Sprintf("Failed to write file: %v", err), http.StatusInternalServerError)
		return
	}

	log.Printf("Updated rule at line %d in file %s", lineNum, fileName)

	api.sendJSON(w, map[string]interface{}{
		"success": true,
		"message": "Rule updated successfully",
	})
}

// handleAppendNIPLine appends a raw NIP line to a file (bypasses validation)
func (api *PickitAPI) handleAppendNIPLine(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		api.sendError(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	pickitPath := r.URL.Query().Get("path")
	characterID := r.URL.Query().Get("character")
	fileName := r.URL.Query().Get("file")

	if fileName == "" {
		api.sendError(w, "Missing required parameter: file", http.StatusBadRequest)
		return
	}

	// Read the NIP line from request body
	body, err := io.ReadAll(r.Body)
	if err != nil {
		api.sendError(w, "Failed to read request body", http.StatusBadRequest)
		return
	}

	var requestData struct {
		NIPLine string `json:"nipLine"`
	}
	if err := json.Unmarshal(body, &requestData); err != nil {
		api.sendError(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	if requestData.NIPLine == "" {
		api.sendError(w, "NIP line cannot be empty", http.StatusBadRequest)
		return
	}

	// Determine pickit directory
	var pickitDir string
	if pickitPath != "" {
		pickitDir = pickitPath
	} else if characterID != "" {
		pickitDir = filepath.Join("config", characterID, "pickit")
	} else {
		api.sendError(w, "Either 'path' or 'character' parameter required", http.StatusBadRequest)
		return
	}

	// Ensure directory exists
	if err := os.MkdirAll(pickitDir, 0755); err != nil {
		api.sendError(w, fmt.Sprintf("Failed to create directory: %v", err), http.StatusInternalServerError)
		return
	}

	filePath := filepath.Join(pickitDir, fileName)

	// Read existing file or create new
	var content string
	if data, err := os.ReadFile(filePath); err == nil {
		content = string(data)
	}

	// Append new rule
	if content != "" && !strings.HasSuffix(content, "\n") {
		content += "\n"
	}
	content += requestData.NIPLine + "\n"

	// Write back to file
	if err := os.WriteFile(filePath, []byte(content), 0644); err != nil {
		api.sendError(w, fmt.Sprintf("Failed to write file: %v", err), http.StatusInternalServerError)
		return
	}

	log.Printf("Appended NIP line to file %s for character %s", fileName, characterID)

	api.sendJSON(w, map[string]interface{}{
		"success": true,
		"message": fmt.Sprintf("Rule appended to %s", fileName),
	})
}

// handleValidateNIPLine validates a raw NIP line syntax (simple validation)
func (api *PickitAPI) handleValidateNIPLine(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		api.sendError(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	// Read the NIP line from request body
	body, err := io.ReadAll(r.Body)
	if err != nil {
		api.sendError(w, "Failed to read request body", http.StatusBadRequest)
		return
	}

	var requestData struct {
		NIPLine string `json:"nipLine"`
	}
	if err := json.Unmarshal(body, &requestData); err != nil {
		api.sendError(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	nipLine := strings.TrimSpace(requestData.NIPLine)

	// Basic validation
	valid := true
	errors := []string{}
	warnings := []string{}

	// Check if line is empty
	if nipLine == "" {
		valid = false
		errors = append(errors, "NIP line cannot be empty")
	}

	// Check if it's just a comment
	if strings.HasPrefix(nipLine, "//") {
		valid = false
		errors = append(errors, "Line is just a comment, no rule defined")
	}

	// Check for basic NIP syntax patterns
	if valid {
		// Should contain at least one [property]
		if !strings.Contains(nipLine, "[") || !strings.Contains(nipLine, "]") {
			errors = append(errors, "No property found. NIP rules must contain at least one [property]")
			valid = false
		}

		// Should contain an operator
		hasOperator := strings.Contains(nipLine, "==") ||
			strings.Contains(nipLine, "!=") ||
			strings.Contains(nipLine, ">=") ||
			strings.Contains(nipLine, "<=") ||
			strings.Contains(nipLine, ">") ||
			strings.Contains(nipLine, "<")

		if !hasOperator {
			errors = append(errors, "No operator found. Use ==, !=, >=, <=, >, or <")
			valid = false
		}

		// Common properties
		commonProps := []string{"name", "type", "quality", "sockets", "defense", "flag"}
		hasCommonProp := false
		for _, prop := range commonProps {
			if strings.Contains(nipLine, "["+prop+"]") {
				hasCommonProp = true
				break
			}
		}

		if !hasCommonProp {
			warnings = append(warnings, "No common properties found. Make sure property names are correct.")
		}

		// Check for maxquantity syntax
		if strings.Contains(nipLine, "maxquantity") && !strings.Contains(nipLine, "# #") {
			warnings = append(warnings, "maxquantity should be after # # delimiter")
		}

		// Check for stats after #
		parts := strings.Split(nipLine, "#")
		if len(parts) > 1 && len(parts[1]) > 0 {
			// Has right side (stats)
			rightSide := strings.TrimSpace(parts[1])
			if rightSide != "" && !strings.HasPrefix(rightSide, "#") {
				// Contains stats, good
				warnings = append(warnings, "Rule has stat requirements on right side (after #)")
			}
		}
	}

	log.Printf("Validated NIP line: %s - Valid: %v", nipLine, valid)

	api.sendJSON(w, map[string]interface{}{
		"valid":    valid,
		"errors":   errors,
		"warnings": warnings,
		"nipLine":  nipLine,
	})
}

// handleBrowseFolder allows browsing for a folder using native Windows dialog
// handleBrowseFolder opens a native Windows folder browser dialog
func (api *PickitAPI) handleBrowseFolder(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		api.sendError(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	// Open native Windows folder browser
	selectedPath, err := utils.BrowseForFolder("Select Pickit Folder")
	if err != nil {
		api.sendJSON(w, map[string]interface{}{
			"path":  "",
			"error": fmt.Sprintf("Error opening folder browser: %v", err),
		})
		return
	}

	// If user cancelled (empty path), return without error
	if selectedPath == "" {
		api.sendJSON(w, map[string]interface{}{
			"path":      "",
			"cancelled": true,
		})
		return
	}

	// Validate that the selected path exists and is a directory
	fileInfo, err := os.Stat(selectedPath)
	if err != nil {
		api.sendJSON(w, map[string]interface{}{
			"path":  "",
			"error": fmt.Sprintf("Selected path does not exist: %v", err),
		})
		return
	}

	if !fileInfo.IsDir() {
		api.sendJSON(w, map[string]interface{}{
			"path":  "",
			"error": "Selected path is not a directory",
		})
		return
	}

	// Return the selected path
	log.Printf("User selected pickit folder: %s", selectedPath)
	api.sendJSON(w, map[string]interface{}{
		"path":    selectedPath,
		"success": true,
	})
}
