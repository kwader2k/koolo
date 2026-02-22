package server

import (
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"net/url"
	"slices"
	"sort"
	"strings"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/pickit"
)

type RunewordHistoryEntry struct {
	Timestamp     time.Time `json:"timestamp"`
	Supervisor    string    `json:"supervisor"`
	Runeword      string    `json:"runeword"`
	Attempt       int       `json:"attempt"`
	TargetStats   string    `json:"targetStats"`
	ActualStats   string    `json:"actualStats"`
	Success       bool      `json:"success"`
	FailureReason string    `json:"failureReason"`
}

func (s *HttpServer) HandleRunewordHistory(_ context.Context, e event.Event) error {
	evt, ok := e.(event.RunewordRerollEvent)
	if !ok {
		return nil
	}

	entry := RunewordHistoryEntry{
		Timestamp:     evt.OccurredAt(),
		Supervisor:    evt.Supervisor(),
		Runeword:      evt.Runeword,
		TargetStats:   evt.TargetStats,
		ActualStats:   evt.ActualStats,
		Success:       evt.Success,
		FailureReason: evt.FailureReason,
	}
	s.appendRunewordHistory(entry)
	return nil
}

func (s *HttpServer) appendRunewordHistory(entry RunewordHistoryEntry) {
	s.RunewordMux.Lock()
	defer s.RunewordMux.Unlock()

	attempt := 1
	for _, existing := range s.RunewordHistory {
		if existing.Supervisor == entry.Supervisor && existing.Runeword == entry.Runeword {
			if existing.Attempt >= attempt {
				attempt = existing.Attempt + 1
			}
		}
	}
	entry.Attempt = attempt

	s.RunewordHistory = append([]RunewordHistoryEntry{entry}, s.RunewordHistory...)
	if len(s.RunewordHistory) > 200 {
		s.RunewordHistory = s.RunewordHistory[:200]
	}
}

func (s *HttpServer) getRunewordHistory(name string, supervisor string) []RunewordHistoryEntry {
	s.RunewordMux.Lock()
	defer s.RunewordMux.Unlock()

	history := make([]RunewordHistoryEntry, 0, len(s.RunewordHistory))
	for _, entry := range s.RunewordHistory {
		if name != "" && entry.Runeword != name {
			continue
		}
		if supervisor != "" && entry.Supervisor != supervisor {
			continue
		}
		history = append(history, entry)
	}
	return history
}

func (s *HttpServer) runewordHistory(w http.ResponseWriter, r *http.Request) {
	name := r.URL.Query().Get("name")
	supervisor := r.URL.Query().Get("supervisor")
	history := s.getRunewordHistory(name, supervisor)

	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(history)
}

// runewordRolls returns the rollable stats for a given runeword, based on action.Runewords.
// It is used by the runeword settings UI to populate target stat selectors.
func (s *HttpServer) runewordRolls(w http.ResponseWriter, r *http.Request) {
	name := r.URL.Query().Get("name")
	if name == "" {
		http.Error(w, "missing runeword name", http.StatusBadRequest)
		return
	}

	type rollDTO struct {
		StatID int     `json:"statId"`
		Label  string  `json:"label"`
		Min    float64 `json:"min"`
		Max    float64 `json:"max"`
		Layer  int     `json:"layer"`
		Group  string  `json:"group,omitempty"`
	}

	var rolls []rollDTO
	for _, rw := range action.Runewords {
		if string(rw.Name) != name {
			continue
		}

		uiRolls := action.BuildRunewordUIRolls(rw)
		for _, ur := range uiRolls {
			rolls = append(rolls, rollDTO{
				StatID: int(ur.StatID),
				Label:  ur.Label,
				Min:    ur.Min,
				Max:    ur.Max,
				Layer:  ur.Layer,
				Group:  ur.Group,
			})
		}

		break
	}

	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(rolls); err != nil {
		http.Error(w, "failed to encode response", http.StatusInternalServerError)
	}
}

// runewordBaseTypes returns the allowed base item types for a given runeword.
// It is used by the runeword settings UI to populate the Base type selector.
func (s *HttpServer) runewordBaseTypes(w http.ResponseWriter, r *http.Request) {
	name := r.URL.Query().Get("name")
	if name == "" {
		http.Error(w, "missing runeword name", http.StatusBadRequest)
		return
	}

	type baseTypeDTO struct {
		Code  string `json:"code"`
		Label string `json:"label"`
	}

	// Find the matching runeword recipe so we can take into account
	// the number of runes (sockets) required when deciding which base
	// item types are actually usable.
	var recipe *action.Runeword
	for i := range action.Runewords {
		if string(action.Runewords[i].Name) == name {
			recipe = &action.Runewords[i]
			break
		}
	}

	var bases []baseTypeDTO
	if recipe == nil {
		// If no recipe is found, return an empty list rather than erroring.
		w.Header().Set("Content-Type", "application/json")
		_ = json.NewEncoder(w).Encode(bases)
		return
	}

	requiredSockets := len(recipe.Runes)

	seen := make(map[string]bool)

	for _, bt := range recipe.BaseItemTypes {
		if bt == "" || seen[bt] {
			continue
		}

		// Only expose this base type if there exists at least one d2go
		// item with matching type code and enough sockets to hold the
		// runeword. This avoids showing types like Club/Wand/etc. for
		// 5-socket runewords where no actual 5-socket base exists.
		hasUsableBase := false
		for _, desc := range item.Desc {
			if desc.Type != bt {
				continue
			}
			if requiredSockets > 0 && desc.MaxSockets < requiredSockets {
				continue
			}
			hasUsableBase = true
			break
		}
		if !hasUsableBase {
			continue
		}

		seen[bt] = true
		label := action.PrettyRunewordBaseTypeLabel(bt)
		bases = append(bases, baseTypeDTO{
			Code:  bt,
			Label: label,
		})
	}

	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(bases); err != nil {
		http.Error(w, "failed to encode response", http.StatusInternalServerError)
	}
}

// runewordBases returns concrete base item names for a given runeword, base type, and tier.
// This is used to populate the Base name selector in the runeword settings UI.
func (s *HttpServer) runewordBases(w http.ResponseWriter, r *http.Request) {
	name := r.URL.Query().Get("name")
	baseTypeCode := r.URL.Query().Get("baseType")
	tier := strings.ToLower(r.URL.Query().Get("tier"))

	if name == "" {
		http.Error(w, "missing runeword name", http.StatusBadRequest)
		return
	}

	type baseItemDTO struct {
		Code string `json:"code"`
		Name string `json:"name"`
	}

	var result []baseItemDTO

	// Find the matching runeword recipe so we can infer default base types
	// and required sockets directly from d2go data.
	var recipe *action.Runeword
	for i := range action.Runewords {
		if string(action.Runewords[i].Name) == name {
			recipe = &action.Runewords[i]
			break
		}
	}
	if recipe == nil {
		// If we cannot find the recipe, return an empty list rather than erroring.
		w.Header().Set("Content-Type", "application/json")
		_ = json.NewEncoder(w).Encode(result)
		return
	}

	// Determine which item type codes are allowed. If a specific base type
	// is selected, we only use that; otherwise, we use all recipe base types.
	allowedTypeCodes := make([]string, 0)
	if baseTypeCode != "" {
		allowedTypeCodes = append(allowedTypeCodes, baseTypeCode)
	} else {
		allowedTypeCodes = append(allowedTypeCodes, recipe.BaseItemTypes...)
	}

	// Deduplicate type codes
	{
		seen := make(map[string]bool)
		unique := make([]string, 0, len(allowedTypeCodes))
		for _, code := range allowedTypeCodes {
			if code == "" || seen[code] {
				continue
			}
			seen[code] = true
			unique = append(unique, code)
		}
		allowedTypeCodes = unique
	}

	// Map tier string to d2go item tier
	var tierFilter item.Tier
	useTierFilter := false
	switch tier {
	case "elite":
		tierFilter = item.TierElite
		useTierFilter = true
	case "exceptional":
		tierFilter = item.TierExceptional
		useTierFilter = true
	case "normal":
		tierFilter = item.TierNormal
		useTierFilter = true
	}

	requiredSockets := len(recipe.Runes)
	seenBases := make(map[string]bool)

	// Build the base list directly from d2go item descriptions.
	for _, desc := range item.Desc {
		// Filter by allowed d2go item type codes (TypeArmor, TypeBow, etc.)
		if len(allowedTypeCodes) > 0 {
			match := false
			for _, code := range allowedTypeCodes {
				if desc.Type == code {
					match = true
					break
				}
			}
			if !match {
				continue
			}
		}

		// Filter by tier if requested.
		if useTierFilter && desc.Tier() != tierFilter {
			continue
		}

		// Ensure the base can support the required number of sockets.
		if requiredSockets > 0 && desc.MaxSockets < requiredSockets {
			continue
		}

		nipName := pickit.ToNIPName(desc.Name)
		if nipName == "" || seenBases[nipName] {
			continue
		}
		seenBases[nipName] = true

		result = append(result, baseItemDTO{
			Code: nipName,
			Name: desc.Name,
		})
	}

	// Sort alphabetically for a stable UI.
	sort.Slice(result, func(i, j int) bool {
		return result[i].Name < result[j].Name
	})

	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(result); err != nil {
		http.Error(w, "failed to encode response", http.StatusInternalServerError)
	}
}

// runewordSettings renders a dedicated per-character runeword configuration page.
func (s *HttpServer) runewordSettings(w http.ResponseWriter, r *http.Request) {
	characterName := r.URL.Query().Get("characterName")
	if characterName == "" {
		http.Error(w, "characterName is required", http.StatusBadRequest)
		return
	}

	cfg, found := config.GetCharacter(characterName)
	if !found || cfg == nil {
		http.Error(w, "character config not found", http.StatusNotFound)
		return
	}

	saved := r.URL.Query().Get("saved") == "1"
	if r.Method == http.MethodPost {
		if err := r.ParseForm(); err != nil {
			s.templates.ExecuteTemplate(w, "runewords.gohtml", CharacterSettings{
				Version:            config.Version,
				Supervisor:         characterName,
				Config:             cfg,
				Saved:              false,
				ErrorMessage:       err.Error(),
				RunewordRecipeList: availableRunewordRecipesForCharacter(cfg),
				RunewordFavoriteRecipes: config.Koolo.RunewordFavoriteRecipes,
				RunewordRuneNames:  buildRunewordRuneNames(),
				RunewordRerollable: buildRunewordRerollable(),
			})
			return
		}

		cfg.CubeRecipes.PrioritizeRunewords = r.Form.Has("rwPrioritizeBeforeCubing")

		// Parse global runeword maker options
		cfg.Game.RunewordMaker.AutoUpgrade = r.Form.Has("rwAutoUpgrade")
		cfg.Game.RunewordMaker.OnlyIfWearable = r.Form.Has("rwOnlyIfWearable")
		cfg.Game.RunewordMaker.AutoTierByDifficulty = r.Form.Has("rwAutoTierByDifficulty")

		if _, ok := r.Form["runewordMakerEnabled"]; ok {
			cfg.Game.RunewordMaker.Enabled = r.Form.Has("runewordMakerEnabled")
		}
		enabledRunewordRecipes := sanitizeEnabledRunewordSelection(r.Form["runewordMakerEnabledRecipes"], cfg)
		cfg.Game.RunewordMaker.EnabledRecipes = enabledRunewordRecipes
		if len(enabledRunewordRecipes) > 0 {
			cfg.Game.RunewordMaker.Enabled = true
		} else {
			cfg.Game.RunewordMaker.Enabled = false
		}
		favoriteRunewordRecipes := sanitizeFavoriteRunewordSelection(r.Form["runewordFavoriteRecipes"])
		visibleRunewords := availableRunewordRecipesForCharacter(cfg)
		visibleSet := make(map[string]struct{}, len(visibleRunewords))
		for _, name := range visibleRunewords {
			visibleSet[name] = struct{}{}
		}
		mergedFavorites := make([]string, 0, len(favoriteRunewordRecipes)+len(config.Koolo.RunewordFavoriteRecipes))
		seenFavorites := make(map[string]struct{}, len(favoriteRunewordRecipes)+len(config.Koolo.RunewordFavoriteRecipes))
		for _, name := range favoriteRunewordRecipes {
			if _, ok := seenFavorites[name]; ok {
				continue
			}
			seenFavorites[name] = struct{}{}
			mergedFavorites = append(mergedFavorites, name)
		}
		for _, name := range config.Koolo.RunewordFavoriteRecipes {
			if _, visible := visibleSet[name]; visible {
				continue
			}
			if _, ok := seenFavorites[name]; ok {
				continue
			}
			seenFavorites[name] = struct{}{}
			mergedFavorites = append(mergedFavorites, name)
		}
		config.Koolo.RunewordFavoriteRecipes = mergedFavorites

		// Parse and save per-runeword overrides into cfg.Game.RunewordOverrides.
		// Currently the UI only edits a single runeword at a time, identified
		// by the hidden rwCurrentName field.
		selectedRuneword := strings.TrimSpace(r.Form.Get("rwCurrentName"))
		if selectedRuneword != "" {
			if !slices.Contains(enabledRunewordRecipes, selectedRuneword) {
				if cfg.Game.RunewordOverrides != nil {
					delete(cfg.Game.RunewordOverrides, selectedRuneword)
				}
				if cfg.Game.RunewordRerollRules != nil {
					delete(cfg.Game.RunewordRerollRules, selectedRuneword)
				}
				goto saveConfig
			}

			if cfg.Game.RunewordOverrides == nil {
				cfg.Game.RunewordOverrides = make(map[string]config.RunewordOverrideConfig)
			}

			override := cfg.Game.RunewordOverrides[selectedRuneword]

			// Basic base / quality preferences for the maker logic
			override.EthMode = strings.TrimSpace(r.Form.Get("rwEthMode"))
			override.QualityMode = strings.TrimSpace(r.Form.Get("rwQualityMode"))
			override.BaseType = strings.TrimSpace(r.Form.Get("rwBaseType"))
			override.BaseTier = strings.TrimSpace(r.Form.Get("rwBaseTier"))
			override.BaseName = strings.TrimSpace(r.Form.Get("rwBaseName"))

			cfg.Game.RunewordOverrides[selectedRuneword] = override

			// Optionally handle reroll rules posted from the maker page.
			rawRules := strings.TrimSpace(r.Form.Get("rrRulesJSON"))
			if rawRules != "" {
				if cfg.Game.RunewordRerollRules == nil {
					cfg.Game.RunewordRerollRules = make(map[string][]config.RunewordRerollRule)
				}

				var rules []config.RunewordRerollRule
				if err := json.Unmarshal([]byte(rawRules), &rules); err != nil {
				s.templates.ExecuteTemplate(w, "runewords.gohtml", CharacterSettings{
					Version:                 config.Version,
					Supervisor:              characterName,
					Config:                  cfg,
					Saved:                   false,
					ErrorMessage:            fmt.Sprintf("failed to parse reroll rules: %v", err),
					RunewordRecipeList:      availableRunewordRecipesForCharacter(cfg),
					RunewordFavoriteRecipes: config.Koolo.RunewordFavoriteRecipes,
					RunewordRuneNames:       buildRunewordRuneNames(),
					RunewordRerollable:      buildRunewordRerollable(),
				})
					return
				}

				if len(rules) == 0 {
					delete(cfg.Game.RunewordRerollRules, selectedRuneword)
				} else {
					cfg.Game.RunewordRerollRules[selectedRuneword] = rules
				}
			} else {
				// Empty payload means clear rules for this runeword
				if cfg.Game.RunewordRerollRules != nil {
					delete(cfg.Game.RunewordRerollRules, selectedRuneword)
				}
			}
		}

	saveConfig:
		if err := config.SaveKooloConfig(config.Koolo); err != nil {
			s.templates.ExecuteTemplate(w, "runewords.gohtml", CharacterSettings{
				Version:                 config.Version,
				Supervisor:              characterName,
				Config:                  cfg,
				Saved:                   false,
				ErrorMessage:            err.Error(),
				RunewordRecipeList:      availableRunewordRecipesForCharacter(cfg),
				RunewordFavoriteRecipes: config.Koolo.RunewordFavoriteRecipes,
				RunewordRuneNames:       buildRunewordRuneNames(),
				RunewordRerollable:      buildRunewordRerollable(),
			})
			return
		}
		if err := config.SaveSupervisorConfig(characterName, cfg); err != nil {
			s.templates.ExecuteTemplate(w, "runewords.gohtml", CharacterSettings{
				Version:                 config.Version,
				Supervisor:              characterName,
				Config:                  cfg,
				Saved:                   false,
				ErrorMessage:            err.Error(),
				RunewordRecipeList:      availableRunewordRecipesForCharacter(cfg),
				RunewordFavoriteRecipes: config.Koolo.RunewordFavoriteRecipes,
				RunewordRuneNames:       buildRunewordRuneNames(),
				RunewordRerollable:      buildRunewordRerollable(),
			})
			return
		}

		http.Redirect(w, r, "/runewords?characterName="+url.QueryEscape(characterName)+"&saved=1", http.StatusSeeOther)
		return
	}

	s.templates.ExecuteTemplate(w, "runewords.gohtml", CharacterSettings{
		Version:                 config.Version,
		Supervisor:              characterName,
		Config:                  cfg,
		Saved:                   saved,
		RunewordRecipeList:      availableRunewordRecipesForCharacter(cfg),
		RunewordFavoriteRecipes: config.Koolo.RunewordFavoriteRecipes,
		RunewordRuneNames:       buildRunewordRuneNames(),
		RunewordRerollable:      buildRunewordRerollable(),
	})
}

// buildRunewordRerollable returns a map of runeword name -> whether
// this runeword actually supports reroll rules (i.e. it has at least
// one rollable stat in Rolls). The UI uses this to hide the reroll
// controls for fixed-stat runewords.
func buildRunewordRerollable() map[string]bool {
	result := make(map[string]bool, len(action.Runewords))

	for _, rw := range action.Runewords {
		// The UI toggles reroll controls based on whether the recipe has rolls.
		result[string(rw.Name)] = len(rw.Rolls) > 0
	}

	return result
}

// buildRunewordRuneNames returns a map from human-readable runeword name
// (e.g. "Call to Arms") to a comma-separated list of rune names in order,
// e.g. "Amn, Ral, Mal, Ist, Ohm".
func buildRunewordRuneNames() map[string]string {
	result := make(map[string]string, len(action.Runewords))

	for _, rw := range action.Runewords {
		if len(rw.Runes) == 0 {
			continue
		}

		labels := make([]string, 0, len(rw.Runes))
		for _, r := range rw.Runes {
			name := r
			if strings.HasSuffix(name, "Rune") {
				name = name[:len(name)-len("Rune")]
			}
			labels = append(labels, name)
		}

		result[string(rw.Name)] = strings.Join(labels, ", ")
	}

	return result
}

var ladderOnlyRunewords = map[string]struct{}{
	string(item.RunewordBulwark):       {},
	string(item.RunewordCure):          {},
	string(item.RunewordGround):        {},
	string(item.RunewordHearth):        {},
	string(item.RunewordTemper):        {},
	string(item.RunewordHustle):        {},
	string(item.RunewordMosaic):        {},
	string(item.RunewordMetamorphosis): {},
}

func availableRunewordRecipesForCharacter(cfg *config.CharacterCfg) []string {
	if cfg == nil {
		return config.AvailableRunewordRecipes
	}

	return filterLadderRunewords(config.AvailableRunewordRecipes, cfg.Game.IsNonLadderChar)
}

func filterLadderRunewords(list []string, isNonLadder bool) []string {
	if !isNonLadder {
		return list
	}

	result := make([]string, 0, len(list))
	for _, name := range list {
		if _, restricted := ladderOnlyRunewords[name]; restricted {
			continue
		}
		result = append(result, name)
	}
	return result
}

func sanitizeEnabledRunewordSelection(selected []string, cfg *config.CharacterCfg) []string {
	if cfg == nil {
		return selected
	}
	return filterLadderRunewords(selected, cfg.Game.IsNonLadderChar)
}

func sanitizeFavoriteRunewordSelection(selected []string) []string {
	allowed := make(map[string]struct{}, len(config.AvailableRunewordRecipes))
	for _, name := range config.AvailableRunewordRecipes {
		allowed[name] = struct{}{}
	}

	result := make([]string, 0, len(selected))
	seen := make(map[string]struct{}, len(selected))
	for _, name := range selected {
		if _, ok := allowed[name]; !ok {
			continue
		}
		if _, ok := seen[name]; ok {
			continue
		}
		seen[name] = struct{}{}
		result = append(result, name)
	}

	return result
}
