package drop

import (
	"strings"
	"sync"

	"github.com/hectorgimenez/d2go/pkg/data/item"
)

// ItemQuantity represents an item name together with an optional max Drop quota.
// Quantity == 0 means "unlimited" so any matching item can be Droppered.
type ItemQuantity struct {
	Name     string `json:"name"`
	Quantity int    `json:"quantity"` // 0 means unlimited
}

// Filters holds Drop preferences (filters/quotas) shared between UI/server and bot runtime.
// It defines which runes/gems/custom items are considered Dropperable and in what mode.
type Filters struct {
	Enabled             bool           `json:"enabled"`
	DropperOnlySelected bool           `json:"DropperOnlySelected"`
	SelectedRunes       []ItemQuantity `json:"selectedRunes"`
	SelectedGems        []ItemQuantity `json:"selectedGems"`
	SelectedKeyTokens   []ItemQuantity `json:"selectedKeyTokens"`
	CustomItems         []string       `json:"customItems"`      // Legacy: simple names without quantity information
	AllowedQualities    []string       `json:"allowedQualities"` // e.g., base, magic, rare, set, unique, crafted, runeword
}

// Normalize trims whitespace, removes empty values and duplicates, and returns
// a normalized Filters value. The Enabled flag is preserved as-is.
func (f Filters) Normalize() Filters {
	// Keep Enabled flag as-is
	f.SelectedRunes = normalizeItemQuantities(f.SelectedRunes)
	f.SelectedGems = normalizeItemQuantities(f.SelectedGems)
	f.SelectedKeyTokens = normalizeItemQuantities(f.SelectedKeyTokens)
	f.CustomItems = normalizeList(f.CustomItems)
	f.AllowedQualities = normalizeList(f.AllowedQualities)
	return f
}

// BuildSet collects the names of all selected items (lowercased) and returns
// a map usable for fast membership checks. Note: quantity information is
// discarded here; use GetItemQuantity to inspect quota limits.
func (f Filters) BuildSet() map[string]struct{} {
	set := make(map[string]struct{})
	addQuantities := func(items []ItemQuantity) {
		for _, item := range items {
			if item.Name == "" {
				continue
			}
			set[strings.ToLower(item.Name)] = struct{}{}
		}
	}
	addStrings := func(items []string) {
		for _, item := range items {
			if item == "" {
				continue
			}
			set[strings.ToLower(item)] = struct{}{}
		}
	}

	addQuantities(f.SelectedRunes)
	addQuantities(f.SelectedGems)
	addQuantities(f.SelectedKeyTokens)
	addStrings(f.CustomItems)
	return set
}

// GetItemQuantity returns the configured max Drop quantity for the given
// item name. If there is no matching configuration, it returns 0 (unlimited).
func (f Filters) GetItemQuantity(itemName string) int {
	lowerName := strings.ToLower(itemName)

	for _, item := range f.SelectedRunes {
		if strings.ToLower(item.Name) == lowerName {
			return item.Quantity
		}
	}

	for _, item := range f.SelectedGems {
		if strings.ToLower(item.Name) == lowerName {
			return item.Quantity
		}
	}
	for _, item := range f.SelectedKeyTokens {
		if strings.ToLower(item.Name) == lowerName {
			return item.Quantity
		}
	}

	// CustomItems don't have quantity limits
	return 0
}

// ContextFilters stores runtime state used when evaluating Drop filters.
type ContextFilters struct {
	mu        sync.RWMutex
	filters   Filters
	filterSet map[string]struct{}
	Droppered map[string]int
}

var runeNames = map[string]struct{}{
	"elrune": {}, "eldrune": {}, "tirrune": {}, "nefrune": {}, "ethrune": {}, "ithrune": {}, "talrune": {}, "ralrune": {},
	"ortrune": {}, "thulrune": {}, "amnrune": {}, "solrune": {}, "shaelrune": {}, "dolrune": {}, "helrune": {}, "iorune": {},
	"lumrune": {}, "korune": {}, "falrune": {}, "lemrune": {}, "pulrune": {}, "umrune": {}, "malrune": {}, "istrune": {},
	"gulrune": {}, "vexrune": {}, "ohmrune": {}, "lorune": {}, "surrune": {}, "berrune": {}, "jahrune": {}, "chamrune": {},
	"zodrune": {},
}

var gemNames = map[string]struct{}{
	"perfectamethyst": {}, "perfectdiamond": {}, "perfectemerald": {}, "perfectruby": {}, "perfectsapphire": {}, "perfecttopaz": {}, "perfectskull": {},
}

var keyTokenNames = map[string]struct{}{
	"keyofterror":       {},
	"keyofhate":         {},
	"keyofdestruction":  {},
	"tokenofabsolution": {},
}

// type codes for any gem/rune tier
var gemTypes = map[string]struct{}{
	item.TypeAmethyst: {},
	item.TypeDiamond:  {},
	item.TypeEmerald:  {},
	item.TypeRuby:     {},
	item.TypeSapphire: {},
	item.TypeTopaz:    {},
	item.TypeSkull:    {},
	item.TypeQuest:    {}, // quest items should never be Droppered via quality-only filters
}

func isRuneOrGemType(t string) bool {
	l := strings.ToLower(t)
	if l == strings.ToLower(item.TypeRune) {
		return true
	}
	if _, ok := gemTypes[l]; ok {
		return true
	}
	return false
}

// NewContextFilters initializes an empty filter state for a single supervisor.
func NewContextFilters() *ContextFilters {
	return &ContextFilters{
		filters:   Filters{},
		filterSet: make(map[string]struct{}),
		Droppered: make(map[string]int),
	}
}

// UpdateFilters replaces the stored Filters with the provided value.
func (s *ContextFilters) UpdateFilters(filters Filters) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.filters = filters.Normalize()
	s.filterSet = s.filters.BuildSet()
}

// ShouldDropperItem reports whether the given item name/type/quality should be Droppered.
func (s *ContextFilters) ShouldDropperItem(name string, quality item.Quality, itemType string, isRuneword bool) bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	if !s.filters.Enabled || s.filterSet == nil {
		return false
	}

	lowerName := strings.ToLower(name)
	nameMatch := false
	if _, ok := s.filterSet[lowerName]; ok {
		nameMatch = true
	}

	qualityMatch := false
	if len(s.filters.AllowedQualities) > 0 {
		if isRuneword {
			if s.qualityGroupAllowed("runeword") {
				qualityMatch = true
			}
		} else if s.qualityAllowed(quality) && !isRuneOrGem(name) && !isRuneOrGemType(itemType) {
			qualityMatch = true
		}
	}
	return nameMatch || qualityMatch
}

// HasRemainingDropQuota reports whether the item has not yet reached its configured quota.
func (s *ContextFilters) HasRemainingDropQuota(name string) bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	maxQty := s.filters.GetItemQuantity(name)
	if maxQty <= 0 {
		return true
	}
	return s.Droppered[strings.ToLower(name)] < maxQty
}

// ResetDropperedItemCounts clears all per-item Droppered counters for the current run.
func (s *ContextFilters) ResetDropperedItemCounts() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.Droppered = make(map[string]int)
}

// RecordDropperedItem increments the Droppered count used for quota tracking.
func (s *ContextFilters) RecordDropperedItem(name string) {
	key := strings.ToLower(name)
	s.mu.Lock()
	defer s.mu.Unlock()
	if maxQty := s.filters.GetItemQuantity(name); maxQty > 0 {
		if s.Droppered == nil {
			s.Droppered = make(map[string]int)
		}
		s.Droppered[key] = s.Droppered[key] + 1
	}
}

// GetDropperedItemCount returns how many of the named item have been Droppered so far.
func (s *ContextFilters) GetDropperedItemCount(name string) int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.Droppered[strings.ToLower(name)]
}

// DropperOnlySelected reports whether "Dropper only selected items" mode is enabled.
func (s *ContextFilters) DropperOnlySelected() bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.filters.Enabled && s.filters.DropperOnlySelected
}

// DropFiltersEnabled reports whether Drop filters are enabled in general.
func (s *ContextFilters) DropFiltersEnabled() bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.filters.Enabled
}

// GetDropItemQuantity returns the configured max Drop quantity for the given item.
func (s *ContextFilters) GetDropItemQuantity(itemName string) int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.filters.GetItemQuantity(itemName)
}

// HasDropQuotaLimits reports whether at least one item has a finite Drop quota.
func (s *ContextFilters) HasDropQuotaLimits() bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	if !s.filters.Enabled {
		return false
	}
	for _, item := range s.filters.SelectedRunes {
		if item.Quantity > 0 {
			return true
		}
	}
	for _, item := range s.filters.SelectedGems {
		if item.Quantity > 0 {
			return true
		}
	}
	for _, item := range s.filters.SelectedKeyTokens {
		if item.Quantity > 0 {
			return true
		}
	}
	return false
}

// AreDropQuotasSatisfied reports whether all finite quotas are satisfied (no more items to Dropper).
func (s *ContextFilters) AreDropQuotasSatisfied() bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	if !s.filters.Enabled {
		return false
	}
	hasFinite := false
	for _, item := range s.filters.SelectedRunes {
		if item.Quantity <= 0 {
			continue
		}
		hasFinite = true
		if s.Droppered[strings.ToLower(item.Name)] < item.Quantity {
			return false
		}
	}
	for _, item := range s.filters.SelectedGems {
		if item.Quantity <= 0 {
			continue
		}
		hasFinite = true
		if s.Droppered[strings.ToLower(item.Name)] < item.Quantity {
			return false
		}
	}
	for _, item := range s.filters.SelectedKeyTokens {
		if item.Quantity <= 0 {
			continue
		}
		hasFinite = true
		if s.Droppered[strings.ToLower(item.Name)] < item.Quantity {
			return false
		}
	}
	return hasFinite
}

func (s *ContextFilters) qualityAllowed(q item.Quality) bool {
	if len(s.filters.AllowedQualities) == 0 {
		return true
	}
	group := qualityToGroup(q)
	if group == "" {
		return false
	}
	return s.qualityGroupAllowed(group)
}

func (s *ContextFilters) qualityGroupAllowed(group string) bool {
	for _, allowed := range s.filters.AllowedQualities {
		if strings.EqualFold(allowed, group) {
			return true
		}
	}
	return false
}

func qualityToGroup(q item.Quality) string {
	switch q {
	case item.QualityNormal, item.QualitySuperior:
		return "base"
	case item.QualityMagic:
		return "magic"
	case item.QualityRare:
		return "rare"
	case item.QualitySet:
		return "set"
	case item.QualityUnique:
		return "unique"
	case item.QualityCrafted:
		return "crafted"
	default:
		return ""
	}
}

func isRuneOrGem(name string) bool {
	l := strings.ToLower(name)
	if _, ok := runeNames[l]; ok {
		return true
	}
	if _, ok := gemNames[l]; ok {
		return true
	}
	if _, ok := keyTokenNames[l]; ok {
		return true
	}
	return false
}

// normalizeItemQuantities trims names, removes empties and duplicates,
// and clamps negative quantities to 0 (unlimited).
func normalizeItemQuantities(values []ItemQuantity) []ItemQuantity {
	seen := make(map[string]struct{}, len(values))
	norm := make([]ItemQuantity, 0, len(values))
	for _, v := range values {
		name := strings.TrimSpace(v.Name)
		if name == "" {
			continue
		}
		if _, ok := seen[name]; ok {
			continue
		}
		seen[name] = struct{}{}
		v.Name = name
		if v.Quantity < 0 {
			v.Quantity = 0
		}
		norm = append(norm, v)
	}
	return norm
}

// normalizeList trims names, removes empties and duplicates, and returns the normalized slice.
func normalizeList(values []string) []string {
	seen := make(map[string]struct{}, len(values))
	norm := make([]string, 0, len(values))
	for _, v := range values {
		name := strings.TrimSpace(v)
		if name == "" {
			continue
		}
		if _, ok := seen[name]; ok {
			continue
		}
		seen[name] = struct{}{}
		norm = append(norm, name)
	}

	return norm
}
