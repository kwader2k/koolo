package delivery

import (
	"strings"
	"sync"
)

// ItemQuantity represents an item name together with an optional max delivery quota.
// Quantity == 0 means "unlimited" so any matching item can be delivered.
type ItemQuantity struct {
	Name     string `json:"name"`
	Quantity int    `json:"quantity"` // 0 means unlimited/all (무제한)
}

// Filters holds delivery preferences (filters/quotas) shared between UI/server and bot runtime.
// It defines which runes/gems/custom items are considered deliverable and in what mode.
type Filters struct {
	Enabled             bool           `json:"enabled"`
	DeliverOnlySelected bool           `json:"deliverOnlySelected"`
	SelectedRunes       []ItemQuantity `json:"selectedRunes"`
	SelectedGems        []ItemQuantity `json:"selectedGems"`
	CustomItems         []string       `json:"customItems"` // Legacy: simple names without quantity information
}

// Normalize trims whitespace, removes empty values and duplicates, and returns
// a normalized Filters value. The Enabled flag is preserved as-is.
func (f Filters) Normalize() Filters {
	// Keep Enabled flag as-is
	f.SelectedRunes = normalizeItemQuantities(f.SelectedRunes)
	f.SelectedGems = normalizeItemQuantities(f.SelectedGems)
	f.CustomItems = normalizeList(f.CustomItems)
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
	addStrings(f.CustomItems)
	return set
}

// GetItemQuantity returns the configured max delivery quantity for the given
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

	// CustomItems don't have quantity limits
	return 0
}

// ContextFilters stores runtime state used when evaluating delivery filters.
type ContextFilters struct {
	mu        sync.RWMutex
	filters   Filters
	filterSet map[string]struct{}
	delivered map[string]int
}

// NewContextFilters initializes an empty filter state for a single supervisor.
func NewContextFilters() *ContextFilters {
	return &ContextFilters{
		filters:   Filters{},
		filterSet: make(map[string]struct{}),
		delivered: make(map[string]int),
	}
}

// UpdateFilters replaces the stored Filters with the provided value.
func (s *ContextFilters) UpdateFilters(filters Filters) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.filters = filters.Normalize()
	s.filterSet = s.filters.BuildSet()
}

// ShouldDeliverItem reports whether the given item name is in the selected set.
func (s *ContextFilters) ShouldDeliverItem(name string) bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	if !s.filters.Enabled || s.filterSet == nil {
		return false
	}
	_, ok := s.filterSet[strings.ToLower(name)]
	return ok
}

// HasRemainingDeliveryQuota reports whether the item has not yet reached its configured quota.
func (s *ContextFilters) HasRemainingDeliveryQuota(name string) bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	maxQty := s.filters.GetItemQuantity(name)
	if maxQty <= 0 {
		return true
	}
	return s.delivered[strings.ToLower(name)] < maxQty
}

// ResetDeliveredItemCounts clears all per-item delivered counters for the current run.
func (s *ContextFilters) ResetDeliveredItemCounts() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.delivered = make(map[string]int)
}

// RecordDeliveredItem increments the delivered count used for quota tracking.
func (s *ContextFilters) RecordDeliveredItem(name string) {
	key := strings.ToLower(name)
	s.mu.Lock()
	defer s.mu.Unlock()
	if maxQty := s.filters.GetItemQuantity(name); maxQty > 0 {
		if s.delivered == nil {
			s.delivered = make(map[string]int)
		}
		s.delivered[key] = s.delivered[key] + 1
	}
}

// GetDeliveredItemCount returns how many of the named item have been delivered so far.
func (s *ContextFilters) GetDeliveredItemCount(name string) int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.delivered[strings.ToLower(name)]
}

// DeliverOnlySelected reports whether "deliver only selected items" mode is enabled.
func (s *ContextFilters) DeliverOnlySelected() bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.filters.Enabled && s.filters.DeliverOnlySelected
}

// DeliveryFiltersEnabled reports whether delivery filters are enabled in general.
func (s *ContextFilters) DeliveryFiltersEnabled() bool {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.filters.Enabled
}

// GetDeliveryItemQuantity returns the configured max delivery quantity for the given item.
func (s *ContextFilters) GetDeliveryItemQuantity(itemName string) int {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.filters.GetItemQuantity(itemName)
}

// HasDeliveryQuotaLimits reports whether at least one item has a finite delivery quota.
func (s *ContextFilters) HasDeliveryQuotaLimits() bool {
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
	return false
}

// AreDeliveryQuotasSatisfied reports whether all finite quotas are satisfied (no more items to deliver).
func (s *ContextFilters) AreDeliveryQuotasSatisfied() bool {
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
		if s.delivered[strings.ToLower(item.Name)] < item.Quantity {
			return false
		}
	}
	for _, item := range s.filters.SelectedGems {
		if item.Quantity <= 0 {
			continue
		}
		hasFinite = true
		if s.delivered[strings.ToLower(item.Name)] < item.Quantity {
			return false
		}
	}
	return hasFinite
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
