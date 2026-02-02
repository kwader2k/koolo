package drop

import (
	"errors"
	"log/slog"
	"strings"
	"sync"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data/item"
)

// Request represents a single Drop request issued for a supervisor.
type Request struct {
	RoomName  string
	Password  string
	Filters   Filters
	CreatedAt time.Time
	CardID    int
	CardName  string
}

// ErrInterrupt is used to interrupt the current run when a Drop is requested.
var ErrInterrupt = errors.New("Drop requested")

// Callbacks holds hooks used by runner/server layers.
type Callbacks struct {
	OnComplete     func(supervisorName string)
	OnResult       func(supervisorName, room, result string, itemsDroppered int, duration time.Duration, errorMsg string, filters Filters)
	OnClearRequest func(supervisorName string)
}

// Manager tracks all Drop runtime state for a single supervisor.
type Manager struct {
	name   string
	logger *slog.Logger

	mu      sync.Mutex
	filters *ContextFilters
	pending []*Request
	active  *Request
	cbs     Callbacks
}

// NewManager creates a new Manager with empty filter state.
func NewManager(name string, logger *slog.Logger) *Manager {
	return &Manager{
		name:    name,
		logger:  logger,
		filters: NewContextFilters(),
	}
}

// UpdateFilters replaces the current filter configuration with the provided Filters.
func (m *Manager) UpdateFilters(filters Filters) {
	if m == nil || m.filters == nil {
		return
	}
	m.filters.UpdateFilters(filters)
}

// RequestDrop enqueues a new Drop request or updates an existing pending one.
func (m *Manager) RequestDrop(room, passwd string) *Request {
	m.mu.Lock()
	defer m.mu.Unlock()

	var currentFilters Filters
	if m.filters != nil {
		m.filters.mu.RLock()
		currentFilters = m.filters.filters
		m.filters.mu.RUnlock()
		// Always re-apply latest filters to ensure correct quotas before Drop
		m.filters.UpdateFilters(currentFilters)
	}

	req := &Request{
		RoomName:  room,
		Password:  passwd,
		Filters:   currentFilters,
		CreatedAt: time.Now(),
	}

	m.pending = append(m.pending, req)
	return req
}

// SetActive marks the given request as the currently active Drop.
func (m *Manager) SetActive(req *Request) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.active = req
}

// Pending returns the currently pending Drop request, if any.
func (m *Manager) Pending() *Request {
	m.mu.Lock()
	defer m.mu.Unlock()
	if len(m.pending) == 0 {
		return nil
	}
	return m.pending[0]
}

// HasPendingRequests reports whether there is any pending Drop request.
func (m *Manager) HasPendingRequests() bool {
	m.mu.Lock()
	defer m.mu.Unlock()
	if len(m.pending) == 0 || m.pending[0] == nil {
		return false
	}
	return strings.TrimSpace(m.pending[0].RoomName) != ""
}

// Active returns the currently active Drop request, if any.
func (m *Manager) Active() *Request {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.active
}

// ClearRequest removes the given request from pending/active state and triggers completion callbacks when appropriate.
func (m *Manager) ClearRequest(req *Request) {
	if m == nil {
		return
	}

	m.mu.Lock()
	defer m.mu.Unlock()

	if len(m.pending) > 0 && m.pending[0] == req {
		if len(m.pending) == 1 {
			m.pending = nil
		} else {
			m.pending = m.pending[1:]
		}
		if m.cbs.OnClearRequest != nil {
			m.cbs.OnClearRequest(m.name)
		}
	}

	if m.active == req {
		m.active = nil
		if m.cbs.OnComplete != nil {
			if m.logger != nil {
				m.logger.Info("Drop complete - clearing individual filters", "supervisor", m.name)
			}
			m.cbs.OnComplete(m.name)
		} else if m.logger != nil {
			m.logger.Warn("Drop complete but OnComplete callback is nil", "supervisor", m.name)
		}
		if m.cbs.OnClearRequest != nil {
			m.cbs.OnClearRequest(m.name)
		}
	}
}

// SetCallbacks configures callbacks used for Drop completion and result reporting.
func (m *Manager) SetCallbacks(cbs Callbacks) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.cbs = cbs
}

// ReportResult notifies listeners that a single Drop has finished.
func (m *Manager) ReportResult(room, result string, itemsDroppered int, duration time.Duration, errorMsg string, filters Filters) {
	m.mu.Lock()
	cbs := m.cbs
	m.mu.Unlock()

	if cbs.OnResult != nil {
		cbs.OnResult(m.name, room, result, itemsDroppered, duration, errorMsg, filters)
	}
}

// Filter helpers ----------------------------------------------------------------

// ShouldDropperItem reports whether the given item name/type/quality should be Droppered under current filters.
func (m *Manager) ShouldDropperItem(name string, quality item.Quality, itemType string, isRuneword bool) bool {
	if m == nil || m.filters == nil {
		return false
	}
	return m.filters.ShouldDropperItem(name, quality, itemType, isRuneword)
}

// HasRemainingDropQuota reports whether there is remaining quota for the given item.
func (m *Manager) HasRemainingDropQuota(name string) bool {
	if m == nil || m.filters == nil {
		return true
	}
	return m.filters.HasRemainingDropQuota(name)
}

// ResetDropperedItemCounts resets per-item Droppered counters for the current run.
func (m *Manager) ResetDropperedItemCounts() {
	if m == nil || m.filters == nil {
		return
	}
	m.filters.ResetDropperedItemCounts()
}

// RecordDropperedItem increments the Droppered count for the given item.
func (m *Manager) RecordDropperedItem(name string) {
	if m == nil || m.filters == nil {
		return
	}
	m.filters.RecordDropperedItem(name)
}

// GetDropperedItemCount returns how many of the given item have been Droppered so far.
func (m *Manager) GetDropperedItemCount(name string) int {
	if m == nil || m.filters == nil {
		return 0
	}
	return m.filters.GetDropperedItemCount(name)
}

// DropperOnlySelected reports whether "Dropper only selected items" mode is enabled.
func (m *Manager) DropperOnlySelected() bool {
	if m == nil || m.filters == nil {
		return false
	}
	return m.filters.DropperOnlySelected()
}

// DropFiltersEnabled reports whether Drop filters are enabled.
func (m *Manager) DropFiltersEnabled() bool {
	if m == nil || m.filters == nil {
		return false
	}
	return m.filters.DropFiltersEnabled()
}

// GetDropItemQuantity returns the configured max Drop quantity for the given item.
func (m *Manager) GetDropItemQuantity(itemName string) int {
	if m == nil || m.filters == nil {
		return 0
	}
	return m.filters.GetDropItemQuantity(itemName)
}

// HasDropQuotaLimits reports whether any item has a finite Drop quota.
func (m *Manager) HasDropQuotaLimits() bool {
	if m == nil || m.filters == nil {
		return false
	}
	return m.filters.HasDropQuotaLimits()
}

// AreDropQuotasSatisfied reports whether all configured finite quotas have been satisfied.
func (m *Manager) AreDropQuotasSatisfied() bool {
	if m == nil || m.filters == nil {
		return false
	}
	return m.filters.AreDropQuotasSatisfied()
}
