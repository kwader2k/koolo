package delivery

import (
	"context"
	"errors"
	"log/slog"
	"sync"
	"time"
)

// Request represents a single delivery request issued for a supervisor.
type Request struct {
	RoomName  string
	Password  string
	CreatedAt time.Time
}

// ErrInterrupt is used to interrupt the current run when a delivery is requested.
var ErrInterrupt = errors.New("delivery requested")

// Callbacks holds hooks used by runner/server layers.
type Callbacks struct {
	OnComplete     func(supervisorName string)
	OnResult       func(supervisorName, room, result string, itemsDelivered int, duration time.Duration, errorMsg string)
	OnClearRequest func(supervisorName string)
}

// Manager tracks all delivery runtime state for a single supervisor.
type Manager struct {
	name   string
	logger *slog.Logger

	mu      sync.Mutex
	filters *ContextFilters
	pending *Request
	active  *Request
	cbs     Callbacks

	ctx    context.Context
	cancel context.CancelFunc
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

// RequestDelivery enqueues a new delivery request or updates an existing pending one.
func (m *Manager) RequestDelivery(room, passwd string) *Request {
	m.mu.Lock()
	defer m.mu.Unlock()

	// Always re-apply latest filters to ensure correct quotas before delivery
	if m.filters != nil {
		m.filters.UpdateFilters(m.filters.filters)
	}

	if m.pending != nil {
		m.pending.RoomName = room
		m.pending.Password = passwd
		return m.pending
	}

	m.pending = &Request{
		RoomName:  room,
		Password:  passwd,
		CreatedAt: time.Now(),
	}

	return m.pending
}

// HasPending reports whether there is a pending delivery request.
func (m *Manager) HasPending() bool {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.pending != nil
}

// SetActive marks the given request as the currently active delivery.
func (m *Manager) SetActive(req *Request) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.active = req
}

// Pending returns the currently pending delivery request, if any.
func (m *Manager) Pending() *Request {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.pending
}

// Active returns the currently active delivery request, if any.
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

	if m.pending == req {
		m.pending = nil
		if m.cbs.OnClearRequest != nil {
			m.cbs.OnClearRequest(m.name)
		}
	}

	if m.active == req {
		m.active = nil
		if m.cbs.OnComplete != nil {
			if m.logger != nil {
				m.logger.Info("Delivery complete - clearing individual filters", "supervisor", m.name)
			}
			m.cbs.OnComplete(m.name)
		} else if m.logger != nil {
			m.logger.Warn("Delivery complete but OnComplete callback is nil", "supervisor", m.name)
		}
		if m.cbs.OnClearRequest != nil {
			m.cbs.OnClearRequest(m.name)
		}
	}
}

// SetCallbacks configures callbacks used for delivery completion and result reporting.
func (m *Manager) SetCallbacks(cbs Callbacks) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.cbs = cbs
}

// ReportResult notifies listeners that a single delivery has finished.
func (m *Manager) ReportResult(room, result string, itemsDelivered int, duration time.Duration, errorMsg string) {
	m.mu.Lock()
	cbs := m.cbs
	m.mu.Unlock()

	if cbs.OnResult != nil {
		cbs.OnResult(m.name, room, result, itemsDelivered, duration, errorMsg)
	}
}

// Filter helpers ----------------------------------------------------------------

// ShouldDeliverItem reports whether the given item name should be delivered under current filters.
func (m *Manager) ShouldDeliverItem(name string) bool {
	if m == nil || m.filters == nil {
		return false
	}
	return m.filters.ShouldDeliverItem(name)
}

// HasRemainingDeliveryQuota reports whether there is remaining quota for the given item.
func (m *Manager) HasRemainingDeliveryQuota(name string) bool {
	if m == nil || m.filters == nil {
		return true
	}
	return m.filters.HasRemainingDeliveryQuota(name)
}

// ResetDeliveredItemCounts resets per-item delivered counters for the current run.
func (m *Manager) ResetDeliveredItemCounts() {
	if m == nil || m.filters == nil {
		return
	}
	m.filters.ResetDeliveredItemCounts()
}

// RecordDeliveredItem increments the delivered count for the given item.
func (m *Manager) RecordDeliveredItem(name string) {
	if m == nil || m.filters == nil {
		return
	}
	m.filters.RecordDeliveredItem(name)
}

// GetDeliveredItemCount returns how many of the given item have been delivered so far.
func (m *Manager) GetDeliveredItemCount(name string) int {
	if m == nil || m.filters == nil {
		return 0
	}
	return m.filters.GetDeliveredItemCount(name)
}

// DeliverOnlySelected reports whether "deliver only selected items" mode is enabled.
func (m *Manager) DeliverOnlySelected() bool {
	if m == nil || m.filters == nil {
		return false
	}
	return m.filters.DeliverOnlySelected()
}

// DeliveryFiltersEnabled reports whether delivery filters are enabled.
func (m *Manager) DeliveryFiltersEnabled() bool {
	if m == nil || m.filters == nil {
		return false
	}
	return m.filters.DeliveryFiltersEnabled()
}

// GetDeliveryItemQuantity returns the configured max delivery quantity for the given item.
func (m *Manager) GetDeliveryItemQuantity(itemName string) int {
	if m == nil || m.filters == nil {
		return 0
	}
	return m.filters.GetDeliveryItemQuantity(itemName)
}

// HasDeliveryQuotaLimits reports whether any item has a finite delivery quota.
func (m *Manager) HasDeliveryQuotaLimits() bool {
	if m == nil || m.filters == nil {
		return false
	}
	return m.filters.HasDeliveryQuotaLimits()
}

// AreDeliveryQuotasSatisfied reports whether all configured finite quotas have been satisfied.
func (m *Manager) AreDeliveryQuotasSatisfied() bool {
	if m == nil || m.filters == nil {
		return false
	}
	return m.filters.AreDeliveryQuotasSatisfied()
}

// Interrupt context helpers -----------------------------------------------------

// StartWatch creates a new cancellable context used to detect delivery interrupts.
func (m *Manager) StartWatch() {
	if m == nil {
		return
	}

	m.mu.Lock()
	defer m.mu.Unlock()
	m.ctx, m.cancel = context.WithCancel(context.Background())
}

// TriggerInterrupt cancels the internal context, asking the game loop to stop as soon as possible.
func (m *Manager) TriggerInterrupt() {
	if m == nil {
		return
	}

	m.mu.Lock()
	defer m.mu.Unlock()
	if m.cancel != nil {
		if m.logger != nil {
			m.logger.Info("Triggering delivery interrupt via context cancellation")
		}
		m.cancel()
	}
}

// ResetContext creates a new delivery context after a run completes.
func (m *Manager) ResetContext() {
	if m == nil {
		return
	}

	m.mu.Lock()
	defer m.mu.Unlock()
	if m.cancel != nil {
		m.cancel()
	}
	m.ctx, m.cancel = context.WithCancel(context.Background())
}
