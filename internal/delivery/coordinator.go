package delivery

import (
	"log/slog"
	"sync"
	"time"
)

// Coordinator manages delivery filter state and server callbacks across supervisors.
// It acts as a lightweight orchestrator on top of per-supervisor Managers.
type Coordinator struct {
	logger *slog.Logger

	// Per-supervisor delivery filter configuration (including optional "global")
	filters   map[string]Filters
	filtersMu sync.RWMutex

	// Callback to notify server when individual filters are cleared
	clearServerFilter func(supervisor string)

	// Callback to clear persistent delivery request
	clearPersistentRequest func(supervisor string)

	// Callback to report delivery run result back to server
	onDeliveryResult func(supervisorName, room, result string, itemsDelivered int, duration time.Duration, errorMsg string)
}

// NewCoordinator creates a new Coordinator and initializes its filter map.
func NewCoordinator(logger *slog.Logger) *Coordinator {
	return &Coordinator{
		logger:  logger,
		filters: make(map[string]Filters),
	}
}

// SetFilters updates the per-supervisor delivery filters and
// applies them immediately to the provided Manager if it is running.
func (c *Coordinator) SetFilters(supervisor string, filters Filters, mgr *Manager) {
	c.filtersMu.Lock()
	c.filters[supervisor] = filters
	c.filtersMu.Unlock()

	if mgr == nil {
		return
	}

	mgr.UpdateFilters(filters)
}

// GetFilters returns the filter configuration for the given supervisor.
func (c *Coordinator) GetFilters(supervisor string) (Filters, bool) {
	c.filtersMu.RLock()
	defer c.filtersMu.RUnlock()

	f, ok := c.filters[supervisor]
	return f, ok
}

// SetClearServerFilterCallback registers a callback to clear server-side filter state.
func (c *Coordinator) SetClearServerFilterCallback(callback func(supervisor string)) {
	c.clearServerFilter = callback
}

// SetClearPersistentRequestCallback registers a callback to clear persistent delivery requests.
func (c *Coordinator) SetClearPersistentRequestCallback(callback func(supervisor string)) {
	c.clearPersistentRequest = callback
}

// SetDeliveryResultCallback registers a callback used to report delivery results to the server.
func (c *Coordinator) SetDeliveryResultCallback(callback func(supervisorName, room, result string, itemsDelivered int, duration time.Duration, errorMsg string)) {
	c.onDeliveryResult = callback
}

// ConfigureCallbacks wires OnComplete/OnResult callbacks into the given Manager.
func (c *Coordinator) ConfigureCallbacks(supervisorName string, mgr *Manager) {
	if mgr == nil {
		return
	}

	mgr.SetCallbacks(Callbacks{
		OnComplete:     c.ClearIndividualFilters,
		OnResult:       c.onDeliveryResult,
		OnClearRequest: c.clearPersistentRequest,
	})
}

// ApplyInitialFilters applies any stored (or default) filters to a new Manager instance.
func (c *Coordinator) ApplyInitialFilters(supervisorName string, mgr *Manager) {
	if mgr == nil {
		return
	}

	c.filtersMu.RLock()
	filters, ok := c.filters[supervisorName]
	c.filtersMu.RUnlock()

	if ok {
		mgr.UpdateFilters(filters)
	} else {
		mgr.UpdateFilters(Filters{DeliverOnlySelected: true})
	}
}

// ClearIndividualFilters clears per-supervisor filters and applies global ones if they exist.
func (c *Coordinator) ClearIndividualFilters(supervisor string) {
	if c.logger != nil {
		c.logger.Info("Clearing individual delivery filters", "supervisor", supervisor)
	}

	c.filtersMu.Lock()
	defer c.filtersMu.Unlock()

	// Remove individual filter for this supervisor
	delete(c.filters, supervisor)
	if c.logger != nil {
		c.logger.Info("Individual filter removed from coordinator", "supervisor", supervisor)
	}

	// Notify server to clear its copy too
	if c.clearServerFilter != nil {
		c.clearServerFilter(supervisor)
	}

	// Note: global filters (key "global") remain in the map; callers that
	// care about re-applying them to running supervisors should query
	// filters via GetFilters and apply as needed.
}
