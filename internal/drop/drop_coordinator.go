package drop

import (
	"log/slog"
	"sync"
	"time"
)

// Coordinator manages Drop filter state and server callbacks across supervisors.
// It acts as a lightweight orchestrator on top of per-supervisor Managers.
type Coordinator struct {
	logger *slog.Logger

	// Per-supervisor Drop filter configuration.
	filters   map[string]Filters
	filtersMu sync.RWMutex

	// Callback to notify server when individual filters are cleared
	clearServerFilter func(supervisor string)

	// Callback to clear persistent Drop request
	clearPersistentRequest func(supervisor string)

	// Callback to report Drop run result back to server
	onDropResult func(supervisorName, room, result string, itemsDroppered int, duration time.Duration, errorMsg string, filters Filters)
}

// NewCoordinator creates a new Coordinator and initializes its filter map.
func NewCoordinator(logger *slog.Logger) *Coordinator {
	return &Coordinator{
		logger:  logger,
		filters: make(map[string]Filters),
	}
}

// SetFilters updates the per-supervisor Drop filters and
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

// SetClearServerFilterCallback registers a callback to clear server-side filter state.
func (c *Coordinator) SetClearServerFilterCallback(callback func(supervisor string)) {
	c.clearServerFilter = callback
}

// SetClearPersistentRequestCallback registers a callback to clear persistent Drop requests.
func (c *Coordinator) SetClearPersistentRequestCallback(callback func(supervisor string)) {
	c.clearPersistentRequest = callback
}

// SetDropResultCallback registers a callback used to report Drop results to the server.
func (c *Coordinator) SetDropResultCallback(callback func(supervisorName, room, result string, itemsDroppered int, duration time.Duration, errorMsg string, filters Filters)) {
	c.onDropResult = callback
}

// ConfigureCallbacks wires OnComplete/OnResult callbacks into the given Manager.
func (c *Coordinator) ConfigureCallbacks(supervisorName string, mgr *Manager) {
	if mgr == nil {
		return
	}

	mgr.SetCallbacks(Callbacks{
		OnComplete:     c.ClearIndividualFilters,
		OnResult:       c.onDropResult,
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
		mgr.UpdateFilters(Filters{DropperOnlySelected: true})
	}
}

// ClearIndividualFilters disables per-supervisor filters and leaves selections intact.
func (c *Coordinator) ClearIndividualFilters(supervisor string) {
	if c.logger != nil {
		c.logger.Info("Disabling individual Drop filters after completion", "supervisor", supervisor)
	}

	c.filtersMu.Lock()
	defer c.filtersMu.Unlock()

	// Disable (but keep) individual filter for this supervisor
	if filters, ok := c.filters[supervisor]; ok {
		filters.Enabled = false
		c.filters[supervisor] = filters
		if c.logger != nil {
			c.logger.Info("Individual filter disabled (selections preserved)", "supervisor", supervisor)
		}
	}

	// Notify server to disable its copy too
	if c.clearServerFilter != nil {
		c.clearServerFilter(supervisor)
	}

	// Note: filters are per-supervisor only.
}
