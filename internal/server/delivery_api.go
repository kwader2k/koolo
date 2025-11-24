package server

import (
	"encoding/json"
	"fmt"
	"net/http"
	"strings"
	"time"

	"github.com/hectorgimenez/koolo/internal/delivery"
)

type scheduledDelivery struct {
	Supervisor string
	Room       string
	Password   string
	NotBefore  time.Time
	CreatedAt  time.Time
}

// DeliveryRequest represents a single delivery request.
type DeliveryRequest struct {
	RoomName string `json:"room"`
	Password string `json:"password"`
}

// DeliveryStatusResponse is the full response for the delivery status API.
type DeliveryStatusResponse struct {
	Supervisors []SupervisorDeliveryStatus  `json:"supervisors"`
	Queue       []DeliveryQueueEntry        `json:"queue"`
	History     []DeliveryHistoryEntry      `json:"history"`
	Filters     map[string]delivery.Filters `json:"filters"`
}

// SupervisorDeliveryStatus describes the delivery state of a single supervisor.
type SupervisorDeliveryStatus struct {
	Name     string    `json:"name"`
	State    string    `json:"state"`
	Room     string    `json:"room"`
	Password string    `json:"password"`
	Since    time.Time `json:"since"`
	Running  bool      `json:"running"`
}

// DeliveryQueueEntry is a queue entry shown in the UI.
type DeliveryQueueEntry struct {
	Supervisor string `json:"supervisor"`
	Room       string `json:"room"`
	Status     string `json:"status"`
	Attempts   int    `json:"attempts"`
	NextAction string `json:"nextAction"`
}

// DeliveryHistoryEntry is a single entry in the delivery history.
type DeliveryHistoryEntry struct {
	Supervisor     string    `json:"supervisor"`
	Room           string    `json:"room"`
	FilterApplied  string    `json:"filterApplied"` // "None", "Global", "Individual"
	FilterMode     string    `json:"filterMode"`    // "Exclusive", "Inclusive", "-"
	Result         string    `json:"result"`        // "Success", "Failed", "Timeout"
	ItemsDelivered int       `json:"itemsDelivered"`
	Duration       string    `json:"duration"` // "45s", "1m12s"
	ErrorMessage   string    `json:"errorMessage"`
	Timestamp      time.Time `json:"timestamp"`
}

// DeliveryBatchRequest requests deliveries for multiple supervisors at once.
type DeliveryBatchRequest struct {
	Supervisors  []string `json:"supervisors"`
	RoomName     string   `json:"room"`
	Password     string   `json:"password"`
	DelaySeconds int      `json:"delaySeconds"`
}

// DeliveryCancelRequest cancels an in-flight or pending delivery for a supervisor.
type DeliveryCancelRequest struct {
	Supervisor string `json:"supervisor"`
}

// initDeliveryCallbacks wires delivery-related callbacks from the delivery service
// into the HTTP server, so results and filter state are reflected in the UI.
func (s *HttpServer) initDeliveryCallbacks() {
	s.manager.DeliveryService().SetClearServerFilterCallback(s.onDeliveryClearFilters)
	s.manager.DeliveryService().SetClearPersistentRequestCallback(s.onDeliveryClearPersistentRequest)
	s.manager.DeliveryService().SetDeliveryResultCallback(s.onDeliveryResult)
}

// onDeliveryClearFilters is invoked when a delivery finishes and per-supervisor
// filters should be cleared on the server side.
func (s *HttpServer) onDeliveryClearFilters(supervisor string) {
	s.deliveryMux.Lock()
	delete(s.deliveryFilters, supervisor)
	s.deliveryMux.Unlock()
}

// onDeliveryClearPersistentRequest is invoked when a delivery request is cleared
// to remove it from persistent storage.
func (s *HttpServer) onDeliveryClearPersistentRequest(supervisor string) {
	s.manager.DeliveryService().ClearPersistentRequest(supervisor)
}

// onDeliveryResult is invoked when a delivery run finishes so that the
// result can be recorded in the in-memory history for the UI.
func (s *HttpServer) onDeliveryResult(supervisorName, room, result string, itemsDelivered int, duration time.Duration, errorMsg string) {
	// Determine which filter configuration was applied
	filterApplied := "-"
	filterMode := "-"

	s.deliveryMux.Lock()
	if filters, exists := s.deliveryFilters[supervisorName]; exists {
		filterApplied = "Individual"
		if filters.DeliverOnlySelected {
			filterMode = "Exclusive"
		} else {
			filterMode = "Inclusive"
		}
	} else if globalFilters, exists := s.deliveryFilters["global"]; exists && globalFilters.Enabled {
		filterApplied = "Global"
		if globalFilters.DeliverOnlySelected {
			filterMode = "Exclusive"
		} else {
			filterMode = "Inclusive"
		}
	}
	s.deliveryMux.Unlock()

	// Format duration nicely for the UI
	durationStr := fmt.Sprintf("%.1fs", duration.Seconds())
	if duration.Minutes() >= 1 {
		durationStr = fmt.Sprintf("%dm%ds", int(duration.Minutes()), int(duration.Seconds())%60)
	}

	s.appendDeliveryHistory(DeliveryHistoryEntry{
		Supervisor:     supervisorName,
		Room:           room,
		FilterApplied:  filterApplied,
		FilterMode:     filterMode,
		Result:         result,
		ItemsDelivered: itemsDelivered,
		Duration:       durationStr,
		ErrorMessage:   errorMsg,
		Timestamp:      time.Now(),
	})
}

func (s *HttpServer) deliveryManagerPage(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/html; charset=utf-8")

	err := s.templates.ExecuteTemplate(w, "delivery_manager.gohtml", nil)
	if err != nil {
		s.logger.Error("Failed to execute delivery_manager template", "error", err)
		http.Error(w, fmt.Sprintf("Template error: %v", err), http.StatusInternalServerError)
		return
	}
}

func (s *HttpServer) registerDeliveryRoutes() {
	http.HandleFunc("/api/delivery/request", s.handleDelivery)
	http.HandleFunc("/api/delivery/status", s.handleDeliveryStatus)
	http.HandleFunc("/api/delivery/batch", s.handleDeliveryBatch)
	http.HandleFunc("/api/delivery/start-deliver", s.handleDeliveryStartDeliver)
	http.HandleFunc("/api/delivery/cancel", s.handleDeliveryCancel)
	http.HandleFunc("/api/delivery/protection", s.handleDeliveryFilters)
	http.HandleFunc("/api/delivery/filters", s.handleDeliveryFilters)
}

func (s *HttpServer) appendDeliveryHistory(entry DeliveryHistoryEntry) {
	s.deliveryMux.Lock()
	defer s.deliveryMux.Unlock()

	s.deliveryHistory = append([]DeliveryHistoryEntry{entry}, s.deliveryHistory...)
	if len(s.deliveryHistory) > 100 {
		s.deliveryHistory = s.deliveryHistory[:100]
	}
}

func (s *HttpServer) rememberDeliveryRequest(supervisor, room, password, result string) {

	s.appendDeliveryHistory(DeliveryHistoryEntry{
		Supervisor:     supervisor,
		Room:           room,
		FilterApplied:  "-",
		FilterMode:     "-",
		Result:         result,
		ItemsDelivered: 0,
		Duration:       "-",
		ErrorMessage:   "",
		Timestamp:      time.Now(),
	})
}

func (s *HttpServer) getDeliveryHistory() []DeliveryHistoryEntry {
	s.deliveryMux.Lock()
	defer s.deliveryMux.Unlock()
	history := make([]DeliveryHistoryEntry, len(s.deliveryHistory))
	copy(history, s.deliveryHistory)
	return history
}

func (s *HttpServer) getScheduledDeliveries() []scheduledDelivery {
	return nil
}

func (s *HttpServer) getDeliveryFilters(supervisor string) delivery.Filters {
	s.deliveryMux.Lock()
	defer s.deliveryMux.Unlock()
	if filters, ok := s.deliveryFilters[supervisor]; ok {
		return filters.Normalize()
	}

	if global, ok := s.deliveryFilters["global"]; ok && global.Enabled {
		return global.Normalize()
	}

	return delivery.Filters{DeliverOnlySelected: false}.Normalize()
}

func (s *HttpServer) setDeliveryFilters(supervisor string, p delivery.Filters) delivery.Filters {
	s.deliveryMux.Lock()
	defer s.deliveryMux.Unlock()
	s.deliveryFilters[supervisor] = p.Normalize()
	return s.deliveryFilters[supervisor]
}

func (s *HttpServer) getAllDeliveryFilters() map[string]delivery.Filters {
	s.deliveryMux.Lock()
	defer s.deliveryMux.Unlock()
	result := make(map[string]delivery.Filters, len(s.deliveryFilters))
	for k, v := range s.deliveryFilters {
		result[k] = v
	}
	return result
}

func (s *HttpServer) submitDeliveryRequest(supervisor, room, password string) error {
	sup := s.manager.GetSupervisor(supervisor)
	if sup == nil {
		return fmt.Errorf("unknown supervisor %s", supervisor)
	}

	ctx := sup.GetContext()
	if ctx == nil {
		return fmt.Errorf("failed to get context for %s", supervisor)
	}

	if ctx.Delivery == nil {
		ctx.Delivery = delivery.NewManager(ctx.Name, ctx.Logger)
	}
	// Always apply latest filters before delivery request
	ctx.Delivery.UpdateFilters(s.getDeliveryFilters(supervisor))
	req := ctx.Delivery.RequestDelivery(room, password)
	ctx.Logger.Info("Delivery request queued", "supervisor", supervisor, "room", room)

	// Store persistent request for 10 minutes
	s.manager.DeliveryService().StorePersistentRequest(supervisor, req)

	ctx.Delivery.StartWatch()
	ctx.Delivery.TriggerInterrupt()

	s.rememberDeliveryRequest(supervisor, room, password, "queued")
	return nil
}

func (s *HttpServer) buildSupervisorStatus(name string) SupervisorDeliveryStatus {
	state := "offline"
	room := ""
	password := ""
	var since time.Time
	running := false

	sup := s.manager.GetSupervisor(name)
	if sup != nil {
		running = true
		state = "idle"
		ctx := sup.GetContext()
		if ctx != nil {
			if ctx.Delivery != nil {
				if active := ctx.Delivery.Active(); active != nil {
					state = "active"
					room = active.RoomName
					password = active.Password
					since = active.CreatedAt
				} else if pending := ctx.Delivery.Pending(); pending != nil {
					room = pending.RoomName
					password = pending.Password
					since = pending.CreatedAt
					state = "pending"
				}
			}
		}
	}
	return SupervisorDeliveryStatus{
		Name:     name,
		State:    state,
		Room:     room,
		Password: password,
		Since:    since,
		Running:  running,
	}
}

func (s *HttpServer) handleDelivery(w http.ResponseWriter, r *http.Request) {
	var req DeliveryRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "invalid JSON", http.StatusBadRequest)
		return
	}

	supervisor := r.URL.Query().Get("supervisor")
	if supervisor == "" {
		http.Error(w, "supervisor is required", http.StatusBadRequest)
		return
	}

	if err := s.submitDeliveryRequest(supervisor, req.RoomName, req.Password); err != nil {
		if strings.Contains(err.Error(), "unknown supervisor") {
			http.Error(w, err.Error(), http.StatusNotFound)
			return
		}
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	w.WriteHeader(http.StatusOK)
	_, _ = w.Write([]byte("delivery request queued"))
}

func (s *HttpServer) handleDeliveryStatus(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	response := DeliveryStatusResponse{
		Supervisors: []SupervisorDeliveryStatus{},
		Queue:       []DeliveryQueueEntry{},
		History:     s.getDeliveryHistory(),
		Filters:     s.getAllDeliveryFilters(),
	}

	for _, name := range s.manager.AvailableSupervisors() {
		status := s.buildSupervisorStatus(name)
		response.Supervisors = append(response.Supervisors, status)

		switch status.State {
		case "pending", "active":
			entry := DeliveryQueueEntry{
				Supervisor: status.Name,
				Room:       status.Room,
				Status:     status.State,
				Attempts:   1,
			}
			switch status.State {
			case "pending":
				entry.NextAction = "waiting for next slot"
			default:
				entry.NextAction = "processing delivery"
			}
			response.Queue = append(response.Queue, entry)
		}
	}

	for _, sched := range s.getScheduledDeliveries() {
		eta := time.Until(sched.NotBefore)
		if eta < 0 {
			eta = 0
		}
		response.Queue = append(response.Queue, DeliveryQueueEntry{
			Supervisor: sched.Supervisor,
			Room:       sched.Room,
			Status:     "scheduled",
			NextAction: fmt.Sprintf("starts in %s", eta.Truncate(time.Second)),
		})
	}

	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(response)
}

func (s *HttpServer) handleDeliveryBatch(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req DeliveryBatchRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "invalid JSON", http.StatusBadRequest)
		return
	}

	if len(req.Supervisors) == 0 {
		http.Error(w, "no supervisors provided", http.StatusBadRequest)
		return
	}

	if req.RoomName == "" {
		http.Error(w, "room name is required", http.StatusBadRequest)
		return
	}

	valid := make([]string, 0, len(req.Supervisors))
	var failed []string
	for _, name := range req.Supervisors {
		if s.manager.GetSupervisor(name) == nil {
			failed = append(failed, fmt.Sprintf("%s: unknown supervisor", name))
			continue
		}
		valid = append(valid, name)
	}

	if len(valid) == 0 {
		http.Error(w, strings.Join(failed, "; "), http.StatusBadRequest)
		return
	}

	delaySeconds := req.DelaySeconds
	if delaySeconds < 0 {
		delaySeconds = 0
	}

	for i, name := range valid {
		if err := s.submitDeliveryRequest(name, req.RoomName, req.Password); err != nil {
			failed = append(failed, fmt.Sprintf("%s: %v", name, err))
		}

		// Apply sequential delay between deliveries (skip for first delivery)
		if i < len(valid)-1 && delaySeconds > 0 {
			time.Sleep(time.Duration(delaySeconds) * time.Second)
		}
	}

	if len(failed) > 0 {
		http.Error(w, strings.Join(failed, "; "), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(map[string]string{"status": "queued"})
}

func (s *HttpServer) handleDeliveryStartDeliver(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req DeliveryRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "invalid JSON", http.StatusBadRequest)
		return
	}

	supervisor := r.URL.Query().Get("supervisor")
	if supervisor == "" {
		http.Error(w, "supervisor is required", http.StatusBadRequest)
		return
	}

	if req.RoomName == "" {
		http.Error(w, "room name is required", http.StatusBadRequest)
		return
	}

	if sup := s.manager.GetSupervisor(supervisor); sup != nil {
		if err := s.submitDeliveryRequest(supervisor, req.RoomName, req.Password); err != nil {
			http.Error(w, err.Error(), http.StatusInternalServerError)
			return
		}
	} else {
		if svc := s.manager.DeliveryService(); svc != nil {
			svc.QueueStartDelivery(supervisor, req.RoomName, req.Password)
		}
	}

	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(map[string]string{"status": "queued"})
}

func (s *HttpServer) handleDeliveryCancel(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req DeliveryCancelRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "invalid JSON", http.StatusBadRequest)
		return
	}

	sup := s.manager.GetSupervisor(req.Supervisor)
	if sup == nil {
		http.Error(w, "unknown supervisor", http.StatusNotFound)
		return
	}

	ctx := sup.GetContext()
	if ctx == nil {
		http.Error(w, "context unavailable", http.StatusInternalServerError)
		return
	}

	room := ""
	if ctx.Delivery != nil {
		if pending := ctx.Delivery.Pending(); pending != nil {
			room = pending.RoomName
			ctx.Delivery.ClearRequest(pending)
			s.manager.DeliveryService().ClearPersistentRequest(req.Supervisor)
		}
		if active := ctx.Delivery.Active(); active != nil {
			if room == "" {
				room = active.RoomName
			}
			ctx.Delivery.ClearRequest(active)
			s.manager.DeliveryService().ClearPersistentRequest(req.Supervisor)
		}
	}

	if room != "" {
		s.appendDeliveryHistory(DeliveryHistoryEntry{
			Supervisor:     req.Supervisor,
			Room:           room,
			FilterApplied:  "-",
			FilterMode:     "-",
			Result:         "cancelled",
			ItemsDelivered: 0,
			Duration:       "-",
			ErrorMessage:   "Cancelled from delivery manager",
			Timestamp:      time.Now(),
		})
	}

	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(map[string]string{"status": "cancelled"})
}

func (s *HttpServer) handleDeliveryFilters(w http.ResponseWriter, r *http.Request) {
	supervisor := r.URL.Query().Get("supervisor")
	if supervisor == "" {
		http.Error(w, "supervisor parameter is required", http.StatusBadRequest)
		return
	}

	switch r.Method {
	case http.MethodGet:
		w.Header().Set("Content-Type", "application/json")
		_ = json.NewEncoder(w).Encode(s.getDeliveryFilters(supervisor))
	case http.MethodPost:
		var req delivery.Filters
		if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
			http.Error(w, "invalid JSON", http.StatusBadRequest)
			return
		}
		normalized := s.setDeliveryFilters(supervisor, req)
		s.manager.DeliveryService().SetFilters(supervisor, normalized, nil)
		w.Header().Set("Content-Type", "application/json")
		_ = json.NewEncoder(w).Encode(map[string]string{"status": "saved"})
	default:
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
	}
}

// Delivery scheduling has been removed; deliveries are executed immediately on request.
