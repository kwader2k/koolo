package server

import (
	"encoding/json"
	"fmt"
	"net/http"
	"strings"
	"time"

	"github.com/hectorgimenez/koolo/internal/drop"
)

// DropRequest represents a single Drop request.
type DropRequest struct {
	RoomName string        `json:"room"`
	Password string        `json:"password"`
	Filter   *drop.Filters `json:"filter"`
	CardID   int           `json:"cardId"`
	CardName string        `json:"cardName"`
}

// DropStatusResponse is the full response for the Drop status API.
type DropStatusResponse struct {
	Supervisors []SupervisorDropStatus `json:"supervisors"`
	Queue       []DropQueueEntry       `json:"queue"`
	History     []DropHistoryEntry     `json:"history"`
}

// SupervisorDropStatus describes the Drop state of a single supervisor.
type SupervisorDropStatus struct {
	Name     string    `json:"name"`
	State    string    `json:"state"`
	Room     string    `json:"room"`
	Password string    `json:"password"`
	Since    time.Time `json:"since"`
	Running  bool      `json:"running"`
}

// DropQueueEntry is a queue entry shown in the UI.
type DropQueueEntry struct {
	Supervisor string `json:"supervisor"`
	Room       string `json:"room"`
	Status     string `json:"status"`
	Attempts   int    `json:"attempts"`
	NextAction string `json:"nextAction"`
}

// DropHistoryEntry is a single entry in the Drop history.
type DropHistoryEntry struct {
	Supervisor     string    `json:"supervisor"`
	Room           string    `json:"room"`
	FilterApplied  string    `json:"filterApplied"` // "None", "Individual"
	FilterMode     string    `json:"filterMode"`    // "Exclusive", "Inclusive", "-"
	Result         string    `json:"result"`        // "Success", "Failed", "Timeout"
	ItemsDroppered int       `json:"itemsDroppered"`
	Duration       string    `json:"duration"` // "45s", "1m12s"
	ErrorMessage   string    `json:"errorMessage"`
	CardID         int       `json:"cardId"`
	CardName       string    `json:"cardName"`
	Timestamp      time.Time `json:"timestamp"`
}

// DropBatchRequest requests Dropperies for multiple supervisors at once.
type DropBatchRequest struct {
	Supervisors  []string      `json:"supervisors"`
	RoomName     string        `json:"room"`
	Password     string        `json:"password"`
	DelaySeconds int           `json:"delaySeconds"`
	Filter       *drop.Filters `json:"filter"`
	CardID       int           `json:"cardId"`
	CardName     string        `json:"cardName"`
}

// DropCancelRequest cancels an in-flight or pending Drop for a supervisor.
type DropCancelRequest struct {
	Supervisor string `json:"supervisor"`
}

// initDropCallbacks wires Drop-related callbacks from the Drop service
// into the HTTP server, so results and filter state are reflected in the UI.
func (s *HttpServer) initDropCallbacks() {
	s.manager.DropService().SetClearServerFilterCallback(s.onDropClearFilters)
	s.manager.DropService().SetClearPersistentRequestCallback(s.onDropClearPersistentRequest)
	s.manager.DropService().SetDropResultCallback(s.onDropResult)
}

// onDropClearFilters is invoked when a Drop finishes and per-supervisor
// filters should be cleared on the server side.
func (s *HttpServer) onDropClearFilters(supervisor string) {
	s.DropMux.Lock()
	if filters, ok := s.DropFilters[supervisor]; ok {
		filters.Enabled = false
		s.DropFilters[supervisor] = filters
	}
	s.DropMux.Unlock()
}

// onDropClearPersistentRequest is invoked when a Drop request is cleared
// to remove it from persistent storage.
func (s *HttpServer) onDropClearPersistentRequest(supervisor string) {
	s.manager.DropService().ClearPersistentRequest(supervisor)
}

// onDropResult is invoked when a Drop run finishes so that the
// result can be recorded in the in-memory history for the UI.
func (s *HttpServer) onDropResult(supervisorName, room, result string, itemsDroppered int, duration time.Duration, errorMsg string, filters drop.Filters) {
	// Determine which filter configuration was actually applied for this run.
	filterApplied := "None"
	filterMode := "-"
	if filters.Enabled {
		filterApplied = "Individual"
		if filters.DropperOnlySelected {
			filterMode = "Exclusive"
		} else {
			filterMode = "Inclusive"
		}
	}

	// Format duration nicely for the UI
	durationStr := fmt.Sprintf("%.1fs", duration.Seconds())
	if duration.Minutes() >= 1 {
		durationStr = fmt.Sprintf("%dm%ds", int(duration.Minutes()), int(duration.Seconds())%60)
	}

	card := s.getDropCardInfo(supervisorName)
	s.appendDropHistory(DropHistoryEntry{
		Supervisor:     supervisorName,
		Room:           room,
		FilterApplied:  filterApplied,
		FilterMode:     filterMode,
		Result:         result,
		ItemsDroppered: itemsDroppered,
		Duration:       durationStr,
		ErrorMessage:   errorMsg,
		CardID:         card.ID,
		CardName:       card.Name,
		Timestamp:      time.Now(),
	})
	s.setDropCardInfo(supervisorName, 0, "")
}

func (s *HttpServer) DropManagerPage(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/html; charset=utf-8")

	err := s.templates.ExecuteTemplate(w, "drop_manager.gohtml", nil)
	if err != nil {
		s.logger.Error("Failed to execute drop_manager template", "error", err)
		http.Error(w, fmt.Sprintf("Template error: %v", err), http.StatusInternalServerError)
		return
	}
}

func (s *HttpServer) registerDropRoutes() {
	http.HandleFunc("/api/Drop/request", s.handleDrop)
	http.HandleFunc("/api/Drop/status", s.handleDropStatus)
	http.HandleFunc("/api/Drop/batch", s.handleDropBatch)
	http.HandleFunc("/api/Drop/start-Dropper", s.handleDropStartDropper)
	http.HandleFunc("/api/Drop/cancel", s.handleDropCancel)
	http.HandleFunc("/api/Drop/protection", s.handleDropFilters)
	http.HandleFunc("/api/Drop/filters", s.handleDropFilters)
}

func (s *HttpServer) appendDropHistory(entry DropHistoryEntry) {
	s.DropMux.Lock()
	defer s.DropMux.Unlock()

	s.DropHistory = append([]DropHistoryEntry{entry}, s.DropHistory...)
	if len(s.DropHistory) > 100 {
		s.DropHistory = s.DropHistory[:100]
	}
}

func (s *HttpServer) rememberDropRequest(supervisor, room, password, result string) {
	card := s.getDropCardInfo(supervisor)
	s.appendDropHistory(DropHistoryEntry{
		Supervisor:     supervisor,
		Room:           room,
		FilterApplied:  "-",
		FilterMode:     "-",
		Result:         result,
		ItemsDroppered: 0,
		Duration:       "-",
		ErrorMessage:   "",
		CardID:         card.ID,
		CardName:       card.Name,
		Timestamp:      time.Now(),
	})
}

func (s *HttpServer) getDropHistory() []DropHistoryEntry {
	s.DropMux.Lock()
	defer s.DropMux.Unlock()
	history := make([]DropHistoryEntry, len(s.DropHistory))
	copy(history, s.DropHistory)
	return history
}

func (s *HttpServer) setDropCardInfo(supervisor string, id int, name string) {
	s.DropMux.Lock()
	defer s.DropMux.Unlock()
	if s.DropCardInfo == nil {
		s.DropCardInfo = make(map[string]dropCardInfo)
	}
	if id == 0 && name == "" {
		delete(s.DropCardInfo, supervisor)
		return
	}
	s.DropCardInfo[supervisor] = dropCardInfo{ID: id, Name: name}
}

func (s *HttpServer) getDropCardInfo(supervisor string) dropCardInfo {
	s.DropMux.Lock()
	defer s.DropMux.Unlock()
	if s.DropCardInfo == nil {
		return dropCardInfo{}
	}
	info, _ := s.DropCardInfo[supervisor]
	return info
}

func (s *HttpServer) getDropFilters(supervisor string) drop.Filters {
	s.DropMux.Lock()
	defer s.DropMux.Unlock()
	if filters, ok := s.DropFilters[supervisor]; ok && filters.Enabled {
		return filters.Normalize()
	}

	// If a supervisor entry exists but is disabled, fall back to defaults (unfiltered)
	return drop.Filters{DropperOnlySelected: false}.Normalize()
}

func (s *HttpServer) setDropFilters(supervisor string, p drop.Filters) drop.Filters {
	s.DropMux.Lock()
	defer s.DropMux.Unlock()
	s.DropFilters[supervisor] = p.Normalize()
	return s.DropFilters[supervisor]
}

func (s *HttpServer) resolveFilterValue(filter *drop.Filters) drop.Filters {
	if filter == nil {
		return drop.Filters{DropperOnlySelected: false}.Normalize()
	}
	return filter.Normalize()
}

func (s *HttpServer) submitDropRequest(supervisor, room, password string, filters *drop.Filters, cardID int, cardName string) error {
	sup := s.manager.GetSupervisor(supervisor)
	if sup == nil {
		return fmt.Errorf("unknown supervisor %s", supervisor)
	}

	ctx := sup.GetContext()
	if ctx == nil {
		return fmt.Errorf("failed to get context for %s", supervisor)
	}

	if ctx.Drop == nil {
		ctx.Drop = drop.NewManager(ctx.Name, ctx.Logger)
	}

	actualFilters := s.resolveFilterValue(filters)
	// Avoid overwriting active Drop filters; apply per-request filters when the run starts.
	if ctx.Drop.Active() == nil {
		ctx.Drop.UpdateFilters(actualFilters)
	}
	s.setDropFilters(supervisor, actualFilters)

	req := ctx.Drop.RequestDrop(room, password)
	req.CardID = cardID
	req.CardName = cardName
	req.Filters = actualFilters
	s.setDropCardInfo(supervisor, cardID, cardName)
	req.Filters = actualFilters
	ctx.Logger.Info("Drop request queued", "supervisor", supervisor, "room", room)

	// Store persistent request for 10 minutes
	s.manager.DropService().StorePersistentRequest(supervisor, req)

	s.rememberDropRequest(supervisor, room, password, "queued")
	return nil
}

func (s *HttpServer) buildSupervisorStatus(name string) SupervisorDropStatus {
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
			if ctx.Drop != nil {
				if active := ctx.Drop.Active(); active != nil {
					state = "active"
					room = active.RoomName
					password = active.Password
					since = active.CreatedAt
				} else if pending := ctx.Drop.Pending(); pending != nil {
					room = pending.RoomName
					password = pending.Password
					since = pending.CreatedAt
					state = "pending"
				}
			}
		}
	}
	return SupervisorDropStatus{
		Name:     name,
		State:    state,
		Room:     room,
		Password: password,
		Since:    since,
		Running:  running,
	}
}

func (s *HttpServer) handleDrop(w http.ResponseWriter, r *http.Request) {
	var req DropRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "invalid JSON", http.StatusBadRequest)
		return
	}

	supervisor := r.URL.Query().Get("supervisor")
	if supervisor == "" {
		http.Error(w, "supervisor is required", http.StatusBadRequest)
		return
	}

	if err := s.submitDropRequest(supervisor, req.RoomName, req.Password, req.Filter, req.CardID, req.CardName); err != nil {
		if strings.Contains(err.Error(), "unknown supervisor") {
			http.Error(w, err.Error(), http.StatusNotFound)
			return
		}
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	w.WriteHeader(http.StatusOK)
	_, _ = w.Write([]byte("Drop request queued"))
}

func (s *HttpServer) handleDropStatus(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	response := DropStatusResponse{
		Supervisors: []SupervisorDropStatus{},
		Queue:       []DropQueueEntry{},
		History:     s.getDropHistory(),
	}

	for _, name := range s.manager.AvailableSupervisors() {
		status := s.buildSupervisorStatus(name)
		response.Supervisors = append(response.Supervisors, status)

		switch status.State {
		case "pending", "active":
			entry := DropQueueEntry{
				Supervisor: status.Name,
				Room:       status.Room,
				Status:     status.State,
				Attempts:   1,
			}
			switch status.State {
			case "pending":
				entry.NextAction = "waiting for next slot"
			default:
				entry.NextAction = "processing Drop"
			}
			response.Queue = append(response.Queue, entry)
		}
	}

	existing := make(map[string]struct{}, len(response.Queue))
	for _, q := range response.Queue {
		existing[q.Supervisor] = struct{}{}
	}

	if svc := s.manager.DropService(); svc != nil {
		for sup, queue := range svc.QueuedStartSnapshot() {
			if len(queue) == 0 {
				continue
			}
			if _, seen := existing[sup]; seen {
				continue
			}
			// Show the head of the queued start list
			next := queue[0]
			response.Queue = append(response.Queue, DropQueueEntry{
				Supervisor: sup,
				Room:       next.Room,
				Status:     "scheduled",
				Attempts:   0,
				NextAction: "waiting for supervisor to come online",
			})
		}
	}

	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(response)
}

func (s *HttpServer) handleDropBatch(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req DropBatchRequest
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

		if err := s.submitDropRequest(name, req.RoomName, req.Password, req.Filter, req.CardID, req.CardName); err != nil {
			failed = append(failed, fmt.Sprintf("%s: %v", name, err))
		}

		// Apply sequential delay between Dropperies (skip for first Drop)
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

func (s *HttpServer) handleDropStartDropper(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req struct {
		RoomName string        `json:"room"`
		Password string        `json:"password"`
		Filter   *drop.Filters `json:"filter"`
		CardID   int           `json:"cardId"`
		CardName string        `json:"cardName"`
	}

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
		if err := s.submitDropRequest(supervisor, req.RoomName, req.Password, req.Filter, req.CardID, req.CardName); err != nil {
			http.Error(w, err.Error(), http.StatusInternalServerError)
			return
		}
	} else {
		actualFilter := s.resolveFilterValue(req.Filter)
		s.setDropFilters(supervisor, actualFilter)

		if svc := s.manager.DropService(); svc != nil {
			svc.QueueStartDrop(supervisor, req.RoomName, req.Password, actualFilter, req.CardID, req.CardName)
			s.setDropCardInfo(supervisor, req.CardID, req.CardName)
		}
	}

	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(map[string]string{"status": "queued"})
}

func (s *HttpServer) handleDropCancel(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req DropCancelRequest
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
	if ctx.Drop != nil {
		if pending := ctx.Drop.Pending(); pending != nil {
			room = pending.RoomName
			ctx.Drop.ClearRequest(pending)
			s.manager.DropService().ClearPersistentRequest(req.Supervisor)
		}
		if active := ctx.Drop.Active(); active != nil {
			if room == "" {
				room = active.RoomName
			}
			ctx.Drop.ClearRequest(active)
			s.manager.DropService().ClearPersistentRequest(req.Supervisor)
		}
	}

	if room != "" {
		card := s.getDropCardInfo(req.Supervisor)
		s.appendDropHistory(DropHistoryEntry{
			Supervisor:     req.Supervisor,
			Room:           room,
			FilterApplied:  "-",
			FilterMode:     "-",
			Result:         "cancelled",
			ItemsDroppered: 0,
			Duration:       "-",
			ErrorMessage:   "Cancelled from Drop manager",
			CardID:         card.ID,
			CardName:       card.Name,
			Timestamp:      time.Now(),
		})
		s.setDropCardInfo(req.Supervisor, 0, "")
	}

	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(map[string]string{"status": "cancelled"})
}

func (s *HttpServer) handleDropFilters(w http.ResponseWriter, r *http.Request) {
	supervisor := r.URL.Query().Get("supervisor")
	if supervisor == "" {
		http.Error(w, "supervisor parameter is required", http.StatusBadRequest)
		return
	}

	switch r.Method {
	case http.MethodGet:
		w.Header().Set("Content-Type", "application/json")
		_ = json.NewEncoder(w).Encode(s.getDropFilters(supervisor))
	case http.MethodPost:
		var req drop.Filters
		if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
			http.Error(w, "invalid JSON", http.StatusBadRequest)
			return
		}
		normalized := s.setDropFilters(supervisor, req)
		s.manager.DropService().SetFilters(supervisor, normalized, nil)
		w.Header().Set("Content-Type", "application/json")
		_ = json.NewEncoder(w).Encode(map[string]string{"status": "saved"})
	default:
		http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
	}
}
