package context

import (
	"fmt"
	"log/slog"
	"sync"
	"time"
)

type EventType string

const (
	// Companion/Following events
	EventTypeGameInfo       EventType = "game_info"
	EventTypeSupervisorName EventType = "supervisor_name"
	EventTypeFollow         EventType = "follow"
	EventTypeStopFollow     EventType = "stop_follow"
	EventTypeTPWait         EventType = "tp_wait"

	// Group Leveling events
	EventTypeGameCreated         EventType = "game_created"
	EventTypeActStart            EventType = "act_start"
	EventTypeQuestComplete       EventType = "quest_complete"
	EventTypeBossKill            EventType = "boss_kill"
	EventTypeLeaderChange        EventType = "leader_change"
	EventTypeSynchroReady        EventType = "synchro_ready"
	EventTypeSynchroStateRequest EventType = "synchro_state_request"
	EventTypePositionUpdate      EventType = "position_update"
	EventTypeGroupMemberJoined   EventType = "group_member_joined"
	EventTypeCurrentRun          EventType = "current_run"
)

type ICCEvent struct {
	Type      EventType
	Source    string
	Timestamp time.Time
	Data      map[string]interface{}
}

type EventHandler func(event ICCEvent) error

type RequestHandler func(source string, data map[string]interface{}) (map[string]interface{}, error)

type ICCManager struct {
	logger          *slog.Logger
	supervisors     map[string]interface{} // Using interface{} to avoid circular import with bot.Supervisor
	subscriptions   map[EventType][]EventHandler
	requestHandlers map[string]RequestHandler
	pendingRequests map[string]chan map[string]interface{}
	activeRequests  map[string]map[string]bool
	mu              sync.RWMutex
}

func NewICCManager(logger *slog.Logger) *ICCManager {
	return &ICCManager{
		logger:          logger,
		supervisors:     make(map[string]interface{}),
		subscriptions:   make(map[EventType][]EventHandler),
		requestHandlers: make(map[string]RequestHandler),
		pendingRequests: make(map[string]chan map[string]interface{}),
		activeRequests:  make(map[string]map[string]bool),
	}
}

func (icc *ICCManager) RegisterSupervisor(name string, supervisor interface{}) {
	icc.mu.Lock()
	defer icc.mu.Unlock()
	icc.supervisors[name] = supervisor
	icc.logger.Debug("Registered supervisor with ICC", "supervisor", name)
}

func (icc *ICCManager) UnregisterSupervisor(supervisor interface{}) {
	icc.mu.Lock()
	defer icc.mu.Unlock()

	for name, sup := range icc.supervisors {
		if sup == supervisor {
			delete(icc.supervisors, name)
			icc.logger.Debug("Unregistered supervisor from ICC", "supervisor", name)
			return
		}
	}
}

func (icc *ICCManager) Subscribe(eventType EventType, handler EventHandler) {
	icc.mu.Lock()
	defer icc.mu.Unlock()
	icc.subscriptions[eventType] = append(icc.subscriptions[eventType], handler)
	icc.logger.Debug("Subscribed to ICC event", "event_type", eventType)
}

func (icc *ICCManager) PublishEvent(eventType EventType, source string, data map[string]interface{}) {
	icc.mu.RLock()
	handlers := icc.subscriptions[eventType]
	icc.mu.RUnlock()

	event := ICCEvent{
		Type:      eventType,
		Source:    source,
		Timestamp: time.Now(),
		Data:      data,
	}

	icc.logger.Debug("Publishing ICC event",
		"event_type", eventType,
		"source", source,
		"handlers", len(handlers))

	for _, handler := range handlers {
		if err := handler(event); err != nil {
			icc.logger.Error("Error in ICC event handler",
				"event_type", eventType,
				"source", source,
				"error", err)
		}
	}
}

func (icc *ICCManager) RegisterRequestHandler(requestType string, handler RequestHandler) {
	icc.mu.Lock()
	defer icc.mu.Unlock()
	icc.requestHandlers[requestType] = handler
	icc.logger.Debug("Registered ICC request handler", "request_type", requestType)
}

func (icc *ICCManager) Request(requestType string, targetSupervisor string, data map[string]interface{}, timeout time.Duration) (map[string]interface{}, error) {
	requestID := fmt.Sprintf("%s_%s_%d", requestType, targetSupervisor, time.Now().UnixNano())

	icc.mu.Lock()
	// Check for duplicate active requests
	if icc.activeRequests[targetSupervisor] == nil {
		icc.activeRequests[targetSupervisor] = make(map[string]bool)
	}
	if icc.activeRequests[targetSupervisor][requestType] {
		icc.mu.Unlock()
		icc.logger.Debug("Duplicate request filtered",
			"request_type", requestType,
			"target", targetSupervisor)
		return nil, fmt.Errorf("duplicate request: %s to %s", requestType, targetSupervisor)
	}
	icc.activeRequests[targetSupervisor][requestType] = true

	responseChan := make(chan map[string]interface{}, 1)
	icc.pendingRequests[requestID] = responseChan

	_, exists := icc.supervisors[targetSupervisor]
	icc.mu.Unlock()

	// Cleanup on exit
	defer func() {
		icc.mu.Lock()
		delete(icc.pendingRequests, requestID)
		if icc.activeRequests[targetSupervisor] != nil {
			delete(icc.activeRequests[targetSupervisor], requestType)
		}
		icc.mu.Unlock()
	}()

	if !exists {
		return nil, fmt.Errorf("supervisor not found: %s", targetSupervisor)
	}

	icc.logger.Debug("Sending ICC request",
		"request_id", requestID,
		"request_type", requestType,
		"target", targetSupervisor)

	// Send request via event with response channel ID
	requestData := make(map[string]interface{})
	for k, v := range data {
		requestData[k] = v
	}
	requestData["_request_id"] = requestID
	requestData["_request_type"] = requestType

	// Publish request as event
	icc.PublishEvent(EventType(requestType), requestID, requestData)

	// Notify target supervisor (this would need type assertion in bot package)
	// For now, we'll handle via the request handler system

	icc.mu.RLock()
	handler, hasHandler := icc.requestHandlers[targetSupervisor]
	icc.mu.RUnlock()

	if hasHandler {
		// Call handler directly
		response, err := handler(requestType, data)
		if err != nil {
			return nil, err
		}
		return response, nil
	}

	// Wait for response or timeout
	select {
	case response := <-responseChan:
		icc.logger.Debug("Received ICC request response",
			"request_id", requestID,
			"request_type", requestType)
		return response, nil
	case <-time.After(timeout):
		icc.logger.Warn("ICC request timeout",
			"request_id", requestID,
			"request_type", requestType,
			"target", targetSupervisor,
			"timeout", timeout)
		return nil, fmt.Errorf("request timeout: %s to %s", requestType, targetSupervisor)
	}
}

func (icc *ICCManager) RespondToRequest(requestID string, response map[string]interface{}) error {
	icc.mu.Lock()
	responseChan, exists := icc.pendingRequests[requestID]
	icc.mu.Unlock()

	if !exists {
		return fmt.Errorf("no pending request with ID: %s", requestID)
	}

	select {
	case responseChan <- response:
		icc.logger.Debug("Sent ICC request response", "request_id", requestID)
		return nil
	default:
		return fmt.Errorf("response channel full or closed for request: %s", requestID)
	}
}

func (icc *ICCManager) GetSupervisorNames() []string {
	icc.mu.RLock()
	defer icc.mu.RUnlock()

	names := make([]string, 0, len(icc.supervisors))
	for name := range icc.supervisors {
		names = append(names, name)
	}
	return names
}
