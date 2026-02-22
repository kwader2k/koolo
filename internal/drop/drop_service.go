package drop

import (
	"log/slog"
	"sync"
	"time"
)

// Service: main Drop entry point
type Service struct {
	coord *Coordinator
	mu    sync.Mutex

	// queued start-Drop requests per supervisor
	queuedStart map[string][]StartRequest
	// persistent Drop requests per supervisor (3min timeout)
	persistentRequests map[string][]*Request
}

const persistentRequestTTL = 3 * time.Minute

// Service constructor
func NewService(logger *slog.Logger) *Service {
	return &Service{
		coord:              NewCoordinator(logger),
		queuedStart:        make(map[string][]StartRequest),
		persistentRequests: make(map[string][]*Request),
	}
}

// StartRequest: pending request to apply when supervisor is attached
type StartRequest struct {
	Room     string
	Password string
	Filter   Filters
	CardID   int
	CardName string
}

// Attach a Manager for a supervisor
func (s *Service) AttachManager(supervisorName string, mgr *Manager) {
	if mgr == nil {
		return
	}

	// Apply filters and callbacks
	s.coord.ApplyInitialFilters(supervisorName, mgr)
	s.coord.ConfigureCallbacks(supervisorName, mgr)

	for {
		req, ok := s.consumeQueuedStart(supervisorName)
		if !ok {
			break
		}
		if req.Filter.Enabled {
			mgr.UpdateFilters(req.Filter)
		}
		dropReq := mgr.RequestDrop(req.Room, req.Password)
		if dropReq != nil {
			dropReq.CardID = req.CardID
			dropReq.CardName = req.CardName
			dropReq.Filters = req.Filter
		}
	}

	for {
		persistentReq, ok := s.consumePersistentRequest(supervisorName)
		if !ok {
			break
		}
		if persistentReq.Filters.Enabled {
			mgr.UpdateFilters(persistentReq.Filters)
		}
		dropReq := mgr.RequestDrop(persistentReq.RoomName, persistentReq.Password)
		if dropReq != nil {
			dropReq.CardID = persistentReq.CardID
			dropReq.CardName = persistentReq.CardName
			dropReq.Filters = persistentReq.Filters
		}
	}

}

// Set filters for a supervisor
func (s *Service) SetFilters(supervisor string, filters Filters, mgr *Manager) {
	s.coord.SetFilters(supervisor, filters, mgr)
}

// Store persistent Drop request for a supervisor
func (s *Service) StorePersistentRequest(supervisorName string, req *Request) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.pruneExpiredLocked()
	s.persistentRequests[supervisorName] = append(s.persistentRequests[supervisorName], req)
}

// Clear persistent Drop request for a supervisor
func (s *Service) ClearPersistentRequest(supervisorName string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	delete(s.persistentRequests, supervisorName)
}

// Return and remove the next non-expired persistent request for a supervisor
func (s *Service) consumePersistentRequest(supervisor string) (*Request, bool) {
	if s == nil || s.persistentRequests == nil {
		return nil, false
	}

	s.mu.Lock()
	defer s.mu.Unlock()
	s.pruneExpiredLocked()
	queue := s.persistentRequests[supervisor]
	for len(queue) > 0 {
		req := queue[0]
		queue = queue[1:]

		if len(queue) == 0 {
			delete(s.persistentRequests, supervisor)
		} else {
			s.persistentRequests[supervisor] = queue
		}

		if req != nil && time.Since(req.CreatedAt) < persistentRequestTTL {
			return req, true
		}
		// skip expired and continue
	}
	return nil, false
}

// Register server filter clear callback
func (s *Service) SetClearServerFilterCallback(callback func(supervisor string)) {
	s.coord.SetClearServerFilterCallback(callback)
}

// Register persistent request clear callback
func (s *Service) SetClearPersistentRequestCallback(callback func(supervisor string)) {
	s.coord.SetClearPersistentRequestCallback(callback)
}

// Register Drop result callback
func (s *Service) SetDropResultCallback(callback func(supervisorName, room, result string, itemsDroppered int, duration time.Duration, errorMsg string, filters Filters)) {
	s.coord.SetDropResultCallback(callback)
}

// Store start-Drop request to apply when supervisor is attached
func (s *Service) QueueStartDrop(supervisor, room, password string, filter Filters, cardID int, cardName string) {
	if s == nil {
		return
	}
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.queuedStart == nil {
		s.queuedStart = make(map[string][]StartRequest)
	}
	s.queuedStart[supervisor] = append(s.queuedStart[supervisor], StartRequest{
		Room:     room,
		Password: password,
		Filter:   filter,
		CardID:   cardID,
		CardName: cardName,
	})
}

// Return and remove queued start-Drop request
func (s *Service) consumeQueuedStart(supervisor string) (StartRequest, bool) {
	if s == nil || s.queuedStart == nil {
		return StartRequest{}, false
	}
	s.mu.Lock()
	defer s.mu.Unlock()

	queue := s.queuedStart[supervisor]
	if len(queue) == 0 {
		return StartRequest{}, false
	}
	req := queue[0]
	if len(queue) == 1 {
		delete(s.queuedStart, supervisor)
	} else {
		s.queuedStart[supervisor] = queue[1:]
	}
	return req, true
}

// QueuedStartSnapshot returns a copy of queued start requests per supervisor.
func (s *Service) QueuedStartSnapshot() map[string][]StartRequest {
	s.mu.Lock()
	defer s.mu.Unlock()

	result := make(map[string][]StartRequest, len(s.queuedStart))
	for sup, queue := range s.queuedStart {
		copied := make([]StartRequest, len(queue))
		copy(copied, queue)
		result[sup] = copied
	}
	return result
}

// pruneExpiredLocked removes persistent requests older than the TTL; caller must hold s.mu.
func (s *Service) pruneExpiredLocked() {
	now := time.Now()
	for sup, queue := range s.persistentRequests {
		keep := queue[:0]
		for _, req := range queue {
			if req != nil && now.Sub(req.CreatedAt) < persistentRequestTTL {
				keep = append(keep, req)
			}
		}
		if len(keep) == 0 {
			delete(s.persistentRequests, sup)
		} else {
			s.persistentRequests[sup] = keep
		}
	}
}
