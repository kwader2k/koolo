package messagebus

import (
	"log/slog"
	"sync"
)

// Topic represents a message topic for pub/sub routing
type Topic string

const (
	// TopicLeaderStatus is used for leader presence/absence notifications
	TopicLeaderStatus Topic = "leader_status"
	// TopicGameCoordination is used for game join/leave coordination
	TopicGameCoordination Topic = "game_coordination"
	// TopicBotStatus is used for bot status updates
	TopicBotStatus Topic = "bot_status"
)

// Message is the interface all message types must implement
type Message interface {
	Topic() Topic
	Sender() string
}

// Subscriber is a function that receives messages
type Subscriber func(msg Message)

// Bus is a simple in-memory pub/sub message bus for bot intercommunication
type Bus struct {
	mu          sync.RWMutex
	subscribers map[Topic]map[string]Subscriber // topic -> subscriberID -> handler
	logger      *slog.Logger
}

var (
	globalBus  *Bus
	globalOnce sync.Once
)

// GetBus returns the global message bus singleton
func GetBus() *Bus {
	globalOnce.Do(func() {
		globalBus = NewBus(nil)
	})
	return globalBus
}

// InitBus initializes the global message bus with a logger
func InitBus(logger *slog.Logger) *Bus {
	globalOnce.Do(func() {
		globalBus = NewBus(logger)
	})
	// If already initialized, update logger if provided
	if logger != nil && globalBus.logger == nil {
		globalBus.logger = logger
	}
	return globalBus
}

// NewBus creates a new message bus instance
func NewBus(logger *slog.Logger) *Bus {
	return &Bus{
		subscribers: make(map[Topic]map[string]Subscriber),
		logger:      logger,
	}
}

// Subscribe registers a subscriber for a specific topic
// Returns a function to unsubscribe
func (b *Bus) Subscribe(topic Topic, subscriberID string, handler Subscriber) func() {
	b.mu.Lock()
	defer b.mu.Unlock()

	if b.subscribers[topic] == nil {
		b.subscribers[topic] = make(map[string]Subscriber)
	}

	b.subscribers[topic][subscriberID] = handler

	if b.logger != nil {
		b.logger.Debug("Subscriber registered",
			slog.String("topic", string(topic)),
			slog.String("subscriber", subscriberID))
	}

	// Return unsubscribe function
	return func() {
		b.Unsubscribe(topic, subscriberID)
	}
}

// Unsubscribe removes a subscriber from a topic
func (b *Bus) Unsubscribe(topic Topic, subscriberID string) {
	b.mu.Lock()
	defer b.mu.Unlock()

	if topicSubs, ok := b.subscribers[topic]; ok {
		delete(topicSubs, subscriberID)
		if b.logger != nil {
			b.logger.Debug("Subscriber unregistered",
				slog.String("topic", string(topic)),
				slog.String("subscriber", subscriberID))
		}
	}
}

// UnsubscribeAll removes all subscriptions for a subscriber ID across all topics
func (b *Bus) UnsubscribeAll(subscriberID string) {
	b.mu.Lock()
	defer b.mu.Unlock()

	for topic, subs := range b.subscribers {
		if _, ok := subs[subscriberID]; ok {
			delete(subs, subscriberID)
			if b.logger != nil {
				b.logger.Debug("Subscriber unregistered from topic",
					slog.String("topic", string(topic)),
					slog.String("subscriber", subscriberID))
			}
		}
	}
}

// Publish sends a message to all subscribers of its topic
func (b *Bus) Publish(msg Message) {
	b.mu.RLock()
	defer b.mu.RUnlock()

	topic := msg.Topic()
	topicSubs, ok := b.subscribers[topic]
	if !ok {
		return
	}

	if b.logger != nil {
		b.logger.Debug("Publishing message",
			slog.String("topic", string(topic)),
			slog.String("sender", msg.Sender()),
			slog.Int("subscribers", len(topicSubs)))
	}

	// Deliver to all subscribers (excluding sender to avoid loops)
	for subID, handler := range topicSubs {
		if subID != msg.Sender() {
			// Call handler in goroutine to avoid blocking
			go handler(msg)
		}
	}
}

// PublishSync sends a message synchronously (waits for all handlers to complete)
func (b *Bus) PublishSync(msg Message) {
	b.mu.RLock()
	topicSubs, ok := b.subscribers[msg.Topic()]
	if !ok {
		b.mu.RUnlock()
		return
	}

	// Copy handlers to avoid holding lock during execution
	handlers := make([]Subscriber, 0, len(topicSubs))
	for subID, handler := range topicSubs {
		if subID != msg.Sender() {
			handlers = append(handlers, handler)
		}
	}
	b.mu.RUnlock()

	// Execute handlers synchronously
	for _, handler := range handlers {
		handler(msg)
	}
}

// SubscriberCount returns the number of subscribers for a topic
func (b *Bus) SubscriberCount(topic Topic) int {
	b.mu.RLock()
	defer b.mu.RUnlock()

	if topicSubs, ok := b.subscribers[topic]; ok {
		return len(topicSubs)
	}
	return 0
}

// HasSubscribers returns true if the topic has any subscribers
func (b *Bus) HasSubscribers(topic Topic) bool {
	return b.SubscriberCount(topic) > 0
}

