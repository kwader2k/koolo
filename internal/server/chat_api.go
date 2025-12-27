package server

import (
	"encoding/json"
	"fmt"
	"net/http"
	"time"

	"github.com/hectorgimenez/koolo/internal/bot"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

var (
	// MessageHistory stores recent chat messages
	MessageHistory = make([]ChatMessage, 0, 100)
	// MaxMessages limits the history size
	MaxMessages = 100
)

// ChatMessage represents a message sent or received
type ChatMessage struct {
	Message   string    `json:"message"`
	Timestamp time.Time `json:"timestamp"`
	Direction string    `json:"direction"` // "sent" or "received"
	Sender    string    `json:"sender"`    // Username or "Self"
}

// SendChatMessageRequest is the request body for sending a message
type SendChatMessageRequest struct {
	Message string `json:"message"`
}

// ChatHistoryResponse is the response containing chat history
type ChatHistoryResponse struct {
	Messages []ChatMessage `json:"messages"`
}

// SendChatMessage handles sending a message in-game
func (s *HttpServer) SendChatMessage(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req SendChatMessageRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	if req.Message == "" {
		http.Error(w, "Message cannot be empty", http.StatusBadRequest)
		return
	}

	// Validate message length (Diablo 2 usually has character limits for chat)
	if len(req.Message) > 255 {
		http.Error(w, "Message too long (max 255 characters)", http.StatusBadRequest)
		return
	}

	// Get the context from the active supervisor (not from context.Get() which doesn't work in HTTP handlers)
	var ctx *context.Context
	if s.manager != nil {
		supervisors := s.manager.GetAllActiveSupervisors()
		if len(supervisors) > 0 && supervisors[0] != nil {
			ctx = supervisors[0].GetContext()
		}
	}

	s.logger.Info("Chat send attempt", "message", req.Message, "hasContext", ctx != nil, "hasHID", ctx != nil && ctx.HID != nil)

	if ctx == nil || ctx.HID == nil {
		errMsg := "Game context not available - start a character/game in Koolo first"
		s.logger.Warn("Chat message failed - context unavailable", "error", errMsg, "ctx", ctx != nil)
		http.Error(w, errMsg, http.StatusBadRequest)
		return
	}

	// Send the message to the game
	if err := sendMessageToGame(ctx, req.Message, s.manager); err != nil {
		errMsg := fmt.Sprintf("Failed to send message: %v", err)
		s.logger.Error("Chat message failed - send error", "error", err)
		http.Error(w, errMsg, http.StatusInternalServerError)
		return
	}

	// Add to message history
	msg := ChatMessage{
		Message:   req.Message,
		Timestamp: time.Now(),
		Direction: "sent",
		Sender:    "Self",
	}
	addMessageToHistory(msg)

	s.logger.Info("Chat message sent successfully", "message", req.Message)
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "success"})
}

// GetChatHistory retrieves the message history
func (s *HttpServer) GetChatHistory(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	response := ChatHistoryResponse{
		Messages: MessageHistory,
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(response)
}

// ClearChatHistory clears the message history
func (s *HttpServer) ClearChatHistory(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	MessageHistory = make([]ChatMessage, 0, 100)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "success"})
}

// sendMessageToGame sends the message to the game by temporarily pausing the bot
func sendMessageToGame(ctx *context.Context, message string, manager *bot.SupervisorManager) error {
	if ctx == nil || ctx.HID == nil {
		return fmt.Errorf("game context not available")
	}

	// Get the first active supervisor to pause/resume
	var supervisorName string
	if manager != nil {
		supervisors := manager.GetAllActiveSupervisors()
		if len(supervisors) > 0 && supervisors[0] != nil {
			// Try to get the supervisor name from context
			if ctx.Name != "" {
				supervisorName = ctx.Name
			}
		}
	}

	// Pause the bot if we have a supervisor name
	if supervisorName != "" && manager != nil {
		manager.TogglePause(supervisorName)
		utils.Sleep(200) // Minimal wait - pause is instant
	}

	// Now send the message with the bot paused
	// Click on game window to ensure focus
	ctx.HID.Click(1, 640, 360)
	utils.Sleep(100) // Brief wait for focus

	// Open chat (Enter key)
	ctx.HID.PressKey(win.VK_RETURN)
	utils.Sleep(150) // Quick wait for chat to open

	// Type message
	for _, char := range message {
		asciiCode := ctx.HID.GetASCIICode(string(char))
		if asciiCode > 0 {
			ctx.HID.PressKey(asciiCode)
			utils.Sleep(40) // Fast typing
		}
	}

	// Send the message
	utils.Sleep(100)
	ctx.HID.PressKey(win.VK_RETURN)
	utils.Sleep(150)

	// Resume the bot if we paused it
	if supervisorName != "" && manager != nil {
		manager.TogglePause(supervisorName)
	}

	return nil
}

// addMessageToHistory adds a message to the history, maintaining max size
func addMessageToHistory(msg ChatMessage) {
	MessageHistory = append(MessageHistory, msg)
	if len(MessageHistory) > MaxMessages {
		MessageHistory = MessageHistory[1:]
	}
}

// AddReceivedMessage adds a received message to history
// This should be called from the game reader when a message is received
func AddReceivedMessage(sender, message string) {
	msg := ChatMessage{
		Message:   message,
		Timestamp: time.Now(),
		Direction: "received",
		Sender:    sender,
	}
	addMessageToHistory(msg)
}

// GameChatPage handler renders the chat page
func (s *HttpServer) gameChatPage(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	s.templates.ExecuteTemplate(w, "game_chat.gohtml", nil)
}

// GetGameStatus returns whether the game is currently running
func (s *HttpServer) GetGameStatus(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodGet {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	inGame := false
	debugInfo := map[string]interface{}{
		"method": "GetGameStatus",
	}

	// Get context from the HTTP request's goroutine (won't work)
	ctx := context.Get()
	if ctx != nil {
		debugInfo["httpGoroutineContextFound"] = true
		if ctx.Manager != nil && ctx.Manager.InGame() {
			inGame = true
			s.logger.Info("Game detected via context.Get() Manager.InGame()")
		}
	} else {
		debugInfo["httpGoroutineContextFound"] = false
	}

	// If HTTP goroutine context didn't work, check through the supervisor manager
	if !inGame && s.manager != nil {
		supervisors := s.manager.GetAllActiveSupervisors()
		debugInfo["supervisorsFound"] = len(supervisors)

		for _, supervisor := range supervisors {
			if supervisor == nil {
				continue
			}

			supCtx := supervisor.GetContext()
			if supCtx == nil {
				continue
			}

			// Check Manager.InGame()
			if supCtx.Manager != nil && supCtx.Manager.InGame() {
				inGame = true
				debugInfo["detectionMethod"] = "supervisor.Manager.InGame()"
				s.logger.Info("Game detected via supervisor Manager.InGame()")
				break
			}

			// Check GameReader.IsIngame()
			if supCtx.GameReader != nil && supCtx.GameReader.IsIngame() {
				inGame = true
				debugInfo["detectionMethod"] = "supervisor.GameReader.IsIngame()"
				s.logger.Info("Game detected via supervisor GameReader.IsIngame()")
				break
			}

			// Check PlayerUnit position
			if supCtx.Data != nil && supCtx.Data.PlayerUnit.Position.X > 0 && supCtx.Data.PlayerUnit.Position.Y > 0 {
				inGame = true
				debugInfo["detectionMethod"] = "supervisor.PlayerUnit.Position"
				debugInfo["positionX"] = supCtx.Data.PlayerUnit.Position.X
				debugInfo["positionY"] = supCtx.Data.PlayerUnit.Position.Y
				s.logger.Info("Game detected via supervisor PlayerUnit position", "x", supCtx.Data.PlayerUnit.Position.X, "y", supCtx.Data.PlayerUnit.Position.Y)
				break
			}
		}
	} else if !inGame {
		debugInfo["managerAvailable"] = false
	}

	s.logger.Debug("Game status check result", debugInfo, "inGame", inGame)

	w.Header().Set("Content-Type", "application/json")

	// Check if debug mode is requested
	debugMode := r.URL.Query().Get("debug") == "1"

	if debugMode {
		// Return detailed debug info
		response := map[string]interface{}{
			"inGame": inGame,
			"debug":  debugInfo,
		}
		json.NewEncoder(w).Encode(response)
	} else {
		// Return simple response for normal operation
		json.NewEncoder(w).Encode(map[string]bool{"inGame": inGame})
	}
}
