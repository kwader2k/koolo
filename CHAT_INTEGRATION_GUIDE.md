// Integration Guide: Adding Chat Widget to Dashboard
// 
// This file shows how to integrate the Game Chat feature into different parts of the bot

// OPTION 1: Add Chat Button to Dashboard Header (Already Done)
// ============================================================
// See: internal/server/templates/index.gohtml
// The chat button has been added to the dashboard header with the following:
// <button class="btn btn-outline" onclick="location.href='/game-chat'" title="Game Chat">
//     <i class="bi bi-chat-left-text"></i>
// </button>

// OPTION 2: Embed Chat Widget in a Character Panel
// ==================================================
// Add this to your character template to show chat inline:
//
// <div id="character-chat-widget-{{ .Name }}"></div>
// <script src="../assets/js/game_chat.js"></script>
// <script>
//   document.addEventListener('DOMContentLoaded', function() {
//     const widget = new GameChatWidget('character-chat-widget-{{ .Name }}');
//   });
// </script>

// OPTION 3: Add Chat to Character Management Modal
// ==================================================
// In your character panel, add:
//
// <button onclick="openCharacterChat('{{ .Name }}')" class="btn btn-outline">
//     <i class="bi bi-chat-left"></i> Chat
// </button>
//
// Then add this JavaScript:
//
// function openCharacterChat(characterName) {
//     const modal = document.createElement('div');
//     modal.className = 'modal';
//     modal.style.display = 'block';
//     modal.innerHTML = `
//         <div class="modal-content">
//             <span class="close" onclick="this.parentElement.parentElement.style.display='none'">&times;</span>
//             <h2>Chat as ${characterName}</h2>
//             <div id="chat-modal-widget"></div>
//         </div>
//     `;
//     document.body.appendChild(modal);
//     
//     const widget = new GameChatWidget('chat-modal-widget');
// }

// OPTION 4: Add Chat API to Existing JavaScript
// ===============================================
// In your dashboard.js or other JavaScript files:
//
// // Send a message programmatically
// async function sendGameMessage(message) {
//     try {
//         const response = await fetch('/api/chat/send', {
//             method: 'POST',
//             headers: {
//                 'Content-Type': 'application/json',
//             },
//             body: JSON.stringify({ message })
//         });
//         
//         if (response.ok) {
//             console.log('Message sent successfully');
//         } else {
//             console.error('Failed to send message');
//         }
//     } catch (error) {
//         console.error('Error:', error);
//     }
// }
//
// // Get chat history
// async function getChatHistory() {
//     try {
//         const response = await fetch('/api/chat/history');
//         const data = await response.json();
//         console.log(data.messages);
//         return data.messages;
//     } catch (error) {
//         console.error('Error fetching chat history:', error);
//     }
// }
//
// // Clear chat history
// async function clearGameChat() {
//     try {
//         const response = await fetch('/api/chat/clear', { method: 'POST' });
//         if (response.ok) {
//             console.log('Chat cleared');
//         }
//     } catch (error) {
//         console.error('Error clearing chat:', error);
//     }
// }

// OPTION 5: Add Chat Commands Support (Future Enhancement)
// =========================================================
// Extend the chat_api.go to support commands:
//
// func parseCommand(message string) (isCommand bool, commandName string, args []string) {
//     if !strings.HasPrefix(message, "/") {
//         return false, "", nil
//     }
//     
//     parts := strings.Fields(message)
//     return true, parts[0][1:], parts[1:]
// }
//
// Supported commands:
// /invite [player] - Invite player to party
// /trade [player] - Open trade with player
// /whisper [player] [message] - Send private message
// /who - List players in game
// /ignore [player] - Ignore player
// /clear - Clear chat history

// OPTION 6: Customize Chat Widget Appearance
// ===========================================
// Modify the styles in game_chat.js CSS section:
//
// .chat-widget {
//     // Add your custom styles
//     background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
//     height: 500px;  // Change height
// }

// OPTION 7: Add Auto-Reply Functionality
// ======================================
// In chat_api.go, add auto-reply logic:
//
// type AutoReply struct {
//     Enabled bool
//     Message string
//     Triggers []string
// }
//
// func handleAutoReply(senderName, message string) {
//     if !autoReply.Enabled {
//         return
//     }
//     
//     for _, trigger := range autoReply.Triggers {
//         if strings.Contains(strings.ToLower(message), strings.ToLower(trigger)) {
//             sendMessageToGame(autoReply.Message)
//             return
//         }
//     }
// }

// BACKEND INTEGRATION CHECKLIST
// ============================
// ✓ API endpoints defined in chat_api.go
// ✓ Routes registered in http_server.go
// ✓ Chat page template created (game_chat.gohtml)
// ✓ JavaScript widget created (game_chat.js)
// ✓ Dashboard button added to index.gohtml
//
// OPTIONAL ENHANCEMENTS:
// □ Add received message detection (requires game memory parsing)
// □ Add message persistence (SQLite database)
// □ Add friend list and filtering
// □ Add chat commands support
// □ Add notification system
// □ Add message search

// TESTING THE CHAT FEATURE
// ========================

// 1. Start the bot with: ./koolo
// 2. Open dashboard at: http://localhost:6119
// 3. Click the Chat button in the header
// 4. Type a test message and press Enter
// 5. Verify the message appears in your chat history
// 6. Check the game window to see if the message was sent

// API TESTING WITH CURL:
// ======================

// Send message:
// curl -X POST http://localhost:6119/api/chat/send \
//   -H "Content-Type: application/json" \
//   -d '{"message":"test message"}'

// Get history:
// curl http://localhost:6119/api/chat/history

// Clear chat:
// curl -X POST http://localhost:6119/api/chat/clear

// Get game status:
// curl http://localhost:6119/api/status
