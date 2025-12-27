# Game Chat Feature Documentation

## Overview

The Game Chat feature allows you to send and receive in-game messages directly from the Koolo dashboard without interrupting your bot's operation. Messages stay in the UI, and the bot continues running seamlessly.

## Features

✅ **Send messages while bot is running** - No need to switch to the game window
✅ **Message history** - All messages are stored and can be reviewed
✅ **Live updates** - New messages appear in real-time
✅ **Game status indicator** - Shows whether the game is currently active
✅ **Embeddable widget** - Chat widget can be embedded in the main dashboard or accessed as a standalone page
✅ **Responsive design** - Works on desktop and mobile devices
✅ **Non-intrusive** - Sending messages doesn't interrupt bot activity

## Accessing the Chat

### Option 1: Standalone Chat Page
Click the **Chat** button in the dashboard header (looks like a speech bubble icon). This opens a full-screen chat interface.

### Option 2: Embedded Widget (Future)
You can embed the chat widget directly in the dashboard. Add this to any template:

```html
<div id="game-chat-widget"></div>
<script src="../assets/js/game_chat.js"></script>
```

## How It Works

1. **Type your message** in the message input field
2. **Press Enter** or click the **Send** button
3. The message is transmitted to the game
4. Your character will open chat (presses Enter) and send the message
5. The message appears in your chat history with a timestamp

## API Endpoints

All chat functionality is accessed through REST API endpoints:

### Send Message
```
POST /api/chat/send
Content-Type: application/json

{
  "message": "Hello in game!"
}
```

**Response:**
```json
{
  "status": "success"
}
```

### Get Chat History
```
GET /api/chat/history
```

**Response:**
```json
{
  "messages": [
    {
      "message": "Hello",
      "timestamp": "2025-12-25T12:34:56Z",
      "direction": "sent",
      "sender": "Self"
    },
    {
      "message": "Hi there!",
      "timestamp": "2025-12-25T12:35:10Z",
      "direction": "received",
      "sender": "SomePlayer"
    }
  ]
}
```

### Clear Chat History
```
POST /api/chat/clear
```

**Response:**
```json
{
  "status": "success"
}
```

### Get Game Status
```
GET /api/status
```

**Response:**
```json
{
  "inGame": true
}
```

## Message Format

Messages have the following properties:

- **message**: The text content (max 255 characters, Diablo 2 chat limit)
- **timestamp**: ISO 8601 formatted time
- **direction**: Either "sent" or "received"
- **sender**: Username or "Self" for messages you sent

## JavaScript Widget API

The chat widget is a self-contained JavaScript class that can be instantiated programmatically:

```javascript
// Initialize widget
const chatWidget = new GameChatWidget('element-id');

// Send a message
chatWidget.sendMessage();

// Load chat history
chatWidget.loadChatHistory();

// Clear chat
chatWidget.clearChat();

// Toggle minimize
chatWidget.toggleMinimize();

// Destroy widget
chatWidget.destroy();
```

## Configuration

### Message History Size
The bot stores up to 100 messages in memory by default. To change this, modify the `MaxMessages` variable in `chat_api.go`:

```go
MaxMessages = 200  // Store more messages
```

### Character Limits
Diablo 2 chat messages have a practical limit of 255 characters. The UI enforces this with `maxlength="255"`.

## Technical Details

### How Messages Are Sent

The bot uses keyboard simulation to send messages:

1. Presses **Enter** to open the chat window
2. Types each character individually using keyboard events
3. Presses **Enter** again to send the message

This approach ensures:
- ✅ No game interruption
- ✅ Natural keyboard input (no clipboard abuse)
- ✅ Compatible with all Diablo 2 versions
- ✅ Works while bot is automating

### Files Involved

**Backend:**
- `internal/server/chat_api.go` - API handlers and message logic
- `internal/server/http_server.go` - Route registration

**Frontend:**
- `internal/server/templates/game_chat.gohtml` - Full-page chat UI
- `internal/server/assets/js/game_chat.js` - Chat widget JavaScript
- `internal/server/templates/index.gohtml` - Dashboard header button

## Limitations

⚠️ **Single Character**: Messages are sent from your current character. If you have multiple characters, you'll need to switch to message as a different character.

⚠️ **Received Messages**: Currently, received messages must be added programmatically through `AddReceivedMessage(sender, message)`. Auto-detection of received messages requires game memory parsing integration.

⚠️ **Chat Window State**: The bot assumes standard Diablo 2 chat bindings (Enter key). Custom key bindings are not yet supported.

## Adding Received Message Support

To add automatic received message detection, integrate with the game reader:

```go
// In your game data reader
func (gr *GameReader) ReadChatMessages() {
    // Parse game memory for recent chat messages
    // Call AddReceivedMessage for each new message
    server.AddReceivedMessage("PlayerName", "Your message content")
}
```

## Future Enhancements

Potential improvements for future versions:

- [ ] Auto-detect received messages from game memory
- [ ] Message persistence to disk (SQLite/JSON)
- [ ] Player filtering and friend list
- [ ] Custom notification sounds
- [ ] Message search and filtering
- [ ] Multiple character support
- [ ] Chat commands (/invite, /trade, etc.)
- [ ] Emoji and rich text support
- [ ] Chat logs export

## Troubleshooting

### Messages not sending
- Ensure the game window is in focus
- Check that you're logged into the game
- Look at the browser console (F12) for errors

### Status shows "offline"
- The game is not currently running
- Check if the game process is active
- Try clicking the refresh button

### Chat window not opening
- Clear your browser cache (Ctrl+Shift+Delete)
- Try a different browser
- Check browser console for JavaScript errors

## Example Usage Scripts

### JavaScript - Send message from console
```javascript
fetch('/api/chat/send', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({message: 'Hello from console!'})
});
```

### cURL - Send message from command line
```bash
curl -X POST http://localhost:6119/api/chat/send \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello from curl!"}'
```

### Python - Get chat history
```python
import requests
import json

response = requests.get('http://localhost:6119/api/chat/history')
messages = response.json()['messages']

for msg in messages:
    print(f"{msg['sender']}: {msg['message']}")
```

## Support

For issues or feature requests, check the Koolo repository issues or create a new one with details about your bot setup and the issue encountered.
