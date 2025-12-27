# Game Chat Feature - Quick Reference Card

## 🎯 At a Glance

**What**: Send/receive in-game messages from the dashboard without interrupting the bot
**Where**: Dashboard Chat button (header)  
**How**: Type message → Press Enter → Message sent to game
**Cost**: Free, built-in feature

## 📍 Where to Access

| Method | Steps |
|--------|-------|
| **Dashboard** | Click the Chat button in the header (speech bubble icon) |
| **Direct URL** | `http://localhost:6119/game-chat` |
| **Embedded** | Add widget to any page (see docs) |

## ⌨️ Quick Commands

| Action | How |
|--------|-----|
| Send message | Type → Press Enter (or click Send) |
| Clear history | Click Clear button in chat |
| View history | Chat loads automatically on page load |
| Check game status | Status indicator in header (green = online) |

## 🔌 API Quick Reference

```
POST   /api/chat/send      → Send message
GET    /api/chat/history   → Get all messages
POST   /api/chat/clear     → Clear history
GET    /api/status         → Check if game running
GET    /game-chat          → View chat page
```

## 📝 Example Messages

```
"Hey, trading at Chaos!"
"Can anyone help with Baal run?"
"WTS SoJ, pst for price"
"Thanks for the group!"
```

## ⚙️ Configuration

| Setting | Location | Default |
|---------|----------|---------|
| Max messages | chat_api.go | 100 |
| Update interval | game_chat.js | 2 seconds |
| Max length | game_chat.gohtml | 255 chars |

## 🚦 Status Indicator

| Color | Meaning |
|-------|---------|
| 🟢 Green | Game is running and ready |
| 🔴 Red | Game is not running |
| ⚪ Gray | Cannot determine status |

## 💡 Tips & Tricks

1. **Keep Dashboard Open** - Monitor messages in real-time while bot farms
2. **Quick Typing** - Messages are sent instantly (bot uses keyboard simulation)
3. **No Window Switch** - Stay in dashboard, message goes to game
4. **History Intact** - All messages saved until cleared or bot restarts
5. **Use Wisely** - Message while bot is doing non-critical actions

## 🔧 Keyboard Shortcuts

| Key | Action |
|-----|--------|
| **Enter** | Send message (when typing) |
| **Tab** | Focus message input |
| **Ctrl+A** | Select all text in input |
| **Ctrl+C** | Copy from chat |

## 📊 Files Changed/Created

**New Files:**
- `internal/server/chat_api.go` - Backend API
- `internal/server/templates/game_chat.gohtml` - Chat UI
- `internal/server/assets/js/game_chat.js` - Widget JS
- `GAME_CHAT_README.md` - Full documentation
- `CHAT_INTEGRATION_GUIDE.md` - Integration examples

**Modified Files:**
- `internal/server/http_server.go` - Added routes
- `internal/server/templates/index.gohtml` - Added button

## ✅ Verification Checklist

- [x] Chat button appears in dashboard header
- [x] Clicking Chat opens `/game-chat` page
- [x] Message input field accepts text
- [x] Send button works or Enter key sends
- [x] Messages appear in history with timestamps
- [x] Clear button removes history
- [x] Status indicator shows game status
- [x] Auto-refresh updates messages

## 🐛 Common Issues & Fixes

| Issue | Fix |
|-------|-----|
| Messages not sending | Game window not focused / Not in game |
| Chat page blank | Clear browser cache |
| Status always offline | Game process not running |
| Widget not loading | Check browser console (F12) |

## 📚 Documentation Files

| File | Purpose |
|------|---------|
| `GAME_CHAT_README.md` | Complete feature docs |
| `CHAT_INTEGRATION_GUIDE.md` | How to integrate |
| `CHAT_FEATURE_SUMMARY.md` | Implementation overview |

## 🎓 Learning Path

1. **Start Here** → Read `CHAT_FEATURE_SUMMARY.md`
2. **Get Details** → Read `GAME_CHAT_README.md`
3. **Integrate** → Check `CHAT_INTEGRATION_GUIDE.md`
4. **Customize** → Modify `game_chat.js` or `chat_api.go`

## 🚀 Next Steps

1. Open your bot dashboard
2. Click the Chat button
3. Type a test message
4. Watch it send to your game!
5. Check message history

## 🔐 Security

- Requires dashboard admin access
- Messages limited to 255 characters
- Input validated on server
- No external API calls
- Safe keyboard simulation

## 📞 Need Help?

Refer to:
- `GAME_CHAT_README.md` - Full troubleshooting guide
- `CHAT_INTEGRATION_GUIDE.md` - Integration examples
- Code comments in `chat_api.go` and `game_chat.js`

---

**Version**: 1.0  
**Status**: ✅ Production Ready  
**Last Updated**: 2025-12-25
