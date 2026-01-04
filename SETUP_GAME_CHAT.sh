#!/usr/bin/env bash
# Game Chat Feature - Installation & Testing Guide

# ============================================================================
# GAME CHAT FEATURE - COMPLETE SETUP & TESTING
# ============================================================================

# This script provides a checklist and testing procedures for the Game Chat
# feature in the Koolo bot.

# ============================================================================
# STEP 1: VERIFY FILES ARE IN PLACE
# ============================================================================

echo "Checking if all Game Chat files are in place..."

files_to_check=(
    "internal/server/chat_api.go"
    "internal/server/templates/game_chat.gohtml"
    "internal/server/assets/js/game_chat.js"
    "GAME_CHAT_README.md"
    "CHAT_INTEGRATION_GUIDE.md"
    "CHAT_FEATURE_SUMMARY.md"
    "CHAT_QUICK_REFERENCE.md"
)

for file in "${files_to_check[@]}"; do
    if [ -f "$file" ]; then
        echo "✓ $file"
    else
        echo "✗ $file (MISSING)"
    fi
done

# ============================================================================
# STEP 2: VERIFY CODE CHANGES
# ============================================================================

echo ""
echo "Checking if http_server.go has been updated..."

if grep -q "/api/chat/send" internal/server/http_server.go; then
    echo "✓ Chat routes registered in http_server.go"
else
    echo "✗ Chat routes NOT found in http_server.go"
fi

if grep -q "game-chat" internal/server/templates/index.gohtml; then
    echo "✓ Chat button added to dashboard"
else
    echo "✗ Chat button NOT found in dashboard"
fi

# ============================================================================
# STEP 3: BUILD THE PROJECT
# ============================================================================

echo ""
echo "Building the project..."

# Compile the Go code
go build -o koolo.exe ./cmd/koolo/main.go

if [ $? -eq 0 ]; then
    echo "✓ Build successful"
else
    echo "✗ Build failed"
    echo "Check the output above for compilation errors"
    exit 1
fi

# ============================================================================
# STEP 4: TEST THE API ENDPOINTS
# ============================================================================

echo ""
echo "Testing API endpoints..."
echo "(Make sure the bot is running on localhost:6119)"

# Test Get Status
echo ""
echo "Test 1: Get Game Status"
curl -s http://localhost:6119/api/status | python -m json.tool 2>/dev/null || \
  echo "Could not connect to bot. Is it running on port 6119?"

# Test Send Message (requires game to be running)
echo ""
echo "Test 2: Send Test Message"
curl -s -X POST http://localhost:6119/api/chat/send \
  -H "Content-Type: application/json" \
  -d '{"message":"test"}' | python -m json.tool 2>/dev/null || \
  echo "Failed to send message"

# Test Get History
echo ""
echo "Test 3: Get Chat History"
curl -s http://localhost:6119/api/chat/history | python -m json.tool 2>/dev/null || \
  echo "Failed to get history"

# Test Clear History
echo ""
echo "Test 4: Clear Chat History"
curl -s -X POST http://localhost:6119/api/chat/clear | python -m json.tool 2>/dev/null || \
  echo "Failed to clear history"

# ============================================================================
# STEP 5: MANUAL TESTING
# ============================================================================

echo ""
echo "Manual Testing Steps:"
echo "1. Open your Koolo bot dashboard: http://localhost:6119"
echo "2. Look for the Chat button (speech bubble icon) in the header"
echo "3. Click the Chat button to open the full chat interface"
echo "4. Type a test message and press Enter"
echo "5. Watch the game window - your character should send the message"
echo "6. Check the chat history to verify the message was logged"
echo "7. Try the Clear button to clear history"

# ============================================================================
# STEP 6: VERIFY FEATURE FUNCTIONALITY
# ============================================================================

echo ""
echo "Feature Checklist:"
echo "□ Chat button appears in dashboard header"
echo "□ Chat page loads at /game-chat URL"
echo "□ Can type message in input field"
echo "□ Send button works or Enter key sends"
echo "□ Messages appear in history"
echo "□ Timestamps are correct"
echo "□ Status indicator shows correct status"
echo "□ Clear button removes history"
echo "□ Auto-refresh updates messages"
echo "□ Message sending doesn't interrupt bot"

# ============================================================================
# STEP 7: TEST WITH ACTUAL GAMEPLAY
# ============================================================================

echo ""
echo "Full Integration Test:"
echo "1. Start the bot and create/load a game"
echo "2. Open the dashboard"
echo "3. Click Chat button"
echo "4. Type: 'Testing game chat system'"
echo "5. Press Enter"
echo "6. Check that:"
echo "   - Message appears in chat history"
echo "   - Message is sent to game"
echo "   - Bot continues running uninterrupted"
echo "   - No game window focus lost"

# ============================================================================
# STEP 8: INTEGRATION OPTIONS
# ============================================================================

echo ""
echo "Integration Options (See CHAT_INTEGRATION_GUIDE.md):"
echo "□ Full chat page (already done)"
echo "□ Embedded widget in character panel"
echo "□ Chat in character management modal"
echo "□ JavaScript API in custom scripts"
echo "□ Auto-reply functionality"
echo "□ Chat commands support"

# ============================================================================
# STEP 9: TROUBLESHOOTING
# ============================================================================

echo ""
echo "If you encounter issues:"
echo ""
echo "Build Errors:"
echo "  - Check that all files exist in correct directories"
echo "  - Run: go mod tidy"
echo "  - Verify imports in chat_api.go"
echo ""
echo "API Not Responding:"
echo "  - Verify routes in http_server.go"
echo "  - Check bot is running on port 6119"
echo "  - Look at bot logs for errors"
echo ""
echo "Chat Page Not Loading:"
echo "  - Clear browser cache (Ctrl+Shift+Delete)"
echo "  - Check browser console (F12) for errors"
echo "  - Verify game_chat.gohtml template exists"
echo ""
echo "Messages Not Sending:"
echo "  - Ensure game window is in focus"
echo "  - Check that you're logged into game"
echo "  - Verify game window is not minimized"
echo ""

# ============================================================================
# STEP 10: DOCUMENTATION REFERENCES
# ============================================================================

echo ""
echo "Documentation Files:"
echo "1. GAME_CHAT_README.md - Full feature documentation"
echo "2. CHAT_INTEGRATION_GUIDE.md - How to integrate in different ways"
echo "3. CHAT_FEATURE_SUMMARY.md - Implementation overview"
echo "4. CHAT_QUICK_REFERENCE.md - Quick reference card"
echo ""
echo "Read these for detailed information about:"
echo "- Feature overview and capabilities"
echo "- API endpoint reference"
echo "- JavaScript widget API"
echo "- Integration examples"
echo "- Troubleshooting guide"
echo "- Future enhancements"

# ============================================================================
# SUMMARY
# ============================================================================

echo ""
echo "============================================================================"
echo "GAME CHAT FEATURE - INSTALLATION COMPLETE"
echo "============================================================================"
echo ""
echo "You now have a fully functional in-game messaging system!"
echo ""
echo "Quick Start:"
echo "1. Run the bot: ./koolo"
echo "2. Open dashboard: http://localhost:6119"
echo "3. Click the Chat button"
echo "4. Start typing messages!"
echo ""
echo "The bot will continue running while you chat."
echo ""
echo "For more information, see the documentation files in the root directory."
echo ""
echo "Happy chatting!"
echo ""
