// Game Chat Widget for Dashboard
// Embed in-game chat directly in the dashboard without leaving the UI

class GameChatWidget {
    constructor(containerId = 'game-chat-widget') {
        this.containerId = containerId;
        this.messageHistory = [];
        this.isOpen = false;
        this.updateInterval = null;
        this.init();
    }

    init() {
        this.createWidget();
        this.setupEventListeners();
        this.loadChatHistory();
        this.startAutoRefresh();
    }

    createWidget() {
        const container = document.getElementById(this.containerId);
        if (!container) {
            console.error(`Container ${this.containerId} not found`);
            return;
        }

        container.innerHTML = `
            <div class="chat-widget">
                <div class="chat-widget-header">
                    <h3>Game Chat</h3>
                    <div class="chat-widget-controls">
                        <button class="chat-minimize-btn" title="Minimize/Maximize" onclick="window.gameChatWidget.toggleMinimize()">_</button>
                        <button class="chat-clear-btn" title="Clear Chat" onclick="window.gameChatWidget.clearChat()">✕</button>
                    </div>
                </div>
                <div class="chat-widget-messages" id="chatMessages">
                    <div class="chat-empty">No messages yet</div>
                </div>
                <div class="chat-widget-input">
                    <input 
                        type="text" 
                        class="chat-input" 
                        id="chatInput" 
                        placeholder="Type message..."
                        maxlength="255"
                    >
                    <button class="chat-send-btn" onclick="window.gameChatWidget.sendMessage()" title="Send">➤</button>
                </div>
            </div>
        `;

        this.addStyles();
    }

    addStyles() {
        const styleId = 'chat-widget-styles';
        if (document.getElementById(styleId)) return;

        const styles = document.createElement('style');
        styles.id = styleId;
        styles.textContent = `
            .chat-widget {
                display: flex;
                flex-direction: column;
                height: 400px;
                background: var(--card-background-color);
                border: 1px solid var(--border-color);
                border-radius: 8px;
                overflow: hidden;
                box-shadow: 0 2px 8px rgba(0, 0, 0, 0.15);
                font-family: system-ui, -apple-system, sans-serif;
            }

            .chat-widget-header {
                display: flex;
                justify-content: space-between;
                align-items: center;
                padding: 1rem;
                background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
                color: white;
                border-bottom: 2px solid #764ba2;
            }

            .chat-widget-header h3 {
                margin: 0;
                font-size: 1rem;
                font-weight: 600;
            }

            .chat-widget-controls {
                display: flex;
                gap: 0.5rem;
            }

            .chat-minimize-btn,
            .chat-clear-btn {
                background: rgba(255, 255, 255, 0.2);
                border: none;
                color: white;
                padding: 0.25rem 0.5rem;
                border-radius: 4px;
                cursor: pointer;
                font-size: 0.9rem;
                transition: background 0.2s;
            }

            .chat-minimize-btn:hover,
            .chat-clear-btn:hover {
                background: rgba(255, 255, 255, 0.3);
            }

            .chat-widget-messages {
                flex: 1;
                overflow-y: auto;
                padding: 1rem;
                display: flex;
                flex-direction: column;
                gap: 0.5rem;
            }

            .chat-empty {
                display: flex;
                align-items: center;
                justify-content: center;
                height: 100%;
                color: var(--muted-color);
                text-align: center;
                font-size: 0.9rem;
            }

            .chat-message {
                padding: 0.5rem 0.75rem;
                border-radius: 6px;
                font-size: 0.85rem;
                word-wrap: break-word;
                animation: slideInChat 0.3s ease-in-out;
            }

            @keyframes slideInChat {
                from {
                    opacity: 0;
                    transform: translateY(10px);
                }
                to {
                    opacity: 1;
                    transform: translateY(0);
                }
            }

            .chat-message.sent {
                background: #4CAF50;
                color: white;
                margin-left: 1rem;
                text-align: left;
            }

            .chat-message.received {
                background: #2196F3;
                color: white;
                margin-right: 1rem;
                text-align: left;
            }

            .chat-message-sender {
                font-weight: 600;
                font-size: 0.75rem;
                opacity: 0.9;
                margin-bottom: 0.25rem;
            }

            .chat-message-text {
                word-break: break-word;
            }

            .chat-message-time {
                font-size: 0.7rem;
                opacity: 0.7;
                margin-top: 0.25rem;
            }

            .chat-widget-input {
                display: flex;
                gap: 0.5rem;
                padding: 0.75rem;
                background: var(--card-background-color);
                border-top: 1px solid var(--border-color);
            }

            .chat-input {
                flex: 1;
                padding: 0.5rem;
                border: 1px solid var(--form-element-border-color);
                border-radius: 4px;
                background: var(--form-element-background-color);
                color: var(--text-color);
                font-size: 0.85rem;
            }

            .chat-input:focus {
                outline: none;
                border-color: #667eea;
                box-shadow: 0 0 0 2px rgba(102, 126, 234, 0.1);
            }

            .chat-send-btn {
                padding: 0.5rem 0.75rem;
                background: #667eea;
                border: none;
                color: white;
                border-radius: 4px;
                cursor: pointer;
                font-weight: 600;
                transition: background 0.2s;
            }

            .chat-send-btn:hover {
                background: #764ba2;
            }

            .chat-send-btn:active {
                transform: scale(0.95);
            }

            .chat-widget.minimized {
                height: auto;
            }

            .chat-widget.minimized .chat-widget-messages,
            .chat-widget.minimized .chat-widget-input {
                display: none;
            }

            @media (max-width: 768px) {
                .chat-widget {
                    height: 300px;
                }
            }
        `;
        document.head.appendChild(styles);
    }

    setupEventListeners() {
        const input = document.getElementById('chatInput');
        if (input) {
            input.addEventListener('keypress', (e) => {
                if (e.key === 'Enter') {
                    this.sendMessage();
                }
            });
            input.addEventListener('focus', () => {
                this.isOpen = true;
            });
            input.addEventListener('blur', () => {
                this.isOpen = false;
            });
        }
    }

    async sendMessage() {
        const input = document.getElementById('chatInput');
        const message = input.value.trim();

        if (!message) return;

        try {
            const response = await fetch('/api/chat/send', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ message }),
            });

            if (!response.ok) {
                const error = await response.text();
                console.error('Failed to send message:', error);
                this.showNotification('Failed to send message');
                return;
            }

            input.value = '';
            input.focus();
            this.loadChatHistory();
        } catch (error) {
            console.error('Error sending message:', error);
            this.showNotification('Error sending message');
        }
    }

    async loadChatHistory() {
        try {
            const response = await fetch('/api/chat/history');
            if (!response.ok) {
                throw new Error('Failed to load chat history');
            }

            const data = await response.json();
            this.displayMessages(data.messages || []);
        } catch (error) {
            console.error('Error loading chat history:', error);
        }
    }

    displayMessages(messages) {
        const messagesContainer = document.getElementById('chatMessages');
        if (!messagesContainer) return;

        if (!messages || messages.length === 0) {
            messagesContainer.innerHTML = '<div class="chat-empty">No messages yet</div>';
            return;
        }

        const shouldScroll = messagesContainer.scrollTop + messagesContainer.clientHeight >=
            messagesContainer.scrollHeight - 50;

        messagesContainer.innerHTML = messages.map(msg => `
            <div class="chat-message ${msg.direction}">
                <div class="chat-message-sender">${this.escapeHtml(msg.sender)}</div>
                <div class="chat-message-text">${this.escapeHtml(msg.message)}</div>
                <div class="chat-message-time">${this.formatTime(msg.timestamp)}</div>
            </div>
        `).join('');

        if (shouldScroll) {
            this.scrollToBottom();
        }
    }

    scrollToBottom() {
        const container = document.getElementById('chatMessages');
        if (container) {
            container.scrollTop = container.scrollHeight;
        }
    }

    formatTime(timestamp) {
        const date = new Date(timestamp);
        return date.toLocaleTimeString('en-US', {
            hour: '2-digit',
            minute: '2-digit',
            hour12: true
        });
    }

    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }

    async clearChat() {
        if (!confirm('Clear chat history?')) return;

        try {
            const response = await fetch('/api/chat/clear', {
                method: 'POST',
            });

            if (!response.ok) {
                this.showNotification('Failed to clear chat');
                return;
            }

            this.loadChatHistory();
        } catch (error) {
            console.error('Error clearing chat:', error);
            this.showNotification('Error clearing chat');
        }
    }

    toggleMinimize() {
        const widget = document.querySelector('.chat-widget');
        if (widget) {
            widget.classList.toggle('minimized');
        }
    }

    showNotification(message) {
        console.warn('[Chat Widget]', message);
    }

    startAutoRefresh() {
        this.updateInterval = setInterval(() => {
            this.loadChatHistory();
        }, 2000);
    }

    stopAutoRefresh() {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
        }
    }

    destroy() {
        this.stopAutoRefresh();
        const container = document.getElementById(this.containerId);
        if (container) {
            container.innerHTML = '';
        }
    }
}

// Auto-initialize widget if container exists
window.addEventListener('load', () => {
    if (document.getElementById('game-chat-widget')) {
        window.gameChatWidget = new GameChatWidget('game-chat-widget');
    }
});
