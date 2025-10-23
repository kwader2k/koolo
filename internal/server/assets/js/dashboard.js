/**
 * Dashboard WebSocket Client
 * Handles real-time updates and character management
 */

// ========================================
// CONSTANTS & CONFIGURATION
// ========================================
const CONFIG = {
    WS_URL: `ws://${window.location.host}/ws`,
    MAX_RECONNECT_ATTEMPTS: 5,
    RECONNECT_DELAY: 3000,
    FETCH_TIMEOUT: 10000,
    STORAGE_KEY_EXPANDED: 'expandedCards'
};

const XP_TABLE = {
    1: [0, 500], 2: [500, 1000], 3: [1500, 2250], 4: [3750, 4125], 5: [7875, 6300],
    6: [14175, 8505], 7: [22680, 10206], 8: [32886, 11510], 9: [44396, 13319],
    10: [57715, 14429], 11: [72144, 18036], 12: [90180, 22545], 13: [112725, 28181],
    14: [140906, 35226], 15: [176132, 44033], 16: [220165, 55042], 17: [275207, 68801],
    18: [344008, 86002], 19: [430010, 107503], 20: [537513, 134378], 21: [671891, 167973],
    22: [839864, 209966], 23: [1049830, 262457], 24: [1312287, 328072], 25: [1640359, 410090],
    26: [2050449, 512612], 27: [2563061, 640765], 28: [3203826, 698434], 29: [3902260, 761293],
    30: [4663553, 829810], 31: [5493363, 904492], 32: [6397855, 985897], 33: [7383752, 1074627],
    34: [8458379, 1171344], 35: [9629723, 1276765], 36: [10906488, 1391674], 37: [12298162, 1516924],
    38: [13815086, 1653448], 39: [15468534, 1802257], 40: [17270791, 1964461], 41: [19235252, 2141263],
    42: [21376515, 2333976], 43: [23710491, 2544034], 44: [26254525, 2772997], 45: [29027522, 3022566],
    46: [32050088, 3294598], 47: [35344686, 3591112], 48: [38935798, 3914311], 49: [42850109, 4266600],
    50: [47116709, 4650593], 51: [51767302, 5069147], 52: [56836449, 5525370], 53: [62361819, 6022654],
    54: [68384473, 6564692], 55: [74949165, 7155515], 56: [82104680, 7799511], 57: [89904191, 8501467],
    58: [98405658, 9266598], 59: [107672256, 10100593], 60: [117772849, 11009646], 61: [128782495, 12000515],
    62: [140783010, 13080560], 63: [153863570, 14257811], 64: [168121381, 15541015], 65: [183662396, 16939705],
    66: [200602101, 18464279], 67: [219066380, 20126064], 68: [239192444, 21937409], 69: [261129853, 23911777],
    70: [285041630, 26063836], 71: [311105466, 28409582], 72: [339515048, 30966444], 73: [370481492, 33753424],
    74: [404234916, 36791232], 75: [441026148, 40102443], 76: [481128591, 43711663], 77: [524840254, 47645713],
    78: [572485967, 51933826], 79: [624419793, 56607872], 80: [681027665, 61702579], 81: [742730244, 67255812],
    82: [809986056, 73308835], 83: [883294891, 79906630], 84: [963201521, 87098226], 85: [1050299747, 94937067],
    86: [1145236814, 103481403], 87: [1248718217, 112794729], 88: [1361512946, 122946255], 89: [1484459201, 134011418],
    90: [1618470619, 146072446], 91: [1764543065, 159218965], 92: [1923762030, 173548673], 93: [2097310703, 189168053],
    94: [2286478756, 206193177], 95: [2492671933, 224750564], 96: [2717422497, 244978115], 97: [2962400612, 267026144],
    98: [3229426756, 291058498], 99: [3520485254, 0]
};

// ========================================
// STATE MANAGEMENT
// ========================================
class DashboardState {
    constructor() {
        this.socket = null;
        this.reconnectAttempts = 0;
        this.expandedCards = this.loadExpandedState();
    }

    loadExpandedState() {
        try {
            const stored = localStorage.getItem(CONFIG.STORAGE_KEY_EXPANDED);
            return stored ? new Set(JSON.parse(stored)) : new Set();
        } catch (error) {
            console.error('Failed to load expanded state:', error);
            return new Set();
        }
    }

    saveExpandedState() {
        try {
            localStorage.setItem(
                CONFIG.STORAGE_KEY_EXPANDED,
                JSON.stringify([...this.expandedCards])
            );
        } catch (error) {
            console.error('Failed to save expanded state:', error);
        }
    }

    toggleCard(cardId) {
        if (this.expandedCards.has(cardId)) {
            this.expandedCards.delete(cardId);
        } else {
            this.expandedCards.add(cardId);
        }
        this.saveExpandedState();
    }

    isCardExpanded(cardId) {
        return this.expandedCards.has(cardId);
    }
}

// ========================================
// WEBSOCKET MANAGER
// ========================================
class WebSocketManager {
    constructor(state) {
        this.state = state;
        this.socket = null;
        this.reconnectAttempts = 0;
    }

    connect() {
        try {
            this.socket = new WebSocket(CONFIG.WS_URL);

            this.socket.onopen = () => {
                console.log('WebSocket connected');
                this.reconnectAttempts = 0;
            };

            this.socket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    updateDashboard(data);
                } catch (error) {
                    console.error('Failed to parse WebSocket message:', error);
                }
            };

            this.socket.onclose = () => {
                console.log('WebSocket disconnected');
                this.handleReconnect();
            };

            this.socket.onerror = (error) => {
                console.error('WebSocket error:', error);
            };
        } catch (error) {
            console.error('Failed to create WebSocket:', error);
            this.handleReconnect();
        }
    }

    handleReconnect() {
        if (this.reconnectAttempts < CONFIG.MAX_RECONNECT_ATTEMPTS) {
            this.reconnectAttempts++;
            console.log(`Reconnecting... Attempt ${this.reconnectAttempts}`);
            setTimeout(() => this.connect(), CONFIG.RECONNECT_DELAY);
        } else {
            console.error('Max reconnect attempts reached');
        }
    }

    disconnect() {
        if (this.socket) {
            this.socket.close();
            this.socket = null;
        }
    }
}

// ========================================
// API CLIENT
// ========================================
class APIClient {
    static async fetch(url, options = {}) {
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), CONFIG.FETCH_TIMEOUT);

        try {
            const response = await fetch(url, {
                ...options,
                signal: controller.signal
            });
            clearTimeout(timeoutId);

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            return await response.json();
        } catch (error) {
            clearTimeout(timeoutId);
            if (error.name === 'AbortError') {
                throw new Error('Request timeout');
            }
            throw error;
        }
    }

    static async fetchInitialData() {
        try {
            const data = await this.fetch('/initial-data');
            updateDashboard(data);

            const loading = document.getElementById('loading');
            const dashboard = document.getElementById('dashboard');

            if (loading) loading.style.display = 'none';
            if (dashboard) dashboard.style.display = 'block';
        } catch (error) {
            console.error('Error fetching initial data:', error);
        }
    }

    static async performAction(action, characterName) {
        try {
            const data = await this.fetch(`/${action}?characterName=${encodeURIComponent(characterName)}`);
            return data;
        } catch (error) {
            console.error(`Error performing ${action}:`, error);
            throw error;
        }
    }

    static async reloadConfig() {
        try {
            const response = await fetch('/api/reload-config');
            if (!response.ok) {
                throw new Error('Failed to reload config');
            }
        } catch (error) {
            console.error('Error reloading config:', error);
            throw error;
        }
    }
}

// ========================================
// DOM UTILITIES
// ========================================
const DOMUtils = {
    createElement(tag, attributes = {}, children = []) {
        const element = document.createElement(tag);

        Object.entries(attributes).forEach(([key, value]) => {
            if (key === 'className') {
                element.className = value;
            } else if (key === 'dataset') {
                Object.entries(value).forEach(([dataKey, dataValue]) => {
                    element.dataset[dataKey] = dataValue;
                });
            } else if (key.startsWith('on')) {
                const eventName = key.slice(2).toLowerCase();
                element.addEventListener(eventName, value);
            } else {
                element.setAttribute(key, value);
            }
        });

        children.forEach(child => {
            if (typeof child === 'string') {
                element.appendChild(document.createTextNode(child));
            } else if (child instanceof Node) {
                element.appendChild(child);
            }
        });

        return element;
    },

    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
};

// ========================================
// UTILITY FUNCTIONS
// ========================================
const Utils = {
    formatNumber(n) {
        try {
            return Number(n).toLocaleString();
        } catch {
            return String(n);
        }
    },

    formatDuration(ms) {
        if (!isFinite(ms) || ms < 0) return 'N/A';

        const seconds = Math.floor(ms / 1000);
        const minutes = Math.floor(seconds / 60);
        const hours = Math.floor(minutes / 60);
        const days = Math.floor(hours / 24);

        if (days > 0) return `${days}d ${hours % 24}h`;
        if (hours > 0) return `${hours}h ${minutes % 60}m`;
        if (minutes > 0) return `${minutes}m ${seconds % 60}s`;
        return `${seconds}s`;
    },

    titleCase(str) {
        if (!str) return str;
        return str.charAt(0).toUpperCase() + str.slice(1).toLowerCase();
    },

    deriveClassName(raw) {
        if (!raw) return '';

        const lower = raw.toLowerCase();
        const knownClasses = {
            'amazon': 'Amazon',
            'assassin': 'Assassin',
            'barbarian': 'Barbarian',
            'druid': 'Druid',
            'necromancer': 'Necromancer',
            'paladin': 'Paladin',
            'sorceress': 'Sorceress'
        };

        // Check exact match first
        if (knownClasses[lower]) return knownClasses[lower];

        // Heuristic matching
        if (lower.includes('sorc')) return 'Sorceress';
        if (lower.includes('paladin')) return 'Paladin';
        if (lower.includes('barb')) return 'Barbarian';
        if (lower.includes('assassin') || lower.includes('sin')) return 'Assassin';
        if (lower.includes('druid')) return 'Druid';
        if (lower.includes('amazon')) return 'Amazon';
        if (lower.includes('necro')) return 'Necromancer';

        // Fallback
        const base = lower.split('_')[0];
        return this.titleCase(base);
    },

    calculateXP(ui, level) {
        if (!XP_TABLE[level]) {
            return { gained: 0, needed: 1, toNext: 0, pct: 0, nextThreshold: ui.Experience || 0 };
        }

        const [floor, toNext] = XP_TABLE[level];
        const exp = ui.Experience || 0;
        const nextThreshold = toNext > 0 ? floor + toNext : exp;
        const gained = Math.max(0, exp - floor);
        const needed = Math.max(1, toNext > 0 ? toNext : 1);
        const pct = Math.max(0, Math.min(1, toNext > 0 ? gained / needed : 1));

        return { gained, needed, toNext, pct, nextThreshold };
    }
};

// ========================================
// CHARACTER CARD MANAGEMENT
// ========================================
class CharacterCard {
    constructor(key, state) {
        this.key = key;
        this.state = state;
        this.element = null;
    }

    create() {
        const card = document.createElement('div');
        card.className = 'character-card';
        card.id = `card-${this.key}`;

        // Check if card should be expanded
        if (this.state.isCardExpanded(card.id)) {
            card.classList.add('expanded');
        }

        card.innerHTML = this.getTemplate();
        this.element = card;
        this.attachEventListeners();
        return card;
    }

    getTemplate() {
        return `
            <div class="character-header">
                <div class="character-name">
                    <span>${DOMUtils.escapeHtml(this.key)}</span>
                    <div class="status-indicator"></div>
                </div>
                <div class="character-controls">
                    <button class="btn btn-outline companion-join-btn" style="display:none;">
                        <i class="bi bi-door-open btn-icon"></i>Join Game
                    </button>
                    <button class="btn btn-outline" onclick="location.href='/debug?characterName=${encodeURIComponent(this.key)}'">
                        <i class="bi bi-bug btn-icon"></i>Debug    
                    </button>
                    <button class="btn btn-outline reset-muling-btn" data-character-name="${DOMUtils.escapeHtml(this.key)}" title="Reset Muling Progress">
                        <i class="bi bi-arrow-counterclockwise"></i>
                    </button>
                    <button class="btn btn-outline" onclick="location.href='/supervisorSettings?supervisor=${encodeURIComponent(this.key)}'">
                        <i class="bi bi-gear btn-icon"></i>Settings
                    </button>
                    <button class="start-pause btn btn-start" data-character="${DOMUtils.escapeHtml(this.key)}">
                        <i class="bi bi-play-fill btn-icon"></i>Start
                    </button>
                    <button class="stop btn btn-stop" data-character="${DOMUtils.escapeHtml(this.key)}" style="display:none;">
                        <i class="bi bi-stop-fill btn-icon"></i>Stop
                    </button>
                    <button class="btn btn-outline attach-btn" style="display:none;">
                        <i class="bi bi-link-45deg btn-icon"></i>Attach
                    </button>
                    <button class="toggle-details">
                        <i class="bi bi-chevron-down"></i>
                    </button>
                </div>
            </div>
            <div class="character-summary">
                <div class="co-line">
                    <span class="co-classlevel">—</span>
                    <span class="co-dot">•</span>
                    <span class="co-difficulty">—</span>
                    <span class="co-dot">•</span>
                    <span class="co-area">—</span>
                </div>
                <div class="co-line">
                    <div class="co-xp">
                        <div class="xp-bar">
                            <div class="xp-bar-fill"></div>
                        </div>
                        <span class="xp-percent">0%</span>
                    </div>
                </div>
                <div class="co-line">
                    <span class="co-life">Life: —</span>
                    <span class="co-sep">/</span>
                    <span class="co-mana">Mana: —</span>
                    <span class="co-dot">•</span>
                    <span class="co-mf">MF: —</span>
                    <span class="co-dot">•</span>
                    <span class="co-gf">GF: —</span>
                    <span class="co-dot">•</span>
                    <span class="co-gold">Gold: —</span>
                    <span class="co-dot">•</span>
                    <span class="co-res">Res: —</span>
                </div>
            </div>
            <div class="character-details">
                <div class="status-details">
                    <span class="status-badge"></span>
                </div>
                <div class="stats-grid">
                    <div class="stat-item">
                        <div class="stat-label">Games</div>
                        <div class="stat-value runs">0</div>
                    </div>
                    <div class="stat-item">
                        <div class="stat-label">Drops</div>
                        <div class="stat-value drops">None</div>
                    </div>
                    <div class="stat-item">
                        <div class="stat-label">Chickens</div>
                        <div class="stat-value chickens">0</div>
                    </div>
                    <div class="stat-item">
                        <div class="stat-label">Deaths</div>
                        <div class="stat-value deaths">0</div>
                    </div>
                    <div class="stat-item">
                        <div class="stat-label">Errors</div>
                        <div class="stat-value errors">0</div>
                    </div>
                </div>
                <div class="run-stats"></div>
            </div>
        `;
    }

    attachEventListeners() {
        if (!this.element) return;

        const toggleBtn = this.element.querySelector('.toggle-details');
        const startPauseBtn = this.element.querySelector('.start-pause');
        const stopBtn = this.element.querySelector('.stop');
        const attachBtn = this.element.querySelector('.attach-btn');
        const companionJoinBtn = this.element.querySelector('.companion-join-btn');
        const resetMuleBtn = this.element.querySelector('.reset-muling-btn');

        if (toggleBtn) {
            toggleBtn.addEventListener('click', () => {
                this.element.classList.toggle('expanded');
                this.state.toggleCard(this.element.id);
            });
        }

        if (startPauseBtn) {
            startPauseBtn.addEventListener('click', async () => {
                const currentStatus = startPauseBtn.className.includes('btn-start') ? 'Not Started' :
                    startPauseBtn.className.includes('btn-pause') ? 'In game' : 'Paused';

                const action = currentStatus === 'Not Started' ? 'start' : 'togglePause';

                try {
                    const data = await APIClient.performAction(action, this.key);
                    updateDashboard(data);
                } catch (error) {
                    console.error(`Failed to ${action}:`, error);
                }
            });
        }

        if (stopBtn) {
            stopBtn.addEventListener('click', async () => {
                try {
                    await APIClient.performAction('stop', this.key);
                    await APIClient.fetchInitialData();
                } catch (error) {
                    console.error('Failed to stop:', error);
                }
            });
        }

        if (attachBtn) {
            attachBtn.addEventListener('click', () => showAttachPopup(this.key));
        }

        if (companionJoinBtn) {
            companionJoinBtn.addEventListener('click', () => showCompanionJoinPopup(this.key));
        }

        if (resetMuleBtn) {
            resetMuleBtn.addEventListener('click', async (e) => {
                e.stopPropagation();
                if (confirm(`Are you sure you want to reset the muling progress for ${this.key}? This should only be done if you have manually emptied the mules.`)) {
                    try {
                        const response = await fetch(`/reset-muling?characterName=${encodeURIComponent(this.key)}`, {
                            method: 'POST'
                        });
                        if (response.ok) {
                            alert(`Muling progress for ${this.key} has been reset.`);
                        } else {
                            alert('Failed to reset muling progress.');
                        }
                    } catch (error) {
                        console.error('Failed to reset muling:', error);
                        alert('Failed to reset muling progress.');
                    }
                }
            });
        }
    }

    update(value, dropCount) {
        if (!this.element) return;

        this.updateButtons(value.SupervisorStatus);
        this.updateStatus(value.SupervisorStatus);
        this.updateStatusIndicator(value.SupervisorStatus);
        this.updateCompanionJoinButton(value.IsCompanionFollower, value.SupervisorStatus);
        this.updateStats(value.Games, dropCount);
        this.updateRunStats(value.Games);
        this.updateCharacterOverview(value.UI || value.ui, value.SupervisorStatus);
        this.updateStartedTime(value.StartedAt);
    }

    updateButtons(status) {
        const startPauseBtn = this.element.querySelector('.start-pause');
        const stopBtn = this.element.querySelector('.stop');
        const attachBtn = this.element.querySelector('.attach-btn');

        if (!startPauseBtn || !stopBtn || !attachBtn) return;

        if (status === 'Paused') {
            startPauseBtn.innerHTML = '<i class="bi bi-play-fill btn-icon"></i>Resume';
            startPauseBtn.className = 'start-pause btn btn-resume';
            stopBtn.style.display = 'inline-block';
            attachBtn.style.display = 'none';
        } else if (status === 'In game' || status === 'Starting') {
            startPauseBtn.innerHTML = '<i class="bi bi-pause-fill btn-icon"></i>Pause';
            startPauseBtn.className = 'start-pause btn btn-pause';
            stopBtn.style.display = 'inline-block';
            attachBtn.style.display = 'none';
        } else {
            startPauseBtn.innerHTML = '<i class="bi bi-play-fill btn-icon"></i>Start';
            startPauseBtn.className = 'start-pause btn btn-start';
            stopBtn.style.display = 'none';
            attachBtn.style.display = 'inline-block';
        }
    }

    updateStatus(status) {
        const statusBadge = this.element.querySelector('.status-badge');
        if (!statusBadge) return;

        const statusText = status || 'Not started';
        statusBadge.innerHTML = `<span class="status-label">Status:</span> <span class="status-value">${DOMUtils.escapeHtml(statusText)}</span>`;
        statusBadge.className = `status-badge status-${statusText.toLowerCase().replace(' ', '')}`;
    }

    updateStatusIndicator(status) {
        const indicator = this.element.querySelector('.status-indicator');
        if (!indicator) return;

        indicator.classList.remove('in-game', 'paused', 'stopped');

        if (status === 'In game') {
            indicator.classList.add('in-game');
        } else if (status === 'Starting' || status === 'Paused') {
            indicator.classList.add('paused');
        } else {
            indicator.classList.add('stopped');
        }
    }

    updateCompanionJoinButton(isCompanionFollower, status) {
        const btn = this.element.querySelector('.companion-join-btn');
        if (!btn) return;

        const isRunning = ['In game', 'Paused', 'Starting'].includes(status);
        btn.style.display = (isCompanionFollower && isRunning) ? 'inline-flex' : 'none';
    }

    updateStats(games, dropCount) {
        const stats = this.calculateStats(games);

        const runsEl = this.element.querySelector('.runs');
        const dropsEl = this.element.querySelector('.drops');
        const chickensEl = this.element.querySelector('.chickens');
        const deathsEl = this.element.querySelector('.deaths');
        const errorsEl = this.element.querySelector('.errors');

        if (runsEl) runsEl.textContent = stats.totalGames;
        if (dropsEl) {
            dropsEl.innerHTML = dropCount === undefined || dropCount === 0
                ? 'None'
                : `<a href="/drops?supervisor=${encodeURIComponent(this.key)}">${dropCount}</a>`;
        }
        if (chickensEl) chickensEl.textContent = stats.totalChickens;
        if (deathsEl) deathsEl.textContent = stats.totalDeaths;
        if (errorsEl) errorsEl.textContent = stats.totalErrors;
    }

    calculateStats(games) {
        if (!games || games.length === 0) {
            return { totalGames: 0, totalChickens: 0, totalDeaths: 0, totalErrors: 0 };
        }

        return games.reduce((acc, game) => {
            acc.totalGames++;
            if (game.Reason === 'chicken') acc.totalChickens++;
            else if (game.Reason === 'death') acc.totalDeaths++;
            else if (game.Reason === 'error') acc.totalErrors++;
            return acc;
        }, { totalGames: 0, totalChickens: 0, totalDeaths: 0, totalErrors: 0 });
    }

    updateCharacterOverview(ui, status) {
        const isActive = ['In game', 'Paused', 'Starting'].includes(status);

        if (!ui || !isActive) {
            this.clearCharacterOverview();
            return;
        }

        const level = ui.Level || 0;
        const xpData = Utils.calculateXP(ui, level);

        const classLevelEl = this.element.querySelector('.co-classlevel');
        const diffEl = this.element.querySelector('.co-difficulty');
        const areaEl = this.element.querySelector('.co-area');
        const xpFill = this.element.querySelector('.xp-bar-fill');
        const xpPct = this.element.querySelector('.xp-percent');
        const lifeEl = this.element.querySelector('.co-life');
        const manaEl = this.element.querySelector('.co-mana');
        const mfEl = this.element.querySelector('.co-mf');
        const gfEl = this.element.querySelector('.co-gf');
        const goldEl = this.element.querySelector('.co-gold');
        const resEl = this.element.querySelector('.co-res');

        const className = Utils.deriveClassName(ui.Class || '');
        const pctText = isFinite(xpData.pct) ? `${(xpData.pct * 100).toFixed(1)}%` : '100%';

        if (classLevelEl) {
            classLevelEl.textContent = `${className} / Level: ${level} (${pctText})`;
            classLevelEl.title = `XP: ${Utils.formatNumber(ui.Experience)} / Next: ${Utils.formatNumber(xpData.nextThreshold)} (Gained: ${Utils.formatNumber(xpData.gained)} | To Next: ${Utils.formatNumber(xpData.toNext)})`;
        }

        if (xpFill) xpFill.style.width = `${Math.max(0, Math.min(100, xpData.pct * 100)).toFixed(1)}%`;
        if (xpPct) xpPct.textContent = pctText;

        if (diffEl) diffEl.textContent = Utils.titleCase(ui.Difficulty || '');
        if (areaEl) areaEl.textContent = ui.Area || '—';

        if (lifeEl) lifeEl.textContent = `Life: ${ui.Life || 0}/${ui.MaxLife || 0}`;
        if (manaEl) manaEl.textContent = `Mana: ${ui.Mana || 0}/${ui.MaxMana || 0}`;
        if (mfEl) mfEl.textContent = `MF: ${ui.MagicFind || 0}%`;
        if (gfEl) gfEl.textContent = `GF: ${ui.GoldFind || 0}%`;
        if (goldEl) goldEl.textContent = `Gold: ${ui.Gold || 0}`;

        if (resEl) {
            resEl.innerHTML = `<span class="res-fr">FR: ${ui.FireResist || 0}</span> / <span class="res-cr">CR: ${ui.ColdResist || 0}</span> / <span class="res-lr">LR: ${ui.LightningResist || 0}</span> / <span class="res-pr">PR: ${ui.PoisonResist || 0}</span>`;
        }
    }

    clearCharacterOverview() {
        const classLevelEl = this.element.querySelector('.co-classlevel');
        const diffEl = this.element.querySelector('.co-difficulty');
        const areaEl = this.element.querySelector('.co-area');
        const lifeEl = this.element.querySelector('.co-life');
        const manaEl = this.element.querySelector('.co-mana');
        const mfEl = this.element.querySelector('.co-mf');
        const gfEl = this.element.querySelector('.co-gf');
        const goldEl = this.element.querySelector('.co-gold');
        const resEl = this.element.querySelector('.co-res');
        const xpFill = this.element.querySelector('.xp-bar-fill');
        const xpPct = this.element.querySelector('.xp-percent');

        if (classLevelEl) classLevelEl.textContent = '—';
        if (diffEl) diffEl.textContent = '—';
        if (areaEl) areaEl.textContent = '—';
        if (lifeEl) lifeEl.textContent = 'Life: —';
        if (manaEl) manaEl.textContent = 'Mana: —';
        if (mfEl) mfEl.textContent = 'MF: —';
        if (gfEl) gfEl.textContent = 'GF: —';
        if (goldEl) goldEl.textContent = 'Gold: —';
        if (resEl) resEl.textContent = 'Res: —';
        if (xpFill) xpFill.style.width = '0%';
        if (xpPct) xpPct.textContent = '0%';
    }

    updateStartedTime(startedAt) {
        const statusDetails = this.element.querySelector('.status-details');
        if (!statusDetails) return;

        let runningForElement = statusDetails.querySelector('.running-for');
        if (!runningForElement) {
            runningForElement = document.createElement('div');
            runningForElement.className = 'running-for';
            statusDetails.appendChild(runningForElement);
        }

        const startTime = new Date(startedAt);
        const now = new Date();

        if (startTime.getFullYear() === 1) {
            runningForElement.textContent = 'Running for: N/A';
            return;
        }

        const timeDiff = now - startTime;
        if (timeDiff < 0) {
            runningForElement.textContent = 'Running for: N/A';
            return;
        }

        const duration = Utils.formatDuration(timeDiff);
        runningForElement.textContent = `Running for: ${duration}`;
    }

    updateRunStats(games) {
        const runStatsElement = this.element.querySelector('.run-stats');
        if (!runStatsElement) return;

        runStatsElement.innerHTML = '<h3>Run Statistics</h3>';

        const runStats = this.calculateRunStats(games);
        if (Object.keys(runStats).length === 0) {
            runStatsElement.innerHTML += '<p>No run data available yet.</p>';
            return;
        }

        const runStatsGrid = document.createElement('div');
        runStatsGrid.className = 'run-stats-grid';

        Object.entries(runStats).forEach(([runName, stats]) => {
            const runElement = document.createElement('div');
            runElement.className = 'run-stat';
            if (stats.isCurrentRun) {
                runElement.classList.add('current-run');
            }

            runElement.innerHTML = `
                <h4>${DOMUtils.escapeHtml(runName)}${stats.isCurrentRun ? ' <span class="current-run-indicator">Current</span>' : ''}</h4>
                <div class="run-stat-content">
                    <div class="run-stat-item" title="Fastest Run">
                        <span class="stat-label">Fastest:</span> ${Utils.formatDuration(stats.shortestTime)}
                    </div>
                    <div class="run-stat-item" title="Slowest Run">
                        <span class="stat-label">Slowest:</span> ${Utils.formatDuration(stats.longestTime)}
                    </div>
                    <div class="run-stat-item" title="Average Run">
                        <span class="stat-label">Average:</span> ${Utils.formatDuration(stats.averageTime)}
                    </div>
                    <div class="run-stat-item" title="Total Runs">
                        <span class="stat-label">Total:</span> ${stats.runCount}
                    </div>
                    <div class="run-stat-item" title="Errors">
                        <span class="stat-label">Errors:</span> ${stats.errorCount}
                    </div>
                    <div class="run-stat-item" title="Chickens">
                        <span class="stat-label">Chickens:</span> ${stats.runChickens}
                    </div>
                    <div class="run-stat-item" title="Deaths">
                        <span class="stat-label">Deaths:</span> ${stats.runDeaths}
                    </div>
                </div>
            `;
            runStatsGrid.appendChild(runElement);
        });

        runStatsElement.appendChild(runStatsGrid);
    }

    calculateRunStats(games) {
        if (!games || games.length === 0) return {};

        const runStats = {};

        games.forEach(game => {
            if (!game.Runs || !Array.isArray(game.Runs)) return;

            game.Runs.forEach(run => {
                if (!runStats[run.Name]) {
                    runStats[run.Name] = {
                        shortestTime: Infinity,
                        longestTime: 0,
                        totalTime: 0,
                        errorCount: 0,
                        runCount: 0,
                        runChickens: 0,
                        runDeaths: 0,
                        successfulRunCount: 0,
                        isCurrentRun: false
                    };
                }

                if (run.Reason === "") {
                    runStats[run.Name].isCurrentRun = true;
                }

                const runTime = new Date(run.FinishedAt) - new Date(run.StartedAt);
                if (run.FinishedAt !== "0001-01-01T00:00:00Z" && runTime > 0) {
                    runStats[run.Name].runCount++;

                    if (run.Reason === 'ok') {
                        runStats[run.Name].shortestTime = Math.min(runStats[run.Name].shortestTime, runTime);
                        runStats[run.Name].longestTime = Math.max(runStats[run.Name].longestTime, runTime);
                        runStats[run.Name].totalTime += runTime;
                        runStats[run.Name].successfulRunCount++;
                    }
                }

                if (run.Reason === 'error') runStats[run.Name].errorCount++;
                if (run.Reason === 'chicken') runStats[run.Name].runChickens++;
                if (run.Reason === 'death') runStats[run.Name].runDeaths++;
            });
        });

        // Calculate averages
        Object.values(runStats).forEach(stats => {
            if (stats.successfulRunCount > 0) {
                stats.averageTime = stats.totalTime / stats.successfulRunCount;
            } else {
                stats.shortestTime = 0;
                stats.longestTime = 0;
                stats.averageTime = 0;
            }
        });

        return runStats;
    }
}

// ========================================
// DASHBOARD UPDATE
// ========================================
function updateDashboard(data) {
    // Update version
    const versionElement = document.getElementById('version');
    if (versionElement && data.Version) {
        versionElement.textContent = data.Version === "dev" ? "Development Version" : data.Version;
        if (data.Version === "dev") {
            versionElement.style.backgroundColor = "#dc3545";
        }
    }

    const container = document.getElementById('characters-container');
    if (!container) return;

    // Handle empty state
    if (!data.Status || Object.keys(data.Status).length === 0) {
        container.innerHTML = '<article><p>No characters found, start adding a new character.</p></article>';
        return;
    }

    // Update or create cards
    Object.entries(data.Status).forEach(([key, value]) => {
        let card = document.getElementById(`card-${key}`);

        if (!card) {
            const characterCard = new CharacterCard(key, dashboardState);
            card = characterCard.create();
            container.appendChild(card);
        }

        // Find the CharacterCard instance and update
        const characterCard = new CharacterCard(key, dashboardState);
        characterCard.element = card;
        characterCard.update(value, data.DropCount[key]);
    });

    // Remove cards for non-existent characters
    Array.from(container.children).forEach(card => {
        const cardKey = card.id.replace('card-', '');
        if (!data.Status.hasOwnProperty(cardKey)) {
            container.removeChild(card);
        }
    });
}

// ========================================
// POPUP FUNCTIONS
// ========================================
function showAttachPopup(characterName) {
    const popup = document.createElement('div');
    popup.className = 'attach-popup';
    popup.innerHTML = `
        <h3>Attach to Process</h3>
        <div id="popup-content">
            <div class="loading-spinner"></div>
            <p>Loading processes...</p>
        </div>
    `;
    document.body.appendChild(popup);
    fetchProcessList(characterName);
}

async function fetchProcessList(characterName) {
    try {
        const processes = await APIClient.fetch('/process-list');
        const popup = document.querySelector('.attach-popup');

        if (!processes || processes.length === 0) {
            popup.innerHTML = `
                <h3>No D2R Processes Found</h3>
                <p>There are no Diablo II: Resurrected processes currently running.</p>
                <button onclick="closeAttachPopup()" class="btn btn-primary">Close</button>
            `;
            return;
        }

        popup.innerHTML = `
            <h3>Attach to Process</h3>
            <input type="text" id="process-search" class="process-search" placeholder="Search processes...">
            <div class="process-list">
                <table>
                    <thead>
                        <tr>
                            <th>Window Title</th>
                            <th>Process Name</th>
                            <th>PID</th>
                        </tr>
                    </thead>
                    <tbody id="process-list-body"></tbody>
                </table>
            </div>
            <div class="selected-process">
                <span>Selected Process: </span>
                <span id="selected-pid">None</span>
            </div>
            <div class="popup-buttons">
                <button id="choose-process" class="btn btn-primary" disabled>Attach</button>
                <button id="cancel-attach" class="btn btn-outline">Cancel</button>
            </div>
        `;

        const tbody = document.getElementById('process-list-body');
        processes.forEach(process => {
            const row = document.createElement('tr');
            row.innerHTML = `
                <td>${DOMUtils.escapeHtml(process.windowTitle)}</td>
                <td>${DOMUtils.escapeHtml(process.processName)}</td>
                <td>${process.pid}</td>
            `;
            row.addEventListener('click', () => selectProcess(row, process.pid));
            tbody.appendChild(row);
        });

        document.getElementById('choose-process').addEventListener('click', () => chooseProcess(characterName));
        document.getElementById('cancel-attach').addEventListener('click', closeAttachPopup);
        document.getElementById('process-search').addEventListener('input', filterProcessList);

    } catch (error) {
        console.error('Error fetching process list:', error);
        const popup = document.querySelector('.attach-popup');
        popup.innerHTML = `
            <h3>Error</h3>
            <p>An error occurred while fetching the process list.</p>
            <button onclick="closeAttachPopup()" class="btn btn-primary">Close</button>
        `;
    }
}

function selectProcess(row, pid) {
    document.querySelectorAll('#process-list-body tr').forEach(r => r.classList.remove('selected'));
    row.classList.add('selected');
    document.getElementById('choose-process').disabled = false;
    document.getElementById('choose-process').dataset.pid = pid;
    document.getElementById('selected-pid').textContent = pid;
}

async function chooseProcess(characterName) {
    const pid = document.getElementById('choose-process').dataset.pid;
    if (!pid) return;

    const popup = document.querySelector('.attach-popup');
    popup.innerHTML = `
        <h3>Attaching to Process</h3>
        <div class="loading-spinner"></div>
        <p>Please wait...</p>
    `;

    try {
        const data = await fetch(`/attach-process?characterName=${encodeURIComponent(characterName)}&pid=${pid}`, {
            method: 'POST'
        }).then(r => r.json());

        if (data.success) {
            popup.innerHTML = `
                <h3>Success</h3>
                <p>Successfully attached to process ${pid} for character ${DOMUtils.escapeHtml(characterName)}</p>
            `;
            setTimeout(() => {
                closeAttachPopup();
                APIClient.fetchInitialData();
            }, 2000);
        } else {
            popup.innerHTML = `
                <h3>Error</h3>
                <p>Failed to attach to process: ${DOMUtils.escapeHtml(data.error)}</p>
                <button onclick="closeAttachPopup()" class="btn btn-primary">Close</button>
            `;
        }
    } catch (error) {
        console.error('Error attaching to process:', error);
        popup.innerHTML = `
            <h3>Error</h3>
            <p>An error occurred while attaching to the process.</p>
            <button onclick="closeAttachPopup()" class="btn btn-primary">Close</button>
        `;
    }
}

function closeAttachPopup() {
    const popup = document.querySelector('.attach-popup');
    if (popup) popup.remove();
}

function filterProcessList() {
    const searchTerm = document.getElementById('process-search').value.toLowerCase();
    const rows = document.querySelectorAll('#process-list-body tr');

    rows.forEach(row => {
        const windowTitle = row.cells[0].textContent.toLowerCase();
        const processName = row.cells[1].textContent.toLowerCase();
        row.style.display = (windowTitle.includes(searchTerm) || processName.includes(searchTerm)) ? '' : 'none';
    });
}

function showCompanionJoinPopup(characterName) {
    const popup = document.createElement('div');
    popup.className = 'attach-popup';
    popup.innerHTML = `
        <h3>Join Game as Companion</h3>
        <div class="popup-content">
            <div class="form-group">
                <label for="game-name">Game Name:</label>
                <input type="text" id="game-name" placeholder="Enter game name">
            </div>
            <div class="form-group">
                <label for="game-password">Game Password:</label>
                <input type="text" id="game-password" placeholder="Enter game password">
            </div>
            <div class="popup-buttons">
                <button id="join-game-btn" class="btn btn-primary">Request Join</button>
                <button id="cancel-join" class="btn btn-outline">Cancel</button>
            </div>
        </div>
    `;
    document.body.appendChild(popup);

    document.getElementById('join-game-btn').addEventListener('click', () => {
        const gameName = document.getElementById('game-name').value.trim();
        const password = document.getElementById('game-password').value.trim();

        if (!gameName) {
            alert('Please enter a game name');
            return;
        }

        requestCompanionJoin(characterName, gameName, password);
    });

    document.getElementById('cancel-join').addEventListener('click', closeCompanionJoinPopup);
}

function closeCompanionJoinPopup() {
    const popup = document.querySelector('.attach-popup');
    if (popup) popup.remove();
}

async function requestCompanionJoin(supervisor, gameName, password) {
    const popup = document.querySelector('.attach-popup');
    popup.innerHTML = `
        <h3>Requesting Game Join</h3>
        <div class="loading-spinner"></div>
        <p>Please wait...</p>
    `;

    try {
        const response = await fetch('/api/companion-join', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ supervisor, gameName, password })
        });

        const data = await response.json();

        if (data.success) {
            popup.innerHTML = `
                <h3>Success</h3>
                <p>Join request sent for game "${DOMUtils.escapeHtml(gameName)}"</p>
            `;
            setTimeout(closeCompanionJoinPopup, 2000);
        } else {
            popup.innerHTML = `
                <h3>Error</h3>
                <p>Failed to send join request: ${DOMUtils.escapeHtml(data.error || 'Unknown error')}</p>
                <button onclick="closeCompanionJoinPopup()" class="btn btn-primary">Close</button>
            `;
        }
    } catch (error) {
        console.error('Error sending join request:', error);
        popup.innerHTML = `
            <h3>Error</h3>
            <p>An error occurred while sending the join request.</p>
            <button onclick="closeCompanionJoinPopup()" class="btn btn-primary">Close</button>
        `;
    }
}

async function reloadConfig() {
    const btn = document.getElementById('reloadConfigBtn');
    if (!btn) return;

    const icon = btn.querySelector('i');
    btn.disabled = true;
    if (icon) icon.classList.add('rotate');

    try {
        await APIClient.reloadConfig();
    } catch (error) {
        console.error('Error reloading config:', error);
    } finally {
        btn.disabled = false;
        if (icon) icon.classList.remove('rotate');
    }
}

// ========================================
// INITIALIZATION
// ========================================
const dashboardState = new DashboardState();
const wsManager = new WebSocketManager(dashboardState);

document.addEventListener('DOMContentLoaded', () => {
    APIClient.fetchInitialData();
    wsManager.connect();
});
