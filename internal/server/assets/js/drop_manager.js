// 1. CONFIG & CONSTANTS

const CONFIG = {
  KEYS: {
    ROOM: "DropManager.lastRoom",
    PASSWORD: "DropManager.lastPassword",
    CARDS: "DropManager.cards",
    HISTORY: "DropManager.historyCache",
  },
  RUNES: [
    { label: "El", value: "ElRune" }, { label: "Eld", value: "EldRune" }, { label: "Tir", value: "TirRune" },
    { label: "Nef", value: "NefRune" }, { label: "Eth", value: "EthRune" }, { label: "Ith", value: "IthRune" },
    { label: "Tal", value: "TalRune" }, { label: "Ral", value: "RalRune" }, { label: "Ort", value: "OrtRune" },
    { label: "Thul", value: "ThulRune" }, { label: "Amn", value: "AmnRune" }, { label: "Sol", value: "SolRune" },
    { label: "Shael", value: "ShaelRune" }, { label: "Dol", value: "DolRune" }, { label: "Hel", value: "HelRune" },
    { label: "Io", value: "IoRune" }, { label: "Lum", value: "LumRune" }, { label: "Ko", value: "KoRune" },
    { label: "Fal", value: "FalRune" }, { label: "Lem", value: "LemRune" }, { label: "Pul", value: "PulRune" },
    { label: "Um", value: "UmRune" }, { label: "Mal", value: "MalRune" }, { label: "Ist", value: "IstRune" },
    { label: "Gul", value: "GulRune" }, { label: "Vex", value: "VexRune" }, { label: "Ohm", value: "OhmRune" },
    { label: "Lo", value: "LoRune" }, { label: "Sur", value: "SurRune" }, { label: "Ber", value: "BerRune" },
    { label: "Jah", value: "JahRune" }, { label: "Cham", value: "ChamRune" }, { label: "Zod", value: "ZodRune" },
  ],
  GEMS: [
    { label: "Amethyst", value: "PerfectAmethyst" }, { label: "Diamond", value: "PerfectDiamond" },
    { label: "Emerald", value: "PerfectEmerald" }, { label: "Ruby", value: "PerfectRuby" },
    { label: "Sapphire", value: "PerfectSapphire" }, { label: "Topaz", value: "PerfectTopaz" },
    { label: "Skull", value: "PerfectSkull" },
  ],
  KEY_TOKENS: [
    { label: "T-key", value: "keyofterror" },
    { label: "H-key", value: "keyofhate" },
    { label: "D-Key", value: "keyofdestruction" },
    { label: "Token", value: "tokenofabsolution" },
  ]
};

const DROP_MODES = {
  ALL_EXCEPT_MATERIALS: "drop-except-materials",
  REQUIRE_FILTER: "require-filter"
};

const DropPreferences = {
  KEY: "DropManager.dropMode",
  currentMode: DROP_MODES.ALL_EXCEPT_MATERIALS,
  promptKey: "DropManager.dropModePrompted",

  init() {
    const stored = localStorage.getItem(this.KEY);
    if (stored && Object.values(DROP_MODES).includes(stored)) {
      this.currentMode = stored;
    }
    this.syncUI();
  },

  set(mode) {
    if (!Object.values(DROP_MODES).includes(mode)) return;
    this.currentMode = mode;
    localStorage.setItem(this.KEY, mode);
    localStorage.setItem(this.promptKey, "1");
    this.syncUI();
  },

  syncUI() {
    const radio = document.querySelector(`input[name="dm-drop-mode-option"][value="${this.currentMode}"]`);
    if (radio) {
      radio.checked = true;
    }
    this.updateButtonState();
  },

  requireFilter() {
    return this.currentMode === DROP_MODES.REQUIRE_FILTER;
  },

  updateButtonState() {
    const btn = $.get("dm-drop-mode-settings");
    if (!btn) return;
    if (this.requireFilter()) {
      btn.classList.remove("btn-danger");
      btn.classList.add("btn-outline");
      btn.title = "Filter required mode active";
    } else {
      btn.classList.remove("btn-outline");
      btn.classList.add("btn-danger");
      btn.title = "Standard mode active";
    }
  }
,

  maybePromptInitialMode() {
    if (localStorage.getItem(this.KEY)) return;
    if (localStorage.getItem(this.promptKey)) return;
    DropModeModal.open();
    localStorage.setItem(this.promptKey, "1");
  }
};

// 2. UTILS & DOM HELPER

const $ = {
  get: (id) => document.getElementById(id),
  val: (id) => document.getElementById(id)?.value.trim() || "",
  setVal: (id, v) => { const el = document.getElementById(id); if (el) el.value = v; },
  
  el: (tag, props = {}, ...children) => {
    const el = document.createElement(tag);
    Object.entries(props).forEach(([k, v]) => {
      if (k === 'on' && typeof v === 'object') {
        Object.entries(v).forEach(([evt, handler]) => el.addEventListener(evt, handler));
      } 
      else if (k === 'style' && typeof v === 'object') {
        Object.assign(el.style, v);
      } 
      else if (k === 'dataset' && typeof v === 'object') {
        Object.assign(el.dataset, v);
      } 
      else if (k === 'className' || k === 'classname') {
        el.className = v;
      }
      else if (k === 'checked' || k === 'disabled' || k === 'value' || k === 'title') {
        el[k] = v;
      } 
      else {
        el.setAttribute(k, v);
      }
    });
    
    children.flat().forEach(child => {
      if (!child && child !== 0) return;
      if (child instanceof Node) el.appendChild(child);
      else el.appendChild(document.createTextNode(String(child)));
    });
    return el;
  },

  toast: (message, type = 'info') => {
    const container = $.get('toast-container');
    if (!container) return;
    const toast = $.el('div', { className: `toast ${type}` }, message);
    container.appendChild(toast);
    setTimeout(() => toast.classList.add('show'), 10);
    setTimeout(() => {
      toast.classList.remove('show');
      setTimeout(() => container.removeChild(toast), 300);
    }, 3000);
  },

  statusClass: (sup) => {
    if (!sup || !sup.running) return "stopped";
    const s = (sup.state || "").toLowerCase();
    if (s.includes("pending") || s.includes("scheduled") || s.includes("paused") || s.includes("starting")) {
      return "paused";
    }
    return "in-game";
  },

  formatTime: (ts) => ts ? new Date(ts).toLocaleTimeString() : "-",
  
  getCheckedValues: (containerId) => {
    const items = [];
    document.querySelectorAll(`#${containerId} .filter-checkbox-item`).forEach(item => {
      const cb = item.querySelector('input[type="checkbox"]');
      if (cb?.checked) {
        const qty = item.querySelector('.quantity-value');
        items.push({ name: cb.value, quantity: qty ? (parseInt(qty.value) || 0) : 0 });
      }
    });
    return items;
  },
  
  setCheckboxValues: (containerId, values) => {
    const map = new Map();
    values.forEach(v => typeof v === 'string' ? map.set(v, 0) : map.set(v.name, v.quantity || 0));
    document.querySelectorAll(`#${containerId} .filter-checkbox-item`).forEach(item => {
      const cb = item.querySelector('input[type="checkbox"]');
      const qty = item.querySelector('.quantity-value');
      if (cb && map.has(cb.value)) {
        cb.checked = true;
        if (qty) { qty.classList.add('active'); qty.value = map.get(cb.value); }
      } else {
        cb.checked = false;
        if (qty) qty.classList.remove('active');
      }
    });
  }
};

// 3. STATE MANAGEMENT

const State = {
  supervisors: [],
  queue: [], 
  queueArchive: [], 
  historyCache: [],
  cards: [],
  cardIdSeq: 1,
  currentCardFilterId: null,
  cardNameHints: new Map(), 
  filterHints: new Map(),   
  queueModalOpen: false,

  init() {
    this.loadHistory();
    this.loadCards();
  },

  loadHistory() {
    try {
      const raw = localStorage.getItem(CONFIG.KEYS.HISTORY);
      this.historyCache = raw ? JSON.parse(raw) : [];
    } catch { this.historyCache = []; }
  },
  
  saveHistory() {
    localStorage.setItem(CONFIG.KEYS.HISTORY, JSON.stringify(this.historyCache.slice(0, 200)));
  },

  loadCards() {
    try {
      const raw = localStorage.getItem(CONFIG.KEYS.CARDS);
      if (!raw) {
        if (!this.cards.length) this.addCard(); 
        return;
      }
      
      const parsed = JSON.parse(raw);
      if (Array.isArray(parsed)) {
        parsed.forEach((c) => {
          if (c.filter) {
             c.filter.selectedKeyTokens = c.filter.selectedKeyTokens || [];
             c.filter.customItems = c.filter.customItems || [];
             c.filter.selectedRunes = c.filter.selectedRunes || [];         
             c.filter.selectedGems = c.filter.selectedGems || [];           
             c.filter.allowedQualities = c.filter.allowedQualities || [];   
          }
          this.addCard({ ...c, pinned: true });
        });
        
        const maxId = parsed.reduce((m, c) => Math.max(m, c.id || 0), 0);
        this.cardIdSeq = Math.max(this.cardIdSeq, maxId + 1);
      }
    } catch (e) { 
      console.warn("Failed to load pinned cards. Clearing cache.", e); 
      localStorage.removeItem(CONFIG.KEYS.CARDS); 
    }
    
    if (!this.cards.length) this.addCard();
  },

  saveCards() {
    const pinned = this.cards.filter(c => c.pinned);
    localStorage.setItem(CONFIG.KEYS.CARDS, JSON.stringify(pinned));
  },

  addCard(prefill = {}) {
    const id = prefill.id || this.cardIdSeq++;
    this.cards.push({
      id,
      name: prefill.name || "",
      room: prefill.room || "",
      password: prefill.password || "",
      delay: prefill.delay || 15,
      filter: prefill.filter || this.defaultFilter(),
      supervisors: prefill.supervisors ? [...prefill.supervisors] : [],
      pinned: !!prefill.pinned,
    });
    return id;
  },

  removeCard(id) {
    if (id === 1) return; 
    this.cards = this.cards.filter(c => c.id !== id);
    this.saveCards();
  },

  getCard(id) { return this.cards.find(c => c.id === id); },
  
  defaultFilter: () => ({
    enabled: false, DropperOnlySelected: true,
    selectedRunes: [], selectedGems: [], selectedKeyTokens: [], customItems: [], allowedQualities: []
  }),

  getHintKey(sup, room) {
      return `${sup}|${(room||"").toLowerCase().trim()}`;
  },

  findExistingEntry(sup, room, id) {
    if (id) {
        const byId = this.queueArchive.find(e => e.id === id);
        if (byId) return byId;
    }
    const targetRoom = (room || "").toLowerCase().trim();
    const now = Date.now();
    
    const candidates = this.queueArchive.filter(e => 
        e.supervisor === sup && 
        (e.room || "").toLowerCase().trim() === targetRoom
    ).sort((a, b) => b.createdAt - a.createdAt);

    if (candidates.length > 0) {
        const latest = candidates[0];
        if ((now - latest.createdAt) < 600000) { 
            return latest;
        }
    }
    return null;
  },

  mergeQueue(serverQueue) {
    const now = Date.now();
    
    serverQueue.forEach(entry => {
      const card = this.findCardForEntry(entry);
      const hintKey = this.getHintKey(entry.supervisor, entry.room);
      
      if (card) {
          this.cardNameHints.set(hintKey, this.cardDisplayName(card));
          if (!this.filterHints.has(hintKey)) {
              this.filterHints.set(hintKey, this.hasActiveFilter(card.filter));
          }
      }

      const existing = this.findExistingEntry(entry.supervisor, entry.room, entry.id);

      if (existing) {
        Object.assign(existing, { 
            status: entry.status || existing.status, 
            nextAction: entry.nextAction, 
            updatedAt: now 
        });
        if (entry.id) existing.id = entry.id; 
        
        if (this.filterHints.has(hintKey)) {
            existing.filterEnabled = this.filterHints.get(hintKey);
        }
      } else {
        this.queueArchive.push({
          id: entry.id || `${now}-${entry.supervisor}-${entry.room}`,
          supervisor: entry.supervisor, room: entry.room,
          status: entry.status || "pending", nextAction: entry.nextAction,
          cardId: card?.id, cardName: card ? this.cardDisplayName(card) : undefined,
          filterEnabled: card ? this.hasActiveFilter(card.filter) : (this.filterHints.get(hintKey) || false),
          createdAt: now, updatedAt: now
        });
      }
    });
  },

  updateArchiveStatus(sup, room, status, nextAction, card) {
    const existing = this.findExistingEntry(sup, room);
    const hintKey = this.getHintKey(sup, room);

    if (existing) {
      existing.status = status;
      if (nextAction) existing.nextAction = nextAction;
      existing.updatedAt = Date.now();
      
      if (this.filterHints.has(hintKey)) {
          existing.filterEnabled = this.filterHints.get(hintKey);
      }
    } else {
      let foundCard = card;
      if (!foundCard) foundCard = this.findCardForEntry({ supervisor: sup, room: room });
      
      let resolvedName = foundCard ? this.cardDisplayName(foundCard) : undefined;
      if (!resolvedName) resolvedName = this.cardNameHints.get(hintKey);

      let isFilterOn = foundCard ? this.hasActiveFilter(foundCard.filter) : false;
      if (!foundCard && this.filterHints.has(hintKey)) {
          isFilterOn = this.filterHints.get(hintKey);
      }

      this.queueArchive.push({
        supervisor: sup, room, status: status || 'failed', nextAction: nextAction || "",
        cardId: foundCard?.id, 
        cardName: resolvedName || "Manual Run",
        filterEnabled: isFilterOn,
        createdAt: Date.now(), updatedAt: Date.now()
      });
    }
  },

  findCardForEntry(entry) {
    if (!entry?.room) return null;
    const targetRoom = entry.room.trim().toLowerCase();
    const byRoom = this.cards.filter(c => (c.room||"").trim().toLowerCase() === targetRoom);
    if (byRoom.length > 0) {
        return byRoom.find(c => c.supervisors.includes(entry.supervisor)) || byRoom[0];
    }
    return null;
  },

  cardDisplayName(card) {
    if (card && card.name && card.name.trim().length) {
        return card.name.trim(); 
    }
    if (card.id === 1) return "Drop Now"; 
    return `Drop Run #${card.id}`;
  },

  hasActiveFilter(f) { 
    if(!f) return false;
    return f.enabled || (f.selectedRunes?.length || f.selectedGems?.length || f.selectedKeyTokens?.length || f.customItems?.length || f.allowedQualities?.length); 
  }
};

const DropModeModal = {
  modal: null,

  init() {
    this.modal = $.get("dm-drop-mode-modal");
  },

  open() {
    if (!this.modal) return;
    DropPreferences.syncUI();
    this.modal.style.display = "flex";
  },

  close() {
    if (!this.modal) return;
    this.modal.style.display = "none";
  },

  getSelectedMode() {
    const checked = document.querySelector("input[name='dm-drop-mode-option']:checked");
    return checked?.value || DropPreferences.currentMode;
  }
};

// 4. API LAYER

const API = {
  async req(url, { method = "GET", body, errorMsg } = {}) {
    try {
      const res = await fetch(url, {
        method,
        headers: { "Content-Type": "application/json" },
        body: body ? JSON.stringify(body) : undefined
      });
      if (!res.ok) throw new Error((await res.text()) || errorMsg || "Request failed");
      return await res.json().catch(() => ({}));
    } catch (err) {
      if (err.name !== "AbortError" && !err.message?.includes("fetch")) throw err;
      return {}; 
    }
  },

  async getStatus() {
    const data = await API.req("/api/Drop/status", { errorMsg: "Failed to load status" });
    if (!data) return;
    
    State.supervisors = (data.supervisors || []).sort((a, b) => a.name.localeCompare(b.name));
    State.queue = data.queue || [];
    State.mergeQueue(State.queue);
    
    if (data.history) {
      const newHist = data.history.filter(h => !State.historyCache.some(c => c.timestamp === h.timestamp));
      State.historyCache = [...newHist, ...State.historyCache].slice(0, 200);
      State.saveHistory();
      
      newHist.forEach(h => {
        const status = (h.result||"").match(/success/i) ? "completed" : "failed";
        const msg = h.errorMessage || h.result; 
        State.updateArchiveStatus(h.supervisor, h.room, status, msg);
      });
    }

    UI.renderAll();
  },

  batchDrop: (payload) => API.req("/api/Drop/batch", { method: "POST", body: payload }),
  cancelDrop: (supervisor) => API.req("/api/Drop/cancel", { method: "POST", body: { supervisor } }),
  applyFilter: (sup, payload) => API.req(`/api/Drop/protection?supervisor=${encodeURIComponent(sup)}`, { method: "POST", body: payload }),
  startDropper: (sup, body) => API.req(`/api/Drop/start-Dropper?supervisor=${encodeURIComponent(sup)}`, { method: "POST", body }),
  startSupervisor: (name) => fetch(`/start?characterName=${encodeURIComponent(name)}`)
};

/**
 * ============================================================================
 * 5. UI RENDERERS
 * ============================================================================
 */
const UI = {
  renderAll() {
    this.renderSupervisors();
    this.renderCards();
    this.renderHistory();
    if (State.queueModalOpen) this.renderQueue();
  },

  renderSupervisors() {
    const container = $.get("dm-supervisor-list");
    if (!container) return;
    container.innerHTML = "";

    if (!State.supervisors.length) {
      container.appendChild($.el("p", { style: { opacity: 0.7 } }, "No supervisors available."));
      return;
    }

    State.supervisors.forEach(sup => {
      const queueEntry = State.queue.find(q => q.supervisor === sup.name);
      
      const row = $.el("div", { className: "supervisor-tag" },
        $.el("div", { className: "supervisor-info" },
          $.el("div", { className: "supervisor-row" },
            $.el("span", {}, sup.name),
            $.el("div", { className: `status-indicator ${$.statusClass(sup)}`, title: sup.state })
          ),
          queueEntry 
            ? $.el("div", { className: "supervisor-meta" }, 
                $.el("span", { className: "queue-inline-status" }, `Room: ${queueEntry.room || '-'}  Status: ${queueEntry.status}`))
            : (sup.running && sup.room)
              ? $.el("div", { className: "supervisor-meta" }, $.el("span", {}, `Last Room: ${sup.room}`), $.el("span", {}, `Since: ${$.formatTime(sup.since)}`))
              : null
        ),
        $.el("div", { className: "supervisor-actions" },
          queueEntry
            ? $.el("button", { className: "btn btn-outline btn-small", on: { click: () => API.cancelDrop(sup.name).then(API.getStatus) } }, "Cancel")
            : (sup.running 
                ? $.el("button", { className: "btn btn-outline btn-small", on: { click: () => Handlers.queueDrop([sup.name]) } }, "Dropper")
                : $.el("button", { className: "btn btn-primary btn-small", on: { click: (e) => Handlers.startAndDropper(sup.name, e.target) } }, "Express")
              )
        )
      );
      container.appendChild(row);
    });
  },

  renderCards() {
    const container = $.get("dm-card-list");
    if (!container) return;
    container.innerHTML = "";
    
    if (!State.cards.length) {
      container.appendChild($.el("p", { style: { opacity: 0.7 } }, "No Drop cards added yet."));
      return;
    }

    State.cards.forEach(card => {
      const activeFilter = State.hasActiveFilter(card.filter);
      
      const el = $.el("div", { className: "Drop-card Drop-card--inline" },
        $.el("div", { className: "card-head-row" },
          $.el("div", { className: "card-title-row" },
            $.el("button", { 
              className: `icon-button card-pin-button ${card.pinned ? 'pinned' : ''}`, 
              title: "Pin card",
              on: { click: () => { card.pinned = !card.pinned; State.saveCards(); UI.renderCards(); } }
            }, $.el("i", { className: `bi bi-pin-angle${card.pinned ? '-fill' : ''}` })),
            $.el("strong", { 
              className: "card-title-text", style: { cursor: "pointer" },
              on: { click: () => { const n = prompt("Name", State.cardDisplayName(card)); if(n!==null) { card.name=n; State.saveCards(); UI.renderCards(); }}}
            }, State.cardDisplayName(card), " ", $.el("i", { className: "bi bi-pencil-square", style: { fontSize: "0.8em", opacity: 0.7 } }))
          ),
          
          UI.createFilterSummaryPreview(card.filter),

          $.el("div", { style: { display: "flex", gap: "0.55rem", alignItems: "center" } },
            $.el("button", { 
              className: `btn btn-filter btn-small${activeFilter ? " active" : ""}`,
              on: { click: () => Handlers.openFilter(card.id) },
              style: { display: "flex", alignItems: "center", gap: "0.15rem" }
            },
              $.el("i", { className: "bi bi-funnel-fill" }),
              activeFilter ? "Filter (On)" : "Filter"
            ),

            $.el("button", { 
                className: "btn btn-primary btn-small", 
                on: { click: () => Handlers.requestCard(card) } 
            }, $.el("i", { className: "bi bi-send-fill" }), " Request"),

            $.el("button", { 
              className: "btn btn-danger btn-small", 
              on: { click: () => { if(confirm(card.id===1 ? "Reset?" : "Delete?")) card.id===1 ? Handlers.resetCard(card) : State.removeCard(card.id); UI.renderCards(); }},
              style: { display: "flex", alignItems: "center", gap: "0.15rem" }
            },
              $.el("i", { className: "bi bi-arrow-counterclockwise" }),
              card.id===1 ? "Reset" : "Delete"
            )
          )
        ),
        
        $.el("div", { className: "Drop-form-grid" },
          this.inputField("Room", "text", card.room, v => { card.room = v; State.saveCards(); }),
          this.inputField("Password", "text", card.password, v => { card.password = v; State.saveCards(); }),
          this.inputField("Delay (seconds)", "number", card.delay, v => { card.delay = parseInt(v)||0; State.saveCards(); })
        ),
        $.el("div", { className: "card-supervisors" },
          $.el("button", { className: "card-subtitle card-subtitle-selectall", on: { click: () => {
             const all = State.supervisors.map(s => s.name);
             card.supervisors = card.supervisors.length === all.length ? [] : all;
             State.saveCards(); UI.renderCards();
          }}}, "Supervisors (Select All)"),
          $.el("div", { className: "card-supervisor-list" },
             State.supervisors.length ? State.supervisors.map(sup => $.el("label", { className: "card-supervisor-chip" },
                $.el("input", { 
                  type: "checkbox", checked: card.supervisors.includes(sup.name),
                  on: { change: (e) => { 
                    e.target.checked ? card.supervisors.push(sup.name) : (card.supervisors = card.supervisors.filter(n => n !== sup.name));
                    State.saveCards();
                  }}
                }),
                $.el("span", { className: "card-supervisor-name" }, sup.name),
                $.el("div", { className: `status-indicator ${$.statusClass(sup)}` })
             )) : $.el("p", { style: { opacity: 0.7 } }, "No supervisors")
          )
        )
      );
      container.appendChild(el);
    });
  },

  inputField(label, type, val, onChange) {
    return $.el("label", {}, label, $.el("input", { type, value: val, on: { input: (e) => onChange(e.target.value) } }));
  },

  renderHistory() {
    const tbody = document.querySelector("#dm-history-table tbody");
    if (!tbody) return;
    tbody.innerHTML = "";
    if (!State.historyCache.length) {
      tbody.appendChild($.el("tr", {}, $.el("td", { colSpan: 5, style: { textAlign: "center" } }, "No drop history yet.")));
      return;
    }
    State.historyCache.forEach(h => {
      const color = (h.result||"").match(/success/i) ? "#2ecc71" : "#ff6384";
      tbody.appendChild($.el("tr", {},
        $.el("td", {}, $.formatTime(h.timestamp)),
        $.el("td", {}, h.cardName || "-"),
        $.el("td", {}, $.el("span", { title: h.supervisor }, h.supervisor)),
        $.el("td", {}, h.room),
        $.el("td", { style: { color } }, h.result)
      ));
    });
  },

  renderQueue() {
    const body = $.get("dm-queue-body");
    if (!body) return;
    
    if (!State.queueArchive.length) {
      body.innerHTML = `
        <div style="display:flex; flex-direction:column; align-items:center; justify-content:center; height:150px; color:var(--text-sub);">
            <i class="bi bi-list-task" style="font-size:2rem; margin-bottom:0.5rem; opacity:0.5;"></i>
            <span>No tasks found</span>
        </div>`;
      return;
    }

    const tableHeader = $.el("div", { className: "queue-table-header" },
        $.el("div", { className: "col-time" }, "Req. Time"),
        $.el("div", { className: "col-run" }, "Run Name"),
        $.el("div", { className: "col-room" }, "Room"),
        $.el("div", { className: "col-filter" }, "Filter"),
        $.el("div", { className: "col-sup" }, "Sup"),
        $.el("div", { className: "col-status" }, "Status"),
        $.el("div", { className: "col-done" }, "Completed")
    );

    const listContainer = $.el("div", { className: "queue-list-container" });
    const sortedQueue = State.queueArchive.slice().sort((a, b) => b.createdAt - a.createdAt);

    sortedQueue.forEach(e => {
        const status = (e.status || "pending").toLowerCase();
        const isFinished = ["completed", "success", "failed", "timeout", "done", "error"].includes(status);
        const doneTime = isFinished ? $.formatTime(e.updatedAt) : "-";
        const runName = e.cardName || "Manual";
        const roomName = e.room || "-";
        const filterState = e.filterEnabled ? "ON" : "OFF";
        const filterClass = e.filterEnabled ? "badge-filter-on" : "badge-filter-off";

        const row = $.el("div", { className: "queue-table-row" },
            $.el("div", { className: "col-time" }, $.formatTime(e.createdAt)),
            $.el("div", { className: "col-run", title: runName }, runName),
            $.el("div", { className: "col-room", title: roomName }, roomName),
            $.el("div", { className: "col-filter" }, $.el("span", { className: filterClass }, filterState)),
            $.el("div", { className: "col-sup" }, 
            $.el("i", { className: "bi bi-person-fill", style:{marginRight:"4px", opacity:0.7} }), e.supervisor),
            $.el("div", { className: "col-status" }, $.el("span", { className: `q-badge q-badge-${status}` }, status)),
            $.el("div", { className: "col-done" }, doneTime)
        );
        listContainer.appendChild(row);
    });

    body.innerHTML = "";
    body.appendChild(tableHeader);
    body.appendChild(listContainer);
  },

  buildFilterOptions() {
    const makeItem = (label, value) => $.el("div", { className: "filter-checkbox-item" },
        $.el("label", {}, $.el("input", { type: "checkbox", value }), label),
        $.el("input", { 
            type: "number", 
            className: "quantity-value", 
            value: 0, 
            min: 0, 
            placeholder: "All", 
            title: "0 means Drop All (Unlimited)",
            on: { input: (e) => { if(e.target.value < 0) e.target.value = 0; } }
        })
    );
    const runeBox = $.get("dm-card-rune-checkboxes");
    if(runeBox) { runeBox.innerHTML=""; CONFIG.RUNES.forEach(r => runeBox.appendChild(makeItem(r.label, r.value))); }
    const gemBox = $.get("dm-card-gem-checkboxes");
    if(gemBox) { gemBox.innerHTML=""; CONFIG.GEMS.forEach(g => gemBox.appendChild(makeItem(g.label, g.value))); }
    const keyTokenBox = $.get("dm-card-keytoken-checkboxes");
    if(keyTokenBox) { keyTokenBox.innerHTML=""; CONFIG.KEY_TOKENS.forEach(k => keyTokenBox.appendChild(makeItem(k.label, k.value))); }
  },

  populateFilterModal(filter) {
    const enableToggle = $.get("dm-card-filter-enabled");
    if (enableToggle) enableToggle.checked = !!filter.enabled;

    const modeVal = filter.DropperOnlySelected ? "exclusive" : "inclusive";
    document.querySelectorAll("input[name='dm-card-filter-mode']").forEach(r => {
        r.checked = r.value === modeVal;
    });

    $.setCheckboxValues("dm-card-rune-checkboxes", filter.selectedRunes || []);
    $.setCheckboxValues("dm-card-gem-checkboxes", filter.selectedGems || []);
    $.setCheckboxValues("dm-card-keytoken-checkboxes", filter.selectedKeyTokens || []);
    $.setVal("dm-card-custom-items", (filter.customItems || []).join("\n"));

    const qualities = filter.allowedQualities || [];
    document.querySelectorAll("#dm-card-quality-checkboxes input").forEach(cb => {
        cb.checked = qualities.includes(cb.value);
    });
  },

  createFilterSummaryPreview(filter) {
    const container = $.el("div", { className: "filter-summary-preview" });
    if (!filter.enabled) {
        return container; 
    }

    let items = [];
    const maxChips = 10; 

    const getFormattedItems = (values) => {
        return (values || []).map(v => {
            const name = typeof v === 'string' ? v : v.name;
            let displayName = name;
            if (displayName.endsWith('Rune')) displayName = displayName.slice(0, -4); 
            else if (displayName.startsWith('Perfect')) displayName = displayName.replace('Perfect', '');
            return displayName.trim();
        });
    };

    items.push(...getFormattedItems(filter.selectedRunes));
    items.push(...getFormattedItems(filter.selectedGems));
    items.push(...getFormattedItems(filter.selectedKeyTokens)); 
    
    if (filter.allowedQualities?.length) {
        const qualityLabels = {
            base: "White",
            magic: "Magic",
            rare: "Rare",
            set: "Set",
            unique: "Unique",
            crafted: "Crafted",
            runeword: "Runeword"
        };
        items.push(...filter.allowedQualities.map(q => qualityLabels[q] || q));
    }
    if (filter.customItems?.length) items.push(`Custom (${filter.customItems.length})`);

    const chipsToDisplay = items.slice(0, maxChips);
    chipsToDisplay.forEach(item => {
        container.appendChild($.el("span", { className: "filter-summary-chip" }, item));
    });

    if (items.length > maxChips) {
        container.appendChild($.el("span", { className: "filter-summary-chip filter-more" }, `+${items.length - maxChips} more`));
    }

    return container;
  },

  renderFilterSummary() {
    const container = $.get("dm-filter-summary-chips");
    if (!container) return;
    container.innerHTML = "";

    const createChip = (text, onClickRemove) => {
        const chip = $.el("span", { className: "filter-chip" },
            text,
            $.el("span", { 
                className: "chip-close", 
                title: "Remove",
                on: { click: (e) => { e.stopPropagation(); onClickRemove(); } } 
            }, "×")
        );
        container.appendChild(chip);
    };
    
    const formatItemName = (name) => {
        let displayName = name;
        if (displayName.endsWith('Rune')) {
            displayName = displayName.slice(0, -4); 
        } else if (displayName.startsWith('Perfect')) {
            displayName = displayName.replace('Perfect', ''); 
        }
        return displayName.trim();
    };

    let hasItems = false;

    const runes = $.getCheckedValues("dm-card-rune-checkboxes");
    const gems = $.getCheckedValues("dm-card-gem-checkboxes");
    const keyTokens = $.getCheckedValues("dm-card-keytoken-checkboxes"); 
    
    [...runes, ...gems, ...keyTokens].forEach(item => {
        hasItems = true;
        const qtyText = item.quantity > 0 ? `(${item.quantity})` : "(All)";
        const displayName = formatItemName(item.name);
        
        createChip(`${displayName} ${qtyText}`, () => {
            const cb = document.querySelector(`input[value="${item.name}"]`);
            if(cb) { cb.checked = false; cb.dispatchEvent(new Event('change', {bubbles:true})); }
        });
    });

    const qualityLabels = {
        base: "White",
        magic: "Magic",
        rare: "Rare",
        set: "Set",
        unique: "Unique",
        crafted: "Crafted",
        runeword: "Runeword"
    };

    document.querySelectorAll("#dm-card-quality-checkboxes input:checked").forEach(cb => {
        hasItems = true;
        const label = qualityLabels[cb.value] || cb.value;
        createChip(`Quality: ${label}`, () => {
            cb.checked = false; 
            cb.dispatchEvent(new Event('change', {bubbles:true}));
        });
    });

    const customText = $.val("dm-card-custom-items");
    if (customText.trim()) {
        const lines = customText.split("\n").filter(l => l.trim());
        if (lines.length > 0) {
            hasItems = true;
            const displayCount = lines.length === 1 ? lines[0] : `Custom Items (${lines.length})`;
            createChip(displayCount, () => {
                if(confirm("Clear all custom items?")) {
                    $.setVal("dm-card-custom-items", "");
                    $.get("dm-card-custom-items").dispatchEvent(new Event('change', {bubbles:true}));
                }
            });
        }
    }

    if (!hasItems) {
        container.innerHTML = '<span style="color:#555; font-size:0.8rem; font-style:italic;">No items selected</span>';
    }
  },

  hasUserSelection() {
    return $.getCheckedValues("dm-card-rune-checkboxes").length > 0 ||
           $.getCheckedValues("dm-card-gem-checkboxes").length > 0 ||
           $.getCheckedValues("dm-card-keytoken-checkboxes").length > 0 ||
           document.querySelectorAll("#dm-card-quality-checkboxes input:checked").length > 0 ||
           $.val("dm-card-custom-items").trim().length > 0;
  }
};

// 6. EVENT HANDLERS

const Handlers = {
  queueDrop(supervisors) {
    if (DropPreferences.requireFilter()) {
      return $.toast("Drop mode requires an enabled filter before requesting runs.", "error");
    }
    if (!supervisors.length) return $.toast("Select a supervisor", "error");
    const room = $.val("dm-room");
    if (!room) return $.toast("Room required", "error");
    
    localStorage.setItem(CONFIG.KEYS.ROOM, room);
    const pwd = $.val("dm-password");
    localStorage.setItem(CONFIG.KEYS.PASSWORD, pwd);

    API.batchDrop({ supervisors, room, password: pwd, delaySeconds: Number($.val("dm-delay")) || 15 })
      .then(API.getStatus)
      .catch(e => $.toast(e.message, "error"));
  },

requestCard(card) {
  if (!card.room || !card.supervisors.length) 
    return $.toast("Room and Supervisors required", "error");
  if (DropPreferences.requireFilter() && !State.hasActiveFilter(card.filter)) {
    return $.toast("Drop mode requires an enabled filter before requesting this card.", "error");
  }
  
  const now = Date.now();
  card.supervisors.forEach((s, i) => {
      State.queueArchive.push({ 
          id: `${now}-${i}`, 
          supervisor: s, 
          room: card.room, 
          status: 'pending', 
          nextAction: 'queued',
          cardId: card.id, 
          cardName: State.cardDisplayName(card), 
          createdAt: now
      });
  });
  UI.renderQueue();

  const filter = card.filter || State.defaultFilter();
  
  const filterPayload = {
    enabled: !!filter.enabled,
    DropperOnlySelected: filter.DropperOnlySelected,
    selectedRunes: filter.selectedRunes || [],
    selectedGems: filter.selectedGems || [],
    selectedKeyTokens: filter.selectedKeyTokens || [],
    allowedQualities: filter.allowedQualities || [],
    customItems: filter.customItems || []
  };

  const online = card.supervisors.filter(n => 
    State.supervisors.find(s => s.name === n && s.running)
  );
  const offline = card.supervisors.filter(n => !online.includes(n));
  

  if (online.length) {
    API.batchDrop({ 
      supervisors: online, 
      room: card.room, 
      password: card.password, 
      delaySeconds: card.delay,
      filter: filterPayload,
      cardId: card.id,
      cardName: State.cardDisplayName(card)
    });
  }
  
  offline.forEach((name, idx) => {
    setTimeout(() => {
      API.startDropper(name, { 
        room: card.room, 
        password: card.password,
        cardId: card.id,
        cardName: State.cardDisplayName(card),
        filter: filterPayload
      })
      .then(() => API.startSupervisor(name))
      .catch(e => $.toast(`Start failed: ${name}`, 'error'));
    }, idx * (card.delay * 1000));
  });
  
  $.toast(`Queued for ${card.supervisors.length} supervisors`);
},

  submitAllCards() {
    if(!State.cards.length) return $.toast("No cards", "error");
    if(!confirm("Request Drop for ALL cards?")) return;
    let chain = Promise.resolve();
    State.cards.forEach((card, i) => {
        chain = chain.then(() => {
            Handlers.requestCard(card);
            return new Promise(r => setTimeout(r, (card.delay * 1000) || 2000));
        });
    });
  },

  resetCard(card) {
    Object.assign(card, { name: "", room: "", password: "", delay: 15, supervisors: [], filter: State.defaultFilter() });
    State.saveCards(); 
    UI.renderCards();
  },

  openFilter(cardId) {
    const card = State.getCard(cardId);
    if (!card) return;
    State.currentCardFilterId = cardId;
    const modal = $.get("dm-card-filter-modal");
    
    const title = $.get("dm-card-filter-title");
    if(title) title.textContent = `${State.cardDisplayName(card)} Filter`;
    
    const f = card.filter || State.defaultFilter();
    UI.populateFilterModal(f); 

    if(modal) modal.style.display = "flex";
    UI.renderFilterSummary(); 
  },

  saveFilter() {
    const card = State.getCard(State.currentCardFilterId);
    if(!card) return;
    
    const mode = document.querySelector("input[name='dm-card-filter-mode']:checked");
    const qualities = [];
    document.querySelectorAll("#dm-card-quality-checkboxes input:checked").forEach(cb => qualities.push(cb.value));

    const customLines = $.val("dm-card-custom-items").split("\n").map(s => s.trim()).filter(Boolean);


    card.filter = {
        enabled: $.get("dm-card-filter-enabled")?.checked,
        DropperOnlySelected: mode ? mode.value === "exclusive" : true,
        selectedRunes: $.getCheckedValues("dm-card-rune-checkboxes"),
        selectedGems: $.getCheckedValues("dm-card-gem-checkboxes"),
        selectedKeyTokens: $.getCheckedValues("dm-card-keytoken-checkboxes"),
        allowedQualities: qualities,
        customItems: customLines
    };
    State.saveCards();
    UI.renderCards();
    $.get("dm-card-filter-modal").style.display = "none";
  },

  resetFilterForm() {
    if (!confirm("Reset all selections in this filter?")) return;
    UI.populateFilterModal(State.defaultFilter());
    UI.renderFilterSummary(); 
    $.toast("Filter selections reset to default.");
  },

  startAndDropper(name, btn) {
    if (DropPreferences.requireFilter()) {
      return $.toast("Drop mode requires a filter before starting a run.", "error");
    }
    const room = $.val("dm-room");
    if (!room) return $.toast("Room required", "error");
    if (btn) btn.disabled = true;
    API.startDropper(name, { room, password: $.val("dm-password") })
      .then(() => API.startSupervisor(name))
      .then(() => setTimeout(API.getStatus, 5000)) 
      .catch(e => $.toast(e.message, "error"))
      .finally(() => { if (btn) btn.disabled = false; });
  },

  bindFilterEvents() {
    const modalBody = $.get("dm-card-filter-body");
    if (modalBody) {
        modalBody.addEventListener("change", (e) => {
            if (e.target.type === "checkbox" || e.target.tagName === "TEXTAREA") {
                const enableToggle = $.get("dm-card-filter-enabled");
                const hasSelection = UI.hasUserSelection();
                
                if (hasSelection) {
                    if (enableToggle && !enableToggle.checked) enableToggle.checked = true;
                } else {
                    if (enableToggle && enableToggle.checked) enableToggle.checked = false;
                }
                UI.renderFilterSummary(); 
            }
        });
    }

    const enableToggle = $.get("dm-card-filter-enabled");
    if (enableToggle) {
        enableToggle.addEventListener("click", (e) => {
            if (enableToggle.checked) {
                if (!UI.hasUserSelection()) {
                    e.preventDefault(); 
                    enableToggle.checked = false; 
                    $.toast("Select at least one item first!", "error");
                }
            }
        });
    }
  }
};

// 7. INITIALIZATION

document.addEventListener("DOMContentLoaded", () => {
  State.init();
  UI.buildFilterOptions();
  DropModeModal.init();
  DropPreferences.init();
  DropPreferences.maybePromptInitialMode();
  
  $.setVal("dm-room", localStorage.getItem(CONFIG.KEYS.ROOM) || "");
  $.setVal("dm-password", localStorage.getItem(CONFIG.KEYS.PASSWORD) || "");

  UI.renderCards(); 

  const bind = (id, evt, fn) => { const el = $.get(id); if(el) el.addEventListener(evt, (e) => { e.preventDefault(); fn(e); }); };
  
  bind("dm-add-card", "click", () => { State.addCard(); UI.renderCards(); });
  bind("dm-request-cards", "click", Handlers.submitAllCards);
  bind("dm-open-queue", "click", () => { State.queueModalOpen = true; UI.renderQueue(); $.get("dm-queue-modal").style.display = "flex"; });
  bind("dm-queue-close", "click", () => { State.queueModalOpen = false; $.get("dm-queue-modal").style.display = "none"; });
  bind("dm-queue-clear", "click", () => { State.queueArchive = []; UI.renderQueue(); });
  bind("dm-history-clear", "click", () => { State.historyCache = []; State.saveHistory(); UI.renderHistory(); });
  
  bind("dm-card-filter-close", "click", () => $.get("dm-card-filter-modal").style.display = "none");
  bind("dm-card-filter-save", "click", Handlers.saveFilter);
  bind("dm-card-filter-reset", "click", Handlers.resetFilterForm);
  bind("dm-drop-mode-settings", "click", () => DropModeModal.open());
  bind("dm-drop-mode-save", "click", () => {
    const mode = DropModeModal.getSelectedMode();
    if (mode) {
      DropPreferences.set(mode);
      DropModeModal.close();
      $.toast("Drop preferences saved.", "info");
    }
  });
  bind("dm-drop-mode-close", "click", () => DropModeModal.close());

  ["rune", "gem", "quality", "keytoken"].forEach(type => {
      const toggle = $.get(`dm-card-${type}-selectall`);
      const box = $.get(`dm-card-${type}-checkboxes`);
      if(toggle && box) {
          toggle.addEventListener("change", () => {
              box.querySelectorAll("input[type='checkbox']").forEach(cb => {
                  cb.checked = toggle.checked; 
                  cb.dispatchEvent(new Event("change", { bubbles: true }));
              });
          });
      }
      if(box) box.addEventListener("change", (e) => {
          if(e.target.type === "checkbox") {
             const qty = e.target.closest(".filter-checkbox-item")?.querySelector(".quantity-value");
             if(qty) { 
                 if(e.target.checked) { qty.classList.add("active"); if(!qty.value) qty.value=0; }
                 else qty.classList.remove("active");
             }
          }
      });
  });

  Handlers.bindFilterEvents();

  API.getStatus();
  setInterval(API.getStatus, 5000);
});
