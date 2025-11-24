const ROOM_STORAGE_KEY = "deliveryManager.lastRoom";
const PASSWORD_STORAGE_KEY = "deliveryManager.lastPassword";

const runeOptions = [
  { label: "El", value: "ElRune" },
  { label: "Eld", value: "EldRune" },
  { label: "Tir", value: "TirRune" },
  { label: "Nef", value: "NefRune" },
  { label: "Eth", value: "EthRune" },
  { label: "Ith", value: "IthRune" },
  { label: "Tal", value: "TalRune" },
  { label: "Ral", value: "RalRune" },
  { label: "Ort", value: "OrtRune" },
  { label: "Thul", value: "ThulRune" },
  { label: "Amn", value: "AmnRune" },
  { label: "Sol", value: "SolRune" },
  { label: "Shael", value: "ShaelRune" },
  { label: "Dol", value: "DolRune" },
  { label: "Hel", value: "HelRune" },
  { label: "Io", value: "IoRune" },
  { label: "Lum", value: "LumRune" },
  { label: "Ko", value: "KoRune" },
  { label: "Fal", value: "FalRune" },
  { label: "Lem", value: "LemRune" },
  { label: "Pul", value: "PulRune" },
  { label: "Um", value: "UmRune" },
  { label: "Mal", value: "MalRune" },
  { label: "Ist", value: "IstRune" },
  { label: "Gul", value: "GulRune" },
  { label: "Vex", value: "VexRune" },
  { label: "Ohm", value: "OhmRune" },
  { label: "Lo", value: "LoRune" },
  { label: "Sur", value: "SurRune" },
  { label: "Ber", value: "BerRune" },
  { label: "Jah", value: "JahRune" },
  { label: "Cham", value: "ChamRune" },
  { label: "Zod", value: "ZodRune" },
];

const gemOptions = [
  { label: "Amethyst", value: "PerfectAmethyst" },
  { label: "Diamond", value: "PerfectDiamond" },
  { label: "Emerald", value: "PerfectEmerald" },
  { label: "Ruby", value: "PerfectRuby" },
  { label: "Sapphire", value: "PerfectSapphire" },
  { label: "Topaz", value: "PerfectTopaz" },
  { label: "Skull", value: "PerfectSkull" },
];

const deliveryState = {
  supervisors: [],
  queue: [],
  history: [],
  globalFilters: {
    enabled: false,
    deliverOnlySelected: true,
    selectedRunes: [],
    selectedGems: [],
    customItems: [],
  },
  individualFilters: {},
  currentModalSupervisor: null,
};

const selectedSupervisors = new Set();
let statusTimer = null;
let filterSyncBlocked = false;
let filterInputActive = false;
let filterSaveTimer = null;

function showToast(message, type = 'info') {
  const container = document.getElementById('toast-container');
  const toast = document.createElement('div');
  toast.className = `toast ${type}`;
  toast.textContent = message;
  
  container.appendChild(toast);
  
  // Trigger animation
  setTimeout(() => toast.classList.add('show'), 10);
  
  // Auto remove after 3 seconds
  setTimeout(() => {
    toast.classList.remove('show');
    setTimeout(() => container.removeChild(toast), 300);
  }, 3000);
}

document.addEventListener("DOMContentLoaded", () => {
  buildFilterCheckboxes();
  restoreSavedRoomInfo();
  fetchStatus();
  statusTimer = setInterval(fetchStatus, 5000);
  attachHandlers();
  window.addEventListener("beforeunload", disableFiltersOnExit);
  window.addEventListener("pagehide", disableFiltersOnExit);
});

function fetchStatus() {
  return loadDeliveryStatus().catch((err) => {
    // Ignore fetch errors when navigating away
    if (!err.message || !err.message.toLowerCase().includes('fetch')) {
      console.error(err);
    }
  });
}

function loadDeliveryStatus() {
  return fetch("/api/delivery/status")
    .then((res) => {
      if (!res.ok) {
        throw new Error("Failed to load delivery status");
      }
      return res.json();
    })
    .then((data) => {
      deliveryState.supervisors = (data.supervisors || []).sort((a, b) => a.name.localeCompare(b.name));
      deliveryState.queue = data.queue || [];
      deliveryState.history = data.history || [];
      
      const filters = data.filters || {};
      if (filters["global"]) {
        deliveryState.globalFilters = filters["global"];
      }

      Object.keys(filters).forEach((key) => {
        if (key !== "global") {
          deliveryState.individualFilters[key] = filters[key];
        }
      });

      const validNames = new Set(deliveryState.supervisors.map((s) => s.name));
      Array.from(selectedSupervisors).forEach((name) => {
        if (!validNames.has(name)) {
          selectedSupervisors.delete(name);
        }
      });

      renderSupervisors();
      renderHistory();
      populateGlobalFilterForm();
      updateSummary();
    })
    .catch((err) => {
      console.error(err);
      throw err;
    });
}

function renderSupervisors() {
  const container = document.getElementById("dm-supervisor-list");
  container.innerHTML = "";

  if (deliveryState.supervisors.length === 0) {
    container.innerHTML = '<p style="opacity:0.7;">No supervisors available.</p>';
    updateSelectionControls();
    return;
  }

  deliveryState.supervisors.forEach((sup) => {
    const tag = document.createElement("div");
    tag.className = "supervisor-tag";

    const infoBlock = document.createElement("div");
    infoBlock.className = "supervisor-info";

    const label = document.createElement("label");
    label.className = "supervisor-row";

    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.value = sup.name;
    checkbox.checked = selectedSupervisors.has(sup.name);
    if (!sup.running) {
      checkbox.disabled = true;
      checkbox.title = "Character is offline";
      selectedSupervisors.delete(sup.name);
    }
    checkbox.addEventListener("change", () => {
      if (checkbox.checked) {
        selectedSupervisors.add(sup.name);
      } else {
        selectedSupervisors.delete(sup.name);
      }
      updateSelectionControls();
    });

    const nameSpan = document.createElement("span");
    nameSpan.textContent = sup.name;

    const status = document.createElement("span");
    status.className = `status-pill ${statusClass(sup)}`;
    status.textContent = sup.state;

    label.appendChild(checkbox);
    label.appendChild(nameSpan);
    label.appendChild(status);

    const queueEntry = deliveryState.queue.find((entry) => entry.supervisor === sup.name);
    const meta = document.createElement("div");
    meta.className = "supervisor-meta";

    if (!sup.running) {
      meta.textContent = "";
    } else if (sup.room) {
      meta.innerHTML = `<span>Last Room: ${sup.room}</span><span>Since: ${formattedSince(sup.since)}</span>`;
    } else {
      meta.textContent = "No pending delivery.";
    }

    if (queueEntry) {
      const inlineStatus = document.createElement("span");
      inlineStatus.className = "queue-inline-status";
      const details = [];
      if (queueEntry.room) {
        details.push(`Room: ${queueEntry.room}`);
      }
      details.push(`Status: ${queueEntry.status}`);
      if (queueEntry.nextAction) {
        details.push(`Next: ${queueEntry.nextAction}`);
      }
      inlineStatus.textContent = details.join("  ");
      label.appendChild(inlineStatus);
      meta.textContent = "";
    }

    if (!meta.textContent.trim() && meta.children.length === 0) {
      meta.style.display = "none";
    } else {
      meta.style.display = "flex";
    }

    infoBlock.appendChild(label);
    infoBlock.appendChild(meta);

    const actions = document.createElement("div");
    actions.className = "supervisor-actions";


    const filterBtn = document.createElement("button");
    filterBtn.type = "button";
    const hasIndividualFilter = deliveryState.individualFilters[sup.name]?.enabled;
    filterBtn.className = `btn btn-filter btn-small${hasIndividualFilter ? ' active' : ''}`;
    const filterText = hasIndividualFilter ? 'Filter (Active)' : 'Filter';
    filterBtn.innerHTML = `<i class="bi bi-funnel-fill"></i> ${filterText}`;
    filterBtn.title = hasIndividualFilter ? 'Individual filter active - Click to edit' : 'Set individual filter';
    filterBtn.addEventListener("click", (e) => {
      e.preventDefault();
      openIndividualFilterModal(sup.name);
    });
    actions.appendChild(filterBtn);

    if (!queueEntry) {
      const deliverBtn = document.createElement("button");
      deliverBtn.type = "button";
      if (sup.running) {
        deliverBtn.className = "btn btn-outline btn-small";
        deliverBtn.textContent = "Deliver";
        deliverBtn.addEventListener("click", (e) => {
          e.preventDefault();
          deliverSingle(sup.name);
        });
      } else {
        deliverBtn.className = "btn btn-primary btn-small";
        deliverBtn.textContent = "Express";
        deliverBtn.addEventListener("click", (e) => {
          e.preventDefault();
          startAndDeliver(sup.name, deliverBtn);
        });
      }
      actions.appendChild(deliverBtn);
    }

    if (queueEntry) {

      const cancelBtn = document.createElement("button");
      cancelBtn.className = "btn btn-outline btn-small";
      cancelBtn.type = "button";
      cancelBtn.textContent = "Cancel";
      cancelBtn.addEventListener("click", (e) => {
        e.preventDefault();
        cancelDelivery(sup.name);
      });
      actions.appendChild(cancelBtn);
    }

    tag.appendChild(infoBlock);
    tag.appendChild(actions);
    container.appendChild(tag);
  });

  updateSelectionControls();
}

function handleSelectionToggle() {
  const total = deliveryState.supervisors.length;
  if (total === 0) {
    return;
  }

  if (selectedSupervisors.size === total) {
    selectedSupervisors.clear();
  } else {
    deliveryState.supervisors.forEach((sup) => selectedSupervisors.add(sup.name));
  }

  renderSupervisors();
}

function updateSelectionControls() {
  const toggleBtn = document.getElementById("dm-selection-toggle");
  const toggleLabel = document.getElementById("dm-selection-toggle-label");
  const deliverBtn = document.getElementById("dm-deliver-selected");
  const total = deliveryState.supervisors.length;
  const selected = selectedSupervisors.size;
  const allSelected = total > 0 && selected === total;
  const allSelectedOnline = Array.from(selectedSupervisors).every((name) => {
    const sup = deliveryState.supervisors.find((item) => item.name === name);
    return sup?.running;
  });

  if (toggleBtn) {
    toggleBtn.disabled = total === 0;
  }

  if (toggleLabel) {
    toggleLabel.textContent = allSelected ? "Clear Selection" : "Select All";
  }

  if (deliverBtn) {
    deliverBtn.disabled = selected === 0 || !allSelectedOnline;
    deliverBtn.innerHTML = `<i class="bi bi-truck"></i> Delivery (${selected}/${total})`;
  }
}

function statusClass(sup) {
  if (!sup.running) {
    return "offline";
  }

  switch (sup.state) {
    case "active":
      return "active";
    case "pending":
      return "pending";
    case "paused":
      return "paused";
    case "error":
      return "error";
    default:
      return "idle";
  }
}

function renderHistory() {
  const tbody = document.querySelector("#dm-history-table tbody");
  tbody.innerHTML = "";

  if (deliveryState.history.length === 0) {
    const row = document.createElement("tr");
    const cell = document.createElement("td");
    cell.colSpan = 8;
    cell.style.textAlign = "center";
    cell.textContent = "No history yet.";
    row.appendChild(cell);
    tbody.appendChild(row);
    return;
  }

  deliveryState.history.forEach((entry) => {
    const row = document.createElement("tr");
    
    // Format filter info
    let filterInfo = entry.filterApplied || "-";
    if (filterInfo !== "-" && filterInfo !== "None") {
      const modeAbbr = entry.filterMode === "Exclusive" ? "Ex" : entry.filterMode === "Inclusive" ? "In" : "";
      filterInfo = `${filterInfo.charAt(0)} (${modeAbbr})`;
    }
    
    // Format result with color
    let resultClass = "";
    if (entry.result === "Success" || entry.result === "success") resultClass = "style='color: #2ecc71;'";
    else if (entry.result === "Failed" || entry.result === "failed") resultClass = "style='color: #ff6384;'";
    else if (entry.result === "Timeout" || entry.result === "timeout") resultClass = "style='color: #ffc44d;'";
    
    row.innerHTML = `
      <td>${new Date(entry.timestamp).toLocaleTimeString()}</td>
      <td>${entry.supervisor}</td>
      <td>${entry.room}</td>
      <td>${filterInfo}</td>
      <td ${resultClass}>${entry.result}</td>
      <td>${entry.itemsDelivered || 0}</td>
      <td>${entry.duration || "-"}</td>
      <td style="font-size: 0.85em; color: rgba(255,255,255,0.7);">${entry.errorMessage || "-"}</td>
    `;
    tbody.appendChild(row);
  });
}

function updateSummary() {
  const total = deliveryState.supervisors.length;
  const queue = deliveryState.queue.length;
  const active = deliveryState.supervisors.filter((s) => s.state === "active").length;
  const last = deliveryState.history[0];

  document.getElementById("dm-summary-supervisors").textContent = total;
  document.getElementById("dm-summary-queue").textContent = queue;
  document.getElementById("dm-summary-active").textContent = active;
  document.getElementById("dm-summary-last").textContent = last
    ? `${last.supervisor} ??${last.result}`
    : "-";
}

function attachHandlers() {
  document.getElementById("dm-deliver-selected").addEventListener("click", (e) => {
    e.preventDefault();
    queueDelivery(Array.from(selectedSupervisors));
  });

  document.getElementById("dm-selection-toggle").addEventListener("click", (e) => {
    e.preventDefault();
    handleSelectionToggle();
  });

  // Global Filter Handlers
  document
    .getElementById("dm-filter-enabled")
    .addEventListener("change", () => {
      const enabled = document.getElementById("dm-filter-enabled").checked;
      setFilterControlsDisabled(!enabled, "dm-filter-body", "dm-filter-fields");
      saveGlobalFilters({ silent: true });
    });

  document
    .getElementById("dm-save-filters")
    .addEventListener("click", () => saveGlobalFilters());

  document
    .getElementById("dm-reset-filters")
    .addEventListener("click", () => populateGlobalFilterForm());

  document
    .querySelectorAll("input[name='dm-filter-mode']")
    .forEach((radio) => {
      radio.addEventListener("change", () => {
        if (!radio.checked) {
          return;
        }
        saveGlobalFilters({ silent: true });
      });
    });

  ["dm-rune-checkboxes", "dm-gem-checkboxes"].forEach((id) => {
    const container = document.getElementById(id);
    if (!container) {
      return;
    }
    container.addEventListener("change", (event) => {
      if (event.target && event.target.type === "checkbox") {
        saveGlobalFilters({ silent: true });
      }
    });
  });

  const customInput = document.getElementById("dm-custom-items");
  if (customInput) {
    customInput.addEventListener("input", () => {
      filterInputActive = true;
      if (filterSaveTimer) {
        clearTimeout(filterSaveTimer);
      }
      filterSaveTimer = setTimeout(() => {
        filterInputActive = false;
        saveGlobalFilters({ silent: true });
      }, 600);
    });
  }

  // Individual Filter Modal Handlers
  attachIndividualFilterHandlers();
}

function queueDelivery(supervisors) {
  if (!supervisors.length) {
    showToast("Select at least one supervisor.", "error");
    return;
  }

  const payload = buildBatchPayload(supervisors);
  if (!payload) {
    return;
  }

  localStorage.setItem(ROOM_STORAGE_KEY, payload.room);
  localStorage.setItem(PASSWORD_STORAGE_KEY, payload.password || "");

  postJson("/api/delivery/batch", payload)
    .then(() => fetchStatus())
    .catch((err) => {
      if (isIgnorableRequestError(err)) {
        return;
      }
      showToast(err.message || "Failed to queue delivery.", "error");
    });
}

function cancelDelivery(supervisor) {
  postJson("/api/delivery/cancel", { supervisor })
    .then(() => fetchStatus())
    .catch((err) => {
      if (isIgnorableRequestError(err)) {
        return;
      }
      showToast(err.message || "Cancel request failed.", "error");
    });
}

function deliverSingle(supervisor) {
  queueDelivery([supervisor]);
}

function buildBatchPayload(supervisors) {
  const room = document.getElementById("dm-room").value.trim();
  if (!room) {
    showToast("Room name is required.", "error");
    return null;
  }

  return {
    supervisors,
    room,
    password: document.getElementById("dm-password").value.trim(),
    delaySeconds: Number(document.getElementById("dm-delay").value) || 15,
  };
}

function buildFilterCheckboxes() {
  const runeContainer = document.getElementById("dm-rune-checkboxes");
  runeContainer.innerHTML = "";
  runeOptions.forEach(({ label, value }) => {
    runeContainer.appendChild(createFilterCheckboxItem(label, value, 'rune'));
  });

  const gemContainer = document.getElementById("dm-gem-checkboxes");
  gemContainer.innerHTML = "";
  gemOptions.forEach(({ label, value }) => {
    gemContainer.appendChild(createFilterCheckboxItem(label, value, 'gem'));
  });
}

function createFilterCheckboxItem(label, value, type) {
  const container = document.createElement("div");
  container.className = "filter-checkbox-item";
  
  const labelEl = document.createElement("label");
  const checkbox = document.createElement("input");
  checkbox.type = "checkbox";
  checkbox.value = value;
  checkbox.dataset.itemType = type;
  
  labelEl.appendChild(checkbox);
  labelEl.appendChild(document.createTextNode(label));
  container.appendChild(labelEl);
  
  // Quantity input only (no buttons)
  const qtyInput = document.createElement("input");
  qtyInput.type = "number";
  qtyInput.className = "quantity-value";
  qtyInput.value = "0";
  qtyInput.min = "0";
  qtyInput.placeholder = "All";
  qtyInput.title = "0 = All items (scroll or type to change)";
  container.appendChild(qtyInput);
  
  // Event handlers
  checkbox.addEventListener("change", () => {
    if (checkbox.checked) {
      qtyInput.classList.add("active");
      qtyInput.value = "0"; // Default to all
    } else {
      qtyInput.classList.remove("active");
    }
  });
  
  qtyInput.addEventListener("input", () => {
    const val = parseInt(qtyInput.value);
    if (isNaN(val) || val < 0) {
      qtyInput.value = "0";
    }
  });
  
  return container;
}

function startAndDeliver(supervisor, button) {
  const room = document.getElementById("dm-room").value.trim();
  if (!room) {
    showToast("Room name is required.", "error");
    return;
  }

  const password = document.getElementById("dm-password").value.trim();
  localStorage.setItem(ROOM_STORAGE_KEY, room);
  localStorage.setItem(PASSWORD_STORAGE_KEY, password);

  if (button) {
    button.disabled = true;
  }

  postJson(`/api/delivery/start-deliver?supervisor=${encodeURIComponent(supervisor)}`, {
    room,
    password,
  })
    .then(() => startSupervisor(supervisor))
    .then(() => waitForSupervisorOnline(supervisor))
    .then(() => fetchStatus())
    .catch((err) => showToast(err.message || err, "error"))
    .finally(() => {
      if (button) {
        button.disabled = false;
      }
    });
}


function startSupervisor(supervisor) {
  return fetch(`/start?characterName=${encodeURIComponent(supervisor)}`)
    .then((res) => {
      if (!res.ok) {
        throw new Error("Failed to start supervisor.");
      }
      return res.text();
    })
    .then(() => fetchStatus());
}

function waitForSupervisorOnline(supervisor, timeout = 60000, interval = 2000) {
  const startedAt = Date.now();

  function poll() {
    return loadDeliveryStatus().then(() => {
      const entry = deliveryState.supervisors.find((s) => s.name === supervisor);
      if (entry && entry.running) {
        return true;
      }
      if (Date.now() - startedAt > timeout) {
        throw new Error("Supervisor did not start in time.");
      }
      return new Promise((resolve) => setTimeout(resolve, interval)).then(poll);
    });
  }

  return poll();
}

function postJson(url, body) {
  return fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  }).then((res) => {
    if (!res.ok) {
      return res.text().then((text) => {
        throw new Error(text || "Request failed");
      });
    }
    return res.json().catch(() => ({}));
  });
}

function isIgnorableRequestError(err) {
  if (!err) {
    return false;
  }
  if (err.name === "AbortError") {
    return true;
  }
  const message = (err.message || "").toLowerCase();
  return message.includes("failed to fetch");
}

function formattedSince(timestamp) {
  if (!timestamp) {
    return "-";
  }
  const date = new Date(timestamp);
  return date.toLocaleTimeString();
}

// ========== Global Filter Functions ==========

function saveGlobalFilters(options = {}) {
  filterSyncBlocked = true;
  const payload = collectGlobalFilterForm();
  
  postJson(`/api/delivery/protection?supervisor=global`, payload)
    .then(() => {
      if (!options.silent) {
        showToast("Global filter saved successfully.", "success");
      }
      deliveryState.globalFilters = payload;
      setFilterControlsDisabled(!payload.enabled, "dm-filter-body", "dm-filter-fields");
    })
    .catch((err) => {
      if (isIgnorableRequestError(err)) {
        return;
      }
      if (!options.silent) {
        showToast(err.message, "error");
      } else {
        console.error(err);
      }
    })
    .finally(() => {
      filterSyncBlocked = false;
    });
}

function populateGlobalFilterForm() {
  if (filterSyncBlocked || filterInputActive) {
    return;
  }

  const filters = deliveryState.globalFilters;
  const enabled = filters.enabled ?? false;
  document.getElementById("dm-filter-enabled").checked = enabled;
  setFilterControlsDisabled(!enabled, "dm-filter-body", "dm-filter-fields");

  const mode = filters.deliverOnlySelected ? "exclusive" : "inclusive";
  document.querySelectorAll("input[name='dm-filter-mode']").forEach((radio) => {
    radio.checked = radio.value === mode;
  });
  setCheckboxValues("dm-rune-checkboxes", filters.selectedRunes || []);
  setCheckboxValues("dm-gem-checkboxes", filters.selectedGems || []);
  document.getElementById("dm-custom-items").value = (filters.customItems || []).join("\n");
}

function collectGlobalFilterForm() {
  const selectedMode =
    document.querySelector("input[name='dm-filter-mode']:checked")?.value || "inclusive";

  const customItems = document
    .getElementById("dm-custom-items")
    .value.split("\n")
    .map((line) => line.trim())
    .filter((line) => line.length > 0);

  return {
    enabled: document.getElementById("dm-filter-enabled").checked,
    deliverOnlySelected: selectedMode === "exclusive",
    selectedRunes: getCheckedValues("dm-rune-checkboxes"),
    selectedGems: getCheckedValues("dm-gem-checkboxes"),
    customItems,
  };
}

// ========== Individual Filter Functions ==========

function attachIndividualFilterHandlers() {
  const modalClose = document.getElementById("dm-modal-close");
  const modal = document.getElementById("dm-individual-filter-modal");
  
  if (modalClose) {
    modalClose.addEventListener("click", closeIndividualFilterModal);
  }
  
  if (modal) {
    modal.addEventListener("click", (e) => {
      if (e.target === modal) {
        closeIndividualFilterModal();
      }
    });
  }

  document
    .getElementById("dm-save-individual-filters")
    .addEventListener("click", () => saveIndividualFilters());

  document
    .getElementById("dm-reset-individual-filters")
    .addEventListener("click", () => populateIndividualFilterForm());

  document
    .getElementById("dm-clear-individual-filters")
    .addEventListener("click", () => clearIndividualFilters());

  document
    .querySelectorAll("input[name='dm-individual-filter-mode']")
    .forEach((radio) => {
      radio.addEventListener("change", () => {
        if (radio.checked) {
          // Auto-save on change
        }
      });
    });

  // Build individual filter checkboxes
  buildIndividualFilterCheckboxes();
}

function openIndividualFilterModal(supervisor) {
  deliveryState.currentModalSupervisor = supervisor;
  document.getElementById("dm-modal-supervisor-name").textContent = supervisor;
  populateIndividualFilterForm();
  document.getElementById("dm-individual-filter-modal").style.display = "flex";
}

function closeIndividualFilterModal() {
  document.getElementById("dm-individual-filter-modal").style.display = "none";
  deliveryState.currentModalSupervisor = null;
}

function saveIndividualFilters(options = {}) {
  const supervisor = deliveryState.currentModalSupervisor;
  if (!supervisor) {
    return;
  }

  const payload = collectIndividualFilterForm();
  payload.enabled = true; 
  postJson(`/api/delivery/protection?supervisor=${encodeURIComponent(supervisor)}`, payload)
    .then(() => {
      if (!options.silent) {
        showToast(`Individual filter saved for ${supervisor}.`, "success");
      }
      deliveryState.individualFilters[supervisor] = payload;
      
      closeIndividualFilterModal();
      renderSupervisors();
    })
    .catch((err) => {
      if (isIgnorableRequestError(err)) {
        return;
      }
      if (!options.silent) {
        showToast(err.message, "error");
      } else {
        console.error(err);
      }
    });
}

function populateIndividualFilterForm() {
  const supervisor = deliveryState.currentModalSupervisor;
  if (!supervisor) {
    return;
  }

  const filters = deliveryState.individualFilters[supervisor] || {
    enabled: false,
    deliverOnlySelected: true,
    selectedRunes: [],
    selectedGems: [],
    customItems: [],
  };

  const mode = filters.deliverOnlySelected ? "exclusive" : "inclusive";
  document.querySelectorAll("input[name='dm-individual-filter-mode']").forEach((radio) => {
    radio.checked = radio.value === mode;
  });
  setCheckboxValues("dm-individual-rune-checkboxes", filters.selectedRunes || []);
  setCheckboxValues("dm-individual-gem-checkboxes", filters.selectedGems || []);
  document.getElementById("dm-individual-custom-items").value = (filters.customItems || []).join("\n");
}

function collectIndividualFilterForm() {
  const selectedMode =
    document.querySelector("input[name='dm-individual-filter-mode']:checked")?.value || "inclusive";

  const customItems = document
    .getElementById("dm-individual-custom-items")
    .value.split("\n")
    .map((line) => line.trim())
    .filter((line) => line.length > 0);

  return {
    enabled: true, // always send enabled when saving individual filter
    deliverOnlySelected: selectedMode === "exclusive",
    selectedRunes: getCheckedValues("dm-individual-rune-checkboxes"),
    selectedGems: getCheckedValues("dm-individual-gem-checkboxes"),
    customItems,
  };
}

function buildIndividualFilterCheckboxes() {
  const runeContainer = document.getElementById("dm-individual-rune-checkboxes");
  if (runeContainer) {
    runeContainer.innerHTML = "";
    runeOptions.forEach(({ label, value }) => {
      runeContainer.appendChild(createFilterCheckboxItem(label, value, 'rune'));
    });
  }

  const gemContainer = document.getElementById("dm-individual-gem-checkboxes");
  if (gemContainer) {
    gemContainer.innerHTML = "";
    gemOptions.forEach(({ label, value }) => {
      gemContainer.appendChild(createFilterCheckboxItem(label, value, 'gem'));
    });
  }
}

function clearIndividualFilters() {
  const supervisor = deliveryState.currentModalSupervisor;
  if (!supervisor) {
    return;
  }

  if (!confirm(`Clear individual filter for ${supervisor}?`)) {
    return;
  }

  const payload = {
    enabled: false,
    deliverOnlySelected: true,
    selectedRunes: [],
    selectedGems: [],
    customItems: [],
  };
  
  postJson(`/api/delivery/protection?supervisor=${encodeURIComponent(supervisor)}`, payload)
    .then(() => {
      showToast(`Individual filter cleared for ${supervisor}.`, "info");
      delete deliveryState.individualFilters[supervisor];
      closeIndividualFilterModal();
      renderSupervisors();
    })
    .catch((err) => {
      if (!isIgnorableRequestError(err)) {
        showToast(err.message, "error");
      }
    });
}

// ========== Utility Functions ==========

function setFilterControlsDisabled(disabled, bodyId, fieldsetId) {
  const fieldset = document.getElementById(fieldsetId);
  if (fieldset) {
    fieldset.disabled = disabled;
  }

  const body = document.getElementById(bodyId);
  if (body) {
    body.hidden = disabled;
  }
}

function setCheckboxValues(containerId, selectedValues) {
  // selectedValues can be array of strings (legacy) or array of {name, quantity} objects
  const valueMap = new Map();
  
  selectedValues.forEach(item => {
    if (typeof item === 'string') {
      valueMap.set(item, 0); // Legacy: string means unlimited
    } else if (item && item.name) {
      valueMap.set(item.name, item.quantity || 0);
    }
  });
  
  document.querySelectorAll(`#${containerId} .filter-checkbox-item`).forEach(item => {
    const checkbox = item.querySelector('input[type="checkbox"]');
    const qtyInput = item.querySelector('.quantity-value');
    
    if (checkbox && valueMap.has(checkbox.value)) {
      checkbox.checked = true;
      if (qtyInput) {
        qtyInput.classList.add('active');
        qtyInput.value = valueMap.get(checkbox.value);
      }
    } else {
      checkbox.checked = false;
      if (qtyInput) {
        qtyInput.classList.remove('active');
      }
    }
  });
}

function getCheckedValues(containerId) {
  const items = [];
  document.querySelectorAll(`#${containerId} .filter-checkbox-item`).forEach(item => {
    const checkbox = item.querySelector('input[type="checkbox"]');
    if (checkbox && checkbox.checked) {
      const qtyInput = item.querySelector('.quantity-value');
      const quantity = qtyInput ? (parseInt(qtyInput.value) || 0) : 0;
      items.push({
        name: checkbox.value,
        quantity: quantity
      });
    }
  });
  return items;
}

function restoreSavedRoomInfo() {
  const savedRoom = localStorage.getItem(ROOM_STORAGE_KEY);
  const savedPassword = localStorage.getItem(PASSWORD_STORAGE_KEY);
  if (savedRoom) {
    document.getElementById("dm-room").value = savedRoom;
  }
  if (savedPassword) {
    document.getElementById("dm-password").value = savedPassword;
  }
}

function disableFiltersOnExit() {
  const supervisorsToDisable = [];

  if (deliveryState.globalFilters?.enabled) {
    supervisorsToDisable.push("global");
  }

  Object.entries(deliveryState.individualFilters).forEach(([supervisor, filters]) => {
    if (filters?.enabled) {
      supervisorsToDisable.push(supervisor);
    }
  });

  if (supervisorsToDisable.length === 0) {
    return;
  }

  supervisorsToDisable.forEach((supervisor) => {
    const current =
      supervisor === "global"
        ? deliveryState.globalFilters
        : deliveryState.individualFilters[supervisor] || {};

    const payload = {
      enabled: false,
      deliverOnlySelected: current.deliverOnlySelected ?? true,
      selectedRunes: current.selectedRunes || [],
      selectedGems: current.selectedGems || [],
      customItems: current.customItems || [],
    };

    const url = `/api/delivery/protection?supervisor=${encodeURIComponent(supervisor)}`;
    fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
      keepalive: true,
    }).catch(() => {});
  });
}
