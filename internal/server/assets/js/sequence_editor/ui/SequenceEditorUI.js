// @ts-check

/** @typedef {import("../constants.js").DifficultyKey} DifficultyKey */
/** @typedef {import("../constants.js").RunSectionKey} RunSectionKey */

/** @typedef {{difficulty:DifficultyKey, section:RunSectionKey}} AddRunRequest */
/** @typedef {{difficulty:DifficultyKey}} AddConfigRequest */
/** @typedef {{load:((event:Event) => void)|null, save:((event:Event) => void)|null, tabChange:((tabName:string) => void)|null, addRun:((payload:AddRunRequest) => void)|null, addConfig:((payload:AddConfigRequest) => void)|null}} SequenceEditorUIHandlers */
/** @typedef {{load:Event, save:Event, tabChange:string, addRun:AddRunRequest, addConfig:AddConfigRequest}} SequenceEditorUIPayloads */

/**
 * Lightweight facade around DOM controls for the sequence editor modal.
 */
export class SequenceEditorUI {
  /**
   * @param {Document} [root]
   */
  constructor(root = document) {
    this.root = root;
    this.elements = this.cacheElements();
    /** @type {SequenceEditorUIHandlers} */
    this.handlers = {
      load: null,
      save: null,
      tabChange: null,
      addRun: null,
      addConfig: null,
    };
    this.bindStaticEvents();
  }

  /**
   * @returns {{loadButton:HTMLButtonElement|null, saveButton:HTMLButtonElement|null, currentLabel:HTMLElement|null, dirtyIndicator:HTMLElement|null, messages:HTMLElement|null, panel:HTMLElement|null, tabs:HTMLElement[], closeButton:HTMLButtonElement|null, addRunButtons:HTMLButtonElement[], addConfigButtons:HTMLButtonElement[]}}
   */
  cacheElements() {
    const loadButton = /** @type {HTMLButtonElement|null} */ (this.root.getElementById("loadSequenceBtn"));
    const saveButton = /** @type {HTMLButtonElement|null} */ (this.root.getElementById("saveSequenceBtn"));
    const closeButton = /** @type {HTMLButtonElement|null} */ (this.root.getElementById("sequenceEditorCloseBtn"));
    const addRunButtons = /** @type {HTMLButtonElement[]} */ (Array.from(this.root.querySelectorAll(".add-run-btn")));
    const addConfigButtons = /** @type {HTMLButtonElement[]} */ (
      Array.from(this.root.querySelectorAll(".add-config-btn"))
    );

    return {
      loadButton,
      saveButton,
      currentLabel: this.root.getElementById("currentSequenceLabel"),
      dirtyIndicator: this.root.getElementById("dirtyIndicator"),
      messages: this.root.getElementById("editorMessages"),
      panel: this.root.getElementById("sequenceEditorPanel"),
      tabs: Array.from(this.root.querySelectorAll(".tab-bar .tab")),
      closeButton,
      addRunButtons,
      addConfigButtons,
    };
  }

  /** Wires up one-time DOM listeners. */
  bindStaticEvents() {
    this.bindLoadButton();
    this.bindSaveButton();
    this.bindTabButtons();
    this.bindAddButtons();
    this.bindCloseButton();
  }

  /** Connects the load button events. */
  bindLoadButton() {
    const { loadButton } = this.elements;
    if (!loadButton) {
      return;
    }
    loadButton.addEventListener("click", (event) => this.emit("load", event));
    loadButton.addEventListener("keydown", (event) => {
      if (event.key !== "Enter" && event.key !== " ") {
        return;
      }
      event.preventDefault();
      this.emit("load", event);
    });
  }

  /** Connects the save button events. */
  bindSaveButton() {
    const { saveButton } = this.elements;
    if (!saveButton) {
      return;
    }
    saveButton.addEventListener("click", (event) => this.emit("save", event));
  }

  /** Registers handlers for tab navigation. */
  bindTabButtons() {
    this.elements.tabs.forEach((tab) => {
      tab.addEventListener("click", () => {
        const tabName = tab.dataset.tab || "";
        if (tabName) {
          this.emit("tabChange", tabName);
        }
      });
    });
  }

  /** Hooks add-run and add-config buttons. */
  bindAddButtons() {
    this.elements.addRunButtons.forEach((button) => {
      button.addEventListener("click", () => {
        const difficulty = button.dataset.difficulty;
        const section = button.dataset.section;
        if (!difficulty || !section) {
          return;
        }
        this.emit("addRun", {
          difficulty: /** @type {DifficultyKey} */ (difficulty),
          section: /** @type {RunSectionKey} */ (section),
        });
      });
    });

    this.elements.addConfigButtons.forEach((button) => {
      button.addEventListener("click", () => {
        const difficulty = button.dataset.difficulty;
        if (!difficulty) {
          return;
        }
        this.emit("addConfig", {
          difficulty: /** @type {DifficultyKey} */ (difficulty),
        });
      });
    });
  }

  /** Handles close button logic (focus opener or redirect). */
  bindCloseButton() {
    const { closeButton } = this.elements;
    if (!closeButton) {
      return;
    }

    closeButton.addEventListener("click", () => {
      if (window.opener && !window.opener.closed) {
        try {
          window.opener.focus();
        } catch (error) {
          console.warn("Unable to focus opener window", error);
        }
        window.close();
        return;
      }
      window.location.href = "/supervisorSettings";
    });
  }

  /**
   * @param {(event:Event) => void} handler
   */
  onLoadRequested(handler) {
    this.handlers.load = handler;
  }

  /**
   * @param {(event:Event) => void} handler
   */
  onSaveRequested(handler) {
    this.handlers.save = handler;
  }

  /**
   * @param {(tabName:string) => void} handler
   */
  onTabChange(handler) {
    this.handlers.tabChange = handler;
  }

  /**
   * @param {(payload:{difficulty:string,section:string}) => void} handler
   */
  onAddRunRequested(handler) {
    this.handlers.addRun = handler;
  }

  /**
   * @param {(payload:{difficulty:string}) => void} handler
   */
  onAddConfigRequested(handler) {
    this.handlers.addConfig = handler;
  }

  /**
   * @template {keyof SequenceEditorUIHandlers} K
   * @param {K} handlerKey
   * @param {SequenceEditorUIPayloads[K]} payload
   */
  emit(handlerKey, payload) {
    const handler = this.handlers[handlerKey];
    if (typeof handler === "function") {
      /** @type {(arg: SequenceEditorUIPayloads[K]) => void} */ (handler)(payload);
    }
  }

  /**
   * @returns {{panel:HTMLElement|null, dirtyIndicator:HTMLElement|null, saveButton:HTMLButtonElement|null, tabs:HTMLElement[]}}
   */
  getRendererElements() {
    return {
      panel: this.elements.panel,
      dirtyIndicator: this.elements.dirtyIndicator,
      saveButton: this.elements.saveButton,
      tabs: this.elements.tabs,
    };
  }

  /** Reveals the editor panel container. */
  showPanel() {
    if (this.elements.panel) {
      this.elements.panel.hidden = false;
    }
  }

  /**
   * @param {string|null|undefined} name
   */
  setSequenceName(name) {
    if (!this.elements.currentLabel) {
      return;
    }
    this.elements.currentLabel.textContent = name ? `${name}.json` : "Unsaved sequence";
  }

  /**
   * @param {"success"|"error"|"info"|""} type
   * @param {string} message
   * @param {number} [durationMs]
   */
  showMessage(type, message, durationMs = 5000) {
    const container = this.elements.messages;
    if (!container) {
      return;
    }

    container.replaceChildren();

    if (!type || !message) {
      return;
    }

    const div = document.createElement("div");
    div.className = `alert ${type}`;
    div.textContent = message;
    container.appendChild(div);

    window.setTimeout(() => {
      if (container.contains(div)) {
        container.removeChild(div);
      }
    }, durationMs);
  }

  /**
   * @param {boolean} hasUnsavedChanges
   * @returns {boolean}
   */
  confirmDiscardChanges(hasUnsavedChanges) {
    if (!hasUnsavedChanges) {
      return true;
    }
    return window.confirm("Discard unsaved changes before opening a different sequence?");
  }

  /**
   * @param {string} [initialValue]
   * @returns {string|undefined}
   */
  promptSequenceName(initialValue = "") {
    const result = window.prompt("Save sequence as (letters, numbers, underscores, hyphens)", initialValue);
    return result == null ? undefined : result.trim();
  }
}
