import {
  initializeDefaultSequence,
  loadSequenceByName,
  openSequenceDialog,
  saveSequence,
} from "./sequencePersistence.js";

// @ts-check

/**
 * @typedef {import("./constants.js").DifficultyKey} DifficultyKey
 * @typedef {import("./constants.js").RunSectionKey} RunSectionKey
 * @typedef {import("./SequenceEditorState.js").SequenceEditorState} SequenceEditorState
 * @typedef {import("./SequenceApiClient.js").SequenceApiClient} SequenceApiClient
 * @typedef {import("./SequenceDataAdapter.js").SequenceDataAdapter} SequenceDataAdapter
 * @typedef {import("./SequenceEditorRenderer.js").SequenceEditorRenderer} SequenceEditorRenderer
 * @typedef {import("./ui/SequenceEditorUI.js").SequenceEditorUI} SequenceEditorUI
 * @typedef {{difficulty:DifficultyKey, section:RunSectionKey}} AddRunPayload
 */

/** Coordinates UI events, data fetching, and renderer interactions. */
export class SequenceEditorController {
  /**
   * @param {{state:SequenceEditorState, apiClient:SequenceApiClient, dataAdapter:SequenceDataAdapter, renderer:SequenceEditorRenderer, ui:SequenceEditorUI}} deps
   */
  constructor({ state, apiClient, dataAdapter, renderer, ui }) {
    if (!state || !apiClient || !dataAdapter || !renderer || !ui) {
      throw new Error("SequenceEditorController requires state, apiClient, dataAdapter, renderer, and ui instances.");
    }
    this.state = state;
    this.apiClient = apiClient;
    this.dataAdapter = dataAdapter;
    this.renderer = renderer;
    this.ui = ui;
    this.persistenceDeps = { state, apiClient, dataAdapter, renderer, ui };
    this.handleBeforeUnload = this.handleBeforeUnload.bind(this);
    this.initialSequenceName = this.readInitialSequenceName();
  }

  /** Initializes the UI bindings and primes the editor. */
  initialize() {
    this.renderer.setElements(this.ui.getRendererElements());
    this.bindUIEvents();
    this.renderer.updateDirtyIndicator();
    this.renderer.updateSaveState();
    window.addEventListener("beforeunload", this.handleBeforeUnload);
    void this.fetchRuns();
  }

  /** Attaches callbacks for the UI surface. */
  bindUIEvents() {
    this.ui.onLoadRequested((event) => this.handleLoadClick(event));
    this.ui.onSaveRequested((event) => this.handleSaveClick(event));
    this.ui.onTabChange((tabName) => this.renderer.switchTab(tabName));
    this.ui.onAddRunRequested((payload) => {
      const normalized = this.normalizeAddRunPayload(payload);
      if (normalized) {
        this.handleAddRun(normalized);
      }
    });
    this.ui.onAddConfigRequested((payload) => {
      const difficulty = this.normalizeDifficulty(payload?.difficulty);
      if (difficulty) {
        this.handleAddConfig({ difficulty });
      }
    });
  }

  /**
   * @returns {boolean}
   */
  hasUnsavedChanges() {
    return Boolean(this.state.dirty);
  }

  /**
   * @returns {boolean}
   */
  confirmDiscardChanges() {
    return this.ui.confirmDiscardChanges(this.hasUnsavedChanges());
  }

  /**
   * @param {string} [message]
   * @returns {boolean}
   */
  ensureSequenceLoaded(message) {
    if (this.state.data) {
      return true;
    }
    if (message) {
      this.ui.showMessage("info", message);
    }
    return false;
  }

  /**
   * @returns {string|undefined}
   */
  readInitialSequenceName() {
    try {
      const params = new URLSearchParams(window.location.search);
      const sequence = params.get("sequence");
      return sequence && sequence.trim() ? sequence.trim() : undefined;
    } catch (error) {
      console.error(error);
      return undefined;
    }
  }

  /** Loads run catalog data and initializes the editor sequence. */
  async fetchRuns() {
    try {
      const payload = await this.apiClient.fetchRuns();
      this.state.runs = Array.isArray(payload.runs) ? payload.runs.slice() : [];
      this.state.sequencerRuns = Array.isArray(payload.sequencerRuns) ? payload.sequencerRuns.slice() : [];
      this.state.questCatalog = this.normalizeQuestCatalog(payload.questCatalog);

      if (this.state.data) {
        this.renderer.renderEditor();
      } else {
        const initialName = this.initialSequenceName;
        this.initialSequenceName = undefined;
        if (initialName) {
          const loaded = await loadSequenceByName(this.persistenceDeps, initialName, { showMessages: true });
          if (!loaded) {
            await initializeDefaultSequence(this.persistenceDeps);
          }
        } else {
          await initializeDefaultSequence(this.persistenceDeps);
        }
      }
    } catch (error) {
      console.error(error);
      this.ui.showMessage("error", "Failed to load run catalog.");
    }
  }

  /**
   * @param {Event} [event]
   */
  async handleLoadClick(event) {
    event?.preventDefault();

    if (!this.confirmDiscardChanges()) {
      return;
    }
    await openSequenceDialog(this.persistenceDeps);
  }

  /**
   * @param {Event} [event]
   */
  async handleSaveClick(event) {
    event?.preventDefault();

    if (!this.ensureSequenceLoaded("Create or load a sequence before saving.")) {
      return;
    }

    let name = this.state.currentName;
    if (!name) {
      name = this.ui.promptSequenceName("");
      if (name == null) {
        return;
      }
      name = name.trim();
    }

    const validationError = this.validateSequenceName(name);
    if (validationError) {
      this.ui.showMessage("error", validationError);
      return;
    }

    name = name.trim();

    await saveSequence(this.persistenceDeps, name);
  }

  /**
   * @param {AddRunPayload} payload
   */
  handleAddRun(payload) {
    if (!this.ensureSequenceLoaded("Create or load a sequence first.")) {
      return;
    }
    const { difficulty, section } = payload || {};
    if (!difficulty || !section) {
      return;
    }
    this.renderer.addSequenceRow(difficulty, section);
  }

  /**
   * @param {{difficulty:DifficultyKey}} payload
   */
  handleAddConfig(payload) {
    if (!this.ensureSequenceLoaded("Create or load a sequence first.")) {
      return;
    }
    const { difficulty } = payload || {};
    if (!difficulty) {
      return;
    }
    this.renderer.addConfigBlock(difficulty);
  }

  /**
   * @param {BeforeUnloadEvent} event
   */
  handleBeforeUnload(event) {
    if (!this.hasUnsavedChanges()) {
      return;
    }
    event.preventDefault();
    event.returnValue = "";
  }

  /**
   * @param {string} value
   * @returns {string|null}
   */
  validateSequenceName(value) {
    const trimmed = typeof value === "string" ? value.trim() : "";
    if (!trimmed) {
      return "Sequence name is required.";
    }
    if (!/^[A-Za-z0-9_-]+$/.test(trimmed)) {
      return "Sequence name may only contain letters, numbers, underscores, or hyphens.";
    }
    return null;
  }

  /**
   * @param {Array<{run:string, act:number, isMandatory?:boolean}>|undefined} rawCatalog
   * @returns {Array<{run:string, act:number, isMandatory:boolean}>}
   */
  normalizeQuestCatalog(rawCatalog) {
    if (!Array.isArray(rawCatalog)) {
      return [];
    }

    return rawCatalog
      .filter((entry) => entry && typeof entry.run === "string")
      .map((entry) => ({
        run: entry.run,
        act: this.normalizeQuestAct(entry.act),
        isMandatory: Boolean(entry.isMandatory),
      }));
  }

  /**
   * @param {number|string} value
   * @returns {number}
   */
  normalizeQuestAct(value) {
    const actValue = Number(value);
    return Number.isFinite(actValue) ? Math.max(0, Math.trunc(actValue)) : 0;
  }

  /**
   * @param {(AddRunPayload|{difficulty?:string, section?:string}|null|undefined)} payload
   * @returns {AddRunPayload|null}
   */
  normalizeAddRunPayload(payload) {
    if (!payload) {
      return null;
    }
    const candidate = /** @type {{difficulty?:string, section?:string}} */ (payload);
    if (!this.isDifficultyKey(candidate.difficulty) || !this.isRunSection(candidate.section)) {
      return null;
    }
    return { difficulty: candidate.difficulty, section: candidate.section };
  }

  /**
   * @param {string|undefined} value
   * @returns {value is DifficultyKey}
   */
  isDifficultyKey(value) {
    return value === "normal" || value === "nightmare" || value === "hell";
  }

  /**
   * @param {string|undefined} value
   * @returns {value is RunSectionKey}
   */
  isRunSection(value) {
    return value === "beforeQuests" || value === "afterQuests";
  }

  /**
   * @param {string|undefined} value
   * @returns {DifficultyKey|null}
   */
  normalizeDifficulty(value) {
    return this.isDifficultyKey(value) ? value : null;
  }
}
