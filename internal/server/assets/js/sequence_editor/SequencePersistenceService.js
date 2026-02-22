// @ts-check

/**
 * @typedef {import("./SequenceEditorState.js").SequenceEditorState} SequenceEditorState
 * @typedef {import("./SequenceApiClient.js").SequenceApiClient} SequenceApiClient
 * @typedef {import("./SequenceDataAdapter.js").SequenceDataAdapter} SequenceDataAdapter
 * @typedef {import("./SequenceDataAdapter.js").SerializedSequencePayload} SerializedSequencePayload
 * @typedef {import("./SequenceEditorRenderer.js").SequenceEditorRenderer} SequenceEditorRenderer
 * @typedef {import("./ui/SequenceEditorUI.js").SequenceEditorUI} SequenceEditorUI
 * @typedef {{name?:string, settings:SerializedSequencePayload}} SettingsPayload
 */

/** High-level service wrapper for persistence helpers. */
export class SequencePersistenceService {
  /**
   * @param {{state:SequenceEditorState, apiClient:SequenceApiClient, dataAdapter:SequenceDataAdapter, renderer:SequenceEditorRenderer, ui:SequenceEditorUI}} deps
   */
  constructor({ state, apiClient, dataAdapter, renderer, ui }) {
    if (!state || !apiClient || !dataAdapter || !renderer || !ui) {
      throw new Error("SequencePersistenceService requires state, apiClient, dataAdapter, renderer, and ui instances.");
    }

    /** @type {SequenceEditorState} */
    this.state = state;
    /** @type {SequenceApiClient} */
    this.apiClient = apiClient;
    /** @type {SequenceDataAdapter} */
    this.dataAdapter = dataAdapter;
    /** @type {SequenceEditorRenderer} */
    this.renderer = renderer;
    /** @type {SequenceEditorUI} */
    this.ui = ui;
  }

  /**
   * @returns {Promise<boolean>}
   */
  async initializeDefaultSequence() {
    if (this.state.data) {
      this.renderer.renderEditor();
      return true;
    }

    try {
      const payload = await this.apiClient.fetchDefaultSequence();
      this.dataAdapter.hydrateSequenceData(payload);
      this.resetEditingState();
      this.setSequenceName(undefined);
      this.renderer.renderEditor();
      return true;
    } catch (error) {
      console.error(error);
      this.state.data = this.dataAdapter.createEmptySequenceData();
      this.resetEditingState();
      this.setSequenceName(undefined);
      this.renderer.renderEditor();
      this.ui.showMessage("info", "Default sequence template unavailable. Started with a blank sequence.");
      return false;
    }
  }

  /**
   * @returns {Promise<boolean>}
   */
  async openSequenceDialog() {
    try {
      const payload = await this.apiClient.openSequence();
      if (payload?.cancelled) {
        return false;
      }
      this.assertSettingsPayload(payload);
      this.applyLoadedSequence(payload.name, payload.settings);
      const displayName = payload.name ? `"${payload.name}.json"` : "from disk";
      this.ui.showMessage("success", `Loaded sequence ${displayName}.`);
      return true;
    } catch (error) {
      console.error(error);
      const message = error instanceof Error && error.message ? error.message : "Failed to open sequence.";
      this.ui.showMessage("error", message);
      return false;
    }
  }

  /**
   * @param {string} name
   * @param {{showMessages?:boolean}} [options]
   * @returns {Promise<boolean>}
   */
  async loadSequenceByName(name, { showMessages = true } = {}) {
    if (!name) {
      return false;
    }

    try {
      const payload = await this.apiClient.fetchSequenceByName(name);
      this.assertSettingsPayload(payload);
      const resolvedName = payload.name || name;
      this.applyLoadedSequence(resolvedName, payload.settings);
      if (showMessages) {
        this.ui.showMessage("success", `Loaded sequence "${resolvedName.trim()}.json".`);
      }
      return true;
    } catch (error) {
      console.error(error);
      if (showMessages) {
        const message = error instanceof Error && error.message ? error.message : "Failed to load sequence.";
        this.ui.showMessage("error", message);
      }
      return false;
    }
  }

  /**
   * @param {string} name
   * @returns {Promise<boolean>}
   */
  async saveSequence(name) {
    try {
      this.dataAdapter.normalizeClientData();
      const payload = this.dataAdapter.buildSavePayload();

      const result = await this.apiClient.saveSequence({
        name,
        settings: payload,
      });

      this.state.dirty = false;
      this.setSequenceName(name);
      this.renderer.renderEditor();
      this.notifySequenceSaved(name);

      const savedPath = result && typeof result.path === "string" ? result.path : undefined;
      const message = savedPath ? `Saved sequence to ${savedPath}.` : `Saved sequence "${name}.json".`;
      this.ui.showMessage("success", message);
      return true;
    } catch (error) {
      console.error(error);
      const message = error instanceof Error && error.message ? error.message : "Failed to save sequence.";
      this.ui.showMessage("error", message);
      return false;
    }
  }

  /**
   * @param {string|undefined} name
   * @param {SerializedSequencePayload} settings
   */
  applyLoadedSequence(name, settings) {
    this.dataAdapter.hydrateSequenceData(settings);
    this.resetEditingState();
    this.setSequenceName(name);
    this.renderer.renderEditor();
  }

  /** Resets dirty/editing tracking and updates UI. */
  resetEditingState() {
    this.state.clearEditingState();
    this.state.dirty = false;
    this.renderer.updateDirtyIndicator();
    this.renderer.updateSaveState();
  }

  /**
   * @param {string|undefined} name
   */
  setSequenceName(name) {
    this.state.currentName = name && name.trim() ? name.trim() : undefined;
    this.ui.setSequenceName(this.state.currentName);
    this.renderer.updateSaveState();
  }

  /**
   * @param {string|undefined} name
   */
  notifySequenceSaved(name) {
    if (!name) {
      return;
    }

    try {
      if (!window.localStorage) {
        return;
      }
      window.localStorage.setItem("koolo:lastSequenceName", name);
      window.localStorage.setItem("koolo:sequenceRefreshRequired", "1");
    } catch (error) {
      console.warn("Unable to store sequence refresh state", error);
    }
  }

  /**
   * @param {Partial<SettingsPayload>|null|undefined} payload
   * @returns {asserts payload is SettingsPayload}
   */
  assertSettingsPayload(payload) {
    if (!payload || !payload.settings) {
      throw new Error("Sequence settings missing from response.");
    }
  }
}
