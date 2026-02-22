// @ts-check

const REQUIRED_DEPS = ["state", "apiClient", "dataAdapter", "renderer", "ui"];

/**
 * @typedef {import("./SequenceEditorState.js").SequenceEditorState} SequenceEditorState
 * @typedef {import("./SequenceApiClient.js").SequenceApiClient} SequenceApiClient
 * @typedef {import("./SequenceDataAdapter.js").SequenceDataAdapter} SequenceDataAdapter
 * @typedef {import("./SequenceDataAdapter.js").SerializedSequencePayload} SerializedSequencePayload
 * @typedef {import("./SequenceEditorRenderer.js").SequenceEditorRenderer} SequenceEditorRenderer
 * @typedef {import("./ui/SequenceEditorUI.js").SequenceEditorUI} SequenceEditorUI
 * @typedef {{state:SequenceEditorState, apiClient:SequenceApiClient, dataAdapter:SequenceDataAdapter, renderer:SequenceEditorRenderer, ui:SequenceEditorUI}} PersistenceDeps
 * @typedef {{name?:string, settings:SerializedSequencePayload}} SettingsPayload
 */
/**
 * @param {Partial<PersistenceDeps>} deps
 * @returns {PersistenceDeps}
 */
function resolveDeps(deps) {
  if (!deps || typeof deps !== "object") {
    throw new Error("Sequence persistence helpers require dependency references.");
  }

  REQUIRED_DEPS.forEach((key) => {
    if (!deps[key]) {
      throw new Error(`Sequence persistence helpers missing "${key}" dependency.`);
    }
  });

  return /** @type {PersistenceDeps} */ (deps);
}

/**
 * @param {Partial<PersistenceDeps>} deps
 * @returns {Promise<boolean>}
 */
export async function initializeDefaultSequence(deps) {
  const { state, apiClient, dataAdapter, renderer, ui } = resolveDeps(deps);

  if (state.data) {
    renderer.renderEditor();
    return true;
  }

  try {
    const payload = await apiClient.fetchDefaultSequence();
    dataAdapter.hydrateSequenceData(payload);
    resetEditingState(deps);
    setSequenceName(deps, undefined);
    renderer.renderEditor();
    return true;
  } catch (error) {
    console.error(error);
    state.data = dataAdapter.createEmptySequenceData();
    resetEditingState(deps);
    setSequenceName(deps, undefined);
    renderer.renderEditor();
    ui.showMessage("info", "Default sequence template unavailable. Started with a blank sequence.");
    return false;
  }
}

/**
 * @param {Partial<PersistenceDeps>} deps
 * @returns {Promise<boolean>}
 */
export async function openSequenceDialog(deps) {
  const { apiClient, ui } = resolveDeps(deps);
  try {
    const payload = await apiClient.openSequence();
    if (payload?.cancelled) {
      return false;
    }

    assertSettingsPayload(payload);
    applyLoadedSequence(deps, payload.name, payload.settings);
    const displayName = payload.name ? `"${payload.name.trim()}.json"` : "from disk";
    ui.showMessage("success", `Loaded sequence ${displayName}.`);
    return true;
  } catch (error) {
    console.error(error);
    const message = error instanceof Error && error.message ? error.message : "Failed to open sequence.";
    ui.showMessage("error", message);
    return false;
  }
}

/**
 * @param {Partial<PersistenceDeps>} deps
 * @param {string} name
 * @param {{showMessages?:boolean}} [options]
 * @returns {Promise<boolean>}
 */
export async function loadSequenceByName(deps, name, { showMessages = true } = {}) {
  if (!name) {
    return false;
  }

  const { apiClient, ui } = resolveDeps(deps);

  try {
    const payload = await apiClient.fetchSequenceByName(name);
    assertSettingsPayload(payload);
    const resolvedName = payload.name || name;
    applyLoadedSequence(deps, resolvedName, payload.settings);
    if (showMessages) {
      ui.showMessage("success", `Loaded sequence "${resolvedName.trim()}.json".`);
    }
    return true;
  } catch (error) {
    console.error(error);
    if (showMessages) {
      const message = error instanceof Error && error.message ? error.message : "Failed to load sequence.";
      ui.showMessage("error", message);
    }
    return false;
  }
}

/**
 * @param {Partial<PersistenceDeps>} deps
 * @param {string} name
 * @returns {Promise<boolean>}
 */
export async function saveSequence(deps, name) {
  const { apiClient, dataAdapter, renderer, state, ui } = resolveDeps(deps);

  try {
    dataAdapter.normalizeClientData();
    const payload = dataAdapter.buildSavePayload();
    const result = await apiClient.saveSequence({
      name,
      settings: payload,
    });

    state.dirty = false;
    setSequenceName(deps, name);
    renderer.renderEditor();
    notifySequenceSaved(name);

    const savedPath = result && typeof result.path === "string" ? result.path : undefined;
    const message = savedPath ? `Saved sequence to ${savedPath}.` : `Saved sequence "${name}.json".`;
    ui.showMessage("success", message);
    return true;
  } catch (error) {
    console.error(error);
    const message = error instanceof Error && error.message ? error.message : "Failed to save sequence.";
    ui.showMessage("error", message);
    return false;
  }
}

/**
 * @param {Partial<PersistenceDeps>} deps
 * @param {string|undefined} name
 * @param {SerializedSequencePayload} settings
 */
function applyLoadedSequence(deps, name, settings) {
  const { dataAdapter, renderer } = resolveDeps(deps);
  dataAdapter.hydrateSequenceData(settings);
  resetEditingState(deps);
  setSequenceName(deps, name);
  renderer.renderEditor();
}

/**
 * @param {Partial<PersistenceDeps>} deps
 */
function resetEditingState(deps) {
  const { state, renderer } = resolveDeps(deps);
  state.clearEditingState();
  state.dirty = false;
  renderer.updateDirtyIndicator();
  renderer.updateSaveState();
}

/**
 * @param {Partial<PersistenceDeps>} deps
 * @param {string|undefined} name
 */
function setSequenceName(deps, name) {
  const { state, ui, renderer } = resolveDeps(deps);
  state.currentName = name && name.trim() ? name.trim() : undefined;
  ui.setSequenceName(state.currentName);
  renderer.updateSaveState();
}

/**
 * @param {string|undefined} name
 */
function notifySequenceSaved(name) {
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
function assertSettingsPayload(payload) {
  if (!payload || !payload.settings) {
    throw new Error("Sequence settings missing from response.");
  }
}
