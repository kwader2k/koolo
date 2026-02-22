// @ts-check

/** @typedef {import("./SequenceDataAdapter.js").SerializedSequencePayload} SerializedSequencePayload */

/**
 * @typedef {{runs:string[], sequencerRuns:string[], questCatalog:Array<{run:string, act:number, isMandatory?:boolean}>}} RunCatalogResponse
 */

/**
 * @typedef {{name?:string, settings:SerializedSequencePayload}} SequenceFilePayload
 * @typedef {{cancelled?:boolean} & Partial<SequenceFilePayload>} SequenceOpenResult
 * @typedef {{name:string, settings:SerializedSequencePayload}} SequenceSavePayload
 */

/** Handles HTTP requests for the sequence editor backend endpoints. */
export class SequenceApiClient {
  /**
   * Retrieves the available run catalog.
   * @returns {Promise<RunCatalogResponse>}
   */
  async fetchRuns() {
    return this.request("/api/sequence-editor/runs");
  }

  /**
   * Opens the native OS file picker to select a sequence file.
   * @returns {Promise<SequenceOpenResult>}
   */
  async openSequence() {
    return this.request("/api/sequence-editor/open", { method: "POST" });
  }

  /**
   * Downloads a stored sequence template by name.
   * @param {string} name Persisted sequence file identifier.
   * @returns {Promise<SequenceFilePayload>}
   */
  async fetchSequenceByName(name) {
    if (!name) {
      throw new Error("Sequence name is required");
    }
    const params = new URLSearchParams({ name });
    return this.request(`/api/sequence-editor/file?${params.toString()}`);
  }

  /**
   * Saves the active sequence payload.
   * @param {SequenceSavePayload} payload Serialized data from the editor.
   * @returns {Promise<{ok:boolean, path?:string}>}
   */
  async saveSequence(payload) {
    return this.request("/api/sequence-editor/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
  }

  /**
   * Fetches the fallback default sequence definition bundled with the UI.
   * @returns {Promise<SerializedSequencePayload>}
   */
  async fetchDefaultSequence() {
    return this.request("/assets/data/default_sequence.json");
  }

  /**
   * Executes a fetch call and throws on failures.
   * @template T
   * @param {string} url Request URL.
   * @param {RequestInit} [options] Fetch options.
   * @returns {Promise<T>}
   */
  async request(url, options) {
    const response = await fetch(url, options);
    if (!response.ok) {
      const message = await response.text();
      throw new Error(message || "Request failed");
    }
    return /** @type {Promise<T>} */ (response.json());
  }
}
