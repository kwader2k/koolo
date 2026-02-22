// @ts-check

/** @typedef {import("./SequenceDataAdapter.js").DifficultySettings} DifficultySettings */
/** @typedef {import("./SequenceDataAdapter.js").DifficultyKey} DifficultyKey */
/** @typedef {import("./SequenceDataAdapter.js").SequenceRunEntry} SequenceRunEntry */
/** @typedef {import("./SequenceDataAdapter.js").SequenceConfigEntry} SequenceConfigEntry */
/** @typedef {import("./SequenceDataAdapter.js").SequenceConditionEntry} SequenceConditionEntry */
/** @typedef {SequenceRunEntry|SequenceConfigEntry|SequenceConditionEntry|null|undefined} UIDEntry */

/** Maintains shared reactive state for the sequence editor UI. */
export class SequenceEditorState {
  constructor() {
    /** @type {string[]} */
    this.runs = [];
    /** @type {string[]} */
    this.sequencerRuns = [];
    /** @type {Array<{run:string, act:number, isMandatory:boolean}>} */
    this.questCatalog = [];
    /** @type {Record<DifficultyKey, DifficultySettings>|undefined} */
    this.data = undefined;
    /** @type {string|undefined} */
    this.currentName = undefined;
    /** @type {boolean} */
    this.dirty = false;
    /** @type {Set<string>} */
    this.editingKeys = new Set();
    /** @type {number} */
    this.entryUIDCounter = 0;
  }

  /**
   * @param {string} type
   * @param {...string} parts
   * @returns {string}
   */
  buildEditKey(type, ...parts) {
    return [type, ...parts].join("|");
  }

  /**
   * @param {string} type
   * @param {...string} parts
   * @returns {boolean}
   */
  isEditingKey(type, ...parts) {
    return this.editingKeys.has(this.buildEditKey(type, ...parts));
  }

  /**
   * @param {string} type
   * @param {boolean} enabled
   * @param {...string} parts
   */
  setEditingKey(type, enabled, ...parts) {
    const key = this.buildEditKey(type, ...parts);
    if (enabled) {
      this.editingKeys.add(key);
    } else {
      this.editingKeys.delete(key);
    }
  }

  /** Resets editing flags for all sections. */
  clearEditingState() {
    this.editingKeys.clear();
  }

  /**
   * Ensures a hidden UID marker exists on the supplied entry object.
   * @param {UIDEntry} entry
   * @returns {string}
   */
  ensureEntryUID(entry) {
    if (!entry || typeof entry !== "object") {
      return "";
    }

    const typedEntry = /** @type {{__uid?:string}} */ (entry);

    if (!Object.prototype.hasOwnProperty.call(typedEntry, "__uid")) {
      const uid = `uid_${Date.now()}_${this.entryUIDCounter++}`;
      Object.defineProperty(typedEntry, "__uid", {
        value: uid,
        enumerable: false,
        configurable: true,
        writable: false,
      });
    }

    return /** @type {string} */ (typedEntry.__uid);
  }
}
