// @ts-check

import { CONDITION_SECTIONS } from "../constants.js";
import { buildField, parseOptionalNumber } from "../utils.js";

/**
 * @typedef {import("../constants.js").DifficultyKey} DifficultyKey
 * @typedef {import("../SequenceEditorState.js").SequenceEditorState} SequenceEditorState
 * @typedef {import("../SequenceDataAdapter.js").SequenceDataAdapter} SequenceDataAdapter
 * @typedef {import("../SequenceDataAdapter.js").SequenceConditionEntry} SequenceConditionEntry
 * @typedef {import("../dom/DomTargetResolver.js").DomTargetResolver} DomTargetResolver
 */

/** Renders the difficulty condition rows and inline editors. */
export class ConditionSectionRenderer {
  /**
   * @param {{state:SequenceEditorState, dataAdapter:SequenceDataAdapter, markDirty:() => void, isConditionEditing:(difficulty:DifficultyKey,key:string)=>boolean, setConditionEditing:(difficulty:DifficultyKey,key:string,enabled:boolean)=>void, domTargets:DomTargetResolver}} deps
   */
  constructor({ state, dataAdapter, markDirty, isConditionEditing, setConditionEditing, domTargets }) {
    this.state = state;
    this.dataAdapter = dataAdapter;
    this.markDirty = markDirty;
    this.isConditionEditing = isConditionEditing;
    this.setConditionEditing = setConditionEditing;
    this.domTargets = domTargets;
    this._isSyncing = false; // Guard flag to prevent infinite re-render loops

    if (!this.domTargets) {
      throw new Error("ConditionSectionRenderer requires a domTargets resolver instance");
    }
  }

  /**
   * @param {DifficultyKey} difficulty
   */
  render(difficulty) {
    const container = this.domTargets.getConditionContainer(difficulty);
    if (!container || !this.state.data) {
      return;
    }

    const difficultyData = this.state.data[difficulty];
    if (!difficultyData) {
      return;
    }

    const sections = CONDITION_SECTIONS[difficulty] || [];
    container.innerHTML = "";

    if (!sections.length) {
      const empty = document.createElement("div");
      empty.className = "muted";
      empty.textContent = "No difficulty conditions available.";
      container.appendChild(empty);
      return;
    }

    sections.forEach(({ key, title, autoSyncInfo }) => {
      const condition = difficultyData[key];
      const enabled = Boolean(condition);
      const editing = enabled && this.isConditionEditing(difficulty, key);

      const row = document.createElement("div");
      row.className = "sequence-row condition-row";
      row.classList.toggle("editing", editing);
      row.classList.toggle("disabled", !enabled);

      const rowMain = document.createElement("div");
      rowMain.className = "row-main";

      const titleContainer = document.createElement("div");
      titleContainer.className = "row-title-container";

      const titleEl = document.createElement("div");
      titleEl.className = "row-title";
      titleEl.textContent = title;
      titleContainer.appendChild(titleEl);

      if (autoSyncInfo) {
        const infoEl = document.createElement("div");
        infoEl.className = "row-subtitle muted";
        infoEl.textContent = autoSyncInfo;
        titleContainer.appendChild(infoEl);
      }

      rowMain.appendChild(titleContainer);

      const summary = document.createElement("div");
      summary.className = "row-summary";
      this.updateSummary(summary, difficulty, key);
      rowMain.appendChild(summary);

      const actions = document.createElement("div");
      actions.className = "row-actions";

      const toggleLabel = document.createElement("label");
      toggleLabel.className = "condition-toggle";
      const toggle = document.createElement("input");
      toggle.type = "checkbox";
      toggle.checked = enabled;
      toggle.addEventListener("change", (event) => {
        const target = /** @type {HTMLInputElement} */ (event.target);
        if (target.checked) {
          if (!difficultyData[key]) {
            difficultyData[key] = this.dataAdapter.createEmptyConditions();
          }
          this.setConditionEditing(difficulty, key, true);
        } else {
          difficultyData[key] = undefined;
          this.setConditionEditing(difficulty, key, false);
        }
        this.syncDifficultyConditions(difficulty, key);
        this.markDirty();
        this.render(difficulty);
      });
      toggleLabel.appendChild(toggle);
      const toggleText = document.createElement("span");
      toggleText.textContent = "Enabled";
      toggleLabel.appendChild(toggleText);
      actions.appendChild(toggleLabel);

      const editButton = document.createElement("button");
      editButton.type = "button";
      editButton.className = "btn small outline";
      editButton.textContent = editing ? "Done" : "Edit";
      editButton.disabled = !difficultyData[key];
      editButton.addEventListener("click", () => {
        if (!difficultyData[key]) {
          return;
        }
        this.setConditionEditing(difficulty, key, !editing);
        this.render(difficulty);
      });
      actions.appendChild(editButton);

      rowMain.appendChild(actions);
      row.appendChild(rowMain);

      if (enabled && editing) {
        row.appendChild(this.buildEditor(difficulty, key, summary));
      }

      container.appendChild(row);
    });
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {string} key
   * @param {HTMLElement} summary
   * @returns {HTMLDivElement}
   */
  buildEditor(difficulty, key, summary) {
    const difficultyData = this.state.data?.[difficulty];
    const condition = difficultyData?.[key];
    if (!condition) {
      return document.createElement("div");
    }
    const editor = document.createElement("div");
    editor.className = "condition-editor";

    const grid = document.createElement("div");
    grid.className = "condition-editor-grid";

    const numericFields = [
      ["level", "Required Level"],
      ["fireRes", "Fire Res"],
      ["coldRes", "Cold Res"],
      ["lightRes", "Lightning Res"],
      ["poisonRes", "Poison Res"],
    ];

    numericFields.forEach(([field, label]) => {
      const input = document.createElement("input");
      input.type = "number";
      input.placeholder = "Value";
      input.value = condition[field] != null ? String(condition[field]) : "";
      input.addEventListener("input", (event) => {
        const target = /** @type {HTMLInputElement} */ (event.target);
        condition[field] = parseOptionalNumber(target.value);
        this.updateSummary(summary, difficulty, key);
        this.syncDifficultyConditions(difficulty, key);
        this.markDirty();
      });
      grid.appendChild(buildField(label, input, "condition-editor-field"));
    });

    editor.appendChild(grid);

    const flags = document.createElement("div");
    flags.className = "checkbox-grid condition-editor-flags";

    const flagDefinitions = [
      ["aboveLowGold", "Above low gold"],
      ["aboveGoldThreshold", "Above gold threshold"],
    ];

    flagDefinitions.forEach(([field, label]) => {
      const wrapper = document.createElement("label");
      wrapper.className = "checkbox-field";
      const checkbox = document.createElement("input");
      checkbox.type = "checkbox";
      checkbox.checked = Boolean(condition[field]);
      checkbox.addEventListener("change", (event) => {
        const target = /** @type {HTMLInputElement} */ (event.target);
        condition[field] = target.checked;
        this.updateSummary(summary, difficulty, key);
        this.syncDifficultyConditions(difficulty, key);
        this.markDirty();
      });
      const span = document.createElement("span");
      span.textContent = label;
      wrapper.appendChild(checkbox);
      wrapper.appendChild(span);
      flags.appendChild(wrapper);
    });

    editor.appendChild(flags);
    return editor;
  }

  /**
   * Synchronizes difficulty conditions between difficulties to prevent mismatches.
   * Bidirectional sync:
   * - Normal's nextDifficultyConditions ↔ Nightmare's stayDifficultyConditions
   * - Nightmare's nextDifficultyConditions ↔ Hell's stayDifficultyConditions
   * @param {DifficultyKey} difficulty
   * @param {string} key
   */
  syncDifficultyConditions(difficulty, key) {
    if (!this.state.data || this._isSyncing) {
      return;
    }

    // Prevent cascading re-renders and infinite loops
    this._isSyncing = true;

    try {
      // Normal's next conditions ↔ Nightmare's stay conditions
      if (difficulty === "normal" && key === "nextDifficultyConditions") {
        const normalNext = this.state.data.normal?.nextDifficultyConditions;
        if (this.state.data.nightmare) {
          // Propagate both set and unset (undefined) values
          this.state.data.nightmare.stayDifficultyConditions = normalNext 
            ? this.copyCondition(normalNext) 
            : undefined;
          this.render("nightmare");
        }
      }
      
      if (difficulty === "nightmare" && key === "stayDifficultyConditions") {
        const nightmareStay = this.state.data.nightmare?.stayDifficultyConditions;
        if (this.state.data.normal) {
          // Propagate both set and unset (undefined) values
          this.state.data.normal.nextDifficultyConditions = nightmareStay 
            ? this.copyCondition(nightmareStay) 
            : undefined;
          this.render("normal");
        }
      }

      // Nightmare's next conditions ↔ Hell's stay conditions
      if (difficulty === "nightmare" && key === "nextDifficultyConditions") {
        const nightmareNext = this.state.data.nightmare?.nextDifficultyConditions;
        if (this.state.data.hell) {
          // Propagate both set and unset (undefined) values
          this.state.data.hell.stayDifficultyConditions = nightmareNext 
            ? this.copyCondition(nightmareNext) 
            : undefined;
          this.render("hell");
        }
      }
      
      if (difficulty === "hell" && key === "stayDifficultyConditions") {
        const hellStay = this.state.data.hell?.stayDifficultyConditions;
        if (this.state.data.nightmare) {
          // Propagate both set and unset (undefined) values
          this.state.data.nightmare.nextDifficultyConditions = hellStay 
            ? this.copyCondition(hellStay) 
            : undefined;
          this.render("nightmare");
        }
      }
    } finally {
      // Always reset the sync flag, even if an error occurs
      this._isSyncing = false;
    }
  }

  /**
   * Creates a deep copy of a condition entry
   * @param {SequenceConditionEntry} condition
   * @returns {SequenceConditionEntry}
   */
  copyCondition(condition) {
    return {
      level: condition.level,
      fireRes: condition.fireRes,
      coldRes: condition.coldRes,
      lightRes: condition.lightRes,
      poisonRes: condition.poisonRes,
      aboveLowGold: condition.aboveLowGold,
      aboveGoldThreshold: condition.aboveGoldThreshold,
    };
  }

  /**
   * @param {HTMLElement} summaryElement
   * @param {DifficultyKey} difficulty
   * @param {string} key
   */
  updateSummary(summaryElement, difficulty, key) {
    const difficultyData = this.state.data?.[difficulty];
    const condition = difficultyData?.[key];
    if (!condition) {
      summaryElement.textContent = "Disabled";
      summaryElement.classList.add("empty");
      return;
    }

    const summaryText = this.buildConditionSummary(condition);
    summaryElement.textContent = summaryText;
    summaryElement.classList.toggle("empty", summaryText === "No requirements");
  }

  /**
   * @param {SequenceConditionEntry|null} condition
   * @returns {string}
   */
  buildConditionSummary(condition) {
    if (!condition) {
      return "Disabled";
    }

    const parts = [];
    if (condition.level != null) {
      parts.push(`Level ≥ ${condition.level}`);
    }

    [
      ["fireRes", "Fire"],
      ["coldRes", "Cold"],
      ["lightRes", "Lightning"],
      ["poisonRes", "Poison"],
    ].forEach(([field, label]) => {
      const value = condition[field];
      if (value != null) {
        parts.push(`${label} ${value}+`);
      }
    });

    if (condition.aboveLowGold) {
      parts.push("Above low gold");
    }
    if (condition.aboveGoldThreshold) {
      parts.push("Above gold threshold");
    }

    return parts.length ? parts.join(" • ") : "No requirements";
  }
}
