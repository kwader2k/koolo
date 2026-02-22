// @ts-check

import { buildField, createDragHandle, parseOptionalNumber } from "../utils.js";

/**
 * @typedef {import("../constants.js").DifficultyKey} DifficultyKey
 * @typedef {import("../SequenceEditorState.js").SequenceEditorState} SequenceEditorState
 * @typedef {import("../SequenceDataAdapter.js").SequenceDataAdapter} SequenceDataAdapter
 * @typedef {import("../SequenceDataAdapter.js").SequenceConfigEntry} SequenceConfigEntry
 * @typedef {import("../renderers/DragReorderManager.js").DragReorderManager} DragReorderManager
 * @typedef {import("../dom/DomTargetResolver.js").DomTargetResolver} DomTargetResolver
 */

/** Manages rendering for the config override section. */
export class ConfigSectionRenderer {
  constructor({
    state,
    dataAdapter,
    ensureConfigList,
    markDirty,
    dragManager,
    isConfigEditing,
    setConfigEditing,
    domTargets,
  }) {
    this.state = state;
    this.dataAdapter = dataAdapter;
    this.ensureConfigList = ensureConfigList;
    this.markDirty = markDirty;
    this.dragManager = dragManager;
    this.isConfigEditing = isConfigEditing;
    this.setConfigEditing = setConfigEditing;
    this.domTargets = domTargets;

    if (!this.domTargets) {
      throw new Error("ConfigSectionRenderer requires a domTargets resolver instance");
    }
  }

  /**
   * @param {DifficultyKey} difficulty
   */
  render(difficulty) {
    const container = this.domTargets.getConfigContainer(difficulty);
    if (!container || !this.state.data) {
      return;
    }

    const list = this.ensureConfigList(this.state.data[difficulty]);
    container.innerHTML = "";

    if (list.length === 0) {
      const empty = document.createElement("div");
      empty.className = "muted";
      empty.textContent = "No config overrides defined.";
      container.appendChild(empty);
      this.dragManager.teardown(container);
      return;
    }

    list.forEach((config) => {
      this.state.ensureEntryUID(config);
      const editing = this.isConfigEditing(difficulty, config);

      const row = document.createElement("div");
      row.className = "sequence-row config-row";
      row.classList.toggle("editing", editing);
      row.dataset.uid = String(config.__uid);

      const rowMain = document.createElement("div");
      rowMain.className = "row-main";

      const handle = createDragHandle("Reorder config adjustment");
      rowMain.appendChild(handle);

      const title = document.createElement("div");
      title.className = "row-title";
      title.textContent = "Config Override";
      const content = document.createElement("div");
      content.className = "row-content";
      content.appendChild(title);

      const summary = document.createElement("div");
      summary.className = "row-summary";

      const refreshSummary = () => {
        summary.textContent = this.buildConfigSummary(config);
        summary.classList.toggle("empty", summary.textContent === "No adjustments");
      };

      refreshSummary();
      content.appendChild(summary);
      rowMain.appendChild(content);

      const actions = document.createElement("div");
      actions.className = "row-actions";

      const editButton = document.createElement("button");
      editButton.type = "button";
      editButton.className = "btn small outline";
      editButton.textContent = editing ? "Done" : "Edit";
      editButton.addEventListener("click", () => {
        this.setConfigEditing(difficulty, config, !editing);
        this.render(difficulty);
      });
      actions.appendChild(editButton);

      const remove = document.createElement("button");
      remove.type = "button";
      remove.className = "btn small danger";
      remove.textContent = "Remove";
      remove.addEventListener("click", () => {
        if (confirm("Remove this config block?")) {
          const listRef = this.ensureConfigList(this.state.data[difficulty]);
          const currentIndex = listRef.indexOf(config);
          this.setConfigEditing(difficulty, config, false);
          if (currentIndex !== -1) {
            listRef.splice(currentIndex, 1);
          }
          this.render(difficulty);
          this.markDirty();
        }
      });
      actions.appendChild(remove);

      rowMain.appendChild(actions);
      row.appendChild(rowMain);

      if (editing) {
        row.appendChild(this.buildEditor(config, refreshSummary));
      }

      container.appendChild(row);
    });

    this.dragManager.attach(container, list, () => this.render(difficulty));
  }

  /**
   * @param {SequenceConfigEntry} config
   * @param {() => void} refreshSummary
   * @returns {HTMLDivElement}
   */
  buildEditor(config, refreshSummary) {
    const editor = document.createElement("div");
    editor.className = "config-editor";

    if (!config.healthSettings) {
      config.healthSettings = {};
    }

    const percentGrid = document.createElement("div");
    percentGrid.className = "config-editor-grid";
    const inventoryGrid = document.createElement("div");
    inventoryGrid.className = "config-editor-grid";

    const percentFields = [];
    const inventoryFields = [];
    this.healthFieldDefinitions().forEach((fieldDef) => {
      const type = fieldDef[3];
      if (type === "count") {
        inventoryFields.push(fieldDef);
        return;
      }
      percentFields.push(fieldDef);
    });

    const buildNumericField = ([field, editLabel]) => {
      const input = document.createElement("input");
      input.type = "number";
      input.placeholder = "Value";
      input.value = config.healthSettings[field] != null ? String(config.healthSettings[field]) : "";
      input.addEventListener("input", (event) => {
        const target = /** @type {HTMLInputElement} */ (event.target);
        config.healthSettings[field] = parseOptionalNumber(target.value);
        refreshSummary();
        this.markDirty();
      });
      return buildField(editLabel, input, "config-editor-field");
    };

    const levelField = (() => {
      const input = document.createElement("input");
      input.type = "number";
      input.placeholder = "Apply at level";
      input.value = config.level != null ? String(config.level) : "";
      input.addEventListener("input", (event) => {
        const target = /** @type {HTMLInputElement} */ (event.target);
        config.level = parseOptionalNumber(target.value);
        refreshSummary();
        this.markDirty();
      });
      return buildField("Character level", input, "config-editor-field");
    })();
    percentGrid.appendChild(levelField);

    percentFields.forEach((fieldDef) => {
      percentGrid.appendChild(buildNumericField(fieldDef));
    });

    editor.appendChild(percentGrid);

    inventoryFields.forEach((fieldDef) => {
      inventoryGrid.appendChild(buildNumericField(fieldDef));
    });

    if (inventoryFields.length) {
      editor.appendChild(inventoryGrid);
    }

    let beltColumns = this.dataAdapter.normalizeBeltColumns(config.healthSettings.beltColumns);
    const beltGrid = document.createElement("div");
    beltGrid.className = "config-editor-grid";
    const beltOptions = this.dataAdapter.beltColumnOptions();

    this.dataAdapter.beltFieldDefinitions().forEach(({ index, label }) => {
      const select = document.createElement("select");

      const inherit = document.createElement("option");
      inherit.value = "";
      inherit.textContent = "Use character setting";
      select.appendChild(inherit);

      beltOptions.forEach((option) => {
        const opt = document.createElement("option");
        opt.value = option.value;
        opt.textContent = option.label;
        select.appendChild(opt);
      });

      select.value = beltColumns[index] ?? "";
      select.addEventListener("change", (event) => {
        const target = /** @type {HTMLSelectElement} */ (event.target);
        beltColumns[index] = target.value || undefined;
        beltColumns = this.dataAdapter.normalizeBeltColumns(beltColumns);
        if (beltColumns.some(Boolean)) {
          config.healthSettings.beltColumns = beltColumns;
        } else {
          delete config.healthSettings.beltColumns;
        }
        refreshSummary();
        this.markDirty();
      });

      beltGrid.appendChild(buildField(label, select, "config-editor-field"));
    });

    editor.appendChild(beltGrid);

    const curseLabel = document.createElement("div");
    curseLabel.className = "config-editor-section-label";
    curseLabel.textContent = "Chicken on Curses";
    editor.appendChild(curseLabel);

    const curseGrid = document.createElement("div");
    curseGrid.className = "config-editor-grid";

    this.dataAdapter.chickenCurseFieldDefinitions().forEach(([field, editLabel]) => {
      const wrapper = document.createElement("div");
      wrapper.className = "config-editor-field checkbox-field";

      const label = document.createElement("label");
      label.className = "checkbox-label";

      const checkbox = document.createElement("input");
      checkbox.type = "checkbox";
      const currentValue = config.healthSettings[field];
      checkbox.checked = currentValue === true;
      checkbox.indeterminate = currentValue == null;

      checkbox.addEventListener("click", () => {
        const current = config.healthSettings[field];
        if (current == null) {
          config.healthSettings[field] = true;
          checkbox.checked = true;
          checkbox.indeterminate = false;
        } else if (current === true) {
          config.healthSettings[field] = false;
          checkbox.checked = false;
          checkbox.indeterminate = false;
        } else {
          delete config.healthSettings[field];
          checkbox.checked = false;
          checkbox.indeterminate = true;
        }
        refreshSummary();
        this.markDirty();
      });

      const text = document.createElement("span");
      text.textContent = editLabel;

      label.appendChild(checkbox);
      label.appendChild(text);
      wrapper.appendChild(label);
      curseGrid.appendChild(wrapper);
    });

    editor.appendChild(curseGrid);

    // Chicken on Auras section
    const auraLabel = document.createElement("div");
    auraLabel.className = "config-editor-section-label";
    auraLabel.textContent = "Chicken on Auras";
    editor.appendChild(auraLabel);

    const auraGrid = document.createElement("div");
    auraGrid.className = "config-editor-grid";

    this.dataAdapter.chickenAuraFieldDefinitions().forEach(([field, editLabel]) => {
      const wrapper = document.createElement("div");
      wrapper.className = "config-editor-field checkbox-field";

      const label = document.createElement("label");
      label.className = "checkbox-label";

      const checkbox = document.createElement("input");
      checkbox.type = "checkbox";
      const currentValue = config.healthSettings[field];
      checkbox.checked = currentValue === true;
      checkbox.indeterminate = currentValue == null;

      checkbox.addEventListener("click", () => {
        const current = config.healthSettings[field];
        if (current == null) {
          config.healthSettings[field] = true;
          checkbox.checked = true;
          checkbox.indeterminate = false;
        } else if (current === true) {
          config.healthSettings[field] = false;
          checkbox.checked = false;
          checkbox.indeterminate = false;
        } else {
          delete config.healthSettings[field];
          checkbox.checked = false;
          checkbox.indeterminate = true;
        }
        refreshSummary();
        this.markDirty();
      });

      const text = document.createElement("span");
      text.textContent = editLabel;

      label.appendChild(checkbox);
      label.appendChild(text);
      wrapper.appendChild(label);
      auraGrid.appendChild(wrapper);
    });

    editor.appendChild(auraGrid);

    return editor;
  }

  /**
   * @param {SequenceConfigEntry|null} config
   * @returns {string}
   */
  buildConfigSummary(config) {
    if (!config) {
      return "No adjustments";
    }

    const parts = [];
    if (config.level != null) {
      parts.push(`Level ≥ ${config.level}`);
    }

    const settings = config.healthSettings || {};
    this.healthFieldDefinitions().forEach((fieldDef) => {
      const [field, , summaryLabel] = fieldDef;
      const fieldType = fieldDef[3];
      const value = settings[field];
      if (value != null) {
        if (fieldType === "count") {
          parts.push(`${summaryLabel}: ${value}`);
        } else {
          parts.push(`${summaryLabel} @ ${value}%`);
        }
      }
    });

    const beltSummary = this.dataAdapter.formatBeltColumns(settings.beltColumns);
    if (beltSummary) {
      parts.push(`Belt: ${beltSummary}`);
    }

    // Summarize curse settings (only show enabled ones)
    const curseOn = [];
    this.dataAdapter.chickenCurseFieldDefinitions().forEach(([field, , summaryLabel]) => {
      if (settings[field] === true) curseOn.push(summaryLabel);
    });
    if (curseOn.length) parts.push(`Chicken on Curse: ${curseOn.join(", ")}`);

    // Summarize aura settings (only show enabled ones)
    const auraOn = [];
    this.dataAdapter.chickenAuraFieldDefinitions().forEach(([field, , summaryLabel]) => {
      if (settings[field] === true) auraOn.push(summaryLabel);
    });
    if (auraOn.length) parts.push(`Chicken on Aura: ${auraOn.join(", ")}`);

    return parts.length ? parts.join(" • ") : "No adjustments";
  }

  /**
   * @returns {import("../SequenceDataAdapter.js").HealthFieldDefinition[]}
   */
  healthFieldDefinitions() {
    return this.dataAdapter.healthFieldDefinitions();
  }
}
