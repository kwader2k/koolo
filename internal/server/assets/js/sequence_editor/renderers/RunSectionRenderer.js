// @ts-check

import { buildField, createDragHandle, prettifyRunName } from "../utils.js";
import { buildRunSummary, createRunParameterEditor } from "./helpers/RunEntryEditor.js";

/**
 * @typedef {import("../constants.js").DifficultyKey} DifficultyKey
 * @typedef {import("../constants.js").RunSectionKey} RunSectionKey
 * @typedef {import("../SequenceEditorState.js").SequenceEditorState} SequenceEditorState
 * @typedef {import("../SequenceDataAdapter.js").SequenceRunEntry} SequenceRunEntry
 * @typedef {import("../renderers/DragReorderManager.js").DragReorderManager} DragReorderManager
 * @typedef {import("../dom/DomTargetResolver.js").DomTargetResolver} DomTargetResolver
 */

/** Renders the before/after quest run lists with inline editing. */
export class RunSectionRenderer {
  constructor({
    state,
    ensureRunList,
    getAvailableRunsForSection,
    isRunEditing,
    setRunEditing,
    markDirty,
    dragManager,
    domTargets,
  }) {
    this.state = state;
    this.ensureRunList = ensureRunList;
    this.getAvailableRunsForSection = getAvailableRunsForSection;
    this.isRunEditing = isRunEditing;
    this.setRunEditing = setRunEditing;
    this.markDirty = markDirty;
    this.dragManager = dragManager;
    this.domTargets = domTargets;

    if (!this.domTargets) {
      throw new Error("RunSectionRenderer requires a domTargets resolver instance");
    }
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {RunSectionKey} section
   */
  render(difficulty, section) {
    const container = this.domTargets.getRunSectionContainer(difficulty, section);

    if (!container || !this.state.data) {
      return;
    }

    const list = this.ensureRunList(this.state.data[difficulty], section);
    container.innerHTML = "";

    if (!list.length) {
      const emptyState = document.createElement("div");
      emptyState.className = "muted";
      emptyState.textContent = "No runs configured yet.";
      container.appendChild(emptyState);
      this.dragManager.teardown(container);
      return;
    }

    const availableRuns = this.getAvailableRunsForSection(section);

    list.forEach((entry) => {
      this.state.ensureEntryUID(entry);
      const editing = this.isRunEditing(difficulty, section, entry);

      const row = document.createElement("div");
      row.className = "sequence-row run-row";
      row.classList.toggle("editing", editing);
      row.dataset.uid = String(entry.__uid);

      const rowMain = document.createElement("div");
      rowMain.className = "row-main";

      const handle = createDragHandle("Reorder run entry");
      rowMain.appendChild(handle);

      const title = document.createElement("div");
      title.className = "row-title";
      const content = document.createElement("div");
      content.className = "row-content";
      content.appendChild(title);

      const summary = document.createElement("div");
      summary.className = "row-summary";
      content.appendChild(summary);
      rowMain.appendChild(content);

      const actions = document.createElement("div");
      actions.className = "row-actions";

      const editButton = document.createElement("button");
      editButton.type = "button";
      editButton.className = "btn small outline";
      editButton.textContent = editing ? "Done" : "Edit";
      editButton.addEventListener("click", () => {
        this.setRunEditing(difficulty, section, entry, !editing);
        this.render(difficulty, section);
      });
      actions.appendChild(editButton);

      const remove = document.createElement("button");
      remove.type = "button";
      remove.className = "btn small danger";
      remove.textContent = "Remove";
      remove.addEventListener("click", () => {
        if (confirm("Remove this run entry?")) {
          const listRef = this.ensureRunList(this.state.data[difficulty], section);
          const currentIndex = listRef.indexOf(entry);
          this.setRunEditing(difficulty, section, entry, false);
          if (currentIndex !== -1) {
            listRef.splice(currentIndex, 1);
          }
          this.render(difficulty, section);
          this.markDirty();
        }
      });
      actions.appendChild(remove);

      rowMain.appendChild(actions);
      row.appendChild(rowMain);

      const refreshDisplay = () => {
        const runName = entry.run ? prettifyRunName(entry.run) : "Select a run";
        title.textContent = runName;

        const summaryText = entry.run ? buildRunSummary(entry, difficulty) : "No run selected.";
        summary.textContent = summaryText;
        summary.classList.toggle("empty", !entry.run || summaryText === "No modifiers");
        row.classList.toggle("disabled", !entry.run);
      };

      refreshDisplay();

      if (editing) {
        const editorContainer = document.createElement("div");
        editorContainer.className = "run-editor";

        const runSelect = document.createElement("select");
        const placeholder = document.createElement("option");
        placeholder.value = "";
        placeholder.textContent = "Select run";
        runSelect.appendChild(placeholder);

        availableRuns.forEach((runName) => {
          const opt = document.createElement("option");
          opt.value = runName;
          opt.textContent = prettifyRunName(runName);
          runSelect.appendChild(opt);
        });

        if (entry.run && !availableRuns.includes(entry.run)) {
          const opt = document.createElement("option");
          opt.value = entry.run;
          opt.textContent = prettifyRunName(entry.run);
          runSelect.appendChild(opt);
        }

        runSelect.value = entry.run || "";
        runSelect.addEventListener("change", (event) => {
          const target = /** @type {HTMLSelectElement} */ (event.target);
          entry.run = target.value;
          this.markDirty();
          this.render(difficulty, section);
        });

        const runField = buildField("Run", runSelect, "run-editor-field");

        const parametersEditor = createRunParameterEditor(entry, {
          markDirty: this.markDirty,
          onChange: () => {
            refreshDisplay();
          },
          difficulty,
        });

        const grid = parametersEditor.querySelector(".run-parameter-grid");
        if (grid) {
          grid.prepend(runField);
        }

        editorContainer.appendChild(parametersEditor);
        row.appendChild(editorContainer);
      }

      container.appendChild(row);
    });

    this.dragManager.attach(container, list, () => this.render(difficulty, section));
  }
}
