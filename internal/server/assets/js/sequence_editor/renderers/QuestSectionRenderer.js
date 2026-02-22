// @ts-check

import { prettifyRunName, toRoman } from "../utils.js";
import { buildQuestSummary, createQuestParameterEditor } from "./helpers/QuestEntryEditor.js";

/**
 * @typedef {import("../constants.js").DifficultyKey} DifficultyKey
 * @typedef {import("../SequenceEditorState.js").SequenceEditorState} SequenceEditorState
 * @typedef {import("../SequenceDataAdapter.js").SequenceDataAdapter} SequenceDataAdapter
 * @typedef {import("../SequenceDataAdapter.js").SequenceRunEntry} SequenceRunEntry
 * @typedef {import("../dom/DomTargetResolver.js").DomTargetResolver} DomTargetResolver
 */

/** Handles rendering of quest toggles and editors per act. */
export class QuestSectionRenderer {
  constructor({
    state,
    dataAdapter,
    ensureRunList,
    markDirty,
    isQuestEditing,
    setQuestEditing,
    syncQuestData,
    domTargets,
  }) {
    this.state = state;
    this.dataAdapter = dataAdapter;
    this.ensureRunList = ensureRunList;
    this.markDirty = markDirty;
    this.isQuestEditing = isQuestEditing;
    this.setQuestEditing = setQuestEditing;
    this.syncQuestData = syncQuestData;
    this.domTargets = domTargets;
    this.activeActs = new Map();

    if (!this.domTargets) {
      throw new Error("QuestSectionRenderer requires a domTargets resolver instance");
    }
  }

  /**
   * @param {DifficultyKey} difficulty
   */
  render(difficulty) {
    const container = this.domTargets.getQuestContainer(difficulty);
    if (!container || !this.state.data) {
      return;
    }

    container.innerHTML = "";

    if (!this.state.questCatalog.length) {
      const emptyState = document.createElement("div");
      emptyState.className = "muted";
      emptyState.textContent = "Quest catalog unavailable.";
      container.appendChild(emptyState);
      return;
    }

    const settings = this.state.data[difficulty];
    if (!settings) {
      return;
    }

    const list = this.ensureRunList(settings, "quests");
    const questMap = new Map();
    list.forEach((entry) => {
      if (entry && entry.run) {
        questMap.set(entry.run, entry);
      }
    });

    const groupedByAct = new Map();
    this.state.questCatalog.forEach((quest) => {
      const actKey = quest.act || 0;
      if (!groupedByAct.has(actKey)) {
        groupedByAct.set(actKey, []);
      }

      // Special case: Frozen Aura mercenary only available in Nightmare difficulty
      if (quest.run === "frozen_aura_merc" && difficulty !== "nightmare") return;

      groupedByAct.get(actKey).push(quest);
    });

    const orderedActs = Array.from(groupedByAct.keys()).sort((a, b) => a - b);

    if (!orderedActs.length) {
      const empty = document.createElement("div");
      empty.className = "muted";
      empty.textContent = "No quests available.";
      container.appendChild(empty);
      return;
    }

    let activeAct = this.activeActs.get(difficulty);
    if (!orderedActs.includes(activeAct)) {
      activeAct = orderedActs[0];
      this.activeActs.set(difficulty, activeAct);
    }

    if (orderedActs.length > 1) {
      const tabs = document.createElement("div");
      tabs.className = "quest-act-tabs";
      orderedActs.forEach((actNumber) => {
        const label = toRoman(actNumber) || actNumber;
        const tabButton = document.createElement("button");
        tabButton.type = "button";
        tabButton.className = "btn small outline quest-act-tab";
        tabButton.textContent = `Act ${label}`;
        if (actNumber === activeAct) {
          tabButton.classList.add("active");
        }
        tabButton.addEventListener("click", () => {
          if (this.activeActs.get(difficulty) === actNumber) {
            return;
          }
          this.activeActs.set(difficulty, actNumber);
          this.render(difficulty);
        });
        tabs.appendChild(tabButton);
      });
      container.appendChild(tabs);
    }

    const panel = document.createElement("div");
    panel.className = "quest-act-panel";
    container.appendChild(panel);

    const targetAct = activeAct ?? orderedActs[0];
    if (targetAct == null) {
      const empty = document.createElement("div");
      empty.className = "muted";
      empty.textContent = "No quests available.";
      panel.appendChild(empty);
      return;
    }

    const actSection = this.buildActSection({
      actNumber: targetAct,
      groupedByAct,
      questMap,
      difficulty,
    });
    panel.appendChild(actSection);
  }

  /**
   * @param {{actNumber:number, groupedByAct:Map<number, Array<{run:string,isMandatory?:boolean,act:number}>>, questMap:Map<string, SequenceRunEntry|null>, difficulty:DifficultyKey}} payload
   * @returns {HTMLDivElement}
   */
  buildActSection({ actNumber, groupedByAct, questMap, difficulty }) {
    const quests = groupedByAct.get(actNumber) || [];
    const actSection = document.createElement("div");
    actSection.className = "quest-act";

    quests.forEach((quest) => {
      const entry = questMap.get(quest.run) || null;
      const editing = entry && this.isQuestEditing(difficulty, quest.run);

      const row = document.createElement("div");
      row.className = "sequence-row quest-row";
      row.classList.toggle("editing", editing);
      row.classList.toggle("disabled", !entry);

      const rowMain = document.createElement("div");
      rowMain.className = "row-main";

      const toggle = document.createElement("label");
      toggle.className = "quest-toggle";

      const checkbox = document.createElement("input");
      checkbox.type = "checkbox";
      checkbox.checked = Boolean(entry);
      checkbox.addEventListener("change", () => {
        if (quest.isMandatory && !checkbox.checked) {
          const proceed = confirm("This quest is mandatory for progression, are you sure?");

          if (!proceed) {
            checkbox.checked = true;
            return;
          }
        }
        this.setQuestEnabled(difficulty, quest, checkbox.checked);
      });
      toggle.appendChild(checkbox);

      const nameSpan = document.createElement("span");
      nameSpan.className = "quest-name";
      nameSpan.textContent = prettifyRunName(quest.run);
      toggle.appendChild(nameSpan);

      rowMain.appendChild(toggle);

      const summary = document.createElement("div");
      summary.className = "row-summary";
      if (entry) {
        this.updateQuestSummary(summary, entry);
      } else {
        summary.textContent = quest.isMandatory ? "Required quest" : "Excluded";
        summary.classList.add("empty");
      }
      rowMain.appendChild(summary);

      const actions = document.createElement("div");
      actions.className = "row-actions";

      const editButton = document.createElement("button");
      editButton.type = "button";
      editButton.className = "btn small outline";
      editButton.textContent = editing ? "Done" : "Edit";
      editButton.disabled = !entry;
      editButton.addEventListener("click", () => {
        if (!entry) {
          return;
        }
        const currentlyEditing = this.isQuestEditing(difficulty, quest.run);
        this.setQuestEditing(difficulty, quest.run, !currentlyEditing);
        this.render(difficulty);
      });
      actions.appendChild(editButton);

      rowMain.appendChild(actions);
      row.appendChild(rowMain);

      if (entry && editing) {
        const editor = createQuestParameterEditor(entry, {
          markDirty: this.markDirty,
          onChange: () => this.updateQuestSummary(summary, entry),
        });
        row.appendChild(editor);
      }

      actSection.appendChild(row);
    });

    return actSection;
  }

  /**
   * @param {HTMLElement} summaryElement
   * @param {SequenceRunEntry} entry
   */
  updateQuestSummary(summaryElement, entry) {
    const summaryText = buildQuestSummary(entry);
    summaryElement.textContent = summaryText;
    summaryElement.classList.toggle("quest-summary-empty", summaryText === "");
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {{run:string,isMandatory?:boolean}} quest
   * @param {boolean} enabled
   */
  setQuestEnabled(difficulty, quest, enabled) {
    const settings = this.state.data[difficulty];
    if (!settings) {
      return;
    }

    const list = this.ensureRunList(settings, "quests");
    const existingIndex = list.findIndex((entry) => entry.run === quest.run);
    let changed = false;

    if (enabled) {
      if (existingIndex === -1) {
        const entry = this.dataAdapter.createEmptyRunEntry();
        entry.run = quest.run;
        list.push(entry);
        changed = true;
      }
    } else {
      if (existingIndex !== -1) {
        list.splice(existingIndex, 1);
        changed = true;
      }
      this.setQuestEditing(difficulty, quest.run, false);
    }

    if (!changed) {
      return;
    }

    this.syncQuestData();
    this.markDirty();
    this.render(difficulty);
  }
}
