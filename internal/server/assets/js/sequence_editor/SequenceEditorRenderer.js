// @ts-check

import { DomTargetResolver } from "./dom/DomTargetResolver.js";
import { DIFFICULTIES, RENDER_PIPELINE } from "./constants.js";
import { ConditionSectionRenderer } from "./renderers/ConditionSectionRenderer.js";
import { ConfigSectionRenderer } from "./renderers/ConfigSectionRenderer.js";
import { DragReorderManager } from "./renderers/DragReorderManager.js";
import { QuestSectionRenderer } from "./renderers/QuestSectionRenderer.js";
import { RunSectionRenderer } from "./renderers/RunSectionRenderer.js";
import { SectionRendererRegistry } from "./renderers/SectionRendererRegistry.js";

/**
 * @typedef {import("./constants.js").DifficultyKey} DifficultyKey
 * @typedef {import("./constants.js").RunSectionKey} RunSectionKey
 * @typedef {import("./SequenceEditorState.js").SequenceEditorState} SequenceEditorState
 * @typedef {import("./SequenceDataAdapter.js").SequenceDataAdapter} SequenceDataAdapter
 * @typedef {import("./SequenceDataAdapter.js").DifficultySettings} DifficultySettings
 * @typedef {import("./SequenceDataAdapter.js").SequenceRunEntry} SequenceRunEntry
 * @typedef {import("./SequenceDataAdapter.js").SequenceConfigEntry} SequenceConfigEntry
 * @typedef {import("./renderers/RunSectionRenderer.js").RunSectionRenderer} RunRenderer
 * @typedef {import("./renderers/QuestSectionRenderer.js").QuestSectionRenderer} QuestRenderer
 * @typedef {import("./renderers/ConditionSectionRenderer.js").ConditionSectionRenderer} ConditionRenderer
 * @typedef {import("./renderers/ConfigSectionRenderer.js").ConfigSectionRenderer} ConfigRenderer
 * @typedef {RunSectionKey|"quests"} RunListKey
 * @typedef {{panel:HTMLElement|null, dirtyIndicator:HTMLElement|null, saveButton:HTMLButtonElement|null, tabs:HTMLElement[]}} RendererElements
 */

/** Handles DOM updates for each sequence editor section. */
export class SequenceEditorRenderer {
  /**
   * @param {SequenceEditorState} state
   * @param {SequenceDataAdapter} dataAdapter
   * @param {RendererElements} [elements]
   * @param {{domTargets?:DomTargetResolver}} [options]
   */
  constructor(state, dataAdapter, elements, { domTargets } = {}) {
    this.state = state;
    this.dataAdapter = dataAdapter;
    this.elements = elements;
    this.domTargets = domTargets ?? new DomTargetResolver();
    this.sectionRegistry = new SectionRendererRegistry();

    this.markDirty = this.markDirty.bind(this);
    this.ensureRunList = this.ensureRunList.bind(this);
    this.ensureConfigList = this.ensureConfigList.bind(this);

    this.dragManager = new DragReorderManager(this.state, this.markDirty);

    this.runRenderer = new RunSectionRenderer({
      state: this.state,
      ensureRunList: this.ensureRunList,
      getAvailableRunsForSection: (section) => this.getAvailableRunsForSection(section),
      isRunEditing: (difficulty, section, entry) => this.isRunEditing(difficulty, section, entry),
      setRunEditing: (difficulty, section, entry, enabled) => this.setRunEditing(difficulty, section, entry, enabled),
      markDirty: this.markDirty,
      dragManager: this.dragManager,
      domTargets: this.domTargets,
    });

    this.questRenderer = new QuestSectionRenderer({
      state: this.state,
      dataAdapter: this.dataAdapter,
      ensureRunList: this.ensureRunList,
      markDirty: this.markDirty,
      isQuestEditing: (difficulty, run) => this.isQuestEditing(difficulty, run),
      setQuestEditing: (difficulty, run, enabled) => this.setQuestEditing(difficulty, run, enabled),
      syncQuestData: () => this.syncQuestData(),
      domTargets: this.domTargets,
    });

    this.conditionRenderer = new ConditionSectionRenderer({
      state: this.state,
      dataAdapter: this.dataAdapter,
      markDirty: this.markDirty,
      isConditionEditing: this.isConditionEditing.bind(this),
      setConditionEditing: this.setConditionEditing.bind(this),
      domTargets: this.domTargets,
    });

    this.configRenderer = new ConfigSectionRenderer({
      state: this.state,
      dataAdapter: this.dataAdapter,
      ensureConfigList: this.ensureConfigList,
      markDirty: this.markDirty,
      dragManager: this.dragManager,
      isConfigEditing: (difficulty, entry) => this.isConfigEditing(difficulty, entry),
      setConfigEditing: (difficulty, entry, enabled) => this.setConfigEditing(difficulty, entry, enabled),
      domTargets: this.domTargets,
    });

    this.registerSectionRenderers();
  }

  /**
   * @param {RendererElements} elements
   */
  setElements(elements) {
    this.elements = elements;
    this.domTargets.reset();
  }

  /** Renders the entire editor layout for all difficulties. */
  renderEditor() {
    if (!this.state.data || !this.elements?.panel) {
      return;
    }

    this.elements.panel.hidden = false;

    if (this.state.questCatalog.length) {
      this.syncQuestData();
    }

    DIFFICULTIES.forEach((difficulty) => this.renderDifficulty(difficulty));

    this.updateDirtyIndicator();
    this.updateSaveState();
  }

  /**
   * @param {DifficultyKey} difficulty
   */
  renderDifficulty(difficulty) {
    RENDER_PIPELINE.forEach((step) => this.renderSection(difficulty, step));
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {{type:string, section?:RunSectionKey}} step
   */
  renderSection(difficulty, step) {
    if (!step || !step.type) {
      return;
    }
    this.sectionRegistry.render(step.type, {
      difficulty,
      step: /** @type {{type:string, section?:RunSectionKey}} */ (step),
    });
  }

  /** Registers renderer callbacks for each section type. */
  registerSectionRenderers() {
    const registryConfig = {
      run: ({ difficulty, step }) => {
        if (!step?.section) {
          return;
        }
        this.runRenderer.render(difficulty, step.section);
      },
      quest: ({ difficulty }) => {
        this.questRenderer.render(difficulty);
      },
      condition: ({ difficulty }) => {
        this.conditionRenderer.render(difficulty);
      },
      config: ({ difficulty }) => {
        this.configRenderer.render(difficulty);
      },
    };

    Object.entries(registryConfig).forEach(([type, handler]) => {
      this.sectionRegistry.register(type, handler);
    });
  }

  /** Aligns quest lists with the master quest catalog ordering. */
  syncQuestData() {
    if (!this.state.data || !this.state.questCatalog.length) {
      return;
    }

    const order = new Map();
    this.state.questCatalog.forEach((quest, index) => {
      if (quest.run) {
        order.set(quest.run, index);
      }
    });

    DIFFICULTIES.forEach((difficulty) => {
      const settings = this.state.data[difficulty];
      if (!settings) {
        return;
      }

      const list = this.ensureRunList(settings, "quests");
      const questMap = new Map();

      list.forEach((entry) => {
        if (!entry || !entry.run || !order.has(entry.run)) {
          return;
        }
        if (!questMap.has(entry.run)) {
          questMap.set(entry.run, entry);
        }
      });

      const ordered = [];
      this.state.questCatalog.forEach((quest) => {
        const entry = questMap.get(quest.run);
        if (entry) {
          ordered.push(entry);
        }
      });

      settings.quests = ordered;
    });
  }

  /**
   * @param {RunListKey} section
   * @returns {string[]}
   */
  getAvailableRunsForSection(section) {
    if (section === "beforeQuests" || section === "afterQuests") {
      return this.state.sequencerRuns.length ? this.state.sequencerRuns.slice() : this.state.runs.slice();
    }
    return this.state.runs.slice();
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {string} run
   * @returns {boolean}
   */
  isQuestEditing(difficulty, run) {
    return this.state.isEditingKey("quest", difficulty, run);
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {string} run
   * @param {boolean} enabled
   */
  setQuestEditing(difficulty, run, enabled) {
    this.state.setEditingKey("quest", enabled, difficulty, run);
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {RunSectionKey} section
   * @param {import("./SequenceDataAdapter.js").SequenceRunEntry} entry
   * @returns {Array<string>}
   */
  runEntryKeyParts(difficulty, section, entry) {
    const uid = this.state.ensureEntryUID(entry);
    return [difficulty, section, uid];
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {RunSectionKey} section
   * @param {import("./SequenceDataAdapter.js").SequenceRunEntry} entry
   * @returns {boolean}
   */
  isRunEditing(difficulty, section, entry) {
    return this.state.isEditingKey("run", ...this.runEntryKeyParts(difficulty, section, entry));
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {RunSectionKey} section
   * @param {import("./SequenceDataAdapter.js").SequenceRunEntry} entry
   * @param {boolean} enabled
   */
  setRunEditing(difficulty, section, entry, enabled) {
    this.state.setEditingKey("run", enabled, ...this.runEntryKeyParts(difficulty, section, entry));
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {import("./SequenceDataAdapter.js").SequenceConfigEntry} entry
   * @returns {boolean}
   */
  isConfigEditing(difficulty, entry) {
    const uid = this.state.ensureEntryUID(entry);
    return this.state.isEditingKey("config", difficulty, uid);
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {import("./SequenceDataAdapter.js").SequenceConfigEntry} entry
   * @param {boolean} enabled
   */
  setConfigEditing(difficulty, entry, enabled) {
    const uid = this.state.ensureEntryUID(entry);
    this.state.setEditingKey("config", enabled, difficulty, uid);
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {string} key
   * @returns {boolean}
   */
  isConditionEditing(difficulty, key) {
    return this.state.isEditingKey("condition", difficulty, key);
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {string} key
   * @param {boolean} enabled
   */
  setConditionEditing(difficulty, key, enabled) {
    this.state.setEditingKey("condition", enabled, difficulty, key);
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {RunSectionKey} section
   */
  addSequenceRow(difficulty, section) {
    if (!this.state.data || !this.state.data[difficulty]) {
      return;
    }
    const list = this.ensureRunList(this.state.data[difficulty], section);
    const entry = this.dataAdapter.createEmptyRunEntry();
    this.state.ensureEntryUID(entry);
    list.push(entry);
    this.setRunEditing(difficulty, section, entry, true);
    this.runRenderer.render(difficulty, section);
    this.markDirty();
  }

  /**
   * @param {DifficultyKey} difficulty
   */
  addConfigBlock(difficulty) {
    if (!this.state.data || !this.state.data[difficulty]) {
      return;
    }
    const list = this.ensureConfigList(this.state.data[difficulty]);
    const entry = {
      level: null,
      healthSettings: {},
    };
    this.state.ensureEntryUID(entry);
    list.push(entry);
    this.setConfigEditing(difficulty, entry, true);
    this.configRenderer.render(difficulty);
    this.markDirty();
  }

  /**
   * @param {DifficultySettings} target
   * @param {RunListKey} key
   * @returns {SequenceRunEntry[]}
   */
  ensureRunList(target, key) {
    if (!Array.isArray(target[key])) {
      target[key] = [];
    }
    return /** @type {SequenceRunEntry[]} */ (target[key]);
  }

  /**
   * @param {DifficultySettings} target
   * @returns {SequenceConfigEntry[]}
   */
  ensureConfigList(target) {
    if (!Array.isArray(target.configSettings)) {
      target.configSettings = [];
    }
    return target.configSettings;
  }

  /**
   * @param {string} tabName
   */
  switchTab(tabName) {
    this.elements.tabs.forEach((tab) => {
      const isActive = tab.dataset.tab === tabName;
      tab.classList.toggle("active", isActive);
    });
    DIFFICULTIES.forEach((difficulty) => {
      const panel = this.domTargets.getTabPanel(difficulty);
      if (!panel) {
        return;
      }
      panel.classList.toggle("active", difficulty === tabName);
    });
  }

  /** Updates the unsaved indicator visibility. */
  updateDirtyIndicator() {
    if (!this.elements?.dirtyIndicator) {
      return;
    }
    this.elements.dirtyIndicator.hidden = !this.state.dirty;
  }

  /** Enables or disables the save button depending on load state. */
  updateSaveState() {
    if (!this.elements?.saveButton) {
      return;
    }
    this.elements.saveButton.disabled = !this.state.data;
  }

  /** Flags the editor as dirty and refreshes UI affordances. */
  markDirty() {
    this.state.dirty = true;
    this.updateDirtyIndicator();
    this.updateSaveState();
  }
}
