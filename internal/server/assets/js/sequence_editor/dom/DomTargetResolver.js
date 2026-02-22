// @ts-check

/** @typedef {import("../constants.js").DifficultyKey} DifficultyKey */
/** @typedef {import("../constants.js").RunSectionKey} RunSectionKey */

const SELECTORS = {
  run: ({ difficulty, section }) => `.sequence-list[data-difficulty="${difficulty}"][data-section="${section}"]`,
  quest: ({ difficulty }) => `.quest-list[data-difficulty="${difficulty}"]`,
  condition: ({ difficulty }) => `.condition-list[data-difficulty="${difficulty}"]`,
  config: ({ difficulty }) => `.config-settings[data-difficulty="${difficulty}"]`,
  tabPanel: ({ difficulty }) => `.tab-panel[data-difficulty="${difficulty}"]`,
};

const buildKey = (...parts) => parts.filter(Boolean).join(":");

const defaultRoot = typeof document !== "undefined" ? document : null;

/** Resolves frequently accessed DOM nodes with basic caching. */
export class DomTargetResolver {
  /**
   * @param {Document|HTMLElement|null} [queryRoot]
   */
  constructor(queryRoot = defaultRoot) {
    this.queryRoot = queryRoot;
    this.cache = new Map();
  }

  /**
   * @param {Document|HTMLElement|null} queryRoot
   */
  setQueryRoot(queryRoot) {
    if (this.queryRoot === queryRoot) {
      return;
    }
    this.queryRoot = queryRoot;
    this.reset();
  }

  /** Clears the internal selector cache. */
  reset() {
    this.cache.clear();
  }

  /**
   * @param {DifficultyKey} difficulty
   * @param {RunSectionKey} section
   * @returns {HTMLElement|null}
   */
  getRunSectionContainer(difficulty, section) {
    return this.getOrQuery(buildKey("run", difficulty, section), SELECTORS.run({ difficulty, section }));
  }

  /**
   * @param {DifficultyKey} difficulty
   * @returns {HTMLElement|null}
   */
  getQuestContainer(difficulty) {
    return this.getOrQuery(buildKey("quest", difficulty), SELECTORS.quest({ difficulty }));
  }

  /**
   * @param {DifficultyKey} difficulty
   * @returns {HTMLElement|null}
   */
  getConditionContainer(difficulty) {
    return this.getOrQuery(buildKey("condition", difficulty), SELECTORS.condition({ difficulty }));
  }

  /**
   * @param {DifficultyKey} difficulty
   * @returns {HTMLElement|null}
   */
  getConfigContainer(difficulty) {
    return this.getOrQuery(buildKey("config", difficulty), SELECTORS.config({ difficulty }));
  }

  /**
   * @param {DifficultyKey} difficulty
   * @returns {HTMLElement|null}
   */
  getTabPanel(difficulty) {
    return this.getOrQuery(buildKey("tab", difficulty), SELECTORS.tabPanel({ difficulty }));
  }

  /**
   * @param {string} key
   * @param {string} selector
   * @returns {HTMLElement|null}
   */
  getOrQuery(key, selector) {
    if (!selector) {
      return null;
    }

    const cached = this.cache.get(key);
    if (cached && cached.isConnected) {
      return cached;
    }

    const node = this.querySelector(selector);
    if (node) {
      this.cache.set(key, node);
    }

    return node || null;
  }

  /**
   * @param {string} selector
   * @returns {HTMLElement|null}
   */
  querySelector(selector) {
    if (!this.queryRoot || typeof this.queryRoot.querySelector !== "function") {
      return null;
    }
    return this.queryRoot.querySelector(selector);
  }
}
