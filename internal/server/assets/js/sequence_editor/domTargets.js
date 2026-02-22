// @ts-check

import { DomTargetResolver } from "./dom/DomTargetResolver.js";

/** @typedef {import("./constants.js").DifficultyKey} DifficultyKey */
/** @typedef {import("./constants.js").RunSectionKey} RunSectionKey */

/** @type {DomTargetResolver} */
const sharedResolver = new DomTargetResolver();

/**
 * @param {DifficultyKey} difficulty
 * @param {RunSectionKey} section
 * @returns {HTMLElement|null}
 */
export const getRunSectionContainer = (difficulty, section) =>
  sharedResolver.getRunSectionContainer(difficulty, section);

/**
 * @param {DifficultyKey} difficulty
 * @returns {HTMLElement|null}
 */
export const getQuestContainer = (difficulty) => sharedResolver.getQuestContainer(difficulty);

/**
 * @param {DifficultyKey} difficulty
 * @returns {HTMLElement|null}
 */
export const getConditionContainer = (difficulty) => sharedResolver.getConditionContainer(difficulty);

/**
 * @param {DifficultyKey} difficulty
 * @returns {HTMLElement|null}
 */
export const getConfigContainer = (difficulty) => sharedResolver.getConfigContainer(difficulty);

/**
 * @param {DifficultyKey} difficulty
 * @returns {HTMLElement|null}
 */
export const getTabPanel = (difficulty) => sharedResolver.getTabPanel(difficulty);

/** Clears cached DOM lookups so subsequent calls re-query the DOM tree. */
export const resetDomTargetCache = () => {
  sharedResolver.reset();
};

export { DomTargetResolver };
export const domTargetResolver = sharedResolver;
