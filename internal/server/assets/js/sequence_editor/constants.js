// @ts-check

/**
 * @typedef {"normal"|"nightmare"|"hell"} DifficultyKey
 * @typedef {"beforeQuests"|"afterQuests"} RunSectionKey
 * @typedef {"nextDifficultyConditions"|"stayDifficultyConditions"} ConditionSectionKey
 */

/**
 * Ordered list of supported Diablo II difficulties.
 * @type {DifficultyKey[]}
 */
export const DIFFICULTIES = ["normal", "nightmare", "hell"];

/**
 * Sequence in which editor sections render for each tab.
 * @type {Array<{type:"run"|"quest"|"condition"|"config", section?:RunSectionKey}>}
 */
export const RENDER_PIPELINE = [
  { type: "run", section: "beforeQuests" },
  { type: "quest" },
  { type: "run", section: "afterQuests" },
  { type: "condition" },
  { type: "config" },
];

/**
 * Mapping of difficulty to the condition sections shown in the UI.
 * @type {Record<DifficultyKey, Array<{key:ConditionSectionKey, title:string, autoSyncInfo?:string}>>}
 */
export const CONDITION_SECTIONS = {
  normal: [{
    key: "nextDifficultyConditions",
    title: "Next Difficulty Conditions",
    autoSyncInfo: "Auto-synced with Nightmare's Stay Difficulty Conditions"
  }],
  nightmare: [
    {
      key: "stayDifficultyConditions",
      title: "Stay Difficulty Conditions",
      autoSyncInfo: "Auto-synced with Normal's Next Difficulty Conditions"
    },
    {
      key: "nextDifficultyConditions",
      title: "Next Difficulty Conditions",
      autoSyncInfo: "Auto-synced with Hell's Stay Difficulty Conditions"
    },
  ],
  hell: [{
    key: "stayDifficultyConditions",
    title: "Stay Difficulty Conditions",
    autoSyncInfo: "Auto-synced with Nightmare's Next Difficulty Conditions"
  }],
};
