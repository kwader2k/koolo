import { buildField, normalizeRunNumericValue, parseOptionalNumber } from "../../utils.js";

// @ts-check

/** @typedef {import("../../SequenceDataAdapter.js").SequenceRunEntry} SequenceRunEntry */

/**
 * @param {SequenceRunEntry|null} entry
 * @returns {string}
 */
export function buildQuestSummary(entry) {
  if (!entry) {
    return "";
  }

  const parts = [];
  const hasMin = entry.minLevel != null;
  const hasMax = entry.maxLevel != null;

  if (hasMin && hasMax) {
    parts.push(`Level ${entry.minLevel}-${entry.maxLevel}`);
  } else if (hasMin) {
    parts.push(`Level >= ${entry.minLevel}`);
  } else if (hasMax) {
    parts.push(`Level <= ${entry.maxLevel}`);
  }

  if (entry.lowGoldRun) {
    parts.push("Low gold");
  }
  if (entry.skipTownChores) {
    parts.push("Skip chores");
  }
  if (entry.exitGame) {
    parts.push("Exit game");
  }
  if (entry.stopIfCheckFails) {
    parts.push("Stop on fail");
  }

  return parts.join(" • ");
}

/**
 * @param {SequenceRunEntry} entry
 * @param {{markDirty:() => void, onChange:() => void}} callbacks
 * @returns {HTMLDivElement}
 */
export function createQuestParameterEditor(entry, { markDirty, onChange }) {
  const editor = document.createElement("div");
  editor.className = "quest-editor";

  const grid = document.createElement("div");
  grid.className = "quest-editor-grid";

  const notifyChange = () => {
    if (typeof onChange === "function") {
      onChange();
    }
    if (typeof markDirty === "function") {
      markDirty();
    }
  };

  const minInput = document.createElement("input");
  minInput.type = "number";
  minInput.placeholder = "Min level";
  minInput.value = entry.minLevel ?? "";
  minInput.addEventListener("input", (event) => {
    entry.minLevel = normalizeRunNumericValue(parseOptionalNumber(event.target.value));
    notifyChange();
  });
  grid.appendChild(buildField("Min Level", minInput, "quest-editor-field"));

  const maxInput = document.createElement("input");
  maxInput.type = "number";
  maxInput.placeholder = "Max level";
  maxInput.value = entry.maxLevel ?? "";
  maxInput.addEventListener("input", (event) => {
    entry.maxLevel = normalizeRunNumericValue(parseOptionalNumber(event.target.value));
    notifyChange();
  });
  grid.appendChild(buildField("Max Level", maxInput, "quest-editor-field"));

  editor.appendChild(grid);

  const flags = document.createElement("div");
  flags.className = "checkbox-grid quest-editor-flags";

  const flagDefinitions = [
    ["lowGoldRun", "Low gold run"],
    ["skipTownChores", "Skip town chores"],
    ["exitGame", "Exit game after"],
    ["stopIfCheckFails", "Stop on failure"],
  ];

  flagDefinitions.forEach(([field, label]) => {
    const wrapper = document.createElement("label");
    wrapper.className = "checkbox-field";
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.checked = Boolean(entry[field]);
    checkbox.addEventListener("change", (event) => {
      entry[field] = event.target.checked;
      notifyChange();
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
