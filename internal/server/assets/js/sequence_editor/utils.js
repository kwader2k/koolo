// @ts-check

/**
 * Parses a numeric field, returning undefined when blank or invalid.
 * @param {string|number|null|undefined} value
 * @returns {number|undefined}
 */
export function parseOptionalNumber(value) {
  if (value == null || value === "") {
    return undefined;
  }
  const parsed = parseInt(String(value), 10);
  return Number.isNaN(parsed) ? undefined : parsed;
}

/**
 * Normalizes zero/undefined run thresholds to undefined for serialization consistency.
 * @param {number|null|undefined} value
 * @returns {number|undefined}
 */
export function normalizeRunNumericValue(value) {
  if (value == null) {
    return undefined;
  }
  return value === 0 ? undefined : value;
}

/**
 * @param {string} labelText
 * @param {HTMLElement} inputElement
 * @param {string} [extraClass]
 * @returns {HTMLLabelElement}
 */
export function buildField(labelText, inputElement, extraClass) {
  const wrapper = document.createElement("label");
  if (extraClass) {
    wrapper.classList.add(extraClass);
  }
  const labelSpan = document.createElement("span");
  labelSpan.className = "field-label";
  labelSpan.textContent = labelText;
  wrapper.appendChild(labelSpan);
  wrapper.appendChild(inputElement);
  return wrapper;
}

/**
 * @param {string} [name]
 * @returns {string}
 */
export function prettifyRunName(name = "") {
  return name
    .split("_")
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(" ");
}

/**
 * @param {string} label
 * @returns {HTMLButtonElement}
 */
export function createDragHandle(label) {
  const button = document.createElement("button");
  button.type = "button";
  button.className = "drag-handle";
  button.setAttribute("aria-label", label);
  button.tabIndex = -1;

  const svgNS = "http://www.w3.org/2000/svg";
  const svg = document.createElementNS(svgNS, "svg");
  svg.setAttribute("class", "drag-handle-icon");
  svg.setAttribute("width", "12");
  svg.setAttribute("height", "16");
  svg.setAttribute("viewBox", "0 0 12 16");
  svg.setAttribute("aria-hidden", "true");
  svg.setAttribute("focusable", "false");

  const path = document.createElementNS(svgNS, "path");
  path.setAttribute("fill", "currentColor");
  path.setAttribute(
    "d",
    "M3 2a1 1 0 1 1-2 0 1 1 0 0 1 2 0zM3 8a1 1 0 1 1-2 0 1 1 0 0 1 2 0zM3 14a1 1 0 1 1-2 0 1 1 0 0 1 2 0zM11 2a1 1 0 1 1-2 0 1 1 0 0 1 2 0zM11 8a1 1 0 1 1-2 0 1 1 0 0 1 2 0zM11 14a1 1 0 1 1-2 0 1 1 0 0 1 2 0z"
  );
  svg.appendChild(path);
  button.appendChild(svg);

  const srLabel = document.createElement("span");
  srLabel.className = "visually-hidden";
  srLabel.textContent = label;
  button.appendChild(srLabel);

  return button;
}

/**
 * @param {number} value
 * @returns {string}
 */
export function toRoman(value) {
  const numerals = ["I", "II", "III", "IV", "V", "VI", "VII", "VIII", "IX", "X"];
  return numerals[value - 1] || "";
}
