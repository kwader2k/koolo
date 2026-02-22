// @ts-check

/** @typedef {import("../constants.js").DifficultyKey} DifficultyKey */
/** @typedef {import("../constants.js").RunSectionKey} RunSectionKey */
/** @typedef {{difficulty:DifficultyKey, step?:{type:string, section?:RunSectionKey}}} SectionRendererPayload */

/** Utility registry for section renderer callbacks. */
export class SectionRendererRegistry {
  constructor() {
    /** @type {Map<string, (payload:SectionRendererPayload) => void>} */
    this.registry = new Map();
  }

  /**
   * @param {string} type
   * @param {(payload: SectionRendererPayload) => void} handler
   */
  register(type, handler) {
    if (!type || typeof handler !== "function") {
      throw new Error("Section renderer registration requires a type and render function");
    }
    this.registry.set(type, handler);
  }

  /**
   * @param {string} type
   * @param {SectionRendererPayload} payload
   */
  render(type, payload) {
    const handler = this.registry.get(type);
    if (typeof handler !== "function") {
      return;
    }
    handler(payload);
  }
}
