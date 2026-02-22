// @ts-check

import { SequenceEditorState } from "./SequenceEditorState.js";
import { SequenceDataAdapter } from "./SequenceDataAdapter.js";
import { SequenceApiClient } from "./SequenceApiClient.js";
import { SequenceEditorRenderer } from "./SequenceEditorRenderer.js";
import { SequenceEditorController } from "./SequenceEditorController.js";
import { SequenceEditorUI } from "./ui/SequenceEditorUI.js";

/** Bootstraps the leveling sequence editor once the document is ready. */
document.addEventListener("DOMContentLoaded", () => {
  const state = new SequenceEditorState();
  const dataAdapter = new SequenceDataAdapter(state);
  const renderer = new SequenceEditorRenderer(state, dataAdapter);
  const apiClient = new SequenceApiClient();
  const ui = new SequenceEditorUI();

  const controller = new SequenceEditorController({
    state,
    apiClient,
    dataAdapter,
    renderer,
    ui,
  });

  controller.initialize();
});
