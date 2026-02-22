// @ts-check

/**
 * @typedef {import("../SequenceEditorState.js").SequenceEditorState} SequenceEditorState
 */

/** Provides accessible drag + drop reordering for list rows. */
export class DragReorderManager {
  /**
   * @param {SequenceEditorState} state
   * @param {() => void} markDirty
   */
  constructor(state, markDirty) {
    this.state = state;
    this.markDirty = markDirty;
  }

  /**
   * @template T
   * @param {HTMLElement|null} container
   * @param {T[]} list
   * @param {() => void} [onReordered]
   */
  attach(container, list, onReordered) {
    if (!container) {
      return;
    }

    const rows = Array.from(container.querySelectorAll(".sequence-row"));
    rows.forEach((row) => {
      row.draggable = false;
    });

    const handles = rows.map((row) => row.querySelector(".drag-handle")).filter(Boolean);

    if (rows.length <= 1) {
      handles.forEach((handle) => {
        handle.disabled = true;
        handle.setAttribute("aria-disabled", "true");
      });
      this.teardown(container);
      return;
    }

    handles.forEach((handle) => {
      handle.disabled = false;
      handle.removeAttribute("aria-disabled");
    });

    const initialOrder = rows.map((row) => row.dataset.uid);
    let dragRow = null;

    const finalizeReorder = () => {
      if (!dragRow) {
        return;
      }

      const currentRows = Array.from(container.querySelectorAll(".sequence-row"));
      const newOrder = currentRows.map((row) => row.dataset.uid);
      const changed =
        newOrder.length === initialOrder.length && newOrder.some((uid, index) => uid !== initialOrder[index]);

      dragRow = null;

      if (!changed) {
        return;
      }

      const reorderedEntries = newOrder
        .map((uid) => list.find((entry) => String(this.state.ensureEntryUID(entry)) === uid))
        .filter(Boolean);

      if (reorderedEntries.length !== list.length) {
        return;
      }

      list.splice(0, list.length, ...reorderedEntries);
      this.markDirty();
      if (typeof onReordered === "function") {
        onReordered();
      }
    };

    const setDragReady = (row, ready) => {
      if (ready) {
        row.dataset.dragReady = "1";
        row.draggable = true;
      } else {
        row.dataset.dragReady = "";
        if (!row.classList.contains("dragging")) {
          row.draggable = false;
        }
      }
    };

    rows.forEach((row) => {
      const handle = row.querySelector(".drag-handle");
      if (!handle) {
        return;
      }

      handle.addEventListener("pointerdown", (event) => {
        if (handle.disabled) {
          return;
        }
        handle.focus();
        if (typeof handle.setPointerCapture === "function") {
          handle.setPointerCapture(event.pointerId);
        }
        setDragReady(row, true);
      });

      const cancelDragReady = () => {
        if (!row.classList.contains("dragging")) {
          setDragReady(row, false);
        }
      };

      const releasePointer = (event) => {
        if (typeof handle.releasePointerCapture === "function") {
          handle.releasePointerCapture(event.pointerId);
        }
        cancelDragReady();
      };

      handle.addEventListener("pointerup", releasePointer);
      handle.addEventListener("pointercancel", releasePointer);

      row.addEventListener("dragstart", (event) => {
        if (row.dataset.dragReady !== "1") {
          event.preventDefault();
          return;
        }
        const rect = row.getBoundingClientRect();
        const ghost = row.cloneNode(true);
        ghost.classList.add("drag-ghost");
        ghost.style.position = "absolute";
        ghost.style.top = "-1000px";
        ghost.style.left = "-1000px";
        ghost.style.width = `${rect.width}px`;
        ghost.style.pointerEvents = "none";
        document.body.appendChild(ghost);
        row.__dragGhost = ghost;
        dragRow = row;
        row.classList.add("dragging");
        if (event.dataTransfer) {
          event.dataTransfer.effectAllowed = "move";
          event.dataTransfer.setData("text/plain", row.dataset.uid || "");
          const offsetX = event.clientX ? event.clientX - rect.left : rect.width / 2;
          const offsetY = event.clientY ? event.clientY - rect.top : rect.height / 2;
          try {
            event.dataTransfer.setDragImage(ghost, offsetX, offsetY);
          } catch (_error) {
            // Ignore setDragImage failures
          }
        }
      });

      row.addEventListener("dragend", () => {
        row.classList.remove("dragging");
        setDragReady(row, false);
        if (row.__dragGhost) {
          document.body.removeChild(row.__dragGhost);
          row.__dragGhost = null;
        }
        finalizeReorder();
      });

      row.addEventListener("dragover", (event) => {
        if (!dragRow || dragRow === row) {
          return;
        }
        event.preventDefault();
        const rect = row.getBoundingClientRect();
        const offset = event.clientY - rect.top;
        const shouldInsertAfter = offset > rect.height / 2;
        const parent = row.parentNode;
        if (!parent) {
          return;
        }
        const referenceNode = shouldInsertAfter ? row.nextSibling : row;
        if (referenceNode !== dragRow) {
          parent.insertBefore(dragRow, referenceNode);
        }
      });

      row.addEventListener("drop", (event) => {
        if (!dragRow) {
          return;
        }
        event.preventDefault();
      });
    });

    const onContainerDragOver = (event) => {
      if (!dragRow) {
        return;
      }
      const targetRow = event.target.closest(".sequence-row");
      if (targetRow) {
        return;
      }
      event.preventDefault();
      container.appendChild(dragRow);
    };

    const onContainerDrop = (event) => {
      if (!dragRow) {
        return;
      }
      event.preventDefault();
    };

    this.teardown(container);
    container.__dragOverHandler = onContainerDragOver;
    container.__dropHandler = onContainerDrop;

    container.addEventListener("dragover", onContainerDragOver);
    container.addEventListener("drop", onContainerDrop);
  }

  /**
   * @param {HTMLElement|null} container
   */
  teardown(container) {
    if (!container) {
      return;
    }
    if (container.__dragOverHandler) {
      container.removeEventListener("dragover", container.__dragOverHandler);
      delete container.__dragOverHandler;
    }
    if (container.__dropHandler) {
      container.removeEventListener("drop", container.__dropHandler);
      delete container.__dropHandler;
    }
  }
}
