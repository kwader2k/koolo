package run

import (
	stdCtx "context"
	"fmt"
	"os"
	"strings"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	drop "github.com/hectorgimenez/koolo/internal/drop"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

type Drop struct {
}

func NewDrop() Drop {
	return Drop{}
}

func (d Drop) Name() string {
	return "Drop"
}

func (d Drop) CheckConditions(_ *RunParameters) SequencerResult {
	return SequencerOk
}

func (d Drop) Run(_ *RunParameters) error {
	ctx := context.Get()
	processedDrop := false
	for {
		if ctx.Drop == nil || ctx.Drop.Pending() == nil {
			if !processedDrop {
				ctx.Logger.Warn("Drop.Run called but no pending request; skipping")
			} else {
				d.finalizeDropSession(ctx)
			}
			return nil
		}

		if err := d.runSingle(ctx, ctx.Drop.Pending()); err != nil {
			return err
		}
		processedDrop = true

		if ctx.Drop == nil || ctx.Drop.Pending() == nil {
			d.finalizeDropSession(ctx)
			return nil
		}

		ctx.Logger.Info("Drop: additional pending request detected; starting next Drop")
		utils.PingSleep(utils.Medium, 100)
	}
}

func (d Drop) runSingle(ctx *context.Status, req *drop.Request) error {
	if ctx == nil || req == nil {
		return nil
	}

	// Always apply request filters so disabled filters clear any previous state.
	ctx.Drop.UpdateFilters(req.Filters)
	if req.Filters.Enabled {
		ctx.Logger.Debug("Drop: Applied filters from request",
			"room", req.RoomName,
			"filterMode", req.Filters.DropperOnlySelected)
	}

	ctx.Drop.SetActive(req)
	ctx.Logger.Debug("Drop: Set active request", "room", req.RoomName)
	ctx.Drop.ResetDropperedItemCounts()
	runCompleted := false
	startTime := time.Now()
	itemsDroppered := 0
	var DropError error

	defer func() {
		duration := time.Since(startTime)

		if runCompleted {
			ctx.Drop.ClearRequest(req)
			ctx.Drop.ReportResult(req.RoomName, "Success", itemsDroppered, duration, "", req.Filters)
			return
		}

		ctx.Drop.ClearRequest(req)

		if ctx.Manager.InGame() {
			ctx.Logger.Warn("Drop failed while in game, exiting game")
			ctx.Manager.ExitGame()
			utils.Sleep(500)
		}

		if err := d.ensureCharacterSelection(ctx); err != nil {
			ctx.Logger.Error("Drop: Failed to return to character selection, will kill client", "error", err)

			if ctx.GameReader != nil && ctx.GameReader.Process != nil {
				pid := ctx.GameReader.Process.GetPID()
				if process, findErr := os.FindProcess(int(pid)); findErr == nil {
					if killErr := process.Kill(); killErr != nil {
						ctx.Logger.Error("Drop: Failed to kill client", "error", killErr)
					} else {
						ctx.Logger.Debug("Drop: Successfully killed client", "pid", pid)
					}
				}
			}
		}

		errorMsg := "Unknown error"
		if DropError != nil {
			errorMsg = DropError.Error()
		}
		ctx.Drop.ReportResult(req.RoomName, "Failed", itemsDroppered, duration, errorMsg, req.Filters)
	}()
	utils.PingSleep(utils.Medium, 100)

	if ctx.Manager.InGame() {
		ctx.Manager.ExitGame()
		utils.Sleep(500)
	}

	if err := d.prepareForLobbyJoin(ctx); err != nil {
		ctx.Logger.Error("Drop: failed to prepare lobby join", "error", err)
		DropError = err
		return err
	}

	// Try to join the game with 1 retry on failure
	joinErr := ctx.Manager.JoinOnlineGame(req.RoomName, req.Password)
	if joinErr != nil {
		ctx.Logger.Error("Drop: failed to join Drop game", "error", joinErr)
		DropError = joinErr
		return joinErr
	}

	bgCtx, cancel := stdCtx.WithCancel(stdCtx.Background())
	defer cancel()

	// [Local Data Refresh]
	// When a Drop request is triggered, the interrupt causes the main bot's background routines
	// to stop functioning. Therefore, we must run a local goroutine here to keep the
	// Game Data up-to-date during the Drop process.
	// This goroutine is automatically terminated via defer when Drop finishes.

	go func() {
		ticker := time.NewTicker(100 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-bgCtx.Done():
				return
			case <-ticker.C:
				if ctx.Manager.InGame() {
					ctx.RefreshGameData()
				}
			}
		}
	}()

	ctx.WaitForGameToLoad()
	action.SwitchToLegacyMode()

	if err := ctx.GameReader.FetchMapData(); err != nil {
		ctx.Logger.Error("Drop: failed to fetch map data", "error", err)
		return err
	}
	ctx.DisableItemPickup()

	if currentRoom := ctx.Data.Game.LastGameName; currentRoom != "" {
		if currentRoom != req.RoomName {
			ctx.Logger.Warn("Drop: joined room differs from requested", "requested", req.RoomName, "joined", currentRoom)
		} else {
			ctx.Logger.Debug("Drop: confirmed joined room matches request", "room", currentRoom)
		}
	}

	if err := action.RunDropCleanup(); err != nil {
		ctx.Logger.Warn("Drop cleanup warning (continuing anyway)", "error", err)
	}

	itemsDroppered, dropErr := d.dropStashItems(ctx)
	ctx.EnableItemPickup()
	if dropErr != nil {
		ctx.Logger.Error("Drop: stash drop sequence failed", "error", dropErr)
		DropError = dropErr
		return dropErr
	}
	ctx.Logger.Info("Drop: finished stash drop sequence", "itemsDroppered", itemsDroppered)

	runCompleted = true

	ctx.Manager.ExitGame()
	utils.Sleep(500)
	ctx.RefreshGameData()
	return nil
}

func (d Drop) finalizeDropSession(ctx *context.Status) {
	if ctx == nil {
		return
	}
	if err := d.ensureCharacterSelection(ctx); err != nil {
		ctx.Logger.Warn("Drop: failed to reach character selection screen", "error", err)
	}
	if err := d.ensureDropCharacterSelected(ctx); err != nil {
		ctx.Logger.Error("Drop: failed to validate character selection at finish", "error", err)
	} else {
		ctx.Logger.Info("Drop: Character selection verified. Ready for next run.")
	}
}

func (d Drop) ensureCharacterSelection(ctx *context.Status) error {
	// Keep a longer timeout than regular menu polling because Drop can be triggered
	// while client state is still transitioning (splash/login/loading overlays).
	const maxRetries = 120

	for i := 0; i < maxRetries; i++ {
		ctx.RefreshGameData()

		if ctx.GameReader.IsInCharacterSelectionScreen() {
			ctx.Logger.Debug("Drop: Successfully reached character selection screen")
			return nil
		}

		if ctx.GameReader.IsIngame() {
			ctx.Logger.Debug("Drop: Detected In-Game state, exiting to menu...")
			utils.Sleep(500)
			ctx.Manager.ExitGame()
			continue
		}

		if ctx.GameReader.IsInLobby() {
			ctx.Logger.Debug("Drop: Detected Lobby state, pressing ESC to return...")
			utils.Sleep(1000)
			ctx.HID.PressKey(win.VK_ESCAPE)
			continue
		}

		if ctx.GameReader.IsInCharacterCreationScreen() {
			ctx.Logger.Debug("Drop: Detected Character Creation screen, cancelling...")
			utils.Sleep(500)
			ctx.HID.PressKey(win.VK_ESCAPE)
			continue
		}

		// Match normal startup flow: click through splash/login states
		// (e.g. "Press Any Key") to reach character selection.
		ctx.HID.Click(game.LeftButton, 100, 100)

		attempt := i + 1
		if attempt <= 3 || attempt%10 == 0 || attempt == maxRetries {
			ctx.Logger.Debug("Drop: waiting for known state while reaching character selection", "attempt", attempt, "maxAttempts", maxRetries)
		}
		utils.Sleep(500)
	}
	return fmt.Errorf("Drop: failed to reach character selection screen after %d attempts", maxRetries)
}

func (d Drop) ensureInventoryOpen(ctx *context.Status) error {
	const maxAttempts = 4

	for attempt := 1; attempt <= maxAttempts; attempt++ {
		ctx.RefreshGameData()
		if ctx.Data.OpenMenus.Inventory {
			return nil
		}

		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.PingSleep(utils.Medium, 100)
	}

	ctx.RefreshGameData()
	if ctx.Data.OpenMenus.Inventory {
		return nil
	}

	return fmt.Errorf("Drop: inventory UI not open before dropping items")
}

func (d Drop) dropStashItems(ctx *context.Status) (int, error) {

	quotaTracker := newDropQuotaTracker(ctx)
	totalItemsDroppered := 0
	const (
		maxPasses      = 1
		maxItemRetries = 2
		maxTotalTime   = 3 * time.Minute
	)
	// Build stash tabs array dynamically based on SharedStashPages
	// Non-DLC: personal (1) + 3 shared (2-4) = [1,2,3,4]
	// DLC: personal (1) + 5 shared (2-6) = [1,2,3,4,5,6]
	sharedPages := ctx.Data.Inventory.SharedStashPages
	if sharedPages == 0 {
		sharedPages = 3 // Fallback
	}
	stashTabs := make([]int, 1+sharedPages)
	stashTabs[0] = 1 // Personal tab
	for i := 0; i < sharedPages; i++ {
		stashTabs[i+1] = i + 2 // Shared tabs start at 2
	}

	startTime := time.Now()
	for pass := 0; pass < maxPasses; pass++ {
		if time.Since(startTime) > maxTotalTime {
			ctx.Logger.Warn("Drop: timeout reached after processing", "elapsed", time.Since(startTime))
			break
		}
		ctx.Logger.Debug("Drop: stash pass", "pass", pass+1)
		movedItems := false

		for _, tab := range stashTabs {
			ctx.Logger.Debug("Drop: preparing stash tab before processing", "tab", tab)
			if err := d.ensureStashTabReady(ctx, tab); err != nil {
				ctx.Logger.Error("Drop: failed to prepare stash tab", "tab", tab, "error", err)
				continue
			}
			ctx.Logger.Debug("Drop: stash tab ready", "tab", tab, "stashOpen", ctx.Data.OpenMenus.Stash)

			Dropperables := d.collectDropperablesForTab(ctx, tab, quotaTracker)
			if len(Dropperables) == 0 {
				ctx.Logger.Debug("Drop: no Dropperables on tab", "tab", tab)
				continue
			}

			ctx.Logger.Debug("Drop: moving Dropperables from tab", "tab", tab, "count", len(Dropperables))

			queue := append([]data.Item(nil), Dropperables...)
			attempts := make(map[data.UnitID]int, len(Dropperables))

			// Refresh once before processing all items (Mule.go pattern)
			ctx.RefreshGameData()

			for len(queue) > 0 {
				it := queue[0]
				queue = queue[1:]

				_, found := findInventorySpace(ctx, it)
				if !found {
					dropped, err := d.dropInventoryDropperables(ctx, tab, quotaTracker)
					if err != nil {
						return totalItemsDroppered, err
					}
					totalItemsDroppered += dropped
					if d.DropRequestSatisfied(quotaTracker) {
						ctx.Logger.Debug("Drop: requested quotas satisfied while freeing inventory space")
						return totalItemsDroppered, nil
					}

					if _, found = findInventorySpace(ctx, it); !found {
						attempts[it.UnitID]++
						if attempts[it.UnitID] < maxItemRetries {
							queue = append(queue, it)
							ctx.Logger.Debug("Drop: re-queueing item due to lack of space", "item", it.Name, "attempt", attempts[it.UnitID])
							continue
						}

						ctx.Logger.Warn("Drop: inventory still full after drop, skipping item", "item", it.Name)
						if quotaTracker != nil {
							quotaTracker.release(string(it.Name))
						}
						continue
					}
				}

				if _, ok := d.moveStashItemToInventory(ctx, it); ok {
					movedItems = true
				} else {
					attempts[it.UnitID]++
					if attempts[it.UnitID] < maxItemRetries {
						queue = append(queue, it)
						ctx.Logger.Debug("Drop: re-queueing item after failed move", "item", it.Name, "attempt", attempts[it.UnitID])
						continue
					}

					ctx.Logger.Warn("Drop: unable to move item from stash", "item", it.Name)
					if quotaTracker != nil {
						quotaTracker.release(string(it.Name))
					}
				}
			}
			// Refresh after queue completes (Mule.go pattern)
			ctx.RefreshGameData()

			dropped, err := d.dropInventoryDropperables(ctx, tab, quotaTracker)
			if err != nil {
				return totalItemsDroppered, err
			}
			totalItemsDroppered += dropped
			if d.DropRequestSatisfied(quotaTracker) {
				ctx.Logger.Debug("Drop: requested quotas satisfied after dropping items", "tab", tab)
				return totalItemsDroppered, nil
			}
			if err := step.OpenInventory(); err != nil {
				return totalItemsDroppered, fmt.Errorf("Drop: failed to reopen inventory after dropping items: %w", err)
			}
		}
		if !movedItems {
			ctx.Logger.Debug("Drop: no more stash items to Dropper")
			return totalItemsDroppered, nil
		}
	}

	// If there are no finite quotas, treat completion of passes as success.
	if quotaTracker == nil || !quotaTracker.hasFiniteLimits {
		ctx.Logger.Info("Drop: stash passes complete (no finite quotas); treating as success")
		return totalItemsDroppered, nil
	}

	return totalItemsDroppered, fmt.Errorf("Drop: reached max stash passes without emptying items")
}

func (d Drop) moveStashItemToInventory(ctx *context.Status, it data.Item) (data.Item, bool) {
	updated := it
	for _, candidate := range ctx.Data.Inventory.AllItems {
		if candidate.UnitID == it.UnitID {
			updated = candidate
			break
		}
	}

	screenPos := ui.GetScreenCoordsForItem(updated)
	ctx.Logger.Debug("Drop: attempting to move item via ctrl+click", "item", updated.Name, "tab", updated.Location.Page+1, "locationType", updated.Location.LocationType, "gridX", updated.Position.X, "gridY", updated.Position.Y, "screenX", screenPos.X, "screenY", screenPos.Y)
	prevInventoryCount := len(ctx.Data.Inventory.ByLocation(item.LocationInventory))
	ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
	utils.PingSleep(utils.Medium, 100)
	ctx.RefreshInventory()

	for _, invItem := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if invItem.UnitID == it.UnitID {
			ctx.Logger.Debug("Drop: item detected in inventory after move", "item", it.Name)
			logDropQuotaProgress(ctx, "moved-to-inventory", invItem)
			return invItem, true
		}
	}

	newInventoryCount := len(ctx.Data.Inventory.ByLocation(item.LocationInventory))
	if newInventoryCount > prevInventoryCount {
		ctx.Logger.Debug("Drop: inventory count increased, assuming item moved", "item", it.Name, "beforeCount", prevInventoryCount, "afterCount", newInventoryCount)
		logDropQuotaProgress(ctx, "moved-to-inventory", it)
		return it, true
	}

	for _, stashItem := range ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash) {
		if stashItem.UnitID == it.UnitID {
			ctx.Logger.Debug("Drop: item still present in stash after move attempt", "item", it.Name)
			return it, false
		}
	}

	ctx.Logger.Debug("Drop: unable to determine new location of item after move attempt", "item", updated.Name)
	return updated, false
}

func (d Drop) dropInventoryDropperables(ctx *context.Status, reopenTab int, quotas *DropQuotaTracker) (int, error) {
	ctx.RefreshInventory()

	stashWasOpen := ctx.Data.OpenMenus.Stash
	if stashWasOpen {
		if err := action.CloseStash(); err != nil {
			ctx.Logger.Error("Drop: failed to close stash before dropping items", "error", err)
			return 0, err
		}
		utils.PingSleep(utils.Medium, 100)
	}

	if err := d.ensureInventoryOpen(ctx); err != nil {
		return 0, err
	}

	invItems := ctx.Data.Inventory.ByLocation(item.LocationInventory)
	dropped := 0

	for _, it := range invItems {
		if action.IsInLockedInventorySlot(it) {
			ctx.Logger.Debug("Drop: skipping locked inventory slot", "item", it.Name, "x", it.Position.X, "y", it.Position.Y)
			continue
		}

		if action.IsDropProtected(it) {
			continue
		}

		screenPos := ui.GetScreenCoordsForItem(it)
		ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
		utils.PingSleep(utils.Medium, 100)
		dropped++
		// Count this item towards Drop quotas
		if ctx.Drop != nil {
			ctx.Drop.RecordDropperedItem(string(it.Name))
		}
		logDropQuotaProgress(ctx, "dropped-from-inventory", it)
		if quotas != nil {
			quotas.markDroppered(string(it.Name))
		}
	}

	if dropped > 0 {
		ctx.Logger.Debug("Drop: dropped inventory items", "count", dropped)
	}
	ctx.RefreshGameData()

	if ctx.Data.OpenMenus.Inventory {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.PingSleep(utils.Medium, 100)
	}

	if reopenTab > 0 || stashWasOpen {
		if !ctx.Data.OpenMenus.Stash {
			if err := d.ensureStashOpen(ctx); err != nil {
				ctx.Logger.Error("Drop: failed to reopen stash after dropping items", "error", err)
				return dropped, err
			}
		}
		if reopenTab > 0 {
			if err := d.ensureStashTabReady(ctx, reopenTab); err != nil {
				ctx.Logger.Error("Drop: failed to prepare stash tab after reopening", "tab", reopenTab, "error", err)
				return dropped, err
			}
		}
	}

	return dropped, nil
}

func (d Drop) collectDropperablesForTab(ctx *context.Status, tab int, quotas *DropQuotaTracker) []data.Item {
	stashItems := ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash)
	Dropperables := make([]data.Item, 0, len(stashItems))

	for _, it := range stashItems {
		if action.IsDropProtected(it) {
			continue
		}

		if d.itemBelongsToTab(it, tab) {
			itemName := string(it.Name)
			if quotas != nil && !quotas.reserve(itemName) {
				continue
			}
			Dropperables = append(Dropperables, it)
		}
	}

	return Dropperables
}

func (d Drop) itemBelongsToTab(it data.Item, tab int) bool {
	switch it.Location.LocationType {
	case item.LocationStash:
		return tab == 1
	case item.LocationSharedStash:
		return tab == it.Location.Page+1
	default:
		return false
	}
}

type DropQuotaTracker struct {
	ctx             *context.Status
	reserved        map[string]int
	hasFiniteLimits bool
}

func newDropQuotaTracker(ctx *context.Status) *DropQuotaTracker {
	if ctx == nil || ctx.Context == nil {
		return nil
	}
	return &DropQuotaTracker{
		ctx:             ctx,
		reserved:        make(map[string]int),
		hasFiniteLimits: ctx.Drop != nil && ctx.Drop.HasDropQuotaLimits(),
	}
}

func (t *DropQuotaTracker) reserve(name string) bool {
	if t == nil {
		return true
	}
	if t.ctx.Drop == nil {
		return true
	}
	limit := t.ctx.Drop.GetDropItemQuantity(name)
	if limit <= 0 {
		return true
	}
	key := strings.ToLower(name)
	Droppered := t.ctx.Drop.GetDropperedItemCount(name)
	if Droppered+t.reserved[key] >= limit {
		return false
	}
	t.reserved[key]++
	return true
}

func (t *DropQuotaTracker) release(name string) {
	if t == nil {
		return
	}
	key := strings.ToLower(name)
	if t.reserved[key] > 0 {
		t.reserved[key]--
	}
}

func (t *DropQuotaTracker) markDroppered(name string) {
	t.release(name)
}

func (t *DropQuotaTracker) fulfilled() bool {
	if t == nil || !t.hasFiniteLimits {
		return false
	}
	if t.ctx.Drop == nil {
		return false
	}
	return t.ctx.Drop.AreDropQuotasSatisfied()
}

func (d Drop) DropRequestSatisfied(quotas *DropQuotaTracker) bool {
	return quotas != nil && quotas.fulfilled()
}

func logDropQuotaProgress(ctx *context.Status, stage string, it data.Item) {
	if ctx == nil || ctx.Drop == nil {
		return
	}
	limit := ctx.Drop.GetDropItemQuantity(string(it.Name))
	Droppered := ctx.Drop.GetDropperedItemCount(string(it.Name))
	remaining := limit - Droppered
	if remaining < 0 {
		remaining = 0
	}
	ctx.Logger.Debug("Drop: quota checkpoint", "stage", stage, "item", string(it.Name), "Droppered", Droppered, "limit", limit, "remaining", remaining)
}

func (d Drop) prepareForLobbyJoin(ctx *context.Status) error {
	if ctx.GameReader.IsInLobby() {
		return nil
	}

	if err := d.ensureCharacterSelection(ctx); err != nil {
		return fmt.Errorf("Drop: failed to reach character selection before lobby join: %w", err)
	}

	if err := d.ensureOnlineForDrop(ctx); err != nil {
		return err
	}

	if err := d.ensureDropCharacterSelected(ctx); err != nil {
		ctx.Logger.Error("Drop: CRITICAL - Wrong character selected, stopping run!", "error", err)
		return err
	}
	return d.enterLobbyForDrop(ctx)
}

func (d Drop) refreshCharacterList(ctx *context.Status) error {
	const (
		CharCreateBtnX = 1125
		CharCreateBtnY = 640
	)
	ctx.Logger.Debug("Drop: Simple Refresh - Clicking Create Character...")

	ctx.HID.Click(game.LeftButton, CharCreateBtnX, CharCreateBtnY)

	utils.PingSleep(utils.Critical, 500)

	ctx.HID.PressKey(win.VK_ESCAPE)

	utils.PingSleep(utils.Critical, 500)

	return nil
}

func (d Drop) verifyCharacterListUI(ctx *context.Status, panel data.Panel, characterList []string, listRefreshed *bool) bool {

	isListEmpty := panel.NumChildren == 0 || len(characterList) == 0

	if !isListEmpty {
		return true
	}

	if *listRefreshed {
		ctx.Logger.Debug("Drop: Single character verified via empty panel detected.")
		return true
	}

	ctx.Logger.Info("Drop: Selection UI appears blank, forcing refresh...")
	d.refreshCharacterList(ctx)
	*listRefreshed = true

	return false
}

func (d Drop) ensureDropCharacterSelected(ctx *context.Status) error {
	target := ctx.CharacterCfg.CharacterName
	if target == "" {
		return nil
	}

	// Track refresh status for single-character accounts
	listRefreshed := false

	for i := 0; i < 25; i++ {
		ctx.RefreshGameData()
		current := ctx.GameReader.GetSelectedCharacterName()

		// Fetch UI Data
		panel := ctx.GameReader.GetPanel("CharacterSelectPanel")
		characterList := ctx.GameReader.GetCharacterList()

		// (Optional) Logging can be simplified or moved to debug level if too verbose
		ctx.Logger.Debug("Drop: checking character selection",
			"iteration", i+1,
			"current", current,
			"target", target,
			"listCount", len(characterList))

		if strings.EqualFold(current, target) {
			// Use Helper Function to validate UI integrity
			if d.verifyCharacterListUI(ctx, panel, characterList, &listRefreshed) {
				ctx.Logger.Debug("Drop: Character selection confirmed.", "name", current)
				return nil
			}
			// If verification failed (refresh triggered), continue loop to re-check
			continue
		}

		utils.PingSleep(utils.Light, 250)
	}

	return fmt.Errorf("Drop: character %s not highlighted", target)
}

func (d Drop) ensureOnlineForDrop(ctx *context.Status) error {
	if ctx.CharacterCfg.AuthMethod == "None" || ctx.GameReader.IsOnline() {
		return nil
	}
	const maxRetries = 5
	for attempt := 0; attempt < maxRetries; attempt++ {
		ctx.Logger.Debug("Drop: attempting to connect to battle.net", "attempt", attempt+1)

		ctx.HID.Click(game.LeftButton, 1090, 32)
		utils.PingSleep(utils.Critical, 500)

		ctx.RefreshGameData()

		blocking := ctx.GameReader.GetPanel("BlockingPanel")
		modal := ctx.GameReader.GetPanel("DismissableModal")

		if blocking.PanelName != "" && blocking.PanelEnabled && blocking.PanelVisible {
			utils.PingSleep(utils.Critical, 1000)
			ctx.RefreshGameData()
		}
		modal = ctx.GameReader.GetPanel("DismissableModal")
		if modal.PanelName != "" && modal.PanelEnabled && modal.PanelVisible {
			ctx.HID.PressKey(0x1B) // ESC
			utils.PingSleep(utils.Medium, 300)
			continue
		}
		if ctx.GameReader.IsOnline() {
			return nil
		}
		utils.RetrySleep(attempt+1, 1.0, 500)
	}
	return fmt.Errorf("Drop: failed to connect to battle.net")
}

func (d Drop) enterLobbyForDrop(ctx *context.Status) error {
	const maxAttempts = 5

	for attempt := 0; attempt < maxAttempts; attempt++ {
		ctx.RefreshGameData()

		if ctx.GameReader.IsInLobby() {
			return nil
		}

		if !ctx.GameReader.IsInCharacterSelectionScreen() {
			ctx.Logger.Debug("Drop: Not in character selection screen, waiting...", "attempt", attempt+1)

			if err := d.ensureCharacterSelection(ctx); err != nil {
				return err
			}
			continue
		}

		ctx.Logger.Debug("Drop: Clicking 'Play' to enter lobby", "attempt", attempt+1)
		ctx.HID.Click(game.LeftButton, 744, 650)

		timeout := time.After(2 * time.Second)
		checkTicker := time.NewTicker(100 * time.Millisecond)

		entered := false
		for !entered {
			select {
			case <-timeout:
				entered = true
			case <-checkTicker.C:
				ctx.RefreshGameData()
				if ctx.GameReader.IsInLobby() {
					checkTicker.Stop()
					return nil
				}
			}
		}
		checkTicker.Stop()
	}
	return fmt.Errorf("Drop: failed to enter lobby")
}

func (d Drop) ensureStashOpen(ctx *context.Status) error {
	const maxAttempts = 5

	if ctx.Data.AreaData.Grid == nil {
		ctx.Logger.Info("Drop: map data missing before stash open, fetching...")
		if err := ctx.GameReader.FetchMapData(); err != nil {
			return fmt.Errorf("Drop: failed to fetch map data before stash open: %w", err)
		}
		ctx.RefreshGameData()
		if ctx.Data.AreaData.Grid == nil {
			return fmt.Errorf("Drop: map data unavailable for stash interactions")
		}
	}

	for attempt := 1; attempt <= maxAttempts; attempt++ {
		ctx.RefreshGameData()
		if ctx.Data.OpenMenus.Stash {
			return nil
		}

		if attempt > 1 {

			utils.Sleep(5000) // Wait 5 seconds if unable to interact with the stash due to congestion
			ctx.Logger.Debug("Drop: stash not open, waiting...", "attempt", attempt)

			if attempt >= 4 {
				if err := d.repositionNearStash(ctx); err != nil {
					ctx.Logger.Warn("Drop: failed to reposition before opening stash", "error", err)
				}
			}
			if ctx.Data.OpenMenus.Stash {
				return nil
			}
		}

		ctx.Logger.Debug("Drop: opening stash", "attempt", attempt)
		if err := action.OpenStash(); err != nil {
			ctx.Logger.Error("Drop: failed to open stash", "attempt", attempt, "error", err)
		}

		utils.PingSleep(utils.Medium, 100)
		ctx.RefreshGameData()
		if ctx.Data.OpenMenus.Stash {
			return nil
		}

	}

	ctx.RefreshGameData()
	if ctx.Data.OpenMenus.Stash {
		return nil
	}

	return fmt.Errorf("Drop: stash UI not open after initial setup")
}
func (d Drop) repositionNearStash(ctx *context.Status) error {

	bank, found := ctx.Data.Objects.FindOne(object.Bank)
	if !found {
		return fmt.Errorf("Drop: stash object not found in area %v", ctx.Data.PlayerUnit.Area)
	}
	if ctx.Data.AreaData.Grid == nil {
		ctx.Logger.Debug("Drop: skipping stash reposition; map data unavailable")
		return nil
	}
	ctx.Logger.Debug("Drop: moving closer to stash before reopening", "x", bank.Position.X, "y", bank.Position.Y)
	if err := action.MoveToCoords(bank.Position, step.WithDistanceToFinish(6)); err != nil {
		return fmt.Errorf("Drop: failed to reposition near stash: %w", err)
	}
	utils.PingSleep(utils.Medium, 100)
	return nil
}

func (d Drop) ensureStashTabReady(ctx *context.Status, tab int) error {
	if !ctx.Data.OpenMenus.Stash {
		ctx.Logger.Debug("Drop: stash closed before ensuring tab, reopening", "tab", tab)
		if err := d.ensureStashOpen(ctx); err != nil {
			return err
		}
		utils.PingSleep(utils.Medium, 100)
	}

	action.SwitchStashTab(tab)
	utils.PingSleep(utils.Light, 50)
	ctx.RefreshGameData()

	ctx.Logger.Debug("Drop: switched stash tab", "tab", tab, "inventoryItems", len(ctx.Data.Inventory.ByLocation(item.LocationInventory)))

	if !ctx.Data.OpenMenus.Stash {
		return fmt.Errorf("Drop: stash UI not open after switching tabs")
	}
	ctx.Logger.Debug("Drop: stash tab ready", "tab", tab)
	return nil
}
