package run

import (
	"fmt"
	"strings"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

type Delivery struct {
}

func NewDelivery() Delivery {
	return Delivery{}
}

func (d Delivery) Name() string {
	return "Delivery"
}

func (d Delivery) CheckConditions(_ *RunParameters) SequencerResult {
	return SequencerOk
}

func (d Delivery) Run(_ *RunParameters) error {
	ctx := context.Get()

	if ctx.Delivery == nil || ctx.Delivery.Pending() == nil {
		ctx.Logger.Warn("Delivery.Run called but no pending request; skipping")
		return nil
	}

	req := ctx.Delivery.Pending()
	ctx.Delivery.SetActive(req)
	ctx.Delivery.ResetDeliveredItemCounts()
	runCompleted := false
	startTime := time.Now()
	itemsDelivered := 0
	var deliveryError error

	defer func() {
		duration := time.Since(startTime)

		if runCompleted {
			ctx.Delivery.ClearRequest(req)
			// Report success to server
			ctx.Delivery.ReportResult(req.RoomName, "Success", itemsDelivered, duration, "")
			return
		}
		ctx.Delivery.ClearRequest(req)

		if ctx.Manager.InGame() {
			ctx.Logger.Info("Delivery failed while in game, exiting game")
			ctx.Manager.ExitGame()
			utils.Sleep(1000)
		}

		if err := d.ensureCharacterSelection(ctx); err != nil {
			ctx.Logger.Warn("Delivery: failed to return to character selection after failure", "error", err)
		}

		// Report failure to server
		errorMsg := "Unknown error"
		if deliveryError != nil {
			errorMsg = deliveryError.Error()
		}
		ctx.Delivery.ReportResult(req.RoomName, "Failed", itemsDelivered, duration, errorMsg)
	}()

	utils.Sleep(100)

	if ctx.Manager.InGame() {
		ctx.Manager.ExitGame()
		utils.Sleep(1000)
	}

	if err := d.prepareForLobbyJoin(ctx); err != nil {
		ctx.Logger.Error("Delivery: failed to prepare lobby join", "error", err)
		deliveryError = err
		return err
	}

	// Try to join the game with 1 retry on failure
	joinErr := ctx.Manager.JoinOnlineGame(req.RoomName, req.Password)
	if joinErr != nil {
		ctx.Logger.Error("Delivery: failed to join delivery game", "error", joinErr)
		deliveryError = joinErr
		return joinErr
	}

	ctx.WaitForGameToLoad()
	ctx.RefreshGameData()
	// Ensure legacy graphics are toggled before interacting with stash/inventory.
	action.SwitchToLegacyMode()
	action.SwitchToLegacyMode()

	if err := ctx.GameReader.FetchMapData(); err != nil {
		ctx.Logger.Error("Delivery: failed to fetch map data", "error", err)
		return err
	}
	ctx.DisableItemPickup()
	ctx.RefreshGameData()

	if err := d.initialSetupInGame(ctx); err != nil {
		ctx.Logger.Error("Delivery: initial setup failed", "error", err)
		return err
	}

	itemsDelivered, dropErr := d.dropStashItems(ctx)
	ctx.EnableItemPickup()
	if dropErr != nil {
		ctx.Logger.Error("Delivery: stash drop sequence failed", "error", dropErr)
		deliveryError = dropErr
		return dropErr
	}
	ctx.Logger.Info("Delivery: finished stash drop sequence", "itemsDelivered", itemsDelivered)

	ctx.Manager.ExitGame()
	utils.Sleep(2000)
	if err := d.ensureCharacterSelection(ctx); err != nil {
		ctx.Logger.Warn("Delivery: failed to return to character selection", "error", err)
	}

	runCompleted = true
	return nil
}

func (d Delivery) ensureCharacterSelection(ctx *context.Status) error {
	ctx.RefreshGameData()
	const maxAttempts = 6

	for attempt := 0; attempt < maxAttempts; attempt++ {
		if ctx.GameReader.IsInCharacterSelectionScreen() {
			return nil
		}
		if ctx.GameReader.IsInLobby() {
			ctx.Logger.Debug("Delivery: still in lobby after exit, sending ESC to return to character screen", "attempt", attempt+1)
			ctx.HID.PressKey(win.VK_ESCAPE)
			utils.Sleep(1000) // Increased from 500ms to 1000ms for character loading
			ctx.RefreshGameData()
			continue
		}
		utils.Sleep(500)
		ctx.RefreshGameData()
	}

	if ctx.GameReader.IsInCharacterSelectionScreen() {
		return nil
	}
	return fmt.Errorf("delivery: unable to reach character selection after exiting game")
}

func (d Delivery) ensureInventoryOpen(ctx *context.Status) error {
	const maxAttempts = 4

	for attempt := 1; attempt <= maxAttempts; attempt++ {
		ctx.RefreshGameData()
		if ctx.Data.OpenMenus.Inventory {
			return nil
		}

		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.Sleep(250)
	}

	ctx.RefreshGameData()
	if ctx.Data.OpenMenus.Inventory {
		return nil
	}

	return fmt.Errorf("delivery: inventory UI not open before dropping items")
}

func (d Delivery) initialSetupInGame(ctx *context.Status) error {
	ctx.RefreshGameData()

	if !ctx.Data.PlayerUnit.Area.IsTown() {
		ctx.Logger.Error("Delivery: not in town at initial setup", "area", ctx.Data.PlayerUnit.Area)
		return fmt.Errorf("delivery: can only run in town")
	}

	if err := d.ensureStashOpen(ctx); err != nil {
		return err
	}
	utils.Sleep(500)
	ctx.RefreshGameData()

	if !ctx.Data.OpenMenus.Inventory {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.Sleep(500)
	}
	ctx.RefreshGameData()

	if !ctx.Data.OpenMenus.Stash {
		ctx.Logger.Error("Delivery: stash UI not open after initial setup", "area", ctx.Data.PlayerUnit.Area)
		return fmt.Errorf("delivery: stash UI not open after initial setup")
	}

	return nil
}

func (d Delivery) dropStashItems(ctx *context.Status) (int, error) {
	if err := d.ensureStashOpen(ctx); err != nil {
		return 0, fmt.Errorf("delivery: unable to open stash before drop: %w", err)
	}

	quotaTracker := newDeliveryQuotaTracker(ctx)

	ctx.RefreshGameData()
	if !ctx.Data.OpenMenus.Stash {
		return 0, fmt.Errorf("delivery: stash UI not open before drop")
	}

	const (
		maxPasses      = 1
		maxItemRetries = 2
		maxTotalTime   = 3 * time.Minute
	)
	stashTabs := []int{1, 2, 3, 4}
	totalItemsDelivered := 0

	if dropped, err := d.dropInventoryDeliverables(ctx, 0, quotaTracker); err != nil {
		return 0, err
	} else {
		totalItemsDelivered += dropped
		if d.deliveryRequestSatisfied(quotaTracker) {
			ctx.Logger.Info("Delivery: requested quotas satisfied after initial drop; skipping stash traversal")
			return totalItemsDelivered, nil
		}
	}
	if err := step.OpenInventory(); err != nil {
		return totalItemsDelivered, fmt.Errorf("delivery: failed to reopen inventory after clearing: %w", err)
	}
	ctx.RefreshGameData()

	startTime := time.Now()
	for pass := 0; pass < maxPasses; pass++ {
		if time.Since(startTime) > maxTotalTime {
			ctx.Logger.Warn("Delivery: timeout reached after processing", "elapsed", time.Since(startTime))
			break
		}
		ctx.Logger.Info("Delivery: stash pass", "pass", pass+1)
		movedItems := false

		for _, tab := range stashTabs {
			ctx.Logger.Debug("Delivery: preparing stash tab before processing", "tab", tab)
			if err := d.ensureStashTabReady(ctx, tab); err != nil {
				ctx.Logger.Error("Delivery: failed to prepare stash tab", "tab", tab, "error", err)
				continue
			}
			ctx.Logger.Debug("Delivery: stash tab ready", "tab", tab, "stashOpen", ctx.Data.OpenMenus.Stash)

			deliverables := d.collectDeliverablesForTab(ctx, tab, quotaTracker)
			if len(deliverables) == 0 {
				ctx.Logger.Debug("Delivery: no deliverables on tab", "tab", tab)
				continue
			}

			ctx.Logger.Info("Delivery: moving deliverables from tab", "tab", tab, "count", len(deliverables))

			queue := append([]data.Item(nil), deliverables...)
			attempts := make(map[data.UnitID]int, len(deliverables))

			// Refresh once before processing all items (Mule.go pattern)
			ctx.RefreshGameData()

			for len(queue) > 0 {
				it := queue[0]
				queue = queue[1:]

				_, found := findInventorySpace(ctx, it)
				if !found {
					dropped, err := d.dropInventoryDeliverables(ctx, tab, quotaTracker)
					if err != nil {
						return totalItemsDelivered, err
					}
					totalItemsDelivered += dropped
					if d.deliveryRequestSatisfied(quotaTracker) {
						ctx.Logger.Info("Delivery: requested quotas satisfied while freeing inventory space")
						return totalItemsDelivered, nil
					}
					if dropped > 0 {
						ctx.RefreshGameData()
					}

					if _, found = findInventorySpace(ctx, it); !found {
						attempts[it.UnitID]++
						if attempts[it.UnitID] < maxItemRetries {
							queue = append(queue, it)
							ctx.Logger.Debug("Delivery: re-queueing item due to lack of space", "item", it.Name, "attempt", attempts[it.UnitID])
							continue
						}

						ctx.Logger.Warn("Delivery: inventory still full after drop, skipping item", "item", it.Name)
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
						ctx.Logger.Debug("Delivery: re-queueing item after failed move", "item", it.Name, "attempt", attempts[it.UnitID])
						continue
					}

					ctx.Logger.Warn("Delivery: unable to move item from stash", "item", it.Name)
					if quotaTracker != nil {
						quotaTracker.release(string(it.Name))
					}
				}
			}

			// Refresh after queue completes (Mule.go pattern)
			ctx.RefreshGameData()

			dropped, err := d.dropInventoryDeliverables(ctx, tab, quotaTracker)
			if err != nil {
				return totalItemsDelivered, err
			}
			totalItemsDelivered += dropped
			if d.deliveryRequestSatisfied(quotaTracker) {
				ctx.Logger.Info("Delivery: requested quotas satisfied after dropping items", "tab", tab)
				return totalItemsDelivered, nil
			}
			if err := step.OpenInventory(); err != nil {
				return totalItemsDelivered, fmt.Errorf("delivery: failed to reopen inventory after dropping items: %w", err)
			}
			// Consolidate: refresh only if items were dropped
			if dropped > 0 {
				ctx.RefreshGameData()
			}
		}

		if !movedItems {
			ctx.Logger.Info("Delivery: no more stash items to deliver")
			return totalItemsDelivered, nil
		}
	}

	return totalItemsDelivered, fmt.Errorf("delivery: reached max stash passes without emptying items")
}

func (d Delivery) moveStashItemToInventory(ctx *context.Status, it data.Item) (data.Item, bool) {
	updated := it
	for _, candidate := range ctx.Data.Inventory.AllItems {
		if candidate.UnitID == it.UnitID {
			updated = candidate
			break
		}
	}

	screenPos := ui.GetScreenCoordsForItem(updated)
	ctx.Logger.Debug("Delivery: attempting to move item via ctrl+click", "item", updated.Name, "tab", updated.Location.Page+1, "locationType", updated.Location.LocationType, "gridX", updated.Position.X, "gridY", updated.Position.Y, "screenX", screenPos.X, "screenY", screenPos.Y)
	prevInventoryCount := len(ctx.Data.Inventory.ByLocation(item.LocationInventory))
	ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
	utils.Sleep(500)
	ctx.RefreshInventory()

	for _, invItem := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if invItem.UnitID == it.UnitID {
			ctx.Logger.Debug("Delivery: item detected in inventory after move", "item", it.Name)
			logDeliveryQuotaProgress(ctx, "moved-to-inventory", invItem)
			return invItem, true
		}
	}

	newInventoryCount := len(ctx.Data.Inventory.ByLocation(item.LocationInventory))
	if newInventoryCount > prevInventoryCount {
		ctx.Logger.Debug("Delivery: inventory count increased, assuming item moved", "item", it.Name, "beforeCount", prevInventoryCount, "afterCount", newInventoryCount)
		logDeliveryQuotaProgress(ctx, "moved-to-inventory", it)
		return it, true
	}

	for _, stashItem := range ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash) {
		if stashItem.UnitID == it.UnitID {
			ctx.Logger.Debug("Delivery: item still present in stash after move attempt", "item", it.Name)
			return it, false
		}
	}

	ctx.Logger.Debug("Delivery: unable to determine new location of item after move attempt", "item", updated.Name)
	return updated, false
}

func (d Delivery) dropInventoryDeliverables(ctx *context.Status, reopenTab int, quotas *deliveryQuotaTracker) (int, error) {
	ctx.RefreshInventory()

	stashWasOpen := ctx.Data.OpenMenus.Stash
	if stashWasOpen {
		if err := action.CloseStash(); err != nil {
			ctx.Logger.Error("Delivery: failed to close stash before dropping items", "error", err)
			return 0, err
		}
		utils.Sleep(250)
		ctx.RefreshInventory()
	}

	if err := d.ensureInventoryOpen(ctx); err != nil {
		return 0, err
	}
	ctx.RefreshInventory()

	invItems := ctx.Data.Inventory.ByLocation(item.LocationInventory)
	dropped := 0

	for _, it := range invItems {
		if action.IsInLockedInventorySlot(it) {
			ctx.Logger.Debug("Delivery: skipping locked inventory slot", "item", it.Name, "x", it.Position.X, "y", it.Position.Y)
			continue
		}

		if action.IsDeliveryProtected(it) {
			continue
		}

		screenPos := ui.GetScreenCoordsForItem(it)
		ctx.HID.ClickWithModifier(game.LeftButton, screenPos.X, screenPos.Y, game.CtrlKey)
		utils.Sleep(150)
		dropped++
		// Count this item towards delivery quotas
		if ctx.Delivery != nil {
			ctx.Delivery.RecordDeliveredItem(string(it.Name))
		}
		logDeliveryQuotaProgress(ctx, "dropped-from-inventory", it)
		if quotas != nil {
			quotas.markDelivered(string(it.Name))
		}
	}

	if dropped > 0 {
		ctx.Logger.Info("Delivery: dropped inventory items", "count", dropped)
	}

	ctx.RefreshGameData()
	if ctx.Data.OpenMenus.Inventory {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.Sleep(250)
		ctx.RefreshGameData()
		ctx.RefreshInventory()
	}

	if reopenTab > 0 || stashWasOpen {
		if !ctx.Data.OpenMenus.Stash {
			if err := d.ensureStashOpen(ctx); err != nil {
				ctx.Logger.Error("Delivery: failed to reopen stash after dropping items", "error", err)
				return dropped, err
			}
		}
		if reopenTab > 0 {
			if err := d.ensureStashTabReady(ctx, reopenTab); err != nil {
				ctx.Logger.Error("Delivery: failed to prepare stash tab after reopening", "tab", reopenTab, "error", err)
				return dropped, err
			}
		}
	}

	return dropped, nil
}

func (d Delivery) collectDeliverablesForTab(ctx *context.Status, tab int, quotas *deliveryQuotaTracker) []data.Item {
	stashItems := ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash)
	deliverables := make([]data.Item, 0, len(stashItems))

	for _, it := range stashItems {
		if action.IsDeliveryProtected(it) {
			continue
		}

		if d.itemBelongsToTab(it, tab) {
			itemName := string(it.Name)
			if quotas != nil && !quotas.reserve(itemName) {
				continue
			}
			deliverables = append(deliverables, it)
		}
	}

	return deliverables
}

func (d Delivery) itemBelongsToTab(it data.Item, tab int) bool {
	switch it.Location.LocationType {
	case item.LocationStash:
		return tab == 1
	case item.LocationSharedStash:
		return tab == it.Location.Page+1
	default:
		return false
	}
}

type deliveryQuotaTracker struct {
	ctx             *context.Status
	reserved        map[string]int
	hasFiniteLimits bool
}

func newDeliveryQuotaTracker(ctx *context.Status) *deliveryQuotaTracker {
	if ctx == nil || ctx.Context == nil {
		return nil
	}
	return &deliveryQuotaTracker{
		ctx:             ctx,
		reserved:        make(map[string]int),
		hasFiniteLimits: ctx.Delivery != nil && ctx.Delivery.HasDeliveryQuotaLimits(),
	}
}

func (t *deliveryQuotaTracker) reserve(name string) bool {
	if t == nil {
		return true
	}
	if t.ctx.Delivery == nil {
		return true
	}
	limit := t.ctx.Delivery.GetDeliveryItemQuantity(name)
	if limit <= 0 {
		return true
	}
	key := strings.ToLower(name)
	delivered := t.ctx.Delivery.GetDeliveredItemCount(name)
	if delivered+t.reserved[key] >= limit {
		return false
	}
	t.reserved[key]++
	return true
}

func (t *deliveryQuotaTracker) release(name string) {
	if t == nil {
		return
	}
	key := strings.ToLower(name)
	if t.reserved[key] > 0 {
		t.reserved[key]--
	}
}

func (t *deliveryQuotaTracker) markDelivered(name string) {
	t.release(name)
}

func (t *deliveryQuotaTracker) fulfilled() bool {
	if t == nil || !t.hasFiniteLimits {
		return false
	}
	if t.ctx.Delivery == nil {
		return false
	}
	return t.ctx.Delivery.AreDeliveryQuotasSatisfied()
}

func (d Delivery) deliveryRequestSatisfied(quotas *deliveryQuotaTracker) bool {
	return quotas != nil && quotas.fulfilled()
}

func logDeliveryQuotaProgress(ctx *context.Status, stage string, it data.Item) {
	if ctx == nil || ctx.Delivery == nil {
		return
	}
	limit := ctx.Delivery.GetDeliveryItemQuantity(string(it.Name))
	delivered := ctx.Delivery.GetDeliveredItemCount(string(it.Name))
	remaining := limit - delivered
	if remaining < 0 {
		remaining = 0
	}
	ctx.Logger.Debug("Delivery: quota checkpoint", "stage", stage, "item", string(it.Name), "delivered", delivered, "limit", limit, "remaining", remaining)
}

func (d Delivery) prepareForLobbyJoin(ctx *context.Status) error {
	if ctx.GameReader.IsInLobby() {
		return nil
	}

	if err := d.waitUntilCharacterSelection(ctx); err != nil {
		return err
	}

	if ctx.GameReader.IsInLobby() {
		return nil
	}

	if err := d.ensureOnlineForDelivery(ctx); err != nil {
		return err
	}

	if err := d.ensureDeliveryCharacterSelected(ctx); err != nil {
		ctx.Logger.Warn("Delivery: failed to verify selected character", "error", err)
	}

	return d.enterLobbyForDelivery(ctx)
}

func (d Delivery) waitUntilCharacterSelection(ctx *context.Status) error {
	timeout := time.After(45 * time.Second)

	for {
		if ctx.GameReader.IsInCharacterSelectionScreen() || ctx.GameReader.IsInLobby() {
			return nil
		}

		ctx.HID.Click(game.LeftButton, 100, 100)
		utils.Sleep(250)

		select {
		case <-timeout:
			return fmt.Errorf("delivery: timed out waiting for character selection")
		default:
		}
	}
}

func (d Delivery) ensureDeliveryCharacterSelected(ctx *context.Status) error {
	target := ctx.CharacterCfg.CharacterName
	if target == "" {
		return nil
	}

	for i := 0; i < 25; i++ {
		current := ctx.GameReader.GetSelectedCharacterName()
		if strings.EqualFold(current, target) {
			return nil
		}

		ctx.HID.PressKey(win.VK_DOWN)
		utils.Sleep(250)
	}

	return fmt.Errorf("delivery: character %s not highlighted", target)
}

func (d Delivery) ensureOnlineForDelivery(ctx *context.Status) error {
	if ctx.CharacterCfg.AuthMethod == "None" || ctx.GameReader.IsOnline() {
		return nil
	}

	const maxRetries = 5
	for attempt := 0; attempt < maxRetries; attempt++ {
		ctx.Logger.Debug("Delivery: attempting to connect to battle.net", "attempt", attempt+1)
		ctx.HID.Click(game.LeftButton, 1090, 32)
		utils.Sleep(2000)

		for {
			blocking := ctx.GameReader.GetPanel("BlockingPanel")
			modal := ctx.GameReader.GetPanel("DismissableModal")

			if blocking.PanelName != "" && blocking.PanelEnabled && blocking.PanelVisible {
				utils.Sleep(2000)
				continue
			}

			if modal.PanelName != "" && modal.PanelEnabled && modal.PanelVisible {
				ctx.HID.PressKey(0x1B)
				utils.Sleep(1000)
				break
			}

			break
		}

		if ctx.GameReader.IsOnline() {
			return nil
		}
	}

	return fmt.Errorf("delivery: failed to connect to battle.net")
}

func (d Delivery) enterLobbyForDelivery(ctx *context.Status) error {
	if ctx.GameReader.IsInLobby() {
		return nil
	}

	const maxAttempts = 5
	for attempt := 0; attempt < maxAttempts; attempt++ {
		ctx.Logger.Info("Delivery: entering lobby", "attempt", attempt+1)
		ctx.HID.Click(game.LeftButton, 744, 650)
		utils.Sleep(1000)
		if ctx.GameReader.IsInLobby() {
			return nil
		}
	}

	return fmt.Errorf("delivery: failed to enter lobby")
}

func (d Delivery) ensureStashOpen(ctx *context.Status) error {
	const maxAttempts = 5

	if ctx.Data.AreaData.Grid == nil {
		ctx.Logger.Info("Delivery: map data missing before stash open, fetching...")
		if err := ctx.GameReader.FetchMapData(); err != nil {
			return fmt.Errorf("delivery: failed to fetch map data before stash open: %w", err)
		}
		ctx.RefreshGameData()
		if ctx.Data.AreaData.Grid == nil {
			return fmt.Errorf("delivery: map data unavailable for stash interactions")
		}
	}

	for attempt := 1; attempt <= maxAttempts; attempt++ {
		ctx.RefreshGameData()
		if ctx.Data.OpenMenus.Stash {
			return nil
		}

		if attempt > 1 {
			if err := d.repositionNearStash(ctx); err != nil {
				ctx.Logger.Warn("Delivery: failed to reposition before opening stash", "error", err)
			}
			ctx.RefreshGameData()
			if ctx.Data.OpenMenus.Stash {
				return nil
			}
		}

		ctx.Logger.Info("Delivery: opening stash", "attempt", attempt)
		if err := action.OpenStash(); err != nil {
			ctx.Logger.Error("Delivery: failed to open stash", "attempt", attempt, "error", err)
		}

		utils.Sleep(500)
	}

	ctx.RefreshGameData()
	if ctx.Data.OpenMenus.Stash {
		return nil
	}

	return fmt.Errorf("delivery: stash UI not open after initial setup")
}

func (d Delivery) repositionNearStash(ctx *context.Status) error {
	ctx.RefreshGameData()
	bank, found := ctx.Data.Objects.FindOne(object.Bank)
	if !found {
		return fmt.Errorf("delivery: stash object not found in area %v", ctx.Data.PlayerUnit.Area)
	}

	if ctx.Data.AreaData.Grid == nil {
		ctx.Logger.Debug("Delivery: skipping stash reposition; map data unavailable")
		return nil
	}

	ctx.Logger.Debug("Delivery: moving closer to stash before reopening", "x", bank.Position.X, "y", bank.Position.Y)
	if err := action.MoveToCoords(bank.Position, step.WithDistanceToFinish(6)); err != nil {
		return fmt.Errorf("delivery: failed to reposition near stash: %w", err)
	}

	utils.Sleep(250)
	return nil
}

func (d Delivery) ensureStashTabReady(ctx *context.Status, tab int) error {
	ctx.RefreshGameData()
	if !ctx.Data.OpenMenus.Stash {
		ctx.Logger.Debug("Delivery: stash closed before ensuring tab, reopening", "tab", tab)
		if err := d.ensureStashOpen(ctx); err != nil {
			return err
		}
		utils.Sleep(250)
		ctx.RefreshGameData()
	}

	action.SwitchStashTab(tab)
	utils.Sleep(500)
	ctx.RefreshGameData()
	ctx.Logger.Debug("Delivery: switched stash tab", "tab", tab, "inventoryItems", len(ctx.Data.Inventory.ByLocation(item.LocationInventory)))

	if !ctx.Data.OpenMenus.Stash {
		return fmt.Errorf("delivery: stash UI not open after switching tabs")
	}

	ctx.Logger.Debug("Delivery: stash tab ready", "tab", tab)

	return nil
}
