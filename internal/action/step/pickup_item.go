package step

import (
	"errors"
	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/mode"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/utils"
)

var (
	ErrItemTooFar        = errors.New("item is too far away")
	ErrNoLOSToItem       = errors.New("no line of sight to item")
	ErrMonsterAroundItem = errors.New("monsters detected around item")
	ErrCastingMoving     = errors.New("char casting or moving")
)

func PickupItem(it data.Item, itemPickupAttempt int) error {
	ctx := context.Get()
	ctx.SetLastStep("PickupItem")

	// Wait for the character to finish casting or moving before proceeding.
	// We'll use a local timeout to prevent an indefinite wait.
	waitingStartTime := time.Now()
	for ctx.Data.PlayerUnit.Mode == mode.CastingSkill || ctx.Data.PlayerUnit.Mode == mode.Running || ctx.Data.PlayerUnit.Mode == mode.Walking || ctx.Data.PlayerUnit.Mode == mode.WalkingInTown {
		if time.Since(waitingStartTime) > 2*time.Second {
			ctx.Logger.Warn("Timeout waiting for character to stop moving or casting, proceeding anyway.")
			break
		}
		time.Sleep(25 * time.Millisecond)
		ctx.RefreshGameData()
	}

	// Check for monsters first
	if hasHostileMonstersNearby(it.Position) {
		return ErrMonsterAroundItem
	}

	// Validate line of sight
	if !ctx.PathFinder.LineOfSight(ctx.Data.PlayerUnit.Position, it.Position) {
		return ErrNoLOSToItem
	}

	// Check distance
	distance := ctx.PathFinder.DistanceFromMe(it.Position)
	if distance >= 7 {
		return fmt.Errorf("%w (%d): %s", ErrItemTooFar, distance, it.Desc().Name)
	}

	ctx.Logger.Debug(fmt.Sprintf("Picking up: %s [%s]", it.Desc().Name, it.Quality.ToString()))

	// Track interaction state
	targetItem := it

	ctx.PauseIfNotPriority()
	ctx.RefreshGameData()

	if hasHostileMonstersNearby(it.Position) {
		return ErrMonsterAroundItem
	}

	// Check if item still exists
	currentItem, exists := findItemOnGround(targetItem.UnitID)

	if !exists {
		ctx.Logger.Info(fmt.Sprintf("Picked up: %s [%s] | Item Pickup Attempt:%d", targetItem.Desc().Name, targetItem.Quality.ToString(), itemPickupAttempt))
		ctx.CurrentGame.PickedUpItems[int(targetItem.UnitID)] = int(ctx.Data.PlayerUnit.Area.Area().ID)
		return nil // Success!
	}

	isItemPickedUp := pickupItem(ctx, currentItem)

	if isItemPickedUp {
		return nil
	}

	return fmt.Errorf("failed to pick up %s | Attempt: %d", it.Desc().Name, itemPickupAttempt)
}

func pickupItem(ctx *context.Status, target data.Item) bool {
	// Check if packet casting is enabled for item pickup
	if ctx.CharacterCfg.PacketCasting.UseForItemPickup {
		ctx.Logger.Debug("Attempting item pickup via packet method")
		err := ctx.PacketSender.PickUpItem(target)

		if err != nil {
			ctx.Logger.Error("Packet pickup failed", "error", err)
			return false
		}

		utils.Sleep(100)
		ctx.RefreshInventory()
		_, exists := findItemOnGround(target.UnitID)
		if !exists {
			ctx.Logger.Debug("Item pickup via packet successful")
			return true
		}

		ctx.Logger.Error("Packet sent but item still on ground")
		return false
	}

	// Use mouse-based pickup (original implementation)
	return pickupItemMouse(ctx, target)
}

func pickupItemMouse(ctx *context.Status, target data.Item) bool {
	const maxInteractions = 15
	const spiralDelay = 50 * time.Millisecond
	const clickDelay = 100 * time.Millisecond
	const pickupTimeout = 3 * time.Second

	startTime := time.Now()
	var waitingForInteraction time.Time
	spiralAttempt := 0

	baseScreenX, baseScreenY := ctx.PathFinder.GameCoordsToScreenCords(target.Position.X, target.Position.Y)

	for {
		ctx.RefreshGameData()

		// Check if item was picked up
		_, exists := findItemOnGround(target.UnitID)
		if !exists {
			return true
		}

		// Check timeout conditions
		if spiralAttempt > maxInteractions ||
			(!waitingForInteraction.IsZero() && time.Since(waitingForInteraction) > pickupTimeout) ||
			time.Since(startTime) > pickupTimeout {
			return false
		}

		offsetX, offsetY := utils.ItemSpiral(spiralAttempt)
		cursorX := baseScreenX + offsetX
		cursorY := baseScreenY + offsetY

		// Move cursor directly to target position
		ctx.HID.MovePointer(cursorX, cursorY)
		time.Sleep(spiralDelay)

		// Click on item if mouse is hovering over
		currentItem, itemExists := findItemOnGround(target.UnitID)
		if itemExists && currentItem.UnitID == ctx.GameReader.GameReader.GetData().HoverData.UnitID {
			ctx.HID.Click(game.LeftButton, cursorX, cursorY)
			time.Sleep(clickDelay)

			if waitingForInteraction.IsZero() {
				waitingForInteraction = time.Now()
			}
			continue
		}

		// Sometimes we got stuck because mouse is hovering a chest and item is behind it
		if isChestorShrineHovered() {
			ctx.HID.Click(game.LeftButton, cursorX, cursorY)
			time.Sleep(50 * time.Millisecond)
		}

		spiralAttempt++
	}
}

func isChestorShrineHovered() bool {
	ctx := context.Get()
	hoverData := ctx.Data.HoverData
	return hoverData.IsHovered && (hoverData.UnitType == 2 || hoverData.UnitType == 5)
}

func hasHostileMonstersNearby(pos data.Position) bool {
	ctx := context.Get()

	for _, monster := range ctx.Data.Monsters.Enemies() {
		if monster.Stats[stat.Life] > 0 && pather.DistanceFromPoint(pos, monster.Position) <= 4 {
			return true
		}
	}
	return false
}

func findItemOnGround(targetID data.UnitID) (data.Item, bool) {
	ctx := context.Get()

	for _, i := range ctx.Data.Inventory.ByLocation(item.LocationGround) {
		if i.UnitID == targetID {
			return i, true
		}
	}
	return data.Item{}, false
}
