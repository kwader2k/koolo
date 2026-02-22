package step

import (
	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/mode"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

func PickupItemPacket(it data.Item, itemPickupAttempt int) error {
	ctx := context.Get()

	// Wait for the character to finish casting or moving before proceeding.
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

	ctx.Logger.Debug(fmt.Sprintf("Picking up (packet): %s [%s]", it.Desc().Name, it.Quality.ToString()))

	targetItem := it

	ctx.PauseIfNotPriority()
	ctx.RefreshGameData()

	if hasHostileMonstersNearby(it.Position) {
		return ErrMonsterAroundItem
	}

	// Check if item still exists
	_, exists := findItemOnGround(targetItem.UnitID)
	if !exists {
		ctx.Logger.Info(fmt.Sprintf("Picked up (already gone): %s [%s] | Item Pickup Attempt:%d", targetItem.Desc().Name, targetItem.Quality.ToString(), itemPickupAttempt))
		ctx.CurrentGame.PickedUpItems[int(targetItem.UnitID)] = int(ctx.Data.PlayerUnit.Area.Area().ID)
		return nil
	}

	// Send packet to pick up item
	err := ctx.PacketSender.PickUpItem(targetItem)
	if err != nil {
		ctx.Logger.Error("Packet pickup failed", "error", err)
		return fmt.Errorf("packet pickup failed: %w", err)
	}

	for i := 0; i < 5; i++ {
		utils.PingSleep(utils.Light, 150)
		ctx.RefreshInventory()

		// Verify pickup
		_, stillExists := findItemOnGround(targetItem.UnitID)
		if !stillExists {
			ctx.Logger.Info(fmt.Sprintf("Picked up (packet): %s [%s] | Item Pickup Attempt:%d", targetItem.Desc().Name, targetItem.Quality.ToString(), itemPickupAttempt))
			ctx.CurrentGame.PickedUpItems[int(targetItem.UnitID)] = int(ctx.Data.PlayerUnit.Area.Area().ID)
			return nil
		}
	}

	ctx.Logger.Warn("Packet sent but item still on ground")
	return fmt.Errorf("packet pickup failed - item still on ground")
}
