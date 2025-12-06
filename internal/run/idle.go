package run

import (
	"fmt"
	"log/slog"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Idle struct {
	ctx *context.Status
}

func NewIdle() *Idle {
	return &Idle{
		ctx: context.Get(),
	}
}

func (i *Idle) Name() string {
	return string(config.IdleRun)
}

func (i *Idle) SkipTownRoutines() bool {
	return true
}

func (i *Idle) CheckConditions(parameters *RunParameters) SequencerResult {
	return SequencerOk
}

func (i *Idle) Run(parameters *RunParameters) error {
	i.ctx.Logger.Info("Idle mode: Character will stay idle. Use this to manually read coordinates.")

	// Log initial position
	position := i.ctx.Data.PlayerUnit.Position
	i.ctx.Logger.Info(
		"Current position",
		slog.String("area", i.ctx.Data.PlayerUnit.Area.Area().Name),
		slog.Int("posX", position.X),
		slog.Int("posY", position.Y),
	)

	// Log all inventory items once at start
	i.ctx.RefreshGameData()
	i.logAllInventoryItems()

	// Simple loop that just waits and logs position periodically
	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()

	for {
		if i.ctx.ExecutionPriority == context.PriorityStop {
			i.ctx.Logger.Info("Idle mode stopped")
			return nil
		}

		select {
		case <-ticker.C:
			i.ctx.RefreshGameData()
			position := i.ctx.Data.PlayerUnit.Position
			i.ctx.Logger.Info(
				"Current position",
				slog.String("area", i.ctx.Data.PlayerUnit.Area.Area().Name),
				slog.Int("posX", position.X),
				slog.Int("posY", position.Y),
			)
			// Log inventory items every 5 seconds
			i.logAllInventoryItems()
		default:
			utils.Sleep(100)
		}
	}
}

// logAllInventoryItems logs all items in inventory with their properties
func (i *Idle) logAllInventoryItems() {
	inventoryItems := i.ctx.Data.Inventory.ByLocation(item.LocationInventory)
	i.ctx.Logger.Info(fmt.Sprintf("=== INVENTORY ITEMS (Total: %d) ===", len(inventoryItems)))

	for idx, itm := range inventoryItems {
		i.ctx.Logger.Info(
			fmt.Sprintf("Item #%d", idx+1),
			slog.String("Name", string(itm.Name)),
			slog.String("Desc().Name", itm.Desc().Name),
			slog.String("Desc().Type", itm.Desc().Type),
			slog.String("IdentifiedName", itm.IdentifiedName),
			slog.String("Quality", itm.Quality.ToString()),
			slog.Uint64("UnitID", uint64(itm.UnitID)),
			slog.Bool("Identified", itm.Identified),
			slog.Any("Position", itm.Position),
		)
	}

	i.ctx.Logger.Info("=== END INVENTORY ITEMS ===")
}

