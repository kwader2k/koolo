package run

import (
	"errors"
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	MuleActionDelay = 500 // in milliseconds
)

var (
	ErrItemNotFound = errors.New("item not found")
)

type Mule struct {
}

func NewMule() Mule {
	return Mule{}
}

func (m Mule) Name() string {
	return "Mule"
}

// initialSetup ensures the bot is in a valid state to start muling.
func (m Mule) initialSetup(ctx *context.Status) error {
	ctx.WaitForGameToLoad()
	if !ctx.Data.PlayerUnit.Area.IsTown() {
		return errors.New("mule run can only be started in town")
	}
	if err := action.OpenStash(); err != nil {
		return fmt.Errorf("error opening stash: %w", err)
	}
	if !ctx.Data.OpenMenus.Inventory {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.Sleep(MuleActionDelay)
	}
	return nil
}

func (m Mule) Run() error {
	ctx := context.Get()

	returnToChar := ctx.CharacterCfg.Muling.ReturnTo
	ctx.Logger.Info("Starting mule run", "muleCharacter", ctx.Name)

	if returnToChar == "" {
		ctx.Logger.Error("Mule run started, but 'ReturnTo' is not configured in settings. Stopping.")
		return nil // Stop cleanly
	}

	// Run initial setup.
	if err := m.initialSetup(ctx); err != nil {
		ctx.Logger.Error("Mule initial setup failed, switching back to original character.", "error", err)
	} else {

		for {
			movedItemInLoop := false

			// Phase 1: Move items from all shared tabs to inventory
			for sharedTab := 2; sharedTab <= 4; sharedTab++ {
				action.SwitchStashTab(sharedTab)
				utils.Sleep(MuleActionDelay)

				ctx.RefreshGameData()
				itemsToMove := ctx.Data.Inventory.ByLocation(item.LocationSharedStash)
				if len(itemsToMove) > 0 {
					ctx.Logger.Info("Found items in shared stash", "tab", sharedTab, "count", len(itemsToMove))
				}

				for _, itemToMove := range itemsToMove {
					if _, found := findInventorySpace(ctx, itemToMove); !found {
						ctx.Logger.Info("Inventory is full, cannot pick up more items.")
						break
					}

					ctx.HID.ClickWithModifier(game.LeftButton, ui.GetScreenCoordsForItem(itemToMove).X, ui.GetScreenCoordsForItem(itemToMove).Y, game.CtrlKey)
					utils.Sleep(MuleActionDelay)
					movedItemInLoop = true
				}
			}

			// Phase 2: Move all items from inventory to private stash
			action.SwitchStashTab(1)
			utils.Sleep(MuleActionDelay)

			ctx.RefreshGameData()
			itemsToDeposit := ctx.Data.Inventory.ByLocation(item.LocationInventory)
			if len(itemsToDeposit) > 0 {
				ctx.Logger.Info("Depositing items from inventory to private stash", "count", len(itemsToDeposit))
			}

			for _, itemToDeposit := range itemsToDeposit {
				if _, found := findStashSpace(ctx, itemToDeposit); !found {
					ctx.Logger.Info("Private stash is full, cannot deposit more items.")
					break
				}

				ctx.HID.ClickWithModifier(game.LeftButton, ui.GetScreenCoordsForItem(itemToDeposit).X, ui.GetScreenCoordsForItem(itemToDeposit).Y, game.CtrlKey)
				utils.Sleep(MuleActionDelay)
				movedItemInLoop = true
			}

			if !movedItemInLoop {
				ctx.Logger.Info("Muling complete: No items could be moved in a full cycle.")
				break
			}
		}
	}

	ctx.Logger.Info("Preparing to switch back to original character",
		"from", ctx.Name,
		"to", returnToChar)

	ctx.CurrentGame.SwitchToCharacter = returnToChar
	ctx.RestartWithCharacter = returnToChar
	ctx.CleanStopRequested = true

	if err := ctx.Manager.ExitGame(); err != nil {
		ctx.Logger.Error("Failed to exit game before character switch", "error", err)
	}
	utils.Sleep(2000)

	ctx.StopSupervisor()
	return nil
}

// findStashSpace finds the top-left grid coordinates for a free spot in the personal stash.
func findStashSpace(ctx *context.Status, itm data.Item) (data.Position, bool) {
	stash := ctx.Data.Inventory.ByLocation(item.LocationStash)
	occupied := [10][10]bool{}
	for _, i := range stash {
		for y := 0; y < i.Desc().InventoryHeight; y++ {
			for x := 0; x < i.Desc().InventoryWidth; x++ {
				if i.Position.Y+y < 10 && i.Position.X+x < 10 {
					occupied[i.Position.Y+y][i.Position.X+x] = true
				}
			}
		}
	}
	w := itm.Desc().InventoryWidth
	h := itm.Desc().InventoryHeight
	for y := 0; y <= 10-h; y++ {
		for x := 0; x <= 10-w; x++ {
			fits := true
			for j := 0; j < h; j++ {
				for i := 0; i < w; i++ {
					if occupied[y+j][x+i] {
						fits = false
						break
					}
				}
				if !fits {
					break
				}
			}
			if fits {
				return data.Position{X: x, Y: y}, true
			}
		}
	}
	return data.Position{}, false
}

// findInventorySpace finds the top-left grid coordinates for a free spot in the inventory.
func findInventorySpace(ctx *context.Status, itm data.Item) (data.Position, bool) {
	inventory := ctx.Data.Inventory.ByLocation(item.LocationInventory)
	lockConfig := ctx.CharacterCfg.Inventory.InventoryLock
	occupied := [4][10]bool{}
	for _, i := range inventory {
		for y := 0; y < i.Desc().InventoryHeight; y++ {
			for x := 0; x < i.Desc().InventoryWidth; x++ {
				if i.Position.Y+y < 4 && i.Position.X+x < 10 {
					occupied[i.Position.Y+y][i.Position.X+x] = true
				}
			}
		}
	}
	for y, row := range lockConfig {
		if y < 4 {
			for x, cell := range row {
				if x < 10 && cell == 0 {
					occupied[y][x] = true
				}
			}
		}
	}
	w := itm.Desc().InventoryWidth
	h := itm.Desc().InventoryHeight
	for y := 0; y <= 4-h; y++ {
		for x := 0; x <= 10-w; x++ {
			fits := true
			for j := 0; j < h; j++ {
				for i := 0; i < w; i++ {
					if occupied[y+j][x+i] {
						fits = false
						break
					}
				}
				if !fits {
					break
				}
			}
			if fits {
				return data.Position{X: x, Y: y}, true
			}
		}
	}
	return data.Position{}, false
}
