package run

import (
	"errors"
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
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
	return string(config.MuleRun)
}

func (m Mule) CheckConditions(parameters *RunParameters) SequencerResult {
	return SequencerError
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

func (m Mule) Run(parameters *RunParameters) error {
	ctx := context.Get()

	returnToChar := ctx.CharacterCfg.Muling.ReturnTo
	muleProfiles := ctx.CharacterCfg.Muling.MuleProfiles
	ctx.Logger.Info("Starting mule run", "muleCharacter", ctx.Name)

	//	if len(muleProfiles) == 0 {
	//		ctx.Logger.Error("Mule run started, but no 'muleProfiles' are configured in settings. Stopping.")
	//		return nil // Stop cleanly
	//	}

	if returnToChar == "" {
		ctx.Logger.Error("Mule run started, but 'ReturnTo' is not configured in settings. Stopping.")
		return nil // Stop cleanly
	}

	// Run initial setup.
	if err := m.initialSetup(ctx); err != nil {
		ctx.Logger.Error("Mule initial setup failed, switching back to original character.", "error", err)
		// Even if setup fails, we should try to switch back
		ctx.CurrentGame.SwitchToCharacter = returnToChar
		ctx.RestartWithCharacter = returnToChar
		ctx.CleanStopRequested = true
		ctx.StopSupervisor()
		return err
	}

	// Check if the current mule's private stash is already full
	if isPrivateStashFull(ctx) {
		ctx.Logger.Info("Current mule's stash is full, checking for the next one.")
		ctx.CurrentGame.CurrentMuleIndex++
		if ctx.CurrentGame.CurrentMuleIndex < len(muleProfiles) {
			// We have another mule to switch to
			nextMule := muleProfiles[ctx.CurrentGame.CurrentMuleIndex]
			ctx.Logger.Info("Switching to next mule", "mule", nextMule)
			ctx.CurrentGame.SwitchToCharacter = nextMule
		} else {
			// No more mules, return to the farming character
			ctx.Logger.Info("All available mules are full, returning to farming character.")
			ctx.CurrentGame.SwitchToCharacter = returnToChar
		}
	} else {
		// Stash is not full, proceed with muling logic
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
				ctx.Logger.Info("Muling complete: No more items to move in this cycle.")
				break
			}
		}

		// After muling, decide who to switch to
		if isPrivateStashFull(ctx) {
			ctx.Logger.Info("Mule is now full, checking for the next mule.")
			ctx.CurrentGame.CurrentMuleIndex++
			if ctx.CurrentGame.CurrentMuleIndex < len(muleProfiles) {
				// We have another mule to switch to
				nextMule := muleProfiles[ctx.CurrentGame.CurrentMuleIndex]
				ctx.Logger.Info("Switching to next mule", "mule", nextMule)
				ctx.CurrentGame.SwitchToCharacter = nextMule
			} else {
				ctx.Logger.Info("All available mules are now full, returning to farming character.")
				ctx.CurrentGame.SwitchToCharacter = returnToChar
			}
		} else {
			// Muling is done and the current mule is not full, return to farmer
			ctx.Logger.Info("Muling finished, returning to farming character.")
			ctx.CurrentGame.SwitchToCharacter = returnToChar
		}
	}

	ctx.Logger.Info("Preparing to switch character",
		"from", ctx.Name,
		"to", ctx.CurrentGame.SwitchToCharacter)

	ctx.RestartWithCharacter = ctx.CurrentGame.SwitchToCharacter
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

// isPrivateStashFull checks if the personal stash has any 2x2 free space.
// This is a simple heuristic to determine if the stash is "full".
func isPrivateStashFull(ctx *context.Status) bool {
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

	// Check for a 2x2 free space
	for y := 0; y <= 8; y++ {
		for x := 0; x <= 8; x++ {
			if !occupied[y][x] && !occupied[y+1][x] && !occupied[y][x+1] && !occupied[y+1][x+1] {
				return false // Found a 2x2 space, so not full
			}
		}
	}

	return true // No 2x2 space found
}
