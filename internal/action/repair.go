package action

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	botCtx "github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

func Repair() error {
	ctx := context.Get()
	ctx.SetLastAction("Repair")

	for _, i := range ctx.Data.Inventory.ByLocation(item.LocationEquipped) {
		// var
		triggerRepair := false
		logMessage := ""

		_, indestructible := i.FindStat(stat.Indestructible, 0)
		quantity, quantityFound := i.FindStat(stat.Quantity, 0)

		// skip indestructible and no quantity
		if indestructible && !quantityFound {
			continue
		}

		// skip eth and no qnt
		if i.Ethereal && !quantityFound {
			continue
		}

		// qantity check
		if quantityFound {
			// Low quantity (under 15) or broken (quantity 0)
			if quantity.Value < 15 || i.IsBroken {
				triggerRepair = true
				logMessage = fmt.Sprintf("Replenishing %s, quantity is %d", i.Name, quantity.Value)
			}
		} else {
			// check item durability (Durability)
			durability, found := i.FindStat(stat.Durability, 0)
			maxDurability, maxDurabilityFound := i.FindStat(stat.MaxDurability, 0)
			durabilityPercent := -1

			if maxDurabilityFound && found {
				durabilityPercent = int((float64(durability.Value) / float64(maxDurability.Value)) * 100)
			}

			// if item is broken or under 20%
			if i.IsBroken || (durabilityPercent != -1 && durabilityPercent <= 20) {
				triggerRepair = true
				logMessage = fmt.Sprintf("Repairing %s, item durability is %d percent", i.Name, durabilityPercent)
			}
		}

		// trigger
		if triggerRepair {
			ctx.Logger.Info(logMessage)

			repairNPC := town.GetTownByArea(ctx.Data.PlayerUnit.Area).RepairNPC()
			if repairNPC == npc.Larzuk {
				MoveToCoords(data.Position{X: 5135, Y: 5046})
			}
			if repairNPC == npc.Hratli {

				if err := FindHratliEverywhere(); err != nil {
					// If moveToHratli returns an error, it means a forced game quit is required.
					return err
				}
				// If no error, Hratli was found at the final position, and we continue to interact and repair.
			}

			if err := InteractNPC(repairNPC); err != nil {
				return err
			}

			if repairNPC != npc.Halbu {
				ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
			} else {
				ctx.HID.KeySequence(win.VK_HOME, win.VK_RETURN)
			}

			utils.Sleep(100)
			if ctx.Data.LegacyGraphics {
				ctx.HID.Click(game.LeftButton, ui.RepairButtonXClassic, ui.RepairButtonYClassic)
			} else {
				ctx.HID.Click(game.LeftButton, ui.RepairButtonX, ui.RepairButtonY)
			}
			utils.Sleep(500)

			return step.CloseAllMenus()
		}
	}

	return nil
}

func RepairRequired() bool {
	ctx := context.Get()
	ctx.SetLastAction("RepairRequired")

	for _, i := range ctx.Data.Inventory.ByLocation(item.LocationEquipped) {
		_, indestructible := i.FindStat(stat.Indestructible, 0)
		quantity, quantityFound := i.FindStat(stat.Quantity, 0)

		if indestructible && !quantityFound {
			continue
		}

		if i.Ethereal && !quantityFound {
			continue
		}

		// qnt check
		if quantityFound {
			if quantity.Value < 15 || i.IsBroken {
				return true
			}
		} else {
			// durability check
			durability, found := i.FindStat(stat.Durability, 0)
			maxDurability, maxDurabilityFound := i.FindStat(stat.MaxDurability, 0)

			if i.IsBroken || (maxDurabilityFound && !found) {
				return true
			}

			if found && maxDurabilityFound {
				durabilityPercent := int((float64(durability.Value) / float64(maxDurability.Value)) * 100)
				if durabilityPercent <= 20 {
					return true
				}
			}
		}
	}

	return false
}

func IsEquipmentBroken() bool {
	ctx := context.Get()
	ctx.SetLastAction("EquipmentBroken")

	for _, i := range ctx.Data.Inventory.ByLocation(item.LocationEquipped) {
		_, indestructible := i.FindStat(stat.Indestructible, 0)
		_, quantityFound := i.FindStat(stat.Quantity, 0)

		// eth quantity
		if i.Ethereal && !quantityFound {
			continue
		}

		// indestructible with quantity
		if indestructible && !quantityFound {
			continue
		}

		// Check if item is broken, works for quantity if its 0.
		if i.IsBroken {
			ctx.Logger.Debug("Equipment is broken, returning to town", "item", i.Name)
			return true
		}
	}

	return false
}

func FindHratliEverywhere() error {
	ctx := botCtx.Get()
	ctx.SetLastStep("FindHratliEverywhere")

	// 1. Move to Hratli's final (default) position to check if he is there.
	finalPos := data.Position{X: 5224, Y: 5045}
	MoveToCoords(finalPos)

	// 2. Check if Hratli is found in the vicinity (meaning he is at his final position).
	_, found := ctx.Data.Monsters.FindOne(npc.Hratli, data.MonsterTypeNone)

	if !found {
		// Hratli is NOT found after moving to his final spot. Assume he is at the start position.
		ctx.Logger.Warn("Hratli not found at final position. Moving to start position to trigger quest update and force quitting game.")

		// Start Position: {X: 5116, Y: 5167}
		startPos := data.Position{X: 5116, Y: 5167}
		MoveToCoords(startPos)

		// Interact with him there (to satisfy the requirement/quest logic)
		if err := InteractNPC(npc.Hratli); err != nil {
			ctx.Logger.Warn("Failed to interact with Hratli at start position.", "error", err)
		}

		// Close menus and force game quit
		step.CloseAllMenus()
		return nil
	}

	// 3. If found, we are already at his final position and can proceed with interaction.
	return nil
}
