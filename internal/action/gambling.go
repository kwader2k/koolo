package action

import (
	"errors"
	"log/slog"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/nip"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

func Gamble() error {
	ctx := context.Get()
	ctx.SetLastAction("Gamble")

	stashedGold, _ := ctx.Data.PlayerUnit.FindStat(stat.StashGold, 0)
	if ctx.CharacterCfg.Gambling.Enabled && stashedGold.Value >= 2480000 {
		ctx.Logger.Info("Time to gamble! Visiting vendor...")

		vendorNPC := town.GetTownByArea(ctx.Data.PlayerUnit.Area).GamblingNPC()

		// Fix for Anya position
		if vendorNPC == npc.Drehya {
			_ = MoveToCoords(data.Position{
				X: 5107,
				Y: 5119,
			})
		}

		InteractNPC(vendorNPC)
		// Jamella gamble button is the second one
		if vendorNPC == npc.Jamella {
			ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
		} else {
			ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_DOWN, win.VK_RETURN)
		}

		if !ctx.Data.OpenMenus.NPCShop {
			return errors.New("failed opening gambling window")
		}

		return gambleItems()
	}

	return nil
}

func GambleSingleItem(items []string, desiredQuality item.Quality) error {
	ctx := context.Get()
	ctx.SetLastAction("GambleSingleItem")

	charGold := ctx.Data.PlayerUnit.TotalPlayerGold()
	var itemBought data.Item

	// Check if we have enough gold to gamble
	if charGold >= 150000 {
		ctx.Logger.Info("Gambling for items", slog.Any("items", items))

		vendorNPC := town.GetTownByArea(ctx.Data.PlayerUnit.Area).GamblingNPC()

		// Fix for Anya position
		if vendorNPC == npc.Drehya {
			_ = MoveToCoords(data.Position{
				X: 5107,
				Y: 5119,
			})
		}

		InteractNPC(vendorNPC)
		// Jamella gamble button is the second one
		if vendorNPC == npc.Jamella {
			ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
		} else {
			ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_DOWN, win.VK_RETURN)
		}

		if !ctx.Data.OpenMenus.NPCShop {
			return errors.New("failed opening gambling window")
		}
	}

	for {
		if itemBought.Name != "" {
			for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
				if itm.UnitID == itemBought.UnitID {
					itemBought = itm
					ctx.Logger.Debug("Gambled for item", slog.Any("item", itemBought))
					break
				}
			}

			// Check if the item matches our NIP rules
			if _, result := ctx.Data.CharacterCfg.Runtime.Rules.EvaluateAll(itemBought); result == nip.RuleResultFullMatch {
				// Filter not pass, selling the item
				ctx.Logger.Info("Found item matching nip rules, will be kept", slog.Any("item", itemBought))
				itemBought = data.Item{}
				continue
			} else {
				// Doesn't match NIP rules but check if the item matches our desired quality
				if itemBought.Quality == desiredQuality {
					ctx.Logger.Info("Found item matching desired quality, will be kept", slog.Any("item", itemBought))
					return step.CloseAllMenus()
				} else {
					town.SellItem(itemBought)
					itemBought = data.Item{}
				}
			}
		}

		if ctx.Data.PlayerUnit.TotalPlayerGold() < 150000 {
			return errors.New("gold is below 150000, stopping gamble")
		}

		// Check for any of the desired items in the vendor's inventory
		for _, itmName := range items {
			itm, found := ctx.Data.Inventory.Find(item.Name(itmName), item.LocationVendor)
			if found {
				town.BuyItem(itm, 1)
				itemBought = itm
				break
			}
		}

		// If no desired item was found, refresh the gambling window
		if itemBought.Name == "" {
			ctx.Logger.Debug("Desired items not found in gambling window, refreshing...", slog.Any("items", items))

			if ctx.Data.LegacyGraphics {
				ctx.HID.Click(game.LeftButton, ui.GambleRefreshButtonXClassic, ui.GambleRefreshButtonYClassic)
			} else {
				ctx.HID.Click(game.LeftButton, ui.GambleRefreshButtonX, ui.GambleRefreshButtonY)
			}

			utils.Sleep(500)
		}
	}
}

func gambleItems() error {
	ctx := context.Get()
	ctx.SetLastAction("gambleItems")

	var itemBought data.Item
	var refreshAttempts int
	const maxRefreshAttempts = 11
	const maxPurchasesPerItem = 5
	const coronetCircletGroup = "coronet_circlet_group"

	isGroupItem := func(name string) bool {
		n := strings.ToLower(name)
		return n == "coronet" || n == "circlet"
	}

	getCounterKey := func(itemName string) string {
		if isGroupItem(itemName) {
			return coronetCircletGroup
		}
		return strings.ToLower(itemName)
	}

	purchaseCounters := make(map[string]int, len(ctx.Data.CharacterCfg.Gambling.Items))
	for _, itemName := range ctx.Data.CharacterCfg.Gambling.Items {
		purchaseCounters[getCounterKey(itemName)] = 0
	}

	checkAndResetCounters := func() {
		for _, itemName := range ctx.Data.CharacterCfg.Gambling.Items {
			if purchaseCounters[getCounterKey(itemName)] < maxPurchasesPerItem {
				return
			}
		}
		for key := range purchaseCounters {
			purchaseCounters[key] = 0
		}
	}

	for {
		ctx.PauseIfNotPriority()
		ctx.RefreshGameData()

		if ctx.Data.PlayerUnit.TotalPlayerGold() < 500000 {
			ctx.Logger.Info("Finished gambling - gold below 500k",
				slog.Int("currentGold", ctx.Data.PlayerUnit.TotalPlayerGold()))
			return step.CloseAllMenus()
		}

		checkAndResetCounters()

		if itemBought.Name != "" {
			originalItemName := string(itemBought.Name)
			for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
				if itm.UnitID == itemBought.UnitID {
					itemBought = itm
					ctx.Logger.Debug("Gambled for item", slog.Any("item", itemBought))
					break
				}
			}

			if _, result := ctx.Data.CharacterCfg.Runtime.Rules.EvaluateAll(itemBought); result == nip.RuleResultFullMatch {
				ctx.Logger.Info("Found item matching NIP rules, keeping", slog.Any("item", itemBought))
			} else {
				ctx.Logger.Debug("Item doesn't match NIP rules, selling", slog.Any("item", itemBought))
				town.SellItem(itemBought)
			}

			purchaseCounters[getCounterKey(originalItemName)]++
			itemBought = data.Item{}
			refreshAttempts = 0
			continue
		}

		var bestItem data.Item
		vendorItems := ctx.Data.Inventory.ByLocation(item.LocationVendor)

		for _, itemName := range ctx.Data.CharacterCfg.Gambling.Items {
			if purchaseCounters[getCounterKey(itemName)] >= maxPurchasesPerItem {
				continue
			}

			if isGroupItem(itemName) {
				for _, vendorItem := range vendorItems {
					if isGroupItem(string(vendorItem.Name)) {
						bestItem = vendorItem
						break
					}
				}
			} else {
				bestItem, _ = ctx.Data.Inventory.Find(item.Name(itemName), item.LocationVendor)
			}

			if bestItem.Name != "" {
				break
			}
		}

		if bestItem.Name != "" {
			town.BuyItem(bestItem, 1)
			itemBought = bestItem
		} else {
			refreshAttempts++
			if refreshAttempts >= maxRefreshAttempts {
				ctx.Logger.Info("Too many refresh attempts without finding items, reopening gambling window")
				if err := step.CloseAllMenus(); err != nil {
					return err
				}
				utils.Sleep(200)

				vendorNPC := town.GetTownByArea(ctx.Data.PlayerUnit.Area).GamblingNPC()
				if err := InteractNPC(vendorNPC); err != nil {
					return err
				}

				if vendorNPC == npc.Jamella {
					ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
				} else {
					ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_DOWN, win.VK_RETURN)
				}

				refreshAttempts = 0
				continue
			}

			ctx.Logger.Debug("Refreshing.. ", slog.Int("Attempt", refreshAttempts))
			RefreshGamblingWindow(ctx)
			utils.Sleep(500)
		}
	}
}
func RefreshGamblingWindow(ctx *context.Status) {
	if ctx.Data.LegacyGraphics {
		ctx.HID.Click(game.LeftButton, ui.GambleRefreshButtonXClassic, ui.GambleRefreshButtonYClassic)
	} else {
		ctx.HID.Click(game.LeftButton, ui.GambleRefreshButtonX, ui.GambleRefreshButtonY)
	}
}
