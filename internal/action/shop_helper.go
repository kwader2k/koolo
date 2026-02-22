package action

import (
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

// BuyAct2Flails attempts to purchase 3-socket normal Flails from Fara in Act 2 for Barbarian characters.
// These are used for creating Spirit runewords.
func BuyAct2Flails(ctx *context.Status) error {
	// Barbarian-exclusive shopping for 3-socket Flails
	if ctx.CharacterCfg.Character.Class != "barbarian" {
		return nil
	}

	// Helper to refresh shop inventory: exit to Rocky Waste and return to Lut Gholein
	refreshVendors := func() {
		step.CloseAllMenus()
		_ = MoveToArea(area.RockyWaste)
		_ = MoveToArea(area.LutGholein)
	}

	// Count current 3-socket Flails in equipped + inventory + stash
	have := 0
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationEquipped, item.LocationInventory, item.LocationStash, item.LocationSharedStash) {
		if string(itm.Name) == "Flail" {
			if sockets, ok := itm.FindStat(stat.NumSockets, 0); ok && sockets.Value == 3 {
				have++
			}
		}
	}

	need := 2 - have
	if need <= 0 {
		ctx.Logger.Info("Already have 2 three-socketed Flails, skipping purchase.")
		return nil
	}

	// Skip if too low on gold
	const minGoldReserve = 15000
	if ctx.Data.PlayerUnit.TotalPlayerGold() < minGoldReserve {
		ctx.Logger.Info("Not enough gold to buy Flails, skipping.", "gold", ctx.Data.PlayerUnit.TotalPlayerGold())
		return nil
	}

	ctx.Logger.Info("Attempting to buy 3-socketed Flails from Fara", "need", need)
	maxTries := 8
	for t := 0; t < maxTries && need > 0; t++ {
		if err := InteractNPC(npc.Fara); err != nil {
			ctx.Logger.Error("Failed to interact with Fara", "error", err)
			continue
		}
		// Trade option for Fara (first option is repair, second is trade)
		ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
		utils.Sleep(1000)

		ctx.GameReader.GetData()
		if !ctx.Data.OpenMenus.NPCShop {
			ctx.Logger.Debug("Shop menu not open, closing and retrying")
			step.CloseAllMenus()
			continue
		}

		// Switch to weapons tab (tab 2, zero-indexed)
		SwitchVendorTab(2)
		utils.Sleep(500)
		ctx.GameReader.GetData()

		bought := false
		for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationVendor) {
			if need <= 0 {
				break
			}
			if string(itm.Name) == "Flail" {
				if sockets, ok := itm.FindStat(stat.NumSockets, 0); ok && sockets.Value == 3 {
					// Check if it's normal quality (best for Spirit runeword)
					if itm.Quality == item.QualityNormal {
						if ctx.Data.PlayerUnit.TotalPlayerGold() < 10000 {
							ctx.Logger.Info("Not enough gold remaining, stopping Flail shopping")
							break
						}
						ctx.Logger.Info("Buying 3-socket Flail", "name", itm.Name, "quality", itm.Quality)
						town.BuyItem(itm, 1)
						bought = true
						need--
					}
				}
			}
		}
		step.CloseAllMenus()

		if bought {
			ctx.Logger.Info("Flail(s) purchased, refreshing game data.")
			ctx.RefreshGameData()
		}

		if need > 0 {
			// Force shop refresh by zone hop
			ctx.Logger.Debug("Refreshing vendors", "stillNeed", need, "attempt", t+1)
			refreshVendors()
			utils.Sleep(250)
		}
	}

	if need > 0 {
		ctx.Logger.Info("Could not find enough 3-socket Flails", "stillNeed", need)
	} else {
		ctx.Logger.Info("Successfully purchased all needed 3-socket Flails")
	}

	return nil
}

// BuyAct2BoneWands attempts to purchase 2-socket Bone Wands from Drognan in Act 2 for Necromancer characters.
// These are used for creating White runewords.
func BuyAct2BoneWands(ctx *context.Status) error {
	// Necromancer-exclusive shopping for 2-socket Bone Wands
	if ctx.CharacterCfg.Character.Class != "necromancer_leveling" && ctx.CharacterCfg.Character.Class != "necromancer" {
		return nil
	}

	// Helper to refresh shop inventory: exit to Rocky Waste and return to Lut Gholein
	refreshVendors := func() {
		step.CloseAllMenus()
		_ = MoveToArea(area.RockyWaste)
		_ = MoveToArea(area.LutGholein)
	}

	// Count current 2-socket Bone Wands in equipped + inventory + stash
	have := 0
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationEquipped, item.LocationInventory, item.LocationStash, item.LocationSharedStash) {
		if string(itm.Name) == "BoneWand" {
			if sockets, ok := itm.FindStat(stat.NumSockets, 0); ok && sockets.Value == 2 {
				have++
			}
		}
	}

	need := 1 - have
	if need <= 0 {
		ctx.Logger.Info("Already have a two-socketed Bone Wand, skipping purchase.")
		return nil
	}

	// Skip if too low on gold
	const minGoldReserve = 20000
	if ctx.Data.PlayerUnit.TotalPlayerGold() < minGoldReserve {
		ctx.Logger.Info("Not enough gold to buy Bone Wands, skipping.", "gold", ctx.Data.PlayerUnit.TotalPlayerGold())
		return nil
	}

	ctx.Logger.Info("Attempting to buy 2-socketed Bone Wands from Drognan", "need", need)
	maxTries := 25
	for t := 0; t < maxTries && need > 0; t++ {
		if err := InteractNPC(npc.Drognan); err != nil {
			ctx.Logger.Error("Failed to interact with Drognan", "error", err)
			continue
		}
		// Trade option for Drognan (first option is trade)
		ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
		utils.Sleep(1000)

		ctx.GameReader.GetData()
		if !ctx.Data.OpenMenus.NPCShop {
			ctx.Logger.Debug("Shop menu not open, closing and retrying")
			step.CloseAllMenus()
			continue
		}

		// Switch to weapons tab (tab 2, zero-indexed)
		SwitchVendorTab(2)
		utils.Sleep(500)
		ctx.GameReader.GetData()

		bought := false
		for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationVendor) {
			if need <= 0 {
				break
			}
			if string(itm.Name) == "BoneWand" {
				if sockets, ok := itm.FindStat(stat.NumSockets, 0); ok && sockets.Value == 2 {
					// Check if it's normal quality (best for White runeword)
					if itm.Quality == item.QualityNormal {
						if ctx.Data.PlayerUnit.TotalPlayerGold() < 5000 {
							ctx.Logger.Info("Not enough gold remaining, stopping Bone Wand shopping")
							break
						}
						ctx.Logger.Info("Buying 2-socket Bone Wand", "name", itm.Name, "quality", itm.Quality)
						town.BuyItem(itm, 1)
						bought = true
						need--
					}
				}
			}
		}
		step.CloseAllMenus()

		if bought {
			ctx.Logger.Info("Bone Wand(s) purchased, refreshing game data.")
			ctx.RefreshGameData()
		}

		if need > 0 {
			// Force shop refresh by zone hop
			ctx.Logger.Debug("Refreshing vendors", "stillNeed", need, "attempt", t+1)
			refreshVendors()
			utils.Sleep(250)
		}
	}

	if need > 0 {
		ctx.Logger.Info("Could not find enough 2-socket Bone Wands", "stillNeed", need)
	} else {
		ctx.Logger.Info("Successfully purchased all needed 2-socket Bone Wands")
	}

	return nil
}
