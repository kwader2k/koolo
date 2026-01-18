package run

import (
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

func (a Leveling) act3() error {
	running := false

	if running || a.ctx.Data.PlayerUnit.Area != area.KurastDocks {
		return nil
	}

	// Try to find Hratli at pier, if he's there, talk to him, so he will move to the normal position later
	hratli, found := a.ctx.Data.Monsters.FindOne(npc.Hratli, data.MonsterTypeNone)
	if found {
		action.InteractNPC(hratli.Name)
	}

	running = true

	action.VendorRefill(action.VendorRefillOpts{SellJunk: true, BuyConsumables: true})

	_, potionFound := a.ctx.Data.Inventory.Find("PotionOfLife", item.LocationInventory)
	q := a.ctx.Data.Quests[quest.Act3TheGoldenBird]
	if (q.Completed() && potionFound) ||
		(!q.NotStarted() && !q.Completed()) {
		a.jadefigurine()
	}

	// Gold Farming Logic for Lower Kurast (and immediate return if farming is needed)
	if action.IsLowGold() {

		a.ctx.Logger.Info("Low on gold. Initiating Lower Kurast Chests gold farm.")
		if err := NewLowerKurastChest().Run(nil); err != nil {
			a.ctx.Logger.Error("Error during Lower Kurast Chests gold farm: %v", err)
			return err
		}
		a.ctx.Logger.Info("Lower Kurast Chests gold farming completed. Quitting current run to re-evaluate in next game.")
		return nil
	}

	/*if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Hell {

		NewMausoleum().Run()
		err := action.WayPoint(area.KurastDocks)
		if err != nil {
			a.ctx.Logger.Error(fmt.Sprintf("Waypoint to Lut Gholein failed after farming: %s.", err.Error()))
		}
	}*/

	if a.ctx.Data.Quests[quest.Act3TheGuardian].Completed() {

		a.ctx.Logger.Info("Attempting to reach Act 4 via The Pandemonium Fortress waypoint.")
		err := action.WayPoint(area.ThePandemoniumFortress)
		if err == nil {
			a.ctx.Logger.Info("Successfully reached Act 4 via waypoint. Ending Act 3 script.")
			return nil
		} else {
			a.ctx.Logger.Info("Could not use waypoint to The Pandemonium Fortress. Falling back to manual portal entry.")
		}

		// Use waypoint to DuranceOfHateLevel2
		err = action.WayPoint(area.DuranceOfHateLevel2)
		if err != nil {
			return err
		}

		// Move to DuranceOfHateLevel3
		if err = action.MoveToArea(area.DuranceOfHateLevel3); err != nil {
			return err
		}

		a.ctx.Logger.Debug("Moving to bridge")
		action.MoveToCoords(data.Position{X: 17588, Y: 8068})
		//Wait for bridge to rise
		utils.Sleep(1000)

		a.ctx.Logger.Debug("Moving to red portal")
		portal, _ := a.ctx.Data.Objects.FindOne(object.HellGate)
		action.MoveToCoords(portal.Position)

		action.InteractObject(portal, func() bool {
			return a.ctx.Data.PlayerUnit.Area == area.ThePandemoniumFortress
		})

		utils.Sleep(500)
		a.HoldKey(win.VK_SPACE, 3000)
		utils.Sleep(500)

		return nil

	}

	_, willFound := a.ctx.Data.Inventory.Find("KhalimsWill", item.LocationInventory, item.LocationStash, item.LocationEquipped)

	if a.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		a.ctx.Logger.Info("Khalims will completed. Starting Mephisto.")
		a.ctx.CharacterCfg.Game.Mephisto.OpenChests = false
		a.ctx.CharacterCfg.Game.Mephisto.KillCouncilMembers = false
		a.ctx.CharacterCfg.Game.Mephisto.ExitToA4 = true
		err := NewMephisto(nil).Run(nil)
		if err != nil {
			a.ctx.Logger.Error("Mephisto run failed, ending Act 3 script.", err)
			return err
		}
		// If Mephisto run completes (successfully or not, but without an explicit error from NewMephisto),
		// check if The Guardian quest is completed and attempt to go to Act 4.
		if a.ctx.Data.Quests[quest.Act3TheGuardian].Completed() {
			hellgate, found := a.ctx.Data.Objects.FindOne(object.HellGate)
			if !found {
				a.ctx.Logger.Info("Gate to Pandemonium Fortress not found after killing Mephisto. Ending script.")
				return nil
			}
			err = action.InteractObject(hellgate, func() bool {
				utils.Sleep(500)
				return a.ctx.Data.PlayerUnit.Area == area.ThePandemoniumFortress
			})
			if err != nil {
				a.ctx.Logger.Error("Failed to interact with Hell Gate, ending Act 3 script.", err)
				return err //
			}
			a.ctx.Logger.Info("Successfully interacted with Hell Gate. Attempting to skip cinematic.")
			utils.Sleep(500)
			a.HoldKey(win.VK_SPACE, 3000)
			utils.Sleep(500)
			// If we successfully interacted with the Hell Gate, we assume the attempt to go to A4 is complete.
			a.ctx.Logger.Info("Successfully attempted to enter Act 4. Ending Act 3 script.")
			return nil
		} else {
			a.ctx.Logger.Info("Mephisto run completed, but 'The Guardian' quest is not marked as complete. Ending Act 3 script.")
			return nil
		}
	}

	// This block only runs if the character is NOT ready for Mephisto (based on the previous `if` condition)
	// and checks if Khalim's Will is already found (skipping artifact collection).
	if willFound {
		a.ctx.Logger.Debug("Khalim's Will found (or quest completed), skipping Khalims Will quests")
		// This still needs to be here to handle the case where Will is found but not yet used to open stairs
		err := a.openMephistoStairs()
		if err != nil {
			return err // Return any error from opening Mephisto stairs
		}
		// After attempting to open Mephisto stairs, we should check if The Guardian quest is complete
		// and try to enter Act 4, then exit.
		if a.ctx.Data.Quests[quest.Act3TheGuardian].Completed() {
			hellgate, found := a.ctx.Data.Objects.FindOne(object.HellGate)
			if !found {
				a.ctx.Logger.Info("Gate to Pandemonium Fortress not found after using Khalim's Will. Ending script.")
				return nil // Exit if portal not found
			}
			err = action.InteractObject(hellgate, func() bool {
				utils.Sleep(500)
				utils.Sleep(1000)
				a.HoldKey(win.VK_SPACE, 3000) // Hold the Escape key (VK_ESCAPE or 0x1B) for 2000 milliseconds (2 seconds)
				utils.Sleep(1000)
				return a.ctx.Data.PlayerUnit.Area == area.ThePandemoniumFortress
			})
			if err != nil {
				utils.Sleep(1000)
				a.HoldKey(win.VK_SPACE, 3000) // Hold the Escape key (VK_ESCAPE or 0x1B) for 2000 milliseconds (2 seconds)
				utils.Sleep(1000)
				return err // Exit on error interacting with portal
			}
			utils.Sleep(1000)
			a.HoldKey(win.VK_SPACE, 3000) // Hold the Escape key (VK_ESCAPE or 0x1B) for 2000 milliseconds (2 seconds)
			utils.Sleep(1000)
			return nil // Exit if successfully interacted with portal
		}

		a.ctx.Logger.Info("Khalim's Will found and used, but Mephisto not yet dead. Ending Act 3 script for this run.")
		return nil // Exit gracefully
	}

	// Find KhalimsEye
	_, found = a.ctx.Data.Inventory.Find("KhalimsEye", item.LocationInventory, item.LocationStash)
	if found {
		a.ctx.Logger.Info("KhalimsEye found, skipping quest")
	} else if !willFound && !a.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		a.ctx.Logger.Info("KhalimsEye not found, starting quest")
		a.findKhalimsEye()
	}

	// Find KhalimsBrain
	_, found = a.ctx.Data.Inventory.Find("KhalimsBrain", item.LocationInventory, item.LocationStash)
	if found {
		a.ctx.Logger.Info("KhalimsBrain found, skipping quest")
	} else if !willFound && !a.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		a.ctx.Logger.Info("KhalimsBrain not found, starting quest")
		NewEndugu().Run(nil)
		// Sometimes it doesn't pick up the brain
		utils.Sleep(500)
		action.ItemPickup(10)
	}

	// Find KhalimsHeart
	_, found = a.ctx.Data.Inventory.Find("KhalimsHeart", item.LocationInventory, item.LocationStash)
	if found {
		a.ctx.Logger.Info("KhalimsHeart found, skipping quest")
	} else if !willFound && !a.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		a.ctx.Logger.Info("KhalimsHeart not found, starting quest")
		a.findKhalimsHeart()
	}

	// Find KhalimsFlail
	_, found = a.ctx.Data.Inventory.Find("KhalimsFlail", item.LocationInventory, item.LocationStash)
	if found {
		// Trav
		// If flail is found, it means all parts are likely collected, so we try to open Mephisto stairs.
		a.openMephistoStairs()
		// After attempting to open Mephisto stairs, we should check if The Guardian quest is complete
		// and try to enter Act 4, then exit.
		if a.ctx.Data.Quests[quest.Act3TheGuardian].Completed() {
			hellgate, found := a.ctx.Data.Objects.FindOne(object.HellGate)
			if !found {
				a.ctx.Logger.Info("Gate to Pandemonium Fortress not found after using Khalim's Will. Ending script.")
				return nil // Exit if portal not found
			}
			err := action.InteractObject(hellgate, func() bool {
				utils.Sleep(500)
				utils.Sleep(1000)
				a.HoldKey(win.VK_SPACE, 2000) // Hold the Escape key (VK_ESCAPE or 0x1B) for 2000 milliseconds (2 seconds)
				utils.Sleep(1000)
				return a.ctx.Data.PlayerUnit.Area == area.ThePandemoniumFortress
			})
			if err != nil {
				utils.Sleep(1000)
				a.HoldKey(win.VK_SPACE, 2000) // Hold the Escape key (VK_ESCAPE or 0x1B) for 2000 milliseconds (2 seconds)
				utils.Sleep(1000)
				return err // Exit on error interacting with portal
			}
			utils.Sleep(1000)
			a.HoldKey(win.VK_SPACE, 2000) // Hold the Escape key (VK_ESCAPE or 0x1B) for 2000 milliseconds (2 seconds)
			utils.Sleep(1000)
			return nil // Exit if successfully interacted with portal
		}
		a.ctx.Logger.Info("Khalim's Flail found and Mephisto stairs opened, but Mephisto not yet dead. Ending Act 3 script for this run.")
		return nil // Exit gracefully
	} else if !willFound && !a.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		a.ctx.Logger.Info("KhalimsFlail not found, starting quest")
		if err := NewTravincal().Run(nil); err != nil {
			return err
		}
	}

	return nil
}

func (a Leveling) findKhalimsEye() error {
	err := action.WayPoint(area.SpiderForest)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.SpiderCavern)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveTo(func() (data.Position, bool) {
		for _, o := range a.ctx.Data.Objects {
			if o.Name == object.InifussTree {
				return o.Position, true
			}
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	err = action.MoveTo(func() (data.Position, bool) {
		chest, found := a.ctx.Data.Objects.FindOne(object.KhalimChest3)
		if found {
			a.ctx.Logger.Info("Khalm Chest found, moving to that room")
			return chest.Position, true
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())

	kalimchest3, found := a.ctx.Data.Objects.FindOne(object.KhalimChest3)
	if !found {
		a.ctx.Logger.Debug("Khalim Chest not found")
	}

	err = action.InteractObject(kalimchest3, func() bool {
		chest, _ := a.ctx.Data.Objects.FindOne(object.KhalimChest3)
		return !chest.Selectable
	})
	if err != nil {
		return err
	}

	return nil
}

func (a Leveling) findKhalimsHeart() error {
	err := action.WayPoint(area.KurastBazaar)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.SewersLevel1Act3)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveTo(func() (data.Position, bool) {
		for _, l := range a.ctx.Data.AdjacentLevels {
			if l.Area == area.SewersLevel2Act3 {
				return l.Position, true
			}
		}
		return data.Position{}, false
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())

	stairs, found := a.ctx.Data.Objects.FindOne(object.Act3SewerStairsToLevel3)
	if !found {
		a.ctx.Logger.Debug("Khalim Chest not found")
	}

	err = action.InteractObject(stairs, func() bool {
		o, _ := a.ctx.Data.Objects.FindOne(object.Act3SewerStairsToLevel3)

		return !o.Selectable
	})
	if err != nil {
		return err
	}

	time.Sleep(4000 * time.Millisecond)

	err = action.MoveToArea(area.SewersLevel2Act3)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveTo(func() (data.Position, bool) {
		a.ctx.Logger.Info("Khalm Chest found, moving to that room")
		chest, found := a.ctx.Data.Objects.FindOne(object.KhalimChest1)

		return chest.Position, found
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())

	kalimchest1, found := a.ctx.Data.Objects.FindOne(object.KhalimChest1)
	if !found {
		a.ctx.Logger.Debug("Khalim Chest not found")
	}

	err = action.InteractObject(kalimchest1, func() bool {
		chest, _ := a.ctx.Data.Objects.FindOne(object.KhalimChest1)
		return !chest.Selectable
	})
	if err != nil {
		return err
	}

	return nil
}

func (a Leveling) prepareWill() error {
	return prepareKhalimsWill(a.ctx)
}

func (a Leveling) openMephistoStairs() error {
	// Use Travincal/Council run to kill the council
	if err := a.prepareWill(); err != nil {
		return err
	}

	if !a.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		if _, _, err := ensureQuestWeaponEquipped(a.ctx, "KhalimsWill", swapWeaponSlot); err != nil {
			return err
		}
	}

	err := action.WayPoint(area.Travincal)
	if err != nil {
		return err
	}

	// Interact with the Compelling Orb to open the stairs
	compellingorb, found := a.ctx.Data.Objects.FindOne(object.CompellingOrb)
	if !found {
		a.ctx.Logger.Debug("Compelling Orb not found")
	}
	// Swap weapons, equip Khalim's Will

	action.MoveToCoords(compellingorb.Position)
	if !a.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		if err := withQuestWeaponSlot(a.ctx, "KhalimsWill", func() error {
			if err := action.InteractObject(compellingorb, func() bool {
				o, _ := a.ctx.Data.Objects.FindOne(object.CompellingOrb)
				return !o.Selectable
			}); err != nil {
				return err
			}
			utils.Sleep(300)
			return nil
		}); err != nil {
			return err
		}
	}

	// Wait for quest/stairs state update, but do not block longer than needed.
	waitDeadline := time.Now().Add(12 * time.Second)
	for {
		a.ctx.RefreshGameData()
		if a.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
			break
		}
		if _, found := a.ctx.Data.Objects.FindOne(object.StairSR); found {
			break
		}
		if time.Now().After(waitDeadline) {
			break
		}
		time.Sleep(250 * time.Millisecond)
	}

	if a.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		// Interact with the stairs to go to Durance of Hate Level 1
		stairsr, found := a.ctx.Data.Objects.FindOne(object.StairSR)
		if !found {
			a.ctx.Logger.Debug("Stairs to Durance not found")
		}

		err := action.InteractObject(stairsr, func() bool {
			return a.ctx.Data.PlayerUnit.Area == area.DuranceOfHateLevel1
		})
		if err != nil {
			return err
		}

		// Move to Durance of Hate Level 2 and discover the waypoint
		action.MoveToArea(area.DuranceOfHateLevel2)
		err = action.DiscoverWaypoint()
		if err != nil {
			return err
		}
	}

	return nil
}

func (a Leveling) jadefigurine() error {
	_, jadefigureFound := a.ctx.Data.Inventory.Find("AJadeFigurine", item.LocationInventory)
	if jadefigureFound {
		action.InteractNPC(npc.Meshif2)
	}

	_, goldenbirdFound := a.ctx.Data.Inventory.Find("TheGoldenBird", item.LocationInventory)
	if goldenbirdFound {
		// Talk to Alkor
		action.InteractNPC(npc.Alkor)
		action.InteractNPC(npc.Ormus)
		action.InteractNPC(npc.Alkor)
	}
	utils.Sleep(500)
	lifepotion, lifepotfound := a.ctx.Data.Inventory.Find("PotionOfLife", item.LocationInventory)
	if lifepotfound {
		a.ctx.HID.PressKeyBinding(a.ctx.Data.KeyBindings.Inventory)
		screenPos := ui.GetScreenCoordsForItem(lifepotion)
		a.ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
		step.CloseAllMenus()
	}
	return nil
}
