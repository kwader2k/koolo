package run

import (
	"fmt"
	"time"

	"github.com/hectorgimenez/koolo/internal/action/step"

	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config" // Make sure this import is present
	//	"github.com/lxn/win"
)

func (a Leveling) act5() error {
	if a.ctx.Data.PlayerUnit.Area != area.Harrogath && a.ctx.Data.PlayerUnit.Area != area.FrozenRiver {
		return nil
	}

	action.UpdateQuestLog()

	action.VendorRefill(true, true)

	// Gold Farming Logic (and immediate return if farming is needed)
	if (a.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal && a.ctx.Data.PlayerUnit.TotalPlayerGold() < 30000) ||
		(a.ctx.CharacterCfg.Game.Difficulty == difficulty.Nightmare && a.ctx.Data.PlayerUnit.TotalPlayerGold() < 50000) ||
		(a.ctx.CharacterCfg.Game.Difficulty == difficulty.Hell && a.ctx.Data.PlayerUnit.TotalPlayerGold() < 70000) {

		a.ctx.Logger.Info("Low on gold. Initiating Crystalline Passage gold farm.")
		if err := a.CrystallinePassage(); err != nil {
			a.ctx.Logger.Error("Error during Crystalline Passage gold farm: %v", err)
			return err // Propagate error if farming fails
		}
		a.ctx.Logger.Info("Gold farming completed. Quitting current run to re-evaluate in next game.")
		return nil // Key: This immediately exits the 'act5' function, ending the current game run.
	}
	// If we reach this point, it means gold is sufficient, and we skip farming for this run.

	lvl, _ := a.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)

	// Use a flag to indicate if difficulty was changed and needs saving
	difficultyChanged := false

	// Logic for Act5EveOfDestruction quest completion
	if a.ctx.Data.Quests[quest.Act5EveOfDestruction].Completed() {

		a.ctx.Logger.Info("Eve of Destruction completed")

		currentDifficulty := a.ctx.CharacterCfg.Game.Difficulty
		switch currentDifficulty {
		case difficulty.Normal:
			if lvl.Value >= 41 {
				a.ctx.CharacterCfg.Game.Difficulty = difficulty.Nightmare
				difficultyChanged = true
			}
		case difficulty.Nightmare:
			// Get current FireResist and LightningResist values using FindStat on PlayerUnit
			rawFireRes, _ := a.ctx.Data.PlayerUnit.FindStat(stat.FireResist, 0)
			rawLightRes, _ := a.ctx.Data.PlayerUnit.FindStat(stat.LightningResist, 0)

			// Apply Nightmare difficulty penalty (-40) to resistances for effective values
			effectiveFireRes := rawFireRes.Value - 40
			effectiveLightRes := rawLightRes.Value - 40

			// Check conditions using effective resistance values
			if lvl.Value >= 70 && effectiveFireRes >= 75 && effectiveLightRes >= 50 {
				a.ctx.CharacterCfg.Game.Difficulty = difficulty.Hell

				difficultyChanged = true
			}
		}

		if difficultyChanged {
			a.ctx.Logger.Info("Difficulty changed to %s. Saving character configuration...", a.ctx.CharacterCfg.Game.Difficulty)
			// Use the new ConfigFolderName field here!
			if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
				a.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())
				return fmt.Errorf("failed to save character configuration: %w", err)
			}
			return nil
		}
	}

	if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Nightmare && lvl.Value < 60 || a.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal && lvl.Value < 33 {

		diabloRun := NewDiablo()
		err := diabloRun.Run()
		if err != nil {
			return err
		}
	}

	// Logic for Act5RiteOfPassage quest completion
	if a.ctx.Data.Quests[quest.Act5RiteOfPassage].Completed() && a.ctx.Data.Quests[quest.Act5PrisonOfIce].Completed() {
		a.ctx.Logger.Info("Starting Baal run...")
		if a.ctx.CharacterCfg.Game.Difficulty != difficulty.Normal {
			a.ctx.CharacterCfg.Game.Baal.SoulQuit = true
		}
		NewBaal(nil).Run()

		return nil
	}

	wp, _ := a.ctx.Data.Objects.FindOne(object.ExpansionWaypoint)
	action.MoveToCoords(wp.Position)

	anyaQuest := a.ctx.Data.Quests[quest.Act5PrisonOfIce]
	_, anyaInTown := a.ctx.Data.Monsters.FindOne(npc.Drehya, data.MonsterTypeNone)

	if !anyaQuest.Completed() {
		_, hasPotion := a.ctx.Data.Inventory.Find("Malah's Potion")

		if !anyaInTown {
			if !anyaQuest.Completed() {
				a.ctx.Logger.Info("Step 1: Quest has not been started. Going to Anya in the Frozen River.")
				NewQuests().rescueAnyaQuest()
				action.MoveToCoords(data.Position{X: 5107, Y: 5119})
				action.InteractNPC(npc.Drehya)
				utils.Sleep(500)
				step.OpenPortal()
				for i := 0; i < 5; i++ {
					if _, pFound := a.ctx.Data.Objects.FindOne(object.TownPortal); pFound {
						break
					}
					time.Sleep(time.Second)
				}
				if portal, pFound := a.ctx.Data.Objects.FindOne(object.TownPortal); pFound {
					action.InteractObject(portal, nil)
				}
				return nil
			}

			if a.ctx.Data.PlayerUnit.Area == area.Harrogath {
				if !hasPotion {
					a.ctx.Logger.Info("Step 2: Talking to Malah for the potion.")
					action.InteractNPC(npc.Malah)
					utils.Sleep(500)
					return nil
				}

				a.ctx.Logger.Info("Step 3: Returning to Anya with the potion.")
				if portal, found := a.ctx.Data.Objects.FindOne(object.TownPortal); found {
					action.MoveToCoords(portal.Position)
					action.InteractObject(portal, nil)
					action.UsePortalInTown()
					return nil
				}
				// FALLBACK: If portal is gone walk back.
				a.ctx.Logger.Warn("Portal not found! Using waypoint fallback.")
				action.WayPoint(area.CrystallinePassage)
				NewQuests().rescueAnyaQuest()
				return nil
			}

			if a.ctx.Data.PlayerUnit.Area == area.FrozenRiver && hasPotion {
				a.ctx.Logger.Info("Step 3: Thawing Anya.")
				action.MoveToCoords(data.Position{X: 5107, Y: 5119})
				action.InteractNPC(npc.Drehya)
				utils.Sleep(1500)
				if portal, found := a.ctx.Data.Objects.FindOne(object.PermanentTownPortal); found {
					action.InteractObject(portal, nil)
					action.ReturnTown()
					return nil
				}
				step.OpenPortal()
				for i := 0; i < 5; i++ {
					if _, pFound := a.ctx.Data.Objects.FindOne(object.TownPortal); pFound {
						break
					}
					time.Sleep(time.Second)
				}
				if portal, pFound := a.ctx.Data.Objects.FindOne(object.TownPortal); pFound {
					action.InteractObject(portal, nil)
				}
				return nil
			}
		} else { // Anya is in town, but the quest is not complete. Force the final steps.
			a.ctx.Logger.Info("Step 4: Anya is in town. Talking to Malah for reward.")
			// Move close to Malah before interacting
			if malah, found := a.ctx.Data.Monsters.FindOne(npc.Malah, data.MonsterTypeNone); found {
				action.MoveToCoords(malah.Position)
			}
			action.InteractNPC(npc.Malah)

			// Adding a longer delay to ensure the game state has time to update
			utils.Sleep(2500)

			a.ctx.Logger.Info("Step 5: Talking to Anya to complete the quest.")
			// Using static coordinates for Anya as dynamic detection is failing.
			anyaPosition := data.Position{X: 5130, Y: 5120}
			action.MoveToCoords(anyaPosition)

			action.InteractNPC(npc.Drehya)
			utils.Sleep(1000)

			// End the run here. This ensures the quest completion is registered before the next run starts.
			return nil
		}
	}

	if _, found := a.ctx.Data.Inventory.Find("ScrollOfResistance"); found {
		a.ctx.Logger.Info("ScrollOfResistance found in inventory, attempting to use it.")
		step.CloseAllMenus()
		a.ctx.HID.PressKeyBinding(a.ctx.Data.KeyBindings.Inventory)
		utils.Sleep(500) // Give time for inventory to open and data to refresh

		// Re-find the item after opening inventory to ensure correct screen position
		if itm, foundAgain := a.ctx.Data.Inventory.Find("ScrollOfResistance"); foundAgain {
			screenPos := ui.GetScreenCoordsForItem(itm)
			utils.Sleep(200)
			a.ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
			utils.Sleep(500) // Give time for the scroll to be used
			a.ctx.Logger.Info("ScrollOfResistance used.")
		} else {
			a.ctx.Logger.Warn("ScrollOfResistance disappeared from inventory before it could be used.")
		}
		step.CloseAllMenus() // Close inventory after attempt
	}

	if !a.ctx.Data.Quests[quest.Act5RiteOfPassage].Completed() {
		err := NewQuests().killAncientsQuest()
		if err != nil {
			return err
		}
	}

	return nil
}

func (a Leveling) CrystallinePassage() error {
	a.ctx.Logger.Info("Entering Crystalline Passage for gold farming...")

	err := action.WayPoint(area.CrystallinePassage)
	if err != nil {
		a.ctx.Logger.Error("Failed to move to Crystalline Passage area: %v", err)
		return err
	}
	a.ctx.Logger.Info("Successfully reached Crystalline Passage.")

	err = action.ClearCurrentLevel(false, data.MonsterAnyFilter())
	if err != nil {
		a.ctx.Logger.Error("Failed to clear Crystalline Passage area: %v", err)
		return err
	}
	a.ctx.Logger.Info("Successfully cleared Crystalline Passage area.")

	return nil

}
