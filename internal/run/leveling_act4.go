package run

import (
	"errors"

	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

func ToKeyBinding(keyCode byte) data.KeyBinding {
	return data.KeyBinding{
		Key1: [2]byte{keyCode, 0},
		Key2: [2]byte{0, 0},
	}
}

// holdKey simulates holding a key down for a specified duration.
func (a Leveling) HoldKey(keyCode byte, durationMs int) {
	kb := ToKeyBinding(keyCode)                              // Convert byte to data.KeyBinding
	a.ctx.HID.KeyDown(kb)                                    // Simulate pressing the key down
	time.Sleep(time.Duration(durationMs) * time.Millisecond) // Wait for the specified duration
	a.ctx.HID.KeyUp(kb)                                      // Simulate releasing the key
}

func (a Leveling) act4() error {
	running := false
	if running || a.ctx.Data.PlayerUnit.Area != area.ThePandemoniumFortress {
		return nil
	}

	running = true

	action.VendorRefill(action.VendorRefillOpts{SellJunk: true, BuyConsumables: true})

	rawFireRes, _ := a.ctx.Data.PlayerUnit.FindStat(stat.FireResist, 0)
	rawLightRes, _ := a.ctx.Data.PlayerUnit.FindStat(stat.LightningResist, 0)

	// Apply Nightmare difficulty penalty (-40) to resistances for effective values
	effectiveFireRes := rawFireRes.Value - 40
	effectiveLightRes := rawLightRes.Value - 40

	// Log the effective resistance values
	a.ctx.Logger.Info(fmt.Sprintf("Current effective resistances (Nightmare penalty applied) - Fire: %d, Lightning: %d", effectiveFireRes, effectiveLightRes))

	lvl, _ := a.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
	_, found := a.ctx.Data.Objects.FindOne(object.LastLastPortal)
	if !found && a.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() && ((lvl.Value >= 60 && a.ctx.CharacterCfg.Game.Difficulty == difficulty.Nightmare && effectiveFireRes >= 75 && effectiveLightRes >= 50) || (lvl.Value >= 30 && a.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal)) {
		err := action.InteractNPC(npc.Tyrael2)
		if err != nil {
			return err // It's good practice to handle errors
		}

		harrogathPortal, found := a.ctx.Data.Objects.FindOne(object.LastLastPortal)
		if !found { // portal was already opened before so we must talk to Tyrael to get to A5
			a.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
			// After attempting to open it with key sequence, you should re-check if it's found
			// If still not found, then it's an error.

		}

		err = action.InteractObject(harrogathPortal, func() bool {
			utils.Sleep(100)
			ctx := context.Get()
			return !ctx.Manager.InGame()
		})

		// Skip Cinematic
		utils.Sleep(2000)
		a.HoldKey(win.VK_SPACE, 2000)
		utils.Sleep(2000)
		a.HoldKey(win.VK_SPACE, 2000)

		return nil
	}

	if action.IsLowGold() {

		a.ctx.Logger.Info("Low on gold. Initiating Chest Run.")

		NewLowerKurastChest().Run(nil)

		err := action.WayPoint(area.ThePandemoniumFortress)
		if err != nil {
			return err
		}

		return nil
	}

	if a.ctx.CharacterCfg.Game.Difficulty == difficulty.Hell {

		//Deactivate shrine interaction for late leveling phase (with low gear in hell searching for shrines leads to more problems than benefits)
		if a.ctx.CharacterCfg.Game.InteractWithShrines {
			a.ctx.CharacterCfg.Game.InteractWithShrines = false

			if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
				a.ctx.Logger.Error(fmt.Sprintf("Failed to save character configuration: %s", err.Error()))
			}
		}

		a.ctx.Logger.Info("Under level 90 we assume we must still farm items")

		NewLowerKurastChest().Run(nil)
		NewMephisto(nil).Run(nil)
		NewMausoleum().Run(nil)
		err := action.WayPoint(area.ThePandemoniumFortress)
		if err != nil {
			return err
		}

		diabloRun := NewDiablo()
		err = diabloRun.Run(nil)
		if err != nil {
			return err
		}
	}

	if !a.ctx.Data.Quests[quest.Act4TheFallenAngel].Completed() {
		err := NewQuests().killIzualQuest() // No immediate 'return' here
		a.ctx.Logger.Debug("After Izual attempt, Izual quest completed status: %v", a.ctx.Data.Quests[quest.Act4TheFallenAngel].Completed())
		if err != nil {
			return err
		}
	}

	if (a.ctx.Data.Quests[quest.Act4TheFallenAngel].Completed() && !a.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed()) || (a.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() && a.ctx.CharacterCfg.Game.Difficulty == difficulty.Nightmare && (lvl.Value < 60 || effectiveFireRes < 75 || effectiveLightRes < 50)) || (a.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() && a.ctx.CharacterCfg.Game.Difficulty == difficulty.Normal && lvl.Value < 30) {
		diabloRun := NewDiablo()
		err := diabloRun.Run(nil)
		if err != nil {
			return err
		}
	} else {
		err := action.InteractNPC(npc.Tyrael2)
		if err != nil {
			return err // It's good practice to handle errors
		}

		harrogathPortal, found := a.ctx.Data.Objects.FindOne(object.LastLastPortal)
		if !found { // portal was already opened before so we must talk to Tyrael to get to A5
			a.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
			// After attempting to open it with key sequence, you should re-check if it's found
			// If still not found, then it's an error.

		}

		err = action.InteractObject(harrogathPortal, func() bool {
			utils.Sleep(100)
			ctx := context.Get()
			return !ctx.Manager.InGame()
		})

		// Skip Cinematic
		utils.Sleep(2000)
		a.HoldKey(win.VK_SPACE, 2000)
		utils.Sleep(2000)
		a.HoldKey(win.VK_SPACE, 2000)

		return nil
	}

	a.ctx.Logger.Debug("Current Izual quest completed status: %v", a.ctx.Data.Quests[quest.Act4TheFallenAngel].Completed())

	if !a.ctx.Data.Quests[quest.Act4TheFallenAngel].Completed() {
		err := NewQuests().killIzualQuest() // No immediate 'return' here
		a.ctx.Logger.Debug("After Izual attempt, Izual quest completed status: %v", a.ctx.Data.Quests[quest.Act4TheFallenAngel].Completed())
		if err != nil {
			return err
		}
	}

	if !a.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() {
		diabloRun := NewDiablo()
		err := diabloRun.Run(nil)
		if err != nil {
			return err
		}
	} else {
		err := action.InteractNPC(npc.Tyrael2)
		if err != nil {
			return err
		}
		harrogathPortal, found := a.ctx.Data.Objects.FindOne(object.LastLastPortal)
		if !found {
			return errors.New("portal to Harrogath not found")
		}

		err = action.InteractObject(harrogathPortal, func() bool {
			utils.Sleep(100)
			ctx := context.Get()
			return !ctx.Manager.InGame()
		})

		// Skip Cinematic
		utils.Sleep(2000)
		a.HoldKey(win.VK_SPACE, 2000)
		utils.Sleep(2000)
		a.HoldKey(win.VK_SPACE, 2000)

		return nil
	}
	return nil
}

func (a Leveling) OuterSteppes() error {
	a.ctx.Logger.Debug("Entering OuterSteppes for gold farming...")

	err := action.MoveToArea(area.OuterSteppes)
	if err != nil {
		a.ctx.Logger.Error("Failed to move to Outer Steppes area: %v", err)
		return err
	}
	a.ctx.Logger.Debug("Successfully reached Outer Steppes.")

	err = action.ClearCurrentLevel(false, data.MonsterAnyFilter())
	if err != nil {
		a.ctx.Logger.Error("Failed to clear Outer Steppes area: %v", err)
		return err
	}
	a.ctx.Logger.Debug("Successfully cleared Outer Steppes area.")

	return nil
}
