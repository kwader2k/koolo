package run

import (
	"errors"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Anya struct {
	ctx *context.Status
}

func NewAnya() *Anya {
	return &Anya{
		ctx: context.Get(),
	}
}

func (a Anya) Name() string {
	return string(config.AnyaRun)
}

func (a Anya) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}
	if !a.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() {
		return SequencerStop
	}
	if _, found := a.ctx.Data.Inventory.Find("ScrollOfResistance"); found {
		return SequencerOk
	}
	if a.ctx.Data.Quests[quest.Act5PrisonOfIce].Completed() {
		a5q4 := a.ctx.Data.Quests[quest.Act5BetrayalOfHarrogath]
		if !a5q4.NotStarted() || a5q4.Completed() {
			return SequencerSkip
		}
	}
	return SequencerOk
}

func (a Anya) Run(parameters *RunParameters) error {
	a.ctx.Logger.Info("Starting Rescuing Anya Quest...")

	//Quest already complete, try finish (use scroll, talk to anya)
	if a.ctx.Data.Quests[quest.Act5PrisonOfIce].Completed() {
		err := action.InteractNPC(npc.Malah)
		if err != nil {
			return err
		}

		return a.finishQuest()
	}

	//Go to anya in frozen river
	err := action.WayPoint(area.CrystallinePassage)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.FrozenRiver)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveTo(func() (data.Position, bool) {
		anya, found := a.ctx.Data.NPCs.FindOne(793)
		return anya.Positions[0], found
	})
	if err != nil {
		return err
	}

	a.ctx.RefreshGameData()
	utils.Sleep(200)

	err = action.MoveTo(func() (data.Position, bool) {
		anya, found := a.ctx.Data.Objects.FindOne(object.FrozenAnya)
		return anya.Position, found
	})
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(15, data.MonsterAnyFilter())

	anya, found := a.ctx.Data.Objects.FindOne(object.FrozenAnya)
	if !found {
		a.ctx.Logger.Debug("Frozen Anya not found")
	}

	//If potion is not in inventory, interact with anya & go get it
	if !a.hasPotion() {
		for range 3 {
			utils.Sleep(300)
			err = action.InteractObject(anya, nil)
			if err != nil {
				return err
			}
		}
		utils.Sleep(300)

		err = action.ReturnTown()
		if err != nil {
			return err
		}

		action.IdentifyAll(false)
		action.Stash(false)
		action.ReviveMerc()
		action.Repair()
		action.VendorRefill(action.VendorRefillOpts{SellJunk: true, BuyConsumables: true})

		for range 5 {
			err = action.InteractNPC(npc.Malah)
			if err != nil {
				return err
			}
			a.ctx.RefreshGameData()
			utils.Sleep(200)
			if a.hasScroll() || a.hasPotion() {
				break
			}
		}

		if !a.hasScroll() && !a.hasPotion() {
			return errors.New("failed to get potion from malah")
		}

		err = action.UsePortalInTown()
		if err != nil {
			return err
		}
		utils.Sleep(500)
	}

	//Unfreeze anya
	err = action.InteractObject(anya, nil)
	if err != nil {
		return err
	}

	utils.Sleep(10000)

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	//Get scroll
	for range 10 {
		err = action.InteractNPC(npc.Malah)
		if err != nil {
			return err
		}
		a.ctx.RefreshGameData()
		utils.Sleep(200)
		if a.hasScroll() {
			break
		}
	}

	//Talk to anya
	return a.finishQuest()
}

func (a Anya) hasScroll() bool {
	_, found := a.ctx.Data.Inventory.Find("ScrollOfResistance")
	return found
}

func (a Anya) hasPotion() bool {
	_, found := a.ctx.Data.Inventory.Find("MalahsPotion")
	return found
}

func (a Anya) tryUseScroll() bool {
	if a.hasScroll() {
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
		return true
	}
	return false
}

func (a Anya) finishQuest() error {
	a.tryUseScroll()

	anyaTownPos, found := a.ctx.Data.Objects.FindOne(object.DrehyaTownStartPosition)
	if !found {
		return errors.New("couldn't find anya pos in town")
	}
	action.MoveToCoords(anyaTownPos.Position)
	err := action.InteractNPC(npc.Drehya)
	if err != nil {
		return err
	}

	return nil
}
