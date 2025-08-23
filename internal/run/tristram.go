package run

import (
	"errors"
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)


var TristramlStartingPosition = data.Position{
	X: 25173,
	Y: 5087,
}

var TristramClearPos1 = data.Position{
	X: 25173,
	Y: 5113,
}

var TristramClearPos2 = data.Position{
	X: 25175,
	Y: 5166,
}

var TristramClearPos3 = data.Position{
	X: 25163,
	Y: 5192,
}	
var TristramClearPos4 = data.Position{
	X: 25139,
	Y: 5186,
}
var TristramClearPos5 = data.Position{
	X: 25126,
	Y: 5167,
}
var TristramClearPos6 = data.Position{
	X: 25122,
	Y: 5151,
}
var TristramClearPos7 = data.Position{
	X: 25123,
	Y: 5140,	
}


type Tristram struct {
	ctx *context.Status
}

func NewTristram() *Tristram {
	return &Tristram{
		ctx: context.Get(),
	}
}

func (t Tristram) Name() string {
	return string(config.TristramRun)
}

func (t Tristram) Run() error {

	// Use waypoint to StonyField
	err := action.WayPoint(area.StonyField)
	if err != nil {
		return err
	}

	// Find the Cairn Stone Alpha
	cairnStone := data.Object{}
	for _, o := range t.ctx.Data.Objects {
		if o.Name == object.CairnStoneAlpha {
			cairnStone = o
		}
	}

	// Move to the cairnStone
	action.MoveToCoords(cairnStone.Position)

	// Clear area around the portal
	_, isLevelingChar := t.ctx.Char.(context.LevelingCharacter)
	if t.ctx.CharacterCfg.Game.Tristram.ClearPortal || isLevelingChar && t.ctx.CharacterCfg.Game.Difficulty == difficulty.Nightmare {
		action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())
	}

	// Handle opening Tristram Portal, will be skipped if its already opened
	if err = t.openPortalIfNotOpened(); err != nil {
		return err
	}

	// Enter Tristram portal

	// Find the portal object
	tristPortal, _ := t.ctx.Data.Objects.FindOne(object.PermanentTownPortal)

	// Interact with the portal
	if err = action.InteractObject(tristPortal, func() bool {
		return t.ctx.Data.PlayerUnit.Area == area.Tristram && t.ctx.Data.AreaData.IsInside(t.ctx.Data.PlayerUnit.Position)
	}); err != nil {
		return err
	}

	// Open a TP if we're the leader
	action.OpenTPIfLeader()

	// Check if Cain is rescued
	if o, found := t.ctx.Data.Objects.FindOne(object.CainGibbet); found && o.Selectable {

		// Move to cain
		action.MoveToCoords(o.Position)

		action.InteractObject(o, func() bool {
			obj, _ := t.ctx.Data.Objects.FindOne(object.CainGibbet)

			return !obj.Selectable
		})
	} else {

						t.ctx.CharacterCfg.Character.ClearPathDist = 25
	if err := config.SaveSupervisorConfig(t.ctx.CharacterCfg.ConfigFolderName, t.ctx.CharacterCfg); err != nil {
		t.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())}
		
		t.ctx.Logger.Info("Clearing Tristram")
		action.MoveToCoords(TristramlStartingPosition)
		action.ClearAreaAroundPlayer(25, data.MonsterAnyFilter())
		action.MoveToCoords(TristramClearPos1)
		action.ClearAreaAroundPlayer(25, data.MonsterAnyFilter())
		action.MoveToCoords(TristramClearPos2)
		action.ClearAreaAroundPlayer(35, data.MonsterAnyFilter())
		action.MoveToCoords(TristramClearPos3)
		action.ClearAreaAroundPlayer(40, data.MonsterAnyFilter())
		action.MoveToCoords(TristramClearPos4)
		action.ClearAreaAroundPlayer(40, data.MonsterAnyFilter())
		action.MoveToCoords(TristramClearPos5)
		action.ClearAreaAroundPlayer(40, data.MonsterAnyFilter())
		action.MoveToCoords(TristramClearPos6)
		action.ClearAreaAroundPlayer(40, data.MonsterAnyFilter())
		action.MoveToCoords(TristramClearPos7)
		action.ClearAreaAroundPlayer(40, data.MonsterAnyFilter())
	
	}

	return nil
}

func (t Tristram) openPortalIfNotOpened() error {

	// If the portal already exists, skip this
	if _, found := t.ctx.Data.Objects.FindOne(object.PermanentTownPortal); found {
		return nil
	}

	t.ctx.Logger.Debug("Tristram portal not detected, trying to open it")

	for range 6 {
		stoneTries := 0
		activeStones := 0
		for _, cainStone := range []object.Name{
			object.CairnStoneAlpha,
			object.CairnStoneBeta,
			object.CairnStoneGamma,
			object.CairnStoneDelta,
			object.CairnStoneLambda,
		} {
			st := cainStone
			stone, _ := t.ctx.Data.Objects.FindOne(st)
			if stone.Selectable {

				action.InteractObject(stone, func() bool {

					if stoneTries < 5 {
						stoneTries++
						utils.Sleep(200)
						x, y := t.ctx.PathFinder.GameCoordsToScreenCords(stone.Position.X, stone.Position.Y)
						t.ctx.HID.Click(game.LeftButton, x+3*stoneTries, y)
						t.ctx.Logger.Debug(fmt.Sprintf("Tried to click %s at screen pos %vx%v", stone.Desc().Name, x, y))
						return false
					}
					stoneTries = 0
					return true
				})

			} else {
				utils.Sleep(200)
				activeStones++
			}
			_, tristPortal := t.ctx.Data.Objects.FindOne(object.PermanentTownPortal)
			if activeStones >= 5 || tristPortal {
				break
			}
		}

	}

	// Wait upto 15 seconds for the portal to open, checking every second if its up
	for range 15 {
		// Wait a second
		utils.Sleep(1000)

		if _, portalFound := t.ctx.Data.Objects.FindOne(object.PermanentTownPortal); portalFound {
			return nil
		}
	}

	return errors.New("failed to open Tristram portal")
}
