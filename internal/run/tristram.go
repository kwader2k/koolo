package run

import (
	"errors"
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

var areas = []data.Position{
	{X: 25173, Y: 5087}, // TristramlStartingPosition
	{X: 25173, Y: 5113}, // TristramClearPos1
	{X: 25175, Y: 5166}, // TristramClearPos2
	{X: 25163, Y: 5192}, // TristramClearPos3
	{X: 25139, Y: 5186}, // TristramClearPos4
	{X: 25126, Y: 5167}, // TristramClearPos5
	{X: 25122, Y: 5151}, // TristramClearPos6
	{X: 25123, Y: 5140}, // TristramClearPos7
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

func (t Tristram) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) {
		return SequencerError
	}
	if !t.ctx.Data.Quests[quest.Act1TheSearchForCain].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (t Tristram) Run(parameters *RunParameters) error {

	if t.shouldTakeRejuvsAndLeave() {
		return nil
	}

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

	if t.shouldTakeRejuvsAndLeave() {
		return nil
	}

	// Clear area around the portal
	_, isLevelingChar := t.ctx.Char.(context.LevelingCharacter)
	if t.ctx.CharacterCfg.Game.Tristram.ClearPortal || isLevelingChar && t.ctx.CharacterCfg.Game.Difficulty == difficulty.Nightmare {
		action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())

		if t.shouldTakeRejuvsAndLeave() {
			return nil
		}
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
	}

	t.ctx.CharacterCfg.Character.ClearPathDist = 25
	if err := config.SaveSupervisorConfig(t.ctx.CharacterCfg.ConfigFolderName, t.ctx.CharacterCfg); err != nil {
		t.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())
	}

	t.ctx.Logger.Info("Clearing Tristram")
	for _, pos := range areas {
		if t.shouldTakeRejuvsAndLeave() {
			return nil
		}
		action.MoveToCoords(pos)
		action.ClearAreaAroundPlayer(40, data.MonsterAnyFilter())
	}

	return nil
}

func (t Tristram) shouldTakeRejuvsAndLeave() bool {
	if t.ctx.CharacterCfg.Game.Tristram.OnlyFarmRejuvs {
		missingRejuvPotionCount := t.ctx.BeltManager.GetMissingCount(data.RejuvenationPotion)
		t.ctx.Logger.Debug("missing rejuv potions", "count", missingRejuvPotionCount)
		return missingRejuvPotionCount == 0
	} else {
		t.ctx.Logger.Debug("Goodbye Tristram, and thanks for all the juvs!")
		return false
	}
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
			object.CairnStoneGamma,
			object.CairnStoneBeta,
			object.CairnStoneLambda,
			object.CairnStoneDelta,
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
