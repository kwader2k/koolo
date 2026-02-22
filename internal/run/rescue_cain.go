package run

import (
	"errors"
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const ScrollInifussUnitID = 539
const ScrollInifussAfterAkara = 540
const ScrollInifussName = "Scroll of Inifuss"

type RescueCain struct {
	ctx *context.Status
}

func NewRescueCain() *RescueCain {
	return &RescueCain{
		ctx: context.Get(),
	}
}

func (rc RescueCain) Name() string {
	return string(config.RescueCainRun)
}

func (rc RescueCain) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}
	if rc.ctx.Data.Quests[quest.Act1TheSearchForCain].Completed() {
		return SequencerSkip
	}

	return SequencerOk
}

func (rc RescueCain) Run(parameters *RunParameters) error {
	rc.ctx.Logger.Info("Starting Rescue Cain Quest...")

	// --- Navigation to the Dark Wood and a safe zone near the Inifuss Tree ---
	err := action.WayPoint(area.RogueEncampment)
	if err != nil {
		return err
	}

	needToGoToTristram := (rc.ctx.Data.Quests[quest.Act1TheSearchForCain].HasStatus(quest.StatusInProgress2) ||
		rc.ctx.Data.Quests[quest.Act1TheSearchForCain].HasStatus(quest.StatusInProgress3) ||
		rc.ctx.Data.Quests[quest.Act1TheSearchForCain].HasStatus(quest.StatusEnterArea))

	infusInInventory := false
	for _, itm := range rc.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if itm.ID == ScrollInifussUnitID || itm.ID == ScrollInifussAfterAkara {
			infusInInventory = true
			break
		}
	}

	if !infusInInventory && !needToGoToTristram {
		err = rc.gatherInfussScroll()
		if err != nil {
			return err
		}
		infusInInventory = true
	}

	if infusInInventory {
		err = action.InteractNPC(npc.Akara)
		if err != nil {
			return err
		}

		step.CloseAllMenus()
	}

	// Use waypoint to StonyField
	err = action.WayPoint(area.StonyField)
	if err != nil {
		return err
	}

	// Find the Cairn Stone Alpha
	cairnStone := data.Object{}
	for _, o := range rc.ctx.Data.Objects {
		if o.Name == object.CairnStoneAlpha {
			cairnStone = o
		}
	}

	// Move to the cairnStone
	action.MoveToCoords(cairnStone.Position)
	action.ClearAreaAroundPlayer(10, data.MonsterAnyFilter())

	// Handle opening Tristram Portal, will be skipped if its already opened
	if err = rc.openPortalIfNotOpened(); err != nil {
		return err
	}

	// Enter Tristram portal
	// Find the portal object
	tristPortal, _ := rc.ctx.Data.Objects.FindOne(object.PermanentTownPortal)

	// Interact with the portal
	if err = action.InteractObject(tristPortal, func() bool {
		return rc.ctx.Data.PlayerUnit.Area == area.Tristram && rc.ctx.Data.AreaData.IsInside(rc.ctx.Data.PlayerUnit.Position)
	}); err != nil {
		return err
	}

	// Check if Cain is rescued
	if o, found := rc.ctx.Data.Objects.FindOne(object.CainGibbet); found && o.Selectable {

		// Move to cain
		action.MoveToCoords(o.Position)

		action.InteractObject(o, func() bool {
			obj, _ := rc.ctx.Data.Objects.FindOne(object.CainGibbet)

			return !obj.Selectable
		})
	}

	action.ReturnTown()

	utils.Sleep(10000)

	err = action.InteractNPC(npc.DeckardCain5)
	if err != nil {
		return err
	}

	step.CloseAllMenus()

	err = action.InteractNPC(npc.Akara)
	if err != nil {
		return err
	}

	step.CloseAllMenus()

	return nil
}

func (rc RescueCain) gatherInfussScroll() error {
	rc.ctx.CharacterCfg.Character.ClearPathDist = 20
	if err := config.SaveSupervisorConfig(rc.ctx.CharacterCfg.ConfigFolderName, rc.ctx.CharacterCfg); err != nil {
		rc.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())
	}

	err := action.WayPoint(area.DarkWood)
	if err != nil {
		return err
	}

	rc.ctx.CharacterCfg.Character.ClearPathDist = 30
	if err := config.SaveSupervisorConfig(rc.ctx.CharacterCfg.ConfigFolderName, rc.ctx.CharacterCfg); err != nil {
		rc.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())
	}

	// Find the Inifuss Tree position.
	var inifussTreePos data.Position
	var foundTree bool
	for _, o := range rc.ctx.Data.Objects {
		if o.Name == object.InifussTree {
			inifussTreePos = o.Position
			foundTree = true
			break
		}
	}
	if !foundTree {
		rc.ctx.Logger.Error("InifussTree not found, aborting quest.")
		return errors.New("InifussTree not found")
	}

	err = action.MoveToCoords(inifussTreePos)
	if err != nil {
		return err
	}

	obj, found := rc.ctx.Data.Objects.FindOne(object.InifussTree)
	if !found {
		rc.ctx.Logger.Error("InifussTree not found, aborting quest.")
		return errors.New("InifussTree not found")
	}

	err = action.InteractObject(obj, func() bool {
		updatedObj, found := rc.ctx.Data.Objects.FindOne(object.InifussTree)
		return found && !updatedObj.Selectable
	})
	if err != nil {
		return fmt.Errorf("error interacting with Inifuss Tree: %w", err)
	}

PickupLoop:
	for i := 0; i < 5; i++ {
		rc.ctx.RefreshGameData()

		foundInInv := false
		for _, itm := range rc.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
			if itm.ID == ScrollInifussUnitID {
				foundInInv = true
				break
			}
		}

		if foundInInv {
			rc.ctx.Logger.Info(fmt.Sprintf("%s found in inventory. Proceeding with quest.", ScrollInifussName))
			break PickupLoop
		}

		// Find the scroll on the ground.
		var scrollObj data.Item
		foundOnGround := false
		for _, itm := range rc.ctx.Data.Inventory.ByLocation(item.LocationGround) {
			if itm.ID == ScrollInifussUnitID {
				scrollObj = itm
				foundOnGround = true
				break
			}
		}

		if foundOnGround {
			rc.ctx.Logger.Info(fmt.Sprintf("%s found on the ground at position %v. Attempting pickup (Attempt %d)...", ScrollInifussName, scrollObj.Position, i+1))

			playerPos := rc.ctx.Data.PlayerUnit.Position
			safeAwayPos := atDistance(scrollObj.Position, playerPos, -5)

			pickupAttempts := 0
			for pickupAttempts < 8 {
				rc.ctx.Logger.Debug("Moving away from scroll for a brief moment...")
				moveAwayErr := action.MoveToCoords(safeAwayPos)
				if moveAwayErr != nil {
					rc.ctx.Logger.Warn(fmt.Sprintf("Failed to move away from scroll: %v", moveAwayErr))
				}
				utils.Sleep(200)

				moveErr := action.MoveToCoords(scrollObj.Position)
				if moveErr != nil {
					rc.ctx.Logger.Error(fmt.Sprintf("Failed to move to scroll position: %v", moveErr))
					utils.Sleep(500)
					pickupAttempts++
					continue
				}

				// --- Refresh game data just before pickup attempt ---
				rc.ctx.RefreshGameData()

				pickupErr := action.ItemPickup(10)
				if pickupErr != nil {
					rc.ctx.Logger.Warn(fmt.Sprintf("Pickup attempt %d failed: %v", pickupAttempts+1, pickupErr))
					utils.Sleep(500)
					pickupAttempts++
					continue
				}

				rc.ctx.RefreshGameData()
				foundInInvAfterPickup := false
				for _, itm := range rc.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
					if itm.ID == ScrollInifussUnitID {
						foundInInvAfterPickup = true
						break
					}
				}
				if foundInInvAfterPickup {
					rc.ctx.Logger.Info(fmt.Sprintf("Pickup confirmed for %s after %d attempts. Proceeding.", ScrollInifussName, pickupAttempts+1))
					break PickupLoop
				}
				pickupAttempts++
			}
		} else {
			rc.ctx.Logger.Debug(fmt.Sprintf("%s not found on the ground on attempt %d. Retrying.", ScrollInifussName, i+1))
			utils.Sleep(1000)
		}
	}

	infusInInventory := false
	for _, itm := range rc.ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if itm.ID == ScrollInifussUnitID {
			infusInInventory = true
			break
		}
	}
	if !infusInInventory {
		rc.ctx.Logger.Error(fmt.Sprintf("Failed to pick up %s after all attempts. Aborting current run.", ScrollInifussName))
		return errors.New("failed to pick up Scroll of Inifuss")
	}

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	return nil
}
func (rc RescueCain) openPortalIfNotOpened() error {

	// If the portal already exists, skip this
	if _, found := rc.ctx.Data.Objects.FindOne(object.PermanentTownPortal); found {
		return nil
	}

	rc.ctx.Logger.Debug("Tristram portal not detected, trying to open it")

	for range 6 {
		//stoneTries := 0
		activeStones := 0
		for _, cainStone := range []object.Name{
			object.CairnStoneAlpha,
			object.CairnStoneGamma,
			object.CairnStoneBeta,
			object.CairnStoneLambda,
			object.CairnStoneDelta,
		} {
			stone, _ := rc.ctx.Data.Objects.FindOne(cainStone)
			if stone.Selectable {
				rc.ctx.PathFinder.RandomMovement()
				utils.Sleep(250)
				action.InteractObject(stone, func() bool {
					st, _ := rc.ctx.Data.Objects.FindOne(cainStone)
					return !st.Selectable
				})

			} else {
				utils.Sleep(200)
				activeStones++
			}
			_, tristPortal := rc.ctx.Data.Objects.FindOne(object.PermanentTownPortal)
			if activeStones >= 5 || tristPortal {
				break
			}
		}

	}

	// Wait upto 15 seconds for the portal to open, checking every second if its up
	for range 15 {
		// Wait a second
		utils.Sleep(1000)

		if _, portalFound := rc.ctx.Data.Objects.FindOne(object.PermanentTownPortal); portalFound {
			return nil
		}
	}

	return errors.New("failed to open Tristram portal")
}
