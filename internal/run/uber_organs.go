package run

import (
	"errors"
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Organs struct {
	ctx *context.Status
}

func NewOrgans() *Organs {
	return &Organs{
		ctx: context.Get(),
	}
}

func (o Organs) Name() string {
	return string(config.OrgansRun)
}

func (o Organs) CheckConditions(parameters *RunParameters) SequencerResult {
	if !hasKeys(o.ctx) {
		o.ctx.Logger.Warn("Not enough keys in stash. Need 3x Key of Terror, 3x Key of Destruction, 3x Key of Hate")
		return SequencerSkip
	}
	return SequencerOk
}

func (o Organs) Run(parameters *RunParameters) error {
	if err := goToAct5(o.ctx); err != nil {
		return err
	}

	utils.Sleep(1500)
	o.ctx.RefreshGameData()

	if !hasKeys(o.ctx) {
		return errors.New("not enough keys in stash. Need 3x Key of Terror, 3x Key of Destruction, 3x Key of Hate")
	}

	if err := getCube(o.ctx); err != nil {
		return err
	}

	if err := checkForRejuv(o.ctx); err != nil {
		o.ctx.Logger.Warn(fmt.Sprintf("Failed to check/fill rejuvs: %v", err))
	}

	completedAreas := make(map[string]bool)

	for portalNum := 1; portalNum <= 3; portalNum++ {
		if err := openStash(o.ctx); err != nil {
			o.ctx.Logger.Warn(fmt.Sprintf("Failed to open stash for portal %d: %v", portalNum, err))
			continue
		}

		portalKeys, err := getKeySet(o.ctx)
		if err != nil {
			o.ctx.Logger.Warn(fmt.Sprintf("Failed to get keys for portal %d: %v", portalNum, err))
			continue
		}

		portal, areaName, err := o.openPortal(portalKeys, portalPositions[portalNum-1], portalNum)
		if err != nil {
			o.ctx.Logger.Warn(fmt.Sprintf("Failed to open portal %d: %v", portalNum, err))
			continue
		}

		if completedAreas[areaName] {
			continue
		}

		if err := o.enterPortal(portal, areaName); err != nil {
			o.ctx.Logger.Warn(fmt.Sprintf("Failed to enter portal to %s: %v", areaName, err))
			continue
		}

		o.ctx.Logger.Info(fmt.Sprintf("Starting %s dungeon", areaName))
		dungeonErr := o.runDungeon(areaName, parameters)
		if dungeonErr != nil {
			o.ctx.Logger.Warn(fmt.Sprintf("Failed to complete %s: %v", areaName, dungeonErr))
		} else {
			o.ctx.Logger.Info(fmt.Sprintf("Successfully completed %s", areaName))
		}

		if err := action.ReturnTown(); err != nil {
			o.ctx.Logger.Warn(fmt.Sprintf("Failed to return to town: %v", err))
			continue
		}

		utils.Sleep(200)
		o.ctx.RefreshGameData()

		goToMalahIfInHarrogath(o.ctx)
		if err := action.VendorRefill(action.VendorRefillOpts{ForceRefill: true, SellJunk: true, BuyConsumables: true}); err != nil {
			o.ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor after portal %d: %v", portalNum, err))
		}

		action.ReviveMerc()

		if err := action.Stash(true); err != nil {
			o.ctx.Logger.Warn(fmt.Sprintf("Failed to stash items after portal %d: %v", portalNum, err))
		}

		if dungeonErr == nil {
			completedAreas[areaName] = true
		}
	}

	return nil
}

var portalPositions = []data.Position{
	{X: 5131, Y: 5055},
	{X: 5140, Y: 5057},
	{X: 5131, Y: 5068},
}

func (o Organs) openPortal(keys []data.Item, position data.Position, portalNum int) (data.Object, string, error) {
	if err := action.CubeAddItems(keys[0], keys[1], keys[2]); err != nil {
		return data.Object{}, "", fmt.Errorf("failed to add keys to cube: %w", err)
	}

	if err := step.CloseAllMenus(); err != nil {
		return data.Object{}, "", fmt.Errorf("failed to close menus: %w", err)
	}

	if err := action.MoveToCoords(position); err != nil {
		return data.Object{}, "", fmt.Errorf("failed to move to portal position: %w", err)
	}

	if err := o.openInventory(); err != nil {
		return data.Object{}, "", fmt.Errorf("failed to open inventory: %w", err)
	}

	if err := action.CubeTransmute(); err != nil {
		return data.Object{}, "", fmt.Errorf("failed to transmute keys: %w", err)
	}

	if err := step.CloseAllMenus(); err != nil {
		return data.Object{}, "", fmt.Errorf("failed to close menus after transmute: %w", err)
	}

	utils.Sleep(500)
	o.ctx.RefreshGameData()

	var portal data.Object
	portalFound := false
	for _, obj := range o.ctx.Data.Objects {
		if obj.IsRedPortal() {
			portal = obj
			portalFound = true
			break
		}
	}

	if !portalFound {
		return data.Object{}, "", errors.New("failed to find newly created portal")
	}

	areaName := o.getPortalArea(portal)
	if areaName == "" {
		areaName = o.identifyPortal(portal)
		if areaName == "" {
			return data.Object{}, "", errors.New("failed to identify portal area")
		}
		if err := action.ReturnTown(); err != nil {
			o.ctx.Logger.Warn(fmt.Sprintf("Failed to return to town after identifying portal: %v", err))
		}
		o.ctx.RefreshGameData()
		portalFound = false
		for _, obj := range o.ctx.Data.Objects {
			if obj.IsRedPortal() && obj.Position.X == portal.Position.X && obj.Position.Y == portal.Position.Y {
				portal = obj
				portalFound = true
				break
			}
		}
		if !portalFound {
			return data.Object{}, "", errors.New("failed to find portal after identification")
		}
	}

	return portal, areaName, nil
}

func (o Organs) getPortalArea(portal data.Object) string {
	destArea := portal.PortalData.DestArea
	switch destArea {
	case area.FurnaceOfPain:
		return "FurnaceOfPain"
	case area.ForgottenSands:
		return "ForgottenSands"
	case area.MatronsDen:
		return "MatronsDen"
	default:
		return ""
	}
}

func (o Organs) identifyPortal(portal data.Object) string {
	err := action.InteractObject(portal, func() bool {
		return !o.ctx.Data.PlayerUnit.Area.IsTown()
	})
	if err != nil {
		o.ctx.Logger.Warn(fmt.Sprintf("Failed to enter portal for identification: %v", err))
		return ""
	}

	utils.Sleep(500)
	o.ctx.RefreshGameData()

	currentArea := o.ctx.Data.PlayerUnit.Area
	switch currentArea {
	case area.FurnaceOfPain:
		return "FurnaceOfPain"
	case area.ForgottenSands:
		return "ForgottenSands"
	case area.MatronsDen:
		return "MatronsDen"
	default:
		o.ctx.Logger.Warn(fmt.Sprintf("Unknown area: %d (%s)", currentArea, currentArea.Area().Name))
		return ""
	}
}

func (o Organs) runDungeon(areaName string, parameters *RunParameters) error {
	var runErr error
	switch areaName {
	case "FurnaceOfPain":
		runErr = NewUberIzual().Run(parameters)
	case "ForgottenSands":
		runErr = NewUberDuriel().Run(parameters)
	case "MatronsDen":
		runErr = NewLilith().Run(parameters)
	default:
		return fmt.Errorf("unknown area: %s", areaName)
	}

	if runErr != nil {
		return fmt.Errorf("failed to run %s: %w", areaName, runErr)
	}

	return nil
}

func (o Organs) enterPortal(portal data.Object, areaName string) error {
	portalObj, found := o.ctx.Data.Objects.FindByID(portal.ID)
	if !found {
		return fmt.Errorf("portal not found")
	}
	objectX := portalObj.Position.X - 2
	objectY := portalObj.Position.Y - 2
	mX, mY := ui.GameCoordsToScreenCords(objectX, objectY)

	o.ctx.HID.Click(game.LeftButton, mX, mY)
	utils.Sleep(500)

	maxWaitAttempts := 30
	for attempt := 0; attempt < maxWaitAttempts; attempt++ {
		o.ctx.RefreshGameData()
		if o.isInArea(areaName) {
			return nil
		}
		utils.Sleep(200)
	}

	return fmt.Errorf("timeout waiting for area change to %s", areaName)
}

func (o Organs) openInventory() error {
	if !o.ctx.Data.OpenMenus.Inventory {
		o.ctx.HID.PressKeyBinding(o.ctx.Data.KeyBindings.Inventory)
		utils.Sleep(300)
		o.ctx.RefreshGameData()
		if !o.ctx.Data.OpenMenus.Inventory {
			return errors.New("failed to open inventory window")
		}
	}
	return nil
}

func (o Organs) isInArea(areaName string) bool {
	currentAreaID := o.ctx.Data.PlayerUnit.Area
	switch areaName {
	case "FurnaceOfPain":
		return currentAreaID == area.FurnaceOfPain
	case "ForgottenSands":
		return currentAreaID == area.ForgottenSands
	case "MatronsDen":
		return currentAreaID == area.MatronsDen
	}
	return false
}
