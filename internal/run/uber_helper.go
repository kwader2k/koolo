package run

import (
	"errors"
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

var (
	utPortalPos = data.Position{X: 25100, Y: 5093}
	torchMiddle = data.Position{X: 25129, Y: 5135}
	mephC1      = data.Position{X: 25070, Y: 5085}
	mephC2      = data.Position{X: 25058, Y: 5097}
	mephC3      = data.Position{X: 25072, Y: 5132}
	mephLure    = data.Position{X: 25090, Y: 5136}
	mephSafe    = data.Position{X: 25073, Y: 5125}
	diaC1       = data.Position{X: 25112, Y: 5065}
	diaC2       = data.Position{X: 25167, Y: 5085}
	diaC3       = data.Position{X: 25168, Y: 5113}
	diaLure     = data.Position{X: 25145, Y: 5117}
	diaSafe1    = data.Position{X: 25162, Y: 5120}
	diaSafe2    = data.Position{X: 25166, Y: 5105}
	diaSafe3    = data.Position{X: 25166, Y: 5090}
	malahPos    = data.Position{X: 5071, Y: 5023}
)

func portalPos() data.Position      { return utPortalPos }
func mephPath() []data.Position     { return []data.Position{mephC1, mephC2, mephC3} }
func diaPath() []data.Position      { return []data.Position{diaC1, diaC2, diaC3} }
func diaSafeSeq() []data.Position   { return []data.Position{diaSafe1, diaSafe2, diaSafe3} }
func mephLurePos() data.Position    { return mephLure }
func mephSafePos() data.Position    { return mephSafe }
func diaLurePos() data.Position     { return diaLure }
func diaSafeFirst() data.Position   { return diaSafe1 }
func torchMiddlePos() data.Position { return torchMiddle }
func malahPosition() data.Position  { return malahPos }

func goToAct5(ctx *context.Status) error {
	if ctx.Data.PlayerUnit.Area.Act() != 5 {
		if err := action.WayPoint(area.Harrogath); err != nil {
			return fmt.Errorf("failed to go to Act 5: %w", err)
		}
	}

	if !ctx.Data.PlayerUnit.Area.IsTown() {
		if err := action.ReturnTown(); err != nil {
			return fmt.Errorf("failed to return to town: %w", err)
		}
	}

	return nil
}

func getCube(ctx *context.Status) error {
	_, cubeInInventory := ctx.Data.Inventory.Find("HoradricCube", item.LocationInventory)
	if !cubeInInventory {
		return errors.New("horadric cube not found in inventory. Please place it in a locked inventory slot")
	}
	return nil
}

func openStash(ctx *context.Status) error {
	if ctx.Data.OpenMenus.Stash {
		return nil
	}

	bank, found := ctx.Data.Objects.FindOne(object.Bank)
	if !found {
		return errors.New("stash not found")
	}

	err := action.InteractObject(bank, func() bool {
		return ctx.Data.OpenMenus.Stash
	})
	if err != nil {
		return err
	}

	// The first stash open each game lands on personal; subsequent opens
	// remember the last tab/page.
	if !ctx.CurrentGame.HasOpenedStash {
		ctx.CurrentGame.CurrentStashTab = 1
		ctx.CurrentGame.HasOpenedStash = true
	}
	return nil
}

func IsInNoTPArea(ctx *context.Status) bool {
	return ctx.Data.PlayerUnit.Area == area.UberTristram
}

func findTownPortal(ctx *context.Status) (data.Object, error) {
	ctx.RefreshGameData()

	var portal data.Object
	portalFound := false

	for _, obj := range ctx.Data.Objects {
		if obj.IsRedPortal() {
			if obj.PortalData.DestArea != area.UberTristram {
				portal = obj
				portalFound = true
				break
			}
		}
	}

	if !portalFound {
		for _, obj := range ctx.Data.Objects {
			if obj.IsRedPortal() {
				portal = obj
				portalFound = true
				break
			}
		}
	}

	if !portalFound {
		return data.Object{}, errors.New("failed to find portal back to town")
	}

	return portal, nil
}

func enterTownPortal(ctx *context.Status, portal data.Object) error {
	portalObj, found := ctx.Data.Objects.FindByID(portal.ID)
	if !found {
		return fmt.Errorf("portal not found")
	}
	portalPos := portalObj.Position

	const maxAttempts = 3
	for attempt := 0; attempt < maxAttempts; attempt++ {
		objectX := portalPos.X - 2
		objectY := portalPos.Y - 2
		mX, mY := ui.GameCoordsToScreenCords(objectX, objectY)

		ctx.HID.Click(game.LeftButton, mX, mY)
		utils.Sleep(500)

		maxWaitAttempts := 30
		for waitAttempt := 0; waitAttempt < maxWaitAttempts; waitAttempt++ {
			ctx.RefreshGameData()
			if ctx.Data.PlayerUnit.Area != area.UberTristram && ctx.Data.PlayerUnit.Area.IsTown() {
				return nil
			}
			utils.Sleep(200)
		}

		if attempt < maxAttempts-1 {
			ctx.Logger.Warn(fmt.Sprintf("Portal entry failed (attempt %d/%d), checking for monsters...", attempt+1, maxAttempts))
			ctx.RefreshGameData()

			if hasMonstersNearPortal(ctx, portalPos) {
				if err := clearPortalArea(ctx, portalPos); err != nil {
					ctx.Logger.Warn(fmt.Sprintf("Failed to clear portal area: %v", err))
				}
			}
		}
	}

	return fmt.Errorf("timeout waiting for area change to town after %d attempts", maxAttempts)
}

func returnToUberTristram(ctx *context.Status, findPortalFunc func() (data.Object, error), enterPortalFunc func(data.Object) error) error {
	portal, err := findPortalFunc()
	if err != nil {
		return fmt.Errorf("failed to find Uber Tristram portal: %w", err)
	}

	if err := enterPortalFunc(portal); err != nil {
		return fmt.Errorf("failed to enter Uber Tristram portal: %w", err)
	}

	action.Buff()

	return nil
}

func findUberTristramPortal(ctx *context.Status) (data.Object, error) {
	var portal data.Object
	portalFound := false

	for _, obj := range ctx.Data.Objects {
		if obj.IsRedPortal() {
			if obj.PortalData.DestArea == area.UberTristram {
				portal = obj
				portalFound = true
				break
			}
		}
	}

	if !portalFound {
		for _, obj := range ctx.Data.Objects {
			if obj.IsRedPortal() {
				portal = obj
				portalFound = true
				break
			}
		}
	}

	if !portalFound {
		return data.Object{}, errors.New("failed to find Uber Tristram portal")
	}

	return portal, nil
}

func enterUberTristramPortal(ctx *context.Status, portal data.Object) error {
	step.CloseAllMenus()
	utils.Sleep(200)
	ctx.RefreshGameData()

	portalObj, found := ctx.Data.Objects.FindByID(portal.ID)
	if !found {
		return fmt.Errorf("portal not found")
	}
	objectX := portalObj.Position.X - 2
	objectY := portalObj.Position.Y - 2
	mX, mY := ui.GameCoordsToScreenCords(objectX, objectY)

	ctx.HID.Click(game.LeftButton, mX, mY)
	utils.Sleep(500)

	maxWaitAttempts := 30
	for attempt := 0; attempt < maxWaitAttempts; attempt++ {
		ctx.RefreshGameData()
		if ctx.Data.PlayerUnit.Area == area.UberTristram {
			return nil
		}
		utils.Sleep(200)
	}

	return fmt.Errorf("timeout waiting for area change to Uber Tristram")
}

func isInUberTristram(ctx *context.Status) bool {
	return ctx.Data.PlayerUnit.Area == area.UberTristram
}

func hasOrgans(ctx *context.Status) bool {
	brainCount := 0
	hornCount := 0
	eyeCount := 0

	for _, itm := range action.FilterDLCGhostItems(ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash, item.LocationMaterialsTab)) {
		switch string(itm.Name) {
		case "MephistosBrain":
			brainCount++
		case "DiablosHorn":
			hornCount++
		case "BaalsEye":
			eyeCount++
		}
	}

	return brainCount >= 1 && hornCount >= 1 && eyeCount >= 1
}

func getOrganSet(ctx *context.Status) ([]data.Item, error) {
	var organs []data.Item
	brainFound := false
	hornFound := false
	eyeFound := false

	for _, itm := range action.FilterDLCGhostItems(ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash, item.LocationMaterialsTab)) {
		if !brainFound && string(itm.Name) == "MephistosBrain" {
			organs = append(organs, itm)
			brainFound = true
		} else if !hornFound && string(itm.Name) == "DiablosHorn" {
			organs = append(organs, itm)
			hornFound = true
		} else if !eyeFound && string(itm.Name) == "BaalsEye" {
			organs = append(organs, itm)
			eyeFound = true
		}

		if brainFound && hornFound && eyeFound {
			break
		}
	}

	if len(organs) != 3 {
		return nil, errors.New("failed to find complete organ set (need 1x Mephisto's Brain, 1x Diablo's Horn, 1x Baal's Eye)")
	}

	return organs, nil
}

func checkForRejuv(ctx *context.Status) error {
	if ctx.CharacterCfg.Inventory.BeltColumns.Total(data.RejuvenationPotion) == 0 {
		return nil
	}

	rejuvCount := 0
	var rejuvsInStash []data.Item
	for _, itm := range action.FilterDLCGhostItems(ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash, item.LocationMaterialsTab)) {
		if itm.IsRejuvPotion() {
			// DLC stacked items: count actual quantity, not just entries
			qty := 1
			if itm.Location.LocationType == item.LocationMaterialsTab && itm.StackedQuantity > 0 {
				qty = itm.StackedQuantity
			}
			rejuvCount += qty
			rejuvsInStash = append(rejuvsInStash, itm)
		}
	}

	if rejuvCount == 0 {
		return nil
	}

	missingRejuvCount := ctx.BeltManager.GetMissingCount(data.RejuvenationPotion)
	if missingRejuvCount == 0 {
		return nil
	}

	if err := openStash(ctx); err != nil {
		return fmt.Errorf("failed to open stash: %w", err)
	}

	rejuvsToMove := missingRejuvCount
	if rejuvsToMove > rejuvCount {
		rejuvsToMove = rejuvCount
	}

	moved := 0
	for _, rejuv := range rejuvsInStash {
		if moved >= rejuvsToMove {
			break
		}

		switch rejuv.Location.LocationType {
		case item.LocationStash:
			action.SwitchStashTab(1)
		case item.LocationSharedStash:
			action.SwitchStashTab(rejuv.Location.Page + 1)
		case item.LocationMaterialsTab:
			action.SwitchStashTab(action.StashTabMaterials)
		}
		utils.Sleep(300)
		ctx.RefreshGameData()
		utils.Sleep(150)

		// For stacked DLC items, each Ctrl+click takes one from the stack
		clicksNeeded := 1
		if rejuv.Location.LocationType == item.LocationMaterialsTab && rejuv.StackedQuantity > 0 {
			clicksNeeded = rejuv.StackedQuantity
			if clicksNeeded > rejuvsToMove-moved {
				clicksNeeded = rejuvsToMove - moved
			}
		}

		stashCoords := ui.GetScreenCoordsForItem(rejuv)
		for c := 0; c < clicksNeeded; c++ {
			ctx.HID.ClickWithModifier(game.LeftButton, stashCoords.X, stashCoords.Y, game.CtrlKey)
			utils.Sleep(500)
			ctx.RefreshGameData()
			utils.Sleep(150)
		}
		moved += clicksNeeded
	}

	if err := action.RefillBeltFromInventory(); err != nil {
		return fmt.Errorf("failed to refill belt from inventory: %w", err)
	}

	if err := step.CloseAllMenus(); err != nil {
		return fmt.Errorf("failed to close menus: %w", err)
	}

	return nil
}

func hasKeys(ctx *context.Status) bool {
	terrorCount := 0
	destructionCount := 0
	hateCount := 0

	for _, itm := range action.FilterDLCGhostItems(ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash, item.LocationMaterialsTab)) {
		// DLC stacked items: count actual quantity, not just entries
		qty := 1
		if itm.Location.LocationType == item.LocationMaterialsTab && itm.StackedQuantity > 0 {
			qty = itm.StackedQuantity
		}
		switch string(itm.Name) {
		case "KeyOfTerror":
			terrorCount += qty
		case "KeyOfDestruction":
			destructionCount += qty
		case "KeyOfHate":
			hateCount += qty
		}
	}

	return terrorCount >= 3 && destructionCount >= 3 && hateCount >= 3
}

func getKeySet(ctx *context.Status) ([]data.Item, error) {
	var portalKeys []data.Item
	terrorFound := false
	destructionFound := false
	hateFound := false

	for _, itm := range action.FilterDLCGhostItems(ctx.Data.Inventory.ByLocation(item.LocationStash, item.LocationSharedStash, item.LocationMaterialsTab)) {
		if !terrorFound && string(itm.Name) == "KeyOfTerror" {
			portalKeys = append(portalKeys, itm)
			terrorFound = true
		} else if !destructionFound && string(itm.Name) == "KeyOfDestruction" {
			portalKeys = append(portalKeys, itm)
			destructionFound = true
		} else if !hateFound && string(itm.Name) == "KeyOfHate" {
			portalKeys = append(portalKeys, itm)
			hateFound = true
		}

		if terrorFound && destructionFound && hateFound {
			break
		}
	}

	if len(portalKeys) != 3 {
		return nil, errors.New("failed to find complete key set (need 1x Terror, 1x Destruction, 1x Hate)")
	}

	return portalKeys, nil
}

func walkCoords(ctx *context.Status, coords ...data.Position) error {
	for _, p := range coords {
		if err := action.MoveToCoords(p); err != nil {
			ctx.Logger.Warn(fmt.Sprintf("MoveToCoords failed to %+v: %v", p, err))
			return err
		}
		ctx.RefreshGameData()
	}
	return nil
}

func clearPortalArea(ctx *context.Status, portalPos data.Position) error {
	currentDistance := ctx.PathFinder.DistanceFromMe(portalPos)
	if currentDistance > 5 {
		if err := action.MoveToCoords(portalPos, step.WithDistanceToFinish(3), step.WithIgnoreMonsters()); err != nil {
			ctx.Logger.Warn(fmt.Sprintf("Failed to move closer to portal: %v", err))
		} else {
			utils.Sleep(500)
			ctx.RefreshGameData()
		}
	}

	const clearRadius = 3

	if err := action.ClearAreaAroundPosition(portalPos, clearRadius, data.MonsterAnyFilter()); err != nil {
		return fmt.Errorf("failed to clear portal area: %w", err)
	}

	ctx.RefreshGameData()
	return nil
}

func hasMonstersNearPortal(ctx *context.Status, portalPos data.Position) bool {
	const checkRadius = 3

	for _, m := range ctx.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}
		distance := ctx.PathFinder.DistanceFromMe(m.Position)
		if distance <= checkRadius {
			return true
		}
	}

	return false
}

func isBossNearby(ctx *context.Status, bossNPC npc.ID, maxDistance int) (bool, data.Monster, int) {
	for _, m := range ctx.Data.Monsters.Enemies() {
		if m.Name == bossNPC && m.Stats[stat.Life] > 0 {
			distance := ctx.PathFinder.DistanceFromMe(m.Position)
			if distance <= maxDistance {
				return true, m, distance
			}
		}
	}
	return false, data.Monster{}, 0
}

func isUberMephistoNearby(ctx *context.Status, maxDistance int) (bool, data.Monster, int) {
	return isBossNearby(ctx, npc.UberMephisto, maxDistance)
}

func isUberDiabloNearby(ctx *context.Status, maxDistance int) (bool, data.Monster, int) {
	return isBossNearby(ctx, npc.UberDiablo, maxDistance)
}

func isUberBaalNearby(ctx *context.Status, maxDistance int) (bool, data.Monster, int) {
	return isBossNearby(ctx, npc.UberBaal, maxDistance)
}

func enterTownFromPortal(ctx *context.Status, path []data.Position) error {
	if err := walkCoords(ctx, path...); err != nil {
		return err
	}
	portal, err := findTownPortal(ctx)
	if err != nil {
		return fmt.Errorf("failed to find town portal: %w", err)
	}

	if err := enterTownPortal(ctx, portal); err != nil {
		return fmt.Errorf("failed to enter town portal: %w", err)
	}
	return nil
}

func goToMalahIfInHarrogath(ctx *context.Status) {
	if ctx.Data.PlayerUnit.Area == area.Harrogath {
		if err := action.MoveToCoords(malahPosition(), step.WithIgnoreMonsters()); err != nil {
			ctx.Logger.Warn(fmt.Sprintf("Failed to move to Malah's position: %v", err))
		} else {
			utils.Sleep(300)
			ctx.RefreshGameData()
		}
	}
}

func vendorRefillOrHeal(ctx *context.Status) error {
	needsRefill := false
	if len(town.ItemsToBeSold()) > 0 {
		needsRefill = true
	} else if ctx.Data.PlayerUnit.TotalPlayerGold() >= 1000 {
		if ctx.BeltManager.ShouldBuyPotions() || town.ShouldBuyTPs() || town.ShouldBuyIDs() {
			needsRefill = true
		}
	}

	if needsRefill {
		goToMalahIfInHarrogath(ctx)
		if err := action.VendorRefill(action.VendorRefillOpts{SellJunk: true, BuyConsumables: true}); err != nil {
			ctx.Logger.Warn(fmt.Sprintf("Failed to visit vendor: %v", err))
		}
	} else {
		vendorNPC := town.GetTownByArea(ctx.Data.PlayerUnit.Area).HealNPC()

		goToMalahIfInHarrogath(ctx)

		if err := action.InteractNPC(vendorNPC); err != nil {
			ctx.Logger.Warn(fmt.Sprintf("Failed to interact with NPC: %v", err))
		} else {
			utils.Sleep(300)
			step.CloseAllMenus()
		}
	}

	return nil
}

func goToStashPosition(ctx *context.Status) error {
	bank, found := ctx.Data.Objects.FindOne(object.Bank)
	if !found {
		return errors.New("stash not found")
	}

	if err := action.MoveToCoords(bank.Position, step.WithIgnoreMonsters()); err != nil {
		return fmt.Errorf("failed to move to stash: %w", err)
	}

	return nil
}

func findTorchInInventory(ctx *context.Status, excludeUnitID data.UnitID) (data.Item, bool) {
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if itm.Quality == item.QualityUnique && itm.UnitID != excludeUnitID {
			if itm.Name == "LargeCharm" || itm.IdentifiedName == "Hellfire Torch" {
				return itm, true
			}
		}
	}
	return data.Item{}, false
}

func stashToSharedStash(ctx *context.Status, itm data.Item) (int, error) {
	for tab := 2; tab <= 4; tab++ {
		action.SwitchStashTab(tab)
		utils.Sleep(300)
		ctx.RefreshGameData()

		invCoords := ui.GetScreenCoordsForItem(itm)
		ctx.HID.ClickWithModifier(game.LeftButton, invCoords.X, invCoords.Y, game.CtrlKey)
		utils.Sleep(500)
		ctx.RefreshGameData()

		found := false
		for _, invItem := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
			if invItem.UnitID == itm.UnitID {
				found = true
				break
			}
		}
		if !found {
			return tab, nil
		}
	}
	return 0, fmt.Errorf("failed to stash item - all shared stash tabs may be full")
}

func restoreFromSharedStash(ctx *context.Status, unitID data.UnitID, stashTab int) (data.Item, bool) {
	action.SwitchStashTab(stashTab)
	utils.Sleep(300)
	ctx.RefreshGameData()

	for _, stashItem := range ctx.Data.Inventory.ByLocation(item.LocationSharedStash) {
		if stashItem.UnitID == unitID {
			stashCoords := ui.GetScreenCoordsForItem(stashItem)
			ctx.HID.ClickWithModifier(game.LeftButton, stashCoords.X, stashCoords.Y, game.CtrlKey)
			utils.Sleep(500)
			ctx.RefreshGameData()

			for _, invItem := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
				if invItem.UnitID == unitID {
					return invItem, true
				}
			}
		}
	}
	return data.Item{}, false
}

func moveItemToPosition(ctx *context.Status, itm data.Item, targetPos data.Position) error {
	if itm.Position.X == targetPos.X && itm.Position.Y == targetPos.Y {
		return nil
	}

	invCoords := ui.GetScreenCoordsForItem(itm)
	ctx.HID.Click(game.LeftButton, invCoords.X, invCoords.Y)
	utils.Sleep(200)
	targetCoords := ui.GetScreenCoordsForInventoryPosition(targetPos, item.LocationInventory)
	ctx.HID.Click(game.LeftButton, targetCoords.X, targetCoords.Y)
	utils.Sleep(300)
	ctx.RefreshGameData()
	return nil
}

func standardofHeros(ctx *context.Status) error {
	for _, invItem := range ctx.Data.Inventory.ByLocation(item.LocationInventory) {
		if invItem.Name == "StandardOfHeroes" || invItem.IdentifiedName == "Standard of Heroes" {
			if _, err := stashToSharedStash(ctx, invItem); err != nil {
				return fmt.Errorf("failed to stash Standard of Heroes: %w", err)
			}
			ctx.Logger.Info("Stashed Standard of Heroes before torch swap")
			utils.Sleep(300)
			ctx.RefreshGameData()
			return nil
		}
	}
	return nil
}

func openUT(ctx *context.Status, organs []data.Item) (data.Object, error) {
	torchPortalPos := data.Position{X: 5135, Y: 5061}
	if err := action.CubeAddItems(organs[0], organs[1], organs[2]); err != nil {
		return data.Object{}, fmt.Errorf("failed to add organs to cube: %w", err)
	}

	if err := step.CloseAllMenus(); err != nil {
		return data.Object{}, fmt.Errorf("failed to close menus: %w", err)
	}

	if err := action.MoveToCoords(torchPortalPos); err != nil {
		return data.Object{}, fmt.Errorf("failed to move to portal position: %w", err)
	}

	if !ctx.Data.OpenMenus.Inventory {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.Sleep(300)
		ctx.RefreshGameData()
		if !ctx.Data.OpenMenus.Inventory {
			return data.Object{}, errors.New("failed to open inventory window")
		}
	}

	if err := action.CubeTransmute(); err != nil {
		return data.Object{}, fmt.Errorf("failed to transmute organs: %w", err)
	}

	if err := step.CloseAllMenus(); err != nil {
		return data.Object{}, fmt.Errorf("failed to close menus after transmute: %w", err)
	}

	utils.Sleep(500)
	ctx.RefreshGameData()
	var portal data.Object
	portalFound := false
	for _, obj := range ctx.Data.Objects {
		if obj.IsRedPortal() && obj.PortalData.DestArea == area.UberTristram {
			portal = obj
			portalFound = true
			break
		}
	}

	if !portalFound {
		return data.Object{}, errors.New("failed to find newly created Uber Tristram portal")
	}

	return portal, nil
}
