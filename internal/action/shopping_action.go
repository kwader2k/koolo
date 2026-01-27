package action

import (
	"errors"
	"fmt"
	"log/slog"
	"sort"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/nip"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/drop"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// ActionShoppingPlan holds runtime options for shopping: enable flag, refreshes, gold limits, selected vendors, optional rules and item type filters.
type ActionShoppingPlan struct {
	Enabled         bool
	RefreshesPerRun int
	MinGoldReserve  int
	Vendors         []npc.ID
	Rules           nip.Rules // optional override; if empty, shouldBePickedUp() is used
	Types           []string  // optional allow-list of item types (string of item.Desc().Type)
}

// NewActionShoppingPlanFromConfig builds a runtime plan from the YAML-backed character config.

func NewActionShoppingPlanFromConfig(cfg config.ShoppingConfig) ActionShoppingPlan {
	return ActionShoppingPlan{
		Enabled:         cfg.Enabled,
		RefreshesPerRun: cfg.RefreshesPerRun,
		MinGoldReserve:  cfg.MinGoldReserve,
		Vendors:         vendorListFromConfig(cfg),
		Rules:           nil,
		Types:           nil,
	}
}

// vendorListFromConfig returns the vendors selected in the config.

func vendorListFromConfig(cfg config.ShoppingConfig) []npc.ID {
	// Use the method from the config struct directly
	vendors := cfg.SelectedVendors()
	if len(vendors) > 0 {
		return vendors
	}

	// Fallback (shouldn't happen if config is properly initialized)
	return []npc.ID{}
}

// RunShoppingFromConfig runs the shopping routine using values from config.

func RunShoppingFromConfig(cfg *config.ShoppingConfig) error {
	if cfg == nil {
		return fmt.Errorf("nil shopping config")
	}
	return RunShopping(NewActionShoppingPlanFromConfig(*cfg))
}

// RunShopping iterates towns and vendors, buying items according to the plan and performing optional town refreshes.

func RunShopping(plan ActionShoppingPlan) error {
	ctx := context.Get()
	if !plan.Enabled {
		ctx.Logger.Debug("Shopping disabled")
		return nil
	}
	if len(plan.Vendors) == 0 {
		ctx.Logger.Warn("No vendors selected for shopping")
		return nil
	}
	if ctx.Drop != nil && ctx.Drop.Pending() != nil && ctx.Drop.Active() == nil {
		return drop.ErrInterrupt
	}

	// Ensure enough adjacent space (two columns) before starting
	if !ensureTwoFreeColumnsStrict() {
		ctx.Logger.Warn("Not enough adjacent space (two full columns) even after stashing; aborting shopping")
		return nil
	}

	// Group vendors by town; iterate towns within each pass
	townOrder, vendorsByTown := groupVendorsByTown(plan.Vendors)
	ctx.Logger.Debug("Shopping towns planned", slog.Int("count", len(townOrder)))

	passes := plan.RefreshesPerRun
	if passes < 0 {
		passes = 0
	}

	checkDropInterrupt := func() error {
		if ctx.Drop != nil && ctx.Drop.Pending() != nil && ctx.Drop.Active() == nil {
			return drop.ErrInterrupt
		}
		return nil
	}

	for pass := 0; pass <= passes; pass++ {
		if err := checkDropInterrupt(); err != nil {
			return err
		}
		ctx.Logger.Info("Shopping pass", slog.Int("pass", pass))

		for _, townID := range townOrder {
			if err := checkDropInterrupt(); err != nil {
				return err
			}
			vendors := vendorsByTown[townID]
			if len(vendors) == 0 {
				continue
			}

			if err := ensureInTown(townID); err != nil {
				if errors.Is(err, drop.ErrInterrupt) {
					return err
				}
				ctx.Logger.Warn("Skipping town; cannot reach", slog.String("town", townID.Area().Name), slog.Any("err", err))
				continue
			}
			ctx.RefreshGameData()

			if !ensureTwoFreeColumnsStrict() {
				ctx.Logger.Warn("Insufficient space after stashing; skipping town batch", slog.String("town", townID.Area().Name))
				continue
			}

			for _, v := range vendors {
				if err := checkDropInterrupt(); err != nil {
					return err
				}
				if !ensureTwoFreeColumnsStrict() {
					ctx.Logger.Warn("Skipping vendor due to inventory space (need two free columns)", slog.Int("vendor", int(v)))
					break
				}
				if _, _, err := shopVendorSinglePass(v, plan); err != nil {
					if errors.Is(err, drop.ErrInterrupt) {
						return err
					}
					ctx.Logger.Warn("Vendor pass failed", slog.Int("vendor", int(v)), slog.Any("err", err))
				}
				step.CloseAllMenus()
				ctx.RefreshGameData()
			}
		}

		// Refresh town after visiting all selected vendors in this pass (if more passes remain)
		if pass < passes {
			if err := checkDropInterrupt(); err != nil {
				return err
			}
			lastTown := townOrder[len(townOrder)-1]
			vendorsLast := vendorsByTown[lastTown]
			onlyAnya := len(vendorsLast) == 1 && vendorsLast[0] == npc.Drehya
			if err := refreshTownPreferAnyaPortal(lastTown, onlyAnya); err != nil {
				if errors.Is(err, drop.ErrInterrupt) {
					return err
				}
				ctx.Logger.Warn("Town refresh failed; falling back to waypoint", slog.Any("err", err))
				if err := refreshTownViaWaypoint(lastTown); err != nil {
					if errors.Is(err, drop.ErrInterrupt) {
						return err
					}
				}
			}
		}
	}

	return nil
}

// ensureTwoFreeColumnsStrict ensures two adjacent inventory columns are free; stashes once if needed.

func ensureTwoFreeColumnsStrict() bool {
	if hasTwoFreeColumns() {
		return true
	}
	ctx := context.Get()
	step.CloseAllMenus()
	utils.Sleep(30)
	ctx.RefreshGameData()
	if err := Stash(false); err != nil {
		ctx.Logger.Warn("Stash failed while ensuring two free columns", slog.Any("err", err))
		return false
	}
	utils.Sleep(50)
	ctx.RefreshGameData()
	return hasTwoFreeColumns()
}

// hasFreeRect returns true if a w×h rectangle fits into the inventory grid.

func hasFreeRect(grid [4][10]bool, w, h int) bool {
	H := len(grid)
	if H == 0 {
		return false
	}
	W := len(grid[0])
	if W == 0 {
		return false
	}

	for y := 0; y <= H-h; y++ {
		for x := 0; x <= W-w; x++ {
			free := true
			for dy := 0; dy < h && free; dy++ {
				for dx := 0; dx < w; dx++ {
					if grid[y+dy][x+dx] {
						free = false
						break
					}
				}
			}
			if free {
				return true
			}
		}
	}
	return false
}

// hasTwoFreeColumns returns true if two adjacent inventory columns are free.

func hasTwoFreeColumns() bool {
	ctx := context.Get()
	grid := ctx.Data.Inventory.Matrix()
	h := len(grid)
	if h == 0 {
		return false
	}
	return hasFreeRect(grid, 2, h)
}

// switchVendorTabFast clicks the vendor tab using stash-tab coordinates; caller should refresh UI state after switching.
func switchVendorTabFast(tab int) {
	if tab < 1 || tab > 4 {
		return
	}
	ctx := context.Get()
	var x, y int
	if ctx.GameReader.LegacyGraphics() {
		x = ui.SwitchStashTabBtnXClassic + (tab-1)*ui.SwitchStashTabBtnTabSizeClassic + ui.SwitchStashTabBtnTabSizeClassic/2
		y = ui.SwitchStashTabBtnYClassic
	} else {
		x = ui.SwitchStashTabBtnX + (tab-1)*ui.SwitchStashTabBtnTabSize + ui.SwitchStashTabBtnTabSize/2
		y = ui.SwitchStashTabBtnY
	}
	ctx.HID.Click(game.LeftButton, x, y)
	ctx.RefreshGameData()
}

// scanAndPurchaseItems scans vendor tabs, buys matching items, and stashes/returns if space is insufficient.

func scanAndPurchaseItems(vendorID npc.ID, plan ActionShoppingPlan) (itemsPurchased int, goldSpent int) {
	ctx := context.Get()

	currentGold := ctx.Data.PlayerUnit.TotalPlayerGold()
	if currentGold < plan.MinGoldReserve {
		ctx.Logger.Info("Not enough gold to shop", slog.Int("currentGold", currentGold))
		return 0, 0
	}

	// Cache per-tab vendor spots by SCREEN coords (stable across reopen until town refresh).
	type vendorSpot struct{ SX, SY int }
	perTab := make(map[int][]vendorSpot, 4)

	for tab := 1; tab <= 4; tab++ {
		switchVendorTabFast(tab)
		ctx.RefreshGameData()

		// Cache this - don't call multiple times
		vendorItems := ctx.Data.Inventory.ByLocation(item.LocationVendor)
		for _, it := range vendorItems {
			if (it.Location.Page + 1) != tab {
				continue
			}
			if !typeMatch(it, plan.Types) {
				continue
			}
			if !shouldMatchRulesOnly(it) {
				continue
			}

			coords := ui.GetScreenCoordsForItem(it)
			perTab[tab] = append(perTab[tab], vendorSpot{SX: coords.X, SY: coords.Y})
		}
	}

	if len(perTab) == 0 {
		return 0, 0
	}

	// Deterministic tab order
	tabs := make([]int, 0, len(perTab))
	for t := range perTab {
		tabs = append(tabs, t)
	}
	sort.Ints(tabs)

	// Resolve live item by screen coords on the current tab
	// Define once outside the loop
	findItemAtSpot := func(tab int, spot vendorSpot, vendorItems []data.Item) (data.Item, bool) {
		for _, it := range vendorItems {
			if (it.Location.Page + 1) != tab {
				continue
			}
			coords := ui.GetScreenCoordsForItem(it)
			if coords.X == spot.SX && coords.Y == spot.SY {
				return it, true
			}
		}
		return data.Item{}, false
	}

	for _, tab := range tabs {
		switchVendorTabFast(tab)
		ctx.RefreshGameData()

		spots := perTab[tab]
		for i := 0; i < len(spots); i++ {
			spot := spots[i]

			// Ensure space; stash+return if needed
			if !hasTwoFreeColumns() {
				if !stashAndReturnToVendor(vendorID, tab) {
					ctx.Logger.Warn("Pre-purchase stash+return failed", slog.Int("tab", tab))
					return itemsPurchased, goldSpent
				}
				switchVendorTabFast(tab)
				ctx.RefreshGameData()
			}

			// Cache this - pass to function
			vendorItems := ctx.Data.Inventory.ByLocation(item.LocationVendor)
			it, ok := findItemAtSpot(tab, spot, vendorItems)
			if !ok {
				continue
			}
			if !typeMatch(it, plan.Types) || !shouldMatchRulesOnly(it) {
				continue
			}

			prevGold := ctx.Data.PlayerUnit.TotalPlayerGold()

			// Buy using town helper (consistent with gambling)
			town.BuyItem(it, 1)

			utils.Sleep(40)
			ctx.RefreshGameData()

			itemsPurchased++
			goldSpent += prevGold - ctx.Data.PlayerUnit.TotalPlayerGold()

			// If space tight now, stash and resume same tab
			if !hasTwoFreeColumns() {
				if !stashAndReturnToVendor(vendorID, tab) {
					ctx.Logger.Warn("Post-purchase stash+return failed", slog.Int("tab", tab))
					return itemsPurchased, goldSpent
				}
				switchVendorTabFast(tab)
				ctx.RefreshGameData()
			}
		}
	}

	return itemsPurchased, goldSpent
}

// stashAndReturnToVendor stashes items and reopens the vendor on the specified tab; returns whether it succeeded.

func stashAndReturnToVendor(vendorID npc.ID, tab int) bool {
	ctx := context.Get()

	step.CloseAllMenus()
	if err := Stash(false); err != nil {
		ctx.Logger.Warn("Stash failed", slog.Any("err", err))
		return false
	}
	utils.Sleep(60)
	ctx.RefreshGameData()

	if err := moveToVendor(vendorID); err != nil {
		ctx.Logger.Warn("Return to vendor failed", slog.Int("vendor", int(vendorID)), slog.Any("err", err))
		return false
	}
	if err := InteractNPC(vendorID); err != nil {
		ctx.Logger.Warn("Re-interact vendor failed", slog.Int("vendor", int(vendorID)), slog.Any("err", err))
		return false
	}
	openVendorTrade(vendorID)
	if !ctx.Data.OpenMenus.NPCShop {
		ctx.Logger.Warn("Vendor trade window did not open on return", slog.Int("vendor", int(vendorID)))
		return false
	}

	switchVendorTabFast(tab)
	ctx.RefreshGameData()
	return true
}

// typeMatch applies an optional allow-list by item type string; empty allow-list allows all.

func typeMatch(it data.Item, allow []string) bool {
	if len(allow) == 0 {
		return true
	}
	t := string(it.Desc().Type)
	for _, a := range allow {
		if a == t {
			return true
		}
	}
	return false
}

// openVendorTrade opens the vendor trade dialog using keyboard navigation.

func openVendorTrade(vendorID npc.ID) {
	ctx := context.Get()
	if vendorID == npc.Halbu {
		ctx.HID.KeySequence(0x24 /*HOME*/, 0x0D /*ENTER*/)
	} else {
		ctx.HID.KeySequence(0x24 /*HOME*/, 0x28 /*DOWN*/, 0x0D /*ENTER*/)
	}
	utils.Sleep(20)
	ctx.RefreshGameData()
}

// distanceSquared computes squared tile distance for cheap range checks.
func distanceSquared(a, b data.Position) int {
	dx := int(a.X) - int(b.X)
	dy := int(a.Y) - int(b.Y)
	return dx*dx + dy*dy
}

// moveToVendor navigates to the target vendor. For Anya (Drehya), it resolves via the monsters list with a fixed-position fallback.
func moveToVendor(vendorID npc.ID) error {
	ctx := context.Get()

	// Special-case Anya (Drehya): sometimes listed as Monster rather than NPC.
	if vendorID == npc.Drehya {
		if m, found := ctx.Data.Monsters.FindOne(npc.Drehya, data.MonsterTypeNone); found {
			_ = MoveToCoords(m.Position)
			// Confirm arrival within ~2 tiles
			const arrivalThreshold = 4 // 2*2 tiles squared
			for k := 0; k < 12; k++ {
				ctx.RefreshGameData()
				p := ctx.Data.PlayerUnit.Position
				if distanceSquared(p, m.Position) <= arrivalThreshold {
					return nil
				}
				utils.Sleep(40)
			}
			return nil
		}

		// Fallback: walk to stable anchor near her red portal
		anchor := data.Position{X: 5116, Y: 5121}
		_ = MoveToCoords(anchor)
		const arrivalThreshold = 4
		for k := 0; k < 12; k++ {
			ctx.RefreshGameData()
			p := ctx.Data.PlayerUnit.Position
			if distanceSquared(p, anchor) <= arrivalThreshold {
				return nil
			}
			utils.Sleep(40)
		}
		return nil
	}

	// Default path: use NPC list and move to nearest known position
	n, ok := ctx.Data.NPCs.FindOne(vendorID)
	if !ok || len(n.Positions) == 0 {
		return fmt.Errorf("vendor %d not found", int(vendorID))
	}

	cur := ctx.Data.PlayerUnit.Position
	target := n.Positions[0]
	bestd := distanceSquared(cur, target)
	for _, p := range n.Positions[1:] {
		if d := distanceSquared(cur, p); d < bestd {
			target, bestd = p, d
		}
	}

	return MoveTo(func() (data.Position, bool) {
		return target, true
	})
}

// refreshTownPreferAnyaPortal refreshes Harrogath via Anya's red portal when only Anya is selected; otherwise uses the waypoint method.

func refreshTownPreferAnyaPortal(town area.ID, onlyAnya bool) error {
	ctx := context.Get()
	if town == area.Harrogath && onlyAnya {
		ctx.Logger.Debug("Refreshing town via Anya red portal (preferred)")
		_ = MoveToCoords(data.Position{X: 5116, Y: 5121})
		utils.Sleep(600)
		ctx.RefreshGameData()

		if redPortal, found := ctx.Data.Objects.FindOne(object.PermanentTownPortal); found {
			if err := InteractObject(redPortal, func() bool {
				return ctx.Data.AreaData.Area == area.NihlathaksTemple && ctx.Data.AreaData.IsInside(ctx.Data.PlayerUnit.Position)
			}); err == nil {
				utils.Sleep(120)
				ctx.RefreshGameData()
				if err2 := returnToTownViaAnyaRedPortalFromTemple(); err2 == nil {
					return nil
				}
				ctx.Logger.Debug("Temple->Town red-portal failed; falling back to waypoint")
			}
		}
	}
	return refreshTownViaWaypoint(town)
}

// refreshTownViaWaypoint refreshes a town by stepping out to a nearby area and returning.

func refreshTownViaWaypoint(town area.ID) error {
	// prefer a tagged switch (satisfies QF1003), silence exhaustive with nolint
	//nolint:exhaustive
	switch town {
	case area.RogueEncampment:
		return hopOutAndBack(town, []area.ID{
			area.ColdPlains, area.StonyField, area.DarkWood, area.BlackMarsh, area.OuterCloister,
		})
	case area.LutGholein:
		return hopOutAndBack(town, []area.ID{
			area.DryHills, area.FarOasis, area.LostCity, area.CanyonOfTheMagi, area.ArcaneSanctuary,
		})
	case area.KurastDocks:
		return hopOutAndBack(town, []area.ID{
			area.SpiderForest, area.GreatMarsh, area.FlayerJungle, area.LowerKurast,
		})
	case area.ThePandemoniumFortress:
		return hopOutAndBack(town, []area.ID{
			area.CityOfTheDamned, area.RiverOfFlame,
		})
	case area.Harrogath:
		return hopOutAndBack(town, []area.ID{
			area.FrigidHighlands, area.ArreatPlateau, area.CrystallinePassage,
		})
	default:
		return fmt.Errorf("no viable waypoint refresh for %s", town.Area().Name)
	}
}

// hopOutAndBack leaves town to the first reachable target and returns to refresh vendors.

func hopOutAndBack(town area.ID, candidates []area.ID) error {
	ctx := context.Get()
	for _, a := range candidates {
		if a == town {
			continue
		}
		if err := WayPoint(a); err == nil {
			utils.Sleep(70)
			ctx.RefreshGameData()
			if err := WayPoint(town); err == nil {
				utils.Sleep(70)
				ctx.RefreshGameData()
				return nil
			}
		}
	}
	return fmt.Errorf("no candidate waypoint worked for %s", town.Area().Name)
}

// returnToTownViaAnyaRedPortalFromTemple returns from Nihlathak's Temple to Harrogath using the red portal.

func returnToTownViaAnyaRedPortalFromTemple() error {
	ctx := context.Get()
	if ctx.Data.AreaData.Area != area.NihlathaksTemple {
		return fmt.Errorf("not in Nihlathak's Temple")
	}
	anchor := data.Position{X: 10073, Y: 13311}
	_ = MoveToCoords(anchor)
	utils.Sleep(70)
	ctx.RefreshGameData()

	redPortal, found := ctx.Data.Objects.FindOne(object.PermanentTownPortal)
	if !found {
		probes := []data.Position{
			{X: 2, Y: 0},
			{X: -2, Y: 0},
			{X: 0, Y: 2},
			{X: 0, Y: -2},
		}
		for _, off := range probes {
			_ = MoveToCoords(data.Position{X: anchor.X + off.X, Y: anchor.Y + off.Y})
			utils.Sleep(50)
			ctx.RefreshGameData()
			if rp, ok := ctx.Data.Objects.FindOne(object.PermanentTownPortal); ok {
				redPortal, found = rp, true
				break
			}
		}
	}
	if !found {
		return fmt.Errorf("temple red portal not found")
	}
	utils.Sleep(2000) // cooldown before reusing the red portal
	if err := InteractObject(redPortal, func() bool {
		return ctx.Data.AreaData.Area == area.Harrogath && ctx.Data.AreaData.IsInside(ctx.Data.PlayerUnit.Position)
	}); err != nil {
		return err
	}
	utils.Sleep(80)
	ctx.RefreshGameData()
	return nil
}

// ensureInTown moves the player to the specified town using waypoints or return-to-town.

func ensureInTown(target area.ID) error {
	ctx := context.Get()
	if ctx.Data.PlayerUnit.Area == target {
		return nil
	}
	if err := WayPoint(target); err == nil {
		utils.Sleep(50)
		ctx.RefreshGameData()
		return nil
	}
	return ReturnTown()
}

// groupVendorsByTown groups vendors by town and returns visit order with per-town lists.

func groupVendorsByTown(list []npc.ID) (townOrder []area.ID, byTown map[area.ID][]npc.ID) {
	byTown = make(map[area.ID][]npc.ID, 5)
	townOrder = make([]area.ID, 0, 5)
	seen := make(map[area.ID]bool, 5)

	for _, v := range list {
		townID, ok := VendorLocationMap[v]
		if !ok {
			continue
		}
		if !seen[townID] {
			seen[townID] = true
			townOrder = append(townOrder, townID)
		}
		byTown[townID] = append(byTown[townID], v)
	}
	return
}

// shopVendorSinglePass handles a single vendor visit: approach, open trade, scan/buy, close menus.

func shopVendorSinglePass(vendorID npc.ID, plan ActionShoppingPlan) (int, int, error) {
	ctx := context.Get()
	ctx.SetLastAction("shopVendorSinglePass")

	goldSpent := 0
	itemsPurchased := 0

	// Approach and open trade
	if err := moveToVendor(vendorID); err != nil {
		if errors.Is(err, drop.ErrInterrupt) {
			return 0, 0, err
		}
		ctx.Logger.Warn("MoveTo vendor reported error", slog.Int("vendor", int(vendorID)), slog.Any("err", err))
	}
	utils.Sleep(60)
	ctx.RefreshGameData()

	if err := InteractNPC(vendorID); err != nil {
		return 0, 0, fmt.Errorf("failed to interact with vendor %d: %w", int(vendorID), err)
	}
	openVendorTrade(vendorID)
	if !ctx.Data.OpenMenus.NPCShop {
		return 0, 0, fmt.Errorf("failed to open trade window for vendor %d", int(vendorID))
	}

	// Let vendor items populate
	for i := 0; i < 5; i++ {
		if len(ctx.Data.Inventory.ByLocation(item.LocationVendor)) > 0 {
			break
		}
		utils.Sleep(60)
		ctx.RefreshGameData()
	}

	currentGold := ctx.Data.PlayerUnit.TotalPlayerGold()
	if currentGold < plan.MinGoldReserve {
		ctx.Logger.Info("Not enough gold to shop", slog.Int("currentGold", currentGold))
		return 0, 0, nil
	}

	purchased, spent := scanAndPurchaseItems(vendorID, plan)
	itemsPurchased += purchased
	goldSpent += spent

	// Close inventory/shop to clean state
	step.CloseAllMenus()
	utils.Sleep(40)
	ctx.RefreshGameData()

	return goldSpent, itemsPurchased, nil
}
