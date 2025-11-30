package action

import (
	"errors"
	"fmt"
	"log/slog"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

func WayPoint(dest area.ID) error {
	ctx := context.Get()
	ctx.SetLastAction("WayPoint")
	ctx.WaitForGameToLoad()

	if ctx.Data.PlayerUnit.Area == dest {
		return nil
	}

	wpArea, wpCoords, hasWP, err := nearestWaypointArea(ctx, dest)
	if err != nil {
		return err
	}

	if !hasWP {
		ctx.Logger.Info("No usable waypoint found, walking to destination", slog.String("destination", dest.Area().Name))
		if err := traverseRemainder(ctx.Data.PlayerUnit.Area, dest); err != nil {
			return err
		}
		return verifyArrival(ctx, dest)
	}

	if err := ensureWaypointAccess(ctx); err != nil {
		return err
	}

	ctx.RefreshGameData()

	if wpArea != ctx.Data.PlayerUnit.Area {
		if err := selectWaypoint(ctx, wpCoords, wpArea); err != nil {
			if ctx.Data.PlayerUnit.Area.Act() != dest.Act() {
				// we can't fallback to walking if the acts are different
				return fmt.Errorf("failed to select waypoint to %s: %w", area.Areas[dest].Name, err)
			}
			ctx.Logger.Warn("Waypoint selection failed, walking remainder", slog.String("destination", area.Areas[dest].Name), slog.String("reason", err.Error()))
			step.CloseAllMenus()
			return traverseRemainder(ctx.Data.PlayerUnit.Area, dest)
		}
		ctx.WaitForGameToLoad()
	}

	if err := traverseRemainder(wpArea, dest); err != nil {
		return err
	}

	if err := verifyArrival(ctx, dest); err != nil {
		return err
	}

	// apply buffs after exiting a waypoint if configured
	if ctx.CharacterCfg.Character.BuffAfterWP {
		Buff()
	}

	return nil
}

func traverseRemainder(wpArea, dest area.ID) error {
	ctx := context.Get()
	ctx.SetLastAction("useWP")

	if wpArea == dest {
		return nil
	}

	path, ok := staticAreaPath(wpArea, dest)
	if !ok {
		return fmt.Errorf("no static route found from %s to %s", area.Areas[wpArea].Name, area.Areas[dest].Name)
	}

	pathNames := make([]string, len(path))
	for i, a := range path {
		pathNames[i] = area.Areas[a].Name
	}

	ctx.Logger.Info("Traversing areas to reach destination", slog.String("areas", strings.Join(pathNames, "-> ")))

	for i := 1; i < len(path); i++ {
		dst := path[i]
		if err := moveToAreaSingle(ctx, dst, false); err != nil {
			return err
		}

		if err := DiscoverWaypoint(); err != nil {
			return err
		}
	}

	return nil
}

func verifyArrival(ctx *context.Status, dest area.ID) error {
	ctx.WaitForGameToLoad()
	ctx.RefreshGameData()
	if err := ensureAreaSync(ctx, dest); err != nil {
		return fmt.Errorf("failed to reach destination area %s using waypoint: %w", area.Areas[dest].Name, err)
	}
	if ctx.Data.PlayerUnit.Area != dest {
		return fmt.Errorf("failed to reach destination area %s using waypoint", area.Areas[dest].Name)
	}
	return nil
}

func ensureWaypointAccess(ctx *context.Status) error {

	if !ctx.Data.PlayerUnit.Area.IsTown() {

		if ctx.Data.CanTeleport() && hasWaypointInCurrentArea(ctx) {
			return nil
		}

		if err := ReturnTown(); err != nil {
			return err
		}
		ctx.RefreshGameData()
		utils.Sleep(300)
	}
	if !hasWaypointInCurrentArea(ctx) {
		return fmt.Errorf("should be in town, but no waypoint found in %s", ctx.Data.PlayerUnit.Area.Area().Name)
	}
	return nil
}

func hasWaypointInCurrentArea(ctx *context.Status) bool {
	for _, o := range ctx.Data.Objects {
		if o.IsWaypoint() {
			return true
		}
	}
	return false
}

func openWaypointMenu(ctx *context.Status) error {
	for _, o := range ctx.Data.Objects {
		if o.IsWaypoint() {
			return InteractObject(o, func() bool {
				return ctx.Data.OpenMenus.Waypoint
			})
		}
	}
	utils.Sleep(500)
	return fmt.Errorf("no waypoint object found in %s", ctx.Data.PlayerUnit.Area.Area().Name)
}

func selectWaypoint(ctx *context.Status, wpCoords area.WPAddress, dest area.ID) error {
	ctx.SetLastAction("selectWP")
	for range 3 {
		if ctx.Data.OpenMenus.LoadingScreen {
			ctx.Logger.Debug("Loading screen detected. Waiting for game to load before selecting waypoint...")
			ctx.WaitForGameToLoad()
		}
		ctx.RefreshGameData()
		ClearMessages()
		utils.Sleep(200)
		if ctx.Data.PlayerUnit.Area == dest {
			return nil
		}
		if err := ensureAreaSync(ctx, ctx.Data.PlayerUnit.Area); err != nil {
			return err
		}
		if !ctx.Data.OpenMenus.Waypoint {
			if err := openWaypointMenu(ctx); err != nil {
				return err
			}
		}
		utils.Sleep(100)
		if !ctx.Data.OpenMenus.Waypoint {
			return errors.New("failed to open waypoint menu")
		}
		if ctx.Data.LegacyGraphics {
			actTabX := ui.WpTabStartXClassic + (wpCoords.Tab-1)*ui.WpTabSizeXClassic + (ui.WpTabSizeXClassic / 2)
			ctx.HID.Click(game.LeftButton, actTabX, ui.WpTabStartYClassic)
		} else {
			actTabX := ui.WpTabStartX + (wpCoords.Tab-1)*ui.WpTabSizeX + (ui.WpTabSizeX / 2)
			ctx.HID.Click(game.LeftButton, actTabX, ui.WpTabStartY)
		}

		utils.Sleep(200)

		if ctx.Data.LegacyGraphics {
			areaBtnY := ui.WpListStartYClassic + (wpCoords.Row-1)*ui.WpAreaBtnHeightClassic + (ui.WpAreaBtnHeightClassic / 2)
			ctx.HID.Click(game.LeftButton, ui.WpListPositionXClassic, areaBtnY)
		} else {
			areaBtnY := ui.WpListStartY + (wpCoords.Row-1)*ui.WpAreaBtnHeight + (ui.WpAreaBtnHeight / 2)
			ctx.HID.Click(game.LeftButton, ui.WpListPositionX, areaBtnY)
		}
		ctx.WaitForGameToLoad()
		utils.Sleep(200)
		ctx.RefreshGameData()
	}
	if ctx.Data.PlayerUnit.Area == dest {
		return nil
	}
	return errors.New("failed to select waypoint destination")
}

func nearestWaypointArea(ctx *context.Status, dest area.ID) (area.ID, area.WPAddress, bool, error) {
	act := dest.Act()

	wpMap := availableWaypointMap(ctx)
	if wpCoords, ok := area.WPAddresses[dest]; ok {
		if _, known := wpMap[dest]; known {
			return dest, wpCoords, true, nil
		}
	}

	visited := map[area.ID]struct{}{dest: {}}
	queue := []area.ID{dest}

	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]

		for _, next := range staticNeighbors(current) {
			if next == 0 || next.Act() != act {
				continue
			}
			if _, ok := visited[next]; ok {
				continue
			}
			visited[next] = struct{}{}

			if wpCoords, ok := area.WPAddresses[next]; ok {
				if _, known := wpMap[next]; known {
					return next, wpCoords, true, nil
				}
			}

			queue = append(queue, next)
		}
	}

	if act == ctx.Data.PlayerUnit.Area.Act() {
		// no waypoint found but we can walk, let the caller handle it
		return 0, area.WPAddress{}, false, nil
	}

	return 0, area.WPAddress{}, false, fmt.Errorf("failed to locate a waypoint reachable from %s", area.Areas[dest].Name)
}
