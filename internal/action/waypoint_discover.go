package action

import (
	"log/slog"

	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
)

func DiscoverWaypoint() error {
	ctx := context.Get()
	isTown := ctx.Data.PlayerUnit.Area.Area().IsTown()
	if isTown {
		return nil
	}
	areaName := ctx.Data.PlayerUnit.Area.Area().Name
	ctx.SetLastAction("DiscoverWaypoint " + areaName)

	ctx.Logger.Info("Trying to autodiscover Waypoint for current area", slog.String("area", areaName))
	for _, o := range ctx.Data.Objects {
		if o.IsWaypoint() {
			err := InteractObject(o, func() bool {
				return ctx.Data.OpenMenus.Waypoint
			})
			if err != nil {
				return err
			}

			ctx.Logger.Info("Waypoint discovered", slog.String("area", ctx.Data.PlayerUnit.Area.Area().Name))
			step.CloseAllMenus()
		}
	}
	return nil
}
