package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type Countess struct {
	ctx *context.Status
	clearMonsterFilter data.MonsterFilter
}

func NewCountess(clearMonsterFilter data.MonsterFilter) *Countess {
	return &Countess{
		ctx: context.Get(),
		clearMonsterFilter: clearMonsterFilter,
	}
}

func (c Countess) Name() string {
	return string(config.CountessRun)
}

func (c Countess) Run() error {
	isTZRun := c.clearMonsterFilter != nil
	// Travel to boss level
	err := action.WayPoint(area.BlackMarsh)
	if err != nil {
		return err
	}

	areas := []area.ID{
		area.ForgottenTower,
		area.TowerCellarLevel1,
		area.TowerCellarLevel2,
		area.TowerCellarLevel3,
		area.TowerCellarLevel4,
		area.TowerCellarLevel5,
	}

	for _, a := range areas {
		err = action.MoveToArea(a)
		if err != nil {
			return err
		}

		if isTZRun {
				c.ctx.Logger.Info(fmt.Sprintf("Clearing Terror Zone: %s", a.Area().Name))
				action.ClearCurrentLevel(c.ctx.CharacterCfg.Game.TerrorZone.OpenChests, c.clearMonsterFilter)
		}		

	}

	err = action.MoveTo(func() (data.Position, bool) {
		areaData := c.ctx.Data.Areas[area.TowerCellarLevel5]
		countessNPC, found := areaData.NPCs.FindOne(740)
		if !found {
			return data.Position{}, false
		}

			return countessNPC.Positions[0], true
		})

		if err != nil {
		c.ctx.Logger.Warn(fmt.Sprintf("Failed to move to Countess (already dead or not found): %v", err))
		if isTZRun {
			return nil
		}
		return err
	}
	c.ctx.Logger.Info("Killing Countess.")
	return c.ctx.Char.KillCountess()
}
