package run

import (
	"errors"

	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Summoner struct {
	ctx *context.Status
}

func NewSummoner() *Summoner {
	return &Summoner{
		ctx: context.Get(),
	}
}

func (s Summoner) Name() string {
	return string(config.SummonerRun)
}

func (s Summoner) Run() error {
	// Use the waypoint to get to Arcane Sanctuary
	if s.ctx.CharacterCfg.Game.Summoner.KillFireEye {
		NewFireEye().Run()

		obj, _ := s.ctx.Data.Objects.FindOne(object.ArcaneSanctuaryPortal)

		err := action.InteractObject(obj, func() bool {
			updatedObj, found := s.ctx.Data.Objects.FindOne(object.ArcaneSanctuaryPortal)
			if found {
				if !updatedObj.Selectable {
					s.ctx.Logger.Debug("Interacted with ArcaneSanctuaryPortal")
				}
				return !updatedObj.Selectable
			}
			return false
		})

		if err != nil {
			return err
		}

		err = action.InteractObject(obj, func() bool {
			return s.ctx.Data.PlayerUnit.Area == area.ArcaneSanctuary
		})

		if err != nil {
			return err
		}

		utils.Sleep(300)

	}

	if !s.ctx.CharacterCfg.Game.Summoner.KillFireEye {
		err := action.WayPoint(area.ArcaneSanctuary)
		if err != nil {
			return err
		}
	}

	action.Buff()

	// Get the Summoner's position from the cached map data
	areaData := s.ctx.Data.Areas[area.ArcaneSanctuary]
	summonerNPC, found := areaData.NPCs.FindOne(npc.Summoner)
	if !found || len(summonerNPC.Positions) == 0 {
		return errors.New("failed to find the Summoner")
	}

	// Move to the Summoner's position using the static coordinates from map data
	if err := action.MoveToCoords(summonerNPC.Positions[0]); err != nil {
		return err
	}

	// Kill Summoner
	return s.ctx.Char.KillSummoner()
}
