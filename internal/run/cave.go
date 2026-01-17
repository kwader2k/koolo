package run

import (
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

type Cave struct {
	ctx *context.Status
}

func NewCave() *Cave {
	return &Cave{
		ctx: context.Get(),
	}
}

func (c Cave) Name() string {
	return string(config.CaveRun)
}

func (c Cave) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) {
		return SequencerError
	}
	return SequencerOk
}

func (c Cave) Run(parameters *RunParameters) error {
	if err := action.WayPoint(area.ColdPlains); err != nil {
		return err
	}

	if err := action.MoveToArea(area.CaveLevel1); err != nil {
		return err
	}

	coldcrowPos := data.Position{}
	if areaData, ok := c.ctx.Data.Areas[area.CaveLevel1]; ok {
		npcData, found := areaData.NPCs.FindOne(npc.DarkRanger)
		if !found {
			for _, entry := range areaData.NPCs {
				if strings.EqualFold(entry.Name, "Coldcrow") {
					npcData = entry
					found = true
					break
				}
			}
		}

		if found && len(npcData.Positions) > 0 {
			coldcrowPos = npcData.Positions[0]
		} else {
			c.ctx.Logger.Warn("Cave run: Coldcrow position not found in map data, exploring")
		}
	} else {
		c.ctx.Logger.Warn("Cave run: map data missing for Cave Level 1, exploring")
	}

	if coldcrowPos == (data.Position{}) {
		if err := action.ClearCurrentLevelEx(false, data.MonsterAnyFilter(), func() bool {
			if m, found := c.ctx.Data.Monsters.FindOne(npc.DarkRanger, data.MonsterTypeSuperUnique); found {
				coldcrowPos = m.Position
				c.ctx.Logger.Info("Cave run: Coldcrow found during exploration")
				return true
			}
			return false
		}); err != nil {
			return err
		}

		if coldcrowPos == (data.Position{}) {
			c.ctx.Logger.Warn("Cave run: Coldcrow not found after exploration")
		}
	}

	if coldcrowPos != (data.Position{}) {
		if err := action.MoveToCoords(coldcrowPos); err != nil {
			c.ctx.Logger.Warn("Cave run: failed moving to Coldcrow", "error", err)
		}
	}

	if err := c.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		if m, found := d.Monsters.FindOne(npc.DarkRanger, data.MonsterTypeSuperUnique); found {
			return m.UnitID, true
		}
		return 0, false
	}, nil); err != nil {
		return err
	}

	action.ItemPickup(30)

	if err := action.MoveToArea(area.CaveLevel2); err != nil {
		return err
	}

	return action.ClearCurrentLevel(true, data.MonsterAnyFilter())
}
