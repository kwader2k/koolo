package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/superunique"
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

	if err := c.killColdcrow(); err != nil {
		return err
	}

	if err := action.MoveToArea(area.CaveLevel2); err != nil {
		return err
	}

	return action.ClearCurrentLevel(true, data.MonsterAnyFilter())
}

func (c Cave) killColdcrow() error {
	monsterPosition := data.Position{}
	if npcData, found := c.ctx.Data.NPCs.FindOneBySuperUniqueID(superunique.Coldcrow); found {
		monsterPosition = npcData.Positions[0]
	}

	if monsterPosition == (data.Position{}) {
		c.ctx.Logger.Warn("Cave run: super unique not found, exploring area")
		if err := action.ClearCurrentLevelEx(true, data.MonsterAnyFilter(), func() bool {
			if monster, found := c.ctx.Data.Monsters.FindOne(npc.DarkRanger, data.MonsterTypeSuperUnique); found {
				monsterPosition = monster.Position
				c.ctx.Logger.Warn("Cave run: super unique found during exploration")
				return true
			}

			return false
		}); err != nil {
			return err
		}
		if monsterPosition == (data.Position{}) {
			c.ctx.Logger.Warn("Cave run: super unique not found after exploration")
		}
	}

	if monsterPosition != (data.Position{}) {
		if err := action.MoveToCoords(monsterPosition); err != nil {
			return err
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

	if err := action.ItemPickup(30); err != nil {
		return err
	}

	return nil
}
