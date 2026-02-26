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

type Jail struct {
	ctx *context.Status
}

func NewJail() *Jail {
	return &Jail{
		ctx: context.Get(),
	}
}

func (j Jail) Name() string {
	return string(config.JailRun)
}

func (j Jail) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) {
		return SequencerError
	}
	return SequencerOk
}

func (j Jail) Run(parameters *RunParameters) error {
	if err := action.WayPoint(area.InnerCloister); err != nil {
		return err
	}

	if err := action.MoveToArea(area.JailLevel3); err != nil {
		return err
	}

	if err := action.MoveToArea(area.JailLevel2); err != nil {
		return err
	}

	if err := j.killPitspawn(); err != nil {
		return err
	}

	if err := action.MoveToArea(area.JailLevel1); err != nil {
		return err
	}

	if err := action.MoveToArea(area.Barracks); err != nil {
		return err
	}

	// In order to prevent interact between exit and TP portal if we do any
	j.ctx.PathFinder.RandomMovement()

	return nil
}

func (j Jail) killPitspawn() error {
	monsterPosition := data.Position{}
	if npcData, found := j.ctx.Data.NPCs.FindOneBySuperUniqueID(superunique.PitspawnFouldog); found {
		monsterPosition = npcData.Positions[0]
	}

	if monsterPosition == (data.Position{}) {
		j.ctx.Logger.Warn("Jail run: super unique not found, exploring area")
		if err := action.ClearCurrentLevelEx(true, data.MonsterAnyFilter(), func() bool {
			if monster, found := j.ctx.Data.Monsters.FindOne(npc.Tainted, data.MonsterTypeSuperUnique); found {
				monsterPosition = monster.Position
				j.ctx.Logger.Warn("Jail run: super unique found during exploration")
				return true
			}

			return false
		}); err != nil {
			return err
		}
		if monsterPosition == (data.Position{}) {
			j.ctx.Logger.Warn("Jail run: super unique not found after exploration")
		}
	}

	if monsterPosition != (data.Position{}) {
		if err := action.MoveToCoords(monsterPosition); err != nil {
			return err
		}
	}

	if err := j.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		if m, found := d.Monsters.FindOne(npc.Tainted, data.MonsterTypeSuperUnique); found {
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
