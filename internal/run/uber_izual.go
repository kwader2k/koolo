package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type UberIzual struct {
	ctx *context.Status
}

func NewUberIzual() *UberIzual {
	return &UberIzual{
		ctx: context.Get(),
	}
}

func (u UberIzual) Name() string {
	return string(config.UberIzualRun)
}

func (u UberIzual) CheckConditions(parameters *RunParameters) SequencerResult {
	if u.ctx.Data.PlayerUnit.Area != area.FurnaceOfPain {
		return SequencerSkip
	}
	return SequencerOk
}

func (u UberIzual) Run(parameters *RunParameters) error {
	action.Buff()

	areaData := u.ctx.Data.Areas[area.FurnaceOfPain]
	uberIzualNPC, found := areaData.NPCs.FindOne(npc.UberIzual)
	if found && len(uberIzualNPC.Positions) > 0 {
		if err := action.MoveToCoords(uberIzualNPC.Positions[0], step.WithIgnoreMonsters()); err != nil {
			u.ctx.Logger.Warn(fmt.Sprintf("Failed to teleport to area data position: %v", err))
		}
	}

	exploreFilter := func(m data.Monsters) []data.Monster {
		return []data.Monster{}
	}

	bossFound := false
	err := action.ClearCurrentLevelEx(false, exploreFilter, func() bool {
		if boss, found := u.ctx.Data.Monsters.FindOne(npc.UberIzual, data.MonsterTypeUnique); found {
			if err := action.MoveToCoords(boss.Position, step.WithIgnoreMonsters()); err != nil {
				u.ctx.Logger.Warn(fmt.Sprintf("Failed to teleport to boss: %v", err))
			}
			bossFound = true
			return true
		}
		return false
	})
	if err != nil {
		return err
	}

	if !bossFound {
		u.ctx.Logger.Warn("Uber Izual not found during exploration")
		return fmt.Errorf("UberIzual not found during exploration")
	}

	u.ctx.Logger.Info("Found Uber Izual, starting fight")
	if err := u.ctx.Char.KillUberIzual(); err != nil {
		return err
	}

	action.ItemPickup(30)

	u.ctx.Logger.Info("Successfully killed Uber Izual")
	return nil
}
