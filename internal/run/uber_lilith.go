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

type Lilith struct {
	ctx *context.Status
}

func NewLilith() *Lilith {
	return &Lilith{
		ctx: context.Get(),
	}
}

func (l Lilith) Name() string {
	return string(config.LilithRun)
}

func (l Lilith) CheckConditions(parameters *RunParameters) SequencerResult {
	if l.ctx.Data.PlayerUnit.Area != area.MatronsDen {
		return SequencerSkip
	}
	return SequencerOk
}

func (l Lilith) Run(parameters *RunParameters) error {
	action.Buff()

	areaData := l.ctx.Data.Areas[area.MatronsDen]
	lilithNPC, found := areaData.NPCs.FindOne(npc.Lilith)
	if found && len(lilithNPC.Positions) > 0 {
		if err := action.MoveToCoords(lilithNPC.Positions[0], step.WithIgnoreMonsters()); err != nil {
			l.ctx.Logger.Warn(fmt.Sprintf("Failed to teleport to area data position: %v", err))
		}
	}

	exploreFilter := func(m data.Monsters) []data.Monster {
		return []data.Monster{}
	}

	bossFound := false
	err := action.ClearCurrentLevelEx(false, exploreFilter, func() bool {
		if boss, found := l.ctx.Data.Monsters.FindOne(npc.Lilith, data.MonsterTypeUnique); found {
			if err := action.MoveToCoords(boss.Position, step.WithIgnoreMonsters()); err != nil {
				l.ctx.Logger.Warn(fmt.Sprintf("Failed to teleport to boss: %v", err))
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
		l.ctx.Logger.Warn("Lilith not found during exploration")
		return fmt.Errorf("Lilith not found during exploration")
	}

	l.ctx.Logger.Info("Found Lilith, starting fight")
	if err := l.ctx.Char.KillLilith(); err != nil {
		return err
	}

	action.ItemPickup(30)

	l.ctx.Logger.Info("Successfully killed Lilith")
	return nil
}
