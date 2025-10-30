package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type LowerKurast struct {
	ctx *context.Status
}

func NewLowerKurast() *LowerKurast {
	return &LowerKurast{
		ctx: context.Get(),
	}
}

func (a LowerKurast) Name() string {
	return string(config.LowerKurastRun)
}

func (a LowerKurast) CheckConditions(parameters *RunParameters) SequencerResult {
	if !IsFarmingRun(parameters) {
		return SequencerError
	}
	if !a.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (a LowerKurast) Run(parameters *RunParameters) error {

	// Use Waypoint to Lower Kurast
	err := action.WayPoint(area.LowerKurast)
	if err != nil {
		return err
	}

	// Clear Lower Kurast
	return action.ClearCurrentLevel(true, data.MonsterAnyFilter())

}
