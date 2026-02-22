package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type FlayerJungle struct {
	ctx *context.Status
}

func NewFlayerJungle() *FlayerJungle {
	return &FlayerJungle{
		ctx: context.Get(),
	}
}

func (a FlayerJungle) Name() string {
	return string(config.FlayerJungleRun)
}

func (a FlayerJungle) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) {
		return SequencerError
	}
	if !a.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (a FlayerJungle) Run(parameters *RunParameters) error {
	// Use Waypoint to Flayer Jungle
	err := action.WayPoint(area.FlayerJungle)
	if err != nil {
		return err
	}

	// Clear Flayer Jungle
	return action.ClearCurrentLevel(true, data.MonsterAnyFilter())
}
