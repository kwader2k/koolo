package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type ArachnidLair struct {
	ctx *context.Status
}

func NewArachnidLair() *ArachnidLair {
	return &ArachnidLair{
		ctx: context.Get(),
	}
}

func (a ArachnidLair) Name() string {
	return string(config.ArachnidLairRun)
}

func (a ArachnidLair) CheckConditions(parameters *RunParameters) SequencerResult {
	if !IsFarmingRun(parameters) {
		return SequencerError
	}
	if !a.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (a ArachnidLair) Run(parameters *RunParameters) error {
	filter := data.MonsterAnyFilter()
	if a.ctx.CharacterCfg.Game.ArachnidLair.FocusOnElitePacks {
		filter = data.MonsterEliteFilter()
	}

	err := action.WayPoint(area.SpiderForest)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.SpiderCave)
	if err != nil {
		return err
	}

	action.OpenTPIfLeader()

	// Clear ArachnidLair
	return action.ClearCurrentLevel(a.ctx.CharacterCfg.Game.ArachnidLair.OpenChests, filter)
}
