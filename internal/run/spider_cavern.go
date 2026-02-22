package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type SpiderCavern struct {
	ctx *context.Status
}

func NewSpiderCavern() *SpiderCavern {
	return &SpiderCavern{
		ctx: context.Get(),
	}
}

func (run SpiderCavern) Name() string {
	return string(config.SpiderCavernRun)
}

func (run SpiderCavern) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) {
		return SequencerError
	}
	if !run.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (run SpiderCavern) Run(parameters *RunParameters) error {
	// Define a default monster filter
	monsterFilter := data.MonsterAnyFilter()

	// Update filter if we selected to clear only elites
	if run.ctx.CharacterCfg.Game.SpiderCavern.FocusOnElitePacks {
		monsterFilter = data.MonsterEliteFilter()
	}

	// Use waypoint to Spider Forest
	err := action.WayPoint(area.SpiderForest)
	if err != nil {
		return err
	}

	// Move to the correct area
	if err = action.MoveToArea(area.SpiderCavern); err != nil {
		return err
	}

	// Clear the area
	action.ClearCurrentLevel(run.ctx.CharacterCfg.Game.SpiderCavern.OpenChests, monsterFilter)

	// Return to town
	if err = action.ReturnTown(); err != nil {
		return err
	}

	// Move to A4 if possible to shorten the run time
	err = action.WayPoint(area.ThePandemoniumFortress)
	if err != nil {
		return err
	}

	return nil
}
