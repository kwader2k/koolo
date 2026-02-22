package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type DrifterCavern struct {
	ctx *context.Status
}

func NewDriverCavern() *DrifterCavern {
	return &DrifterCavern{
		ctx: context.Get(),
	}
}

func (s DrifterCavern) Name() string {
	return string(config.DrifterCavernRun)
}

func (a DrifterCavern) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) {
		return SequencerError
	}
	if !a.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (s DrifterCavern) Run(parameters *RunParameters) error {
	// Define a default monster filter
	monsterFilter := data.MonsterAnyFilter()

	// Update filter if we selected to clear only elites
	if s.ctx.CharacterCfg.Game.DrifterCavern.FocusOnElitePacks {
		monsterFilter = data.MonsterEliteFilter()
	}

	// Use the waypoint
	err := action.WayPoint(area.GlacialTrail)
	if err != nil {
		return err
	}

	// Move to the correct area
	if err = action.MoveToArea(area.DrifterCavern); err != nil {
		return err
	}

	// Clear the area
	return action.ClearCurrentLevel(s.ctx.CharacterCfg.Game.DrifterCavern.OpenChests, monsterFilter)
}
