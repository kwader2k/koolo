package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type TalRashaTombs struct {
	ctx *context.Status
}

func NewTalRashaTombs() *TalRashaTombs {
	return &TalRashaTombs{
		ctx: context.Get(),
	}
}

func (a TalRashaTombs) Name() string {
	return string(config.TalRashaTombsRun)
}

func (a TalRashaTombs) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) {
		return SequencerError
	}
	if !a.ctx.Data.Quests[quest.Act2TheSummoner].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

var talRashaTombs = []area.ID{area.TalRashasTomb1, area.TalRashasTomb2, area.TalRashasTomb3, area.TalRashasTomb4, area.TalRashasTomb5, area.TalRashasTomb6, area.TalRashasTomb7}

func (a TalRashaTombs) Run(parameters *RunParameters) error {

	// Iterate over all Tal Rasha Tombs
	for _, tomb := range talRashaTombs {

		// Use the waypoint to travel to Canyon Of The Magi
		err := action.WayPoint(area.CanyonOfTheMagi)
		if err != nil {
			return err
		}

		// Move to the next Tomb
		if err = action.MoveToArea(tomb); err != nil {
			return err
		}

		// Open a TP if we're the leader
		action.OpenTPIfLeader()

		// Buff before we start
		action.Buff()

		a.ctx.CharacterCfg.Character.ClearPathDist = 20
		if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
			a.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())
		}

		shouldInterrupt := func() bool {
			if parameters != nil && parameters.SequenceSettings != nil && parameters.SequenceSettings.MaxLevel != nil {
				ctx := context.Get()
				if lvl, found := ctx.Data.PlayerUnit.FindStat(stat.Level, 0); found {
					return lvl.Value > *parameters.SequenceSettings.MaxLevel
				}
			}
			return false
		}

		// Clear the Tomb
		if err = action.ClearCurrentLevelEx(true, data.MonsterAnyFilter(), shouldInterrupt); err != nil {
			return err
		}

		// Return to town
		if err = action.ReturnTown(); err != nil {
			return err
		}

		if shouldInterrupt() {
			return nil
		}
	}

	return nil
}
