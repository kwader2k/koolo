package run

import (
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Leveling struct {
	ctx *context.Status
}

func NewLeveling() *Leveling {
	return &Leveling{
		ctx: context.Get(),
	}
}

func (a Leveling) Name() string {
	return string(config.LevelingRun)
}

func (a Leveling) Run() error {
	// Adjust settings based on difficulty
	a.AdjustDifficultyConfig()

	a.GoToCurrentProgressionTown()

	if err := a.act1(); err != nil {
		return err
	}
	if err := a.act2(); err != nil {
		return err
	}
	if err := a.act3(); err != nil {
		return err
	}
	if err := a.act4(); err != nil {
		return err
	}
	if err := a.act5(); err != nil {
		return err
	}

	return nil
}

func (a Leveling) GoToCurrentProgressionTown() error {
	action.UpdateQuestLog(true)

	if !a.ctx.Data.PlayerUnit.Area.IsTown() {
		if err := action.ReturnTown(); err != nil {
			return err
		}
	}

	targetArea := a.GetCurrentProgressionTownWP()

	if targetArea != a.ctx.Data.PlayerUnit.Area {
		if err := action.WayPoint(a.GetCurrentProgressionTownWP()); err != nil {
			return err
		}
	}
	utils.Sleep(500)
	return nil
}

func (a Leveling) GetCurrentProgressionTownWP() area.ID {
	if a.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() {
		return area.Harrogath
	} else if a.ctx.Data.Quests[quest.Act3TheGuardian].Completed() {
		return area.ThePandemoniumFortress
	} else if a.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return area.KurastDocks
	} else if a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed() {
		return area.LutGholein
	}
	return area.RogueEncampment
}
