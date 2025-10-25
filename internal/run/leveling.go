package run

import (
	"errors"
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
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

	if err := a.AdjustGameDifficulty(); err != nil {
		return err
	}

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

func (a Leveling) AdjustGameDifficulty() error {
	currentDifficulty := a.ctx.CharacterCfg.Game.Difficulty
	difficultyChanged := false
	lvl, _ := a.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
	rawFireRes, _ := a.ctx.Data.PlayerUnit.FindStat(stat.FireResist, 0)
	rawLightRes, _ := a.ctx.Data.PlayerUnit.FindStat(stat.LightningResist, 0)
	// Apply Hell difficulty penalty (-100) to resistances for effective values
	// TODO need to adjust penalty for classic (-60)
	effectiveFireRes := rawFireRes.Value - 100
	effectiveLightRes := rawLightRes.Value - 100

	switch currentDifficulty {
	case difficulty.Normal:
		//Switch to nightmare check
		if a.ctx.Data.Quests[quest.Act5EveOfDestruction].Completed() {
			if lvl.Value >= a.ctx.CharacterCfg.Game.Leveling.NightmareRequiredLevel {
				a.ctx.CharacterCfg.Game.Difficulty = difficulty.Nightmare
				difficultyChanged = true
			}
		}
	case difficulty.Nightmare:
		//switch to hell check
		isLowGold := action.IsLowGold()
		if a.ctx.Data.Quests[quest.Act5EveOfDestruction].Completed() {
			if lvl.Value >= a.ctx.CharacterCfg.Game.Leveling.HellRequiredLevel &&
				effectiveFireRes >= a.ctx.CharacterCfg.Game.Leveling.HellRequiredFireRes &&
				effectiveLightRes >= a.ctx.CharacterCfg.Game.Leveling.HellRequiredLightRes &&
				!isLowGold {
				a.ctx.CharacterCfg.Game.Difficulty = difficulty.Hell

				difficultyChanged = true
			}
		}
	case difficulty.Hell:
		// Revert to nightmare check
		totalGold := a.ctx.Data.PlayerUnit.TotalPlayerGold()
		if effectiveFireRes < a.ctx.CharacterCfg.Game.Leveling.HellRequiredFireRes ||
			effectiveLightRes < a.ctx.CharacterCfg.Game.Leveling.HellRequiredLightRes ||
			totalGold < 10000 {
			a.ctx.CharacterCfg.Game.Difficulty = difficulty.Nightmare
			difficultyChanged = true
		}
	}

	if difficultyChanged {
		a.ctx.Logger.Info("Difficulty changed to %s. Saving character configuration...", a.ctx.CharacterCfg.Game.Difficulty)
		// Use the new ConfigFolderName field here!
		if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
			a.ctx.Logger.Error("Failed to save character configuration: %s", err.Error())
			return fmt.Errorf("failed to save character configuration: %w", err)
		}
		return errors.New("res too low for hell")
	}
	return nil
}
