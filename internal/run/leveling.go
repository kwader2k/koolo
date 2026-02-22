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

func (a Leveling) CheckConditions(parameters *RunParameters) SequencerResult {
	return SequencerError
}

func (a Leveling) Run(parameters *RunParameters) error {
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
		if a.ctx.Data.Quests[quest.Act5EveOfDestruction].Completed() {
			if lvl.Value >= a.ctx.CharacterCfg.Game.Leveling.HellRequiredLevel &&
				effectiveFireRes >= a.ctx.CharacterCfg.Game.Leveling.HellRequiredFireRes &&
				effectiveLightRes >= a.ctx.CharacterCfg.Game.Leveling.HellRequiredLightRes &&
				!action.IsBelowGoldPickupThreshold() {
				a.ctx.CharacterCfg.Game.Difficulty = difficulty.Hell

				difficultyChanged = true
			}
		}
	case difficulty.Hell:
		if effectiveFireRes < a.ctx.CharacterCfg.Game.Leveling.HellRequiredFireRes ||
			effectiveLightRes < a.ctx.CharacterCfg.Game.Leveling.HellRequiredLightRes ||
			action.IsLowGold() {
			a.ctx.CharacterCfg.Game.Difficulty = difficulty.Nightmare
			difficultyChanged = true
		}
	}

	if difficultyChanged {
		a.ctx.Logger.Info("Difficulty changed to %s. Saving character configuration...", a.ctx.CharacterCfg.Game.Difficulty)
		// Use the new ConfigFolderName field here!
		if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
			return fmt.Errorf("failed to save character configuration: %w", err)
		}

		if currentDifficulty == difficulty.Hell {
			return errors.New("res too low for hell, reverted to nightmare")
		} else {
			return errors.New("difficulty changed, restart")
		}
	}
	return nil
}

// setupLevelOneConfig centralizes the configuration logic for a new character.
func (a Leveling) setupLevelOneConfig() {
	a.ctx.CharacterCfg.Game.Difficulty = difficulty.Normal
	a.ctx.CharacterCfg.Game.Leveling.EnsurePointsAllocation = true
	a.ctx.CharacterCfg.Game.Leveling.EnsureKeyBinding = true
	a.ctx.CharacterCfg.Game.Leveling.AutoEquip = true
	a.ctx.CharacterCfg.Game.RunewordMaker.Enabled = true
	a.ctx.CharacterCfg.Game.RunewordMaker.EnabledRecipes = a.GetRunewords()
	a.ctx.CharacterCfg.Character.UseTeleport = false
	a.ctx.CharacterCfg.Character.UseMerc = false
	a.ctx.CharacterCfg.Character.StashToShared = false
	a.ctx.CharacterCfg.Game.UseCainIdentify = true
	a.ctx.CharacterCfg.ClassicMode = true
	a.ctx.CharacterCfg.Health.HealingPotionAt = 40
	a.ctx.CharacterCfg.Health.ManaPotionAt = 25
	a.ctx.CharacterCfg.Health.RejuvPotionAtLife = 0
	a.ctx.CharacterCfg.Health.ChickenAt = 7
	a.ctx.CharacterCfg.Health.TownChickenAt = 15
	a.ctx.CharacterCfg.Gambling.Enabled = true
	a.ctx.CharacterCfg.Health.MercRejuvPotionAt = 40
	a.ctx.CharacterCfg.Health.MercChickenAt = 0
	a.ctx.CharacterCfg.Health.MercHealingPotionAt = 25
	a.ctx.CharacterCfg.MaxGameLength = 1200
	a.ctx.CharacterCfg.CubeRecipes.Enabled = true
	a.ctx.CharacterCfg.CubeRecipes.EnabledRecipes = []string{"Perfect Amethyst", "Reroll GrandCharms", "Caster Amulet"}
	a.ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "mana"}
	a.ctx.CharacterCfg.BackToTown.NoHpPotions = true
	a.ctx.CharacterCfg.BackToTown.NoMpPotions = true
	a.ctx.CharacterCfg.BackToTown.MercDied = false
	a.ctx.CharacterCfg.BackToTown.EquipmentBroken = false
	a.ctx.CharacterCfg.Game.Tristram.ClearPortal = false
	a.ctx.CharacterCfg.Game.Tristram.FocusOnElitePacks = true
	a.ctx.CharacterCfg.Game.Countess.ClearFloors = false
	a.ctx.CharacterCfg.Game.Pit.MoveThroughBlackMarsh = true
	a.ctx.CharacterCfg.Game.Pit.OpenChests = true
	a.ctx.CharacterCfg.Game.Pit.FocusOnElitePacks = false
	a.ctx.CharacterCfg.Game.Pit.OnlyClearLevel2 = false
	a.ctx.CharacterCfg.Game.Andariel.ClearRoom = true
	a.ctx.CharacterCfg.Game.Mephisto.KillCouncilMembers = false
	a.ctx.CharacterCfg.Game.Mephisto.OpenChests = false
	a.ctx.CharacterCfg.Game.Mephisto.ExitToA4 = true
	a.ctx.CharacterCfg.Inventory.InventoryLock = [][]int{
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	}
	a.ctx.CharacterCfg.Game.InteractWithShrines = true
	a.ctx.CharacterCfg.Character.BuffOnNewArea = true
	a.ctx.CharacterCfg.Character.BuffAfterWP = true
	a.ctx.CharacterCfg.Character.UseExtraBuffs = true
	a.ctx.CharacterCfg.Game.MinGoldPickupThreshold = 2000
	a.ctx.CharacterCfg.Inventory.HealingPotionCount = 4
	a.ctx.CharacterCfg.Inventory.ManaPotionCount = 8
	a.ctx.CharacterCfg.Inventory.RejuvPotionCount = 0
	a.ctx.CharacterCfg.Character.ShouldHireAct2MercFrozenAura = true

	a.ensureDifficultySwitchSettings()
	levelingCharacter, isLevelingChar := a.ctx.Char.(context.LevelingCharacter)
	if isLevelingChar {
		levelingCharacter.InitialCharacterConfigSetup()
	}

	if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
		a.ctx.Logger.Error(fmt.Sprintf("Failed to save character configuration: %s", err.Error()))
	}
}

// adjustDifficultyConfig centralizes difficulty-based configuration changes.
func (a Leveling) AdjustDifficultyConfig() {
	lvl, _ := a.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)

	//Let setupLevelOneConfig do the initial setup
	if lvl.Value == 1 {
		a.setupLevelOneConfig()
		return
	}

	a.ensureDifficultySwitchSettings()

	a.ctx.CharacterCfg.Game.RunewordMaker.EnabledRecipes = a.GetRunewords()
	a.ctx.CharacterCfg.Game.MinGoldPickupThreshold = 5000 * lvl.Value
	if lvl.Value >= 4 && lvl.Value < 24 {
		a.ctx.CharacterCfg.Health.HealingPotionAt = 85
		a.ctx.CharacterCfg.Health.TownChickenAt = 25
		a.ctx.CharacterCfg.Character.ClearPathDist = 15
	}
	if lvl.Value >= 24 {
		switch a.ctx.CharacterCfg.Game.Difficulty {
		case difficulty.Normal:
			a.ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "mana"}
			a.ctx.CharacterCfg.Health.MercHealingPotionAt = 55
			a.ctx.CharacterCfg.Health.MercRejuvPotionAt = 0
			a.ctx.CharacterCfg.Health.HealingPotionAt = 85
			a.ctx.CharacterCfg.Health.ChickenAt = 30
			a.ctx.CharacterCfg.Health.TownChickenAt = 50
			a.ctx.CharacterCfg.Character.ClearPathDist = 15

		case difficulty.Nightmare:
			a.ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "mana"}
			a.ctx.CharacterCfg.Health.MercHealingPotionAt = 55
			a.ctx.CharacterCfg.Health.MercRejuvPotionAt = 0
			a.ctx.CharacterCfg.Health.HealingPotionAt = 85
			a.ctx.CharacterCfg.Health.ChickenAt = 30
			a.ctx.CharacterCfg.Health.TownChickenAt = 50
			a.ctx.CharacterCfg.Character.ClearPathDist = 15

		case difficulty.Hell:
			a.ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "rejuvenation"}
			a.ctx.CharacterCfg.Health.MercHealingPotionAt = 80
			a.ctx.CharacterCfg.Health.MercRejuvPotionAt = 40
			a.ctx.CharacterCfg.Health.HealingPotionAt = 90
			a.ctx.CharacterCfg.Health.RejuvPotionAtLife = 70
			a.ctx.CharacterCfg.Health.ChickenAt = 40
			a.ctx.CharacterCfg.Health.TownChickenAt = 60
			a.ctx.CharacterCfg.Character.ClearPathDist = 15
			a.ctx.CharacterCfg.Inventory.ManaPotionCount = 4
		}
	}

	levelingCharacter, isLevelingChar := a.ctx.Char.(context.LevelingCharacter)
	if isLevelingChar {
		levelingCharacter.AdjustCharacterConfig()
	}

	if err := config.SaveSupervisorConfig(a.ctx.CharacterCfg.ConfigFolderName, a.ctx.CharacterCfg); err != nil {
		a.ctx.Logger.Error(fmt.Sprintf("Failed to save character configuration: %s", err.Error()))
	}
}

func (a Leveling) GetRunewords() []string {
	enabledRunewordRecipes := []string{"Ancients' Pledge", "Lore", "Insight", "Smoke", "Treachery", "Call to Arms"}

	if !a.ctx.CharacterCfg.Game.IsNonLadderChar {
		enabledRunewordRecipes = append(enabledRunewordRecipes, "Bulwark", "Hustle")
		a.ctx.Logger.Info("Ladder character detected. Adding Bulwark and Hustle runewords.")
	}

	ch, isLevelingChar := a.ctx.Char.(context.LevelingCharacter)
	if isLevelingChar {
		additionalRunewords := ch.GetAdditionalRunewords()
		enabledRunewordRecipes = append(enabledRunewordRecipes, additionalRunewords...)
	}

	return enabledRunewordRecipes
}

func (a Leveling) ensureDifficultySwitchSettings() {
	//Values have never been set (or user is dumb), reset to default
	if a.ctx.CharacterCfg.Game.Leveling.NightmareRequiredLevel <= 1 &&
		a.ctx.CharacterCfg.Game.Leveling.HellRequiredLevel == 1 &&
		a.ctx.CharacterCfg.Game.Leveling.HellRequiredFireRes == 0 &&
		a.ctx.CharacterCfg.Game.Leveling.HellRequiredLightRes == 0 {
		a.ctx.CharacterCfg.Game.Leveling.NightmareRequiredLevel = 41
		a.ctx.CharacterCfg.Game.Leveling.HellRequiredLevel = 70
		a.ctx.CharacterCfg.Game.Leveling.HellRequiredFireRes = 15
		a.ctx.CharacterCfg.Game.Leveling.HellRequiredLightRes = -10
	}
}
