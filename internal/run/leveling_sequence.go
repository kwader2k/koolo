package run

import (
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"slices"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type LevelingSequence struct {
	ctx      *context.Status
	Settings *LevelingSequenceSettings
	DoneRuns []string
}

type LevelingSequenceSettings struct {
	Normal    DifficultyLevelingSettings `json:"normal"`
	Nightmare DifficultyLevelingSettings `json:"nightmare"`
	Hell      DifficultyLevelingSettings `json:"hell"`
}

type DifficultyLevelingSettings struct {
	Quests                   []SequenceSettings            `json:"quests"`
	BeforeQuests             []SequenceSettings            `json:"beforeQuests"`
	AfterQuests              []SequenceSettings            `json:"afterQuests"`
	NextDifficultyConditions *DifficultyConditionsSettings `json:"nextDifficultyConditions,omitempty"`
	StayDifficultyConditions *DifficultyConditionsSettings `json:"stayDifficultyConditions,omitempty"`
	ConfigSettings           []ConfigLevelingSettings      `json:"configSettings"`
}

type SequenceSettings struct {
	Run                          string `json:"run"`
	MinLevel                     *int   `json:"minLevel,omitempty"`
	MaxLevel                     *int   `json:"maxLevel,omitempty"`
	LowGoldRun                   bool   `json:"lowGoldRun,omitempty"`
	SkipTownChores               bool   `json:"skipTownChores,omitempty"`
	ExitGame                     bool   `json:"exitGame,omitempty"`
	StopIfCheckFails             bool   `json:"stopIfCheckFails,omitempty"`
	SkipCountessWhenStealthReady bool   `json:"skipCountessWhenStealthReady,omitempty"`
	Parameters                   string `json:"parameters,omitempty"`
}

type DifficultyConditionsSettings struct {
	Level              *int `json:"level,omitempty"`
	FireRes            *int `json:"fireRes,omitempty"`
	ColdRes            *int `json:"coldRes,omitempty"`
	LightRes           *int `json:"lightRes,omitempty"`
	PoisonRes          *int `json:"poisonRes,omitempty"`
	AboveLowGold       bool `json:"aboveLowGold,omitempty"`
	AboveGoldThreshold bool `json:"aboveGoldThreshold,omitempty"`
}

type ConfigLevelingSettings struct {
	Level          *int                    `json:"level,omitempty"`
	HealthSettings *HealthLevelingSettings `json:"healthSettings,omitempty"`
}

type HealthLevelingSettings struct {
	HealingPotionAt      *int      `json:"healingPotionAt,omitempty"`
	ManaPotionAt         *int      `json:"manaPotionAt,omitempty"`
	RejuvPotionAtLife    *int      `json:"rejuvPotionAtLife,omitempty"`
	RejuvPotionAtMana    *int      `json:"rejuvPotionAtMana,omitempty"`
	MercHealingPotionAt  *int      `json:"mercHealingPotionAt,omitempty"`
	MercRejuvPotionAt    *int      `json:"mercRejuvPotionAt,omitempty"`
	ChickenAt            *int      `json:"chickenAt,omitempty"`
	TownChickenAt        *int      `json:"townChickenAt,omitempty"`
	MercChickenAt        *int      `json:"mercChickenAt,omitempty"`
	HealingPotionCount   *int      `json:"healingPotionCount,omitempty"`
	ManaPotionCount      *int      `json:"manaPotionCount,omitempty"`
	RejuvPotionCount     *int      `json:"rejuvPotionCount,omitempty"`
	BeltColumns          *[]string `json:"beltColumns,omitempty"`
	ChickenAmplifyDamage *bool     `json:"chickenAmplifyDamage,omitempty"`
	ChickenDecrepify     *bool     `json:"chickenDecrepify,omitempty"`
	ChickenLowerResist   *bool     `json:"chickenLowerResist,omitempty"`
	ChickenBloodMana     *bool     `json:"chickenBloodMana,omitempty"`
	ChickenFanaticism    *bool     `json:"chickenFanaticism,omitempty"`
	ChickenMight         *bool     `json:"chickenMight,omitempty"`
	ChickenConviction    *bool     `json:"chickenConviction,omitempty"`
	ChickenHolyFire      *bool     `json:"chickenHolyFire,omitempty"`
	ChickenBlessedAim    *bool     `json:"chickenBlessedAim,omitempty"`
	ChickenHolyFreeze    *bool     `json:"chickenHolyFreeze,omitempty"`
	ChickenHolyShock     *bool     `json:"chickenHolyShock,omitempty"`
}

func NewLevelingSequence() *LevelingSequence {
	return &LevelingSequence{
		ctx: context.Get(),
	}
}

func (ls LevelingSequence) Name() string {
	return string(config.LevelingSequenceRun)
}

func (ls LevelingSequence) CheckConditions(parameters *RunParameters) SequencerResult {
	return SequencerError
}

func (ls LevelingSequence) Run(parameters *RunParameters) error {
	if loadErr := ls.LoadSettings(); loadErr != nil {
		return loadErr
	}

	ls.GoToCurrentProgressionTown()

	difficultyChanged, difErr := ls.AdjustDifficulty()
	if difErr != nil {
		return difErr
	} else if difficultyChanged {
		return nil
	}

	ls.AdjustDifficultyConfig()

	if healthErr := ls.AdjustHealthConfig(); healthErr != nil {
		return healthErr
	}

	difficultySettings := ls.GetCurrentDifficultySettings()
	if difficultySettings == nil {
		return errors.New("couldn't find current difficulty leveling settings")
	}
	if sequencesErr := ls.RunDifficultySequences(difficultySettings); sequencesErr != nil {
		ls.GoToCurrentProgressionTown()
		return sequencesErr
	}
	return nil
}

func (ls *LevelingSequence) RunDifficultySequences(settings *DifficultyLevelingSettings) error {

	shouldContinue, err := ls.RunSequences(settings.BeforeQuests, true)
	if err != nil {
		return fmt.Errorf("error during before quests farming : %s", err)
	} else if !shouldContinue {
		return nil
	}

	shouldContinue, err = ls.RunSequences(settings.Quests, false)
	if err != nil {
		return fmt.Errorf("error during quests : %s", err)
	} else if !shouldContinue {
		return nil
	}

	shouldContinue, err = ls.RunSequences(settings.AfterQuests, true)
	if err != nil {
		return fmt.Errorf("error during after quests farming : %s", err)
	} else if !shouldContinue {
		return nil
	}

	return nil
}

func (ls *LevelingSequence) RunSequences(sequences []SequenceSettings, farmSequence bool) (bool, error) {
	for _, sequenceSettings := range sequences {
		run := BuildRun(sequenceSettings.Run)
		if run == nil {
			return false, fmt.Errorf("couldn't build run %s", sequenceSettings.Run)
		}

		requirementsOk, checkErr := ls.CheckSequenceRequirements(run, sequenceSettings, farmSequence)
		if checkErr != nil {
			return false, fmt.Errorf("error while checking sequence %s : %s", sequenceSettings.Run, checkErr)
		}

		if !requirementsOk {
			if sequenceSettings.StopIfCheckFails {
				return true, nil
			}
		} else {
			parameters := BuildRunParameters(farmSequence, &sequenceSettings)
			runCondition := run.CheckConditions(parameters)
			switch runCondition {
			case SequencerError:
				if farmSequence {
					return false, fmt.Errorf("run %s not supported as farming sequence", sequenceSettings.Run)
				} else {
					return false, fmt.Errorf("run %s not supported as quest sequence", sequenceSettings.Run)
				}
			case SequencerStop:
				if IsQuestRun(parameters) {
					ls.ctx.Logger.Info("Stopping quest sequences due to condition check.", "run", sequenceSettings.Run)
				} else {
					ls.ctx.Logger.Info("Stopping farming sequences due to condition check.", "run", sequenceSettings.Run)
				}
				return true, nil
			case SequencerSkip:
				continue
			}

			if slices.Contains(ls.DoneRuns, sequenceSettings.Run) {
				continue
			}

			if IsQuestRun(parameters) {
				ls.ctx.Logger.Info("Starting quest run", "run", sequenceSettings.Run)
			} else {
				ls.ctx.Logger.Info("Starting farming run", "run", sequenceSettings.Run)
			}

			shouldContinue, runErr := ls.RunSequence(run, sequenceSettings, parameters)
			if runErr != nil {
				return false, fmt.Errorf("error during run %s : %s", sequenceSettings.Run, runErr)
			}

			if IsQuestRun(parameters) {
				ls.ctx.Logger.Info("Quest run ended successfully", "run", sequenceSettings.Run)
			} else {
				ls.ctx.Logger.Info("Farming run ended successfully", "run", sequenceSettings.Run)
			}

			ls.DoneRuns = append(ls.DoneRuns, sequenceSettings.Run)
			if !shouldContinue {
				if IsQuestRun(parameters) {
					ls.ctx.Logger.Info("Stopping quest sequences after run", "run", sequenceSettings.Run)
				} else {
					ls.ctx.Logger.Info("Stopping farming sequences after run", "run", sequenceSettings.Run)
				}
				return false, nil
			}
		}
	}
	return true, nil
}

func (ls LevelingSequence) CheckSequenceRequirements(run Run, sequenceSettings SequenceSettings, farmSequence bool) (bool, error) {
	lvl, found := ls.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
	if !found {
		return false, errors.New("adjust health config : level not found")
	}

	playerLevel := lvl.Value

	if sequenceSettings.MinLevel != nil {
		if playerLevel < *sequenceSettings.MinLevel {
			return false, nil
		}
	}

	if sequenceSettings.MaxLevel != nil {
		if playerLevel > *sequenceSettings.MaxLevel {
			return false, nil
		}
	}

	if sequenceSettings.LowGoldRun && !action.IsLowGold() {
		return false, nil
	}

	return true, nil
}

func (ls LevelingSequence) RunSequence(run Run, sequenceSettings SequenceSettings, parameters *RunParameters) (bool, error) {
	if err := run.Run(parameters); err != nil {
		return false, err
	}

	if sequenceSettings.ExitGame {
		return false, nil
	}

	if !ls.ctx.Data.PlayerUnit.Area.IsTown() {
		if backErr := action.ReturnTown(); backErr != nil {
			return false, backErr
		}
	}

	if !sequenceSettings.SkipTownChores {
		ls.GoToCurrentProgressionTown()
		action.PreRun(false)
	}

	return true, nil
}

func (ls *LevelingSequence) LoadSettings() error {
	cwd, err := os.Getwd()
	if err != nil {
		return fmt.Errorf("error getting current working directory: %w", err)
	}
	getAbsPath := func(relPath string) string {
		return filepath.Join(cwd, relPath)
	}
	rawName := strings.TrimSpace(ls.ctx.CharacterCfg.Game.LevelingSequence.SequenceFile)
	if rawName == "" {
		message := "Stopping supervisor now. Leveling sequence file is not set.\nSelect a sequence.json file in your config and restart the supervisor"
		ls.ctx.Logger.Error("leveling sequence file is not set")
		utils.ShowDialog("Missing leveling sequence for supervisor "+ls.ctx.Name, message)
		ls.ctx.StopSupervisor()
		return errors.New("leveling sequence file is not set")
	}
	fileName := rawName
	if strings.HasSuffix(strings.ToLower(fileName), ".json") {
		fileName = strings.TrimSuffix(fileName, filepath.Ext(fileName))
	}
	levelingSequencesPath := getAbsPath(filepath.Join("config", "template", "sequences_leveling"))
	sequenceFilePath := filepath.Join(levelingSequencesPath, fileName+".json")
	jsonData, err := utils.GetJsonData(sequenceFilePath)
	if err != nil {
		message := fmt.Sprintf("Stopping supervisor now. Unable to load leveling sequence file: %s.json", fileName)
		ls.ctx.Logger.Error("failed to load sequence", "file name", fileName, "error", err)
		utils.ShowDialog("Invalid leveling sequence for supervisor "+ls.ctx.Name, message)
		ls.ctx.StopSupervisor()
		return err
	}

	var sequenceSettings LevelingSequenceSettings
	err = json.Unmarshal(jsonData, &sequenceSettings)
	if err != nil {
		message := fmt.Sprintf("Stopping supervisor now. Invalid leveling sequence JSON: %s.json", fileName)
		ls.ctx.Logger.Error("failed to parse sequence json", "file name", fileName, "error", err)
		utils.ShowDialog("Invalid leveling sequence for supervisor "+ls.ctx.Name, message)
		ls.ctx.StopSupervisor()
		return err
	}

	ls.Settings = &sequenceSettings
	return nil
}

func (ls LevelingSequence) AdjustHealthConfig() error {
	if ls.Settings == nil {
		return errors.New("sequence settings not loaded")
	}

	lvl, found := ls.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
	if !found {
		return errors.New("adjust health config : level not found")
	}

	playerLevel := lvl.Value

	currentDifficulty := ls.ctx.CharacterCfg.Game.Difficulty
	settingsApplied := ls.ApplyConfigSettings(ls.Settings.Normal.ConfigSettings, playerLevel)

	if currentDifficulty == difficulty.Nightmare || currentDifficulty == difficulty.Hell {
		settingsApplied = ls.ApplyConfigSettings(ls.Settings.Nightmare.ConfigSettings, playerLevel) || settingsApplied
	}

	if currentDifficulty == difficulty.Hell {
		settingsApplied = ls.ApplyConfigSettings(ls.Settings.Hell.ConfigSettings, playerLevel) || settingsApplied
	}

	if settingsApplied {
		if err := config.SaveSupervisorConfig(ls.ctx.CharacterCfg.ConfigFolderName, ls.ctx.CharacterCfg); err != nil {
			return err
		}
	}

	return nil
}

func (ls LevelingSequence) ApplyConfigSettings(configSettings []ConfigLevelingSettings, playerLevel int) bool {
	settingsApplied := false

	for _, configSetting := range configSettings {
		needApplySettings := false

		if configSetting.Level != nil {
			if playerLevel >= *configSetting.Level {
				needApplySettings = true
			} else {
				break
			}
		} else {
			needApplySettings = true
		}

		if needApplySettings {
			if configSetting.HealthSettings != nil {
				ls.ApplyHealthSetting(*configSetting.HealthSettings)
				settingsApplied = true
			}
		}
	}

	return settingsApplied
}

func (ls LevelingSequence) ApplyHealthSetting(healthSetting HealthLevelingSettings) error {
	if healthSetting.HealingPotionAt != nil {
		ls.ctx.CharacterCfg.Health.HealingPotionAt = *healthSetting.HealingPotionAt
	}
	if healthSetting.ManaPotionAt != nil {
		ls.ctx.CharacterCfg.Health.ManaPotionAt = *healthSetting.ManaPotionAt
	}
	if healthSetting.RejuvPotionAtLife != nil {
		ls.ctx.CharacterCfg.Health.RejuvPotionAtLife = *healthSetting.RejuvPotionAtLife
	}
	if healthSetting.RejuvPotionAtMana != nil {
		ls.ctx.CharacterCfg.Health.RejuvPotionAtMana = *healthSetting.RejuvPotionAtMana
	}
	if healthSetting.MercHealingPotionAt != nil {
		ls.ctx.CharacterCfg.Health.MercHealingPotionAt = *healthSetting.MercHealingPotionAt
	}
	if healthSetting.MercRejuvPotionAt != nil {
		ls.ctx.CharacterCfg.Health.MercRejuvPotionAt = *healthSetting.MercRejuvPotionAt
	}
	if healthSetting.ChickenAt != nil {
		ls.ctx.CharacterCfg.Health.ChickenAt = *healthSetting.ChickenAt
	}
	if healthSetting.TownChickenAt != nil {
		ls.ctx.CharacterCfg.Health.TownChickenAt = *healthSetting.TownChickenAt
	}
	if healthSetting.MercChickenAt != nil {
		ls.ctx.CharacterCfg.Health.MercChickenAt = *healthSetting.MercChickenAt
	}
	if healthSetting.HealingPotionCount != nil {
		ls.ctx.CharacterCfg.Inventory.HealingPotionCount = *healthSetting.HealingPotionCount
	}
	if healthSetting.ManaPotionCount != nil {
		ls.ctx.CharacterCfg.Inventory.ManaPotionCount = *healthSetting.ManaPotionCount
	}
	if healthSetting.RejuvPotionCount != nil {
		ls.ctx.CharacterCfg.Inventory.RejuvPotionCount = *healthSetting.RejuvPotionCount
	}
	if healthSetting.BeltColumns != nil {
		ls.applyBeltColumnsOverride(*healthSetting.BeltColumns)
	}
	if healthSetting.ChickenAmplifyDamage != nil {
		ls.ctx.CharacterCfg.ChickenOnCurses.AmplifyDamage = *healthSetting.ChickenAmplifyDamage
	}
	if healthSetting.ChickenDecrepify != nil {
		ls.ctx.CharacterCfg.ChickenOnCurses.Decrepify = *healthSetting.ChickenDecrepify
	}
	if healthSetting.ChickenLowerResist != nil {
		ls.ctx.CharacterCfg.ChickenOnCurses.LowerResist = *healthSetting.ChickenLowerResist
	}
	if healthSetting.ChickenBloodMana != nil {
		ls.ctx.CharacterCfg.ChickenOnCurses.BloodMana = *healthSetting.ChickenBloodMana
	}
	if healthSetting.ChickenFanaticism != nil {
		ls.ctx.CharacterCfg.ChickenOnAuras.Fanaticism = *healthSetting.ChickenFanaticism
	}
	if healthSetting.ChickenMight != nil {
		ls.ctx.CharacterCfg.ChickenOnAuras.Might = *healthSetting.ChickenMight
	}
	if healthSetting.ChickenConviction != nil {
		ls.ctx.CharacterCfg.ChickenOnAuras.Conviction = *healthSetting.ChickenConviction
	}
	if healthSetting.ChickenHolyFire != nil {
		ls.ctx.CharacterCfg.ChickenOnAuras.HolyFire = *healthSetting.ChickenHolyFire
	}
	if healthSetting.ChickenBlessedAim != nil {
		ls.ctx.CharacterCfg.ChickenOnAuras.BlessedAim = *healthSetting.ChickenBlessedAim
	}
	if healthSetting.ChickenHolyFreeze != nil {
		ls.ctx.CharacterCfg.ChickenOnAuras.HolyFreeze = *healthSetting.ChickenHolyFreeze
	}
	if healthSetting.ChickenHolyShock != nil {
		ls.ctx.CharacterCfg.ChickenOnAuras.HolyShock = *healthSetting.ChickenHolyShock
	}

	return nil
}

func (ls LevelingSequence) applyBeltColumnsOverride(columns []string) {
	if len(columns) == 0 {
		return
	}
	for idx := range ls.ctx.CharacterCfg.Inventory.BeltColumns {
		if idx >= len(columns) {
			break
		}
		value := columns[idx]
		if value == "" {
			continue
		}
		ls.ctx.CharacterCfg.Inventory.BeltColumns[idx] = value
	}
}

func (ls LevelingSequence) AdjustDifficulty() (bool, error) {
	if ls.Settings == nil {
		return false, errors.New("sequence settings not loaded")
	}

	difficultySettings := ls.GetCurrentDifficultySettings()
	if difficultySettings == nil {
		return false, fmt.Errorf("failed to find difficulty settings for %s", ls.ctx.CharacterCfg.Game.Difficulty)
	}

	difficultyChanged := false

	//Check if we reached the level for current difficulty setting
	difficulties := []difficulty.Difficulty{difficulty.Normal, difficulty.Nightmare}
	for _, difficulty := range difficulties {
		if ls.ctx.CharacterCfg.Game.Difficulty == difficulty {
			break
		}
		diffSettings := ls.GetDifficultySettings(difficulty)
		nextDiff := ls.GetNextDifficulty(difficulty)
		//We don't meet level requirements, revert to this difficulty
		if !ls.CheckDifficultyConditions(diffSettings.NextDifficultyConditions, nextDiff, true) {
			if difficulty != ls.ctx.CharacterCfg.Game.Difficulty {
				ls.ctx.Logger.Info("Reverting difficulty", "difficulty", difficulty)
				ls.ctx.CharacterCfg.Game.Difficulty = difficulty
				difficultyChanged = true
			}
			//We break here to not evaluate next difficutlies
			break
		}
	}

	//Check if we should stay in current difficulty
	if !difficultyChanged && difficultySettings.StayDifficultyConditions != nil {
		if !ls.CheckDifficultyConditions(difficultySettings.StayDifficultyConditions, ls.ctx.CharacterCfg.Game.Difficulty, false) {
			targetDifficulty := ls.GetPreviousDifficulty()
			if targetDifficulty != ls.ctx.CharacterCfg.Game.Difficulty {
				ls.ctx.Logger.Info("Reverting difficulty", "difficulty", targetDifficulty)
				ls.ctx.CharacterCfg.Game.Difficulty = targetDifficulty
				difficultyChanged = true
			}
		}
	}

	//Check if we reached fully requirements for next difficulty
	if !difficultyChanged && difficultySettings.NextDifficultyConditions != nil && ls.ctx.Data.Quests[quest.Act5EveOfDestruction].Completed() {
		nextDifficulty := ls.GetCurrentNextDifficulty()
		if nextDifficulty != ls.ctx.CharacterCfg.Game.Difficulty {
			if ls.CheckDifficultyConditions(difficultySettings.NextDifficultyConditions, nextDifficulty, false) {
				ls.ctx.Logger.Info("Changing difficulty", "difficulty", nextDifficulty)
				ls.ctx.CharacterCfg.Game.Difficulty = nextDifficulty
				difficultyChanged = true
			}
		}
	}

	if difficultyChanged {
		if err := config.SaveSupervisorConfig(ls.ctx.CharacterCfg.ConfigFolderName, ls.ctx.CharacterCfg); err != nil {
			return false, err
		}
		return true, nil
	}

	return false, nil
}

func (ls LevelingSequence) GetPreviousDifficulty() difficulty.Difficulty {
	if ls.ctx.CharacterCfg.Game.Difficulty == difficulty.Hell {
		return difficulty.Nightmare
	}

	return difficulty.Normal
}

func (ls LevelingSequence) GetCurrentNextDifficulty() difficulty.Difficulty {
	return ls.GetNextDifficulty(ls.ctx.CharacterCfg.Game.Difficulty)
}

func (ls LevelingSequence) GetNextDifficulty(diff difficulty.Difficulty) difficulty.Difficulty {
	if diff == difficulty.Normal {
		return difficulty.Nightmare
	}

	return difficulty.Hell
}

func (ls LevelingSequence) GetCurrentDifficultySettings() *DifficultyLevelingSettings {
	return ls.GetDifficultySettings(ls.ctx.CharacterCfg.Game.Difficulty)
}

func (ls LevelingSequence) GetDifficultySettings(diff difficulty.Difficulty) *DifficultyLevelingSettings {
	if ls.Settings == nil {
		ls.ctx.Logger.Error("sequence settings not loaded")
		return nil
	}

	switch diff {
	case difficulty.Normal:
		return &ls.Settings.Normal
	case difficulty.Nightmare:
		return &ls.Settings.Nightmare
	case difficulty.Hell:
		return &ls.Settings.Hell
	}

	return nil
}

func (ls LevelingSequence) CheckDifficultyConditions(conditions *DifficultyConditionsSettings, targetDifficulty difficulty.Difficulty, levelOnly bool) bool {
	if conditions == nil {
		return true
	}

	//Level check
	if conditions.Level != nil {
		if lvl, found := ls.ctx.Data.PlayerUnit.FindStat(stat.Level, 0); found {
			if lvl.Value < *conditions.Level {
				return false
			}
		} else {
			ls.ctx.Logger.Error("leveling difficulty check : couldn't find player level")
			return false
		}
	}
	if levelOnly {
		return true
	}

	resPenalty := 0
	switch targetDifficulty {
	case difficulty.Nightmare:
		resPenalty = 40
		//TODO Classic
		//resPenalty = 20
	case difficulty.Hell:
		resPenalty = 100
		//TODO Classic
		//resPenalty = 50
	}

	if !ls.CheckResCondition(stat.LightningResist, conditions.LightRes, resPenalty) {
		return false
	}

	if !ls.CheckResCondition(stat.ColdResist, conditions.ColdRes, resPenalty) {
		return false
	}

	if !ls.CheckResCondition(stat.FireResist, conditions.FireRes, resPenalty) {
		return false
	}

	if !ls.CheckResCondition(stat.PoisonResist, conditions.PoisonRes, resPenalty) {
		return false
	}

	if conditions.AboveLowGold && action.IsLowGold() {
		return false
	}

	if conditions.AboveGoldThreshold && action.IsBelowGoldPickupThreshold() {
		return false
	}

	return true
}

func (ls LevelingSequence) CheckResCondition(resType stat.ID, resTarget *int, resPenalty int) bool {

	if resTarget != nil {
		if res, found := ls.ctx.Data.PlayerUnit.FindStat(resType, 0); found {
			if res.Value-resPenalty < *resTarget {
				return false
			}
		} else {
			ls.ctx.Logger.Error("leveling difficulty check : couldn't find player resistance stat")
			return false
		}
	}
	return true
}

func (ls LevelingSequence) GoToCurrentProgressionTown() error {
	if !ls.ctx.Data.PlayerUnit.Area.IsTown() {
		if err := action.ReturnTown(); err != nil {
			return err
		}
	}

	targetArea := ls.GetCurrentProgressionTownWP()

	if targetArea != ls.ctx.Data.PlayerUnit.Area {
		if err := action.WayPoint(ls.GetCurrentProgressionTownWP()); err != nil {
			return err
		}
	}
	utils.Sleep(500)
	return nil
}

func (ls LevelingSequence) GetCurrentProgressionTownWP() area.ID {
	if ls.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() {
		return area.Harrogath
	} else if ls.ctx.Data.Quests[quest.Act3TheGuardian].Completed() {
		return area.ThePandemoniumFortress
	} else if ls.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return area.KurastDocks
	} else if ls.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed() {
		return area.LutGholein
	}
	return area.RogueEncampment
}

// setupLevelOneConfig centralizes the configuration logic for a new character.
func (ls LevelingSequence) setupLevelOneConfig() {
	ls.ctx.CharacterCfg.Game.Difficulty = difficulty.Normal
	ls.ctx.CharacterCfg.Game.Leveling.EnsurePointsAllocation = true
	ls.ctx.CharacterCfg.Game.Leveling.EnsureKeyBinding = true
	ls.ctx.CharacterCfg.Game.Leveling.AutoEquip = true
	ls.ctx.CharacterCfg.Game.RunewordMaker.Enabled = true
	ls.ctx.CharacterCfg.Game.RunewordMaker.EnabledRecipes = ls.GetRunewords()
	ls.ctx.CharacterCfg.Character.UseTeleport = false
	ls.ctx.CharacterCfg.Character.UseMerc = false
	ls.ctx.CharacterCfg.Character.StashToShared = false
	ls.ctx.CharacterCfg.Game.UseCainIdentify = true
	ls.ctx.CharacterCfg.ClassicMode = true
	ls.ctx.CharacterCfg.Gambling.Enabled = true
	ls.ctx.CharacterCfg.MaxGameLength = 1200
	ls.ctx.CharacterCfg.CubeRecipes.Enabled = true
	ls.ctx.CharacterCfg.CubeRecipes.EnabledRecipes = []string{"Perfect Amethyst", "Reroll GrandCharms", "Caster Amulet"}
	ls.ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "mana"}
	ls.ctx.CharacterCfg.BackToTown.NoHpPotions = true
	ls.ctx.CharacterCfg.BackToTown.NoMpPotions = true
	ls.ctx.CharacterCfg.BackToTown.MercDied = false
	ls.ctx.CharacterCfg.BackToTown.EquipmentBroken = false
	ls.ctx.CharacterCfg.Game.Tristram.ClearPortal = false
	ls.ctx.CharacterCfg.Game.Tristram.FocusOnElitePacks = true
	ls.ctx.CharacterCfg.Game.Pit.MoveThroughBlackMarsh = true
	ls.ctx.CharacterCfg.Game.Pit.OpenChests = true
	ls.ctx.CharacterCfg.Game.Pit.FocusOnElitePacks = false
	ls.ctx.CharacterCfg.Game.Pit.OnlyClearLevel2 = false
	ls.ctx.CharacterCfg.Game.Andariel.ClearRoom = true
	ls.ctx.CharacterCfg.Game.Andariel.UseAntidotes = true
	ls.ctx.CharacterCfg.Game.Mephisto.KillCouncilMembers = false
	ls.ctx.CharacterCfg.Game.Mephisto.OpenChests = false
	ls.ctx.CharacterCfg.Game.Mephisto.ExitToA4 = true
	ls.ctx.CharacterCfg.Game.Countess.ClearFloors = false
	ls.ctx.CharacterCfg.Inventory.InventoryLock = [][]int{
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	}
	ls.ctx.CharacterCfg.Game.InteractWithShrines = true
	ls.ctx.CharacterCfg.Character.BuffOnNewArea = true
	ls.ctx.CharacterCfg.Character.BuffAfterWP = true
	ls.ctx.CharacterCfg.Character.UseExtraBuffs = true
	ls.ctx.CharacterCfg.Game.MinGoldPickupThreshold = 2000
	ls.ctx.CharacterCfg.Inventory.HealingPotionCount = 4
	ls.ctx.CharacterCfg.Inventory.ManaPotionCount = 8
	ls.ctx.CharacterCfg.Inventory.RejuvPotionCount = 0
	ls.ctx.CharacterCfg.Character.ShouldHireAct2MercFrozenAura = true

	levelingCharacter, isLevelingChar := ls.ctx.Char.(context.LevelingCharacter)
	if isLevelingChar {
		levelingCharacter.InitialCharacterConfigSetup()
	}

	if err := config.SaveSupervisorConfig(ls.ctx.CharacterCfg.ConfigFolderName, ls.ctx.CharacterCfg); err != nil {
		ls.ctx.Logger.Error(fmt.Sprintf("Failed to save character configuration: %s", err.Error()))
	}
}

// adjustDifficultyConfig centralizes difficulty-based configuration changes.
func (ls LevelingSequence) AdjustDifficultyConfig() {
	lvl, _ := ls.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)

	//Let setupLevelOneConfig do the initial setup
	if lvl.Value == 1 {
		ls.setupLevelOneConfig()
		return
	}

	ls.ctx.CharacterCfg.Game.RunewordMaker.EnabledRecipes = ls.GetRunewords()
	ls.ctx.CharacterCfg.Game.MinGoldPickupThreshold = 5000 * lvl.Value

	if !ls.ctx.CharacterCfg.Character.UseMerc && (ls.ctx.Data.Quests[quest.Act1SistersBurialGrounds].Completed() || lvl.Value >= 8) {
		ls.ctx.CharacterCfg.Character.UseMerc = true
	}

	if lvl.Value >= 4 && lvl.Value < 24 {
		ls.ctx.CharacterCfg.Character.ClearPathDist = 15
	}
	if lvl.Value >= 24 {
		switch ls.ctx.CharacterCfg.Game.Difficulty {
		case difficulty.Normal:
			ls.ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "mana"}
			ls.ctx.CharacterCfg.Health.MercHealingPotionAt = 55
			ls.ctx.CharacterCfg.Health.MercRejuvPotionAt = 0
			ls.ctx.CharacterCfg.Health.HealingPotionAt = 85
			ls.ctx.CharacterCfg.Health.ChickenAt = 30
			ls.ctx.CharacterCfg.Health.TownChickenAt = 50
			ls.ctx.CharacterCfg.Character.ClearPathDist = 15

		case difficulty.Nightmare:
			ls.ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "mana"}
			ls.ctx.CharacterCfg.Health.MercHealingPotionAt = 55
			ls.ctx.CharacterCfg.Health.MercRejuvPotionAt = 0
			ls.ctx.CharacterCfg.Health.HealingPotionAt = 85
			ls.ctx.CharacterCfg.Health.ChickenAt = 30
			ls.ctx.CharacterCfg.Health.TownChickenAt = 50
			ls.ctx.CharacterCfg.Character.ClearPathDist = 15

		case difficulty.Hell:
			ls.ctx.CharacterCfg.Inventory.BeltColumns = [4]string{"healing", "healing", "mana", "rejuvenation"}
			ls.ctx.CharacterCfg.Health.MercHealingPotionAt = 80
			ls.ctx.CharacterCfg.Health.MercRejuvPotionAt = 40
			ls.ctx.CharacterCfg.Health.HealingPotionAt = 90
			ls.ctx.CharacterCfg.Health.RejuvPotionAtLife = 70
			ls.ctx.CharacterCfg.Health.ChickenAt = 40
			ls.ctx.CharacterCfg.Health.TownChickenAt = 60
			ls.ctx.CharacterCfg.Character.ClearPathDist = 15
			ls.ctx.CharacterCfg.Inventory.ManaPotionCount = 4
		}
	}

	levelingCharacter, isLevelingChar := ls.ctx.Char.(context.LevelingCharacter)
	if isLevelingChar {
		levelingCharacter.AdjustCharacterConfig()
	}

	if err := config.SaveSupervisorConfig(ls.ctx.CharacterCfg.ConfigFolderName, ls.ctx.CharacterCfg); err != nil {
		ls.ctx.Logger.Error(fmt.Sprintf("Failed to save character configuration: %s", err.Error()))
	}
}

func (ls LevelingSequence) GetRunewords() []string {
	enabledRunewordRecipes := []string{"Ancients' Pledge", "Lore", "Insight", "Smoke", "Treachery", "Call to Arms"}

	if !ls.ctx.CharacterCfg.Game.IsNonLadderChar {
		enabledRunewordRecipes = append(enabledRunewordRecipes, "Bulwark", "Hustle")
		ls.ctx.Logger.Info("Ladder character detected. Adding Bulwark and Hustle runewords.")
	}

	ch, isLevelingChar := ls.ctx.Char.(context.LevelingCharacter)
	if isLevelingChar {
		additionalRunewords := ch.GetAdditionalRunewords()
		enabledRunewordRecipes = append(enabledRunewordRecipes, additionalRunewords...)
	}

	return enabledRunewordRecipes
}
