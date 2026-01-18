package run

import (
	"errors"
	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/icc"
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
	// Check if group leveling mode is enabled
	if a.isGroupLevelingEnabled() {
		return a.runWithGroupCoordination(parameters)
	}

	// EXISTING: Solo leveling flow (unchanged)
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

// isGroupLevelingEnabled checks if group leveling is configured
func (a Leveling) isGroupLevelingEnabled() bool {
	return a.ctx.CharacterCfg.GroupLeveling.Enabled
}

// runWithGroupCoordination executes leveling with group coordination
func (a Leveling) runWithGroupCoordination(parameters *RunParameters) error {
	// Check if ICCManager is set in context
	if a.ctx.ICCManager == nil {
		a.ctx.Logger.Error("Group leveling enabled but ICCManager not set in context")
		return errors.New("icc_manager_not_initialized")
	}

	// Cast to *context.ICCManager (direct type instead of interface)
	iccMgr, ok := a.ctx.ICCManager.(*context.ICCManager)
	if !ok {
		a.ctx.Logger.Error("ICCManager is not *context.ICCManager type")
		return errors.New("icc_manager_invalid_type")
	}

	// Create coordinator
	coord := icc.NewGroupLevelingCoordinator(a.ctx, iccMgr)

	a.ctx.Logger.Info("Starting group leveling mode",
		"group_name", a.ctx.CharacterCfg.GroupLeveling.GroupName,
		"role", a.ctx.CharacterCfg.GroupLeveling.Role,
	)

	// Adjust settings based on difficulty
	a.AdjustDifficultyConfig()

	// Wait for all group members to join game
	if err := coord.WaitForGameStart(2 * time.Minute); err != nil {
		a.ctx.Logger.Error("Timeout waiting for group members to join game", "error", err)
		return err
	}

	// ExecuteRun callback for followers
	if !coord.IsLeader() {
		if a.ctx.GroupLevelingState == nil {
			a.ctx.GroupLevelingState = &context.GroupLevelingState{}
		}
		a.ctx.GroupLevelingState.ExecuteRun = func(runName string, runData map[string]interface{}) error {
			runInstance := BuildRun(runName)
			if runInstance == nil {
				return fmt.Errorf("unknown run: %s", runName)
			}
			return runInstance.Run(&RunParameters{})
		}
	}

	// Determine my role
	if coord.IsLeader() {
		return a.runAsLeader(coord, parameters)
	}
	return a.runAsFollower(coord, parameters)
}

// runAsLeader executes leveling as group leader
func (a Leveling) runAsLeader(coord *icc.GroupLevelingCoordinator, parameters *RunParameters) error {
	a.ctx.Logger.Info("Running as group LEADER")

	a.GoToCurrentProgressionTown()

	if err := a.AdjustGameDifficulty(); err != nil {
		return err
	}

	// Execute acts with coordination
	if err := a.runAct1AsLeader(coord); err != nil {
		return err
	}
	if err := a.runAct2AsLeader(coord); err != nil {
		return err
	}
	if err := a.runAct3AsLeader(coord); err != nil {
		return err
	}
	if err := a.runAct4AsLeader(coord); err != nil {
		return err
	}
	return a.runAct5AsLeader(coord)
}

// runAsFollower executes leveling as group follower
func (a Leveling) runAsFollower(coord *icc.GroupLevelingCoordinator, parameters *RunParameters) error {
	a.ctx.Logger.Info("Running as group FOLLOWER")

	a.AdjustDifficultyConfig()

	// Follow leader through acts
	if err := a.runAct1AsFollower(coord); err != nil {
		return err
	}
	if err := a.runAct2AsFollower(coord); err != nil {
		return err
	}
	if err := a.runAct3AsFollower(coord); err != nil {
		return err
	}
	if err := a.runAct4AsFollower(coord); err != nil {
		return err
	}
	return a.runAct5AsFollower(coord)
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
	a.ctx.CharacterCfg.CloseMiniPanel = false
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
	a.ctx.CharacterCfg.Game.Andariel.UseAntidoes = true
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

// --- Group Leveling Act Coordination ---

// runAct1AsLeader executes Act1 with quest/boss announcements
func (a Leveling) runAct1AsLeader(coord *icc.GroupLevelingCoordinator) error {
	a.ctx.Logger.Info("Act1 - Running as LEADER")

	// Announce act start
	coord.AnnounceActStart(1)

	// Wait for all members to be ready before starting
	if err := coord.WaitSynchro("act1_start", 2*time.Minute); err != nil {
		return err
	}

	// Run normal Act1 progression
	if err := a.act1(); err != nil {
		return err
	}

	// After completing act, check for quest/boss credit issues
	denCompleted := a.ctx.Data.Quests[quest.Act1DenOfEvil].Completed()
	coord.AnnounceQuestComplete("act1_den_of_evil", denCompleted)

	ravenCompleted := a.ctx.Data.Quests[quest.Act1SistersBurialGrounds].Completed()
	coord.AnnounceQuestComplete("act1_blood_raven", ravenCompleted)

	cainCompleted := a.ctx.Data.Quests[quest.Act1TheSearchForCain].Completed()
	coord.AnnounceQuestComplete("act1_rescue_cain", cainCompleted)

	andyCompleted := a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed()
	coord.AnnounceQuestComplete("act1_andariel", andyCompleted)
	coord.AnnounceBossKill("Andariel", andyCompleted)

	// Check if leader change is needed (someone missed quest/boss)
	newLeader, needsChange, err := coord.CheckQuestCompletion("act1_andariel", 30*time.Second)
	if err != nil {
		return err
	}
	if needsChange {
		a.ctx.Logger.Warn("Quest credit check failed - promoting new leader", "new_leader", newLeader)
		// In next game, new leader will take over
		return fmt.Errorf("quest_credit_failed_leader_change_required")
	}

	return nil
}

// runAct1AsFollower waits for leader's announcements and assists
func (a Leveling) runAct1AsFollower(coord *icc.GroupLevelingCoordinator) error {
	a.ctx.Logger.Info("Act1 - Running as FOLLOWER")

	// Wait for leader and all members to be ready
	if err := coord.WaitSynchro("act1_start", 2*time.Minute); err != nil {
		return err
	}

	// Follow leader through act (simplified - just run act1 progression)
	if err := a.act1(); err != nil {
		return err
	}

	// Announce own quest completion status
	denCompleted := a.ctx.Data.Quests[quest.Act1DenOfEvil].Completed()
	coord.AnnounceQuestComplete("act1_den_of_evil", denCompleted)

	ravenCompleted := a.ctx.Data.Quests[quest.Act1SistersBurialGrounds].Completed()
	coord.AnnounceQuestComplete("act1_blood_raven", ravenCompleted)

	cainCompleted := a.ctx.Data.Quests[quest.Act1TheSearchForCain].Completed()
	coord.AnnounceQuestComplete("act1_rescue_cain", cainCompleted)

	andyCompleted := a.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed()
	coord.AnnounceQuestComplete("act1_andariel", andyCompleted)
	coord.AnnounceBossKill("Andariel", andyCompleted)

	// Wait for all followers to complete
	if err := coord.WaitSynchro("act1_complete", 5*time.Minute); err != nil {
		return err
	}

	return nil
}

// runAct2AsLeader executes Act2 with quest/boss announcements
func (a Leveling) runAct2AsLeader(coord *icc.GroupLevelingCoordinator) error {
	a.ctx.Logger.Info("Act2 - Running as LEADER")

	coord.AnnounceActStart(2)

	// Wait for all members to be ready before starting
	if err := coord.WaitSynchro("act2_start", 2*time.Minute); err != nil {
		return err
	}

	if err := a.act2(); err != nil {
		return err
	}

	// Announce quest completions
	radamentCompleted := a.ctx.Data.Quests[quest.Act2RadamentsLair].Completed()
	coord.AnnounceQuestComplete("act2_radament", radamentCompleted)

	staffCompleted := a.ctx.Data.Quests[quest.Act2TheHoradricStaff].Completed()
	coord.AnnounceQuestComplete("act2_horadric_staff", staffCompleted)

	durielCompleted := a.ctx.Data.Quests[quest.Act2TheSummoner].Completed()
	coord.AnnounceQuestComplete("act2_summoner", durielCompleted)

	act2Completed := a.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed()
	coord.AnnounceQuestComplete("act2_duriel", act2Completed)
	coord.AnnounceBossKill("Duriel", act2Completed)

	newLeader, needsChange, err := coord.CheckQuestCompletion("act2_duriel", 30*time.Second)
	if err != nil {
		return err
	}
	if needsChange {
		a.ctx.Logger.Warn("Quest credit check failed - promoting new leader", "new_leader", newLeader)
		return fmt.Errorf("quest_credit_failed_leader_change_required")
	}

	return nil
}

// runAct2AsFollower waits for leader and assists
func (a Leveling) runAct2AsFollower(coord *icc.GroupLevelingCoordinator) error {
	a.ctx.Logger.Info("Act2 - Running as FOLLOWER")

	// Wait for leader and all members to be ready
	if err := coord.WaitSynchro("act2_start", 2*time.Minute); err != nil {
		return err
	}

	if err := a.act2(); err != nil {
		return err
	}

	radamentCompleted := a.ctx.Data.Quests[quest.Act2RadamentsLair].Completed()
	coord.AnnounceQuestComplete("act2_radament", radamentCompleted)

	staffCompleted := a.ctx.Data.Quests[quest.Act2TheHoradricStaff].Completed()
	coord.AnnounceQuestComplete("act2_horadric_staff", staffCompleted)

	durielCompleted := a.ctx.Data.Quests[quest.Act2TheSummoner].Completed()
	coord.AnnounceQuestComplete("act2_summoner", durielCompleted)

	act2Completed := a.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed()
	coord.AnnounceQuestComplete("act2_duriel", act2Completed)
	coord.AnnounceBossKill("Duriel", act2Completed)

	if err := coord.WaitSynchro("act2_complete", 5*time.Minute); err != nil {
		return err
	}

	return nil
}

// runAct3AsLeader executes Act3 with quest/boss announcements
func (a Leveling) runAct3AsLeader(coord *icc.GroupLevelingCoordinator) error {
	a.ctx.Logger.Info("Act3 - Running as LEADER")

	coord.AnnounceActStart(3)

	// Wait for all members to be ready before starting
	if err := coord.WaitSynchro("act3_start", 2*time.Minute); err != nil {
		return err
	}

	if err := a.act3(); err != nil {
		return err
	}

	bookCompleted := a.ctx.Data.Quests[quest.Act3LamEsensTome].Completed()
	coord.AnnounceQuestComplete("act3_lam_esens_tome", bookCompleted)

	mephCompleted := a.ctx.Data.Quests[quest.Act3TheGuardian].Completed()
	coord.AnnounceQuestComplete("act3_mephisto", mephCompleted)
	coord.AnnounceBossKill("Mephisto", mephCompleted)

	newLeader, needsChange, err := coord.CheckQuestCompletion("act3_mephisto", 30*time.Second)
	if err != nil {
		return err
	}
	if needsChange {
		a.ctx.Logger.Warn("Quest credit check failed - promoting new leader", "new_leader", newLeader)
		return fmt.Errorf("quest_credit_failed_leader_change_required")
	}

	return nil
}

// runAct3AsFollower waits for leader and assists
func (a Leveling) runAct3AsFollower(coord *icc.GroupLevelingCoordinator) error {
	a.ctx.Logger.Info("Act3 - Running as FOLLOWER")

	// Wait for leader and all members to be ready
	if err := coord.WaitSynchro("act3_start", 2*time.Minute); err != nil {
		return err
	}

	if err := a.act3(); err != nil {
		return err
	}

	bookCompleted := a.ctx.Data.Quests[quest.Act3LamEsensTome].Completed()
	coord.AnnounceQuestComplete("act3_lam_esens_tome", bookCompleted)

	mephCompleted := a.ctx.Data.Quests[quest.Act3TheGuardian].Completed()
	coord.AnnounceQuestComplete("act3_mephisto", mephCompleted)
	coord.AnnounceBossKill("Mephisto", mephCompleted)

	if err := coord.WaitSynchro("act3_complete", 5*time.Minute); err != nil {
		return err
	}

	return nil
}

// runAct4AsLeader executes Act4 with quest/boss announcements
func (a Leveling) runAct4AsLeader(coord *icc.GroupLevelingCoordinator) error {
	a.ctx.Logger.Info("Act4 - Running as LEADER")

	coord.AnnounceActStart(4)

	// Wait for all members to be ready before starting
	if err := coord.WaitSynchro("act4_start", 2*time.Minute); err != nil {
		return err
	}

	if err := a.act4(); err != nil {
		return err
	}

	izualCompleted := a.ctx.Data.Quests[quest.Act4TheFallenAngel].Completed()
	coord.AnnounceQuestComplete("act4_izual", izualCompleted)
	coord.AnnounceBossKill("Izual", izualCompleted)

	diabloCompleted := a.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed()
	coord.AnnounceQuestComplete("act4_diablo", diabloCompleted)
	coord.AnnounceBossKill("Diablo", diabloCompleted)

	newLeader, needsChange, err := coord.CheckQuestCompletion("act4_diablo", 30*time.Second)
	if err != nil {
		return err
	}
	if needsChange {
		a.ctx.Logger.Warn("Quest credit check failed - promoting new leader", "new_leader", newLeader)
		return fmt.Errorf("quest_credit_failed_leader_change_required")
	}

	return nil
}

// runAct4AsFollower waits for leader and assists
func (a Leveling) runAct4AsFollower(coord *icc.GroupLevelingCoordinator) error {
	a.ctx.Logger.Info("Act4 - Running as FOLLOWER")

	// Wait for leader and all members to be ready
	if err := coord.WaitSynchro("act4_start", 2*time.Minute); err != nil {
		return err
	}

	if err := a.act4(); err != nil {
		return err
	}

	izualCompleted := a.ctx.Data.Quests[quest.Act4TheFallenAngel].Completed()
	coord.AnnounceQuestComplete("act4_izual", izualCompleted)
	coord.AnnounceBossKill("Izual", izualCompleted)

	diabloCompleted := a.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed()
	coord.AnnounceQuestComplete("act4_diablo", diabloCompleted)
	coord.AnnounceBossKill("Diablo", diabloCompleted)

	if err := coord.WaitSynchro("act4_complete", 5*time.Minute); err != nil {
		return err
	}

	return nil
}

// runAct5AsLeader executes Act5 with quest/boss announcements
func (a Leveling) runAct5AsLeader(coord *icc.GroupLevelingCoordinator) error {
	a.ctx.Logger.Info("Act5 - Running as LEADER")

	coord.AnnounceActStart(5)

	// Wait for all members to be ready before starting
	if err := coord.WaitSynchro("act5_start", 2*time.Minute); err != nil {
		return err
	}

	if err := a.act5(); err != nil {
		return err
	}

	shenkCompleted := a.ctx.Data.Quests[quest.Act5SiegeOnHarrogath].Completed()
	coord.AnnounceQuestComplete("act5_shenk", shenkCompleted)
	coord.AnnounceBossKill("Shenk", shenkCompleted)

	anyaCompleted := a.ctx.Data.Quests[quest.Act5PrisonOfIce].Completed()
	coord.AnnounceQuestComplete("act5_anya", anyaCompleted)

	ancientsCompleted := a.ctx.Data.Quests[quest.Act5RiteOfPassage].Completed()
	coord.AnnounceQuestComplete("act5_ancients", ancientsCompleted)
	coord.AnnounceBossKill("Ancients", ancientsCompleted)

	baalCompleted := a.ctx.Data.Quests[quest.Act5EveOfDestruction].Completed()
	coord.AnnounceQuestComplete("act5_baal", baalCompleted)
	coord.AnnounceBossKill("Baal", baalCompleted)

	newLeader, needsChange, err := coord.CheckQuestCompletion("act5_baal", 30*time.Second)
	if err != nil {
		return err
	}
	if needsChange {
		a.ctx.Logger.Warn("Quest credit check failed - promoting new leader", "new_leader", newLeader)
		return fmt.Errorf("quest_credit_failed_leader_change_required")
	}

	return nil
}

// runAct5AsFollower waits for leader and assists
func (a Leveling) runAct5AsFollower(coord *icc.GroupLevelingCoordinator) error {
	a.ctx.Logger.Info("Act5 - Running as FOLLOWER")

	// Wait for leader and all members to be ready
	if err := coord.WaitSynchro("act5_start", 2*time.Minute); err != nil {
		return err
	}

	if err := a.act5(); err != nil {
		return err
	}

	shenkCompleted := a.ctx.Data.Quests[quest.Act5SiegeOnHarrogath].Completed()
	coord.AnnounceQuestComplete("act5_shenk", shenkCompleted)
	coord.AnnounceBossKill("Shenk", shenkCompleted)

	anyaCompleted := a.ctx.Data.Quests[quest.Act5PrisonOfIce].Completed()
	coord.AnnounceQuestComplete("act5_anya", anyaCompleted)

	ancientsCompleted := a.ctx.Data.Quests[quest.Act5RiteOfPassage].Completed()
	coord.AnnounceQuestComplete("act5_ancients", ancientsCompleted)
	coord.AnnounceBossKill("Ancients", ancientsCompleted)

	baalCompleted := a.ctx.Data.Quests[quest.Act5EveOfDestruction].Completed()
	coord.AnnounceQuestComplete("act5_baal", baalCompleted)
	coord.AnnounceBossKill("Baal", baalCompleted)

	if err := coord.WaitSynchro("act5_complete", 5*time.Minute); err != nil {
		return err
	}

	return nil
}
