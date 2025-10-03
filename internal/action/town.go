package action

import (
	"errors"
	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

func StashFull() bool {
	ctx := context.Get()
	totalUsedSpace := 0

	// Stash tabs are 1-indexed, so we check tabs 2, 3, and 4.
	// These correspond to the first three shared stash tabs.
	tabsToCheck := []int{2, 3, 4}

	for _, tabIndex := range tabsToCheck {
		SwitchStashTab(tabIndex)
		time.Sleep(time.Millisecond * 500)
		ctx.RefreshGameData()

		sharedItems := ctx.Data.Inventory.ByLocation(item.LocationSharedStash)
		for _, it := range sharedItems {
			totalUsedSpace += it.Desc().InventoryWidth * it.Desc().InventoryHeight
		}
	}

	// 3 tabs, 100 spaces each = 300 total spaces. 80% of 300 is 240.
	return totalUsedSpace > 240
}

func PreRun(firstRun bool) error {
	ctx := context.Get()

	// Muling logic for the main farmer character
	if ctx.CharacterCfg.Muling.Enabled && ctx.CharacterCfg.Muling.ReturnTo == "" {
		isStashFull := StashFull()

		if isStashFull {
			muleProfiles := ctx.CharacterCfg.Muling.MuleProfiles
			muleIndex := ctx.CharacterCfg.MulingState.CurrentMuleIndex

			if muleIndex >= len(muleProfiles) {
				ctx.Logger.Error("All mules are full! Cannot stash more items. Stopping.")
				ctx.StopSupervisor()
				return errors.New("all mules are full")
			}

			nextMule := muleProfiles[muleIndex]
			ctx.Logger.Info("Stash is full, preparing to switch to mule.", "mule", nextMule, "index", muleIndex)

			// Increment the index for the next time we come back
			ctx.CharacterCfg.MulingState.CurrentMuleIndex++

			// CRITICAL: Save the updated index to the config file BEFORE switching
			if err := config.SaveSupervisorConfig(ctx.Name, ctx.CharacterCfg); err != nil {
				ctx.Logger.Error("Failed to save muling state before switching", "error", err)
				return err // Stop if we can't save state
			}

			// Trigger the character switch
			ctx.CurrentGame.SwitchToCharacter = nextMule
			ctx.RestartWithCharacter = nextMule
			ctx.CleanStopRequested = true
			ctx.StopSupervisor()
			return ErrMulingNeeded // Stop current execution
		} else {
			// If stash is NOT full and the index is not 0, it means muling just finished.
			// Reset the index and save.
			if ctx.CharacterCfg.MulingState.CurrentMuleIndex != 0 {
				ctx.Logger.Info("Muling process complete, resetting mule index.")
				ctx.CharacterCfg.MulingState.CurrentMuleIndex = 0
				if err := config.SaveSupervisorConfig(ctx.Name, ctx.CharacterCfg); err != nil {
					ctx.Logger.Error("Failed to reset muling state", "error", err)
				}
			}
		}
	}

	DropMouseItem()
	step.SetSkill(skill.Vigor)
	RecoverCorpse()
	ConsumeMisplacedPotionsInBelt()
	// Just to make sure messages like TZ change or public game spam arent on the way
	ClearMessages()
	RefillBeltFromInventory()
	_, isLevelingChar := ctx.Char.(context.LevelingCharacter)

	if firstRun && !isLevelingChar {
		Stash(false)
	}

	if !isLevelingChar {
		// Store items that need to be left unidentified
		if HaveItemsToStashUnidentified() {
			Stash(false)
		}
	}

	// Identify - either via Cain or Tome
	IdentifyAll(false)

	if ctx.CharacterCfg.Game.Leveling.AutoEquip && isLevelingChar {
		AutoEquip()
	}

	// Stash before vendor
	Stash(false)

	// Refill pots, sell, buy etc
	VendorRefill(false, true)

	// Gamble
	Gamble()

	// Stash again if needed
	Stash(false)

	CubeRecipes()
	MakeRunewords()

	// Leveling related checks
	if ctx.CharacterCfg.Game.Leveling.EnsurePointsAllocation {
		ResetStats()
		EnsureStatPoints()
		EnsureSkillPoints()
	}

	if ctx.CharacterCfg.Game.Leveling.EnsureKeyBinding {
		EnsureSkillBindings()
	}

	HealAtNPC()
	ReviveMerc()
	HireMerc()

	return Repair()
}

func InRunReturnTownRoutine() error {
	ctx := context.Get()

	ctx.PauseIfNotPriority()

	if err := ReturnTown(); err != nil {
		return fmt.Errorf("failed to return to town: %w", err)
	}

	// Validate we're actually in town before proceeding
	if !ctx.Data.PlayerUnit.Area.IsTown() {
		return fmt.Errorf("failed to verify town location after portal")
	}

	step.SetSkill(skill.Vigor)
	RecoverCorpse()
	ctx.PauseIfNotPriority() // Check after RecoverCorpse
	ConsumeMisplacedPotionsInBelt()
	ctx.PauseIfNotPriority() // Check after ManageBelt
	RefillBeltFromInventory()
	ctx.PauseIfNotPriority() // Check after RefillBeltFromInventory

	// Let's stash items that need to be left unidentified
	if ctx.CharacterCfg.Game.UseCainIdentify && HaveItemsToStashUnidentified() {
		Stash(false)
		ctx.PauseIfNotPriority() // Check after Stash
	}

	IdentifyAll(false)

	_, isLevelingChar := ctx.Char.(context.LevelingCharacter)
	if ctx.CharacterCfg.Game.Leveling.AutoEquip && isLevelingChar {
		AutoEquip()
		ctx.PauseIfNotPriority() // Check after AutoEquip
	}

	VendorRefill(false, true)
	ctx.PauseIfNotPriority() // Check after VendorRefill
	Stash(false)
	ctx.PauseIfNotPriority() // Check after Stash
	Gamble()
	ctx.PauseIfNotPriority() // Check after Gamble
	Stash(false)
	ctx.PauseIfNotPriority() // Check after Stash
	CubeRecipes()
	ctx.PauseIfNotPriority() // Check after CubeRecipes
	MakeRunewords()

	if ctx.CharacterCfg.Game.Leveling.EnsurePointsAllocation {
		EnsureStatPoints()
		ctx.PauseIfNotPriority() // Check after EnsureStatPoints
		EnsureSkillPoints()
		ctx.PauseIfNotPriority() // Check after EnsureSkillPoints
	}

	if ctx.CharacterCfg.Game.Leveling.EnsureKeyBinding {
		EnsureSkillBindings()
		ctx.PauseIfNotPriority() // Check after EnsureSkillBindings
	}

	HealAtNPC()
	ctx.PauseIfNotPriority() // Check after HealAtNPC
	ReviveMerc()
	ctx.PauseIfNotPriority() // Check after ReviveMerc
	HireMerc()
	ctx.PauseIfNotPriority() // Check after HireMerc
	Repair()
	ctx.PauseIfNotPriority() // Check after Repair

	if ctx.CharacterCfg.Companion.Leader {
		UsePortalInTown()
		utils.Sleep(500)
		return OpenTPIfLeader()
	}

	return UsePortalInTown()
}
