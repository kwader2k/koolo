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

	// Stash tabs are 1-indexed:
	// Tab 1 = Personal stash
	// Tabs 2-N = Shared stash pages (N = 2 + SharedStashPages - 1)
	// Non-DLC: 3 shared pages (tabs 2-4)
	// DLC: 5 shared pages (tabs 2-6)
	sharedPages := ctx.Data.Inventory.SharedStashPages
	if sharedPages == 0 {
		// Fallback: assume 3 pages if not detected
		sharedPages = 3
	}

	tabsToCheck := make([]int, sharedPages)
	for i := 0; i < sharedPages; i++ {
		tabsToCheck[i] = i + 2 // Tabs start at 2 (first shared page)
	}

	for _, tabIndex := range tabsToCheck {
		SwitchStashTab(tabIndex)
		time.Sleep(time.Millisecond * 500)
		ctx.RefreshGameData()

		sharedItems := ctx.Data.Inventory.ByLocation(item.LocationSharedStash)
		for _, it := range sharedItems {
			totalUsedSpace += it.Desc().InventoryWidth * it.Desc().InventoryHeight
		}
	}

	// Each page has 100 spaces. 80% threshold for muling.
	// Non-DLC: 3 pages × 100 = 300 spaces, 80% = 240
	// DLC: 5 pages × 100 = 500 spaces, 80% = 400
	maxSpace := sharedPages * 100
	threshold := int(float64(maxSpace) * 0.8)
	return totalUsedSpace > threshold
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

	DropAndRecoverCursorItem()
	step.SetSkill(skill.Vigor)
	RecoverCorpse()
	ManageBelt()
	// Just to make sure messages like TZ change or public game spam arent on the way
	ClearMessages()
	RefillBeltFromInventory()

	// barb shield remove under 31
	if firstRun {
		RemoveShield()
	}

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
	VendorRefill(VendorRefillOpts{SellJunk: true, BuyConsumables: true})

	// Gamble
	Gamble()

	// Stash again if needed
	Stash(false)

	if ctx.CharacterCfg.CubeRecipes.PrioritizeRunewords {
		MakeRunewords()
		if !isLevelingChar {
			RerollRunewords()
		}
		CubeRecipes()
	} else {
		CubeRecipes()
		MakeRunewords()
		if !isLevelingChar {
			RerollRunewords()
		}
	}

	// After creating or rerolling runewords, stash newly created bases/runewords
	// so we don't carry them out to the next area unnecessarily.
	Stash(false)

	if ctx.CharacterCfg.Game.Leveling.AutoEquip && isLevelingChar {
		AutoEquip()
	}

	if isLevelingChar {
		OptimizeInventory(item.LocationInventory)
	}

	// Leveling related checks
	if ctx.CharacterCfg.Game.Leveling.EnsurePointsAllocation && isLevelingChar {
		ResetStats()
		EnsureStatPoints()
		EnsureSkillPoints()
	} else if !isLevelingChar && ctx.CharacterCfg.Character.AutoStatSkill.Enabled {
		AutoRespecIfNeeded()
		EnsureStatPoints()
		if !shouldDeferAutoSkillsForStats() {
			EnsureSkillPoints()
			EnsureSkillBindings()
		} else {
			ctx.Logger.Debug("Auto stat targets pending; skipping skill allocation for now.")
		}
	}

	if ctx.CharacterCfg.Game.Leveling.EnsureKeyBinding {
		EnsureSkillBindings()
	}

	HealAtNPC()
	ReviveMerc()
	HireMerc()

	return RepairTownRoutine()
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
	ManageBelt()
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

	VendorRefill(VendorRefillOpts{SellJunk: true, BuyConsumables: true})
	ctx.PauseIfNotPriority() // Check after VendorRefill
	Stash(false)
	ctx.PauseIfNotPriority() // Check after Stash
	Gamble()
	ctx.PauseIfNotPriority() // Check after Gamble
	Stash(false)
	ctx.PauseIfNotPriority() // Check after Stash
	if ctx.CharacterCfg.CubeRecipes.PrioritizeRunewords {
		MakeRunewords()
		// Do not reroll runewords while running the leveling sequences.
		// Leveling characters rely on simpler runeword behavior and base
		// selection, and rerolling could consume resources unexpectedly.
		if !isLevelingChar {
			RerollRunewords()
		}
		CubeRecipes()
		ctx.PauseIfNotPriority() // Check after CubeRecipes
	} else {
		CubeRecipes()
		ctx.PauseIfNotPriority() // Check after CubeRecipes
		MakeRunewords()

		// Do not reroll runewords while running the leveling sequences.
		// Leveling characters rely on simpler runeword behavior and base
		// selection, and rerolling could consume resources unexpectedly.
		if !isLevelingChar {
			RerollRunewords()
		}
	}

	// Ensure any newly created or rerolled runewords/bases are stashed
	// before leaving town.
	Stash(false)
	ctx.PauseIfNotPriority() // Check after post-reroll Stash

	if ctx.CharacterCfg.Game.Leveling.AutoEquip && isLevelingChar {
		AutoEquip()
		ctx.PauseIfNotPriority() // Check after AutoEquip
	}

	if ctx.CharacterCfg.Game.Leveling.EnsurePointsAllocation && isLevelingChar {
		EnsureStatPoints()
		ctx.PauseIfNotPriority() // Check after EnsureStatPoints
		EnsureSkillPoints()
		ctx.PauseIfNotPriority() // Check after EnsureSkillPoints
	} else if !isLevelingChar && ctx.CharacterCfg.Character.AutoStatSkill.Enabled {
		AutoRespecIfNeeded()
		ctx.PauseIfNotPriority() // Check after AutoRespecIfNeeded
		EnsureStatPoints()
		ctx.PauseIfNotPriority() // Check after EnsureStatPoints
		if !shouldDeferAutoSkillsForStats() {
			EnsureSkillPoints()
			ctx.PauseIfNotPriority() // Check after EnsureSkillPoints
			EnsureSkillBindings()
			ctx.PauseIfNotPriority() // Check after EnsureSkillBindings
		} else {
			ctx.Logger.Debug("Auto stat targets pending; skipping skill allocation for now.")
		}
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
	if err := RepairTownRoutine(); err != nil {
		return err
	}
	ctx.PauseIfNotPriority() // Check after RepairTownRoutine

	if ctx.CharacterCfg.Companion.Leader {
		UsePortalInTown()
		utils.Sleep(500)
		return OpenTPIfLeader()
	}

	return UsePortalInTown()
}
