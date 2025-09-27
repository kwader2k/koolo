package action

import (
	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/action/step"
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

	if ctx.CharacterCfg.Muling.Enabled && ctx.CharacterCfg.Muling.ReturnTo == "" {
		if StashFull() {
			return ErrMulingNeeded
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

	UpdateQuestLog()

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
