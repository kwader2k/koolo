package action

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
)

func PreRun(firstRun bool) error {
	ctx := context.Get()

	DropMouseItem()
	step.SetSkill(skill.Vigor)
	RecoverCorpse()
	ConsumeMisplacedPotionsInBelt()
	// Just to make sure messages like TZ change or public game spam arent on the way
	ClearMessages()
	RefillBeltFromInventory()

	if firstRun {
		Stash(false)
	}

	UpdateQuestLog()

	// Store items that need to be left unidentified
	if HaveItemsToStashUnidentified() {
		Stash(false)
	}

	// Identify - either via Cain or Tome
	IdentifyAll(false)

	_, isLevelingChar := ctx.Char.(context.LevelingCharacter)
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
