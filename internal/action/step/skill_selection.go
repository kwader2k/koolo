package step

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// SelectRightSkill selects a skill for the right mouse button
// Uses packets if enabled in config (no keybinding required), otherwise falls back to HID
func SelectRightSkill(skillID skill.ID) error {
	ctx := context.Get()

	// Check if skill is already selected
	if ctx.Data.PlayerUnit.RightSkill == skillID {
		return nil
	}

	// If packets are enabled, use them directly (no keybinding needed)
	if ctx.CharacterCfg.PacketCasting.UseForSkillSelection && ctx.PacketSender != nil {
		if err := ctx.PacketSender.SelectRightSkill(skillID); err != nil {
			// Try HID fallback only if keybinding exists
			return selectSkillViaHIDIfAvailable(skillID)
		}
		utils.Sleep(50)
		return nil
	}

	// When not using packets, keybinding is required
	return selectSkillViaHIDIfAvailable(skillID)
}

// SelectLeftSkill selects a skill for the left mouse button
// Uses packets if enabled in config (no keybinding required), otherwise falls back to HID
func SelectLeftSkill(skillID skill.ID) error {
	ctx := context.Get()

	// Check if skill is already selected
	if ctx.Data.PlayerUnit.LeftSkill == skillID {
		return nil
	}

	// If packets are enabled, use them directly (no keybinding needed)
	if ctx.CharacterCfg.PacketCasting.UseForSkillSelection && ctx.PacketSender != nil {
		if err := ctx.PacketSender.SelectLeftSkill(skillID); err != nil {
			// Try HID fallback only if keybinding exists
			return selectSkillViaHIDIfAvailable(skillID)
		}
		utils.Sleep(50)
		return nil
	}

	// When not using packets, keybinding is required
	return selectSkillViaHIDIfAvailable(skillID)
}

// SelectSkill selects a skill and returns the mouse button it was assigned to.
// Prefer left when both buttons are available, but return whichever is already selected.
func SelectSkill(skillID skill.ID) (game.MouseButton, bool) {
	ctx := context.Get()

	if ctx.Data.PlayerUnit.LeftSkill == skillID {
		return game.LeftButton, true
	}
	if ctx.Data.PlayerUnit.RightSkill == skillID {
		return game.RightButton, true
	}

	skillDesc, found := skill.Skills[skillID]
	if !found {
		return game.LeftButton, false
	}

	selectAndCheck := func(useLeft bool) (game.MouseButton, bool) {
		if useLeft {
			_ = SelectLeftSkill(skillID)
		} else {
			_ = SelectRightSkill(skillID)
		}
		ctx.RefreshGameData()
		if ctx.Data.PlayerUnit.LeftSkill == skillID {
			return game.LeftButton, true
		}
		if ctx.Data.PlayerUnit.RightSkill == skillID {
			return game.RightButton, true
		}
		return game.LeftButton, false
	}

	if skillDesc.LeftSkill {
		if button, ok := selectAndCheck(true); ok {
			return button, true
		}
	}
	if skillDesc.RightSkill {
		if button, ok := selectAndCheck(false); ok {
			return button, true
		}
	}

	return game.LeftButton, false
}

// selectSkillViaHIDIfAvailable attempts to select skill via HID if keybinding exists
func selectSkillViaHIDIfAvailable(skillID skill.ID) error {
	ctx := context.Get()

	kb, found := ctx.Data.KeyBindings.KeyBindingForSkill(skillID)
	if !found {
		return nil
	}

	ctx.HID.PressKeyBinding(kb)
	utils.Sleep(50)
	return nil
}

// SelectRightSkillByKeyBinding selects a skill using its keybinding directly
// Uses packets if enabled in config, otherwise falls back to HID
func SelectRightSkillByKeyBinding(kb data.KeyBinding) error {
	ctx := context.Get()

	// Try to find the skill ID from the keybinding
	for skillID, binding := range ctx.Data.KeyBindings.Skills {
		if binding.Key1[0] == kb.Key1[0] {
			return SelectRightSkill(skill.ID(skillID))
		}
	}

	// If we can't find the skill ID, just use HID
	ctx.HID.PressKeyBinding(kb)
	utils.Sleep(50)
	return nil
}

// SelectLeftSkillByKeyBinding selects a skill using its keybinding directly
// Uses packets if enabled in config, otherwise falls back to HID
func SelectLeftSkillByKeyBinding(kb data.KeyBinding) error {
	ctx := context.Get()

	// Try to find the skill ID from the keybinding
	for skillID, binding := range ctx.Data.KeyBindings.Skills {
		if binding.Key1[0] == kb.Key1[0] {
			return SelectLeftSkill(skill.ID(skillID))
		}
	}

	// If we can't find the skill ID, just use HID
	ctx.HID.PressKeyBinding(kb)
	utils.Sleep(50)
	return nil
}
