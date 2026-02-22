package action

import (
	"log/slog"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// BuffIfRequired keeps the original behavior:
// - checks if rebuff is needed,
// - skips town,
// - avoids buffing when monsters are too close.
func BuffIfRequired() {
	ctx := context.Get()

	if !IsRebuffRequired() || ctx.Data.PlayerUnit.Area.IsTown() {
		return
	}

	// Don't buff if we have 2 or more monsters close to the character.
	// Don't merge with the previous if, because we want to avoid this
	// expensive check if we don't need to buff.
	closeMonsters := 0
	for _, m := range ctx.Data.Monsters {
		if ctx.PathFinder.DistanceFromMe(m.Position) < 15 {
			closeMonsters++
		}
		// cheaper to check here and end function if say first 2 already < 15
		// so no need to compute the rest
		if closeMonsters >= 2 {
			return
		}
	}

	Buff()
}

// Buff keeps original timing / behavior:
// - no buff in town
// - no buff if done in last 30s
// - pre-CTA buffs
// - CTA (BO/BC) buffs
// - post-CTA class buffs
//
// The only extension is: if config.Character.UseSwapForBuffs is true,
// class buffs are cast from the weapon swap (offhand) instead of main hand.
func Buff() {
	ctx := context.Get()
	ctx.SetLastAction("Buff")

	if ctx.Data.PlayerUnit.Area.IsTown() || time.Since(ctx.LastBuffAt) < time.Second*30 {
		return
	}

	// Check if we're in loading screen
	if ctx.Data.OpenMenus.LoadingScreen {
		ctx.Logger.Debug("Loading screen detected. Waiting for game to load before buffing...")
		ctx.WaitForGameToLoad()
		utils.PingSleep(utils.Light, 400)
	}

	// --- Pre-CTA buffs (unchanged) ---
	preKeys := make([]data.KeyBinding, 0)
	for _, buff := range ctx.Char.PreCTABuffSkills() {
		kb, found := ctx.Data.KeyBindings.KeyBindingForSkill(buff)
		if !found {
			ctx.Logger.Info("Key binding not found, skipping buff", slog.String("skill", buff.Desc().Name))
		} else {
			preKeys = append(preKeys, kb)
		}
	}

	if len(preKeys) > 0 {
		ctx.Logger.Debug("PRE CTA Buffing...")
		for _, kb := range preKeys {
			utils.Sleep(100)
			ctx.HID.PressKeyBinding(kb)
			utils.Sleep(180)
			ctx.HID.Click(game.RightButton, 640, 340)
			utils.Sleep(100)
		}
	}

	// --- CTA buffs (unchanged) ---
	buffCTA()

	// --- Post-CTA class buffs (with optional weapon swap) ---

	// Collect post-CTA buff keybindings as before.
	postKeys := make([]data.KeyBinding, 0)
	for _, buff := range ctx.Char.BuffSkills() {
		kb, found := ctx.Data.KeyBindings.KeyBindingForSkill(buff)
		if !found {
			ctx.Logger.Info("Key binding not found, skipping buff", slog.String("skill", buff.Desc().Name))
		} else {
			postKeys = append(postKeys, kb)
		}
	}

	if len(postKeys) > 0 {
		// Read our new toggle from config:
		useSwapForBuffs := ctx.CharacterCfg != nil && ctx.CharacterCfg.Character.UseSwapForBuffs

		// Optionally swap to offhand (CTA / buff weapon) before class buffs.
		if useSwapForBuffs {
			ctx.Logger.Debug("Using weapon swap for class buff skills")
			step.SwapToCTA()
			utils.PingSleep(utils.Light, 400)
		}

		ctx.Logger.Debug("Post CTA Buffing...")
		for _, kb := range postKeys {
			utils.Sleep(100)
			ctx.HID.PressKeyBinding(kb)
			utils.Sleep(180)
			ctx.HID.Click(game.RightButton, 640, 340)
			utils.Sleep(100)
		}

		// If we swapped, make sure we go back to main weapon.
		if useSwapForBuffs {
			utils.PingSleep(utils.Light, 400)
			step.SwapToMainWeapon()
		}
	}

	utils.PingSleep(utils.Light, 200)
	buffsSuccessful := true
	if ctaFound(*ctx.Data) {
		if !ctx.Data.PlayerUnit.States.HasState(state.Battleorders) ||
			!ctx.Data.PlayerUnit.States.HasState(state.Battlecommand) {
			buffsSuccessful = false
			ctx.Logger.Warn("CTA buffs not detected after buffing, not updating LastBuffAt")
		}
	}

	if buffsSuccessful {
		ctx.LastBuffAt = time.Now()
	}
}

// IsRebuffRequired is left as original: 30s cooldown, CTA priority, and
// simple state-based checks for known buff skills.
func IsRebuffRequired() bool {
	ctx := context.Get()
	ctx.SetLastAction("IsRebuffRequired")

	// Don't buff if we are in town, or we did it recently
	// (prevents double buffing because of network lag).
	if ctx.Data.PlayerUnit.Area.IsTown() || time.Since(ctx.LastBuffAt) < time.Second*30 {
		return false
	}

	if ctaFound(*ctx.Data) &&
		(!ctx.Data.PlayerUnit.States.HasState(state.Battleorders) ||
			!ctx.Data.PlayerUnit.States.HasState(state.Battlecommand)) {
		return true
	}

	// TODO: Find a better way to convert skill to state
	buffs := ctx.Char.BuffSkills()
	for _, buff := range buffs {
		if _, found := ctx.Data.KeyBindings.KeyBindingForSkill(buff); found {
			if buff == skill.HolyShield && !ctx.Data.PlayerUnit.States.HasState(state.Holyshield) {
				return true
			}
			if buff == skill.FrozenArmor &&
				(!ctx.Data.PlayerUnit.States.HasState(state.Frozenarmor) &&
					!ctx.Data.PlayerUnit.States.HasState(state.Shiverarmor) &&
					!ctx.Data.PlayerUnit.States.HasState(state.Chillingarmor)) {
				return true
			}
			if buff == skill.EnergyShield && !ctx.Data.PlayerUnit.States.HasState(state.Energyshield) {
				return true
			}
			if buff == skill.CycloneArmor && !ctx.Data.PlayerUnit.States.HasState(state.Cyclonearmor) {
				return true
			}
		}
	}

	return false
}

// buffCTA handles the CTA weapon set: swap, cast BC/BO, swap back.
// This is kept exactly as in the original implementation.
func buffCTA() {
	ctx := context.Get()
	ctx.SetLastAction("buffCTA")

	if ctaFound(*ctx.Data) {
		ctx.Logger.Debug("CTA found: swapping weapon and casting Battle Command / Battle Orders")

		// Swap weapon only in case we don't have the CTA already equipped
		// (for example chicken previous game during buff stage).
		if _, found := ctx.Data.PlayerUnit.Skills[skill.BattleCommand]; !found {
			step.SwapToCTA()
			utils.PingSleep(utils.Light, 150)
		}

		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.MustKBForSkill(skill.BattleCommand))
		utils.Sleep(180)
		ctx.HID.Click(game.RightButton, 300, 300)
		utils.Sleep(100)

		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.MustKBForSkill(skill.BattleOrders))
		utils.Sleep(180)
		ctx.HID.Click(game.RightButton, 300, 300)
		utils.Sleep(100)

		utils.PingSleep(utils.Light, 400)
		step.SwapToMainWeapon()
	}
}

// ctaFound checks if the player has a CTA-like item equipped (providing both BO and BC as NonClassSkill).
func ctaFound(d game.Data) bool {
	for _, itm := range d.Inventory.ByLocation(item.LocationEquipped) {
		_, boFound := itm.FindStat(stat.NonClassSkill, int(skill.BattleOrders))
		_, bcFound := itm.FindStat(stat.NonClassSkill, int(skill.BattleCommand))

		if boFound && bcFound {
			return true
		}
	}

	return false
}
