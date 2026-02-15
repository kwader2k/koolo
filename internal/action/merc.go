package action

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/memory"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

// HireAct2Merc hires an Act 2 mercenary, with different logic for legacy and modern graphics.
// In modern graphics, it will hire the first mercenary in the list.
// In legacy graphics, it will hire the one with Holy Freeze aura.
func HireAct2Merc() error {
	ctx := context.Get()
	if (ctx.CharacterCfg.Game.Difficulty == difficulty.Normal || ctx.CharacterCfg.Game.Difficulty == difficulty.Nightmare) && ctx.Data.MercHPPercent() > 0 && ctx.CharacterCfg.Character.ShouldHireAct2MercFrozenAura {
		ctx.Logger.Info("Hiring Act 2 mercenary...")
		DrinkAllPotionsInInventory()

		ctx.Logger.Info("Un-equipping merc")
		if err := UnEquipMercenary(); err != nil {
			ctx.Logger.Error(fmt.Sprintf("Failed to unequip mercenary: %s", err.Error()))
			return err
		}

		ctx.Logger.Info("Interacting with mercenary NPC")
		if err := InteractNPC(town.GetTownByArea(ctx.Data.PlayerUnit.Area).MercContractorNPC()); err != nil {
			return err
		}
		ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
		utils.Sleep(2000)

		if ctx.Data.LegacyGraphics {
			ctx.Logger.Info("Getting merc list (Legacy)")
			mercList := ctx.GameReader.GetMercList()

			var mercToHire *memory.MercOption
			for i := range mercList {
				if mercList[i].Skill.ID == skill.HolyFreeze {
					mercToHire = &mercList[i]
					break
				}
			}

			if mercToHire == nil {
				ctx.Logger.Info("No merc with Frozen Aura found, cannot hire")
				return nil
			}

			ctx.Logger.Info(fmt.Sprintf("Hiring merc: %s with skill %s", mercToHire.Name, mercToHire.Skill.Name))
			keySequence := []byte{win.VK_HOME}
			for i := 0; i < mercToHire.Index; i++ {
				keySequence = append(keySequence, win.VK_DOWN)
			}
			keySequence = append(keySequence, win.VK_RETURN, win.VK_UP, win.VK_RETURN) // Select merc and confirm hire
			ctx.HID.KeySequence(keySequence...)
		} else {
			ctx.Logger.Info("Hiring first available mercenary (Modern Graphics)")
			ctx.HID.KeySequence(win.VK_HOME)
			utils.Sleep(500)
			ctx.HID.KeySequence(win.VK_RETURN)
			utils.Sleep(500)
			ctx.HID.KeySequence(win.VK_RETURN)
		}

		ctx.CharacterCfg.Character.ShouldHireAct2MercFrozenAura = false

		if err := config.SaveSupervisorConfig(ctx.CharacterCfg.ConfigFolderName, ctx.CharacterCfg); err != nil {
			ctx.Logger.Error(fmt.Sprintf("Failed to save character configuration: %s", err.Error()))
		}

		ctx.Logger.Info("Merc hired successfully, re-equipping merc")
		AutoEquip()
	}

	return nil
}
