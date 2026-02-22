package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/memory"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

type FrozenAuraMerc struct {
	ctx *context.Status
}

func NewFrozenAuraMerc() *FrozenAuraMerc {
	return &FrozenAuraMerc{
		ctx: context.Get(),
	}
}

func (fam FrozenAuraMerc) Name() string {
	return string(config.FrozenAuraMercRun)
}

func (fam FrozenAuraMerc) CheckConditions(parameters *RunParameters) SequencerResult {
	if !IsQuestRun(parameters) || fam.ctx.CharacterCfg.Game.Difficulty != difficulty.Nightmare {
		return SequencerError
	}

	if !fam.ctx.Data.Quests[quest.Act1SistersToTheSlaughter].Completed() {
		return SequencerStop
	}

	if fam.ctx.Data.MercHPPercent() < 0 || !fam.ctx.CharacterCfg.Character.ShouldHireAct2MercFrozenAura {
		return SequencerSkip
	}

	return SequencerOk
}

func (fam FrozenAuraMerc) Run(parameters *RunParameters) error {
	if fam.ctx.Data.PlayerUnit.Area != area.LutGholein {
		action.WayPoint(area.LutGholein)
	}

	fam.ctx.Logger.Info("Start Hiring merc with Frozen Aura")
	action.DrinkAllPotionsInInventory()

	fam.ctx.Logger.Info("Un-equipping merc")
	if err := action.UnEquipMercenary(); err != nil {
		fam.ctx.Logger.Error(fmt.Sprintf("Failed to unequip mercenary: %s", err.Error()))
		return err
	}

	fam.ctx.Logger.Info("Interacting with mercenary NPC")
	if err := action.InteractNPC(town.GetTownByArea(fam.ctx.Data.PlayerUnit.Area).MercContractorNPC()); err != nil {
		return err
	}
	fam.ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_RETURN)
	utils.Sleep(2000)

	fam.ctx.Logger.Info("Getting merc list")
	mercList := fam.ctx.GameReader.GetMercList()

	// get the first with fronzen aura
	var mercToHire *memory.MercOption
	for i := range mercList {
		if mercList[i].Skill.ID == skill.HolyFreeze {
			mercToHire = &mercList[i]
			break
		}
	}

	if mercToHire == nil {
		fam.ctx.Logger.Info("No merc with Frozen Aura found, cannot hire")
		return nil
	}

	fam.ctx.Logger.Info(fmt.Sprintf("Hiring merc: %s with skill %s", mercToHire.Name, mercToHire.Skill.Name))
	keySequence := []byte{win.VK_HOME}
	for i := 0; i < mercToHire.Index; i++ {
		keySequence = append(keySequence, win.VK_DOWN)
	}
	keySequence = append(keySequence, win.VK_RETURN, win.VK_UP, win.VK_RETURN) // Select merc and confirm hire
	fam.ctx.HID.KeySequence(keySequence...)

	fam.ctx.CharacterCfg.Character.ShouldHireAct2MercFrozenAura = false

	if err := config.SaveSupervisorConfig(fam.ctx.CharacterCfg.ConfigFolderName, fam.ctx.CharacterCfg); err != nil {
		fam.ctx.Logger.Error(fmt.Sprintf("Failed to save character configuration: %s", err.Error()))
	}

	fam.ctx.Logger.Info("Merc hired successfully, re-equipping merc")
	action.AutoEquip()
	return nil
}
