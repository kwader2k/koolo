package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

type Ancients struct {
	ctx *context.Status
}

func NewAncients() *Ancients {
	return &Ancients{
		ctx: context.Get(),
	}
}

func (a Ancients) Name() string {
	return string(config.AncientsRun)
}

func (a Ancients) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}
	if !a.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() {
		return SequencerStop
	}
	if a.ctx.Data.Quests[quest.Act5RiteOfPassage].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (a Ancients) Run(parameters *RunParameters) error {
	err := action.WayPoint(area.Harrogath)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.WayPoint(area.TheAncientsWay)
	if err != nil {
		return err
	}
	action.Buff()

	err = action.MoveToArea(area.ArreatSummit)
	if err != nil {
		return err
	}
	action.Buff()

	action.InRunReturnTownRoutine()
	action.Buff()

	a.killAncients()

	action.InRunReturnTownRoutine()

	err = action.MoveToArea(area.TheWorldStoneKeepLevel1)
	if err != nil {
		return err
	}

	err = action.MoveToArea(area.TheWorldStoneKeepLevel2)
	if err != nil {
		return err
	}

	err = action.DiscoverWaypoint()
	if err != nil {
		return err
	}

	// The defer statement above will handle the restoration
	// a.ctx.CharacterCfg.BackToTown = originalBackToTownCfg // This line is now removed
	// a.ctx.Logger.Info("Restored original back-to-town checks after Ancients fight.") // This line is now part of the defer
	return nil
}

func (a Ancients) killAncients() error {
	// Store the original configuration
	originalBackToTownCfg := a.ctx.CharacterCfg.BackToTown
	originalTownChicken := a.ctx.CharacterCfg.Health.TownChickenAt

	// Defer the restoration of the configuration.
	// This will run when the function exits, regardless of how.
	defer func() {
		a.ctx.CharacterCfg.BackToTown = originalBackToTownCfg
		a.ctx.CharacterCfg.Health.TownChickenAt = originalTownChicken
		a.ctx.Logger.Info("Restored original back-to-town checks after Ancients fight.")
	}()

	// Find and interact with the altar object
	altar, found := a.ctx.Data.Objects.FindOne(object.AncientsAltar)
	if !found {
		return fmt.Errorf("AncientsAltar not found")
	}

	err := action.InteractObject(altar, func() bool {
		// After clicking, press Enter to confirm the dialog
		a.ctx.HID.PressKey(win.VK_RETURN)
		utils.Sleep(2000)

		// Check if Ancients spawned (elite monsters appeared)
		ancients := a.ctx.Data.Monsters.Enemies(data.MonsterEliteFilter())
		return len(ancients) > 0
	})
	if err != nil {
		return err
	}

	// Modify the configuration for the Ancients fight
	a.ctx.CharacterCfg.BackToTown.NoHpPotions = false
	a.ctx.CharacterCfg.BackToTown.NoMpPotions = false
	a.ctx.CharacterCfg.BackToTown.EquipmentBroken = false
	a.ctx.CharacterCfg.BackToTown.MercDied = false
	a.ctx.CharacterCfg.Health.TownChickenAt = 0

	for {
		ancients := a.ctx.Data.Monsters.Enemies(data.MonsterEliteFilter())
		if len(ancients) == 0 {
			break
		}

		a.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			for _, m := range d.Monsters.Enemies(data.MonsterEliteFilter()) {
				return m.UnitID, true
			}
			return 0, false
		}, nil)
	}

	return nil
}
