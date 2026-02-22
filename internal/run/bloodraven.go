package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Bloodraven struct {
	ctx *context.Status
}

func NewBloodraven() *Bloodraven {
	return &Bloodraven{
		ctx: context.Get(),
	}
}

func (b Bloodraven) Name() string {
	return string(config.BloodravenRun)
}

func (b Bloodraven) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) && b.ctx.Data.Quests[quest.Act1SistersBurialGrounds].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (b Bloodraven) Run(parameters *RunParameters) error {
	ctx := b.ctx
	ctx.SetLastAction("bloodraven")

	if err := action.WayPoint(area.ColdPlains); err != nil {
		return fmt.Errorf("failed to move to Cold Plains: %w", err)
	}

	if err := action.MoveToArea(area.BurialGrounds); err != nil {
		return fmt.Errorf("failed to move to Burial Grounds: %w", err)
	}

	originalBackToTownCfg := b.ctx.CharacterCfg.BackToTown
	b.ctx.CharacterCfg.BackToTown.NoMpPotions = false
	b.ctx.CharacterCfg.Health.HealingPotionAt = 55

	defer func() {
		b.ctx.CharacterCfg.BackToTown = originalBackToTownCfg
		b.ctx.Logger.Info("Restored original back-to-town checks after Blood Raven fight.")
	}()

	areaData := b.ctx.Data.Areas[area.BurialGrounds]
	bloodRavenNPC, found := areaData.NPCs.FindOne(805)

	if !found || len(bloodRavenNPC.Positions) == 0 {
		b.ctx.Logger.Info("Blood Raven position not found")
		return nil
	}

	action.MoveToCoords(bloodRavenNPC.Positions[0])

	for {
		bloodRaven, found := b.ctx.Data.Monsters.FindOne(npc.BloodRaven, data.MonsterTypeNone)

		if !found {
			break
		}

		b.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			return bloodRaven.UnitID, true
		}, nil)
	}

	action.ItemPickup(30)

	if IsQuestRun(parameters) {
		if err := action.ReturnTown(); err != nil {
			err = action.MoveToArea(area.ColdPlains)
			if err != nil {
				return err
			}
			err = action.FieldWayPoint(area.RogueEncampment)
			if err != nil {
				return err
			}
		}
		utils.Sleep(500)
		action.InteractNPC(npc.Kashya)
	}

	return nil
}
