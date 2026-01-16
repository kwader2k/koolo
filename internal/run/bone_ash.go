package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

type BoneAsh struct {
	ctx *context.Status
}

func NewBoneAsh() *BoneAsh {
	return &BoneAsh{
		ctx: context.Get(),
	}
}

func (b BoneAsh) Name() string {
	return string(config.BoneAshRun)
}

func (b BoneAsh) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) {
		return SequencerError
	}
	return SequencerOk
}

func (b BoneAsh) Run(parameters *RunParameters) error {
	if err := action.WayPoint(area.InnerCloister); err != nil {
		return err
	}

	if err := action.MoveToArea(area.Cathedral); err != nil {
		return err
	}

	if err := action.MoveToCoords(data.Position{X: 20047, Y: 4898}); err != nil {
		b.ctx.Logger.Warn("Bone Ash run: failed moving to Bone Ash", "error", err)
	}

	if err := b.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		if m, found := d.Monsters.FindOne(npc.BurningDeadMage, data.MonsterTypeSuperUnique); found {
			return m.UnitID, true
		}
		return 0, false
	}, nil); err != nil {
		return err
	}

	action.ItemPickup(30)

	return nil
}
