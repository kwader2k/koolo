package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/superunique"
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

	if err := b.killBoneash(); err != nil {
		return err
	}

	return nil
}

func (b BoneAsh) killBoneash() error {
	monsterPosition := data.Position{}
	if npcData, found := b.ctx.Data.NPCs.FindOneBySuperUniqueID(superunique.Boneash); found {
		monsterPosition = npcData.Positions[0]
	}

	if monsterPosition == (data.Position{}) {
		b.ctx.Logger.Warn("Bone Ash run: super unique not found, exploring area")
		if err := action.ClearCurrentLevelEx(true, data.MonsterAnyFilter(), func() bool {
			if monster, found := b.ctx.Data.Monsters.FindOne(npc.BurningDeadMage, data.MonsterTypeSuperUnique); found {
				monsterPosition = monster.Position
				b.ctx.Logger.Warn("Bone Ash run: super unique found during exploration")
				return true
			}

			return false
		}); err != nil {
			return err
		}
		if monsterPosition == (data.Position{}) {
			b.ctx.Logger.Warn("Bone Ash run: super unique not found after exploration")
		}
	}

	if monsterPosition != (data.Position{}) {
		if err := action.MoveToCoords(monsterPosition); err != nil {
			return err
		}
	}

	if err := b.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		if m, found := d.Monsters.FindOne(npc.BurningDeadMage, data.MonsterTypeSuperUnique); found {
			return m.UnitID, true
		}

		return 0, false
	}, nil); err != nil {
		return err
	}

	if err := action.ItemPickup(30); err != nil {
		return err
	}

	return nil
}
