package run

import (
	"sort"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

type RescueBarbs struct {
	ctx *context.Status
}

func NewRescueBarbs() *RescueBarbs {
	return &RescueBarbs{
		ctx: context.Get(),
	}
}

func (rb RescueBarbs) Name() string {
	return string(config.RescueBarbsRun)
}

func (rb RescueBarbs) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}

	if !rb.ctx.Data.Quests[quest.Act4TerrorsEnd].Completed() {
		return SequencerStop
	}

	if rb.ctx.Data.Quests[quest.Act5RescueOnMountArreat].Completed() && !rb.ctx.Data.Quests[quest.Act5RescueOnMountArreat].HasStatus(quest.StatusRewardPending) {
		return SequencerSkip
	}

	return SequencerOk
}

func (rb RescueBarbs) Run(parameters *RunParameters) error {
	if rb.ctx.Data.Quests[quest.Act5RescueOnMountArreat].HasStatus(quest.StatusRewardPending) {
		return action.InteractNPC(npc.QualKehk)
	}

	err := action.WayPoint(area.FrigidHighlands)
	if err != nil {
		return err
	}

	var barbSpots []data.Object

	for _, obj := range rb.ctx.Data.Objects {
		if obj.Name == object.CagedWussie {
			barbSpots = append(barbSpots, obj)
		}
	}

	sort.Slice(barbSpots, func(i, j int) bool {
		distanceI := rb.ctx.PathFinder.DistanceFromMe(barbSpots[i].Position)
		distanceJ := rb.ctx.PathFinder.DistanceFromMe(barbSpots[j].Position)

		return distanceI < distanceJ
	})

	for _, cage := range barbSpots {
		action.MoveToCoords(cage.Position, step.WithDistanceToFinish(10))
		if door, found := rb.ctx.Data.Monsters.FindOne(npc.PrisonDoor, data.MonsterTypeNone); found {
			rb.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
				return door.UnitID, true
			}, nil)
		}
	}

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	err = action.InteractNPC(npc.QualKehk)
	if err != nil {
		return err
	}
	return nil
}
