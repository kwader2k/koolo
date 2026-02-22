package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type FireEye struct {
	ctx *context.Status
}

func NewFireEye() *FireEye {
	return &FireEye{
		ctx: context.Get(),
	}
}

func (f *FireEye) Name() string {
	return "FireEye"
}

func (a *FireEye) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) {
		return SequencerError
	}
	if !a.ctx.Data.Quests[quest.Act2TaintedSun].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (f *FireEye) Run(parameters *RunParameters) error {

	fmt.Println("Fire Eye: Traveling to Arcane Sanctuary...")
	err := action.WayPoint(area.ArcaneSanctuary)
	if err != nil {
		return fmt.Errorf("could not travel to Arcane Sanctuary: %w", err)
	}

	obj, _ := f.ctx.Data.Objects.FindOne(object.ArcaneSanctuaryPortal)

	err = action.InteractObject(obj, func() bool {
		updatedObj, found := f.ctx.Data.Objects.FindOne(object.ArcaneSanctuaryPortal)
		if found {
			if !updatedObj.Selectable {
				f.ctx.Logger.Debug("Interacted with ArcaneSanctuaryPortal")
			}
			return !updatedObj.Selectable
		}
		return false
	})

	if err != nil {
		return err
	}

	err = action.InteractObject(obj, func() bool {
		return f.ctx.Data.PlayerUnit.Area == area.PalaceCellarLevel3
	})

	if err != nil {
		return err
	}

	utils.Sleep(300)

	areaData := f.ctx.Data.Areas[area.PalaceCellarLevel3]
	monster, found := areaData.NPCs.FindOne(750)

	if !found || len(monster.Positions) == 0 {
		f.ctx.Logger.Error("fireEye not found]")
		return err
	}

	action.MoveTo(func() (data.Position, bool) {
		return monster.Positions[0], true
	})

	fireEyeWasFound := false
	isKillable := true

	f.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		fireEye, found := d.Monsters.FindOne(750, data.MonsterTypeSuperUnique)

		if !found {
			fireEye, found = d.Monsters.FindOne(npc.Invader, data.MonsterTypeSuperUnique)
			if !found {
				fireEye, found = d.Monsters.FindOne(npc.Invader2, data.MonsterTypeSuperUnique)
				if !found {
					return 0, false
				}
			}
		}

		if found {
			fireEyeWasFound = true
		}

		if found && fireEye.IsImmune(stat.FireImmune) && fireEye.IsImmune(stat.ColdImmune) && fireEye.Stats[stat.Life] > 0 {
			if !isKillable {
				isKillable = false
			}

			return 0, false
		}

		return fireEye.UnitID, true
	}, nil)

	if !fireEyeWasFound {
		f.ctx.Logger.Error("Fire Eye not found, skipping")
		return nil
	} else if !isKillable {
		f.ctx.Logger.Error("Fire Eye is immune to fire and cold, skipping")
		return nil
	}

	action.ClearAreaAroundPlayer(30, data.MonsterEliteFilter())
	action.ItemPickup(30)

	return nil
}
