package run

import (
	"errors"
	"log/slog"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Nihlathak struct {
	ctx                *context.Status
	clearMonsterFilter data.MonsterFilter // nil = normal MF/quest run, non-nil = TZ full clear
}

func NewNihlathak() *Nihlathak {
	return &Nihlathak{
		ctx: context.Get(),
	}
}

func NewNihlathakTZ(filter data.MonsterFilter) *Nihlathak {
	return &Nihlathak{
		ctx:                context.Get(),
		clearMonsterFilter: filter,
	}
}

func (n Nihlathak) Name() string {
	return string(config.NihlathakRun)
}

func (n Nihlathak) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		if !n.ctx.Data.Quests[quest.Act5BetrayalOfHarrogath].Completed() {
			return SequencerSkip
		}
		return SequencerOk
	}
	if n.ctx.Data.Quests[quest.Act5BetrayalOfHarrogath].Completed() {
		return SequencerSkip
	}
	if !n.ctx.Data.Quests[quest.Act5PrisonOfIce].Completed() {
		return SequencerStop
	}
	return SequencerOk
}

func (n Nihlathak) Run(parameters *RunParameters) error {
	if n.clearMonsterFilter != nil {
		return n.runTerrorZone(parameters)
	}
	return n.runStandard(parameters)
}

func (n Nihlathak) runStandard(parameters *RunParameters) error {
	// Use the waypoint to HallsOfPain
	err := action.WayPoint(area.HallsOfPain)
	if err != nil {
		//WP not found, try get it
		err = n.getHallOfPainWp()
		if err != nil {
			return err
		}
	}

	// Move to Halls Of Vaught
	if err = action.MoveToArea(area.HallsOfVaught); err != nil {
		return err
	}

	var nihlaObject data.Object

	o, found := n.ctx.Data.Objects.FindOne(object.NihlathakWildernessStartPositionName)
	if !found {
		return errors.New("failed to find Nihlathak's Start Position")
	}

	// Move to Nihlathak
	action.MoveToCoords(o.Position)

	// Try to position in the safest corner
	action.MoveToCoords(n.findBestCorner(o.Position))

	// Disable item pickup before the fight
	n.ctx.DisableItemPickup()

	// Kill Nihlathak
	if err = n.ctx.Char.KillNihlathak(); err != nil {
		// Re-enable item pickup even if kill fails
		n.ctx.EnableItemPickup()
		return err
	}

	// Re-enable item pickup after kill
	n.ctx.EnableItemPickup()

	// Clear monsters around the area, sometimes it makes difficult to pickup items if there are many monsters around the area
	if n.ctx.CharacterCfg.Game.Nihlathak.ClearArea {
		n.ctx.Logger.Debug("Clearing monsters around Nihlathak position")

		n.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			for _, m := range d.Monsters.Enemies() {
				if d := pather.DistanceFromPoint(nihlaObject.Position, m.Position); d < 15 {
					return m.UnitID, true
				}
			}

			return 0, false
		}, nil)
	}

	action.ItemPickup(30)

	if IsQuestRun(parameters) {
		err = action.ReturnTown()
		if err != nil {
			return err
		}

		err = n.goToAnyaInTown()
		if err != nil {
			return err
		}

		err = action.InteractNPC(npc.Drehya)
		if err != nil {
			return err
		}
	}

	return nil
}

func (n Nihlathak) findBestCorner(nihlathakPosition data.Position) data.Position {
	corners := [4]data.Position{
		{
			X: nihlathakPosition.X + 20,
			Y: nihlathakPosition.Y + 20,
		},
		{
			X: nihlathakPosition.X - 20,
			Y: nihlathakPosition.Y + 20,
		},
		{
			X: nihlathakPosition.X - 20,
			Y: nihlathakPosition.Y - 20,
		},
		{
			X: nihlathakPosition.X + 20,
			Y: nihlathakPosition.Y - 20,
		},
	}

	bestCorner := 0
	bestCornerDistance := 0
	for i, c := range corners {
		if n.ctx.Data.AreaData.IsWalkable(c) {
			averageDistance := 0
			for _, m := range n.ctx.Data.Monsters.Enemies() {
				averageDistance += pather.DistanceFromPoint(c, m.Position)
			}
			if averageDistance > bestCornerDistance {
				bestCorner = i
				bestCornerDistance = averageDistance
			}
			n.ctx.Logger.Debug("Corner", slog.Int("corner", i), slog.Int("monsters", len(n.ctx.Data.Monsters.Enemies())), slog.Int("distance", averageDistance))
		}
	}

	return corners[bestCorner]
}

func (n Nihlathak) goToAnyaInTown() error {
	// Always make sure we are in Harrogath first.
	if n.ctx.Data.PlayerUnit.Area != area.Harrogath {
		if err := action.WayPoint(area.Harrogath); err != nil {
			return errors.New("could not move to Harrogath")
		}
	}

	anyaTownPos, found := n.ctx.Data.Objects.FindOne(object.DrehyaTownStartPosition)
	if !found {
		return errors.New("Anya town start position not found in Harrogath")
	}

	return action.MoveToCoords(anyaTownPos.Position)
}

func (n Nihlathak) getHallOfPainWp() error {
	// old goToAnya + portal logic is now in useAnyaTemplePortal
	if err := n.useAnyaTemplePortal(); err != nil {
		return err
	}

	if err := action.MoveToArea(area.HallsOfAnguish); err != nil {
		return err
	}

	if err := action.MoveToArea(area.HallsOfPain); err != nil {
		return err
	}

	if err := action.DiscoverWaypoint(); err != nil {
		return err
	}

	return nil
}

// Uses Anya's permanent red portal to enter Nihlathak's Temple.
func (n Nihlathak) useAnyaTemplePortal() error {
	// Go to Anya in town
	if err := n.goToAnyaInTown(); err != nil {
		return err
	}

	templeTp, found := n.ctx.Data.Objects.FindOne(object.PermanentTownPortal)
	if !found {
		// Try to talk to Anya to open the portal
		if err := action.InteractNPC(npc.Drehya); err != nil {
			return err
		}
		utils.Sleep(1000)
		n.ctx.RefreshGameData()
		utils.Sleep(200)

		templeTp, found = n.ctx.Data.Objects.FindOne(object.PermanentTownPortal)
		if !found {
			return errors.New("couldn't find anya pos in town")
		}
	}

	// Take portal into Nihlathak's Temple (Pindle)
	return action.InteractObject(templeTp, func() bool {
		return n.ctx.Data.PlayerUnit.Area == area.NihlathaksTemple
	})
}

func (n Nihlathak) runTerrorZone(parameters *RunParameters) error {
	_ = parameters // currently unused, but kept for symmetry

	tzCfg := n.ctx.CharacterCfg.Game.TerrorZone

	if err := n.useAnyaTemplePortal(); err != nil {
		return err
	}

	if err := n.killPindleFast(); err != nil {
		n.ctx.Logger.Warn("[Nihl TZ] Failed to kill Pindle, continuing",
			slog.Any("error", err))
	}

	// --- Halls of Anguish ---
	if err := action.MoveToArea(area.HallsOfAnguish); err != nil {
		return err
	}
	if err := action.ClearCurrentLevel(tzCfg.OpenChests, n.clearMonsterFilter); err != nil {
		return err
	}

	// --- Halls of Pain ---
	if err := action.MoveToArea(area.HallsOfPain); err != nil {
		return err
	}
	if err := action.ClearCurrentLevel(tzCfg.OpenChests, n.clearMonsterFilter); err != nil {
		return err
	}

	// --- Halls of Vaught (Nihlathak level) ---
	if err := action.MoveToArea(area.HallsOfVaught); err != nil {
		return err
	}
	if err := action.ClearCurrentLevel(tzCfg.OpenChests, n.clearMonsterFilter); err != nil {
		return err
	}

	// ClearCurrentLevel will also kill Nihlathak as part of the level clear.
	// If you want the special corner-position logic for Nihl here too, we can
	// wire that in afterwards as an extra optimization.

	return nil
}

func (n Nihlathak) killPindleFast() error {
	// Reuse pindleSafePosition from pindleskin.go
	_ = action.MoveToCoords(pindleSafePosition)

	if err := n.ctx.Char.KillPindle(); err != nil {
		return err
	}

	action.ItemPickup(30)
	return nil
}
