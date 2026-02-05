package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/context"
	terrorzones "github.com/hectorgimenez/koolo/internal/terrorzone"
)

type TerrorZone struct {
	ctx *context.Status
}

func NewTerrorZone() *TerrorZone {
	return &TerrorZone{
		ctx: context.Get(),
	}
}

func (tz TerrorZone) Name() string {
	tzNames := make([]string, 0)
	for _, tzArea := range tz.AvailableTZs() {
		tzNames = append(tzNames, tzArea.Area().Name)
	}

	return fmt.Sprintf("TerrorZone Run: %v", tzNames)
}

func (tz TerrorZone) CheckConditions(parameters *RunParameters) SequencerResult {
	return SequencerError
}

func (tz TerrorZone) Run(parameters *RunParameters) error {

	availableTzs := tz.AvailableTZs()
	if len(availableTzs) == 0 {
		return nil
	}

	// --- Special-case TZs that already have dedicated runs ---
	switch availableTzs[0] {
	case area.PitLevel1, area.PitLevel2:
		return NewPit().Run(parameters)
	case area.Tristram:
		return NewTristram().Run(parameters)
	case area.MooMooFarm:
		return NewCows().Run(parameters)
	case area.TalRashasTomb1:
		return NewTalRashaTombs().Run(parameters)
	case area.AncientTunnels:
		return NewAncientTunnels().Run(parameters)
	case area.ArcaneSanctuary:
		return NewArcaneSanctuaryTZ(tz.customTZEnemyFilter()).Run(parameters)
	case area.Travincal:
		return NewTravincal().Run(parameters)
	case area.DuranceOfHateLevel1:
		return NewMephisto(tz.customTZEnemyFilter()).Run(parameters)
	case area.ChaosSanctuary:
		return NewDiablo().Run(parameters)
	case area.NihlathaksTemple:
		return NewNihlathakTZ(tz.customTZEnemyFilter()).Run(parameters)
	case area.TheWorldStoneKeepLevel1:
		return NewBaal(tz.customTZEnemyFilter()).Run(parameters)
	}

	// --- Generic TZ handling via centralized routes ---
	primary := availableTzs[0]

	routes := terrorzones.RoutesFor(primary)
	if len(routes) == 0 {
		tz.ctx.Logger.Debug("No terror zone route defined for %v", primary.Area().Name)
		return nil
	}

	for _, route := range routes {
		for idx, step := range route {
			// Navigation: first step via waypoint, rest via MoveToArea
			if idx == 0 {
				if err := action.WayPoint(step.Area); err != nil {
					return err
				}
			} else {
				if err := action.MoveToArea(step.Area); err != nil {
					return err
				}
			}

			// Clearing: only if the route explicitly says so.
			// We trust routes.go + terrorzones.go to define the correct group.
			if step.Kind == terrorzones.StepClear {
				if err := action.ClearCurrentLevel(
					tz.ctx.CharacterCfg.Game.TerrorZone.OpenChests,
					tz.customTZEnemyFilter(),
				); err != nil {
					return err
				}
			}
		}
	}

	return nil
}

func (tz TerrorZone) AvailableTZs() []area.ID {
	tz.ctx.RefreshGameData()
	var availableTZs []area.ID
	for _, tzone := range tz.ctx.Data.TerrorZones {
		for _, tzArea := range tz.ctx.CharacterCfg.Game.TerrorZone.Areas {
			if tzone == tzArea {
				availableTZs = append(availableTZs, tzone)
			}
		}
	}

	return availableTZs
}

func (tz TerrorZone) customTZEnemyFilter() data.MonsterFilter {
	return func(m data.Monsters) []data.Monster {
		var filteredMonsters []data.Monster
		monsterFilter := data.MonsterAnyFilter()
		if tz.ctx.CharacterCfg.Game.TerrorZone.FocusOnElitePacks {
			monsterFilter = data.MonsterEliteFilter()
		}

		for _, mo := range m.Enemies(monsterFilter) {
			isImmune := false
			for _, resist := range tz.ctx.CharacterCfg.Game.TerrorZone.SkipOnImmunities {
				if mo.IsImmune(resist) {
					isImmune = true
				}
			}
			if !isImmune {
				filteredMonsters = append(filteredMonsters, mo)
			}
		}

		return filteredMonsters
	}
}
