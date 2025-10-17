package run

import (
	"github.com/hectorgimenez/koolo/internal/config"
)

type Run interface {
	Name() string
	Run() error
}

func BuildRuns(cfg *config.CharacterCfg, runs []string) (builtRuns []Run) {
	//if cfg.Companion.Enabled && !cfg.Companion.Leader {
	//	return []Run{Companion{baseRun: baseRun}}
	//}

	for _, run := range cfg.Game.Runs {
		// Prepend terror zone runs, we want to run it always first
		if run == config.TerrorZoneRun {
			tz := NewTerrorZone()

			if len(tz.AvailableTZs()) > 0 {
				builtRuns = append(builtRuns, tz)
				// If we are skipping other runs, we can return here
				if cfg.Game.TerrorZone.SkipOtherRuns {
					return builtRuns
				}
			}
		}
	}

	for _, run := range runs {
		switch run {
		case string(config.CountessRun):
			builtRuns = append(builtRuns, NewCountess())
		case string(config.AndarielRun):
			builtRuns = append(builtRuns, NewAndariel())
		case string(config.SummonerRun):
			builtRuns = append(builtRuns, NewSummoner())
		case string(config.DurielRun):
			builtRuns = append(builtRuns, NewDuriel())
		case string(config.MuleRun):
			builtRuns = append(builtRuns, NewMule())
		case string(config.MephistoRun):
			builtRuns = append(builtRuns, NewMephisto(nil))
		case string(config.TravincalRun):
			builtRuns = append(builtRuns, NewTravincal())
		case string(config.DiabloRun):
			builtRuns = append(builtRuns, NewDiablo())
		case string(config.EldritchRun):
			builtRuns = append(builtRuns, NewEldritch())
		case string(config.PindleskinRun):
			builtRuns = append(builtRuns, NewPindleskin())
		case string(config.NihlathakRun):
			builtRuns = append(builtRuns, NewNihlathak())
		case string(config.AncientTunnelsRun):
			builtRuns = append(builtRuns, NewAncientTunnels())
		case string(config.MausoleumRun):
			builtRuns = append(builtRuns, NewMausoleum())
		case string(config.PitRun):
			builtRuns = append(builtRuns, NewPit())
		case string(config.StonyTombRun):
			builtRuns = append(builtRuns, NewStonyTomb())
		case string(config.ArachnidLairRun):
			builtRuns = append(builtRuns, NewArachnidLair())
		case string(config.TristramRun):
			builtRuns = append(builtRuns, NewTristram())
		case string(config.LowerKurastRun):
			builtRuns = append(builtRuns, NewLowerKurast())
		case string(config.LowerKurastChestRun):
			builtRuns = append(builtRuns, NewLowerKurastChest())
		case string(config.BaalRun):
			builtRuns = append(builtRuns, NewBaal(nil))
		case string(config.TalRashaTombsRun):
			builtRuns = append(builtRuns, NewTalRashaTombs())
		case string(config.LevelingRun):
			builtRuns = append(builtRuns, NewLeveling())
		case string(config.QuestsRun):
			builtRuns = append(builtRuns, NewQuests())
		case string(config.CowsRun):
			builtRuns = append(builtRuns, NewCows())
		case string(config.ThreshsocketRun):
			builtRuns = append(builtRuns, NewThreshsocket())
		case string(config.SpiderCavernRun):
			builtRuns = append(builtRuns, NewSpiderCavern())
		case string(config.DrifterCavernRun):
			builtRuns = append(builtRuns, NewDriverCavern())
		case string(config.EnduguRun):
			builtRuns = append(builtRuns, NewEndugu())
		}
	}

	return builtRuns
}
