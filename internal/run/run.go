package run

import (
	"github.com/hectorgimenez/koolo/internal/config"
)

type SequencerResult int8

const (
	SequencerSkip SequencerResult = iota
	SequencerStop
	SequencerOk
	SequencerError
)

type RunParameters struct {
	FarmingRun       bool
	SequenceSettings *SequenceSettings
}

type Run interface {
	Name() string
	Run(parameters *RunParameters) error
	CheckConditions(parameters *RunParameters) SequencerResult
}

// TownRoutineSkipper allows specific runs to suppress the automatic PreRun/PostRun sequences.
type TownRoutineSkipper interface {
	SkipTownRoutines() bool
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
		if runInterface := BuildRun(run); runInterface != nil {
			builtRuns = append(builtRuns, runInterface)
		}
	}

	return builtRuns
}

func BuildRun(run string) Run {
	switch run {
	case string(config.CountessRun):
		return NewCountess()
	case string(config.AndarielRun):
		return NewAndariel()
	case string(config.SummonerRun):
		return NewSummoner()
	case string(config.DurielRun):
		return NewDuriel()
	case string(config.MuleRun):
		return NewMule()
	case string(config.MephistoRun):
		return NewMephisto(nil)
	case string(config.TravincalRun):
		return NewTravincal()
	case string(config.RiverOfFlameRun):
		return NewRiverOfFlame()
	case string(config.DiabloRun):
		return NewDiablo()
	case string(config.EldritchRun):
		return NewEldritch()
	case string(config.PindleskinRun):
		return NewPindleskin()
	case string(config.NihlathakRun):
		return NewNihlathak()
	case string(config.AncientTunnelsRun):
		return NewAncientTunnels()
	case string(config.MausoleumRun):
		return NewMausoleum()
	case string(config.PitRun):
		return NewPit()
	case string(config.StonyTombRun):
		return NewStonyTomb()
	case string(config.ArachnidLairRun):
		return NewArachnidLair()
	case string(config.TristramRun):
		return NewTristram()
	case string(config.JailRun):
		return NewJail()
	case string(config.BoneAshRun):
		return NewBoneAsh()
	case string(config.CaveRun):
		return NewCave()
	case string(config.FlayerJungleRun):
		return NewFlayerJungle()
	case string(config.LowerKurastRun):
		return NewLowerKurast()
	case string(config.LowerKurastChestRun):
		return NewLowerKurastChest()
	case string(config.KurastTemplesRun):
		return NewKurastTemples()
	case string(config.BaalRun):
		return NewBaal(nil)
	case string(config.TalRashaTombsRun):
		return NewTalRashaTombs()
	case string(config.LevelingRun):
		return NewLeveling()
	case string(config.LevelingSequenceRun):
		return NewLevelingSequence()
	case string(config.QuestsRun):
		return NewQuests()
	case string(config.CowsRun):
		return NewCows()
	case string(config.ThreshsocketRun):
		return NewThreshsocket()
	case string(config.SpiderCavernRun):
		return NewSpiderCavern()
	case string(config.DrifterCavernRun):
		return NewDriverCavern()
	case string(config.EnduguRun):
		return NewEndugu()
	case string(config.UtilityRun):
		return NewUtility()
	case string(config.FireEyeRun):
		return NewFireEye()
	case string(config.RakanishuRun):
		return NewRakanishu()
	case string(config.ShoppingRun):
		return NewShopping()
	//Quests Runs
	case string(config.DenRun):
		return NewDen()
	case string(config.BloodravenRun):
		return NewBloodraven()
	case string(config.RescueCainRun):
		return NewRescueCain()
	case string(config.RetrieveHammerRun):
		return NewRetrieveHammer()
	case string(config.RadamentRun):
		return NewRadament()
	case string(config.CubeRun):
		return NewCube()
	case string(config.StaffRun):
		return NewStaff()
	case string(config.AmuletRun):
		return NewAmulet()
	case string(config.JadeFigurineRun):
		return NewJadeFigurine()
	case string(config.GidbinnRun):
		return NewGidbinn()
	case string(config.LamEsenRun):
		return NewLamEsen()
	case string(config.KhalimsEyeRun):
		return NewKhalimsEye()
	case string(config.KhalimsBrainRun):
		return NewKhalimsBrain()
	case string(config.KhalimsHeartRun):
		return NewKhalimsHeart()
	case string(config.IzualRun):
		return NewIzual()
	case string(config.HellforgeRun):
		return NewHellforge()
	case string(config.ShenkRun):
		return NewShenk()
	case string(config.RescueBarbsRun):
		return NewRescueBarbs()
	case string(config.AnyaRun):
		return NewAnya()
	case string(config.AncientsRun):
		return NewAncients()
	case string(config.FrozenAuraMercRun):
		return NewFrozenAuraMerc()
	case string(config.TristramEarlyGoldfarmRun):
		return NewTristramEarlyGoldfarm()
	case string(config.OrgansRun):
		return NewOrgans()
	case string(config.PandemoniumRun):
		return NewTorch()
	case string(config.UberIzualRun):
		return NewUberIzual()
	case string(config.UberDurielRun):
		return NewUberDuriel()
	case string(config.LilithRun):
		return NewLilith()
	// Development / Utility runs
	case string(config.DevelopmentRun):
		return NewDevRun()
	}

	return nil
}

func BuildRunParameters(farmingRun bool, sequenceSettings *SequenceSettings) *RunParameters {
	var RunParameters RunParameters
	RunParameters.FarmingRun = farmingRun
	RunParameters.SequenceSettings = sequenceSettings
	return &RunParameters
}

func IsFarmingRun(parameters *RunParameters) bool {
	return parameters == nil || parameters.FarmingRun
}

func IsQuestRun(parameters *RunParameters) bool {
	return parameters != nil && !parameters.FarmingRun
}
