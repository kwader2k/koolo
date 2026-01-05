package run

import (
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/icc"
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
	var runInstance Run
	switch run {
	case string(config.CountessRun):
		runInstance = NewCountess()
	case string(config.AndarielRun):
		runInstance = NewAndariel()
	case string(config.SummonerRun):
		runInstance = NewSummoner()
	case string(config.DurielRun):
		runInstance = NewDuriel()
	case string(config.MuleRun):
		runInstance = NewMule()
	case string(config.MephistoRun):
		runInstance = NewMephisto(nil)
	case string(config.TravincalRun):
		runInstance = NewTravincal()
	case string(config.DiabloRun):
		runInstance = NewDiablo()
	case string(config.EldritchRun):
		runInstance = NewEldritch()
	case string(config.PindleskinRun):
		runInstance = NewPindleskin()
	case string(config.NihlathakRun):
		runInstance = NewNihlathak()
	case string(config.AncientTunnelsRun):
		runInstance = NewAncientTunnels()
	case string(config.MausoleumRun):
		runInstance = NewMausoleum()
	case string(config.PitRun):
		runInstance = NewPit()
	case string(config.StonyTombRun):
		runInstance = NewStonyTomb()
	case string(config.ArachnidLairRun):
		runInstance = NewArachnidLair()
	case string(config.TristramRun):
		runInstance = NewTristram()
	case string(config.LowerKurastRun):
		runInstance = NewLowerKurast()
	case string(config.LowerKurastChestRun):
		runInstance = NewLowerKurastChest()
	case string(config.BaalRun):
		runInstance = NewBaal(nil)
	case string(config.TalRashaTombsRun):
		runInstance = NewTalRashaTombs()
	case string(config.LevelingRun):
		runInstance = NewLeveling()
	case string(config.LevelingSequenceRun):
		runInstance = NewLevelingSequence()
	case string(config.QuestsRun):
		runInstance = NewQuests()
	case string(config.CowsRun):
		runInstance = NewCows()
	case string(config.ThreshsocketRun):
		runInstance = NewThreshsocket()
	case string(config.SpiderCavernRun):
		runInstance = NewSpiderCavern()
	case string(config.DrifterCavernRun):
		runInstance = NewDriverCavern()
	case string(config.EnduguRun):
		runInstance = NewEndugu()
	case string(config.UtilityRun):
		runInstance = NewUtility()
	case string(config.FireEyeRun):
		runInstance = NewFireEye()
	case string(config.RakanishuRun):
		runInstance = NewRakanishu()
	case string(config.ShoppingRun):
		runInstance = NewShopping()
	//Quests Runs
	case string(config.DenRun):
		runInstance = NewDen()
	case string(config.BloodravenRun):
		runInstance = NewBloodraven()
	case string(config.RescueCainRun):
		runInstance = NewRescueCain()
	case string(config.RetrieveHammerRun):
		runInstance = NewRetrieveHammer()
	case string(config.RadamentRun):
		runInstance = NewRadament()
	case string(config.CubeRun):
		runInstance = NewCube()
	case string(config.StaffRun):
		runInstance = NewStaff()
	case string(config.AmuletRun):
		runInstance = NewAmulet()
	case string(config.JadeFigurineRun):
		runInstance = NewJadeFigurine()
	case string(config.GidbinnRun):
		runInstance = NewGidbinn()
	case string(config.LamEsenRun):
		runInstance = NewLamEsen()
	case string(config.KhalimsEyeRun):
		runInstance = NewKhalimsEye()
	case string(config.KhalimsBrainRun):
		runInstance = NewKhalimsBrain()
	case string(config.KhalimsHeartRun):
		runInstance = NewKhalimsHeart()
	case string(config.IzualRun):
		runInstance = NewIzual()
	case string(config.HellforgeRun):
		runInstance = NewHellforge()
	case string(config.ShenkRun):
		runInstance = NewShenk()
	case string(config.RescueBarbsRun):
		runInstance = NewRescueBarbs()
	case string(config.AnyaRun):
		runInstance = NewAnya()
	case string(config.AncientsRun):
		runInstance = NewAncients()
	case string(config.FrozenAuraMercRun):
		runInstance = NewFrozenAuraMerc()
	case string(config.TristramEarlyGoldfarmRun):
		runInstance = NewTristramEarlyGoldfarm()
	case string(config.OrgansRun):
		runInstance = NewOrgans()
	case string(config.PandemoniumRun):
		runInstance = NewTorch()
	case string(config.UberIzualRun):
		runInstance = NewUberIzual()
	case string(config.UberDurielRun):
		runInstance = NewUberDuriel()
	case string(config.LilithRun):
		runInstance = NewLilith()
	// Development / Utility runs
	case string(config.DevelopmentRun):
		runInstance = NewDevRun()
	}

	return wrapRun(runInstance)
}

// wraps a Run for auto-broadcasting
func wrapRun(runInstance Run) Run {
	if runInstance == nil {
		return nil
	}
	ctx := context.Get()
	if ctx.CharacterCfg.GroupLeveling.Enabled {
		return &broadcastRun{
			wrapped: runInstance,
			ctx:     ctx,
		}
	}
	return runInstance
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

// broadcastRun wraps a Run to auto-broadcast
type broadcastRun struct {
	wrapped Run
	ctx     *context.Status
}

func (br *broadcastRun) Name() string {
	return br.wrapped.Name()
}

func (br *broadcastRun) Run(parameters *RunParameters) error {
	// Broadcast if leader
	if br.ctx.ICCManager != nil {
		if iccMgr, ok := br.ctx.ICCManager.(*context.ICCManager); ok {
			coord := icc.NewGroupLevelingCoordinator(br.ctx, iccMgr)
			if coord.IsLeader() {
				coord.BroadcastRun(br.wrapped.Name(), map[string]interface{}{})
			}
		}
	}
	return br.wrapped.Run(parameters)
}

func (br *broadcastRun) CheckConditions(parameters *RunParameters) SequencerResult {
	return br.wrapped.CheckConditions(parameters)
}
