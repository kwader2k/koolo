package config

type Run string

const (
	CountessRun         Run = "countess"
	AndarielRun         Run = "andariel"
	AncientTunnelsRun   Run = "ancient_tunnels"
	MausoleumRun        Run = "mausoleum"
	SummonerRun         Run = "summoner"
	DurielRun           Run = "duriel"
	MuleRun             Run = "mule"
	MephistoRun         Run = "mephisto"
	TravincalRun        Run = "travincal"
	EldritchRun         Run = "eldritch"
	PindleskinRun       Run = "pindleskin"
	NihlathakRun        Run = "nihlathak"
	TristramRun         Run = "tristram"
	JailRun             Run = "jail"
	BoneAshRun          Run = "bone_ash"
	CaveRun             Run = "cave"
	FlayerJungleRun     Run = "flayer_jungle"
	LowerKurastRun      Run = "lower_kurast"
	LowerKurastChestRun Run = "lower_kurast_chest"
	KurastTemplesRun    Run = "kurast_temples"
	StonyTombRun        Run = "stony_tomb"
	PitRun              Run = "pit"
	ArachnidLairRun     Run = "arachnid_lair"
	TalRashaTombsRun    Run = "tal_rasha_tombs"
	BaalRun             Run = "baal"
	RiverOfFlameRun     Run = "river_of_flame"
	DiabloRun           Run = "diablo"
	CowsRun             Run = "cows"
	LevelingRun         Run = "leveling"
	LevelingSequenceRun Run = "leveling_sequence"
	QuestsRun           Run = "quests"
	TerrorZoneRun       Run = "terror_zone"
	ThreshsocketRun     Run = "threshsocket"
	DrifterCavernRun    Run = "drifter_cavern"
	SpiderCavernRun     Run = "spider_cavern"
	EnduguRun           Run = "endugu"
	UtilityRun          Run = "utility"
	FireEyeRun          Run = "fire_eye"
	RakanishuRun        Run = "rakanishu"
	ShoppingRun         Run = "shopping"
	//Leveling Sequence
	DenRun                   Run = "den"
	BloodravenRun            Run = "bloodraven"
	RescueCainRun            Run = "rescue_cain"
	RetrieveHammerRun        Run = "retrieve_hammer"
	RadamentRun              Run = "radament"
	CubeRun                  Run = "cube"
	StaffRun                 Run = "staff"
	AmuletRun                Run = "amulet"
	JadeFigurineRun          Run = "jade_figurine"
	GidbinnRun               Run = "gidbinn"
	LamEsenRun               Run = "lam_esen"
	KhalimsEyeRun            Run = "khalims_eye"
	KhalimsBrainRun          Run = "khalims_brain"
	KhalimsHeartRun          Run = "khalims_heart"
	IzualRun                 Run = "izual"
	HellforgeRun             Run = "hellforge"
	ShenkRun                 Run = "shenk"
	RescueBarbsRun           Run = "rescue_barbs"
	AnyaRun                  Run = "anya"
	AncientsRun              Run = "ancients"
	FrozenAuraMercRun        Run = "frozen_aura_merc"
	TristramEarlyGoldfarmRun Run = "tristram_early_gold_farm"
	OrgansRun                Run = "uber_organs"
	PandemoniumRun           Run = "uber_torch"
	UberIzualRun             Run = "uber_izual"
	UberDurielRun            Run = "uber_duriel"
	LilithRun                Run = "lilith"
	// Development / Utility runs
	DevelopmentRun Run = "development"
)

type LevelingRunInfo struct {
	Run         Run
	Act         int
	IsMandatory bool
}

var AvailableRuns = map[Run]interface{}{
	CountessRun:         nil,
	AndarielRun:         nil,
	AncientTunnelsRun:   nil,
	MausoleumRun:        nil,
	SummonerRun:         nil,
	DurielRun:           nil,
	MuleRun:             nil,
	MephistoRun:         nil,
	TravincalRun:        nil,
	EldritchRun:         nil,
	PindleskinRun:       nil,
	NihlathakRun:        nil,
	TristramRun:         nil,
	JailRun:             nil,
	BoneAshRun:          nil,
	CaveRun:             nil,
	FlayerJungleRun:     nil,
	LowerKurastRun:      nil,
	LowerKurastChestRun: nil,
	KurastTemplesRun:    nil,
	StonyTombRun:        nil,
	PitRun:              nil,
	ArachnidLairRun:     nil,
	TalRashaTombsRun:    nil,
	BaalRun:             nil,
	RiverOfFlameRun:     nil,
	DiabloRun:           nil,
	CowsRun:             nil,
	LevelingRun:         nil,
	LevelingSequenceRun: nil,
	QuestsRun:           nil,
	TerrorZoneRun:       nil,
	ThreshsocketRun:     nil,
	DrifterCavernRun:    nil,
	SpiderCavernRun:     nil,
	EnduguRun:           nil,
	UtilityRun:          nil,
	FireEyeRun:          nil,
	ShoppingRun:         nil,
	OrgansRun:           nil,
	PandemoniumRun:      nil,
	DevelopmentRun:      nil,
}

var SequencerQuests = []LevelingRunInfo{
	// Act 1
	{Run: DenRun, Act: 1, IsMandatory: false},
	{Run: BloodravenRun, Act: 1, IsMandatory: false},
	{Run: RescueCainRun, Act: 1, IsMandatory: false},
	{Run: CountessRun, Act: 1, IsMandatory: false},
	{Run: RetrieveHammerRun, Act: 1, IsMandatory: false},
	{Run: AndarielRun, Act: 1, IsMandatory: true},
	// Act 2
	{Run: FrozenAuraMercRun, Act: 2, IsMandatory: false},
	{Run: RadamentRun, Act: 2, IsMandatory: false},
	{Run: CubeRun, Act: 2, IsMandatory: true},
	{Run: StaffRun, Act: 2, IsMandatory: true},
	{Run: AmuletRun, Act: 2, IsMandatory: true},
	{Run: SummonerRun, Act: 2, IsMandatory: true},
	{Run: DurielRun, Act: 2, IsMandatory: true},
	// Act 3
	{Run: JadeFigurineRun, Act: 3, IsMandatory: false},
	{Run: KhalimsEyeRun, Act: 3, IsMandatory: true},
	{Run: KhalimsBrainRun, Act: 3, IsMandatory: true},
	{Run: KhalimsHeartRun, Act: 3, IsMandatory: true},
	{Run: GidbinnRun, Act: 3, IsMandatory: false},
	{Run: LamEsenRun, Act: 3, IsMandatory: false},
	{Run: TravincalRun, Act: 3, IsMandatory: true},
	{Run: MephistoRun, Act: 3, IsMandatory: true},
	// Act 4
	{Run: IzualRun, Act: 4, IsMandatory: false},
	{Run: HellforgeRun, Act: 4, IsMandatory: false},
	{Run: DiabloRun, Act: 4, IsMandatory: true},
	// Act 5
	{Run: ShenkRun, Act: 5, IsMandatory: false},
	{Run: RescueBarbsRun, Act: 5, IsMandatory: false},
	{Run: AnyaRun, Act: 5, IsMandatory: false},
	{Run: NihlathakRun, Act: 5, IsMandatory: false},
	{Run: AncientsRun, Act: 5, IsMandatory: true},
	{Run: BaalRun, Act: 5, IsMandatory: true},
}

var SequencerRuns = []Run{
	AmuletRun,
	AncientsRun,
	AndarielRun,
	AnyaRun,
	ArachnidLairRun,
	BaalRun,
	BloodravenRun,
	BoneAshRun,
	CaveRun,
	CountessRun,
	CowsRun,
	CubeRun,
	DenRun,
	DiabloRun,
	DrifterCavernRun,
	DurielRun,
	EldritchRun,
	EnduguRun,
	FireEyeRun,
	FlayerJungleRun,
	FrozenAuraMercRun,
	GidbinnRun,
	IzualRun,
	JadeFigurineRun,
	JailRun,
	KhalimsBrainRun,
	KhalimsEyeRun,
	KhalimsHeartRun,
	KurastTemplesRun,
	LamEsenRun,
	LowerKurastChestRun,
	LowerKurastRun,
	MausoleumRun,
	MephistoRun,
	NihlathakRun,
	PindleskinRun,
	PitRun,
	RadamentRun,
	RakanishuRun,
	RescueBarbsRun,
	RescueCainRun,
	RetrieveHammerRun,
	RiverOfFlameRun,
	ShenkRun,
	SpiderCavernRun,
	StaffRun,
	StonyTombRun,
	SummonerRun,
	TalRashaTombsRun,
	TerrorZoneRun,
	ThreshsocketRun,
	TravincalRun,
	TristramEarlyGoldfarmRun,
	TristramRun,
	OrgansRun,
	UberIzualRun,
	UberDurielRun,
	LilithRun,
}
