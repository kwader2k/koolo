package config

import (
	"encoding/json"
	"errors"
	"fmt"
	"path/filepath"
	"slices"
	"sync"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/koolo/internal/utils"

	"os"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	cp "github.com/otiai10/copy"

	"github.com/hectorgimenez/d2go/pkg/nip"

	"gopkg.in/yaml.v3"
)

var (
	cfgMux     sync.RWMutex
	Koolo      *KooloCfg
	Characters map[string]*CharacterCfg
	Version    = "dev"

	// NIP rules cache - stores compiled rules by path to avoid recompiling for multiple characters
	nipRulesCacheMux sync.RWMutex
	nipRulesCache    = make(map[string]nip.Rules)
)

const (
	GameVersionReignOfTheWarlock = "reign_of_the_warlock"
	GameVersionExpansion         = "expansion"
)

type KooloCfg struct {
	Debug struct {
		Log                       bool `yaml:"log"`
		Screenshots               bool `yaml:"screenshots"`
		RenderMap                 bool `yaml:"renderMap"`
		OpenOverlayMapOnGameStart bool `yaml:"openOverlayMapOnGameStart"`
	} `yaml:"debug"`
	FirstRun              bool   `yaml:"firstRun"`
	UseCustomSettings     bool   `yaml:"useCustomSettings"`
	GameWindowArrangement bool   `yaml:"gameWindowArrangement"`
	LogSaveDirectory      string `yaml:"logSaveDirectory"`
	D2LoDPath             string `yaml:"D2LoDPath"`
	D2RPath               string `yaml:"D2RPath"`
	CentralizedPickitPath string `yaml:"centralizedPickitPath"`
	WindowWidth           int    `yaml:"windowWidth"`
	WindowHeight          int    `yaml:"windowHeight"`
	Discord               struct {
		Enabled                      bool     `yaml:"enabled"`
		EnableGameCreatedMessages    bool     `yaml:"enableGameCreatedMessages"`
		EnableNewRunMessages         bool     `yaml:"enableNewRunMessages"`
		EnableRunFinishMessages      bool     `yaml:"enableRunFinishMessages"`
		EnableDiscordChickenMessages bool     `yaml:"enableDiscordChickenMessages"`
		EnableDiscordErrorMessages   bool     `yaml:"enableDiscordErrorMessages"`
		DisableItemStashScreenshots  bool     `yaml:"disableItemStashScreenshots"`
		IncludePickitInfoInItemText  bool     `yaml:"includePickitInfoInItemText"`
		BotAdmins                    []string `yaml:"botAdmins"`
		ChannelID                    string   `yaml:"channelId"`
		ItemChannelID                string   `yaml:"itemChannelId"`
		Token                        string   `yaml:"token"`
		UseWebhook                   bool     `yaml:"useWebhook"`
		WebhookURL                   string   `yaml:"webhookUrl"`
		ItemWebhookURL               string   `yaml:"itemWebhookUrl"`
	} `yaml:"discord"`
	Telegram struct {
		Enabled bool   `yaml:"enabled"`
		ChatID  int64  `yaml:"chatId"`
		Token   string `yaml:"token"`
	}
	Ngrok struct {
		Enabled       bool   `yaml:"enabled"`
		SendURL       bool   `yaml:"sendUrl"`
		Authtoken     string `yaml:"authtoken"`
		Region        string `yaml:"region"`
		Domain        string `yaml:"domain"`
		BasicAuthUser string `yaml:"basicAuthUser"`
		BasicAuthPass string `yaml:"basicAuthPass"`
	} `yaml:"ngrok"`
	PingMonitor struct {
		Enabled           bool `yaml:"enabled"`
		HighPingThreshold int  `yaml:"highPingThreshold"` // Ping threshold in ms (default 500-1000)
		SustainedDuration int  `yaml:"sustainedDuration"` // Seconds high ping must persist (default 10-30)
	} `yaml:"pingMonitor"`
	AutoStart struct {
		Enabled      bool `yaml:"enabled"`
		DelaySeconds int  `yaml:"delaySeconds"`
	} `yaml:"autoStart"`
	RunewordFavoriteRecipes []string `yaml:"runewordFavoriteRecipes"`
	RunFavoriteRuns         []string `yaml:"runFavoriteRuns"`
}

type Day struct {
	DayOfWeek  int         `yaml:"dayOfWeek"`
	TimeRanges []TimeRange `yaml:"timeRange"`
}

// RunewordOverrideConfig stores a character's overrides keyed by the display name (e.g. "Enigma").
type RunewordOverrideConfig struct {
	EthMode     string `yaml:"ethMode,omitempty"`     // "any", "eth", "noneth"
	QualityMode string `yaml:"qualityMode,omitempty"` // "any", "normal", "superior"
	BaseType    string `yaml:"baseType,omitempty"`    // armor, bow, polearm, etc.
	BaseTier    string `yaml:"baseTier,omitempty"`    // "", "normal", "exceptional", "elite"
	BaseName    string `yaml:"baseName,omitempty"`    // optional specific base name
}

// RunewordTargetStatOverride captures the desired min/max for a stat (and optional layer) when rerolling.
type RunewordTargetStatOverride struct {
	StatID stat.ID  `yaml:"statId"`          // numeric stat ID from d2go
	Layer  int      `yaml:"layer,omitempty"` // optional layer (e.g. skill/aura)
	Min    float64  `yaml:"min"`             // desired minimum value for this stat
	Max    *float64 `yaml:"max,omitempty"`   // optional maximum value for this stat
	Group  string   `yaml:"group,omitempty" json:"group,omitempty"`
}

// RunewordRerollRule defines filters plus target stats that existing items must satisfy before we keep them.
type RunewordRerollRule struct {
	EthMode     string                       `yaml:"ethMode,omitempty"`     // "any", "eth", "noneth"
	QualityMode string                       `yaml:"qualityMode,omitempty"` // "any", "normal", "superior"
	BaseType    string                       `yaml:"baseType,omitempty"`    // armor, bow, polearm, etc.
	BaseTier    string                       `yaml:"baseTier,omitempty"`    // "", "normal", "exceptional", "elite"
	BaseName    string                       `yaml:"baseName,omitempty"`    // optional specific base (NIP code)
	TargetStats []RunewordTargetStatOverride `yaml:"targetStats,omitempty"` // per-stat minimums for this rule
}

type Scheduler struct {
	Enabled bool   `yaml:"enabled"`
	Mode    string `yaml:"mode"` // "timeSlots" (default) or "duration"

	// Time Slots Mode (existing)
	Days              []Day `yaml:"days"`
	GlobalVarianceMin int   `yaml:"globalVarianceMin,omitempty"` // Default variance for all ranges (+/- minutes)

	// Duration Mode
	Duration DurationSchedule `yaml:"duration,omitempty"`
}

// DurationSchedule configures human-like play patterns with randomized breaks
type DurationSchedule struct {
	// Wake Up
	WakeUpTime     string `yaml:"wakeUpTime"`     // Base wake time "HH:MM" (e.g., "08:00")
	WakeUpVariance int    `yaml:"wakeUpVariance"` // +/- minutes (e.g., 30)

	// Play Duration
	PlayHours         int `yaml:"playHours"`         // Base play time per day (e.g., 14)
	PlayHoursVariance int `yaml:"playHoursVariance"` // +/- hours (e.g., 2 means 12-16h)

	// Meal Breaks (longer)
	MealBreakCount    int `yaml:"mealBreakCount"`    // Number of meal breaks (e.g., 2 for lunch+dinner)
	MealBreakDuration int `yaml:"mealBreakDuration"` // Base duration in minutes (e.g., 30)
	MealBreakVariance int `yaml:"mealBreakVariance"` // +/- minutes for duration (e.g., 15)

	// Short Breaks (snack/water/bathroom)
	ShortBreakCount    int `yaml:"shortBreakCount"`    // Number of short breaks (e.g., 3-4)
	ShortBreakDuration int `yaml:"shortBreakDuration"` // Base duration in minutes (e.g., 8)
	ShortBreakVariance int `yaml:"shortBreakVariance"` // +/- minutes for duration (e.g., 5)

	// Timing Variance (when breaks occur)
	BreakTimingVariance int `yaml:"breakTimingVariance"` // +/- minutes for break start times (e.g., 30)

	// Jitter Range - makes variance itself variable, randomized per roll
	JitterMin int `yaml:"jitterMin"` // Min jitter multiplier % (e.g., 30)
	JitterMax int `yaml:"jitterMax"` // Max jitter multiplier % (e.g., 150)
}

type TimeRange struct {
	Start            time.Time `yaml:"start"`
	End              time.Time `yaml:"end"`
	StartVarianceMin int       `yaml:"startVarianceMin,omitempty"` // +/- minutes for start time
	EndVarianceMin   int       `yaml:"endVarianceMin,omitempty"`   // +/- minutes for end time
}

type AutoStatSkillConfig struct {
	Enabled            bool                 `yaml:"enabled"`
	Stats              []AutoStatSkillStat  `yaml:"stats,omitempty"`
	Skills             []AutoStatSkillSkill `yaml:"skills,omitempty"`
	Respec             AutoRespecConfig     `yaml:"autoRespec,omitempty"`
	ExcludeQuestStats  bool                 `yaml:"excludeQuestStats,omitempty"`
	ExcludeQuestSkills bool                 `yaml:"excludeQuestSkills,omitempty"`
}

type AutoStatSkillStat struct {
	Stat   string `yaml:"stat"`
	Target int    `yaml:"target"`
}

type AutoStatSkillSkill struct {
	Skill  string `yaml:"skill"`
	Target int    `yaml:"target"`
}

type AutoRespecConfig struct {
	Enabled     bool `yaml:"enabled"`
	TokenFirst  bool `yaml:"tokenFirst,omitempty"`
	TargetLevel int  `yaml:"targetLevel,omitempty"`
	Applied     bool `yaml:"applied,omitempty"`
}

type CharacterCfg struct {
	MaxGameLength        int    `yaml:"maxGameLength"`
	Username             string `yaml:"username"`
	Password             string `yaml:"password"`
	AuthMethod           string `yaml:"authMethod"`
	AuthToken            string `yaml:"authToken"`
	Realm                string `yaml:"realm"`
	CharacterName        string `yaml:"characterName"`
	AutoCreateCharacter  bool   `yaml:"autoCreateCharacter"`
	CommandLineArgs      string `yaml:"commandLineArgs"`
	KillD2OnStop         bool   `yaml:"killD2OnStop"`
	ClassicMode          bool   `yaml:"classicMode"`
	UseCentralizedPickit bool   `yaml:"useCentralizedPickit"`
	HidePortraits        bool   `yaml:"hidePortraits"`
	AutoStart            bool   `yaml:"autoStart"`

	ConfigFolderName string `yaml:"-"`

	PacketCasting struct {
		UseForEntranceInteraction bool `yaml:"useForEntranceInteraction"`
		UseForItemPickup          bool `yaml:"useForItemPickup"`
		UseForTpInteraction       bool `yaml:"useForTpInteraction"`
		UseForTeleport            bool `yaml:"useForTeleport"`
		UseForEntitySkills        bool `yaml:"useForEntitySkills"`
		UseForSkillSelection      bool `yaml:"useForSkillSelection"`
	} `yaml:"packetCasting"`

	Scheduler Scheduler `yaml:"scheduler"`
	Health    struct {
		HealingPotionAt     int `yaml:"healingPotionAt"`
		ManaPotionAt        int `yaml:"manaPotionAt"`
		RejuvPotionAtLife   int `yaml:"rejuvPotionAtLife"`
		RejuvPotionAtMana   int `yaml:"rejuvPotionAtMana"`
		MercHealingPotionAt int `yaml:"mercHealingPotionAt"`
		MercRejuvPotionAt   int `yaml:"mercRejuvPotionAt"`
		ChickenAt           int `yaml:"chickenAt"`
		TownChickenAt       int `yaml:"townChickenAt"`
		MercChickenAt       int `yaml:"mercChickenAt"`
	} `yaml:"health"`
	ChickenOnCurses struct {
		AmplifyDamage bool `yaml:"amplifyDamage"`
		Decrepify     bool `yaml:"decrepify"`
		LowerResist   bool `yaml:"lowerResist"`
		BloodMana     bool `yaml:"bloodMana"`
	} `yaml:"chickenOnCurses"`
	ChickenOnAuras struct {
		Fanaticism bool `yaml:"fanaticism"`
		Might      bool `yaml:"might"`
		Conviction bool `yaml:"conviction"`
		HolyFire   bool `yaml:"holyFire"`
		BlessedAim bool `yaml:"blessedAim"`
		HolyFreeze bool `yaml:"holyFreeze"`
		HolyShock  bool `yaml:"holyShock"`
	} `yaml:"chickenOnAuras"`
	Inventory struct {
		InventoryLock      [][]int     `yaml:"inventoryLock"`
		BeltColumns        BeltColumns `yaml:"beltColumns"`
		HealingPotionCount int         `yaml:"healingPotionCount"`
		ManaPotionCount    int         `yaml:"manaPotionCount"`
		RejuvPotionCount   int         `yaml:"rejuvPotionCount"`
	} `yaml:"inventory"`
	Character struct {
		Class                        string              `yaml:"class"`
		UseMerc                      bool                `yaml:"useMerc"`
		StashToShared                bool                `yaml:"stashToShared"`
		UseTeleport                  bool                `yaml:"useTeleport"`
		ClearPathDist                int                 `yaml:"clearPathDist"`
		ShouldHireAct2MercFrozenAura bool                `yaml:"shouldHireAct2MercFrozenAura"`
		UseExtraBuffs                bool                `yaml:"useExtraBuffs"`
		UseSwapForBuffs              bool                `yaml:"use_swap_for_buffs"`
		BuffOnNewArea                bool                `yaml:"buffOnNewArea"`
		BuffAfterWP                  bool                `yaml:"buffAfterWP"`
		AutoStatSkill                AutoStatSkillConfig `yaml:"autoStatSkill"`
		BerserkerBarb                struct {
			FindItemSwitch              bool `yaml:"find_item_switch"`
			SkipPotionPickupInTravincal bool `yaml:"skip_potion_pickup_in_travincal"`
			UseHowl                     bool `yaml:"use_howl"`
			HowlCooldown                int  `yaml:"howl_cooldown"`
			HowlMinMonsters             int  `yaml:"howl_min_monsters"`
			UseBattleCry                bool `yaml:"use_battlecry"`
			BattleCryCooldown           int  `yaml:"battlecry_cooldown"`
			BattleCryMinMonsters        int  `yaml:"battlecry_min_monsters"`
			HorkNormalMonsters          bool `yaml:"hork_normal_monsters"`
			HorkMonsterCheckRange       int  `yaml:"hork_monster_check_range"`
		} `yaml:"berserker_barb"`
		WhirlwindBarb struct {
			SkipPotionPickupInTravincal bool `yaml:"skip_potion_pickup_in_travincal"`
			HorkNormalMonsters          bool `yaml:"hork_normal_monsters"`
			HorkMonsterCheckRange       int  `yaml:"hork_monster_check_range"`
		} `yaml:"whirlwind_barb"`
		BlizzardSorceress struct {
			UseMoatTrick        bool `yaml:"use_moat_trick"`
			UseStaticOnMephisto bool `yaml:"use_static_on_mephisto"`

			UseBlizzardPackets bool `yaml:"use_blizzard_packets"`
		} `yaml:"blizzard_sorceress"`
		SorceressLeveling struct {
			UseMoatTrick        bool `yaml:"use_moat_trick"`
			UseStaticOnMephisto bool `yaml:"use_static_on_mephisto"`

			UseBlizzardPackets bool `yaml:"use_blizzard_packets"`
			UsePacketLearning  bool `yaml:"use_packet_learning"`
		} `yaml:"sorceress_leveling"`
		BarbLeveling struct {
			UseHowl              bool `yaml:"use_howl"`
			HowlCooldown         int  `yaml:"howl_cooldown"`
			HowlMinMonsters      int  `yaml:"howl_min_monsters"`
			UseBattleCry         bool `yaml:"use_battlecry"`
			BattleCryCooldown    int  `yaml:"battlecry_cooldown"`
			BattleCryMinMonsters int  `yaml:"battlecry_min_monsters"`
			UsePacketLearning    bool `yaml:"use_packet_learning"`
		} `yaml:"barb_leveling"`
		NovaSorceress struct {
			BossStaticThreshold int `yaml:"boss_static_threshold"`

			AggressiveNovaPositioning bool `yaml:"aggressive_nova_positioning"`
		} `yaml:"nova_sorceress"`
		LightningSorceress struct {
		} `yaml:"lightning_sorceress"`
		HydraOrbSorceress struct {
		} `yaml:"hydraorb_sorceress"`
		FireballSorceress struct {
		} `yaml:"fireball_sorceress"`
		MosaicSin struct {
			UseTigerStrike    bool `yaml:"useTigerStrike"`
			UseCobraStrike    bool `yaml:"useCobraStrike"`
			UseClawsOfThunder bool `yaml:"useClawsOfThunder"`
			UseBladesOfIce    bool `yaml:"useBladesOfIce"`
			UseFistsOfFire    bool `yaml:"useFistsOfFire"`
		} `yaml:"mosaic_sin"`
		AssassinLeveling struct {
			UsePacketLearning bool `yaml:"use_packet_learning"`
		} `yaml:"assassin_leveling"`
		AmazonLeveling struct {
			UsePacketLearning bool `yaml:"use_packet_learning"`
		} `yaml:"amazon_leveling"`
		Javazon struct {
			DensityKillerEnabled           bool `yaml:"density_killer_enabled"`
			DensityKillerIgnoreWhitesBelow int  `yaml:"density_killer_ignore_whites_below"`
			// Force a vendor "Repair All" to replenish javelins in town when quantity is below this % threshold.
			// Only applied for the Javazon build when DensityKillerEnabled is true.
			DensityKillerForceRefillBelowPercent int `yaml:"density_killer_force_refill_below_percent"`
		} `yaml:"javazon"`
		DruidLeveling struct {
			UsePacketLearning bool `yaml:"use_packet_learning"`
		} `yaml:"druid_leveling"`
		NecromancerLeveling struct {
			UsePacketLearning bool `yaml:"use_packet_learning"`
		} `yaml:"necromancer_leveling"`
		PaladinLeveling struct {
			UsePacketLearning bool   `yaml:"use_packet_learning"`
			LevelingBuild     string `yaml:"leveling_build"`
		} `yaml:"paladin_leveling"`
		Paladin struct {
			HammerAura               string `yaml:"hammer_aura"`
			FohAura                  string `yaml:"foh_aura"`
			HolyBoltAura             string `yaml:"holy_bolt_aura"`
			MovementAura             string `yaml:"movement_aura"`
			ZealAura                 string `yaml:"zeal_aura"`
			SmiteAura                string `yaml:"smite_aura"`
			UberMephAura             string `yaml:"uber_meph_aura"`
			UseChargeMovement        bool   `yaml:"use_charge_movement"`
			UseRedemptionOnRaisers   bool   `yaml:"use_redemption_on_raisers"`
			UseRedemptionToReplenish bool   `yaml:"use_redemption_to_replenish"`
			RedemptionHpThreshold    int    `yaml:"redemption_hp_threshold"`
			RedemptionManaThreshold  int    `yaml:"redemption_mana_threshold"`
		} `yaml:"paladin"`
		WarcryBarb struct {
			FindItemSwitch              bool `yaml:"find_item_switch"`
			SkipPotionPickupInTravincal bool `yaml:"skip_potion_pickup_in_travincal"`
			UseHowl                     bool `yaml:"use_howl"`
			HowlCooldown                int  `yaml:"howl_cooldown"`
			HowlMinMonsters             int  `yaml:"howl_min_monsters"`
			UseBattleCry                bool `yaml:"use_battlecry"`
			BattleCryCooldown           int  `yaml:"battlecry_cooldown"`
			BattleCryMinMonsters        int  `yaml:"battlecry_min_monsters"`
			UseGrimWard                 bool `yaml:"use_grim_ward"`
			HorkNormalMonsters          bool `yaml:"hork_normal_monsters"`
			HorkMonsterCheckRange       int  `yaml:"hork_monster_check_range"`
			UsePacketLearning           bool `yaml:"use_packet_learning"`
		} `yaml:"warcry_barb"`
	} `yaml:"character"`

	Game struct {
		MinGoldPickupThreshold  int                   `yaml:"minGoldPickupThreshold"`
		UseCainIdentify         bool                  `yaml:"useCainIdentify"`
		DisableIdentifyTome     bool                  `yaml:"disableIdentifyTome"`
		InteractWithShrines     bool                  `yaml:"interactWithShrines"`
		InteractWithChests      bool                  `yaml:"interactWithChests"`
		InteractWithSuperChests bool                  `yaml:"interactWithSuperChests"`
		StopLevelingAt          int                   `yaml:"stopLevelingAt"`
		GameVersion             string                `yaml:"gameVersion"`
		DLCEnabled              bool                  `yaml:"dlcEnabled"`
		IsNonLadderChar         bool                  `yaml:"isNonLadderChar"`
		IsHardCoreChar          bool                  `yaml:"isHardCoreChar"`
		ClearTPArea             bool                  `yaml:"clearTPArea"`
		Difficulty              difficulty.Difficulty `yaml:"difficulty"`
		RandomizeRuns           bool                  `yaml:"randomizeRuns"`
		Runs                    []Run                 `yaml:"runs"`
		CreateLobbyGames        bool                  `yaml:"createLobbyGames"`
		PublicGameCounter       int                   `yaml:"-"`
		MaxFailedMenuAttempts   int                   `yaml:"maxFailedMenuAttempts"`
		Pindleskin              struct {
			SkipOnImmunities []stat.Resist `yaml:"skipOnImmunities"`
		} `yaml:"pindleskin"`
		Cows struct {
			OpenChests bool `yaml:"openChests"`
		} `yaml:"cows"`
		Pit struct {
			MoveThroughBlackMarsh bool `yaml:"moveThroughBlackMarsh"`
			OpenChests            bool `yaml:"openChests"`
			FocusOnElitePacks     bool `yaml:"focusOnElitePacks"`
			OnlyClearLevel2       bool `yaml:"onlyClearLevel2"`
		} `yaml:"pit"`
		Countess struct {
			ClearFloors bool `yaml:"clearFloors"`
		}
		Andariel struct {
			ClearRoom bool `yaml:"clearRoom"`
			// Deprecated: kept for backwards compatibility with older configs; can be removed in the future.
			UseAntidoesDeprecated bool `yaml:"useAntidoes,omitempty"`
			UseAntidotes          bool `yaml:"useAntidotes"`
		}
		Duriel struct {
			UseThawing bool `yaml:"useThawing"`
		}
		StonyTomb struct {
			OpenChests        bool `yaml:"openChests"`
			FocusOnElitePacks bool `yaml:"focusOnElitePacks"`
		} `yaml:"stony_tomb"`
		Mausoleum struct {
			OpenChests        bool `yaml:"openChests"`
			FocusOnElitePacks bool `yaml:"focusOnElitePacks"`
		} `yaml:"mausoleum"`
		AncientTunnels struct {
			OpenChests        bool `yaml:"openChests"`
			FocusOnElitePacks bool `yaml:"focusOnElitePacks"`
		} `yaml:"ancient_tunnels"`
		Summoner struct {
			KillFireEye bool `yaml:"killFireEye"`
		} `yaml:"summoner"`
		DrifterCavern struct {
			OpenChests        bool `yaml:"openChests"`
			FocusOnElitePacks bool `yaml:"focusOnElitePacks"`
		} `yaml:"drifter_cavern"`
		SpiderCavern struct {
			OpenChests        bool `yaml:"openChests"`
			FocusOnElitePacks bool `yaml:"focusOnElitePacks"`
		} `yaml:"spider_cavern"`
		ArachnidLair struct {
			OpenChests        bool `yaml:"openChests"`
			FocusOnElitePacks bool `yaml:"focusOnElitePacks"`
		} `yaml:"arachnid_lair"`
		Mephisto struct {
			KillCouncilMembers bool `yaml:"killCouncilMembers"`
			OpenChests         bool `yaml:"openChests"`
			ExitToA4           bool `yaml:"exitToA4"`
		} `yaml:"mephisto"`
		Tristram struct {
			ClearPortal       bool `yaml:"clearPortal"`
			FocusOnElitePacks bool `yaml:"focusOnElitePacks"`
			OnlyFarmRejuvs    bool `yaml:"onlyFarmRejuvs"`
		} `yaml:"tristram"`
		Nihlathak struct {
			ClearArea bool `yaml:"clearArea"`
		} `yaml:"nihlathak"`
		Diablo struct {
			KillDiablo                    bool `yaml:"killDiablo"`
			StartFromStar                 bool `yaml:"startFromStar"`
			FocusOnElitePacks             bool `yaml:"focusOnElitePacks"`
			DisableItemPickupDuringBosses bool `yaml:"disableItemPickupDuringBosses"`
			AttackFromDistance            int  `yaml:"attackFromDistance"`
		} `yaml:"diablo"`
		Baal struct {
			KillBaal    bool `yaml:"killBaal"`
			DollQuit    bool `yaml:"dollQuit"`
			SoulQuit    bool `yaml:"soulQuit"`
			ClearFloors bool `yaml:"clearFloors"`
			OnlyElites  bool `yaml:"onlyElites"`
		} `yaml:"baal"`
		Eldritch struct {
			KillShenk bool `yaml:"killShenk"`
		} `yaml:"eldritch"`
		LowerKurastChest struct {
			OpenRacks bool `yaml:"openRacks"`
		} `yaml:"lowerkurastchests"`
		TerrorZone struct {
			FocusOnElitePacks bool          `yaml:"focusOnElitePacks"`
			SkipOnImmunities  []stat.Resist `yaml:"skipOnImmunities"`
			SkipOtherRuns     bool          `yaml:"skipOtherRuns"`
			Areas             []area.ID     `yaml:"areas"`
			OpenChests        bool          `yaml:"openChests"`
		} `yaml:"terror_zone"`
		Leveling struct {
			EnsurePointsAllocation   bool     `yaml:"ensurePointsAllocation"`
			EnsureKeyBinding         bool     `yaml:"ensureKeyBinding"`
			AutoEquip                bool     `yaml:"autoEquip"`
			AutoEquipFromSharedStash bool     `yaml:"autoEquipFromSharedStash"`
			EnableRunewordMaker      bool     `yaml:"enableRunewordMaker"`
			NightmareRequiredLevel   int      `yaml:"nightmareRequiredLevel"`
			HellRequiredLevel        int      `yaml:"hellRequiredLevel"`
			HellRequiredFireRes      int      `yaml:"hellRequiredFireRes"`
			HellRequiredLightRes     int      `yaml:"hellRequiredLightRes"`
			EnabledRunewordRecipes   []string `yaml:"enabledRunewordRecipes"`
		} `yaml:"leveling"`
		RunewordMaker struct {
			Enabled              bool     `yaml:"enabled"`
			EnabledRecipes       []string `yaml:"enabledRunewordRecipes"`
			AutoUpgrade          bool     `yaml:"autoUpgrade"`          // Upgrade when better tier base found
			OnlyIfWearable       bool     `yaml:"onlyIfWearable"`       // Only make if character meets str/dex requirements
			AutoTierByDifficulty bool     `yaml:"autoTierByDifficulty"` // Auto-select tier based on difficulty
		} `yaml:"runewordMaker"`
		LevelingSequence struct {
			SequenceFile string `yaml:"sequenceFile"`
		} `yaml:"leveling_sequence"`
		Quests struct {
			ClearDen       bool `yaml:"clearDen"`
			RescueCain     bool `yaml:"rescueCain"`
			RetrieveHammer bool `yaml:"retrieveHammer"`
			GetCube        bool `yaml:"getCube"`
			KillRadament   bool `yaml:"killRadament"`
			RetrieveBook   bool `yaml:"retrieveBook"`
			KillIzual      bool `yaml:"killIzual"`
			KillShenk      bool `yaml:"killShenk"`
			RescueAnya     bool `yaml:"rescueAnya"`
			KillAncients   bool `yaml:"killAncients"`
		} `yaml:"quests"`
		Utility struct {
			ParkingAct int `yaml:"parkingAct"`
		} `yaml:"utility"`
		// RunewordOverrides and RunewordRerollRules are keyed by the display name shown in the UI.
		RunewordOverrides   map[string]RunewordOverrideConfig `yaml:"runewordOverrides,omitempty"`
		RunewordRerollRules map[string][]RunewordRerollRule   `yaml:"runewordRerollRules,omitempty"`
	} `yaml:"game"`
	Companion struct {
		Enabled               bool   `yaml:"enabled"`
		Leader                bool   `yaml:"leader"`
		LeaderName            string `yaml:"leaderName"`
		GameNameTemplate      string `yaml:"gameNameTemplate"`
		GamePassword          string `yaml:"gamePassword"`
		CompanionGameName     string `yaml:"companionGameName"`
		CompanionGamePassword string `yaml:"companionGamePassword"`
	} `yaml:"companion"`
	Gambling struct {
		Enabled bool     `yaml:"enabled"`
		Items   []string `yaml:"items,omitempty"`
	} `yaml:"gambling"`
	Muling struct {
		Enabled      bool     `yaml:"enabled"`
		SwitchToMule string   `yaml:"switchToMule"`
		ReturnTo     string   `yaml:"returnTo"`
		MuleProfiles []string `yaml:"muleProfiles"`
	} `yaml:"muling"`
	MulingState struct {
		CurrentMuleIndex int `yaml:"currentMuleIndex"`
	} `yaml:"mulingState"`
	CubeRecipes struct {
		Enabled              bool     `yaml:"enabled"`
		EnabledRecipes       []string `yaml:"enabledRecipes"`
		SkipPerfectAmethysts bool     `yaml:"skipPerfectAmethysts"`
		SkipPerfectRubies    bool     `yaml:"skipPerfectRubies"`
		JewelsToKeep         int      `yaml:"jewelsToKeep"` // new field: number of magic jewels to keep
		PrioritizeRunewords  bool     `yaml:"prioritizeRunewords"`
	} `yaml:"cubing"`
	BackToTown struct {
		NoHpPotions     bool `yaml:"noHpPotions"`
		NoMpPotions     bool `yaml:"noMpPotions"`
		MercDied        bool `yaml:"mercDied"`
		EquipmentBroken bool `yaml:"equipmentBroken"`
	} `yaml:"backtotown"`
	Shopping ShoppingConfig `yaml:"shopping"`
	Runtime  struct {
		Rules     nip.Rules   `yaml:"-"`
		TierRules []int       `yaml:"-"`
		Drops     []data.Item `yaml:"-"`
	} `yaml:"-"`
}

type BeltColumns [4]string

func GetCharacter(name string) (*CharacterCfg, bool) {
	cfgMux.RLock()
	defer cfgMux.RUnlock()
	charCfg, exists := Characters[name]
	return charCfg, exists
}

func GetCharacters() map[string]*CharacterCfg {
	cfgMux.RLock()
	defer cfgMux.RUnlock()
	copy := make(map[string]*CharacterCfg, len(Characters))
	for k, v := range Characters {
		copy[k] = v
	}
	return copy
}

func (bm BeltColumns) Total(potionType data.PotionType) int {
	typeString := ""
	switch potionType {
	case data.HealingPotion:
		typeString = "healing"
	case data.ManaPotion:
		typeString = "mana"
	case data.RejuvenationPotion:
		typeString = "rejuvenation"
	}

	total := 0
	for _, v := range bm {
		if strings.EqualFold(v, typeString) {
			total++
		}
	}

	return total
}

func Load() error {
	cfgMux.Lock()
	defer cfgMux.Unlock()
	Characters = make(map[string]*CharacterCfg)

	_, err := os.Getwd()
	if err != nil {
		return fmt.Errorf("error getting current working directory: %w", err)
	}

	kooloPath := getAbsPath("config/koolo.yaml")
	r, err := os.Open(kooloPath)
	if err != nil {
		return fmt.Errorf("error loading koolo.yaml: %w", err)
	}
	defer r.Close()

	d := yaml.NewDecoder(r)
	if err = d.Decode(&Koolo); err != nil {
		return fmt.Errorf("error reading config %s: %w", kooloPath, err)
	}
	if Koolo != nil {
		sanitizeDiscordConfig(Koolo)
	}

	configDir := getAbsPath("config")
	entries, err := os.ReadDir(configDir)
	if err != nil {
		return fmt.Errorf("error reading config directory %s: %w", configDir, err)
	}

	for _, entry := range entries {
		if !entry.IsDir() {
			continue
		}

		charCfg := CharacterCfg{}

		charConfigPath := getAbsPath(filepath.Join("config", entry.Name(), "config.yaml"))
		r, err = os.Open(charConfigPath)
		if err != nil {
			return fmt.Errorf("error loading config.yaml: %w", err)
		}

		d := yaml.NewDecoder(r)
		if err = d.Decode(&charCfg); err != nil {
			_ = r.Close()
			return fmt.Errorf("error reading %s character config: %w", charConfigPath, err)
		}
		_ = r.Close()

		// Deprecated: kept for backwards compatibility with older configs; can be removed in the future.
		if !charCfg.Game.Andariel.UseAntidotes && charCfg.Game.Andariel.UseAntidoesDeprecated {
			charCfg.Game.Andariel.UseAntidotes = true
		}
		charCfg.Game.Andariel.UseAntidoesDeprecated = false
		charCfg.Game.GameVersion = NormalizeGameVersion(charCfg.Game.GameVersion)

		charCfg.ConfigFolderName = entry.Name()
		if charCfg.Game.MaxFailedMenuAttempts == 0 {
			charCfg.Game.MaxFailedMenuAttempts = 10
		}

		if len(charCfg.Gambling.Items) == 0 {
			charCfg.Gambling.Items = []string{"coronet", "circlet", "amulet"}
		}

		var pickitPath string
		if Koolo.CentralizedPickitPath != "" && charCfg.UseCentralizedPickit {
			if _, err := os.Stat(Koolo.CentralizedPickitPath); os.IsNotExist(err) {
				utils.ShowDialog("Error loading pickit rules for "+entry.Name(), "The centralized pickit path does not exist: "+Koolo.CentralizedPickitPath+"\nPlease check your Koolo settings.\nFalling back to local pickit.")
				pickitPath = getAbsPath(filepath.Join("config", entry.Name(), "pickit")) + "\\"
			} else {
				pickitPath = Koolo.CentralizedPickitPath + "\\"
			}
		} else {
			pickitPath = getAbsPath(filepath.Join("config", entry.Name(), "pickit")) + "\\"
		}

		rules, err := getCachedRulesDir(pickitPath)
		if err != nil {
			return fmt.Errorf("error reading pickit directory %s: %w", pickitPath, err)
		}

		// Load the leveling pickit rules
		if hasLevelingRun(charCfg.Game.Runs) {
			nips := getLevelingNipFiles(&charCfg, entry.Name())

			for _, nipFile := range nips {
				classRules, err := readSinglePickitFile(nipFile)
				if err != nil {
					return err
				}
				rules = append(rules, classRules...)
			}
		}

		charCfg.Runtime.Rules = rules

		for ruleIndex, rule := range rules {
			if rule.Tier() > 0 || rule.MercTier() > 0 {
				charCfg.Runtime.TierRules = append(charCfg.Runtime.TierRules, ruleIndex)
			}
		}
		Characters[entry.Name()] = &charCfg
	}

	for _, charCfg := range Characters {
		charCfg.Validate()
	}

	return nil
}

func NormalizeGameVersion(version string) string {
	switch strings.ToLower(strings.TrimSpace(version)) {
	case GameVersionReignOfTheWarlock, "reignofthewarlock", "reign of the warlock", "warlock":
		return GameVersionReignOfTheWarlock
	case GameVersionExpansion:
		return GameVersionExpansion
	default:
		return GameVersionReignOfTheWarlock
	}
}

func sanitizeDiscordConfig(cfg *KooloCfg) {
	if !cfg.Discord.Enabled {
		return
	}
	useWebhook := cfg.Discord.UseWebhook
	webhookURL := strings.TrimSpace(cfg.Discord.WebhookURL)
	token := strings.TrimSpace(cfg.Discord.Token)
	channelID := strings.TrimSpace(cfg.Discord.ChannelID)

	if (useWebhook && webhookURL == "") || (!useWebhook && (token == "" || channelID == "")) {
		cfg.Discord.Enabled = false
	}
}

// ClearNIPCache clears the compiled NIP rules cache, forcing recompilation on next load
func ClearNIPCache() {
	nipRulesCacheMux.Lock()
	nipRulesCache = make(map[string]nip.Rules)
	nipRulesCacheMux.Unlock()
}

// getCachedRulesDir returns cached NIP rules for a directory, compiling only if not cached
func getCachedRulesDir(pickitPath string) (nip.Rules, error) {
	nipRulesCacheMux.RLock()
	if cached, ok := nipRulesCache[pickitPath]; ok {
		nipRulesCacheMux.RUnlock()
		return cached, nil
	}
	nipRulesCacheMux.RUnlock()

	// Not cached, compile the rules
	rules, err := nip.ReadDir(pickitPath)
	if err != nil {
		return nil, err
	}

	// Store in cache
	nipRulesCacheMux.Lock()
	nipRulesCache[pickitPath] = rules
	nipRulesCacheMux.Unlock()

	return rules, nil
}

// getCachedRulesFile returns cached NIP rules for a single file, compiling only if not cached
func getCachedRulesFile(filePath string) (nip.Rules, error) {
	nipRulesCacheMux.RLock()
	if cached, ok := nipRulesCache[filePath]; ok {
		nipRulesCacheMux.RUnlock()
		return cached, nil
	}
	nipRulesCacheMux.RUnlock()

	// Not cached, compile via temp directory workaround
	rules, err := readSinglePickitFileUncached(filePath)
	if err != nil {
		return nil, err
	}

	// Store in cache
	nipRulesCacheMux.Lock()
	nipRulesCache[filePath] = rules
	nipRulesCacheMux.Unlock()

	return rules, nil
}

// Helper function to read a single NIP file using the temp directory workaround (uncached)
func readSinglePickitFileUncached(filePath string) (nip.Rules, error) {
	tempDir := filepath.Join(filepath.Dir(filePath), "temp_single_read")
	if err := os.MkdirAll(tempDir, 0755); err != nil {
		return nil, fmt.Errorf("failed to create temp pickit directory: %w", err)
	}
	defer os.RemoveAll(tempDir)

	destFile := filepath.Join(tempDir, filepath.Base(filePath))
	sourceData, err := os.ReadFile(filePath)
	if err != nil {
		return nil, fmt.Errorf("failed to read source pickit file %s: %w", filePath, err)
	}
	if err := os.WriteFile(destFile, sourceData, 0644); err != nil {
		return nil, fmt.Errorf("failed to write to temp pickit file: %w", err)
	}

	rules, err := nip.ReadDir(tempDir + "\\")
	if err != nil {
		return nil, fmt.Errorf("error reading from temp pickit directory %s: %w", tempDir, err)
	}

	return rules, nil
}

// readSinglePickitFile returns cached NIP rules for a single file (for backwards compatibility)
func readSinglePickitFile(filePath string) (nip.Rules, error) {
	return getCachedRulesFile(filePath)
}

func CreateFromTemplate(name string) error {
	if name == "" {
		return errors.New("name cannot be empty")
	}

	if _, err := os.Stat("config/" + name); !os.IsNotExist(err) {
		return errors.New("configuration with that name already exists")
	}

	err := cp.Copy("config/template", "config/"+name)
	if err != nil {
		return fmt.Errorf("error copying template: %w", err)
	}

	return Load()
}

func ValidateAndSaveConfig(config KooloCfg) error {
	config.D2LoDPath = strings.ReplaceAll(strings.ToLower(config.D2LoDPath), "game.exe", "")
	config.D2RPath = strings.ReplaceAll(strings.ToLower(config.D2RPath), "d2r.exe", "")

	if _, err := os.Stat(config.D2LoDPath + "/d2data.mpq"); os.IsNotExist(err) {
		return errors.New("D2LoDPath is not valid")
	}

	if _, err := os.Stat(config.D2RPath + "/d2r.exe"); os.IsNotExist(err) {
		return errors.New("D2RPath is not valid")
	}

	if config.Discord.Enabled {
		sanitizeDiscordConfig(&config)
	}

	text, err := yaml.Marshal(config)
	if err != nil {
		return fmt.Errorf("error parsing koolo config: %w", err)
	}

	err = os.WriteFile("config/koolo.yaml", text, 0644)
	if err != nil {
		return fmt.Errorf("error writing koolo config: %w", err)
	}

	return Load()
}

func SaveKooloConfig(config *KooloCfg) error {
	if config == nil {
		return errors.New("koolo config is nil")
	}
	text, err := yaml.Marshal(config)
	if err != nil {
		return fmt.Errorf("error parsing koolo config: %w", err)
	}
	if err := os.WriteFile("config/koolo.yaml", text, 0644); err != nil {
		return fmt.Errorf("error writing koolo config: %w", err)
	}
	return nil
}

func SaveSupervisorConfig(supervisorName string, config *CharacterCfg) error {
	filePath := filepath.Join("config", supervisorName, "config.yaml")
	d, err := yaml.Marshal(config)
	config.Validate()
	if err != nil {
		return err
	}

	err = os.WriteFile(filePath, d, 0644)
	if err != nil {
		return fmt.Errorf("error writing supervisor config: %w", err)
	}

	return Load()
}

func (c *CharacterCfg) Validate() {
	if c.Character.Class == "nova" || c.Character.Class == "lightsorc" {
		minThreshold := 65 // Default
		switch c.Game.Difficulty {
		case difficulty.Normal:
			minThreshold = 1
		case difficulty.Nightmare:
			minThreshold = 33
		case difficulty.Hell:
			minThreshold = 50
		}
		if c.Character.NovaSorceress.BossStaticThreshold < minThreshold || c.Character.NovaSorceress.BossStaticThreshold > 100 {
			c.Character.NovaSorceress.BossStaticThreshold = minThreshold
		}
	}
}

func getAbsPath(relPath string) string {
	cwd, err := os.Getwd()
	if err != nil {
		//Error should be checked in the Load function before any calls
		return relPath
	}
	return filepath.Join(cwd, relPath)
}

func hasLevelingRun(runs []Run) bool {
	for _, run := range runs {
		runName := strings.ToLower(string(run))
		if strings.Contains(runName, "leveling") || strings.Contains(runName, "leveling_sequence") {
			return true
		}
	}
	return false
}

func getLevelingNipFiles(charCfg *CharacterCfg, entryName string) []string {
	var nips []string
	levelingPickitPath := getAbsPath(filepath.Join("config", entryName, "pickit_leveling"))
	levelingBuildPath := getAbsPath(filepath.Join("config", "template", "builds_leveling"))
	levelingPickitTemplatePath := getAbsPath(filepath.Join("config", "template", "pickit_leveling"))
	classBuildFile := filepath.Join(levelingBuildPath, charCfg.Character.Class+".json")

	if jsonData, err := utils.GetJsonData(classBuildFile); err == nil {
		var buildConfig LevelingBuildConfig
		err = json.Unmarshal(jsonData, &buildConfig)
		if err == nil {
			for _, nip := range buildConfig.Nips {
				nipPath, err := getNipFilePath(levelingPickitPath, levelingPickitTemplatePath, nip)
				if err == nil {
					nips = append(nips, nipPath)
				}
			}
		}
	}

	// No build found, fallback to class pickit
	if len(nips) == 0 {
		nipPath, err := getNipFilePath(levelingPickitPath, levelingPickitTemplatePath, charCfg.Character.Class+".nip")
		if err == nil {
			nips = append(nips, nipPath)
		}
	}

	// Paladin leveling has an extra pickit layer depending on character type.
	if charCfg.Character.Class == "paladin_leveling" {
		paladinLevelingNip := "paladin_leveling_exp.nip"
		if charCfg.Game.DLCEnabled {
			paladinLevelingNip = "paladin_leveling_dlc.nip"
		}

		nipPath, err := getNipFilePath(levelingPickitPath, levelingPickitTemplatePath, paladinLevelingNip)
		if err == nil && !slices.Contains(nips, nipPath) {
			nips = append(nips, nipPath)
		}
	}

	// Quests nip
	questNipPath, err := getNipFilePath(levelingPickitPath, levelingPickitTemplatePath, "quest.nip")
	if err == nil {
		if !slices.Contains(nips, questNipPath) {
			nips = append(nips, questNipPath)
		}
	}

	return nips
}

func getNipFilePath(charPath, templatePath, nipFile string) (string, error) {
	charNipFile := filepath.Join(charPath, nipFile)
	templateNipFile := filepath.Join(templatePath, nipFile)
	if _, err := os.Stat(charNipFile); err == nil {
		return charNipFile, nil
	} else if _, err := os.Stat(templateNipFile); err == nil {
		return templateNipFile, nil
	}
	return nipFile, errors.New("pickit not found")
}
