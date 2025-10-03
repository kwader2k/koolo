package config

import (
	"errors"
	"fmt"
	"path/filepath"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/koolo/internal/utils"

	"os"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	cp "github.com/otiai10/copy"

	"github.com/hectorgimenez/d2go/pkg/nip"

	"gopkg.in/yaml.v3"
)

var (
	Koolo      *KooloCfg
	Characters map[string]*CharacterCfg
	Version    = "dev"
)

type KooloCfg struct {
	Debug struct {
		Log         bool `yaml:"log"`
		Screenshots bool `yaml:"screenshots"`
		RenderMap   bool `yaml:"renderMap"`
	} `yaml:"debug"`
	FirstRun              bool   `yaml:"firstRun"`
	UseCustomSettings     bool   `yaml:"useCustomSettings"`
	GameWindowArrangement bool   `yaml:"gameWindowArrangement"`
	LogSaveDirectory      string `yaml:"logSaveDirectory"`
	D2LoDPath             string `yaml:"D2LoDPath"`
	D2RPath               string `yaml:"D2RPath"`
	CentralizedPickitPath string `yaml:"centralizedPickitPath"`
	Discord               struct {
		Enabled                      bool     `yaml:"enabled"`
		EnableGameCreatedMessages    bool     `yaml:"enableGameCreatedMessages"`
		EnableNewRunMessages         bool     `yaml:"enableNewRunMessages"`
		EnableRunFinishMessages      bool     `yaml:"enableRunFinishMessages"`
		EnableDiscordChickenMessages bool     `yaml:"enableDiscordChickenMessages"`
		EnableDiscordErrorMessages   bool     `yaml:"enableDiscordErrorMessages"`
		BotAdmins                    []string `yaml:"botAdmins"`
		ChannelID                    string   `yaml:"channelId"`
		Token                        string   `yaml:"token"`
	} `yaml:"discord"`
	Telegram struct {
		Enabled bool   `yaml:"enabled"`
		ChatID  int64  `yaml:"chatId"`
		Token   string `yaml:"token"`
	}
}

type Day struct {
	DayOfWeek  int         `yaml:"dayOfWeek"`
	TimeRanges []TimeRange `yaml:"timeRange"`
}

type Scheduler struct {
	Enabled bool  `yaml:"enabled"`
	Days    []Day `yaml:"days"`
}

type TimeRange struct {
	Start time.Time `yaml:"start"`
	End   time.Time `yaml:"end"`
}

type CharacterCfg struct {
	MaxGameLength        int    `yaml:"maxGameLength"`
	Username             string `yaml:"username"`
	Password             string `yaml:"password"`
	AuthMethod           string `yaml:"authMethod"`
	AuthToken            string `yaml:"authToken"`
	Realm                string `yaml:"realm"`
	CharacterName        string `yaml:"characterName"`
	CommandLineArgs      string `yaml:"commandLineArgs"`
	KillD2OnStop         bool   `yaml:"killD2OnStop"`
	ClassicMode          bool   `yaml:"classicMode"`
	CloseMiniPanel       bool   `yaml:"closeMiniPanel"`
	UseCentralizedPickit bool   `yaml:"useCentralizedPickit"`
	HidePortraits        bool   `yaml:"hidePortraits"`

	ConfigFolderName string `yaml:"-"`

	Scheduler Scheduler `yaml:"scheduler"`
	Health    struct {
		HealingPotionAt     int `yaml:"healingPotionAt"`
		ManaPotionAt        int `yaml:"manaPotionAt"`
		RejuvPotionAtLife   int `yaml:"rejuvPotionAtLife"`
		RejuvPotionAtMana   int `yaml:"rejuvPotionAtMana"`
		MercHealingPotionAt int `yaml:"mercHealingPotionAt"`
		MercRejuvPotionAt   int `yaml:"mercRejuvPotionAt"`
		ChickenAt           int `yaml:"chickenAt"`
		MercChickenAt       int `yaml:"mercChickenAt"`
	} `yaml:"health"`
	Inventory struct {
		InventoryLock      [][]int     `yaml:"inventoryLock"`
		BeltColumns        BeltColumns `yaml:"beltColumns"`
		HealingPotionCount int         `yaml:"healingPotionCount"`
		ManaPotionCount    int         `yaml:"manaPotionCount"`
		RejuvPotionCount   int         `yaml:"rejuvPotionCount"`
	} `yaml:"inventory"`
	Character struct {
		Class                        string `yaml:"class"`
		UseMerc                      bool   `yaml:"useMerc"`
		StashToShared                bool   `yaml:"stashToShared"`
		UseTeleport                  bool   `yaml:"useTeleport"`
		ClearPathDist                int    `yaml:"clearPathDist"`
		ShouldHireAct2MercFrozenAura bool   `yaml:"shouldHireAct2MercFrozenAura"`
		BerserkerBarb                struct {
			FindItemSwitch              bool `yaml:"find_item_switch"`
			SkipPotionPickupInTravincal bool `yaml:"skip_potion_pickup_in_travincal"`
		} `yaml:"berserker_barb"`
		NovaSorceress struct {
			BossStaticThreshold int `yaml:"boss_static_threshold"`
		} `yaml:"nova_sorceress"`
		MosaicSin struct {
			UseTigerStrike    bool `yaml:"useTigerStrike"`
			UseCobraStrike    bool `yaml:"useCobraStrike"`
			UseClawsOfThunder bool `yaml:"useClawsOfThunder"`
			UseBladesOfIce    bool `yaml:"useBladesOfIce"`
			UseFistsOfFire    bool `yaml:"useFistsOfFire"`
		} `yaml:"mosaic_sin"`
	} `yaml:"character"`

	Game struct {
		MinGoldPickupThreshold int                   `yaml:"minGoldPickupThreshold"`
		UseCainIdentify        bool                  `yaml:"useCainIdentify"`
		InteractWithShrines    bool                  `yaml:"interactWithShrines"`
		StopLevelingAt         int                   `yaml:"stopLevelingAt"`
		IsNonLadderChar        bool                  `yaml:"isNonLadderChar"`
		ClearTPArea            bool                  `yaml:"clearTPArea"`
		Difficulty             difficulty.Difficulty `yaml:"difficulty"`
		RandomizeRuns          bool                  `yaml:"randomizeRuns"`
		Runs                   []Run                 `yaml:"runs"`
		CreateLobbyGames       bool                  `yaml:"createLobbyGames"`
		PublicGameCounter      int                   `yaml:"-"`
		MaxFailedMenuAttempts  int                   `yaml:"maxFailedMenuAttempts"`
		Pindleskin             struct {
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
			ClearRoom   bool `yaml:"clearRoom"`
			UseAntidoes bool `yaml:"useAntidoes"`
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
			EnabledRunewordRecipes   []string `yaml:"enabledRunewordRecipes"`
		} `yaml:"leveling"`
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
		Enabled bool        `yaml:"enabled"`
		Items   []item.Name `yaml:"items"`
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
	} `yaml:"cubing"`
	BackToTown struct {
		NoHpPotions     bool `yaml:"noHpPotions"`
		NoMpPotions     bool `yaml:"noMpPotions"`
		MercDied        bool `yaml:"mercDied"`
		EquipmentBroken bool `yaml:"equipmentBroken"`
	} `yaml:"backtotown"`
	Runtime struct {
		Rules nip.Rules   `yaml:"-"`
		Drops []data.Item `yaml:"-"`
	} `yaml:"-"`
}

type BeltColumns [4]string

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
	Characters = make(map[string]*CharacterCfg)

	cwd, err := os.Getwd()
	if err != nil {
		return fmt.Errorf("error getting current working directory: %w", err)
	}

	getAbsPath := func(relPath string) string {
		return filepath.Join(cwd, relPath)
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

		charCfg.ConfigFolderName = entry.Name()

		if charCfg.Game.MaxFailedMenuAttempts == 0 {
			charCfg.Game.MaxFailedMenuAttempts = 10
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

		rules, err := nip.ReadDir(pickitPath)
		if err != nil {
			return fmt.Errorf("error reading pickit directory %s: %w", pickitPath, err)
		}

		// Load the leveling pickit rules
		if len(charCfg.Game.Runs) > 0 && charCfg.Game.Runs[0] == "leveling" {
			levelingPickitPath := getAbsPath(filepath.Join("config", entry.Name(), "pickit_leveling"))
			classPickitFile := filepath.Join(levelingPickitPath, charCfg.Character.Class+".nip")
			questPickitFile := filepath.Join(levelingPickitPath, "quest.nip")

			// Try to load the class-specific nip file first
			if _, errStat := os.Stat(classPickitFile); errStat == nil {
				classRules, err := readSinglePickitFile(classPickitFile)
				if err != nil {
					return err
				}
				rules = append(rules, classRules...)
			} else {
				// Fallback: if no class file, load all files EXCEPT quest.nip (to avoid duplicates)
				if _, err := os.Stat(levelingPickitPath); !os.IsNotExist(err) {
					allLevelingFiles, err := os.ReadDir(levelingPickitPath)
					if err != nil {
						return fmt.Errorf("could not read pickit_leveling dir: %w", err)
					}

					// Create a temporary directory for all non-class, non-quest files
					tempDir := filepath.Join(levelingPickitPath, "temp_fallback")
					if err := os.MkdirAll(tempDir, 0755); err == nil {
						for _, file := range allLevelingFiles {
							// Exclude quest.nip since it will be loaded separately
							if file.Name() != "quest.nip" && strings.HasSuffix(file.Name(), ".nip") {
								sourceData, _ := os.ReadFile(filepath.Join(levelingPickitPath, file.Name()))
								os.WriteFile(filepath.Join(tempDir, file.Name()), sourceData, 0644)
							}
						}

						fallbackRules, _ := nip.ReadDir(tempDir + "\\")
						rules = append(rules, fallbackRules...)
						os.RemoveAll(tempDir)
					}
				}
			}

			// Separately, try to load quest.nip and append its rules
			if _, errStat := os.Stat(questPickitFile); errStat == nil {
				questRules, err := readSinglePickitFile(questPickitFile)
				if err != nil {
					return err
				}
				rules = append(rules, questRules...)
			}
		}

		charCfg.Runtime.Rules = rules
		Characters[entry.Name()] = &charCfg
	}

	for _, charCfg := range Characters {
		charCfg.Validate()
	}

	return nil
}

// Helper function to read a single NIP file using the temp directory workaround
func readSinglePickitFile(filePath string) (nip.Rules, error) {
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
