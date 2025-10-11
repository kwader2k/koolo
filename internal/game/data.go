package game

import (
	"math"
	"sort"
	"strings"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/config"
)

type Data struct {
	Areas    map[area.ID]AreaData `json:"-"`
	AreaData AreaData             `json:"-"`
	data.Data
	CharacterCfg config.CharacterCfg
}

func (d Data) CanTeleport() bool {
	// Check if teleport is generally enabled in character config
	if !d.CharacterCfg.Character.UseTeleport {
		return false
	}

	// Check if player has enough gold
	if d.PlayerUnit.TotalPlayerGold() < 5000 {
		return false
	}

	// Disable teleport if in Arreat Summit and Act 5 Rite of Passage quest is completed
	if d.PlayerUnit.Area == area.ArreatSummit && d.Quests[quest.Act5RiteOfPassage].Completed() {
		return false
	}

	lvl, _ := d.PlayerUnit.FindStat(stat.Level, 0)
	// Disable teleport in Normal difficulty for Act 1 and Act 2, with exceptions
	if d.CharacterCfg.Game.Difficulty == difficulty.Normal && lvl.Value < 24 {
		currentAct := d.PlayerUnit.Area.Act()
		currentAreaID := d.PlayerUnit.Area //

		allowedAct2NormalAreas := map[area.ID]bool{
			area.MaggotLairLevel1:      true,
			area.MaggotLairLevel2:      true,
			area.MaggotLairLevel3:      true,
			area.ArcaneSanctuary:       true,
			area.ClawViperTempleLevel1: true,
			area.ClawViperTempleLevel2: true,
			area.HaremLevel1:           true,
			area.HaremLevel2:           true,
			area.PalaceCellarLevel1:    true,
			area.PalaceCellarLevel2:    true,
			area.PalaceCellarLevel3:    true,
		}

		if currentAct == 1 {
			return false // No teleport in Act 1 Normal
		}

		if currentAct == 2 {
			// Check if the current area is one of the allowed exceptions in Act 2 Normal
			if _, isAllowed := allowedAct2NormalAreas[currentAreaID]; !isAllowed {
				// Check if the area is one of the tombs and the player is level 24 or higher
				tombAreas := map[area.ID]bool{
					area.TalRashasTomb1: true,
					area.TalRashasTomb2: true,
					area.TalRashasTomb3: true,
					area.TalRashasTomb4: true,
					area.TalRashasTomb5: true,
					area.TalRashasTomb6: true,
					area.TalRashasTomb7: true,
				}

				// Safely retrieve the player's level using FindStat, as shown in your example
				lvl, _ := d.PlayerUnit.FindStat(stat.Level, 0)

				if _, isTomb := tombAreas[currentAreaID]; isTomb && lvl.Value >= 24 {
					return true // Allow teleport in tombs if level 24+
				}
				return false // Not an allowed exception, so disallow teleport
			}
		}
	}

	// In Duriel Lair, we can teleport only if Duriel is alive.
	// If Duriel is not found or is dead, teleportation is disallowed.
	if d.PlayerUnit.Area == area.DurielsLair {
		duriel, found := d.Monsters.FindOne(npc.Duriel, data.MonsterTypeUnique)
		// Allow teleport if Duriel is found and his life stat is greater than 0
		if found && duriel.Stats[stat.Life] > 0 {
			return true
		}
		return false // Disallow teleport if Duriel is not found or is dead
	}

	currentManaStat, foundMana := d.PlayerUnit.FindStat(stat.Mana, 0) //
	if !foundMana || currentManaStat.Value < 24 {                     //
		return false
	}

	// Check if the Teleport skill is bound to a key
	_, isTpBound := d.KeyBindings.KeyBindingForSkill(skill.Teleport)

	// Ensure Teleport is bound and the current area is not a town
	return isTpBound && !d.PlayerUnit.Area.IsTown()
}

func (d Data) PlayerCastDuration() time.Duration {
	secs := float64(d.PlayerUnit.CastingFrames())*0.04 + 0.01
	secs = math.Max(0.30, secs)

	return time.Duration(secs*1000) * time.Millisecond
}

func (d Data) MonsterFilterAnyReachable() data.MonsterFilter {
	return func(monsters data.Monsters) (filtered []data.Monster) {
		for _, m := range monsters {
			if d.AreaData.IsWalkable(m.Position) {
				filtered = append(filtered, m)
			}
		}

		return filtered
	}
}

func (d Data) HasPotionInInventory(potionType data.PotionType) bool {
	return len(d.PotionsInInventory(potionType)) > 0
}

func (d Data) PotionsInInventory(potionType data.PotionType) []data.Item {
	items := d.Data.Inventory.ByLocation(item.LocationInventory)
	potions := make([]data.Item, 0)

	for _, i := range items {
		if strings.Contains(string(i.Name), string(potionType)) {
			potions = append(potions, i)
		}
	}

	// Sort potions column-first: leftmost column top-to-bottom, then next column
	sort.Slice(potions, func(i, j int) bool {
		if potions[i].Position.X == potions[j].Position.X {
			return potions[i].Position.Y < potions[j].Position.Y
		}
		return potions[i].Position.X < potions[j].Position.X
	})

	return potions
}

func (d Data) MissingPotionCountInInventory(potionType data.PotionType) int {
	countInInventory := len(d.PotionsInInventory(potionType))
	configuredCount := d.ConfiguredInventoryPotionCount(potionType)

	if countInInventory < configuredCount {
		return configuredCount - countInInventory
	}

	return 0
}

func (d Data) ConfiguredInventoryPotionCount(potionType data.PotionType) int {
	switch potionType {
	case data.HealingPotion:
		return d.CharacterCfg.Inventory.HealingPotionCount
	case data.ManaPotion:
		return d.CharacterCfg.Inventory.ManaPotionCount
	case data.RejuvenationPotion:
		return d.CharacterCfg.Inventory.RejuvPotionCount
	default:
		return 0
	}
}
