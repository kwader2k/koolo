package run

import (
	"fmt"
	"slices"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/context"
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

func (tz TerrorZone) Run() error {
	availableTzs := tz.AvailableTZs()
	if len(availableTzs) == 0 {
		return nil
	}

	switch availableTzs[0] {
	case area.PitLevel1, area.PitLevel2:
		return NewPit().Run()

	case area.Tristram:
		t := NewTristram()

		originalClearSetting := t.ctx.CharacterCfg.Game.Tristram.ClearPortal

		defer func() {
			t.ctx.Logger.Debug("[TZ] Restoring original 'ClearPortal' setting")
			t.ctx.CharacterCfg.Game.Tristram.ClearPortal = originalClearSetting
		}()

		t.ctx.Logger.Debug("[TZ] Forcing 'ClearPortal' option for Tristram TZ")
		t.ctx.CharacterCfg.Game.Tristram.ClearPortal = true

		return t.Run()

	case area.Cathedral, area.InnerCloister, area.CatacombsLevel1,
		area.CatacombsLevel2, area.CatacombsLevel3, area.CatacombsLevel4:
		return NewAndariel(tz.customTZEnemyFilter()).Run()
	case area.MooMooFarm:
		return NewCows().Run()
	case area.ForgottenTower, area.TowerCellarLevel1, area.TowerCellarLevel2,
		area.TowerCellarLevel3, area.TowerCellarLevel4, area.TowerCellarLevel5:
		return NewCountess(tz.customTZEnemyFilter()).Run()
	case area.TalRashasTomb1, area.TalRashasTomb2, area.TalRashasTomb3,
		area.TalRashasTomb4, area.TalRashasTomb5, area.TalRashasTomb6, area.TalRashasTomb7:
		return NewTalRashaTombs().Run()
	case area.AncientTunnels:
		return NewAncientTunnels().Run()
	case area.RockyWaste, area.StonyTombLevel1, area.StonyTombLevel2:
		return NewStonyTomb().Run()
	case area.Travincal:
		return NewTravincal().Run()
	case area.DuranceOfHateLevel1, area.DuranceOfHateLevel2, area.DuranceOfHateLevel3:
		return NewMephisto(tz.customTZEnemyFilter()).Run()
	case area.ArcaneSanctuary:
		return NewSummoner(tz.customTZEnemyFilter()).Run()
	case area.ChaosSanctuary:
		return NewDiablo().Run()
	case area.NihlathaksTemple, area.HallsOfAnguish, area.HallsOfPain, area.HallsOfVaught:
		return NewNihlathak(tz.customTZEnemyFilter()).Run()
	case area.TheWorldStoneKeepLevel1, area.TheWorldStoneKeepLevel2,
		area.TheWorldStoneKeepLevel3, area.ThroneOfDestruction,
		area.TheWorldstoneChamber:
		return NewBaal(tz.customTZEnemyFilter()).Run()
	}

	tzAreaGroups := tz.tzAreaGroups(tz.ctx.Data.TerrorZones[0])
	if len(tzAreaGroups) == 0 {
		return nil
	}

	for _, tzAreaGroup := range tzAreaGroups {
		matchingTzInGroup := false
		for _, tzArea := range tzAreaGroup {
			if slices.Contains(availableTzs, tzArea) {
				matchingTzInGroup = true
				break
			}
		}

		if !matchingTzInGroup {
			continue
		}

for k, tzArea := range tzAreaGroup {
    var err error
	
    if k == 0 {
        isWpActive := isWaypointActive(tz.ctx, tzArea) 

        if isWpActive {
            tz.ctx.Logger.Info(fmt.Sprintf("Attempting to use activated Waypoint to %s.", tzArea.Area().Name))
            err = action.WayPoint(tzArea)
            
            if err != nil {
                tz.ctx.Logger.Error(fmt.Sprintf("CRITICAL: Activated Waypoint to %s failed. Aborting run.", tzArea.Area().Name))
                return fmt.Errorf("activated Waypoint failed to %s: %w", tzArea.Area().Name, err)
            }
        } else {
            tz.ctx.Logger.Warn(fmt.Sprintf("Waypoint to %s is not active, attempting to walk/teleport instead.", tzArea.Area().Name))
            err = action.MoveToArea(tzArea)
            
            if err != nil {
                return fmt.Errorf("failed to reach starting TZ area %s by walking: %w", tzArea.Area().Name, err)
            }
        }
        } else {
            err = action.MoveToArea(tzArea)
            if err != nil {
                return err
            }
        }
			if slices.Contains(availableTzs, tzArea) {
				action.ClearCurrentLevel(tz.ctx.CharacterCfg.Game.TerrorZone.OpenChests, tz.customTZEnemyFilter())
			} else {
				tz.ctx.Logger.Debug("Skipping area %v", tzArea.Area().Name)
			}
		}
	}

	return nil
}

// isWaypointActive checks if the destination WayPoint is discovered by the player.
func isWaypointActive(ctx *context.Status, targetArea area.ID) bool {
    return slices.Contains(ctx.Data.PlayerUnit.AvailableWaypoints, targetArea)
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

func (tz TerrorZone) tzAreaGroups(firstTZ area.ID) [][]area.ID {
	switch firstTZ {

	// ACT 1
	// Blood Moor / Den of Evil
	case area.BloodMoor, area.DenOfEvil:
		return [][]area.ID{{area.RogueEncampment, area.BloodMoor, area.DenOfEvil}}

	// Cold Plains / Cave
	case area.ColdPlains, area.CaveLevel1, area.CaveLevel2:
		return [][]area.ID{{area.ColdPlains, area.CaveLevel1, area.CaveLevel2}}

	// Burial Grounds / Crypt / Mausoleum
	case area.BurialGrounds, area.Crypt, area.Mausoleum:
		return [][]area.ID{
			{area.ColdPlains, area.BurialGrounds, area.Crypt},
			{area.ColdPlains, area.BurialGrounds, area.Mausoleum},
		}
	// Stony Field
	case area.StonyField:
		return [][]area.ID{{area.StonyField}}

	// Dark Wood / Underground Passage
	case area.DarkWood, area.UndergroundPassageLevel1, area.UndergroundPassageLevel2:
		return [][]area.ID{{area.DarkWood, area.UndergroundPassageLevel1, area.UndergroundPassageLevel2}}

	// Black Marsh / The Hole
	case area.BlackMarsh, area.HoleLevel1, area.HoleLevel2:
		return [][]area.ID{{area.BlackMarsh, area.HoleLevel1, area.HoleLevel2}}

	// Barracks / Jail
	case area.Barracks, area.JailLevel1, area.JailLevel2, area.JailLevel3:
		return [][]area.ID{{area.OuterCloister, area.Barracks, area.JailLevel1, area.JailLevel2, area.JailLevel3}}


	// ACT 2
	// Lut Gholein Sewers
	case area.SewersLevel1Act2, area.SewersLevel2Act2, area.SewersLevel3Act2:
		return [][]area.ID{{area.LutGholein, area.SewersLevel1Act2, area.SewersLevel2Act2, area.SewersLevel3Act2}}

	// Dry Hills / Halls of the Dead
	case area.DryHills, area.HallsOfTheDeadLevel1, area.HallsOfTheDeadLevel2, area.HallsOfTheDeadLevel3:
		return [][]area.ID{{area.DryHills, area.HallsOfTheDeadLevel1, area.HallsOfTheDeadLevel2, area.HallsOfTheDeadLevel3}}

	// Far Oasis
	case area.FarOasis:
		return [][]area.ID{{area.FarOasis}}

	// Lost City / Valley of Snakes / Claw Viper Temple
	case area.LostCity, area.ValleyOfSnakes, area.ClawViperTempleLevel1, area.ClawViperTempleLevel2:
		return [][]area.ID{{area.LostCity, area.ValleyOfSnakes, area.ClawViperTempleLevel1, area.ClawViperTempleLevel2}}



	// ACT 3
	// Spider Forest / Spider Cavern
	case area.SpiderForest, area.SpiderCavern:
		return [][]area.ID{{area.SpiderForest, area.SpiderCavern}}

	// Great Marsh
	case area.GreatMarsh:
		return [][]area.ID{{area.GreatMarsh}}

	// Flayer Jungle / Flayer Dungeon
	case area.FlayerJungle, area.FlayerDungeonLevel1, area.FlayerDungeonLevel2, area.FlayerDungeonLevel3:
		return [][]area.ID{{area.FlayerJungle, area.FlayerDungeonLevel1, area.FlayerDungeonLevel2, area.FlayerDungeonLevel3}}

	// Kurast Bazaar / Ruined Temple / Disused Fane
	case area.KurastBazaar, area.RuinedTemple, area.DisusedFane:
		return [][]area.ID{
			{area.KurastBazaar, area.RuinedTemple},
			{area.KurastBazaar, area.DisusedFane},
		}

	// ACT 4
	// Outer Steppes / Plains of Despair
	case area.OuterSteppes, area.PlainsOfDespair:
		return [][]area.ID{{area.ThePandemoniumFortress, area.OuterSteppes, area.PlainsOfDespair}}

	// City of the Damned / River of Flame
	case area.CityOfTheDamned, area.RiverOfFlame:
		return [][]area.ID{{area.CityOfTheDamned, area.RiverOfFlame}}


	// ACT 5
	// Bloody Foothills / Frigid Highlands / Abaddon
	case area.BloodyFoothills, area.FrigidHighlands, area.Abaddon:
		return [][]area.ID{{area.Harrogath, area.BloodyFoothills, area.FrigidHighlands, area.Abaddon}}

	// Glacial Trail / Drifter Cavern
	case area.GlacialTrail, area.DrifterCavern:
		return [][]area.ID{{area.GlacialTrail, area.DrifterCavern}}

	// Crystalline Passage / Frozen River
	case area.CrystallinePassage, area.FrozenRiver:
		return [][]area.ID{{area.CrystallinePassage, area.FrozenRiver}}

	// Arreat Plateau / Pit of Acheron
	case area.ArreatPlateau, area.PitOfAcheron:
		return [][]area.ID{{area.ArreatPlateau, area.PitOfAcheron}}

	// Ancient's Way / Icy Cellar
	case area.TheAncientsWay, area.IcyCellar:
		return [][]area.ID{{area.TheAncientsWay, area.IcyCellar}}
	}

	return [][]area.ID{}
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

