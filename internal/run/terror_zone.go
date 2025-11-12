package run

import (
	"fmt"
	"math"
	"slices"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/pather"
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

	// Special handling for Arcane Sanctuary
	case area.ArcaneSanctuary:
		return tz.runArcaneSanctuaryTZ()

	case area.ChaosSanctuary:
		return NewDiablo().Run()
	case area.NihlathaksTemple, area.HallsOfAnguish, area.HallsOfPain, area.HallsOfVaught:
		return NewNihlathak().Run()
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
			if k == 0 {
				err := action.WayPoint(tzArea)
				if err != nil {
					return err
				}
			} else {
				err := action.MoveToArea(tzArea)
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

// Logic specific to Arcane Sanctuary TZ
func (tz TerrorZone) runArcaneSanctuaryTZ() error {
	ctx := tz.ctx

	ctx.Logger.Info("Starting Arcane Sanctuary Terror Zone run")

	// Move to Waypoint
	err := action.WayPoint(area.ArcaneSanctuary)
	if err != nil {
		return err
	}

	action.Buff()

	// Initialize Arcane Lane system
	lanes := NewArcaneLanes()

	// Check Summoner's location (kill if found)
	areaData := ctx.Data.Areas[area.ArcaneSanctuary]
	summonerNPC, summonerFound := areaData.NPCs.FindOne(npc.Summoner)

	// Clear all 4 lanes
	for lane := 0; lane < 4; lane++ {
		ctx.Logger.Info("Clearing Arcane Sanctuary TZ - Lane %d/4", lane+1)

		// Clear lane
		err := lanes.ClearLane(tz.customTZEnemyFilter(), summonerNPC, summonerFound)
		if err != nil {
			ctx.Logger.Warn("Failed to clear lane %d: %v", lane+1, err)
			// Continue even on error
		}

		// Open chests (if option is enabled)
		if ctx.CharacterCfg.Game.TerrorZone.OpenChests {
			lanes.OpenChestsAtEnd()
		}

		// Rotate to the next lane (90 degrees)
		lanes.RotateToNextLane()
	}

	ctx.Logger.Info("Arcane Sanctuary Terror Zone run completed")

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

func (tz TerrorZone) tzAreaGroups(firstTZ area.ID) [][]area.ID {
	switch firstTZ {

	// ============================================
	// ACT 1
	// ============================================

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

	// Forgotten Tower
	case area.ForgottenTower, area.TowerCellarLevel1, area.TowerCellarLevel2, area.TowerCellarLevel3, area.TowerCellarLevel4, area.TowerCellarLevel5:
		return [][]area.ID{{area.BlackMarsh, area.ForgottenTower, area.TowerCellarLevel1, area.TowerCellarLevel2, area.TowerCellarLevel3, area.TowerCellarLevel4, area.TowerCellarLevel5}}

	// Barracks / Jail
	case area.Barracks, area.JailLevel1, area.JailLevel2, area.JailLevel3:
		return [][]area.ID{{area.OuterCloister, area.Barracks, area.JailLevel1, area.JailLevel2, area.JailLevel3}}

	// ============================================
	// ACT 2
	// ============================================

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

	// Arcane Sanctuary is now specially handled, so it's removed from here
	// case area.ArcaneSanctuary:
	// 	 	 return [][]area.ID{{area.ArcaneSanctuary}}

	// ============================================
	// ACT 3
	// ============================================

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

	// ============================================
	// ACT 4
	// ============================================

	// Outer Steppes / Plains of Despair
	case area.OuterSteppes, area.PlainsOfDespair:
		return [][]area.ID{{area.ThePandemoniumFortress, area.OuterSteppes, area.PlainsOfDespair}}

	// City of the Damned / River of Flame
	case area.CityOfTheDamned, area.RiverOfFlame:
		return [][]area.ID{{area.CityOfTheDamned, area.RiverOfFlame}}

	// ============================================
	// ACT 5
	// ============================================

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

// ============================================
// Arcane Lanes System
// ============================================

// ArcaneLanes is a struct that manages the 4 lanes of Arcane Sanctuary
type ArcaneLanes struct {
	checkPoints []data.Position
	sequence    []int
	clearRange  int
	ctx         *context.Status
}

// NewArcaneLanes creates a new ArcaneLanes instance
func NewArcaneLanes() *ArcaneLanes {
	return &ArcaneLanes{
		checkPoints: []data.Position{
			{X: 25448, Y: 5448}, // Center Point 0
			// Base Lane Coordinates
			{X: 25544, Y: 5446}, // Start 1
			{X: 25637, Y: 5383}, // Center on Right Lane-a 2
			{X: 25754, Y: 5384}, // Center on Right Lane-b 3
			{X: 25853, Y: 5448}, // End Point 4
			{X: 25637, Y: 5506}, // Center on Left Lane 5
			{X: 25683, Y: 5453}, // Center of Lane 6
		},
		sequence:   []int{1, 2, 6, 3, 4, 5, 1, 0},
		clearRange: 30,
		ctx:        context.Get(),
	}
}

// ClearLane clears the current lane
func (al *ArcaneLanes) ClearLane(filter data.MonsterFilter, summonerNPC data.NPC, summonerFound bool) error {
	for _, idx := range al.sequence {
		// Clear while moving along a safe path
		err := action.ClearThroughPath(
			al.checkPoints[idx],
			al.clearRange,
			filter,
		)
		if err != nil {
			al.ctx.Logger.Debug("ClearThroughPath error at checkpoint %d: %v", idx, err)
			// Continue even on error
		}

		// Check for Summoner at the End Point (idx 4)
		if summonerFound && len(summonerNPC.Positions) > 0 && idx == 4 {
			summonerDistance := pather.DistanceFromPoint(
				al.ctx.Data.PlayerUnit.Position,
				summonerNPC.Positions[0],
			)

			if summonerDistance < 20 {
				al.ctx.Logger.Info("Summoner detected on this lane (distance: %d) - killing...", summonerDistance)
				err := al.ctx.Char.KillSummoner()
				if err != nil {
					al.ctx.Logger.Warn("Failed to kill Summoner: %v", err)
				} else {
					al.ctx.Logger.Info("Summoner killed successfully")
				}
			}
		}
	}
	return nil
}

// RotateToNextLane rotates the coordinates 90 degrees for the next lane
func (al *ArcaneLanes) RotateToNextLane() {
	centerX := float64(al.checkPoints[0].X)
	centerY := float64(al.checkPoints[0].Y)

	for i := 1; i < len(al.checkPoints); i++ {
		al.checkPoints[i] = rotatePoint(
			float64(al.checkPoints[i].X),
			float64(al.checkPoints[i].Y),
			centerX,
			centerY,
			90, // 90 degrees counter-clockwise
		)
	}
}

// OpenChestsAtEnd opens chests at the end of the lane
func (al *ArcaneLanes) OpenChestsAtEnd() {
	laneEndPos := al.checkPoints[4] // End Point

	al.ctx.Logger.Debug("Opening chests near lane end")

	chestsOpened := 0
	for _, obj := range al.ctx.Data.Objects {
		if !obj.Selectable {
			continue
		}

		// Only chests at a reasonable distance from the lane end (5-25 distance)
		distance := pather.DistanceFromPoint(obj.Position, laneEndPos)
		if distance >= 5 && distance <= 25 {
			// Move to chest
			err := action.MoveToCoords(obj.Position)
			if err != nil {
				al.ctx.Logger.Debug("Failed to move to chest: %v", err)
				continue
			}

			// Open chest
			err = action.InteractObject(obj, func() bool {
				chest, found := al.ctx.Data.Objects.FindByID(obj.ID)
				return found && !chest.Selectable
			})

			if err != nil {
				al.ctx.Logger.Debug("Failed to open chest: %v", err)
			} else {
				chestsOpened++
			}
		}
	}

	if chestsOpened > 0 {
		al.ctx.Logger.Debug("Opened %d chests at lane end", chestsOpened)
	}
}

// rotatePoint rotates a point around a center point
func rotatePoint(x, y, centerX, centerY, angle float64) data.Position {
	// Translate to origin
	x -= centerX
	y -= centerY

	// Convert to radians
	radAngle := math.Pi * angle / 180

	// Calculate rotation
	newX := x*math.Cos(radAngle) - y*math.Sin(radAngle)
	newY := x*math.Sin(radAngle) + y*math.Cos(radAngle)

	// Translate back to original position
	return data.Position{
		X: int(math.Ceil(newX)) + int(centerX),
		Y: int(math.Ceil(newY)) + int(centerY),
	}
}
