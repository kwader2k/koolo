package terrorzones

import "github.com/hectorgimenez/d2go/pkg/data/area"

// StepKind defines what to do in a route step.
type StepKind int

const (
	StepMove  StepKind = iota // Move to area, do NOT clear
	StepClear                 // Clear area (after moving there)
)

// Step represents one action in a route.
type Step struct {
	Kind StepKind
	Area area.ID
}

// Route is an ordered list of Steps.
type Route []Step

// Beautiful DSL helpers:
func Clear(a area.ID) Step {
	return Step{Kind: StepClear, Area: a}
}

func Move(a area.ID) Step {
	return Step{Kind: StepMove, Area: a}
}

// Routes is the central definition of multi-area TZ runs.
//
// Key   = "primary" terrorized area (ctx.Data.TerrorZones[0])
// Value = one or more routes (alternatives) for that TZ event.
var Routes = map[area.ID][]Route{
	// Act 1
	area.BloodMoor:      {{Move(area.RogueEncampment), Clear(area.BloodMoor), Clear(area.DenOfEvil)}},
	area.ColdPlains:     {{Clear(area.ColdPlains), Clear(area.CaveLevel1), Clear(area.CaveLevel2)}},
	area.BurialGrounds:  {{Move(area.ColdPlains), Clear(area.BurialGrounds), Clear(area.Crypt), Move(area.BurialGrounds), Clear(area.Mausoleum)}},
	area.StonyField:     {{Clear(area.StonyField)}},
	area.DarkWood:       {{Clear(area.DarkWood), Clear(area.UndergroundPassageLevel1), Clear(area.UndergroundPassageLevel2)}},
	area.BlackMarsh:     {{Clear(area.BlackMarsh), Clear(area.HoleLevel1), Clear(area.HoleLevel2)}},
	area.ForgottenTower: {{Move(area.BlackMarsh), Clear(area.ForgottenTower), Clear(area.TowerCellarLevel1), Clear(area.TowerCellarLevel2), Clear(area.TowerCellarLevel3), Clear(area.TowerCellarLevel4), Clear(area.TowerCellarLevel5)}},
	area.Barracks:       {{Move(area.OuterCloister), Clear(area.Barracks), Clear(area.JailLevel1), Clear(area.JailLevel2), Clear(area.JailLevel3)}},
	area.Cathedral:      {{Move(area.InnerCloister), Clear(area.Cathedral), Clear(area.CatacombsLevel1), Clear(area.CatacombsLevel2), Clear(area.CatacombsLevel3), Clear(area.CatacombsLevel4)}},
	//Tristram, Pit, Cows -> terror_zone.go -> NewPit().Run() ect...

	// Act 2
	area.SewersLevel1Act2: {{Move(area.LutGholein), Clear(area.SewersLevel1Act2), Clear(area.SewersLevel2Act2), Clear(area.SewersLevel3Act2)}},
	area.DryHills:         {{Clear(area.DryHills), Clear(area.HallsOfTheDeadLevel1), Clear(area.HallsOfTheDeadLevel2), Clear(area.HallsOfTheDeadLevel3)}},
	area.FarOasis:         {{Clear(area.FarOasis)}},
	area.LostCity:         {{Clear(area.LostCity), Clear(area.ValleyOfSnakes), Clear(area.ClawViperTempleLevel1), Clear(area.ClawViperTempleLevel2)}},
	area.RockyWaste:       {{Move(area.DryHills), Clear(area.RockyWaste), Clear(area.StonyTombLevel1), Clear(area.StonyTombLevel2)}},
	// TalRashasTombs, AncientTunnels, Stonytomb -> terror_zone.go -> NewPit().Run() ect...

	// Act 3
	area.SpiderForest: {{Clear(area.SpiderForest), Clear(area.SpiderCavern)}},
	area.GreatMarsh:   {{Clear(area.GreatMarsh)}},
	area.FlayerJungle: {{Clear(area.FlayerJungle), Clear(area.FlayerDungeonLevel1), Clear(area.FlayerDungeonLevel2), Clear(area.FlayerDungeonLevel3)}},
	area.KurastBazaar: {{Clear(area.KurastBazaar), Clear(area.RuinedTemple), Move(area.KurastBazaar), Clear(area.DisusedFane)}},
	// Travincal, Mephisto -> terror_zone.go -> NewPit().Run() ect...

	// Act 4
	area.OuterSteppes:    {{Move(area.ThePandemoniumFortress), Clear(area.OuterSteppes), Clear(area.PlainsOfDespair)}},
	area.CityOfTheDamned: {{Clear(area.RiverOfFlame), Clear(area.CityOfTheDamned)}},
	// Diablo -> terror_zone.go -> NewPit().Run() ect...

	// Act 5
	area.BloodyFoothills:    {{Move(area.Harrogath), Clear(area.BloodyFoothills), Clear(area.FrigidHighlands), Clear(area.Abaddon)}},
	area.GlacialTrail:       {{Clear(area.GlacialTrail), Clear(area.DrifterCavern)}},
	area.CrystallinePassage: {{Clear(area.CrystallinePassage), Clear(area.FrozenRiver)}},
	area.ArreatPlateau:      {{Clear(area.ArreatPlateau), Clear(area.PitOfAcheron)}},
	area.TheAncientsWay:     {{Clear(area.TheAncientsWay), Clear(area.IcyCellar)}},
	area.NihlathaksTemple:   {{Clear(area.NihlathaksTemple), Clear(area.HallsOfAnguish), Clear(area.HallsOfPain), Clear(area.HallsOfVaught)}},
	// Nihlathak, Baal-> terror_zone.go -> NewPit().Run() ect...
}

// RoutesFor returns all routes for a given primary TZ area.
func RoutesFor(first area.ID) []Route {
	if rs, ok := Routes[first]; ok {
		return rs
	}
	return nil
}
