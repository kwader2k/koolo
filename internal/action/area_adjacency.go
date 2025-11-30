package action

import (
	"sync"

	"github.com/hectorgimenez/d2go/pkg/data/area"
)

// staticAreaAdjacency captures known transitions derived from Kolbot's tile list
// as well as deterministic overworld links used when live map data is missing.
var staticAreaAdjacency = map[area.ID][]area.ID{
	area.MooMooFarm:      {area.RogueEncampment},
	area.RogueEncampment: {area.BloodMoor, area.MooMooFarm},
	area.BloodMoor:       {area.RogueEncampment, area.ColdPlains, area.DenOfEvil},
	area.ColdPlains:      {area.BloodMoor, area.StonyField, area.CaveLevel1, area.BurialGrounds},
	area.BurialGrounds: {
		area.ColdPlains,
		area.Crypt,
		area.Mausoleum,
	},
	area.Crypt:      {area.BurialGrounds},
	area.Mausoleum:  {area.BurialGrounds},
	area.StonyField: {area.ColdPlains, area.UndergroundPassageLevel1, area.Tristram},
	area.Tristram:   {area.StonyField},
	area.DarkWood:   {area.BlackMarsh, area.UndergroundPassageLevel1},
	area.BlackMarsh: {
		area.DarkWood,
		area.TamoeHighland,
		area.HoleLevel1,
		area.ForgottenTower,
	},
	area.TamoeHighland: {area.BlackMarsh, area.MonasteryGate, area.PitLevel1},
	area.MonasteryGate: {area.TamoeHighland, area.OuterCloister},
	area.OuterCloister: {area.MonasteryGate, area.Barracks},
	area.DenOfEvil:     {area.BloodMoor},
	area.CaveLevel1:    {area.ColdPlains, area.CaveLevel2},
	area.UndergroundPassageLevel1: {
		area.StonyField,
		area.DarkWood,
		area.UndergroundPassageLevel2,
	},
	area.HoleLevel1: {area.BlackMarsh, area.HoleLevel2},
	area.PitLevel1:  {area.TamoeHighland, area.PitLevel2},
	area.CaveLevel2: {area.CaveLevel1},
	area.UndergroundPassageLevel2: {
		area.UndergroundPassageLevel1,
	},
	area.HoleLevel2: {area.HoleLevel1},
	area.PitLevel2:  {area.PitLevel1},
	area.ForgottenTower: {
		area.BlackMarsh,
		area.TowerCellarLevel1,
	},
	area.TowerCellarLevel1: {area.ForgottenTower, area.TowerCellarLevel2},
	area.TowerCellarLevel2: {area.TowerCellarLevel1, area.TowerCellarLevel3},
	area.TowerCellarLevel3: {area.TowerCellarLevel2, area.TowerCellarLevel4},
	area.TowerCellarLevel4: {area.TowerCellarLevel3, area.TowerCellarLevel5},
	area.TowerCellarLevel5: {area.TowerCellarLevel4},
	area.Barracks:          {area.OuterCloister, area.JailLevel1},
	area.JailLevel1:        {area.Barracks, area.JailLevel2},
	area.JailLevel2:        {area.JailLevel1, area.JailLevel3},
	area.JailLevel3:        {area.JailLevel2, area.InnerCloister},
	area.InnerCloister:     {area.JailLevel3, area.Cathedral},
	area.Cathedral:         {area.InnerCloister, area.CatacombsLevel1},
	area.CatacombsLevel1:   {area.Cathedral, area.CatacombsLevel2},
	area.CatacombsLevel2:   {area.CatacombsLevel1, area.CatacombsLevel3},
	area.CatacombsLevel3:   {area.CatacombsLevel2, area.CatacombsLevel4},
	area.CatacombsLevel4:   {area.CatacombsLevel3},
	area.LutGholein: {
		area.RockyWaste,
		area.SewersLevel1Act2,
		area.HaremLevel1,
	},
	area.RockyWaste: {area.LutGholein, area.DryHills, area.StonyTombLevel1},
	area.DryHills:   {area.RockyWaste, area.FarOasis, area.HallsOfTheDeadLevel1},
	area.FarOasis:   {area.DryHills, area.LostCity, area.MaggotLairLevel1},
	area.LostCity: {
		area.FarOasis,
		area.ValleyOfSnakes,
		area.AncientTunnels,
	},
	area.ValleyOfSnakes: {
		area.LostCity,
		area.ClawViperTempleLevel1,
	},
	area.CanyonOfTheMagi: {
		area.TalRashasTomb1,
		area.TalRashasTomb2,
		area.TalRashasTomb3,
		area.TalRashasTomb4,
		area.TalRashasTomb5,
		area.TalRashasTomb6,
		area.TalRashasTomb7,
	},
	area.SewersLevel1Act2:     {area.LutGholein, area.SewersLevel2Act2},
	area.SewersLevel2Act2:     {area.SewersLevel1Act2, area.SewersLevel3Act2},
	area.SewersLevel3Act2:     {area.SewersLevel2Act2},
	area.HaremLevel1:          {area.LutGholein, area.HaremLevel2},
	area.HaremLevel2:          {area.HaremLevel1, area.PalaceCellarLevel1},
	area.PalaceCellarLevel1:   {area.HaremLevel2, area.PalaceCellarLevel2},
	area.PalaceCellarLevel2:   {area.PalaceCellarLevel1, area.PalaceCellarLevel3},
	area.PalaceCellarLevel3:   {area.PalaceCellarLevel2, area.ArcaneSanctuary},
	area.StonyTombLevel1:      {area.RockyWaste, area.StonyTombLevel2},
	area.StonyTombLevel2:      {area.StonyTombLevel1},
	area.HallsOfTheDeadLevel1: {area.DryHills, area.HallsOfTheDeadLevel2},
	area.HallsOfTheDeadLevel2: {area.HallsOfTheDeadLevel1, area.HallsOfTheDeadLevel3},
	area.HallsOfTheDeadLevel3: {area.HallsOfTheDeadLevel2},
	area.ClawViperTempleLevel1: {
		area.ValleyOfSnakes,
		area.ClawViperTempleLevel2,
	},
	area.ClawViperTempleLevel2:  {area.ClawViperTempleLevel1},
	area.MaggotLairLevel1:       {area.FarOasis, area.MaggotLairLevel2},
	area.MaggotLairLevel2:       {area.MaggotLairLevel1, area.MaggotLairLevel3},
	area.MaggotLairLevel3:       {area.MaggotLairLevel2},
	area.AncientTunnels:         {area.LostCity},
	area.TalRashasTomb1:         {area.CanyonOfTheMagi},
	area.TalRashasTomb2:         {area.CanyonOfTheMagi},
	area.TalRashasTomb3:         {area.CanyonOfTheMagi},
	area.TalRashasTomb4:         {area.CanyonOfTheMagi},
	area.TalRashasTomb5:         {area.CanyonOfTheMagi},
	area.TalRashasTomb6:         {area.CanyonOfTheMagi},
	area.TalRashasTomb7:         {area.CanyonOfTheMagi},
	area.ArcaneSanctuary:        {area.PalaceCellarLevel3, area.CanyonOfTheMagi},
	area.DuranceOfHateLevel1:    {area.Travincal, area.DuranceOfHateLevel2},
	area.DuranceOfHateLevel2:    {area.DuranceOfHateLevel1, area.DuranceOfHateLevel3},
	area.DuranceOfHateLevel3:    {area.DuranceOfHateLevel2, area.ThePandemoniumFortress},
	area.KurastDocks:            {area.SpiderForest},
	area.SpiderForest:           {area.KurastDocks, area.GreatMarsh, area.FlayerJungle, area.SpiderCave, area.SpiderCavern},
	area.SpiderCave:             {area.SpiderForest},
	area.SpiderCavern:           {area.SpiderForest},
	area.GreatMarsh:             {area.SpiderForest, area.FlayerJungle},
	area.FlayerJungle:           {area.SpiderForest, area.GreatMarsh, area.LowerKurast, area.SwampyPitLevel1, area.FlayerDungeonLevel1},
	area.SwampyPitLevel1:        {area.FlayerJungle, area.SwampyPitLevel2},
	area.SwampyPitLevel2:        {area.SwampyPitLevel1, area.SwampyPitLevel3},
	area.SwampyPitLevel3:        {area.SwampyPitLevel2},
	area.FlayerDungeonLevel1:    {area.FlayerJungle, area.FlayerDungeonLevel2},
	area.FlayerDungeonLevel2:    {area.FlayerDungeonLevel1, area.FlayerDungeonLevel3},
	area.FlayerDungeonLevel3:    {area.FlayerDungeonLevel2},
	area.LowerKurast:            {area.FlayerJungle, area.KurastBazaar},
	area.KurastBazaar:           {area.LowerKurast, area.UpperKurast, area.RuinedTemple, area.SewersLevel1Act3, area.DisusedFane},
	area.RuinedTemple:           {area.KurastBazaar},
	area.DisusedFane:            {area.KurastBazaar},
	area.SewersLevel1Act3:       {area.KurastBazaar, area.UpperKurast, area.SewersLevel2Act3},
	area.SewersLevel2Act3:       {area.SewersLevel1Act3},
	area.UpperKurast:            {area.KurastBazaar, area.KurastCauseway, area.SewersLevel1Act3, area.ForgottenTemple, area.ForgottenReliquary},
	area.ForgottenTemple:        {area.UpperKurast},
	area.ForgottenReliquary:     {area.UpperKurast},
	area.KurastCauseway:         {area.UpperKurast, area.Travincal, area.RuinedFane, area.DisusedReliquary},
	area.RuinedFane:             {area.KurastCauseway},
	area.DisusedReliquary:       {area.KurastCauseway},
	area.Travincal:              {area.KurastCauseway, area.DuranceOfHateLevel1},
	area.ThePandemoniumFortress: {area.OuterSteppes},
	area.OuterSteppes:           {area.ThePandemoniumFortress, area.PlainsOfDespair},
	area.PlainsOfDespair:        {area.OuterSteppes, area.CityOfTheDamned},
	area.CityOfTheDamned:        {area.PlainsOfDespair, area.RiverOfFlame},
	area.RiverOfFlame:           {area.CityOfTheDamned, area.ChaosSanctuary},
	area.ChaosSanctuary:         {area.RiverOfFlame},
	area.CrystallinePassage: {
		area.ArreatPlateau,
		area.FrozenRiver,
		area.GlacialTrail,
	},
	area.GlacialTrail:     {area.CrystallinePassage, area.DrifterCavern, area.FrozenTundra},
	area.FrozenRiver:      {area.CrystallinePassage},
	area.DrifterCavern:    {area.GlacialTrail},
	area.FrozenTundra:     {area.GlacialTrail, area.TheAncientsWay, area.InfernalPit},
	area.InfernalPit:      {area.FrozenTundra},
	area.TheAncientsWay:   {area.FrozenTundra, area.IcyCellar, area.ArreatSummit},
	area.IcyCellar:        {area.TheAncientsWay},
	area.ArreatSummit:     {area.TheAncientsWay, area.TheWorldStoneKeepLevel1},
	area.Harrogath:        {area.BloodyFoothills, area.NihlathaksTemple},
	area.BloodyFoothills:  {area.Harrogath, area.FrigidHighlands},
	area.FrigidHighlands:  {area.BloodyFoothills, area.ArreatPlateau, area.Abaddon},
	area.Abaddon:          {area.FrigidHighlands},
	area.ArreatPlateau:    {area.FrigidHighlands, area.CrystallinePassage, area.PitOfAcheron},
	area.PitOfAcheron:     {area.ArreatPlateau},
	area.NihlathaksTemple: {area.Harrogath, area.HallsOfAnguish},
	area.HallsOfAnguish:   {area.NihlathaksTemple, area.HallsOfPain},
	area.HallsOfPain:      {area.HallsOfAnguish, area.HallsOfVaught},
	area.HallsOfVaught:    {area.HallsOfPain},
	area.TheWorldStoneKeepLevel1: {
		area.ArreatSummit,
		area.TheWorldStoneKeepLevel2,
	},
	area.TheWorldStoneKeepLevel2: {area.TheWorldStoneKeepLevel1, area.TheWorldStoneKeepLevel3},
	area.TheWorldStoneKeepLevel3: {area.TheWorldStoneKeepLevel2, area.ThroneOfDestruction},
	area.ThroneOfDestruction:     {area.TheWorldStoneKeepLevel3},
}

var CanReachNihlathakTempleFrom = []area.ID{
	area.HallsOfAnguish,
	area.HallsOfPain,
	area.HallsOfVaught,
}

var (
	staticAdjacencyMu sync.RWMutex
)

func adjacencyList(id area.ID) []area.ID {
	staticAdjacencyMu.RLock()
	defer staticAdjacencyMu.RUnlock()

	list := staticAreaAdjacency[id]
	out := make([]area.ID, len(list))
	copy(out, list)
	return out
}

func staticNeighbors(id area.ID) []area.ID {
	staticAdjacencyMu.RLock()
	defer staticAdjacencyMu.RUnlock()

	list := staticAreaAdjacency[id]
	out := make([]area.ID, len(list))
	copy(out, list)
	return out
}

func staticAreaPath(start, goal area.ID) ([]area.ID, bool) {
	if start == goal {
		return []area.ID{start}, true
	}

	queue := []area.ID{start}
	visited := map[area.ID]bool{start: true}
	prev := make(map[area.ID]area.ID)

	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]

		for _, next := range staticNeighbors(current) {
			if visited[next] {
				continue
			}
			visited[next] = true
			prev[next] = current
			if next == goal {
				return buildStaticPath(prev, start, goal), true
			}
			queue = append(queue, next)
		}
	}

	return nil, false
}

func buildStaticPath(prev map[area.ID]area.ID, start, goal area.ID) []area.ID {
	path := []area.ID{goal}
	for current := goal; current != start; {
		p, ok := prev[current]
		if !ok {
			return nil
		}
		path = append([]area.ID{p}, path...)
		current = p
	}
	return path
}
