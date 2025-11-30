// internal/action/move.go
package action

import (
	"container/heap"
	"errors"
	"fmt"
	"log/slog"
	"math"
	"slices"
	"sort"
	"strings"
	"time"

	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/utils"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/health"
)

const (
	maxAreaSyncAttempts   = 10
	areaSyncDelay         = 100 * time.Millisecond
	monsterHandleCooldown = 500 * time.Millisecond // Reduced cooldown for more immediate re-engagement
	lootAfterCombatRadius = 25                     // Define a radius for looting after combat
)

var talTombs = []area.ID{area.TalRashasTomb1, area.TalRashasTomb2, area.TalRashasTomb3, area.TalRashasTomb4, area.TalRashasTomb5, area.TalRashasTomb6, area.TalRashasTomb7}

var alwaysTakeShrines = []object.ShrineType{
	object.RefillShrine,
	object.HealthShrine,
	object.ManaShrine,
}

var prioritizedShrines = []struct {
	shrineType object.ShrineType
	state      state.State
}{
	{shrineType: object.ExperienceShrine, state: state.ShrineExperience},
	{shrineType: object.ManaRegenShrine, state: state.ShrineManaRegen},
	{shrineType: object.StaminaShrine, state: state.ShrineStamina},
	{shrineType: object.SkillShrine, state: state.ShrineSkill},
}

var curseBreakingShrines = []object.ShrineType{
	object.ExperienceShrine,
	object.ManaRegenShrine,
	object.StaminaShrine,
	object.SkillShrine,
	object.ArmorShrine,
	object.CombatShrine,
	object.ResistLightningShrine,
	object.ResistFireShrine,
	object.ResistColdShrine,
	object.ResistPoisonShrine,
}

var actTownWaypoints = map[int]area.ID{
	1: area.RogueEncampment,
	2: area.LutGholein,
	3: area.KurastDocks,
	4: area.ThePandemoniumFortress,
	5: area.Harrogath,
}

var (
	ErrArcaneDeadEnd = errors.New("arcane sanctuary dead end")
)

// checkPlayerDeath checks if the player is dead and returns ErrDied if so.
func checkPlayerDeath(ctx *context.Status) error {
	if ctx.Data.PlayerUnit.Area.IsTown() {
		return nil
	}

	if ctx.Data.PlayerUnit.IsDead() {
		return health.ErrDied
	}
	return nil
}

func ensureAreaSync(ctx *context.Status, expectedArea area.ID) error {
	// Skip sync check if we're already in the expected area and have valid area data
	if ctx.Data.PlayerUnit.Area == expectedArea {
		return nil
	}

	// Wait for area data to sync
	for attempts := 0; attempts < maxAreaSyncAttempts; attempts++ {
		ctx.RefreshGameData()

		// Check for death during area sync
		if err := checkPlayerDeath(ctx); err != nil {
			return err
		}

		if ctx.Data.PlayerUnit.Area == expectedArea {
			return nil
		}

		time.Sleep(areaSyncDelay)
	}

	return fmt.Errorf("area sync timeout - expected: %v, current: %v", expectedArea, ctx.Data.PlayerUnit.Area)
}

func findRealTomb(ctx *context.Status) (area.ID, error) {
	var realTomb area.ID

	for _, tomb := range talTombs {
		for _, obj := range ctx.Data.Areas[tomb].Objects {
			if obj.Name == object.HoradricOrifice {
				realTomb = tomb
				break
			}
		}
	}

	if realTomb == 0 {
		return 0, errors.New("failed to find the real Tal Rasha tomb")
	}

	return realTomb, nil
}

func moveToNihlathakTempleFromHarrogath(ctx *context.Status) error {
	if ctx.Data.PlayerUnit.Area != area.Harrogath {
		if err := travelToActTown(ctx, 5); err != nil {
			return err
		}
		ctx.RefreshGameData()
		utils.Sleep(500)
	}
	anyaLocation, found := ctx.Data.Objects.FindOne(object.DrehyaTownStartPosition)
	if !found {
		return errors.New("anya location not found in town")
	}

	if err := MoveToCoords(anyaLocation.Position); err != nil {
		return err
	}

	redPortal, found := ctx.Data.Objects.FindOne(object.PermanentTownPortal)
	if !found {
		// Try to talk to Anya to open portal
		if err := InteractNPC(npc.Drehya); err != nil {
			return err
		}
		utils.Sleep(1000)
		ctx.RefreshGameData()
		utils.Sleep(200)
		redPortal, found = ctx.Data.Objects.FindOne(object.PermanentTownPortal)
		if !found {
			return errors.New("red portal not found in town after talking to Anya")
		}
	}

	err := InteractObject(redPortal, func() bool {
		return ctx.Data.AreaData.Area == area.NihlathaksTemple && ctx.Data.AreaData.IsInside(ctx.Data.PlayerUnit.Position)
	})
	if err != nil {
		return err
	}
	return nil
}

func moveToDurielsLair(ctx *context.Status) error {
	realTomb, tombErr := findRealTomb(ctx)
	if tombErr != nil {
		return tombErr
	}
	if err := MoveToArea(realTomb); err != nil {
		return err
	}
	orifice, found := ctx.Data.Objects.FindOne(object.HoradricOrifice)
	if found {
		if err := MoveToCoords(orifice.Position); err != nil {
			return err
		}
		utils.Sleep(100)
		ctx.RefreshGameData()
		utils.Sleep(500)
		durielslair, found := ctx.Data.Objects.FindOne(object.DurielsLairPortal)
		if !found {
			// try again, sometimes it takes a moment to load..
			ctx.RefreshGameData()
			utils.Sleep(500)
			durielslair, found = ctx.Data.Objects.FindOne(object.DurielsLairPortal)
			if !found {
				return fmt.Errorf("failed to find Duriel's Lair portal")
			}

		}
		if err := MoveToCoords(durielslair.Position); err != nil {
			return err
		}
		if err := InteractObject(durielslair, func() bool {
			return ctx.Data.PlayerUnit.Area == area.DurielsLair && ctx.Data.AreaData.IsInside(ctx.Data.PlayerUnit.Position)
		}); err != nil {
			return err
		}
		return nil

	}
	return fmt.Errorf("failed to move to Duriel's Lair in %s", realTomb.Area().Name)
}

func MoveToArea(dst area.ID) (err error) {
	ctx := context.Get()
	ctx.SetLastAction("MoveToArea")

	// Special handling for Duriel's Lair, need to find the real tomb first
	if dst == area.DurielsLair {
		return moveToDurielsLair(ctx)
	}

	hops := 0
	for {
		if ctx.Data.PlayerUnit.Area == dst {
			return nil
		}

		route, routeErr := buildAreaRoute(ctx, dst)

		routeStrs := make([]string, len(route))
		for i, id := range route {
			routeStrs[i] = id.Area().Name
		}
		ctx.Logger.Debug("Area route", "from", ctx.Data.PlayerUnit.Area.Area().Name, "to", dst.Area().Name, "route", strings.Join(routeStrs, "-> "), "error", routeErr)
		if routeErr != nil {
			err := moveToAreaSingle(ctx, dst, true)
			if err != nil {
				if err := WayPoint(dst); err != nil {
					return fmt.Errorf("failed to move to area %s: waypoint failed: %v", dst.Area().Name, err)
				}
			}

		}

		if len(route) <= 1 {
			// Already in destination area
			ctx.Logger.Debug("Arrived in destination area", "area", dst.Area().Name)
			return nil
		}

		next := route[1]

		if err := moveToAreaSingle(ctx, next, true); err != nil {
			return err
		}

		ctx.RefreshGameData()
		hops++
		if hops > 50 {
			return fmt.Errorf("failed to reach %s after %d hops", dst.Area().Name, hops)
		}
	}
}

func moveToAreaSingle(ctx *context.Status, dst area.ID, allowWaypoints bool) error {
	// Proactive death check at the start of the action
	if err := checkPlayerDeath(ctx); err != nil {
		return err
	}

	if err := ensureAreaSync(ctx, ctx.Data.PlayerUnit.Area); err != nil {
		return err
	}

	// Exceptions for:
	// Arcane Sanctuary <-> Palace Cellar Level 3
	if (dst == area.ArcaneSanctuary && ctx.Data.PlayerUnit.Area == area.PalaceCellarLevel3) || (ctx.Data.PlayerUnit.Area == area.ArcaneSanctuary && dst == area.PalaceCellarLevel3) {
		portal, _ := ctx.Data.Objects.FindOne(object.ArcaneSanctuaryPortal)
		MoveToCoords(portal.Position)

		return step.InteractObject(portal, func() bool {
			return ctx.Data.PlayerUnit.Area == dst
		})
	}
	// Arcane Sanctuary -> Canyon of the Magi
	if dst == area.CanyonOfTheMagi && ctx.Data.PlayerUnit.Area == area.ArcaneSanctuary {
		tome, _ := ctx.Data.Objects.FindOne(object.YetAnotherTome)
		MoveToCoords(tome.Position)
		InteractObject(tome, func() bool {
			if _, found := ctx.Data.Objects.FindOne(object.PermanentTownPortal); found {
				return true
			}
			return false
		})
		portal, _ := ctx.Data.Objects.FindOne(object.PermanentTownPortal)
		MoveToCoords(portal.Position)
		return InteractObject(portal, func() bool {
			return ctx.Data.PlayerUnit.Area == area.CanyonOfTheMagi
		})
	}

	// Nihlathak's Temple -> Harrogath
	if ctx.Data.PlayerUnit.Area == area.NihlathaksTemple && dst == area.Harrogath {
		ctx.Logger.Debug("Leaving Nihlathak's Temple to Harrogath")
		portalPosition := data.Position{X: 10068, Y: 13308}
		MoveToCoords(portalPosition)
		ctx.RefreshGameData()
		utils.Sleep(500)
		portal, found := ctx.Data.Objects.FindOne(object.PermanentTownPortal)
		if !found {
			// try again, sometimes it takes a moment to load..
			ctx.RefreshGameData()
			utils.Sleep(500)
			portal, found = ctx.Data.Objects.FindOne(object.PermanentTownPortal)
			if !found {
				return fmt.Errorf("failed to find Nihlathak's Temple portal to Harrogath")
			}
		}
		MoveToCoords(portal.Position)
		return InteractObject(portal, func() bool {
			return ctx.Data.PlayerUnit.Area == area.Harrogath
		})
	}

	// * -> Nihlathak's Temple
	if dst == area.NihlathaksTemple {
		// Nihlathak temple requires special handling due to the portal interaction
		if ctx.Data.PlayerUnit.Area != area.Harrogath {
			// check if we can go there directly, otherwise go to Harrogath first
			if !slices.Contains(CanReachNihlathakTempleFrom, ctx.Data.PlayerUnit.Area) {
				if err := travelToActTown(ctx, 5); err != nil {
					return err
				}
			} else {
				// move there directly
				if err := moveToAreaSingle(ctx, dst, true); err != nil {
					return err
				}

			}

		}
		// Now we should be in Harrogath, interact with the portal
		return moveToNihlathakTempleFromHarrogath(ctx)
	}

	// Stony Field -> Tristram via permanent portal after quest completion
	if dst == area.Tristram && ctx.Data.PlayerUnit.Area == area.StonyField &&
		ctx.Data.Quests[quest.Act1TheSearchForCain].Completed() {
		return moveToTristramPortal(ctx)
	}

	areaCombinations := func(a, b area.ID) bool {
		return (a == dst && ctx.Data.PlayerUnit.Area == b) || (a == ctx.Data.PlayerUnit.Area && dst == b)
	}

	findLevelData := func() data.Level {
		if areaCombinations(area.Abaddon, area.FrigidHighlands) || areaCombinations(area.PitOfAcheron, area.ArreatPlateau) || areaCombinations(area.FrozenTundra, area.InfernalPit) {
			// it's not mapped correctly, so we hardcode it here
			ctx.RefreshGameData()
			utils.Sleep(500)
			portal, found := ctx.Data.Objects.FindOne(object.PermanentTownPortal)
			if found {
				return data.Level{
					Area:       dst,
					Position:   portal.Position,
					IsEntrance: true,
				}
			}
		}

		for _, a := range ctx.Data.AdjacentLevels {
			if a.Area == dst {
				return a
			}
		}
		return data.Level{}
	}

	triedProxy := false
	var lvl data.Level
	for {
		lvl = findLevelData()
		if lvl.Position.X != 0 || lvl.Position.Y != 0 {
			break
		}

		ctx.Logger.Debug("Destination area not in cache, refreshing data", "area", dst.Area().Name)
		ctx.RefreshGameData()
		lvl = findLevelData()

		if lvl.Position.X != 0 || lvl.Position.Y != 0 {
			break
		}

		if proxy, ok := findProxyArea(ctx, dst); ok && !triedProxy {
			triedProxy = true
			ctx.Logger.Debug("Routing via shared adjacency", "from", ctx.Data.PlayerUnit.Area.Area().Name, "via", proxy.Area().Name, "target", dst.Area().Name)
			if err := MoveToArea(proxy); err != nil {
				return err
			}
			// After moving to the proxy, try again to locate the destination entrance from the new area
			ctx.RefreshGameData()
			utils.Sleep(300)
			continue
		}

		if !allowWaypoints {
			return fmt.Errorf("failed to move to area %s: missing cached data", dst.Area().Name)
		}

		break
	}

	if lvl.Position.X == 0 && lvl.Position.Y == 0 {
		// this means it's not in an adjacent area on the shared grid, navigate there via waypoint
		if allowWaypoints {
			return WayPoint(dst)
		}

		return fmt.Errorf("failed to move to area %s from current area: %s", dst.Area().Name, ctx.Data.PlayerUnit.Area.Area().Name)
	}

	cachedPos := data.Position{}
	if !lvl.IsEntrance && ctx.Data.PlayerUnit.Area != dst {
		objects := ctx.Data.Areas[lvl.Area].Objects
		// Sort objects by the distance from me
		sort.Slice(objects, func(i, j int) bool {
			distanceI := ctx.PathFinder.DistanceFromMe(objects[i].Position)
			distanceJ := ctx.PathFinder.DistanceFromMe(objects[j].Position)

			return distanceI < distanceJ
		})

		// Let's try to find any random object to use as a destination point, once we enter the level we will exit this flow
		for _, obj := range objects {
			_, _, found := ctx.PathFinder.GetPath(obj.Position)
			if found {
				cachedPos = obj.Position
				break
			}
		}

		if cachedPos == (data.Position{}) {
			cachedPos = lvl.Position
		}
	}

	toFun := func() (data.Position, bool) {
		// Check for death during movement target evaluation
		if err := checkPlayerDeath(ctx); err != nil {
			return data.Position{}, false // Signal to stop moving if dead
		}

		if ctx.Data.PlayerUnit.Area == dst {
			ctx.Logger.Debug("Reached area", slog.String("area", dst.Area().Name))
			return data.Position{}, false
		}

		if ctx.Data.PlayerUnit.Area == area.TamoeHighland && dst == area.MonasteryGate {
			ctx.Logger.Debug("Monastery Gate detected, moving to static coords")
			return data.Position{X: 15139, Y: 5056}, true
		}

		if ctx.Data.PlayerUnit.Area == area.MonasteryGate && dst == area.TamoeHighland {
			ctx.Logger.Debug("Monastery Gate detected, moving to static coords")
			return data.Position{X: 15142, Y: 5118}, true
		}

		// To correctly detect the two possible exits from Lut Gholein
		if dst == area.RockyWaste && ctx.Data.PlayerUnit.Area == area.LutGholein {
			if _, _, found := ctx.PathFinder.GetPath(data.Position{X: 5004, Y: 5065}); found {
				return data.Position{X: 4989, Y: 5063}, true
			} else {
				return data.Position{X: 5096, Y: 4997}, true
			}
		}

		// This means it's a cave, we don't want to load the map, just find the entrance and interact
		if lvl.IsEntrance {
			return lvl.Position, true
		}

		return cachedPos, true
	}

	var err error

	// Areas that require a distance override for proper entrance interaction (Tower, Harem, Sewers)
	if dst == area.HaremLevel1 && ctx.Data.PlayerUnit.Area == area.LutGholein ||
		dst == area.SewersLevel3Act2 && ctx.Data.PlayerUnit.Area == area.SewersLevel2Act2 ||
		dst == area.TowerCellarLevel1 && ctx.Data.PlayerUnit.Area == area.ForgottenTower ||
		dst == area.TowerCellarLevel2 && ctx.Data.PlayerUnit.Area == area.TowerCellarLevel1 ||
		dst == area.TowerCellarLevel3 && ctx.Data.PlayerUnit.Area == area.TowerCellarLevel2 ||
		dst == area.TowerCellarLevel4 && ctx.Data.PlayerUnit.Area == area.TowerCellarLevel3 ||
		dst == area.TowerCellarLevel5 && ctx.Data.PlayerUnit.Area == area.TowerCellarLevel4 {
		err = MoveTo(toFun, step.WithDistanceToFinish(7))
	} else {
		err = MoveTo(toFun)
	}

	if err != nil {
		return err
	}

	if lvl.IsEntrance {
		maxAttempts := 3
		for attempt := 0; attempt < maxAttempts; attempt++ {
			// Check current distance
			currentDistance := ctx.PathFinder.DistanceFromMe(lvl.Position)

			if currentDistance > 7 {
				// For distances > 7, recursively call MoveToArea as it includes the entrance interaction
				return MoveToArea(dst)
			} else if currentDistance > 3 && currentDistance <= 7 {
				// For distances between 4 and 7, use direct click
				screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(
					lvl.Position.X-2,
					lvl.Position.Y-2,
				)
				ctx.HID.Click(game.LeftButton, screenX, screenY)
				utils.Sleep(800)
			}

			// Proactive death check before interacting with entrance
			if err := checkPlayerDeath(ctx); err != nil {
				return err
			}

			// Try to interact with the entrance
			err = step.InteractEntrance(dst)
			if err == nil {
				break
			}

			if attempt < maxAttempts-1 {
				ctx.Logger.Debug("Entrance interaction failed, retrying",
					slog.Int("attempt", attempt+1),
					slog.String("error", err.Error()))
				utils.Sleep(1000)
			}
		}

		if err != nil {
			return fmt.Errorf("failed to interact with area %s after %d attempts: %v", dst.Area().Name, maxAttempts, err)
		}

		// Wait for area transition to complete
		if err := ensureAreaSync(ctx, dst); err != nil {
			return err
		}
	}

	// apply buffs after entering a new area if configured
	if ctx.CharacterCfg.Character.BuffOnNewArea {
		Buff()
	}

	event.Send(event.InteractedTo(event.Text(ctx.Name, ""), int(dst), event.InteractionTypeEntrance))
	return nil
}

func travelToActTown(ctx *context.Status, act int) error {
	town, ok := actTownWaypoints[act]
	if !ok {
		return fmt.Errorf("unknown act %d", act)
	}
	if ctx.Data.PlayerUnit.Area == town {
		return nil
	}
	return WayPoint(town)
}

const (
	adjacencyCost             = 1.0
	waypointTeleportPenalty   = 0.1
	waypointNoTeleportPenalty = 1.0
	townPortalPenalty         = 0.5
)

type areaNeighbor struct {
	id   area.ID
	cost float64
}

type areaQueueItem struct {
	area     area.ID
	priority float64
}

type areaQueue []*areaQueueItem

func (pq areaQueue) Len() int { return len(pq) }
func (pq areaQueue) Less(i, j int) bool {
	return pq[i].priority < pq[j].priority
}
func (pq areaQueue) Swap(i, j int) { pq[i], pq[j] = pq[j], pq[i] }
func (pq *areaQueue) Push(x interface{}) {
	*pq = append(*pq, x.(*areaQueueItem))
}
func (pq *areaQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	*pq = old[:n-1]
	return item
}

func buildAreaRoute(ctx *context.Status, dst area.ID) ([]area.ID, error) {
	return buildAreaRouteWithWaypoints(ctx, dst, true)
}

func buildAreaRouteWithWaypoints(ctx *context.Status, dst area.ID, allowWaypoints bool) ([]area.ID, error) {
	if ctx.Data.PlayerUnit.Area == dst {
		return []area.ID{dst}, nil
	}

	start := ctx.Data.PlayerUnit.Area
	path, ok := weightedAreaPath(ctx, start, dst, allowWaypoints)
	if !ok {
		return nil, fmt.Errorf("destination area not found: %s", dst.Area().Name)
	}

	return path, nil
}

func weightedAreaPath(ctx *context.Status, start, goal area.ID, allowWaypoints bool) ([]area.ID, bool) {
	if start == goal {
		return []area.ID{start}, true
	}

	wpMap := map[area.ID]struct{}{}
	if allowWaypoints {
		wpMap = availableWaypointMap(ctx)
	}
	dist := map[area.ID]float64{start: 0}
	prev := make(map[area.ID]area.ID)
	queue := &areaQueue{}
	heap.Init(queue)
	heap.Push(queue, &areaQueueItem{area: start, priority: 0})

	for queue.Len() > 0 {
		current := heap.Pop(queue).(*areaQueueItem)
		if current.area == goal {
			break
		}
		if current.priority > dist[current.area] {
			continue
		}

		for _, neighbor := range areaNeighborsWithCosts(ctx, current.area, wpMap, allowWaypoints) {
			alt := current.priority + neighbor.cost
			if prevCost, ok := dist[neighbor.id]; !ok || alt < prevCost {
				dist[neighbor.id] = alt
				prev[neighbor.id] = current.area
				heap.Push(queue, &areaQueueItem{area: neighbor.id, priority: alt})
			}
		}
	}

	if _, found := dist[goal]; !found {
		return nil, false
	}

	return reconstructAreaPath(prev, start, goal), true
}

func areaNeighborsWithCosts(ctx *context.Status, id area.ID, wpMap map[area.ID]struct{}, allowWaypoints bool) []areaNeighbor {
	dedup := make(map[area.ID]float64)

	for _, neighbor := range collectNeighbors(ctx, id) {
		dedup[neighbor] = adjacencyCost
	}

	if allowWaypoints {
		for _, neighbor := range collectWaypointNeighbors(id, wpMap) {
			cost := waypointTransitionCost(ctx.Data.CanTeleport(), neighbor == actTownWaypoints[id.Act()])
			if existing, ok := dedup[neighbor]; !ok || cost < existing {
				dedup[neighbor] = cost
			}
		}
	}

	out := make([]areaNeighbor, 0, len(dedup))
	for areaID, cost := range dedup {
		out = append(out, areaNeighbor{id: areaID, cost: cost})
	}
	return out
}

func moveToTristramPortal(ctx *context.Status) error {
	if !ctx.Data.Quests[quest.Act1TheSearchForCain].Completed() {
		return fmt.Errorf("quest Search for Cain is not completed, unable to move to Tristram portal")
	}

	if portal, found := ctx.Data.Objects.FindOne(object.PermanentTownPortal); found {
		return enterTristramPortal(ctx, portal)
	}

	ctx.RefreshGameData()

	var stone data.Object
	var found bool
	for _, candidate := range []object.Name{object.CairnStoneAlpha, object.InvisibleObject} {
		stone, found = ctx.Data.Objects.FindOne(candidate)
		if found {
			break
		}
	}

	if !found {
		return fmt.Errorf("failed to locate Tristram portal stone")
	}

	if err := MoveToCoords(stone.Position); err != nil {
		return err
	}

	for attempt := 0; attempt < 3; attempt++ {
		ctx.RefreshGameData()
		if portal, found := ctx.Data.Objects.FindOne(object.PermanentTownPortal); found {
			return enterTristramPortal(ctx, portal)
		}
		utils.Sleep(500)
	}

	return fmt.Errorf("permanent Tristram portal not found after visiting stone")
}

func enterTristramPortal(ctx *context.Status, portal data.Object) error {
	if err := MoveToCoords(portal.Position); err != nil {
		return err
	}
	return InteractObject(portal, func() bool {
		return ctx.Data.PlayerUnit.Area == area.Tristram && ctx.Data.AreaData.IsInside(ctx.Data.PlayerUnit.Position)
	})
}

func collectWaypointNeighbors(id area.ID, wpMap map[area.ID]struct{}) []area.ID {
	destinations := make(map[area.ID]struct{})

	if _, hasWP := wpMap[id]; hasWP {
		for dest := range wpMap {
			if dest == id {
				continue
			}
			destinations[dest] = struct{}{}
		}
	}

	if town, ok := actTownWaypoints[id.Act()]; ok && town != id {
		destinations[town] = struct{}{}
	}

	out := make([]area.ID, 0, len(destinations))
	for dest := range destinations {
		out = append(out, dest)
	}
	return out
}

func availableWaypointMap(ctx *context.Status) map[area.ID]struct{} {
	wpMap := make(map[area.ID]struct{}, len(ctx.Data.PlayerUnit.AvailableWaypoints))
	for _, wp := range ctx.Data.PlayerUnit.AvailableWaypoints {
		wpMap[wp] = struct{}{}
	}
	return wpMap
}

func waypointTransitionCost(canTeleport bool, toTown bool) float64 {
	switch {
	case toTown:
		return adjacencyCost + townPortalPenalty
	case canTeleport:
		return adjacencyCost + waypointTeleportPenalty
	default:
		return adjacencyCost + waypointNoTeleportPenalty
	}
}
func reconstructAreaPath(prev map[area.ID]area.ID, start, goal area.ID) []area.ID {
	path := []area.ID{goal}
	for current := goal; current != start; {
		p, ok := prev[current]
		if !ok {
			break
		}
		path = append([]area.ID{p}, path...)
		current = p
	}
	if path[0] != start {
		path = append([]area.ID{start}, path...)
	}
	return path
}

func collectNeighbors(ctx *context.Status, id area.ID) []area.ID {
	neighbors := make(map[area.ID]struct{})

	if ctx.Data.AreaData.Area == id {
		for _, adj := range ctx.Data.AreaData.AdjacentLevels {
			neighbors[adj.Area] = struct{}{}
		}
	} else if ad, ok := ctx.Data.Areas[id]; ok {
		for _, adj := range ad.AdjacentLevels {
			neighbors[adj.Area] = struct{}{}
		}
	}

	staticAdjacencyMu.RLock()
	for _, adj := range staticAreaAdjacency[id] {
		neighbors[adj] = struct{}{}
	}
	staticAdjacencyMu.RUnlock()

	out := make([]area.ID, 0, len(neighbors))
	for n := range neighbors {
		out = append(out, n)
	}

	return out
}

func findProxyArea(ctx *context.Status, dst area.ID) (area.ID, bool) {
	current := ctx.Data.PlayerUnit.Area
	currNeighbors := collectNeighbors(ctx, current)
	currSet := make(map[area.ID]struct{}, len(currNeighbors))
	for _, n := range currNeighbors {
		currSet[n] = struct{}{}
	}

	dstNeighbors := adjacencyList(dst)
	for _, candidate := range dstNeighbors {
		if candidate == dst || candidate == current {
			continue
		}
		if _, ok := currSet[candidate]; ok {
			return candidate, true
		}
	}
	return 0, false
}

func MoveToCoords(to data.Position, options ...step.MoveOption) error {
	ctx := context.Get()

	// Proactive death check at the start of the action
	if err := checkPlayerDeath(ctx); err != nil {
		return err
	}

	if err := ensureAreaSync(ctx, ctx.Data.PlayerUnit.Area); err != nil {
		return err
	}

	return MoveTo(func() (data.Position, bool) {
		return to, true
	}, options...)
}

func onSafeNavigation() {
	ctx := context.Get()

	if _, isLevelingChar := ctx.Char.(context.LevelingCharacter); isLevelingChar {
		statPoints, hasUnusedPoints := ctx.Data.PlayerUnit.FindStat(stat.StatPoints, 0)
		if hasUnusedPoints && statPoints.Value > 0 {
			ctx.PauseIfNotPriority()
			ctx.DisableItemPickup()
			EnsureSkillPoints()
			EnsureStatPoints()
			EnsureSkillBindings()
			ctx.EnableItemPickup()
		}
		if ctx.HealthManager.IsLowStamina() {
			TryConsumeStaminaPot()
		}
	}

}

func getPathOffsets(to data.Position) (int, int) {
	ctx := context.Get()

	minOffsetX := ctx.Data.AreaData.OffsetX
	minOffsetY := ctx.Data.AreaData.OffsetY

	if !ctx.Data.AreaData.IsInside(to) {
		for _, otherArea := range ctx.Data.AreaData.AdjacentLevels {
			destination := ctx.Data.Areas[otherArea.Area]
			if destination.IsInside(to) {
				minOffsetX = min(minOffsetX, destination.OffsetX)
				minOffsetY = min(minOffsetY, destination.OffsetY)
			}
		}
	}

	return minOffsetX, minOffsetY
}

func MoveTo(toFunc func() (data.Position, bool), options ...step.MoveOption) error {
	ctx := context.Get()
	ctx.SetLastAction("MoveTo")

	// Initialize options
	opts := &step.MoveOpts{}

	// Apply any provided options
	for _, o := range options {
		o(opts)
	}

	minDistanceToFinishMoving := step.DistanceToFinishMoving
	if opts.DistanceToFinish() != nil {
		minDistanceToFinishMoving = *opts.DistanceToFinish()
	}

	// Proactive death check at the start of the action
	if err := checkPlayerDeath(ctx); err != nil {
		return err
	}

	// Ensure no menus are open that might block movement
	for ctx.Data.OpenMenus.IsMenuOpen() {
		ctx.Logger.Debug("Found open menus while moving, closing them...")
		if err := step.CloseAllMenus(); err != nil {
			return err
		}

		utils.Sleep(500)
	}

	clearPathDist := ctx.CharacterCfg.Character.ClearPathDist // Get this once
	overrideClearPathDist := false
	if opts.ClearPathOverride() != nil {
		clearPathDist = *opts.ClearPathOverride()
		overrideClearPathDist = true
	}
	ignoreShrines := !ctx.CharacterCfg.Game.InteractWithShrines
	initialMovementArea := ctx.Data.PlayerUnit.Area
	actionLastMonsterHandlingTime := time.Time{}
	var targetPosition, previousTargetPosition, previousPosition data.Position
	var shrine, chest data.Object
	var pathOffsetX, pathOffsetY int
	var path pather.Path
	var distanceToTarget int
	var pathFound bool
	var stuck bool
	blacklistedInteractions := map[data.UnitID]bool{}
	adjustMinDist := false

	//Arcane sanctuary portal navigation
	var tpPad data.Object
	var blacklistedPads []data.Object

	// Initial sync check
	if err := ensureAreaSync(ctx, ctx.Data.PlayerUnit.Area); err != nil {
		return err
	}

	for {
		ctx.PauseIfNotPriority()
		ctx.RefreshGameData()
		// Check for death after refreshing game data in the loop
		if err := checkPlayerDeath(ctx); err != nil {
			return err
		}

		to, found := toFunc()
		if !found {
			// This covers the case where toFunc itself might return false due to death
			return nil
		}

		targetPosition = to

		//We're not trying to get to town, yet we are. Let bot do his stuff in town and wait to be back on the field
		if !initialMovementArea.IsTown() && ctx.Data.AreaData.Area.IsTown() && !town.IsPositionInTown(targetPosition) {
			utils.Sleep(100)
			continue
		}

		isSafe := true
		if !ctx.Data.AreaData.Area.IsTown() {
			//Safety first, handle enemies
			if !opts.IgnoreMonsters() && (!ctx.Data.CanTeleport() || overrideClearPathDist) && time.Since(actionLastMonsterHandlingTime) > monsterHandleCooldown {
				actionLastMonsterHandlingTime = time.Now()
				filters := opts.MonsterFilters()
				filters = append(filters, func(monsters data.Monsters) (filteredMonsters []data.Monster) {
					for _, m := range monsters {
						if stuck || !ctx.Char.ShouldIgnoreMonster(m) {
							filteredMonsters = append(filteredMonsters, m)
						}
					}
					return filteredMonsters
				})
				_ = ClearAreaAroundPosition(ctx.Data.PlayerUnit.Position, clearPathDist, filters...)
				if !opts.IgnoreItems() {
					// After clearing, immediately try to pick up items
					lootErr := ItemPickup(lootAfterCombatRadius)
					if lootErr != nil {
						ctx.Logger.Warn("Error picking up items after combat", slog.String("error", lootErr.Error()))
					}
				}
			}

			//Check shrine nearby
			if !ignoreShrines && shrine.ID == 0 {
				if closestShrine := findClosestShrine(50.0); closestShrine != nil {
					blacklisted, exists := blacklistedInteractions[closestShrine.ID]
					if !exists || !blacklisted {
						shrine = *closestShrine
						//ctx.Logger.Debug(fmt.Sprintf("MoveTo: Found shrine at %v, redirecting destination from %v", closestShrine.Position, targetPosition))

						//Reset target chest
						chest = (data.Object{})
					}
				}
			}

			//Check chests nearby
			if ctx.CharacterCfg.Game.InteractWithChests && shrine.ID == 0 && chest.ID == 0 {
				if closestChest, chestFound := ctx.PathFinder.GetClosestChest(ctx.Data.PlayerUnit.Position, true); chestFound {
					blacklisted, exists := blacklistedInteractions[closestChest.ID]
					if !exists || !blacklisted {
						chest = *closestChest
						//ctx.Logger.Debug(fmt.Sprintf("MoveTo: Found chest at %v, redirecting destination from %v", chest.Position, targetPosition))
					}
				}
			}

			//Check if we're safe to do some stuff on the field
			if enemyFound, _ := IsAnyEnemyAroundPlayer(max(clearPathDist*2, 30)); !enemyFound {
				onSafeNavigation()
			} else {
				isSafe = false
			}
		}

		//If we have something to interact with, temporarly change target position
		if shrine.ID != 0 {
			targetPosition = shrine.Position
		} else if chest.ID != 0 {
			targetPosition = chest.Position
		} else if !utils.IsZeroPosition(tpPad.Position) {
			targetPosition = tpPad.Position
		}

		//Only recompute path if needed, it can be heavy
		if !utils.IsSamePosition(previousTargetPosition, targetPosition) || !pathFound {
			previousTargetPosition = targetPosition
			path, _, pathFound = ctx.PathFinder.GetPath(targetPosition)
			pathOffsetX, pathOffsetY = getPathOffsets(targetPosition)
		}

		distanceToTarget = ctx.PathFinder.DistanceFromMe(targetPosition)
		if !pathFound {
			//We didn't find a path, try to handle the case
			if ctx.Data.PlayerUnit.Area == area.ArcaneSanctuary {
				//try to go to the end of the tp lane to find target position
				tpPad, err := getArcaneNextTeleportPadPosition(blacklistedPads)
				if err != nil {
					return err
				}
				blacklistedPads = append(blacklistedPads, tpPad)
				continue
			}
			return errors.New("path could not be calculated. Current area: [" + ctx.Data.PlayerUnit.Area.Area().Name + "]. Trying to path to Destination: [" + fmt.Sprintf("%d,%d", to.X, to.Y) + "]")
		}

		//Handle Distance to finish movement
		finishMoveDist := step.DistanceToFinishMoving
		var moveOptions = make([]step.MoveOption, len(options))
		copy(moveOptions, options)
		if minDistanceToFinishMoving != step.DistanceToFinishMoving {
			if utils.IsSamePosition(to, targetPosition) {
				//We don't have any intermediate interactions with objects to do, keep provided parameter
				finishMoveDist = minDistanceToFinishMoving
			} else {
				//Override the parameter with the default value for interactions
				moveOptions = append(moveOptions, step.WithDistanceToFinish(step.DistanceToFinishMoving))
			}
		}

		//We've reached our target destination !
		if distanceToTarget <= finishMoveDist || (adjustMinDist && distanceToTarget <= finishMoveDist*2) {
			if shrine.ID != 0 && targetPosition == shrine.Position {
				//Handle shrine if any
				if err := InteractObject(shrine, func() bool {
					obj, found := ctx.Data.Objects.FindByID(shrine.ID)
					return found && !obj.Selectable
				}); err != nil {
					ctx.Logger.Warn("Failed to interact with shrine", slog.Any("error", err))
				}
				blacklistedInteractions[shrine.ID] = true
				shrine = data.Object{}
				continue
			} else if chest.ID != 0 && targetPosition == chest.Position {
				//Handle chest if any
				if err := InteractObject(chest, func() bool {
					obj, found := ctx.Data.Objects.FindByID(chest.ID)
					return found && !obj.Selectable
				}); err != nil {
					ctx.Logger.Warn("Failed to interact with chest", slog.Any("error", err))
					blacklistedInteractions[chest.ID] = true
				}
				if !opts.IgnoreItems() {
					lootErr := ItemPickup(lootAfterCombatRadius)
					if lootErr != nil {
						ctx.Logger.Warn("Error picking up items after chest opening", slog.String("error", lootErr.Error()))
					}
				}
				chest = data.Object{}
				continue
			} else if !utils.IsZeroPosition(tpPad.Position) && targetPosition == tpPad.Position {
				//Handle arcane sanctuary tp pad if any
				if err := InteractObject(tpPad, func() bool {
					return ctx.PathFinder.DistanceFromMe(tpPad.Position) > 5
				}); err != nil {
					return err
				}
				tpPad = data.Object{}
				exitPad := getClosestTeleportPad(blacklistedPads)
				blacklistedPads = append(blacklistedPads, exitPad)
				continue
			}

			//We've reach the final destination
			return nil
		} else {
			adjustMinDist = false
		}

		//We're not done yet, split the path into smaller segments when outside of town
		nextPosition := targetPosition
		pathStep := 0
		if !ctx.Data.AreaData.Area.IsTown() {
			//Default path step when teleporting
			maxPathStep := 10

			//Restrict path step when walking
			if !ctx.Data.CanTeleport() {
				if isSafe {
					maxPathStep = 8
				} else {
					//baby steps for safety
					maxPathStep = 3
				}
			} else {
				maxPathStep = ctx.PathFinder.GetLastPathIndexOnScreen(path)
			}

			//Pick target position on path and convert path position to global coordinates
			pathStep = min(maxPathStep, len(path)-1)
			nextPathPos := path[pathStep]
			nextPosition = utils.PositionAddCoords(nextPathPos, pathOffsetX, pathOffsetY)
			if pather.DistanceFromPoint(nextPosition, targetPosition) <= minDistanceToFinishMoving {
				nextPosition = targetPosition
			}
		}

		//Do the actual movement...
		moveErr := step.MoveTo(nextPosition, moveOptions...)
		if moveErr != nil {
			//... Reset previous target position to recompute path...
			previousTargetPosition = (data.Position{})

			//... and handle errors if possible
			if errors.Is(moveErr, step.ErrMonstersInPath) {
				continue
			} else if errors.Is(moveErr, step.ErrPlayerStuck) || errors.Is(moveErr, step.ErrPlayerRoundTrip) {
				if (!ctx.Data.CanTeleport() || stuck) || ctx.Data.PlayerUnit.Area.IsTown() {
					ctx.PathFinder.RandomMovement()
					time.Sleep(time.Millisecond * 200)
				}
				stuck = true
				continue
			} else if errors.Is(moveErr, step.ErrNoPath) && pathStep > 0 {
				ctx.PathFinder.RandomMovement()
				time.Sleep(time.Millisecond * 200)
				continue
			}

			//Cannot recover, abort and report error
			return moveErr
		} else if utils.IsSamePosition(previousPosition, ctx.Data.PlayerUnit.Position) {
			adjustMinDist = true
		}

		stuck = false
		previousPosition = ctx.Data.PlayerUnit.Position
		//If we're not in town and moved without errors, move forward in the path
		if !ctx.Data.AreaData.Area.IsTown() {
			path = path[pathStep:]
		}
	}
}

func findClosestShrine(maxScanDistance float64) *data.Object {
	ctx := context.Get()

	// Check if the bot is dead or chickened before proceeding.
	if ctx.Data.PlayerUnit.IsDead() || ctx.Data.PlayerUnit.HPPercent() <= ctx.Data.CharacterCfg.Health.ChickenAt || ctx.Data.AreaData.Area.IsTown() {
		ctx.Logger.Debug("Bot is dead or chickened, skipping shrine search.")
		return nil
	}

	if ctx.Data.AreaData.Area == area.TowerCellarLevel5 {
		return nil
	}

	if ctx.Data.PlayerUnit.States.HasState(state.Amplifydamage) ||
		ctx.Data.PlayerUnit.States.HasState(state.Lowerresist) ||
		ctx.Data.PlayerUnit.States.HasState(state.Decrepify) {

		ctx.Logger.Debug("Curse detected on player. Prioritizing finding any shrine to break it.")

		var closestCurseBreakingShrine *data.Object
		minDistance := maxScanDistance

		for _, o := range ctx.Data.Objects {
			if o.IsShrine() && o.Selectable {
				for _, sType := range curseBreakingShrines {
					if o.Shrine.ShrineType == sType {
						distance := float64(ctx.PathFinder.DistanceFromMe(o.Position))
						if distance < minDistance {
							minDistance = distance
							closestCurseBreakingShrine = &o
						}
					}
				}
			}
		}
		if closestCurseBreakingShrine != nil {
			return closestCurseBreakingShrine
		}
	}

	var closestAlwaysTakeShrine *data.Object
	minDistance := maxScanDistance
	for _, o := range ctx.Data.Objects {
		if o.IsShrine() && o.Selectable {
			for _, sType := range alwaysTakeShrines {
				if o.Shrine.ShrineType == sType {
					if sType == object.HealthShrine && ctx.Data.PlayerUnit.HPPercent() > 95 {
						continue
					}
					if sType == object.ManaShrine && ctx.Data.PlayerUnit.MPPercent() > 95 {
						continue
					}
					if sType == object.RefillShrine && ctx.Data.PlayerUnit.HPPercent() > 95 && ctx.Data.PlayerUnit.MPPercent() > 95 {
						continue
					}

					distance := float64(ctx.PathFinder.DistanceFromMe(o.Position))
					if distance < minDistance {
						minDistance = distance
						closestAlwaysTakeShrine = &o
					}
				}
			}
		}
	}

	if closestAlwaysTakeShrine != nil {
		return closestAlwaysTakeShrine
	}

	var currentPriorityIndex int = -1

	for i, p := range prioritizedShrines {
		if ctx.Data.PlayerUnit.States.HasState(p.state) {
			currentPriorityIndex = i
			break
		}
	}

	for _, o := range ctx.Data.Objects {
		if o.IsShrine() && o.Selectable {
			shrinePriorityIndex := -1
			for i, p := range prioritizedShrines {
				if o.Shrine.ShrineType == p.shrineType {
					shrinePriorityIndex = i
					break
				}
			}

			if shrinePriorityIndex != -1 && (currentPriorityIndex == -1 || shrinePriorityIndex <= currentPriorityIndex) {
				distance := float64(ctx.PathFinder.DistanceFromMe(o.Position))
				if distance < minDistance {
					minDistance = distance
					closestShrine := &o
					return closestShrine
				}
			}
		}
	}

	return nil
}

func getArcaneNextTeleportPadPosition(blacklistedPads []data.Object) (data.Object, error) {
	ctx := context.Get()
	teleportPads := getValidTeleportPads(blacklistedPads)
	var bestPad data.Object
	bestPathDistance := math.MaxInt
	padFound := false

	for _, tpPad := range teleportPads {
		_, distance, found := ctx.PathFinder.GetPath(tpPad.Position)
		//Basic rule to go to the end : go to closest reachable portal
		if found && distance < bestPathDistance {
			bestPad = tpPad
			bestPathDistance = distance
			padFound = true
		}
	}

	if !padFound {
		return bestPad, ErrArcaneDeadEnd
	}
	return bestPad, nil
}

func getClosestTeleportPad(blacklistedPads []data.Object) data.Object {
	ctx := context.Get()
	tpPads := getValidTeleportPads(blacklistedPads)
	var bestPad data.Object
	closestDistance := math.MaxInt

	for _, tpPad := range tpPads {
		distance := ctx.PathFinder.DistanceFromMe(tpPad.Position)
		if distance < closestDistance {
			bestPad = tpPad
			closestDistance = distance
		}
	}

	return bestPad
}

func getValidTeleportPads(blacklistedPads []data.Object) []data.Object {
	ctx := context.Get()
	var teleportPads []data.Object
	for _, obj := range ctx.Data.AreaData.Objects {
		if slices.ContainsFunc(blacklistedPads, func(e data.Object) bool {
			return utils.IsSamePosition(e.Position, obj.Position)
		}) {
			continue
		}
		switch obj.Name {
		case object.TeleportationPad1, object.TeleportationPad2, object.TeleportationPad3, object.TeleportationPad4:
			teleportPads = append(teleportPads, obj)
		}
	}
	return teleportPads
}
