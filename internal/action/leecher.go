package action

import (
	"fmt"
	"log/slog"
	"slices"
	"strings"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// GetLeaderPosition returns the position and area of a player from the roster
// Does NOT refresh game data - caller should do that if needed
// Uses case-insensitive name matching since D2R names can have different casing
func GetLeaderPosition(playerName string) (data.Position, area.ID, bool) {
	ctx := context.Get()
	ctx.SetLastAction("GetLeaderPosition")

	playerNameLower := strings.ToLower(playerName)
	for _, member := range ctx.Data.Roster {
		if strings.ToLower(member.Name) == playerNameLower {
			return member.Position, member.Area, true
		}
	}

	return data.Position{}, 0, false
}

// GetLeaderPositionWithRefresh returns the position and area of a player, refreshing game data first
func GetLeaderPositionWithRefresh(playerName string) (data.Position, area.ID, bool) {
	ctx := context.Get()
	ctx.RefreshGameData()
	return GetLeaderPosition(playerName)
}

// GetActFromArea determines which act an area belongs to
func GetActFromArea(a area.ID) int {
	switch {
	case a >= area.RogueEncampment && a <= area.MooMooFarm:
		return 1
	case a >= area.LutGholein && a <= area.ArcaneSanctuary:
		return 2
	case a >= area.KurastDocks && a <= area.DuranceOfHateLevel3:
		return 3
	case a >= area.ThePandemoniumFortress && a <= area.ChaosSanctuary:
		return 4
	case a >= area.Harrogath && a <= area.TheWorldstoneChamber:
		return 5
	default:
		return 0
	}
}

// IsWithinDistance checks if the player is within a certain distance of another player
func IsWithinDistance(playerName string, maxDistance int) bool {
	ctx := context.Get()
	ctx.SetLastAction("IsWithinDistance")

	leaderPos, leaderArea, found := GetLeaderPosition(playerName)
	if !found {
		return false
	}

	// Must be in the same area
	if ctx.Data.PlayerUnit.Area != leaderArea {
		return false
	}

	// Calculate distance
	myPos := ctx.Data.PlayerUnit.Position
	dx := float64(myPos.X - leaderPos.X)
	dy := float64(myPos.Y - leaderPos.Y)
	distance := int(dx*dx + dy*dy)

	return distance <= maxDistance*maxDistance
}

// FollowPlayer moves toward a player while maintaining a safe distance
// The leecher will NOT attack monsters, NOT loot items, and will avoid combat
func FollowPlayer(playerName string, maxDistance int) error {
	ctx := context.Get()
	ctx.SetLastAction("FollowPlayer")

	// Check if bot is stopping to avoid panic in MoveTo
	if ctx.ExecutionPriority == context.PriorityStop {
		return fmt.Errorf("bot is stopping")
	}

	ctx.RefreshGameData()

	leaderPos, leaderArea, found := GetLeaderPosition(playerName)
	if !found {
		return fmt.Errorf("leader %s not found in roster", playerName)
	}

	myPos := ctx.Data.PlayerUnit.Position
	myArea := ctx.Data.PlayerUnit.Area

	// Must be in the same area
	if myArea != leaderArea {
		return fmt.Errorf("not in same area as leader (my area: %s, leader area: %s)",
			myArea.Area().Name, leaderArea.Area().Name)
	}

	// Calculate distance (squared for efficiency)
	dx := myPos.X - leaderPos.X
	dy := myPos.Y - leaderPos.Y
	distanceSquared := dx*dx + dy*dy
	maxDistSq := maxDistance * maxDistance

	// If within range, no need to move
	if distanceSquared <= maxDistSq {
		ctx.Logger.Debug("Following leader: in range",
			slog.String("leader", playerName),
			slog.Int("distanceSq", distanceSquared),
			slog.Int("maxDistSq", maxDistSq))
		return nil
	}

	// Log that we're moving to catch up with detailed position info
	ctx.Logger.Debug("Following leader: moving closer",
		slog.String("leader", playerName),
		slog.Int("myX", myPos.X),
		slog.Int("myY", myPos.Y),
		slog.Int("leaderX", leaderPos.X),
		slog.Int("leaderY", leaderPos.Y),
		slog.Int("distanceSq", distanceSquared))

	// Check again before movement in case priority changed
	if ctx.ExecutionPriority == context.PriorityStop {
		return fmt.Errorf("bot is stopping")
	}

	// Move toward the leader using pathfinding
	// Use WithIgnoreMonsters() to avoid attacking/engaging monsters
	// Use WithIgnoreItems() to avoid looting which would distract from following
	err := MoveToCoords(leaderPos, step.WithIgnoreMonsters(), step.WithIgnoreItems())
	if err != nil {
		ctx.Logger.Warn("Following leader: movement failed",
			slog.String("leader", playerName),
			slog.String("error", err.Error()))
	}
	return err
}

// Screen distance constants for D2R
// 1 screen ≈ 40 game units
const (
	OneScreenDistance    = 40 // ~1 screen in game units
	SafeMonsterDistance  = 40 // Stay at least 1 screen from monsters
	ConsumableLootRadius = 20 // Pick up consumables within ~0.5 screen
)

// FollowPlayerWithLoot follows the leader with priority-based behavior:
// Priority 1: Stay safe distance from enemies (flee if too close - 1 screen / 40 units)
// Priority 2: Stay within 1 screen of the leader for XP
// Priority 3: Pick up consumables (potions, TP scrolls) when safe and in range
// Uses screen-based distances: 1 screen ≈ 40 game units
func FollowPlayerWithLoot(playerName string) error {
	ctx := context.Get()
	ctx.SetLastAction("FollowPlayerWithLoot")

	if ctx.ExecutionPriority == context.PriorityStop {
		return fmt.Errorf("bot is stopping")
	}

	ctx.RefreshGameData()

	leaderPos, leaderArea, found := GetLeaderPosition(playerName)
	if !found {
		return fmt.Errorf("leader %s not found in roster", playerName)
	}

	myPos := ctx.Data.PlayerUnit.Position
	myArea := ctx.Data.PlayerUnit.Area

	if myArea != leaderArea {
		return fmt.Errorf("not in same area as leader")
	}

	// Calculate distance to leader (squared for efficiency)
	dx := myPos.X - leaderPos.X
	dy := myPos.Y - leaderPos.Y
	distToLeaderSq := dx*dx + dy*dy

	// Use squared distances for efficiency
	safeDistSq := SafeMonsterDistance * SafeMonsterDistance
	maxDistSq := OneScreenDistance * OneScreenDistance // Stay within 1 screen of leader

	// PRIORITY 1: Check if any monsters are too close - flee AWAY from monster
	closestMonsterDistSq, monsterPos := findClosestMonsterDistanceAndPos(myPos)
	if closestMonsterDistSq < safeDistSq {
		// Calculate approximate actual distance for logging
		approxDist := 0
		for d := 0; d*d <= closestMonsterDistSq; d++ {
			approxDist = d
		}
		ctx.Logger.Debug("Leecher: monster too close, fleeing away",
			slog.Int("monsterDist", approxDist),
			slog.Int("safeDist", SafeMonsterDistance))

		// Calculate flee position - move away from the monster
		fleePos := calculateFleePosition(myPos, monsterPos)
		return MoveToCoords(fleePos, step.WithIgnoreMonsters(), step.WithIgnoreItems())
	}

	// PRIORITY 2: Stay within 1 screen of leader
	if distToLeaderSq > maxDistSq {
		ctx.Logger.Debug("Leecher: too far from leader, moving closer",
			slog.Int("distToLeaderSq", distToLeaderSq),
			slog.Int("maxDistSq", maxDistSq))
		// Move toward leader
		return MoveToCoords(leaderPos, step.WithIgnoreMonsters(), step.WithIgnoreItems())
	}

	// PRIORITY 3: Pick up consumables if safe and in range
	consumable, found := findNearbyConsumable(myPos, ConsumableLootRadius)
	if found {
		// Double-check safety before looting
		consumableMonsterDistSq, _ := findClosestMonsterDistanceAndPos(consumable.Position)
		if consumableMonsterDistSq >= safeDistSq {
			ctx.Logger.Debug("Leecher: picking up consumable",
				slog.String("item", string(consumable.Name)),
				slog.Int("x", consumable.Position.X),
				slog.Int("y", consumable.Position.Y))

			// Move to item and pick it up
			if err := MoveToCoords(consumable.Position, step.WithIgnoreMonsters()); err != nil {
				return err
			}
			return step.PickupItem(consumable, 1)
		}
	}

	// All priorities satisfied, stay in place
	return nil
}

// calculateFleePosition calculates a position to flee to, directly away from the monster
// The flee distance is normalized to SafeMonsterDistance units
func calculateFleePosition(myPos, monsterPos data.Position) data.Position {
	// Vector from monster to me (flee direction)
	fleeX := myPos.X - monsterPos.X
	fleeY := myPos.Y - monsterPos.Y

	// If monster is on top of us, pick an arbitrary direction
	if fleeX == 0 && fleeY == 0 {
		return data.Position{X: myPos.X + SafeMonsterDistance, Y: myPos.Y}
	}

	// Calculate the distance (approximate sqrt using integer math)
	distSq := fleeX*fleeX + fleeY*fleeY
	dist := 1
	for dist*dist < distSq {
		dist++
	}

	// Normalize and scale to SafeMonsterDistance
	if dist > 0 {
		fleeX = (fleeX * SafeMonsterDistance) / dist
		fleeY = (fleeY * SafeMonsterDistance) / dist
	}

	return data.Position{X: myPos.X + fleeX, Y: myPos.Y + fleeY}
}

// findClosestMonsterDistanceAndPos returns the squared distance to the closest monster and its position
func findClosestMonsterDistanceAndPos(pos data.Position) (int, data.Position) {
	ctx := context.Get()
	closestDistSq := 999999
	closestPos := data.Position{}

	for _, m := range ctx.Data.Monsters {
		// Skip dead monsters
		if m.Stats[stat.Life] <= 0 {
			continue
		}

		// Skip friendly NPCs (town NPCs have MonsterType = 0)
		if m.Type == data.MonsterTypeNone {
			continue
		}

		dx := pos.X - m.Position.X
		dy := pos.Y - m.Position.Y
		distSq := dx*dx + dy*dy

		if distSq < closestDistSq {
			closestDistSq = distSq
			closestPos = m.Position
		}
	}

	return closestDistSq, closestPos
}

// findNearbyConsumable finds the closest consumable (potion or TP scroll) within radius
func findNearbyConsumable(pos data.Position, radius int) (data.Item, bool) {
	ctx := context.Get()
	radiusSq := radius * radius

	var closestItem data.Item
	closestDistSq := radiusSq + 1
	found := false

	// Check for potions and TP scrolls on the ground
	for _, itm := range ctx.Data.Inventory.ByLocation(item.LocationGround) {
		// Only pick up consumables
		if !isLeecherConsumable(itm) {
			continue
		}

		dx := pos.X - itm.Position.X
		dy := pos.Y - itm.Position.Y
		distSq := dx*dx + dy*dy

		if distSq <= radiusSq && distSq < closestDistSq {
			closestItem = itm
			closestDistSq = distSq
			found = true
		}
	}

	return closestItem, found
}

// isLeecherConsumable returns true if the item is a consumable the leecher should pick up
func isLeecherConsumable(itm data.Item) bool {
	ctx := context.Get()

	// TP scrolls - always pick up if we need them
	if itm.Name == item.ScrollOfTownPortal {
		// Check if we need TP scrolls
		if town.ShouldBuyTPs() {
			return true
		}
	}

	// Potions - pick up if belt/inventory needs them
	if itm.IsPotion() {
		if itm.IsHealingPotion() {
			missing := ctx.BeltManager.GetMissingCount(data.HealingPotion)
			return missing > 0
		}
		if itm.IsManaPotion() {
			missing := ctx.BeltManager.GetMissingCount(data.ManaPotion)
			return missing > 0
		}
		if itm.IsRejuvPotion() {
			missing := ctx.BeltManager.GetMissingCount(data.RejuvenationPotion)
			return missing > 0
		}
	}

	return false
}

// getTownWaypointForAct returns a waypoint area ID that can be used to travel to an act's town
func getTownWaypointForAct(act int) area.ID {
	switch act {
	case 1:
		return area.RogueEncampment
	case 2:
		return area.LutGholein
	case 3:
		return area.KurastDocks
	case 4:
		return area.ThePandemoniumFortress
	case 5:
		return area.Harrogath
	default:
		return area.RogueEncampment
	}
}

// hasWaypointForAct checks if the player has a waypoint that can reach the target act
// It checks for the town waypoint directly, or any waypoint in that act
func hasWaypointForAct(act int) bool {
	ctx := context.Get()
	targetTown := getTownWaypointForAct(act)

	// Check if we have the town waypoint directly
	if slices.Contains(ctx.Data.PlayerUnit.AvailableWaypoints, targetTown) {
		return true
	}

	// Check if we have any waypoint in the target act
	// If so, WayPoint() can use it as a starting point
	for _, wp := range ctx.Data.PlayerUnit.AvailableWaypoints {
		wpAct := GetActFromArea(wp)
		if wpAct == act {
			return true
		}
	}

	return false
}

// GoToLeaderAct navigates to the town of the leader's act
// Uses the already-known leader area instead of re-looking it up
func GoToLeaderAct(playerName string) error {
	ctx := context.Get()
	ctx.SetLastAction("GoToLeaderAct")

	// Check if bot is stopping to avoid panic
	if ctx.ExecutionPriority == context.PriorityStop {
		return fmt.Errorf("bot is stopping")
	}

	// Get leader info without refreshing (should already be fresh from caller)
	_, leaderArea, found := GetLeaderPosition(playerName)
	if !found {
		// If not found, the leader might have left briefly - just stay in current act
		ctx.Logger.Debug("Leader not found in roster for act navigation, staying in current act")
		return nil
	}

	leaderAct := GetActFromArea(leaderArea)
	if leaderAct == 0 {
		ctx.Logger.Debug("Could not determine leader's act from area", slog.Int("leaderArea", int(leaderArea)))
		return nil
	}

	currentAct := GetActFromArea(ctx.Data.PlayerUnit.Area)
	if currentAct == leaderAct {
		// Already in the right act
		ctx.Logger.Debug("Already in leader's act", slog.Int("act", currentAct))
		return nil
	}

	targetTown := getTownWaypointForAct(leaderAct)

	// Check if we have a waypoint to this act
	if !hasWaypointForAct(leaderAct) {
		ctx.Logger.Warn("No waypoint available for leader's act",
			slog.Int("leaderAct", leaderAct))
		return fmt.Errorf("no waypoint available for act %d", leaderAct)
	}

	// Use waypoint to travel
	return WayPoint(targetTown)
}

// GoToAct travels to the specified act using waypoints
func GoToAct(targetAct int) error {
	ctx := context.Get()
	ctx.SetLastAction("GoToAct")

	// Check if bot is stopping to avoid panic
	if ctx.ExecutionPriority == context.PriorityStop {
		return fmt.Errorf("bot is stopping")
	}

	currentAct := GetActFromArea(ctx.Data.PlayerUnit.Area)
	if currentAct == targetAct {
		ctx.Logger.Debug("Already in target act", slog.Int("act", currentAct))
		return nil
	}

	ctx.Logger.Info("Navigating to act",
		slog.Int("currentAct", currentAct),
		slog.Int("targetAct", targetAct))

	// Before checking waypoint availability, we need to populate the AvailableWaypoints data
	// by actually opening the waypoint menu in our current town
	ctx.Logger.Debug("Opening waypoint menu to refresh available waypoints")
	if err := openWaypointMenuToRefreshData(); err != nil {
		ctx.Logger.Warn("Failed to open waypoint menu, continuing anyway",
			slog.String("error", err.Error()))
	}

	// Refresh game data after opening/closing menu
	ctx.RefreshGameData()

	ctx.Logger.Debug("Available waypoints after menu open",
		slog.Int("count", len(ctx.Data.PlayerUnit.AvailableWaypoints)))

	targetTown := getTownWaypointForAct(targetAct)

	// Check if we have a waypoint to this act
	if hasWaypointForAct(targetAct) {
		ctx.Logger.Info("Using waypoint to travel to act",
			slog.Int("targetAct", targetAct))
		return WayPoint(targetTown)
	}

	// No waypoint available - check for special transitions
	// Special case: Act 4 → Act 5 can use red portal if waypoint not available
	if currentAct == 4 && targetAct == 5 {
		ctx.Logger.Info("No waypoint for Act 5, trying Act 4→5 portal")
		return UseAct4ToAct5Portal()
	}

	ctx.Logger.Debug("No waypoint available for target act (will wait for leader)",
		slog.Int("targetAct", targetAct),
		slog.Int("availableWaypointsCount", len(ctx.Data.PlayerUnit.AvailableWaypoints)))
	return fmt.Errorf("no waypoint available for act %d", targetAct)
}

// UseAct4ToAct5Portal uses the red portal in Pandemonium Fortress to travel to Harrogath
func UseAct4ToAct5Portal() error {
	ctx := context.Get()
	ctx.SetLastAction("UseAct4ToAct5Portal")

	// Check if bot is stopping to avoid panic
	if ctx.ExecutionPriority == context.PriorityStop {
		return fmt.Errorf("bot is stopping")
	}

	// Make sure we're in Act 4
	if ctx.Data.PlayerUnit.Area != area.ThePandemoniumFortress {
		ctx.Logger.Warn("Not in Pandemonium Fortress, cannot use Act 4→5 portal")
		return fmt.Errorf("not in pandemonium fortress")
	}

	// Refresh game data
	ctx.RefreshGameData()

	// Find the portal to Harrogath (LastLastPortal)
	harrogathPortal, found := ctx.Data.Objects.FindOne(object.LastLastPortal)
	if !found {
		ctx.Logger.Warn("Act 4→5 portal not found in Pandemonium Fortress")
		return fmt.Errorf("harrogath portal not found")
	}

	ctx.Logger.Info("Using portal to Harrogath",
		slog.Int("portalX", harrogathPortal.Position.X),
		slog.Int("portalY", harrogathPortal.Position.Y))

	// Interact with the portal
	return InteractObject(harrogathPortal, func() bool {
		// Check if we successfully traveled to Harrogath
		ctx.RefreshGameData()
		return ctx.Data.PlayerUnit.Area == area.Harrogath
	})
}

// openWaypointMenuToRefreshData opens the waypoint menu briefly to populate AvailableWaypoints data
func openWaypointMenuToRefreshData() error {
	ctx := context.Get()

	// Make sure we're in town
	if !ctx.Data.PlayerUnit.Area.IsTown() {
		return fmt.Errorf("not in town")
	}

	// Find the waypoint in current town
	var wp data.Object
	found := false
	for _, o := range ctx.Data.Objects {
		if o.IsWaypoint() {
			wp = o
			found = true
			break
		}
	}

	if !found {
		// Try to move toward known waypoint position
		if wpPos, hasPos := townWaypointPositions[ctx.Data.PlayerUnit.Area]; hasPos {
			ctx.Logger.Debug("Waypoint not visible, moving toward it",
				slog.Int("x", wpPos.X),
				slog.Int("y", wpPos.Y))
			if err := MoveToCoords(wpPos); err != nil {
				return fmt.Errorf("failed to move to waypoint: %w", err)
			}
			// Refresh and try to find it again
			ctx.RefreshGameData()
			for _, o := range ctx.Data.Objects {
				if o.IsWaypoint() {
					wp = o
					found = true
					break
				}
			}
		}

		if !found {
			return fmt.Errorf("waypoint not found in town")
		}
	}

	// Move close to waypoint if needed
	distance := ctx.PathFinder.DistanceFromMe(wp.Position)
	if distance > 10 {
		ctx.Logger.Debug("Moving closer to waypoint",
			slog.Int("distance", distance))
		if err := MoveToCoords(wp.Position); err != nil {
			return fmt.Errorf("failed to move to waypoint: %w", err)
		}
	}

	// Interact with waypoint to open menu
	ctx.Logger.Debug("Opening waypoint menu")
	if err := step.InteractObject(wp, func() bool {
		ctx.RefreshGameData()
		return ctx.Data.OpenMenus.Waypoint
	}); err != nil {
		return fmt.Errorf("failed to open waypoint menu: %w", err)
	}

	// Menu is now open, AvailableWaypoints should be populated
	ctx.RefreshGameData()
	ctx.Logger.Debug("Waypoint menu opened, waypoints refreshed",
		slog.Int("availableCount", len(ctx.Data.PlayerUnit.AvailableWaypoints)))

	// Close the menu
	utils.Sleep(300) // Brief delay to ensure data is populated
	return step.CloseAllMenus()
}

// WaitAtPortalArea moves to and waits at the portal waiting area for the current act
// Uses the town package's TPWaitingArea which has proper positions for each act
func WaitAtPortalArea() error {
	ctx := context.Get()
	ctx.SetLastAction("WaitAtPortalArea")

	// Check if bot is stopping to avoid panic
	if ctx.ExecutionPriority == context.PriorityStop {
		return fmt.Errorf("bot is stopping")
	}

	ctx.RefreshGameData()

	// Get the current town handler - this knows the TP waiting area for each act
	currentTown := town.GetTownByArea(ctx.Data.PlayerUnit.Area)
	townArea := currentTown.TownArea()

	// If not in the correct town, try to get there
	if ctx.Data.PlayerUnit.Area != townArea {
		if ctx.Data.PlayerUnit.Area.IsTown() {
			// We're in a different town, use waypoint
			if err := WayPoint(townArea); err != nil {
				ctx.Logger.Warn("Failed to waypoint to correct town", slog.String("error", err.Error()))
				// Continue anyway, just wait where we are
			}
		} else {
			// Not in town at all, return to town first
			if err := ReturnTown(); err != nil {
				return fmt.Errorf("failed to return to town: %w", err)
			}
		}
		// Refresh after any travel
		ctx.RefreshGameData()
		currentTown = town.GetTownByArea(ctx.Data.PlayerUnit.Area)
	}

	// Get the TP waiting position from the town package
	waitPos := currentTown.TPWaitingArea(*ctx.Data)

	ctx.Logger.Debug("Moving to portal waiting area",
		slog.String("town", ctx.Data.PlayerUnit.Area.Area().Name),
		slog.Int("x", waitPos.X),
		slog.Int("y", waitPos.Y))

	// Try MoveToCoords first (proper pathfinding)
	err := MoveToCoords(waitPos)
	if err != nil {
		ctx.Logger.Debug("MoveToCoords failed, trying simple click movement",
			slog.String("error", err.Error()))

		// Fallback: simple click movement for town (safe since no combat)
		// Try a few clicks toward the destination
		for i := 0; i < 5; i++ {
			ctx.RefreshGameData()

			// Check if we're close enough (within 20 units)
			dx := ctx.Data.PlayerUnit.Position.X - waitPos.X
			dy := ctx.Data.PlayerUnit.Position.Y - waitPos.Y
			if dx*dx+dy*dy < 400 { // 20^2 = 400
				ctx.Logger.Debug("Reached portal waiting area via simple movement")
				return nil
			}

			// Click toward destination
			screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(waitPos.X, waitPos.Y)
			ctx.HID.Click(game.LeftButton, screenX, screenY)
			utils.Sleep(800)
		}

		// Even if we didn't reach exact position, we're still in town and can function
		ctx.Logger.Debug("Could not reach exact portal waiting position, staying at current location")
		return nil
	}

	return nil
}

// FindLeaderPortal finds a town portal in the current area
// Note: This function is called frequently, so logging is minimal to reduce spam
// FindLeaderPortal finds any town portal in the current area
// NOTE: This does NOT refresh game data - caller should ensure data is fresh
func FindLeaderPortal(playerName string) (data.Object, bool) {
	ctx := context.Get()
	ctx.SetLastAction("FindLeaderPortal")

	// Look for any town portal (no refresh needed, caller should have done it)
	for _, obj := range ctx.Data.Objects {
		if obj.Name == object.TownPortal || obj.Name == object.PermanentTownPortal {
			return obj, true
		}
	}

	return data.Object{}, false
}

// EnterPlayerPortal finds and enters a portal
func EnterPlayerPortal(playerName string) error {
	ctx := context.Get()
	ctx.SetLastAction("EnterPlayerPortal")

	// Check if bot is stopping to avoid panic
	if ctx.ExecutionPriority == context.PriorityStop {
		return fmt.Errorf("bot is stopping")
	}

	portal, found := FindLeaderPortal(playerName)
	if !found {
		return fmt.Errorf("no portal found")
	}

	// Move to the portal using pathfinding
	if err := MoveToCoords(portal.Position); err != nil {
		return fmt.Errorf("failed to move to portal: %w", err)
	}

	utils.Sleep(500) // Wait a moment after moving

	// Interact with the portal
	return InteractObject(portal, func() bool {
		// Check if we're no longer in town (portal worked)
		return !ctx.Data.PlayerUnit.Area.IsTown()
	})
}

// WaitForLeaderPortal polls for a portal to appear from the leader
func WaitForLeaderPortal(playerName string, timeout time.Duration) (data.Object, bool) {
	ctx := context.Get()
	ctx.SetLastAction("WaitForLeaderPortal")

	deadline := time.Now().Add(timeout)
	pollInterval := 500 * time.Millisecond

	for time.Now().Before(deadline) {
		if ctx.ExecutionPriority == context.PriorityStop {
			return data.Object{}, false
		}

		portal, found := FindLeaderPortal(playerName)
		if found {
			ctx.Logger.Info("Leader portal detected",
				slog.String("leader", playerName))
			return portal, true
		}

		utils.Sleep(int(pollInterval.Milliseconds()))
	}

	return data.Object{}, false
}

// ReturnThroughPortal finds a portal in the current area and uses it to return to town
func ReturnThroughPortal() error {
	ctx := context.Get()
	ctx.SetLastAction("ReturnThroughPortal")

	// Check if bot is stopping to avoid panic
	if ctx.ExecutionPriority == context.PriorityStop {
		return fmt.Errorf("bot is stopping")
	}

	ctx.RefreshGameData()

	// Find any portal in current area
	for _, obj := range ctx.Data.Objects {
		if obj.Name == object.TownPortal {
			// Move to and enter the portal using pathfinding
			if err := MoveToCoords(obj.Position); err != nil {
				return fmt.Errorf("failed to move to portal: %w", err)
			}

			utils.Sleep(500)

			return InteractObject(obj, func() bool {
				// Check if we're now in town
				return ctx.Data.PlayerUnit.Area.IsTown()
			})
		}
	}

	return fmt.Errorf("no portal found in current area")
}
