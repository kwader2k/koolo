package run

import (
	"errors"
	"math"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Summoner struct {
	ctx                *context.Status
	clearMonsterFilter data.MonsterFilter // nil = normal run, non-nil = TZ lane clear
}

// Constructor for normal Summoner (quests / standard run)
func NewSummoner() *Summoner {
	return &Summoner{
		ctx: context.Get(),
	}
}

// Constructor for TZ Summoner (Arcane Sanctuary terror zone)
func NewSummonerTZ(filter data.MonsterFilter) *Summoner {
	return &Summoner{
		ctx:                context.Get(),
		clearMonsterFilter: filter,
	}
}

func (s Summoner) Name() string {
	return string(config.SummonerRun)
}

func (s Summoner) CheckConditions(parameters *RunParameters) SequencerResult {
	farmingRun := IsFarmingRun(parameters)
	questCompleted := s.ctx.Data.Quests[quest.Act2TheSummoner].Completed()
	if (farmingRun && !questCompleted) || (!farmingRun && questCompleted) {
		return SequencerSkip
	}
	return SequencerOk
}

func (s Summoner) Run(parameters *RunParameters) error {
	// If we have a filter, we’re being called from TerrorZone as a TZ run.
	if s.clearMonsterFilter != nil {
		return s.runTerrorZone()
	}

	// Otherwise this is the normal quest/key run.
	return s.runStandard(parameters)
}

// ---------------- TZ ARCANE SANCTUARY ----------------

func (s Summoner) runTerrorZone() error {
	s.ctx.Logger.Info("Starting Arcane Sanctuary Terror Zone run")

	// Move to Waypoint
	if err := action.WayPoint(area.ArcaneSanctuary); err != nil {
		return err
	}

	action.Buff()

	// Initialize Arcane Lane system
	lanes := NewArcaneLanes()

	// Check Summoner's location (if already known from map data)
	areaData := s.ctx.Data.Areas[area.ArcaneSanctuary]
	summonerNPC, summonerFound := areaData.NPCs.FindOne(npc.Summoner)

	// Clear all 4 lanes
	for lane := 0; lane < 4; lane++ {
		s.ctx.Logger.Info("Clearing Arcane Sanctuary TZ - Lane %d/4", lane+1)

		// Clear this lane to End Point (one side)
		if err := lanes.ClearLane(s.clearMonsterFilter, summonerNPC, summonerFound); err != nil {
			s.ctx.Logger.Warn("Lane %d clearing issue: %v", lane+1, err)
		}

		// Open chests at end of lane
		if s.ctx.CharacterCfg.Game.TerrorZone.OpenChests {
			lanes.OpenChestsAtEnd()
		}

		// Return to center via the other side
		if err := lanes.ReturnToCenter(s.clearMonsterFilter); err != nil {
			s.ctx.Logger.Warn("Lane %d return path issue: %v", lane+1, err)
		}

		// Move on to next lane
		lanes.RotateToNextLane()
	}

	s.ctx.Logger.Info("Arcane Sanctuary Terror Zone run completed")
	return nil
}

// ---------------- NORMAL SUMMONER RUN ----------------

func (s Summoner) runStandard(parameters *RunParameters) error {
	s.ctx.Logger.Info("Starting normal Summoner run (Quest/Key)")
	isQuestRun := IsQuestRun(parameters)

	// Use the waypoint / Fire Eye portal to get to Arcane Sanctuary
	if s.ctx.CharacterCfg.Game.Summoner.KillFireEye && !isQuestRun {
		NewFireEye().Run(parameters) // same pattern as other quest runs

		obj, _ := s.ctx.Data.Objects.FindOne(object.ArcaneSanctuaryPortal)

		err := action.InteractObject(obj, func() bool {
			updatedObj, found := s.ctx.Data.Objects.FindOne(object.ArcaneSanctuaryPortal)
			if found {
				if !updatedObj.Selectable {
					s.ctx.Logger.Debug("Interacted with ArcaneSanctuaryPortal")
				}
				return !updatedObj.Selectable
			}
			return false
		})
		if err != nil {
			return err
		}

		err = action.InteractObject(obj, func() bool {
			return s.ctx.Data.PlayerUnit.Area == area.ArcaneSanctuary
		})
		if err != nil {
			return err
		}

		utils.Sleep(300)
	} else {
		if err := action.WayPoint(area.ArcaneSanctuary); err != nil {
			return err
		}

		// This prevents us being blocked from getting into Palace
		if s.ctx.Data.PlayerUnit.Area != area.ArcaneSanctuary && isQuestRun {
			action.InteractNPC(npc.Drognan)
		}
	}

	action.Buff()

	// Get the Summoner's position from the cached map data
	areaData := s.ctx.Data.Areas[area.ArcaneSanctuary]
	summonerNPC, found := areaData.NPCs.FindOne(npc.Summoner)
	if !found || len(summonerNPC.Positions) == 0 {
		return errors.New("failed to find the Summoner")
	}

	// Move to the Summoner's position using the static coordinates from map data
	if err := action.MoveToCoords(summonerNPC.Positions[0]); err != nil {
		return err
	}

	// Kill Summoner
	if err := s.ctx.Char.KillSummoner(); err != nil {
		return err
	}

	if isQuestRun {
		if err := s.goToCanyon(); err != nil {
			return err
		}
	}

	return nil
}

// ---------------- ARCANE LANES SYSTEM ----------------

type ArcaneLanes struct {
	checkPoints    []data.Position
	sequence       []int // Sequence to End Point (one side)
	returnSequence []int // Return path (other side)
	clearRange     int
	ctx            *context.Status
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
		sequence:       []int{1, 2, 6, 3, 4}, // Go to End Point via right side
		returnSequence: []int{5, 6, 1, 0},    // Return to center via left side
		clearRange:     30,
		ctx:            context.Get(),
	}
}

// shouldOpenChest determines if a chest should be opened based on bot settings
// This mimics the logic from action.ClearCurrentLevel for consistency
func (al *ArcaneLanes) shouldOpenChest(obj data.Object) bool {
	// Global settings take precedence (same as ClearCurrentLevel)
	openAllChests := al.ctx.CharacterCfg.Game.InteractWithChests
	openSuperOnly := al.ctx.CharacterCfg.Game.InteractWithSuperChests && !openAllChests

	// Per-run openChests setting (from ArcaneSanctuary or TerrorZone config)
	// This is checked automatically - we don't need a parameter
	openChests := al.ctx.CharacterCfg.Game.ArcaneSanctuary.OpenChests
	// Check if we're in TZ mode by checking if TerrorZone config is active
	if len(al.ctx.Data.TerrorZones) > 0 {
		openChests = al.ctx.CharacterCfg.Game.TerrorZone.OpenChests
	}

	if !obj.Selectable {
		// Even if not selectable, allow Arcane-specific chests and those matching IsChest()
		isArcaneChest := obj.Name == 398 || obj.Name == 399 || obj.Name == 400 || obj.Name == 401 || obj.Name == 402
		if !obj.IsChest() && !isArcaneChest {
			return false
		}
	}

	// Same logic as ClearCurrentLevel (lines 69-76 of clear_level.go)
	switch {
	case openSuperOnly:
		return obj.IsSuperChest()
	case openAllChests:
		return obj.IsChest() || obj.IsSuperChest()
	case openChests:
		return obj.IsChest()
	}

	return false
}

// ClearLane clears the current lane
func (al *ArcaneLanes) ClearLane(filter data.MonsterFilter, summonerNPC data.NPC, summonerFound bool) error {
	for _, idx := range al.sequence {
		// Clear while moving along a safe path
		if err := action.ClearThroughPath(
			al.checkPoints[idx],
			al.clearRange,
			filter,
		); err != nil {
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
				if err := al.ctx.Char.KillSummoner(); err != nil {
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

// ReturnToCenter returns to center via the other side of the lane
func (al *ArcaneLanes) ReturnToCenter(filter data.MonsterFilter) error {
	for _, idx := range al.returnSequence {
		// Clear while moving back to center
		if err := action.ClearThroughPath(
			al.checkPoints[idx],
			al.clearRange,
			filter,
		); err != nil {
			al.ctx.Logger.Debug("ClearThroughPath error at checkpoint %d during return: %v", idx, err)
			// Continue even on error
		}
	}
	return nil
}

// OpenChestsAtEnd opens chests at the end of the lane
func (al *ArcaneLanes) OpenChestsAtEnd() {
	laneEndPos := al.checkPoints[4] // End Point
	chestsOpened := 0

	for _, obj := range al.ctx.Data.Objects {
		// Only check objects within reasonable distance from lane end
		distance := pather.DistanceFromPoint(obj.Position, laneEndPos)
		if distance < 5 || distance > 25 {
			continue
		}

		// Use shouldOpenChest() to respect bot settings (global + per-run)
		if !al.shouldOpenChest(obj) {
			continue
		}

		if err := action.MoveToCoords(obj.Position); err != nil {
			al.ctx.Logger.Debug("Failed to move to chest at distance %d: %v", distance, err)
			continue
		}

		if err := action.InteractObject(obj, func() bool {
			chest, found := al.ctx.Data.Objects.FindByID(obj.ID)
			return found && !chest.Selectable
		}); err != nil {
			al.ctx.Logger.Debug("Failed to open chest Name=%d: %v", obj.Name, err)
		} else {
			chestsOpened++
		}
	}

	if chestsOpened > 0 {
		al.ctx.Logger.Info("Opened %d chests at lane end", chestsOpened)
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

func (s Summoner) goToCanyon() error {
	// Interact with journal to open poratl
	tome, found := s.ctx.Data.Objects.FindOne(object.YetAnotherTome)
	if !found {
		s.ctx.Logger.Error("YetAnotherTome (journal) not found after Summoner kill. This is unexpected.")
		return errors.New("Journal not found after summoner")
	}

	err := action.InteractObject(tome, func() bool {
		_, found := s.ctx.Data.Objects.FindOne(object.PermanentTownPortal)
		return found
	})
	if err != nil {
		return err
	}

	//go through portal
	portal, _ := s.ctx.Data.Objects.FindOne(object.PermanentTownPortal)
	err = action.InteractObject(portal, func() bool {
		return s.ctx.Data.PlayerUnit.Area == area.CanyonOfTheMagi && s.ctx.Data.AreaData.IsInside(s.ctx.Data.PlayerUnit.Position)
	})
	if err != nil {
		return err
	}

	//Get WP
	err = action.DiscoverWaypoint()
	if err != nil {
		return err
	}
	return nil // Return to re-evaluate after completing this chain.
}
