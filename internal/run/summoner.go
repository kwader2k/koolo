package run

import (
	"errors"
	"math"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Summoner struct {
	ctx                *context.Status
	clearMonsterFilter data.MonsterFilter
}

func NewSummoner(clearMonsterFilter data.MonsterFilter) *Summoner {
	return &Summoner{
		ctx:                context.Get(),
		clearMonsterFilter: clearMonsterFilter, 
	}
}

func (s Summoner) Name() string {
	return string(config.SummonerRun)
}

func (s Summoner) Run() error {
	isTZRun := s.clearMonsterFilter != nil 

	if isTZRun {
		s.ctx.Logger.Info("Starting Arcane Sanctuary Terror Zone run")

		// Move to Waypoint
		err := action.WayPoint(area.ArcaneSanctuary)
		if err != nil {
			return err
		}

		action.Buff()

		// Initialize Arcane Lane system
		lanes := NewArcaneLanes()

		// Check Summoner's location (kill if found)
		areaData := s.ctx.Data.Areas[area.ArcaneSanctuary]
		summonerNPC, summonerFound := areaData.NPCs.FindOne(npc.Summoner)

		// Clear all 4 lanes
		for lane := 0; lane < 4; lane++ {
			s.ctx.Logger.Info("Clearing Arcane Sanctuary TZ - Lane %d/4", lane+1)

			// Clear lane (using the TZ filter)
			err := lanes.ClearLane(s.clearMonsterFilter, summonerNPC, summonerFound)
			if err != nil {
				s.ctx.Logger.Warn("Failed to clear lane %d: %v", lane+1, err)
				// Continue even on error
			}

			// Open chests (if option is enabled)
			if s.ctx.CharacterCfg.Game.TerrorZone.OpenChests {
				lanes.OpenChestsAtEnd()
			}

			// Rotate to the next lane (90 degrees)
			lanes.RotateToNextLane()
		}

		s.ctx.Logger.Info("Arcane Sanctuary Terror Zone run completed")
		return nil
	}

	s.ctx.Logger.Info("Starting normal Summoner run (Quest/Key)")

	// Use the waypoint to get to Arcane Sanctuary
	if s.ctx.CharacterCfg.Game.Summoner.KillFireEye {
		NewFireEye().Run()

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

	}

	if !s.ctx.CharacterCfg.Game.Summoner.KillFireEye {
		err := action.WayPoint(area.ArcaneSanctuary)
		if err != nil {
			return err
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
	return s.ctx.Char.KillSummoner()
}


// Arcane Lanes System
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
