package step

import (
	"errors"
	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const DistanceToFinishMoving = 4
const stepMonsterCheckInterval = 100 * time.Millisecond

var (
	ErrMonstersInPath  = errors.New("monsters detected in movement path")
	ErrPlayerStuck     = errors.New("player is stuck")
	ErrPlayerRoundTrip = errors.New("player round trip")
	ErrNoPath          = errors.New("path couldn't be calculated")
)

type MoveOpts struct {
	distanceOverride      *int
	stationaryMinDistance *int
	stationaryMaxDistance *int
	ignoreShrines         bool
	ignoreMonsters        bool
	ignoreItems           bool
	monsterFilters        []data.MonsterFilter
	clearPathOverride     *int
}

type MoveOption func(*MoveOpts)

// WithDistanceToFinish overrides the default DistanceToFinishMoving
func WithDistanceToFinish(distance int) MoveOption {
	return func(opts *MoveOpts) {
		opts.distanceOverride = &distance
	}
}

// WithStationaryDistance configures MoveTo to stop when within a specific range of the destination.
func WithStationaryDistance(min, max int) MoveOption {
	return func(opts *MoveOpts) {
		opts.stationaryMinDistance = &min
		opts.stationaryMaxDistance = &max
	}
}

func WithIgnoreMonsters() MoveOption {
	return func(opts *MoveOpts) {
		opts.ignoreMonsters = true
	}
}

func WithIgnoreItems() MoveOption {
	return func(opts *MoveOpts) {
		opts.ignoreItems = true
	}
}

func IgnoreShrines() MoveOption {
	return func(opts *MoveOpts) {
		opts.ignoreShrines = true
	}
}

func WithMonsterFilter(filters ...data.MonsterFilter) MoveOption {
	return func(opts *MoveOpts) {
		opts.monsterFilters = append(opts.monsterFilters, filters...)
	}
}

func WithClearPathOverride(clearPathOverride int) MoveOption {
	return func(opts *MoveOpts) {
		opts.clearPathOverride = &clearPathOverride
	}
}

func (opts MoveOpts) DistanceToFinish() *int {
	return opts.distanceOverride
}

func (opts MoveOpts) IgnoreMonsters() bool {
	return opts.ignoreMonsters
}

func (opts MoveOpts) IgnoreItems() bool {
	return opts.ignoreItems
}

func (opts MoveOpts) MonsterFilters() []data.MonsterFilter {
	return opts.monsterFilters
}

func (opts MoveOpts) ClearPathOverride() *int {
	return opts.clearPathOverride
}

func MoveTo(dest data.Position, options ...MoveOption) error {
	// Initialize options
	opts := &MoveOpts{}

	// Apply any provided options
	for _, o := range options {
		o(opts)
	}

	minDistanceToFinishMoving := DistanceToFinishMoving
	if opts.distanceOverride != nil {
		minDistanceToFinishMoving = *opts.distanceOverride
	}

	ctx := context.Get()
	ctx.SetLastStep("MoveTo")

	opts.ignoreShrines = !ctx.CharacterCfg.Game.InteractWithShrines
	stepLastMonsterCheck := time.Time{}

	blockThreshold := 200 * time.Millisecond
	stuckThreshold := 2 * time.Second
	stuckCheckStartTime := time.Now()

	roundTripReferencePosition := ctx.Data.PlayerUnit.Position
	roundTripCheckStartTime := time.Now()
	const roundTripThreshold = 10 * time.Second
	const roundTripMaxRadius = 8

	// Adaptive movement refresh intervals based on ping
	// Adjust polling frequency based on network latency
	var walkDuration time.Duration
	if !ctx.Data.AreaData.Area.IsTown() {
		// In dungeons: faster refresh for combat
		baseMin, baseMax := 300, 350
		pingAdjustment := int(float64(ctx.Data.Game.Ping) * 0.5) // Add half ping to base
		walkDuration = utils.RandomDurationMs(baseMin+pingAdjustment, baseMax+pingAdjustment)
	} else {
		// In town: slower refresh is acceptable
		baseMin, baseMax := 500, 800
		pingAdjustment := int(float64(ctx.Data.Game.Ping) * 0.5)
		walkDuration = utils.RandomDurationMs(baseMin+pingAdjustment, baseMax+pingAdjustment)
	}

	lastRun := time.Time{}
	previousPosition := data.Position{}
	clearPathDist := ctx.CharacterCfg.Character.ClearPathDist
	overrideClearPathDist := false
	blocked := false
	if opts.ClearPathOverride() != nil {
		clearPathDist = *opts.ClearPathOverride()
		overrideClearPathDist = true
	}

	for {
		ctx.PauseIfNotPriority()
		ctx.RefreshGameData()
		currentDest := dest

		//Compute distance to destination
		currentDistanceToDest := ctx.PathFinder.DistanceFromMe(currentDest)

		//We've reached the destination, stop movement
		if currentDistanceToDest <= minDistanceToFinishMoving {
			return nil
		} else if blocked {
			//Add tolerance to reach destination if blocked
			if currentDistanceToDest <= minDistanceToFinishMoving*2 {
				return nil
			}
		}

		destOutsideCurrentArea := !ctx.Data.AreaData.IsInside(currentDest)
		// Check for Doors on path & open them. D2BS drills exits on level links so we also check doors
		// when crossing into adjacent areas, even if we can normally teleport over regular doors.
		if !ctx.Data.CanTeleport() || destOutsideCurrentArea {
			if doorFound, doorObj := ctx.PathFinder.HasDoorBetween(ctx.Data.PlayerUnit.Position, currentDest); doorFound {
				doorToOpen := *doorObj
				interactErr := error(nil)
				//Retry a few times (maggot lair slime door fix)
				for range 5 {
					if interactErr = InteractObject(doorToOpen, func() bool {
						door, found := ctx.Data.Objects.FindByID(doorToOpen.ID)
						return found && !door.Selectable
					}); interactErr == nil {
						break
					}
					ctx.PathFinder.RandomMovement()
					utils.Sleep(250)
				}
				if interactErr != nil {
					return interactErr
				}
			}
		}

		//Handle stationary distance (not sure what it refers to...)
		if opts.stationaryMinDistance != nil && opts.stationaryMaxDistance != nil {
			if currentDistanceToDest >= *opts.stationaryMinDistance && currentDistanceToDest <= *opts.stationaryMaxDistance {
				ctx.Logger.Debug(fmt.Sprintf("MoveTo: Reached stationary distance %d-%d (current %d)", *opts.stationaryMinDistance, *opts.stationaryMaxDistance, currentDistanceToDest))
				return nil
			}
		}

		//If teleporting, sleep for the cast duration
		if ctx.Data.CanTeleport() {
			if time.Since(lastRun) < ctx.Data.PlayerCastDuration() {
				time.Sleep(ctx.Data.PlayerCastDuration() - time.Since(lastRun))
				continue
			}
		}

		//Handle monsters if needed
		if !opts.ignoreMonsters && !ctx.Data.AreaData.Area.IsTown() && (!ctx.Data.CanTeleport() || overrideClearPathDist) && clearPathDist > 0 && time.Since(stepLastMonsterCheck) > stepMonsterCheckInterval {
			stepLastMonsterCheck = time.Now()
			monsterFound := false

			for _, m := range ctx.Data.Monsters.Enemies(opts.monsterFilters...) {
				if ctx.Char.ShouldIgnoreMonster(m) {
					continue
				}
				//Check distance first as it is cheaper
				distanceToMonster := ctx.PathFinder.DistanceFromMe(m.Position)
				if distanceToMonster <= clearPathDist {
					//Line of sight second
					if ctx.PathFinder.LineOfSight(ctx.Data.PlayerUnit.Position, m.Position) {
						//Finally door check as it computes path
						if hasDoorBetween, _ := ctx.PathFinder.HasDoorBetween(ctx.Data.PlayerUnit.Position, m.Position); !hasDoorBetween {
							monsterFound = true
							break
						}
					}
				}
			}

			if monsterFound {
				return ErrMonstersInPath
			}
		}

		currentPosition := ctx.Data.PlayerUnit.Position
		blocked = false
		//Detect if player is doing round trips around a position for too long and return error if it's the case
		if utils.CalculateDistance(currentPosition, roundTripReferencePosition) <= roundTripMaxRadius {
			timeInRoundtrip := time.Since(roundTripCheckStartTime)
			if timeInRoundtrip > roundTripThreshold {
				ctx.Logger.Warn("Player is doing round trips. Current area: [" + ctx.Data.PlayerUnit.Area.Area().Name + "]. Trying to path to Destination: [" + fmt.Sprintf("%d,%d", currentDest.X, currentDest.Y) + "]")
				return ErrPlayerRoundTrip
			} else if timeInRoundtrip > roundTripThreshold/2.0 {
				blocked = true
			}
		} else {
			//Player moved significantly, reset Round Trip detection
			roundTripReferencePosition = currentPosition
			roundTripCheckStartTime = time.Now()
		}

		if currentPosition == previousPosition && !ctx.Data.PlayerUnit.States.HasState(state.Stunned) {
			stuckTime := time.Since(stuckCheckStartTime)
			if stuckTime > stuckThreshold {
				//if stuck for too long, abort movement
				return ErrPlayerStuck
			} else if stuckTime > blockThreshold {
				//Detect blocked after short threshold
				blocked = true
			}
		} else {
			//Player moved, reset stuck detection timer
			stuckCheckStartTime = time.Now()
		}

		if blocked {
			//First check if there's a destructible nearby
			if obj, found := ctx.PathFinder.GetClosestDestructible(ctx.Data.PlayerUnit.Position); found {
				if !obj.Selectable {
					// Already destroyed, move on
					continue
				}
				x, y := ui.GameCoordsToScreenCords(obj.Position.X, obj.Position.Y)
				ctx.HID.Click(game.LeftButton, x, y)

				// Adaptive delay for obstacle interaction based on ping
				time.Sleep(time.Millisecond * time.Duration(utils.PingMultiplier(utils.Light, 100)))
			} else if door, found := ctx.PathFinder.GetClosestDoor(ctx.Data.PlayerUnit.Position); found {
				//There's a door really close, try to open it
				doorToOpen := *door
				InteractObject(doorToOpen, func() bool {
					door, found := ctx.Data.Objects.FindByID(door.ID)
					return found && !door.Selectable
				})
			}
		}

		//Handle skills for navigation
		if ctx.Data.CanTeleport() {
			for range 3 {
				ctx.WaitForGameToLoad()

				if ctx.Data.PlayerUnit.RightSkill != skill.Teleport {
					ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.MustKBForSkill(skill.Teleport))
				}
				if ctx.Data.PlayerUnit.RightSkill == skill.Teleport {
					break
				}
				ctx.RefreshGameData()
				utils.Sleep(100)
			}
		} else if kb, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.Vigor); found {
			if ctx.Data.PlayerUnit.RightSkill != skill.Vigor {
				ctx.HID.PressKeyBinding(kb)
			}
		}

		//Compute path to reach destination
		path, _, found := ctx.PathFinder.GetPath(currentDest)
		if !found {
			//Couldn't find path, abort movement
			ctx.Logger.Warn("path could not be calculated. Current area: [" + ctx.Data.PlayerUnit.Area.Area().Name + "]. Trying to path to Destination: [" + fmt.Sprintf("%d,%d", currentDest.X, currentDest.Y) + "]")
			return ErrNoPath
		} else if len(path) == 0 {
			//Path found but it's empty, consider movement done
			//Not sure if it can happen
			ctx.Logger.Warn("path found but it's empty: [" + ctx.Data.PlayerUnit.Area.Area().Name + "]. Trying to path to Destination: [" + fmt.Sprintf("%d,%d", currentDest.X, currentDest.Y) + "]")
			return nil
		}

		//Update values
		lastRun = time.Now()
		previousPosition = ctx.Data.PlayerUnit.Position

		//Perform the movement
		ctx.PathFinder.MoveThroughPath(path, walkDuration)
	}
}
