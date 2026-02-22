package action

import (
	"errors"
	"fmt"
	"log/slog"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// Cow-only tuned clear: aggressive movement + less pickup spam + fixed alive filtering (only inside cows).
func ClearCurrentLevelCows(openChests bool, filter data.MonsterFilter) error {
	ctx := context.Get()
	ctx.SetLastAction("ClearCurrentLevelCows")

	const (
		pickupRadius     = 10 // smaller for cows
		pickupEveryRooms = 4  // pick up every N rooms + last room
		moveClearRadius  = 20 // used by ClearThroughPath
	)

	rooms := ctx.PathFinder.OptimizeRoomsTraverseOrder()

	for i, r := range rooms {
		if errDeath := checkPlayerDeath(ctx); errDeath != nil {
			return errDeath
		}

		// Aggressive “fight-through” movement to room center (no monster filter path-avoidance)
		if err := clearRoomCows(r, filter, moveClearRadius); err != nil {
			ctx.Logger.Warn("Failed to clear room (cows)", slog.Any("error", err))
		}

		// Don’t loot-vacuum after every room
		if (i%pickupEveryRooms == 0) || (i == len(rooms)-1) {
			if err := ItemPickup(pickupRadius); err != nil {
				ctx.Logger.Warn("Failed to pickup items (cows)", slog.Any("error", err))
			}
		}

		// Optional chest opening (usually false for speed)
		if openChests {
			for _, o := range ctx.Data.Objects {
				if r.IsInside(o.Position) && o.IsChest() && o.Selectable {
					if err := MoveToCoords(o.Position); err != nil {
						continue
					}
					_ = InteractObject(o, func() bool {
						chest, _ := ctx.Data.Objects.FindByID(o.ID)
						return !chest.Selectable
					})
					utils.Sleep(250)
				}
			}
		}
	}

	return nil
}

func clearRoomCows(room data.Room, filter data.MonsterFilter, moveClearRadius int) error {
	ctx := context.Get()
	ctx.SetLastAction("clearRoomCows")

	path, _, found := ctx.PathFinder.GetClosestWalkablePath(room.GetCenter())
	if !found {
		return errors.New("failed to find a path to the room center")
	}

	to := data.Position{
		X: path.To().X + ctx.Data.AreaOrigin.X,
		Y: path.To().Y + ctx.Data.AreaOrigin.Y,
	}

	// Clear while moving so we don’t edge-hug around packs
	if err := ClearThroughPath(to, moveClearRadius, filter); err != nil {
		return fmt.Errorf("failed moving/clearing to room center: %w", err)
	}

	for {
		ctx.PauseIfNotPriority()

		if err := checkPlayerDeath(ctx); err != nil {
			return err
		}

		monsters := getMonstersInRoomCows(room, filter)
		if len(monsters) == 0 {
			return nil
		}

		SortEnemiesByPriority(&monsters)

		target := data.Monster{}
		for _, m := range monsters {
			if ctx.Char.ShouldIgnoreMonster(m) {
				continue
			}
			if m.IsMonsterRaiser() {
				target = m
				break
			}
			if target.UnitID == 0 {
				target = m
			}
		}

		if target.UnitID == 0 {
			return nil
		}

		ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			m, ok := d.Monsters.FindByID(target.UnitID)
			if ok && m.Stats[stat.Life] > 0 {
				return target.UnitID, true
			}
			return 0, false
		}, nil)
	}
}

// Cow-only “alive AND (in-room OR near)” so you don’t target corpses near you.
func getMonstersInRoomCows(room data.Room, filter data.MonsterFilter) []data.Monster {
	ctx := context.Get()

	out := make([]data.Monster, 0)
	for _, m := range ctx.Data.Monsters.Enemies(filter) {
		if m.Stats[stat.Life] > 0 && (room.IsInside(m.Position) || ctx.PathFinder.DistanceFromMe(m.Position) < 30) {
			out = append(out, m)
		}
	}
	return out
}
