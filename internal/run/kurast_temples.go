package run

import (
	"slices"
	"sort"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/pather"
)

type KurastTemples struct {
	ctx *context.Status
}

type kurastTempleGroup struct {
	base    area.ID
	temples []area.ID
}

func NewKurastTemples() *KurastTemples {
	return &KurastTemples{
		ctx: context.Get(),
	}
}

func (k KurastTemples) Name() string {
	return string(config.KurastTemplesRun)
}

func (k KurastTemples) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) {
		return SequencerError
	}
	if !k.ctx.Data.Quests[quest.Act2TheSevenTombs].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

func (k KurastTemples) Run(parameters *RunParameters) error {
	// Announce the run start.
	k.ctx.Logger.Info("Starting Kurast Temples run")

	// Travel to the Lower Kurast waypoint to start the loop.
	if err := action.WayPoint(area.LowerKurast); err != nil {
		return err
	}

	// Define each base area and its paired temples.
	groups := []kurastTempleGroup{
		{
			base:    area.KurastBazaar,
			temples: []area.ID{area.RuinedTemple, area.DisusedFane},
		},
		{
			base:    area.UpperKurast,
			temples: []area.ID{area.ForgottenReliquary, area.ForgottenTemple},
		},
		{
			base:    area.KurastCauseway,
			temples: []area.ID{area.RuinedFane, area.DisusedReliquary},
		},
	}

	// Visit each base area and its temple pair.
	for _, group := range groups {
		// Move to the base area if we are not already there.
		if k.ctx.Data.PlayerUnit.Area != group.base {
			if err := action.MoveToArea(group.base); err != nil {
				return err
			}
		}

		// Clear each temple, preferring the closer exit first.
		for _, temple := range k.sortedTemples(group.temples) {
			// Enter the temple.
			if err := action.MoveToArea(temple); err != nil {
				return err
			}

			// Clear the current temple level.
			if err := action.ClearCurrentLevel(false, data.MonsterAnyFilter()); err != nil {
				return err
			}

			// Return to the base area before the next temple.
			if err := action.MoveToArea(group.base); err != nil {
				return err
			}
		}
	}

	return nil
}

func (k KurastTemples) sortedTemples(temples []area.ID) []area.ID {
	// Build a map of temple exits we can see from the base area.
	exitPositions := make(map[area.ID]data.Position, len(k.ctx.Data.AdjacentLevels))
	for _, level := range k.ctx.Data.AdjacentLevels {
		exitPositions[level.Area] = level.Position
	}

	// Track temple distance so we can visit the closest first.
	type templeInfo struct {
		id       area.ID
		distance int
		hasExit  bool
	}

	// Use a large value when we do not know a distance.
	maxDistance := int(^uint(0) >> 1)
	infos := make([]templeInfo, 0, len(temples))
	for _, temple := range temples {
		pos, ok := exitPositions[temple]
		distance := maxDistance
		if ok {
			distance = pather.DistanceFromPoint(k.ctx.Data.PlayerUnit.Position, pos)
		}
		infos = append(infos, templeInfo{id: temple, distance: distance, hasExit: ok})
	}

	// Keep the original order if we cannot see any exits.
	hasAnyExit := false
	for _, info := range infos {
		if info.hasExit {
			hasAnyExit = true
			break
		}
	}
	if !hasAnyExit {
		return slices.Clone(temples)
	}

	// Sort temples by distance to the exit.
	sort.SliceStable(infos, func(i, j int) bool {
		return infos[i].distance < infos[j].distance
	})

	// Extract the ordered temple IDs.
	ordered := make([]area.ID, len(infos))
	for i, info := range infos {
		ordered[i] = info.id
	}
	return ordered
}
