package run

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type ArcaneSanctuary struct {
	ctx                *context.Status
	clearMonsterFilter data.MonsterFilter // nil = normal run, non-nil = TZ run
}

func NewArcaneSanctuary() *ArcaneSanctuary {
	return &ArcaneSanctuary{
		ctx: context.Get(),
	}
}

// NewArcaneSanctuaryTZ creates an Arcane Sanctuary run for Terror Zone with custom filter
func NewArcaneSanctuaryTZ(filter data.MonsterFilter) *ArcaneSanctuary {
	return &ArcaneSanctuary{
		ctx:                context.Get(),
		clearMonsterFilter: filter,
	}
}

func (a ArcaneSanctuary) Name() string {
	return string(config.ArcaneSanctuaryRun)
}

func (a ArcaneSanctuary) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsQuestRun(parameters) {
		return SequencerError
	}
	if !a.ctx.Data.Quests[quest.Act2TheSummoner].Completed() {
		return SequencerSkip
	}
	return SequencerOk
}

// ignoreTowersFilter creates a filter that excludes lightning towers (LightningSpire)
func (a ArcaneSanctuary) ignoreTowersFilter(baseFilter data.MonsterFilter) data.MonsterFilter {
	return func(m data.Monsters) []data.Monster {
		// First apply the base filter (e.g., MonsterAnyFilter or MonsterEliteFilter)
		filtered := baseFilter(m)

		// Then remove LightningSpire NPCs (lightning sentry towers)
		var result []data.Monster
		for _, mo := range filtered {
			if mo.Name != npc.LightningSpire {
				result = append(result, mo)
			}
		}
		return result
	}
}

func (a ArcaneSanctuary) Run(parameters *RunParameters) error {
	var openChests bool
	var filter data.MonsterFilter

	// If we have a custom filter, we're running as Terror Zone
	if a.clearMonsterFilter != nil {
		openChests = a.ctx.CharacterCfg.Game.TerrorZone.OpenChests
		filter = a.clearMonsterFilter
	} else {
		// Normal run - use ArcaneSanctuary config
		openChests = a.ctx.CharacterCfg.Game.ArcaneSanctuary.OpenChests
		onlyElites := a.ctx.CharacterCfg.Game.ArcaneSanctuary.FocusOnElitePacks

		// Start with base filter
		baseFilter := data.MonsterAnyFilter()
		if onlyElites {
			baseFilter = data.MonsterEliteFilter()
		}

		// Apply tower exclusion on top of base filter
		filter = a.ignoreTowersFilter(baseFilter)
	}

	// Use Waypoint to get to Arcane Sanctuary
	err := action.WayPoint(area.ArcaneSanctuary)
	if err != nil {
		return err
	}

	action.Buff()
	action.OpenTPIfLeader()

	// Initialize Arcane Lane system
	lanes := NewArcaneLanes()

	// Check Summoner's location (if already known from map data)
	areaData := a.ctx.Data.Areas[area.ArcaneSanctuary]
	summonerNPC, summonerFound := areaData.NPCs.FindOne(npc.Summoner)

	// Clear all 4 lanes
	for lane := 0; lane < 4; lane++ {
		a.ctx.Logger.Info("Clearing Arcane Sanctuary - Lane %d/4", lane+1)

		// Clear this lane to End Point (one side)
		if err := lanes.ClearLane(filter, summonerNPC, summonerFound); err != nil {
			a.ctx.Logger.Warn("Lane %d clearing issue: %v", lane+1, err)
		}

		// Open chests at end of lane
		if openChests {
			lanes.OpenChestsAtEnd()
		}

		// Return to center via the other side
		if err := lanes.ReturnToCenter(filter); err != nil {
			a.ctx.Logger.Warn("Lane %d return path issue: %v", lane+1, err)
		}

		// Move on to next lane
		lanes.RotateToNextLane()
	}

	a.ctx.Logger.Info("Arcane Sanctuary run completed")
	return nil
}
