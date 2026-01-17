package run

import (
	"slices"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

type Mephisto struct {
	ctx                *context.Status
	clearMonsterFilter data.MonsterFilter
}

func (m Mephisto) ToKeyBinding(keyCode byte) data.KeyBinding {
	return data.KeyBinding{
		Key1: [2]byte{keyCode, 0},
		Key2: [2]byte{0, 0},
	}
}

func (m Mephisto) HoldKey(keyCode byte, durationMs int) {
	kb := m.ToKeyBinding(keyCode)
	m.ctx.HID.KeyDown(kb)
	time.Sleep(time.Duration(durationMs) * time.Millisecond)
	m.ctx.HID.KeyUp(kb)
}

func NewMephisto(tzClearFilter data.MonsterFilter) *Mephisto {
	return &Mephisto{
		ctx:                context.Get(),
		clearMonsterFilter: tzClearFilter,
	}
}

func (m Mephisto) Name() string {
	return string(config.MephistoRun)
}

func (m Mephisto) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		if !m.ctx.Data.Quests[quest.Act3TheGuardian].Completed() {
			return SequencerSkip
		}
		return SequencerOk
	}

	if !m.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		return SequencerStop
	}
	if m.ctx.Data.Quests[quest.Act3TheGuardian].Completed() {
		if slices.Contains(m.ctx.Data.PlayerUnit.AvailableWaypoints, area.ThePandemoniumFortress) ||
			slices.Contains(m.ctx.Data.PlayerUnit.AvailableWaypoints, area.Harrogath) {
			return SequencerSkip
		}

		//Workaround AvailableWaypoints only filled when wp menu has been opened on act page
		//Check if any act 4 quests has started or is completed
		if action.HasAnyQuestStartedOrCompleted(quest.Act4TheFallenAngel, quest.Act4TerrorsEnd) || m.ctx.Data.PlayerUnit.Area.Act() >= 4 {
			return SequencerSkip
		}
	}
	return SequencerOk
}

func (m Mephisto) Run(parameters *RunParameters) error {

	// Use waypoint to DuranceOfHateLevel2
	err := action.WayPoint(area.DuranceOfHateLevel2)
	if err != nil {
		return err
	}

	if m.clearMonsterFilter != nil {
		if err = action.ClearCurrentLevel(m.ctx.CharacterCfg.Game.Mephisto.OpenChests, m.clearMonsterFilter); err != nil {
			return err
		}
	}

	// Move to DuranceOfHateLevel3
	if err = action.MoveToArea(area.DuranceOfHateLevel3); err != nil {
		return err
	}

	lvl, _ := m.ctx.Data.PlayerUnit.FindStat(stat.Level, 0)

	_, isLevelingChar := m.ctx.Char.(context.LevelingCharacter)
	if isLevelingChar && lvl.Value < 80 {

		action.ReturnTown()
		action.IdentifyAll(false)
		action.Stash(false)
		action.ReviveMerc()
		action.Repair()
		action.VendorRefill(action.VendorRefillOpts{ForceRefill: true, SellJunk: true, BuyConsumables: true})

		err = action.UsePortalInTown()
		if err != nil {
			return err
		}

	}

	// Move to the Safe position
	action.MoveToCoords(data.Position{
		X: 17568,
		Y: 8069,
	})

	// Disable item pickup while fighting Mephisto (prevent picking up items if nearby monsters die)
	m.ctx.DisableItemPickup()

	// Kill Mephisto
	err = m.ctx.Char.KillMephisto()

	// Enable item pickup after the fight
	m.ctx.EnableItemPickup()

	if err != nil {
		return err
	}

	action.ItemPickup(30)

	if m.ctx.CharacterCfg.Game.Mephisto.OpenChests || m.ctx.CharacterCfg.Game.Mephisto.KillCouncilMembers {
		if err = action.ClearCurrentLevel(m.ctx.CharacterCfg.Game.Mephisto.OpenChests, m.CouncilMemberFilter()); err != nil {
			return err
		}
	}

	if IsQuestRun(parameters) || m.ctx.CharacterCfg.Game.Mephisto.ExitToA4 {

		_, isLevelingChar := m.ctx.Char.(context.LevelingCharacter)
		if isLevelingChar {
			action.MoveToCoords(data.Position{
				X: 17568,
				Y: 8069,
			})

			if err = action.ClearAreaAroundPlayer(30, m.MephistoFilter()); err != nil {
				return err
			}
		}

		m.ctx.Logger.Debug("Moving to bridge")
		action.MoveToCoords(data.Position{X: 17588, Y: 8068})
		//Wait for bridge to rise
		utils.Sleep(1000)

		if isLevelingChar {
			if err = action.ClearAreaAroundPlayer(30, m.MephistoFilter()); err != nil {
				return err
			}
		}

		m.ctx.Logger.Debug("Moving to red portal")
		portal, _ := m.ctx.Data.Objects.FindOne(object.HellGate)
		action.MoveToCoords(portal.Position)

		action.InteractObject(portal, func() bool {
			return m.ctx.Data.PlayerUnit.Area == area.ThePandemoniumFortress
		})

		if isLevelingChar {
			utils.Sleep(1000)
			m.HoldKey(win.VK_SPACE, 2000)
			utils.Sleep(1000)
		}
	}

	return nil
}

func (m Mephisto) CouncilMemberFilter() data.MonsterFilter {
	return func(m data.Monsters) []data.Monster {
		var filteredMonsters []data.Monster
		for _, mo := range m {
			if mo.Name == npc.CouncilMember || mo.Name == npc.CouncilMember2 || mo.Name == npc.CouncilMember3 {
				filteredMonsters = append(filteredMonsters, mo)
			}
		}

		return filteredMonsters
	}
}

func (m Mephisto) MephistoFilter() data.MonsterFilter {
	return func(m data.Monsters) []data.Monster {
		var filteredMonsters []data.Monster
		for _, mo := range m {
			if mo.Name == npc.Mephisto {
				filteredMonsters = append(filteredMonsters, mo)
			}
		}

		return filteredMonsters
	}
}
