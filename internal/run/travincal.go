package run

import (
	"errors"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/character"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Travincal struct {
	ctx *context.Status
}

func NewTravincal() *Travincal {
	return &Travincal{
		ctx: context.Get(),
	}
}

func (t *Travincal) Name() string {
	return string(config.TravincalRun)
}

func (t *Travincal) CheckConditions(parameters *RunParameters) SequencerResult {
	farmingRun := IsFarmingRun(parameters)
	questCompleted := t.ctx.Data.Quests[quest.Act3TheBlackenedTemple].Completed() && t.ctx.Data.Quests[quest.Act3KhalimsWill].Completed()
	if (farmingRun && !questCompleted) || (!farmingRun && questCompleted) {
		return SequencerSkip
	}
	return SequencerOk
}

func (t *Travincal) Run(parameters *RunParameters) error {
	defer func() {
		t.ctx.CurrentGame.AreaCorrection.Enabled = false
	}()

	// Check if the character is a Berserker and swap to combat gear
	if berserker, ok := t.ctx.Char.(*character.Berserker); ok {
		if t.ctx.CharacterCfg.Character.BerserkerBarb.FindItemSwitch {
			berserker.SwapToSlot(0) // Swap to combat gear (lowest Gold Find)
		}
	}

	err := action.WayPoint(area.Travincal)
	if err != nil {
		return err
	}

	// Only Enable Area Correction for Travincal
	t.ctx.CurrentGame.AreaCorrection.ExpectedArea = area.Travincal
	t.ctx.CurrentGame.AreaCorrection.Enabled = true

	//TODO This is temporary needed for barb because have no cta; isrebuffrequired not working for him. We have ActiveWeaponSlot in d2go ready for that
	action.Buff()

	if !IsQuestRun(parameters) {
		// Blacklist the entrance to durance of hate level 1 to prevent accidental entry
		entranceFound := false
		for _, al := range t.ctx.Data.AdjacentLevels {
			if al.Area == area.DuranceOfHateLevel1 {
				entranceFound = true
				// Get the exact entrance position
				entranceWorldPos := al.Position
				relativePos := t.ctx.Data.AreaData.Grid.RelativePosition(entranceWorldPos)

				// Validate the entrance position is within grid bounds
				if relativePos.X < 0 || relativePos.X >= t.ctx.Data.AreaData.Grid.Width ||
					relativePos.Y < 0 || relativePos.Y >= t.ctx.Data.AreaData.Grid.Height {
					t.ctx.Logger.Warn("Durance of Hate entrance is outside grid bounds", "worldPosition", entranceWorldPos, "gridPosition", relativePos)
					break
				}

				// Mark a 3 unit radius area around the entrance as non-walkable (7x7 grid)
				blacklistRadius := 3
				blacklistedCount := 0
				for dx := -blacklistRadius; dx <= blacklistRadius; dx++ {
					for dy := -blacklistRadius; dy <= blacklistRadius; dy++ {
						x := relativePos.X + dx
						y := relativePos.Y + dy
						if x >= 0 && x < t.ctx.Data.AreaData.Grid.Width &&
							y >= 0 && y < t.ctx.Data.AreaData.Grid.Height {
							t.ctx.Data.AreaData.Grid.Set(x, y, game.CollisionTypeNonWalkable)
							blacklistedCount++
						}
					}
				}
				t.ctx.Logger.Debug("Successfully blacklisted Durance of Hate entrance",
					"worldPosition", entranceWorldPos,
					"gridPosition", relativePos,
					"radius", blacklistRadius,
					"tilesBlacklisted", blacklistedCount)
				break
			}
		}
		if !entranceFound {
			t.ctx.Logger.Warn("Durance of Hate entrance not found in adjacent levels")
		}
	}

	councilPosition := t.findCouncilPosition()

	err = action.MoveToCoords(councilPosition)
	if err != nil {
		t.ctx.Logger.Warn("Error moving to council area", "error", err)
		return err
	}

	// If the council is not found, move to a new council position with different offsets
	monsterNPCs := make([]npc.ID, 0)
	monsterNPCs = append(monsterNPCs, npc.CouncilMember, npc.CouncilMember2, npc.CouncilMember3)
	found := false
	for _, npcID := range monsterNPCs {
		_, found = t.ctx.Data.NPCs.FindOne(npcID)
		if found {
			break
		}
	}
	if !found {
		t.ctx.Logger.Warn("Council not found at initial position, trying a different position")
		compellingOrb, foundOrb := t.ctx.Data.Objects.FindOne(object.CompellingOrb)
		if foundOrb {
			err = action.MoveToCoords(compellingOrb.Position)
			if err != nil {
				t.ctx.Logger.Warn("Error moving to alternate council area", "error", err)
				return err
			}
		} else {
			t.ctx.Logger.Warn("Compelling Orb not found for alternate council position")
			return errors.New("council not found and compelling orb not found for alternate position")
		}
	}

	if err := t.ctx.Char.KillCouncil(); err != nil {
		return err
	}

	action.ItemPickup(30)

	t.ctx.CurrentGame.AreaCorrection.Enabled = false

	if IsQuestRun(parameters) {
		if !t.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
			compellingorb, found := t.ctx.Data.Objects.FindOne(object.CompellingOrb)
			if !found {
				return errors.New("compelling orb not found")
			}

			if err := action.MoveToCoords(compellingorb.Position); err != nil {
				return err
			}
			action.ClearAreaAroundPosition(t.ctx.Data.PlayerUnit.Position, 20)
			if err := action.MoveToCoords(compellingorb.Position); err != nil {
				return err
			}

			if err := action.ReturnTown(); err != nil {
				return err
			}

			if err := t.prepareWill(); err != nil {
				return err
			}
			if err := t.equipWill(); err != nil {
				return err
			}
			if err := action.UsePortalInTown(); err != nil {
				return err
			}
			utils.PingSleep(utils.Critical, 500)
			if err := t.smashOrb(); err != nil {
				return err
			}
			utils.Sleep(12000)
		}
		if err := t.tryReachDuranceWp(); err != nil {
			return err
		}
	}
	return nil
}

func (t *Travincal) findCouncilPosition() data.Position {
	for _, al := range t.ctx.Data.AdjacentLevels {
		if al.Area == area.DuranceOfHateLevel1 {
			return data.Position{
				X: al.Position.X - 1,
				Y: al.Position.Y + 4,
			}
		}
	}

	return data.Position{}
}

func (t Travincal) prepareWill() error {
	return prepareKhalimsWill(t.ctx)
}

func (t Travincal) equipWill() error {
	if t.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		return nil
	}
	_, _, err := ensureQuestWeaponEquipped(t.ctx, "KhalimsWill", swapWeaponSlot)
	return err
}

func (t Travincal) smashOrb() error {
	if t.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		return nil
	}

	// Interact with the Compelling Orb to open the stairs
	compellingorb, found := t.ctx.Data.Objects.FindOne(object.CompellingOrb)
	if !found {
		return errors.New("compelling orb not found")
	}

	if _, _, err := ensureQuestWeaponEquipped(t.ctx, "KhalimsWill", swapWeaponSlot); err != nil {
		return err
	}

	if err := action.MoveToCoords(compellingorb.Position); err != nil {
		return err
	}
	return withQuestWeaponSlot(t.ctx, "KhalimsWill", func() error {
		if err := action.InteractObject(compellingorb, func() bool {
			o, _ := t.ctx.Data.Objects.FindOne(object.CompellingOrb)
			return !o.Selectable
		}); err != nil {
			return err
		}
		utils.Sleep(300)
		return nil
	})
}

func (t Travincal) tryReachDuranceWp() error {
	// Interact with the stairs to go to Durance of Hate Level 1
	_, found := t.ctx.Data.Objects.FindOne(object.StairSR)
	if !found && !t.ctx.Data.Quests[quest.Act3KhalimsWill].Completed() {
		return errors.New("failed to open the Durance stairs")
	}
	if !found {
		t.ctx.Logger.Debug("Stairs to Durance not found")
	}

	err := action.MoveToArea(area.DuranceOfHateLevel1)
	if err != nil {
		return err
	}

	// Move to Durance of Hate Level 2 and discover the waypoint
	err = action.MoveToArea(area.DuranceOfHateLevel2)
	if err != nil {
		return err
	}
	err = action.DiscoverWaypoint()
	if err != nil {
		return err
	}
	return nil
}
