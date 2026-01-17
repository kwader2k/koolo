package run

import (
	"errors"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/character"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
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

	councilPosition := t.findCouncilPosition()

	err = action.MoveToCoords(councilPosition)
	if err != nil {
		t.ctx.Logger.Warn("Error moving to council area", "error", err)
		return err
	}

	if err := t.ctx.Char.KillCouncil(); err != nil {
		return err
	}

	action.ItemPickup(30)

	t.ctx.CurrentGame.AreaCorrection.Enabled = false

	if IsQuestRun(parameters) {
		compellingorb, found := t.ctx.Data.Objects.FindOne(object.CompellingOrb)
		if !found {
			t.ctx.Logger.Debug("Compelling Orb not found")
		}

		action.MoveToCoords(compellingorb.Position)
		action.ClearAreaAroundPosition(t.ctx.Data.PlayerUnit.Position, 20)
		action.MoveToCoords(compellingorb.Position)

		action.ReturnTown()

		if err := t.prepareWill(); err != nil {
			return err
		}
		if err := t.equipWill(); err != nil {
			return err
		}
		action.UsePortalInTown()
		utils.PingSleep(utils.Critical, 500)
		if err := t.smashOrb(); err != nil {
			return err
		}
		utils.Sleep(12000)
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
				Y: al.Position.Y + 3,
			}
		}
	}

	return data.Position{}
}

func (t Travincal) prepareWill() error {
	return prepareKhalimsWill(t.ctx)
}

func (t Travincal) hasKhalimsWill() bool {
	_, found := t.ctx.Data.Inventory.Find("KhalimsWill", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	return found
}

func (t Travincal) hasAllWillIngredients() bool {
	_, found := t.ctx.Data.Inventory.Find("KhalimsEye", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if !found {
		t.ctx.Logger.Info("Khalim's Eye not found, skipping")
		return false
	}

	_, found = t.ctx.Data.Inventory.Find("KhalimsBrain", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if !found {
		t.ctx.Logger.Info("Khalim's Brain not found, skipping")
		return false
	}

	_, found = t.ctx.Data.Inventory.Find("KhalimsHeart", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if !found {
		t.ctx.Logger.Info("Khalim's Heart not found, skipping")
		return false
	}

	_, found = t.ctx.Data.Inventory.Find("KhalimsFlail", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	if !found {
		t.ctx.Logger.Info("Khalim's Flail not found, skipping")
		return false
	}

	return true
}

func (t Travincal) equipWill() error {
	if t.ctx.Data.Quests[quest.Act3TheBlackenedTemple].Completed() {
		return nil
	}
	_, _, err := ensureQuestWeaponEquipped(t.ctx, "KhalimsWill", swapWeaponSlot)
	return err
}

func (t Travincal) smashOrb() error {
	if t.ctx.Data.Quests[quest.Act3TheBlackenedTemple].Completed() {
		return nil
	}

	// Interact with the Compelling Orb to open the stairs
	compellingorb, found := t.ctx.Data.Objects.FindOne(object.CompellingOrb)
	if !found {
		t.ctx.Logger.Debug("Compelling Orb not found")
	}

	action.MoveToCoords(compellingorb.Position)
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
	if t.ctx.Data.Quests[quest.Act3TheBlackenedTemple].Completed() {
		// Interact with the stairs to go to Durance of Hate Level 1
		_, found := t.ctx.Data.Objects.FindOne(object.StairSR)
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
	return errors.New("failed to complete the blackened temple")
}
