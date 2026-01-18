package run

import (
	"errors"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/quest"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

type Hellforge struct {
	ctx *context.Status
}

func NewHellforge() *Hellforge {
	return &Hellforge{
		ctx: context.Get(),
	}
}

func (h Hellforge) Name() string {
	return string(config.IzualRun)
}

func (h Hellforge) CheckConditions(parameters *RunParameters) SequencerResult {
	if IsFarmingRun(parameters) {
		return SequencerError
	}

	if !h.ctx.Data.Quests[quest.Act3TheGuardian].Completed() {
		return SequencerStop
	}

	if h.ctx.Data.Quests[quest.Act4HellForge].Completed() {
		return SequencerSkip
	}

	return SequencerOk
}

func (h Hellforge) Run(parameters *RunParameters) error {
	action.WayPoint(area.RiverOfFlame)

	hellforge, found := h.ctx.Data.Objects.FindOne(object.HellForge)
	if !found {
		return errors.New("couldn't find hellforge")
	}

	err := action.MoveToCoords(hellforge.Position, step.WithDistanceToFinish(20))
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(30, data.MonsterAnyFilter())

	_, corpseFound := h.ctx.Data.Corpses.FindOne(npc.Hephasto, data.MonsterTypeNone)
	if !corpseFound {
		hephasto, found := h.ctx.Data.Monsters.FindOne(npc.Hephasto, data.MonsterTypeNone)
		if found {
			h.ctx.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
				return hephasto.UnitID, true
			}, nil)
		}
	}

	action.ItemPickup(30)

	err = action.MoveToCoords(hellforge.Position)
	if err != nil {
		return err
	}

	action.ClearAreaAroundPlayer(40, data.MonsterAnyFilter())

	action.ItemPickup(40)

	err = action.MoveToCoords(hellforge.Position)
	if err != nil {
		return err
	}

	for h.ctx.PathFinder.DistanceFromMe(hellforge.Position) < 5 {
		h.ctx.PathFinder.RandomMovement()
		utils.Sleep(500)
	}

	err = action.ReturnTown()
	if err != nil {
		return err
	}

	action.InteractNPC(npc.DeckardCain4)

	if err := h.equipHammer(); err != nil {
		return err
	}

	action.UsePortalInTown()

	if !h.hasSoul() {
		return errors.New("mephisto soulstone not found")
	}

	err = h.breakStone()
	if err != nil {
		return err
	}

	start := time.Now()
	for time.Since(start) < time.Millisecond*6000 {
		action.ItemPickup(20)
		utils.Sleep(100)
	}

	action.ReturnTown()

	return nil
}

func (h Hellforge) breakStone() error {
	hellforge, found := h.ctx.Data.Objects.FindOne(object.HellForge)
	if !found {
		return errors.New("couldn't find hellforge")
	}
	err := action.InteractObject(hellforge, func() bool {
		return !h.hasSoul()
	})
	if err != nil {
		return err
	}

	return withQuestWeaponSlot(h.ctx, "HellforgeHammer", func() error {
		if err := action.InteractObject(hellforge, func() bool {
			return !h.hasHammer()
		}); err != nil {
			return err
		}
		utils.Sleep(500)
		return nil
	})
}

func (h Hellforge) hasSoul() bool {
	_, found := h.ctx.Data.Inventory.Find("MephistosSoulstone", item.LocationInventory)
	return found
}

func (h Hellforge) hasHammer() bool {
	_, found := h.ctx.Data.Inventory.Find("HellforgeHammer", item.LocationInventory, item.LocationStash, item.LocationEquipped)
	return found
}

func (h Hellforge) equipHammer() error {
	_, _, err := ensureQuestWeaponEquipped(h.ctx, "HellforgeHammer", swapWeaponSlot)
	if err == nil {
		return nil
	}

	if !h.ctx.Data.PlayerUnit.Area.IsTown() {
		return err
	}

	if _, found := h.ctx.Data.Inventory.Find("HellforgeHammer", item.LocationInventory, item.LocationStash, item.LocationSharedStash); !found {
		return err
	}

	h.ctx.Logger.Debug("Retrying Hellforge Hammer equip after clearing swap weapons")
	for _, loc := range []item.LocationType{item.LocLeftArmSecondary, item.LocRightArmSecondary} {
		equipped := action.GetEquippedItem(h.ctx.Data.Inventory, loc)
		if equipped.UnitID == 0 {
			continue
		}
		if _, unequipErr := action.EnsureItemNotEquipped(equipped); unequipErr != nil {
			return unequipErr
		}
		h.ctx.RefreshGameData()
	}

	_, _, retryErr := ensureQuestWeaponEquipped(h.ctx, "HellforgeHammer", swapWeaponSlot)
	return retryErr
}
