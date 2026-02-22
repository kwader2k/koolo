package step

import (
	"fmt"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/mode"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	maxPacketInteractionAttempts = 5
	packetPortalSyncDelay        = 200
	maxPacketPortalSyncAttempts  = 15
)

func InteractObjectPacket(obj data.Object, isCompletedFn func() bool) error {
	interactionAttempts := 0
	waitingForInteraction := false
	lastRun := time.Time{}

	ctx := context.Get()
	ctx.SetLastStep("InteractObjectPacket")

	// Track starting area to detect portal transitions
	startingArea := ctx.Data.PlayerUnit.Area

	// If there is no completion check, just assume the interaction is completed after clicking
	if isCompletedFn == nil {
		isCompletedFn = func() bool {
			return waitingForInteraction
		}
	}

	// For portals, we need to ensure proper area sync
	expectedArea := area.ID(0)
	if obj.IsRedPortal() {
		// For red portals, we need to determine the expected destination
		switch {
		case obj.Name == object.PermanentTownPortal && ctx.Data.PlayerUnit.Area == area.StonyField:
			expectedArea = area.Tristram
		case obj.Name == object.PermanentTownPortal && ctx.Data.PlayerUnit.Area == area.RogueEncampment:
			expectedArea = area.MooMooFarm
		case obj.Name == object.PermanentTownPortal && ctx.Data.PlayerUnit.Area == area.Harrogath:
			expectedArea = area.NihlathaksTemple
		case obj.Name == object.PermanentTownPortal && ctx.Data.PlayerUnit.Area == area.ArcaneSanctuary:
			expectedArea = area.CanyonOfTheMagi
		case obj.Name == object.BaalsPortal && ctx.Data.PlayerUnit.Area == area.ThroneOfDestruction:
			expectedArea = area.TheWorldstoneChamber
		case obj.Name == object.DurielsLairPortal && (ctx.Data.PlayerUnit.Area >= area.TalRashasTomb1 && ctx.Data.PlayerUnit.Area <= area.TalRashasTomb7):
			expectedArea = area.DurielsLair
		}
	} else if obj.IsPortal() {
		// For blue town portals, determine the town area based on current area
		fromArea := ctx.Data.PlayerUnit.Area
		if !fromArea.IsTown() {
			expectedArea = town.GetTownByArea(fromArea).TownArea()
		} else {
			// When using portal from town, we need to wait for any non-town area
			isCompletedFn = func() bool {
				return !ctx.Data.PlayerUnit.Area.IsTown() &&
					ctx.Data.AreaData.IsInside(ctx.Data.PlayerUnit.Position) &&
					len(ctx.Data.Objects) > 0
			}
		}
	}

	for !isCompletedFn() {
		ctx.PauseIfNotPriority()

		if interactionAttempts >= maxPacketInteractionAttempts {
			return fmt.Errorf("[%s] failed interacting with object via packet [%v] in Area: [%s]", ctx.Name, obj.Name, ctx.Data.PlayerUnit.Area.Area().Name)
		}

		ctx.RefreshGameData()

		// If we've transitioned areas (portal interaction), the object no longer exists in current area
		// Stop trying to interact and let the completion function handle success
		if ctx.Data.PlayerUnit.Area != startingArea {
			// Don't return error - area transition is expected for portals
			// The isCompletedFn will determine if this was successful
			continue
		}

		// Give some time before retrying the interaction
		if waitingForInteraction && time.Since(lastRun) < time.Millisecond*200 {
			utils.Sleep(200)
			continue
		}

		var o data.Object
		var found bool
		if obj.ID != 0 {
			o, found = ctx.Data.Objects.FindByID(obj.ID)
		}
		if !found {
			// Fallback by name in case the object ID changes during sync (e.g., portal opening).
			o, found = ctx.Data.Objects.FindOne(obj.Name)
			if !found {
				return fmt.Errorf("object %v not found", obj)
			}
		}

		lastRun = time.Now()

		// Check portal states
		if o.IsPortal() || o.IsRedPortal() {
			// If portal is still being created, wait
			if o.Mode == mode.ObjectModeOperating {
				utils.Sleep(100)
				continue
			}

			// Only interact when portal is fully opened
			if o.Mode != mode.ObjectModeOpened {
				utils.Sleep(100)
				continue
			}

			// Send packet interaction
			ctx.Logger.Debug("Attempting TP interaction via packet method")
			if err := ctx.PacketSender.InteractWithTp(o); err != nil {
				ctx.Logger.Error("Packet TP interaction failed", "error", err)
				return fmt.Errorf("failed to interact with portal via packet: %w", err)
			}

			waitingForInteraction = true
			interactionAttempts++

			// For portals with expected area, we need to wait for proper area sync
			if expectedArea != 0 {
				utils.Sleep(500) // Initial delay for area transition
				for attempts := 0; attempts < maxPacketPortalSyncAttempts; attempts++ {
					ctx.RefreshGameData()
					if ctx.Data.PlayerUnit.Area == expectedArea {
						if areaData, ok := ctx.Data.Areas[expectedArea]; ok {
							if areaData.IsInside(ctx.Data.PlayerUnit.Position) {
								if expectedArea.IsTown() {
									return nil // For town areas, we can return immediately
								}
								// For special areas, ensure we have proper object data loaded
								if len(ctx.Data.Objects) > 0 {
									return nil
								}
							}
						}
					}
					utils.Sleep(packetPortalSyncDelay)
				}
				return fmt.Errorf("portal sync timeout - expected area: %v, current: %v", expectedArea, ctx.Data.PlayerUnit.Area)
			}
		} else {
			// For non-portal objects, packets are not supported yet
			return fmt.Errorf("packet interaction only supported for portals currently")
		}
	}

	return nil
}
