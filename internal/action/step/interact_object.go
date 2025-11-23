package step

import (
	"fmt"
	"log/slog"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/mode"
	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/town"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	maxInteractionAttempts = 5
	portalSyncDelay        = 200
	maxPortalSyncAttempts  = 15
)

// InteractObject routes to packet or mouse implementation based on config
func InteractObject(obj data.Object, isCompletedFn func() bool) error {
	ctx := context.Get()

	// For portals (blue/red), check if packet mode is enabled
	if (obj.IsPortal() || obj.IsRedPortal()) && ctx.CharacterCfg.PacketCasting.UseForTpInteraction {
		return InteractObjectPacket(obj, isCompletedFn)
	}

	// Check if we should use telekinesis for this object (Sorceress only)
	// Applies to: waypoints, chests, shrines, stashes
	if shouldUseTelekinesisForObject(obj) {
		return InteractObjectTelekinesis(obj, isCompletedFn)
	}

	// Default to mouse interaction
	return InteractObjectMouse(obj, isCompletedFn)
}

// shouldUseTelekinesisForObject checks if telekinesis should be used
func shouldUseTelekinesisForObject(obj data.Object) bool {
	ctx := context.Get()

	// Only for specific object types (waypoint, chest, shrine, stash)
	isTargetObject := obj.IsWaypoint() || obj.IsChest() || obj.IsShrine() || obj.Name == object.Bank
	if !isTargetObject {
		return false
	}

	// Import action package check (will call action.ShouldUseTelekinesis)
	// For now, inline the logic to avoid circular imports

	// Check if telekinesis is enabled in config for this Sorceress build
	tkEnabled := false
	switch ctx.CharacterCfg.Character.Class {
	case "sorceress":
		tkEnabled = ctx.CharacterCfg.Character.BlizzardSorceress.UseTelekinesis
	case "nova":
		tkEnabled = ctx.CharacterCfg.Character.NovaSorceress.UseTelekinesis
	case "lightsorc":
		tkEnabled = ctx.CharacterCfg.Character.LightningSorceress.UseTelekinesis
	case "hydraorb":
		tkEnabled = ctx.CharacterCfg.Character.HydraOrbSorceress.UseTelekinesis
	case "fireballsorc":
		tkEnabled = ctx.CharacterCfg.Character.FireballSorceress.UseTelekinesis
	case "sorceress_leveling":
		tkEnabled = ctx.CharacterCfg.Character.SorceressLeveling.UseTelekinesis
	}

	if !tkEnabled {
		ctx.Logger.Debug("Telekinesis not enabled in config",
			slog.String("class", ctx.CharacterCfg.Character.Class),
		)
		return false
	}

	// Check if character has Telekinesis skill (skill ID 43)
	tkSkill, found := ctx.Data.PlayerUnit.Skills[skill.Telekinesis]
	if !found || tkSkill.Level < 1 {
		ctx.Logger.Debug("Telekinesis skill not available",
			slog.Bool("found", found),
			slog.Uint64("level", uint64(tkSkill.Level)),
		)
		return false
	}

	// Check distance - only reject if more than 21 (will move closer in InteractObjectTelekinesis)
	distance := ctx.PathFinder.DistanceFromMe(obj.Position)

	// Distance 0-21 is acceptable, distance > 21 will be handled by moving closer
	ctx.Logger.Debug("Telekinesis conditions met",
		slog.Any("object_id", obj.Name),
		slog.Int("distance", distance),
		slog.Uint64("skill_level", uint64(tkSkill.Level)),
	)

	return true
}

// InteractObjectTelekinesis uses telekinesis packet to interact with objects
func InteractObjectTelekinesis(obj data.Object, isCompletedFn func() bool) error {
	ctx := context.Get()

	// Check if we should use packet mode or keyboard/mouse mode based on class-specific config
	usePacketMode := false
	switch ctx.CharacterCfg.Character.Class {
	case "sorceress":
		usePacketMode = ctx.CharacterCfg.Character.BlizzardSorceress.UseTelekinesisPackets
	case "nova":
		usePacketMode = ctx.CharacterCfg.Character.NovaSorceress.UseTelekinesisPackets
	case "lightsorc":
		usePacketMode = ctx.CharacterCfg.Character.LightningSorceress.UseTelekinesisPackets
	case "hydraorb":
		usePacketMode = ctx.CharacterCfg.Character.HydraOrbSorceress.UseTelekinesisPackets
	case "fireballsorc":
		usePacketMode = ctx.CharacterCfg.Character.FireballSorceress.UseTelekinesisPackets
	case "sorceress_leveling":
		usePacketMode = ctx.CharacterCfg.Character.SorceressLeveling.UseTelekinesisPackets
	}

	if usePacketMode {
		return InteractObjectTelekinesisPacket(obj, isCompletedFn)
	}
	return InteractObjectTelekinesisKeyboard(obj, isCompletedFn)
}

// InteractObjectTelekinesisPacket uses telekinesis packet to interact with objects
func InteractObjectTelekinesisPacket(obj data.Object, isCompletedFn func() bool) error {
	interactionAttempts := 0
	waitingForInteraction := false
	lastRun := time.Time{}

	ctx := context.Get()
	ctx.SetLastStep("InteractObjectTelekinesis")

	// Check if we should use packet mode for skill selection based on class-specific config
	usePacketMode := false
	switch ctx.CharacterCfg.Character.Class {
	case "sorceress":
		usePacketMode = ctx.CharacterCfg.Character.BlizzardSorceress.UseTelekinesisPackets
	case "nova":
		usePacketMode = ctx.CharacterCfg.Character.NovaSorceress.UseTelekinesisPackets
	case "lightsorc":
		usePacketMode = ctx.CharacterCfg.Character.LightningSorceress.UseTelekinesisPackets
	case "hydraorb":
		usePacketMode = ctx.CharacterCfg.Character.HydraOrbSorceress.UseTelekinesisPackets
	case "fireballsorc":
		usePacketMode = ctx.CharacterCfg.Character.FireballSorceress.UseTelekinesisPackets
	case "sorceress_leveling":
		usePacketMode = ctx.CharacterCfg.Character.SorceressLeveling.UseTelekinesisPackets
	}

	// If there is no completion check, just assume the interaction is completed after sending packet
	if isCompletedFn == nil {
		isCompletedFn = func() bool {
			return waitingForInteraction
		}
	}

	for !isCompletedFn() {
		ctx.PauseIfNotPriority()

		if interactionAttempts >= maxInteractionAttempts {
			return fmt.Errorf("[%s] failed interacting with object via telekinesis [%v] in Area: [%s]", ctx.Name, obj.Name, ctx.Data.PlayerUnit.Area.Area().Name)
		}

		ctx.RefreshGameData()

		interactionCooldown := utils.PingMultiplier(utils.Light, 200)
		if ctx.Data.PlayerUnit.Area.IsTown() {
			interactionCooldown = utils.PingMultiplier(utils.Medium, 400)
		}

		// Give some time before retrying the interaction
		if waitingForInteraction && time.Since(lastRun) < time.Duration(interactionCooldown)*time.Millisecond {
			time.Sleep(10 * time.Millisecond)
			continue
		}

		var o data.Object
		var found bool
		if obj.ID != 0 {
			o, found = ctx.Data.Objects.FindByID(obj.ID)
			if !found {
				return fmt.Errorf("object %v not found", obj)
			}
		} else {
			o, found = ctx.Data.Objects.FindOne(obj.Name)
			if !found {
				return fmt.Errorf("object %v not found", obj)
			}
		}

		lastRun = time.Now()

		// Verify distance is still in range (0-21 units)
		distance := ctx.PathFinder.DistanceFromMe(o.Position)
		if distance > 21 {
			// Too far, move closer to optimal range (around 15 units)
			ctx.Logger.Debug("Object too far for telekinesis, moving closer",
				slog.Any("object_id", o.Name),
				slog.Int("current_distance", distance),
				slog.Int("target_distance", 15),
			)
			if err := MoveTo(o.Position, WithDistanceToFinish(15)); err != nil {
				return fmt.Errorf("failed to move closer for telekinesis: %w", err)
			}
			// Refresh distance after moving
			ctx.RefreshGameData()
			distance = ctx.PathFinder.DistanceFromMe(o.Position)
			ctx.Logger.Debug("Moved closer to object",
				slog.Any("object_id", o.Name),
				slog.Int("new_distance", distance),
			)
		}

		// Switch to Telekinesis skill before interaction (only on first attempt)
		if interactionAttempts == 0 {
			ctx.Logger.Debug("Switching to Telekinesis skill")
			// Use packet skill selection if telekinesis packets are enabled for this character
			if usePacketMode && ctx.PacketSender != nil {
				if ctx.Data.PlayerUnit.RightSkill != skill.Telekinesis {
					if err := ctx.PacketSender.SelectRightSkill(skill.Telekinesis); err != nil {
						ctx.Logger.Warn("Failed to switch to Telekinesis skill via packet", "error", err)
					}
					utils.Sleep(50)
				}
			} else if err := SelectRightSkill(skill.Telekinesis); err != nil {
				ctx.Logger.Warn("Failed to switch to Telekinesis skill", "error", err)
				// Don't fail here - continue with interaction attempt
			}
			// Small delay to let skill switch register
			utils.Sleep(50)
		}

		// Send telekinesis packet
		ctx.Logger.Debug("Attempting object interaction via telekinesis",
			slog.Any("object_id", o.Name),
			slog.Int("distance", distance),
		)

		if err := ctx.PacketSender.TelekinesisInteraction(o.ID); err != nil {
			ctx.Logger.Error("Telekinesis interaction failed", "error", err)
			return fmt.Errorf("failed to interact with object via telekinesis: %w", err)
		}

		waitingForInteraction = true
		interactionAttempts++

		// Wait a bit for interaction to register
		utils.PingSleep(utils.Light, 200)
	}

	return nil
}

// InteractObjectTelekinesisKeyboard uses keyboard/mouse to cast telekinesis on objects
func InteractObjectTelekinesisKeyboard(obj data.Object, isCompletedFn func() bool) error {
	interactionAttempts := 0
	waitingForInteraction := false
	lastRun := time.Time{}

	ctx := context.Get()
	ctx.SetLastStep("InteractObjectTelekinesisKeyboard")

	// Check if we should use packet mode for skill selection based on class-specific config
	usePacketMode := false
	switch ctx.CharacterCfg.Character.Class {
	case "sorceress":
		usePacketMode = ctx.CharacterCfg.Character.BlizzardSorceress.UseTelekinesisPackets
	case "nova":
		usePacketMode = ctx.CharacterCfg.Character.NovaSorceress.UseTelekinesisPackets
	case "lightsorc":
		usePacketMode = ctx.CharacterCfg.Character.LightningSorceress.UseTelekinesisPackets
	case "hydraorb":
		usePacketMode = ctx.CharacterCfg.Character.HydraOrbSorceress.UseTelekinesisPackets
	case "fireballsorc":
		usePacketMode = ctx.CharacterCfg.Character.FireballSorceress.UseTelekinesisPackets
	case "sorceress_leveling":
		usePacketMode = ctx.CharacterCfg.Character.SorceressLeveling.UseTelekinesisPackets
	}

	// Check if Telekinesis is bound
	tkBinding, found := ctx.Data.KeyBindings.KeyBindingForSkill(skill.Telekinesis)
	if !found {
		ctx.Logger.Warn("Telekinesis skill not bound, falling back to mouse interaction")
		return InteractObjectMouse(obj, isCompletedFn)
	}

	// If there is no completion check, just assume the interaction is completed after casting
	if isCompletedFn == nil {
		isCompletedFn = func() bool {
			return waitingForInteraction
		}
	}

	for !isCompletedFn() {
		ctx.PauseIfNotPriority()

		if interactionAttempts >= maxInteractionAttempts {
			return fmt.Errorf("[%s] failed interacting with object via telekinesis keyboard [%v] in Area: [%s]", ctx.Name, obj.Name, ctx.Data.PlayerUnit.Area.Area().Name)
		}

		ctx.RefreshGameData()

		interactionCooldown := utils.PingMultiplier(utils.Light, 200)
		if ctx.Data.PlayerUnit.Area.IsTown() {
			interactionCooldown = utils.PingMultiplier(utils.Medium, 400)
		}

		// Give some time before retrying the interaction
		if waitingForInteraction && time.Since(lastRun) < time.Duration(interactionCooldown)*time.Millisecond {
			time.Sleep(10 * time.Millisecond)
			continue
		}

		var o data.Object
		var found bool
		if obj.ID != 0 {
			o, found = ctx.Data.Objects.FindByID(obj.ID)
			if !found {
				return fmt.Errorf("object %v not found", obj)
			}
		} else {
			o, found = ctx.Data.Objects.FindOne(obj.Name)
			if !found {
				return fmt.Errorf("object %v not found", obj)
			}
		}

		lastRun = time.Now()

		// Verify distance is still in range (0-21 units)
		distance := ctx.PathFinder.DistanceFromMe(o.Position)
		if distance > 21 {
			// Too far, move closer to optimal range (around 15 units)
			ctx.Logger.Debug("Object too far for telekinesis, moving closer",
				slog.Any("object_id", o.Name),
				slog.Int("current_distance", distance),
				slog.Int("target_distance", 15),
			)
			if err := MoveTo(o.Position, WithDistanceToFinish(15)); err != nil {
				return fmt.Errorf("failed to move closer for telekinesis: %w", err)
			}
			// Refresh distance after moving
			ctx.RefreshGameData()
			distance = ctx.PathFinder.DistanceFromMe(o.Position)
			ctx.Logger.Debug("Moved closer to object",
				slog.Any("object_id", o.Name),
				slog.Int("new_distance", distance),
			)
		}

		// Switch to Telekinesis skill on right-click
		// Use packet skill selection if telekinesis packets are enabled for this character (regardless of UseForSkillSelection)
		if usePacketMode && ctx.PacketSender != nil {
			if ctx.Data.PlayerUnit.RightSkill != skill.Telekinesis {
				if err := ctx.PacketSender.SelectRightSkill(skill.Telekinesis); err != nil {
					ctx.Logger.Debug("Failed to switch to Telekinesis skill via packet, attempting HID fallback", "error", err)
					if found {
						ctx.HID.PressKeyBinding(tkBinding)
					}
				}
				time.Sleep(50 * time.Millisecond)
			}
		} else if err := SelectRightSkill(skill.Telekinesis); err != nil {
			ctx.Logger.Debug("Failed to switch to Telekinesis skill, attempting HID fallback", "error", err)
			// If SelectRightSkill failed and we have a keybinding, try manual HID press
			if found {
				ctx.HID.PressKeyBinding(tkBinding)
				time.Sleep(50 * time.Millisecond)
			}
		} else {
			time.Sleep(50 * time.Millisecond)
		}

		// Cast telekinesis on the object
		ctx.Logger.Debug("Casting Telekinesis on object via keyboard/mouse",
			slog.Any("object_id", o.Name),
			slog.Int("distance", distance),
		)

		// Get screen coordinates of the object
		x, y := ui.GameCoordsToScreenCords(o.Position.X, o.Position.Y)

		// Right-click on the object to cast telekinesis
		ctx.HID.Click(game.RightButton, x, y)

		waitingForInteraction = true
		interactionAttempts++

		// Wait a bit for interaction to register
		utils.PingSleep(utils.Light, 200)
	}

	return nil
}

// InteractObjectMouse is the original mouse-based object interaction
func InteractObjectMouse(obj data.Object, isCompletedFn func() bool) error {
	interactionAttempts := 0
	mouseOverAttempts := 0
	waitingForInteraction := false
	currentMouseCoords := data.Position{}
	lastRun := time.Time{}

	ctx := context.Get()
	ctx.SetLastStep("InteractObjectMouse")

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

		if interactionAttempts >= maxInteractionAttempts || mouseOverAttempts >= 20 {
			return fmt.Errorf("[%s] failed interacting with object [%v] in Area: [%s]", ctx.Name, obj.Name, ctx.Data.PlayerUnit.Area.Area().Name)
		}

		ctx.RefreshGameData()

		interactionCooldown := utils.PingMultiplier(utils.Light, 200)
		if ctx.Data.PlayerUnit.Area.IsTown() {
			interactionCooldown = utils.PingMultiplier(utils.Medium, 400)
		}

		// Give some time before retrying the interaction
		if waitingForInteraction && time.Since(lastRun) < time.Duration(interactionCooldown)*time.Millisecond {
			time.Sleep(10 * time.Millisecond)
			continue
		}

		var o data.Object
		var found bool
		if obj.ID != 0 {
			o, found = ctx.Data.Objects.FindByID(obj.ID)
			if !found {
				return fmt.Errorf("object %v not found", obj)
			}
		} else {
			o, found = ctx.Data.Objects.FindOne(obj.Name)
			if !found {
				return fmt.Errorf("object %v not found", obj)
			}
		}

		lastRun = time.Now()

		// Check portal states
		if o.IsPortal() || o.IsRedPortal() {
			// If portal is still being created, wait with escalating delay
			if o.Mode == mode.ObjectModeOperating {
				// Use retry escalation for portal opening waits
				utils.RetrySleep(interactionAttempts, float64(ctx.Data.Game.Ping), 100)
				continue
			}

			// Only interact when portal is fully opened
			if o.Mode != mode.ObjectModeOpened {
				utils.RetrySleep(interactionAttempts, float64(ctx.Data.Game.Ping), 100)
				continue
			}
		}

		if o.IsHovered && !utils.IsZeroPosition(currentMouseCoords) {
			ctx.HID.Click(game.LeftButton, currentMouseCoords.X, currentMouseCoords.Y)

			waitingForInteraction = true
			interactionAttempts++

			// For portals with expected area, we need to wait for proper area sync
			if expectedArea != 0 {
				utils.PingSleep(utils.Medium, 500)

				maxQuickChecks := 5
				for attempts := 0; attempts < maxQuickChecks; attempts++ {
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

					utils.PingSleep(utils.Light, 100)
				}

				// Area transition didn't happen yet - reset hover state to retry portal click
				ctx.Logger.Debug("Portal click may have failed - will retry",
					slog.String("expected_area", expectedArea.Area().Name),
					slog.String("current_area", ctx.Data.PlayerUnit.Area.Area().Name),
					slog.Int("interaction_attempt", interactionAttempts),
				)
				waitingForInteraction = false
				mouseOverAttempts = 0 // Reset to find portal again
			}
			continue
		} else {
			objectX := o.Position.X - 2
			objectY := o.Position.Y - 2
			distance := ctx.PathFinder.DistanceFromMe(o.Position)
			if distance > 15 {
				return fmt.Errorf("object is too far away: %d. Current distance: %d", o.Name, distance)
			}

			mX, mY := ui.GameCoordsToScreenCords(objectX, objectY)
			// In order to avoid the spiral (super slow and shitty) let's try to point the mouse to the top of the portal directly
			if mouseOverAttempts == 2 && o.IsPortal() {
				mX, mY = ui.GameCoordsToScreenCords(objectX-4, objectY-4)
			}

			x, y := utils.Spiral(mouseOverAttempts)
			currentMouseCoords = data.Position{X: mX + x, Y: mY + y}
			ctx.HID.MovePointer(mX+x, mY+y)
			mouseOverAttempts++
		}
	}

	return nil
}
