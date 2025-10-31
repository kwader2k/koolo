package step

import (
	"errors"
	"fmt"
	"log/slog"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data/object"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

var ErrPlayerDied = errors.New("player is dead")

func OpenPortal() error {
	ctx := context.Get()
	ctx.SetLastStep("OpenPortal")

	// Portal cooldown: Prevent rapid portal creation during lag
	// Check last portal time to avoid spam during network delays
	if !ctx.LastPortalTick.IsZero() {
		timeSinceLastPortal := time.Since(ctx.LastPortalTick)
		minPortalCooldown := time.Duration(utils.PingMultiplier(utils.Critical, 1000)) * time.Millisecond
		if timeSinceLastPortal < minPortalCooldown {
			remainingCooldown := minPortalCooldown - timeSinceLastPortal
			ctx.Logger.Debug("Portal cooldown active, waiting",
				"cooldownRemaining", remainingCooldown)
			time.Sleep(remainingCooldown)
		}
	}

	lastRun := time.Time{}
	for {
		// IMPORTANT: Check for player death at the beginning of each loop iteration
		if ctx.Data.PlayerUnit.HPPercent() <= 0 {
			return ErrPlayerDied // Player is dead, stop trying to open portal
		}

		// Pause the execution if the priority is not the same as the execution priority
		ctx.PauseIfNotPriority()

		_, found := ctx.Data.Objects.FindOne(object.TownPortal)
		if found {
			ctx.LastPortalTick = time.Now() // Update portal timestamp on success
			return nil                      // Portal found, success!
		}

		// Give some time to portal to popup before retrying...
		if time.Since(lastRun) < time.Millisecond*1000 {
			continue
		}

		ping := utils.GetCurrentPing()
		delay := utils.PingMultiplier(utils.Medium, 250)
		ctx.Logger.Debug("Opening town portal - adaptive sleep",
			slog.Int("ping_ms", ping),
			slog.Int("min_delay_ms", 250),
			slog.Int("actual_delay_ms", delay),
			slog.String("formula", fmt.Sprintf("%d + (%.1f * %d) = %d", 250, float64(utils.Medium), ping, delay)),
		)

		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.MustKBForSkill(skill.TomeOfTownPortal))
		utils.PingSleep(utils.Medium, 250) // Medium operation: Wait for tome activation
		ctx.HID.Click(game.RightButton, 300, 300)
		lastRun = time.Now()
	}
}
