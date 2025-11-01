package run

import (
	"time"

	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type CompanionIdle struct {
	ctx *context.Status
}

func NewCompanionIdle() *CompanionIdle {
	return &CompanionIdle{
		ctx: context.Get(),
	}
}

func (c CompanionIdle) Name() string {
	return string(config.CompanionIdleRun)
}

func (c CompanionIdle) Run() error {
	// This run does nothing - just idles in town
	// The companion will wait here briefly, then the bot's companion logic
	// (in bot.go) will automatically handle waiting until the leader leaves
	c.ctx.Logger.Info("[CompanionIdle] Companion idling in town, waiting for leader...")

	// Small sleep to prevent immediate exit, but mostly rely on bot.go's companion waiting logic
	time.Sleep(1 * time.Second)

	return nil
}
