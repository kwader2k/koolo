package run

import (
	"log/slog"

	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type Shopping struct{}

func NewShopping() *Shopping { return &Shopping{} }

func (r Shopping) Name() string { return string(config.ShoppingRun) }

func (r Shopping) CheckConditions(parameters *RunParameters) SequencerResult {
	ctx := context.Get()
	shop := ctx.Data.CharacterCfg.Shopping
	if !shop.Enabled || len(shop.SelectedVendors()) == 0 {
		return SequencerSkip
	}
	return SequencerOk
}

func (r Shopping) Run(parameters *RunParameters) error {
	ctx := context.Get()
	shop := ctx.Data.CharacterCfg.Shopping

	// Log what we are about to do for visibility
	vendors := shop.SelectedVendors()
	ctx.Logger.Info("Starting Shopping run",
		slog.Int("vendors", len(vendors)),
		slog.Int("passes", shop.RefreshesPerRun),
	)

	// Delegate to action layer; it adapts the config internally
	return action.RunShoppingFromConfig(&shop)
}
