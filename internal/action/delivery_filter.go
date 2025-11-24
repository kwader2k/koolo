package action

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/koolo/internal/context"
)

// IsDeliveryProtected determines which items must NOT be dropped
func IsDeliveryProtected(i data.Item) bool {
	ctx := context.Get()
	selected := false
	deliverOnly := false
	filtersEnabled := false

	if ctx != nil && ctx.Context != nil {
		if ctx.Context.Delivery != nil {
			filtersEnabled = ctx.Context.Delivery.DeliveryFiltersEnabled()
			if filtersEnabled {
				selected = ctx.Context.Delivery.ShouldDeliverItem(string(i.Name))
				deliverOnly = ctx.Context.Delivery.DeliverOnlySelected()
			}
		}
	}

	// Always keep the cube so the bot can continue farming afterward.
	if i.Name == "HoradricCube" {
		return true
	}

	if selected {
		if ctx != nil && ctx.Context != nil && ctx.Context.Delivery != nil && !ctx.Context.Delivery.HasRemainingDeliveryQuota(string(i.Name)) {
			return true
		}
		return false
	}

	// Keep recipe materials configured in cube settings.
	if shouldKeepRecipeItem(i) {
		return true
	}

	if !filtersEnabled {
		return false
	}

	if deliverOnly {
		return true
	}

	// Everything else should be dropped for delivery to ensure the stash empties fully.
	return false
}
