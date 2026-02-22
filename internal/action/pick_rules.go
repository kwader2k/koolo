package action

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/nip"
	"github.com/hectorgimenez/koolo/internal/context"
)

// ShouldBuyByTiers is an optional hook to force a buy based on auto‑equip tier logic.
// If you already have tier logic elsewhere, assign this variable appropriately.
var ShouldBuyByTiers func(data.Item) bool

// shouldMatchRulesOnly evaluates NIP rules and tiers for shopping without any
// low‑gold fallbacks.  It returns true only when a given item matches
// strict pickit rules or better‑than‑equipped tiers.
func shouldMatchRulesOnly(i data.Item) bool {
	ctx := context.Get()

	// Evaluate tier rules (player and merc tiers).
	playerRule, mercRule := ctx.Data.CharacterCfg.Runtime.Rules.EvaluateTiers(i, ctx.Data.CharacterCfg.Runtime.TierRules)
	if playerRule.Tier() > 0.0 || mercRule.MercTier() > 0.0 {
		// If the item does not need to be identified (QualitySuperior or lower),
		// check whether it actually upgrades the equipment.
		if i.Quality <= item.QualitySuperior {
			if playerRule.Tier() > 0.0 {
				if IsBetterThanEquipped(i, false, PlayerScore) {
					return true
				}
			} else if mercRule.MercTier() > 0.0 {
				if IsBetterThanEquipped(i, true, MercScore) {
					return true
				}
			}
		} else {
			// QualityMagic or higher: pick up for later identification.
			return true
		}
	}

	// Evaluate all rules ignoring tiers.  The result can be FullMatch, Partial, or NoMatch.
	matchedRule, result := ctx.Data.CharacterCfg.Runtime.Rules.EvaluateAllIgnoreTiers(i)
	switch result {
	case nip.RuleResultNoMatch:
		return false
	case nip.RuleResultPartial:
		return true
	}

	// Blacklist the item if it exceeds quantity limits and do not pick it up.
	if doesExceedQuantity(matchedRule) {
		if !IsBlacklisted(i) {
			ctx.CurrentGame.BlacklistedItems = append(ctx.CurrentGame.BlacklistedItems, i)
		}
		return false
	}

	return true
}
