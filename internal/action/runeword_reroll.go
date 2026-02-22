package action

import (
	"fmt"
	"strconv"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/pickit"
)

// runewordMeetsTargetStats reports whether the item meets every configured target stat.
// With an empty target list we treat the rule as automatically satisfied.
func runewordMeetsTargetStats(itm data.Item, targets []config.RunewordTargetStatOverride) bool {
	ctx := context.Get()

	rw, hasRecipe := getRunewordByName(itm.RunewordName)
	supportsGroupedResists := false
	supportsGroupedAttributes := false
	if hasRecipe {
		supportsGroupedResists = runewordSupportsGroupedResists(rw)
		supportsGroupedAttributes = runewordSupportsGroupedAttributes(rw)
	}

	if len(targets) == 0 {
		ctx.Logger.Debug("Runeword reroll: no target stats configured for this rule; treating as always satisfied")
		return true
	}

	for _, ts := range targets {
		maxConstraint := ts.Max

		group := strings.ToLower(strings.TrimSpace(ts.Group))
		switch group {
		case rerollGroupAllResistances:
			if supportsGroupedResists {
				if !checkGroupedRunewordTargets(ctx, itm, groupedResistStatIDs, ts, "all resistances") {
					return false
				}
				continue
			}
		case rerollGroupAllAttributes:
			if supportsGroupedAttributes {
				if !checkGroupedRunewordTargets(ctx, itm, groupedAttributeStatIDs, ts, "all attributes") {
					return false
				}
				continue
			}
		default:
			if group == "" && ts.StatID == stat.Strength && ts.Layer == 0 {
				switch {
				case supportsGroupedResists:
					if !checkGroupedRunewordTargets(ctx, itm, groupedResistStatIDs, ts, "all resistances") {
						return false
					}
					continue
				case supportsGroupedAttributes:
					if !checkGroupedRunewordTargets(ctx, itm, groupedAttributeStatIDs, ts, "all attributes") {
						return false
					}
					continue
				}
			}
		}

		// Enhanced Damage can be missing as an explicit stat (depending on item/runeword).
		// We compute a best-effort value and, when a range is possible, we use the lower bound.
		switch ts.StatID {
		case stat.EnhancedDamage, stat.EnhancedDamageMin, stat.DamagePercent:
			if minED, maxED, exact, ok := GetRunewordWeaponDamageEDPercentRange(itm); ok {
				ed := minED
				if float64(ed) < ts.Min {
					ctx.Logger.Debug("Runeword reroll: weapon ED does not meet minimum; rule not satisfied",
						"statID", int(ts.StatID),
						"requiredMin", ts.Min,
						"ed", ed,
						"edMin", minED,
						"edMax", maxED,
						"edExact", exact,
					)
					return false
				}
				if maxConstraint != nil && float64(ed) > *maxConstraint {
					ctx.Logger.Debug("Runeword reroll: weapon ED exceeds maximum; rule not satisfied",
						"statID", int(ts.StatID),
						"requiredMax", *maxConstraint,
						"ed", ed,
						"edMin", minED,
						"edMax", maxED,
						"edExact", exact,
					)
					return false
				}
				continue
			}
		case stat.EnhancedDefense:
			if minED, maxED, exact, ok := GetRunewordArmorDefenseEDPercentRange(itm); ok {
				if float64(minED) < ts.Min {
					ctx.Logger.Debug("Runeword reroll: armor ED not guaranteed to meet minimum; rule not satisfied",
						"statID", int(ts.StatID),
						"requiredMin", ts.Min,
						"edMin", minED,
						"edMax", maxED,
						"edExact", exact,
					)
					return false
				}
				if maxConstraint != nil && float64(maxED) > *maxConstraint {
					ctx.Logger.Debug("Runeword reroll: armor ED not guaranteed to be <= maximum; rule not satisfied",
						"statID", int(ts.StatID),
						"requiredMax", *maxConstraint,
						"edMin", minED,
						"edMax", maxED,
						"edExact", exact,
					)
					return false
				}
				continue
			}
		case stat.Defense:
			// Runewords that add flat defense fold that into stat.Defense, so strip it out before checking.
			if minFlat, maxFlat, exact, ok := GetRunewordArmorFlatDefenseRange(itm); ok {
				if float64(minFlat) < ts.Min {
					ctx.Logger.Debug("Runeword reroll: flat defense not guaranteed to meet minimum; rule not satisfied",
						"statID", int(ts.StatID),
						"requiredMin", ts.Min,
						"flatMin", minFlat,
						"flatMax", maxFlat,
						"flatExact", exact,
					)
					return false
				}
				if maxConstraint != nil && float64(maxFlat) > *maxConstraint {
					ctx.Logger.Debug("Runeword reroll: flat defense not guaranteed to be <= maximum; rule not satisfied",
						"statID", int(ts.StatID),
						"requiredMax", *maxConstraint,
						"flatMin", minFlat,
						"flatMax", maxFlat,
						"flatExact", exact,
					)
					return false
				}
				continue
			}
		}

		st, found := itm.FindStat(ts.StatID, ts.Layer)
		if !found {
			ctx.Logger.Debug("Runeword reroll: stat missing for rule target; rule not satisfied",
				"statID", int(ts.StatID),
				"layer", ts.Layer,
			)
			return false
		}

		val := float64(st.Value)

		if val < ts.Min {
			ctx.Logger.Debug("Runeword reroll: stat below minimum; rule not satisfied",
				"statID", int(ts.StatID),
				"layer", ts.Layer,
				"value", val,
				"min", ts.Min,
			)
			return false
		}
		if maxConstraint != nil && val > *maxConstraint {
			ctx.Logger.Debug("Runeword reroll: stat above maximum; rule not satisfied",
				"statID", int(ts.StatID),
				"layer", ts.Layer,
				"value", val,
				"max", *maxConstraint,
			)
			return false
		}
	}

	ctx.Logger.Debug("Runeword reroll: all target stats satisfied for this rule")
	return true
}

func checkGroupedRunewordTargets(ctx *context.Status, itm data.Item, ids []stat.ID, ts config.RunewordTargetStatOverride, groupLabel string) bool {
	for _, id := range ids {
		st, found := itm.FindStat(id, 0)
		if !found {
			ctx.Logger.Debug("Runeword reroll: grouped stat missing for rule target",
				"group", groupLabel,
				"statID", int(id),
			)
			return false
		}

		val := float64(st.Value)
		if val < ts.Min {
			ctx.Logger.Debug("Runeword reroll: grouped stat below minimum; rule not satisfied",
				"group", groupLabel,
				"statID", int(id),
				"value", val,
				"min", ts.Min,
			)
			return false
		}
		if ts.Max != nil && val > *ts.Max {
			ctx.Logger.Debug("Runeword reroll: grouped stat above maximum; rule not satisfied",
				"group", groupLabel,
				"statID", int(id),
				"value", val,
				"max", *ts.Max,
			)
			return false
		}
	}

	return true
}

func buildRerollHistorySummary(itm data.Item, rule config.RunewordRerollRule) (string, string) {
	if len(rule.TargetStats) == 0 {
		return "None", "n/a"
	}

	rw, hasRecipe := getRunewordByName(itm.RunewordName)
	supportsGroupedResists := hasRecipe && runewordSupportsGroupedResists(rw)
	supportsGroupedAttributes := hasRecipe && runewordSupportsGroupedAttributes(rw)

	targets := make([]string, 0, len(rule.TargetStats))
	actuals := make([]string, 0, len(rule.TargetStats))

	for _, ts := range rule.TargetStats {
		label, groupTag := rerollTargetLabel(ts, supportsGroupedResists, supportsGroupedAttributes)
		targets = append(targets, formatTargetStat(label, ts))
		actuals = append(actuals, formatActualStat(itm, label, groupTag, ts))
	}

	return strings.Join(targets, ", "), strings.Join(actuals, ", ")
}

func rerollTargetLabel(ts config.RunewordTargetStatOverride, supportsGroupedResists, supportsGroupedAttributes bool) (string, string) {
	group := strings.ToLower(strings.TrimSpace(ts.Group))
	switch group {
	case rerollGroupAllResistances:
		return "All Res", rerollGroupAllResistances
	case rerollGroupAllAttributes:
		return "All Attr", rerollGroupAllAttributes
	default:
		if group == "" && ts.StatID == stat.Strength && ts.Layer == 0 {
			switch {
			case supportsGroupedResists:
				return "All Res", rerollGroupAllResistances
			case supportsGroupedAttributes:
				return "All Attr", rerollGroupAllAttributes
			}
		}
	}

	return PrettyRunewordStatLabel(ts.StatID, ts.Layer), rerollGroupSingle
}

func formatTargetStat(label string, ts config.RunewordTargetStatOverride) string {
	minVal := strconv.FormatFloat(ts.Min, 'f', -1, 64)
	if ts.Max != nil {
		maxVal := strconv.FormatFloat(*ts.Max, 'f', -1, 64)
		return fmt.Sprintf("%s %s-%s", label, minVal, maxVal)
	}
	return fmt.Sprintf("%s >=%s", label, minVal)
}

func formatActualStat(itm data.Item, label string, groupTag string, ts config.RunewordTargetStatOverride) string {
	switch groupTag {
	case rerollGroupAllResistances:
		return fmt.Sprintf("%s %s", label, formatGroupedActualStats(itm, groupedResistStatIDs, map[stat.ID]string{
			stat.FireResist:      "FR",
			stat.ColdResist:      "CR",
			stat.LightningResist: "LR",
			stat.PoisonResist:    "PR",
		}))
	case rerollGroupAllAttributes:
		return fmt.Sprintf("%s %s", label, formatGroupedActualStats(itm, groupedAttributeStatIDs, map[stat.ID]string{
			stat.Strength:  "Str",
			stat.Energy:    "Ene",
			stat.Dexterity: "Dex",
			stat.Vitality:  "Vit",
		}))
	default:
		// Some runeword rolls (notably ED% and EDef%) don't always show up as explicit stats,
		// so we derive them from base/current values when needed.
		switch ts.StatID {
		case stat.EnhancedDamage, stat.EnhancedDamageMin, stat.DamagePercent:
			if minED, _, _, ok := GetRunewordWeaponDamageEDPercentRange(itm); ok {
				return fmt.Sprintf("%s %d", label, minED)
			}
			return fmt.Sprintf("%s n/a", label)
		case stat.EnhancedDefense:
			if minED, maxED, exact, ok := GetRunewordArmorDefenseEDPercentRange(itm); ok {
				if exact {
					return fmt.Sprintf("%s %d", label, minED)
				}
				return fmt.Sprintf("%s %d-%d", label, minED, maxED)
			}
			return fmt.Sprintf("%s n/a", label)
		}

		st, found := itm.FindStat(ts.StatID, ts.Layer)
		if !found {
			return fmt.Sprintf("%s n/a", label)
		}
		return fmt.Sprintf("%s %d", label, st.Value)
	}
}

func formatGroupedActualStats(itm data.Item, ids []stat.ID, labels map[stat.ID]string) string {
	parts := make([]string, 0, len(ids))
	for _, id := range ids {
		label := labels[id]
		if label == "" {
			label = PrettyRunewordStatLabel(id, 0)
		}
		st, found := itm.FindStat(id, 0)
		if !found {
			parts = append(parts, fmt.Sprintf("%s n/a", label))
			continue
		}
		parts = append(parts, fmt.Sprintf("%s %d", label, st.Value))
	}
	return strings.Join(parts, "/")
}

func evaluateRunewordRules(ctx *context.Status, itm data.Item, rules []config.RunewordRerollRule, runewordName string) (bool, bool, *config.RunewordRerollRule) {
	desc := itm.Desc()
	baseCode := pickit.ToNIPName(desc.Name)

	applicableRuleFound := false
	meetsAnyRule := false
	var historyRule *config.RunewordRerollRule

	ctx.Logger.Debug("Runeword reroll: evaluating runeword item against reroll rules",
		"runeword", runewordName,
		"baseCode", baseCode,
	)

	for idx, rule := range rules {
		ctx.Logger.Debug("Runeword reroll: checking rule",
			"runeword", runewordName,
			"ruleIndex", idx,
			"ethMode", rule.EthMode,
			"qualityMode", rule.QualityMode,
			"baseType", rule.BaseType,
			"baseTier", rule.BaseTier,
			"baseName", rule.BaseName,
			"targetStatsCount", len(rule.TargetStats),
		)
		// EthMode filter: if set, restrict rules to eth/non-eth items.
		ethMode := strings.ToLower(strings.TrimSpace(rule.EthMode))
		switch ethMode {
		case "eth":
			if !itm.Ethereal {
				continue
			}
		case "noneth":
			if itm.Ethereal {
				continue
			}
		}

		// QualityMode filter: if set, restrict to Normal/Superior as requested.
		qualityMode := strings.ToLower(strings.TrimSpace(rule.QualityMode))
		switch qualityMode {
		case "normal":
			if itm.Quality != item.QualityNormal {
				continue
			}
		case "superior":
			if itm.Quality != item.QualitySuperior {
				continue
			}
		}

		baseNameExplicitMatch := false
		if rule.BaseName != "" {
			for _, part := range strings.Split(rule.BaseName, ",") {
				if strings.TrimSpace(part) == baseCode {
					baseNameExplicitMatch = true
					break
				}
			}
			if !baseNameExplicitMatch {
				continue
			}
		}

		// Only enforce type/tier filters when BaseName didn't already pin a list.
		if !baseNameExplicitMatch {
			if rule.BaseType != "" && desc.Type != rule.BaseType {
				continue
			}

			if rule.BaseTier != "" {
				itemTier := desc.Tier()
				switch strings.ToLower(rule.BaseTier) {
				case "normal":
					if itemTier != item.TierNormal {
						continue
					}
				case "exceptional":
					if itemTier != item.TierExceptional {
						continue
					}
				case "elite":
					if itemTier != item.TierElite {
						continue
					}
				}
			}
		}

		applicableRuleFound = true
		if historyRule == nil {
			ruleCopy := rule
			historyRule = &ruleCopy
		}
		if runewordMeetsTargetStats(itm, rule.TargetStats) {
			meetsAnyRule = true
			break
		}
	}

	return applicableRuleFound, meetsAnyRule, historyRule
}

func findRerolledRunewordItem(original data.Item, recipe Runeword, excludeIDs map[data.UnitID]struct{}) (data.Item, bool) {
	ctx := context.Get()

	items := ctx.Data.Inventory.ByLocation(
		item.LocationInventory,
		item.LocationStash,
		item.LocationSharedStash,
	)

	origDesc := original.Desc()
	origBaseCode := pickit.ToNIPName(origDesc.Name)

	// Prefer the same UnitID if the base survives unsocketing and re-socketing.
	for _, itm := range items {
		if _, excluded := excludeIDs[itm.UnitID]; excluded {
			continue
		}
		if itm.UnitID != original.UnitID {
			continue
		}
		if itm.IsRuneword && itm.RunewordName == recipe.Name {
			return itm, true
		}
	}

	for _, itm := range items {
		if _, excluded := excludeIDs[itm.UnitID]; excluded {
			continue
		}
		if !itm.IsRuneword || itm.RunewordName != recipe.Name {
			continue
		}
		desc := itm.Desc()
		if pickit.ToNIPName(desc.Name) != origBaseCode {
			continue
		}
		if itm.Ethereal != original.Ethereal {
			continue
		}
		if itm.Quality != original.Quality {
			continue
		}
		return itm, true
	}

	return data.Item{}, false
}

func buildExcludedRunewordUnitIDs(target data.Item, recipe Runeword) map[data.UnitID]struct{} {
	ctx := context.Get()

	items := ctx.Data.Inventory.ByLocation(
		item.LocationInventory,
		item.LocationStash,
		item.LocationSharedStash,
	)

	excludeIDs := make(map[data.UnitID]struct{})
	for _, itm := range items {
		if !itm.IsRuneword || itm.RunewordName != recipe.Name {
			continue
		}
		if itm.UnitID == target.UnitID {
			continue
		}
		excludeIDs[itm.UnitID] = struct{}{}
	}

	return excludeIDs
}

// hasRunesForReroll ensures stash/inventory has the Hel rune for unsocketing plus everything needed to rebuild the recipe.
func hasRunesForReroll(ctx *context.Status, recipe Runeword) bool {
	items := ctx.Data.Inventory.ByLocation(
		item.LocationInventory,
		item.LocationStash,
		item.LocationSharedStash,
	)

	ctx.Logger.Debug("Runeword reroll: checking rune availability for reroll",
		"runeword", string(recipe.Name))

	// Count required runes for the recipe
	required := make(map[string]int)
	for _, r := range recipe.Runes {
		required[r]++
	}

	// Count available runes in inventory/stash
	available := make(map[string]int)
	for _, itm := range items {
		// Runes show up in-game as items with names ending in "Rune".
		name := string(itm.Name)
		if strings.HasSuffix(name, "Rune") {
			available[name]++
		}
	}

	// We always need a Hel rune for the unsocket recipe.
	if available["HelRune"] < 1 {
		ctx.Logger.Debug("Runeword reroll: insufficient runes for reroll",
			"runeword", string(recipe.Name),
			"rune", "HelRune",
			"required", 1,
			"available", available["HelRune"],
		)
		return false
	}

	for runeName, cnt := range required {
		needed := cnt
		// For Hel runes we need an extra one for the unsocket recipe
		if runeName == "HelRune" {
			needed = cnt + 1
		}
		if available[runeName] < needed {
			ctx.Logger.Debug("Runeword reroll: insufficient runes for reroll",
				"runeword", string(recipe.Name),
				"rune", runeName,
				"required", needed,
				"available", available[runeName],
			)
			return false
		}
	}

	ctx.Logger.Debug("Runeword reroll: rune availability OK for reroll",
		"runeword", string(recipe.Name),
		"requiredRunes", fmt.Sprintf("%v", required),
		"availableRunes", fmt.Sprintf("%v", available),
	)
	return true
}

// ensureLooseTownPortalScroll finds or buys a loose TP scroll so the unsocket recipe can run.
// If there's no free scroll already, it will try two vendor refills to top off the tome and buy one more.
func ensureLooseTownPortalScroll() (data.Item, bool) {
	ctx := context.Get()

	// First, look for an existing loose scroll.
	if scroll, found := ctx.Data.Inventory.Find(item.ScrollOfTownPortal,
		item.LocationInventory,
		item.LocationStash,
		item.LocationSharedStash,
	); found {
		ctx.Logger.Debug("Runeword reroll: found existing loose TP scroll for unsocket")
		return scroll, true
	}

	// Otherwise visit a vendor a couple of times.
	for i := 0; i < 2; i++ {
		ctx.Logger.Info("Runeword reroll: no loose TP scroll; attempting VendorRefill to obtain one",
			"attempt", i+1,
		)
		if err := VendorRefill(VendorRefillOpts{ForceRefill: true, BuyConsumables: true}); err != nil {
			ctx.Logger.Warn("Runeword reroll: VendorRefill failed while trying to obtain TP scroll for unsocket",
				"attempt", i+1,
				"error", err,
			)
			return data.Item{}, false
		}

		// Re-scan inventory/stash after each run.
		if scroll, found := ctx.Data.Inventory.Find(item.ScrollOfTownPortal,
			item.LocationInventory,
			item.LocationStash,
			item.LocationSharedStash,
		); found {
			ctx.Logger.Debug("Runeword reroll: obtained loose TP scroll after VendorRefill",
				"attempt", i+1,
			)
			return scroll, true
		}
	}

	ctx.Logger.Warn("Runeword reroll: failed to obtain loose TP scroll for unsocket after VendorRefill attempts")
	return data.Item{}, false
}

// unsocketRuneword runs the (item + Hel + TP scroll) cube recipe after the caller decides a reroll is needed.
func unsocketRuneword(itm data.Item) (bool, string) {
	ctx := context.Get()

	// Find a Hel rune for unsocketing
	helRune, foundHel := ctx.Data.Inventory.Find("HelRune",
		item.LocationInventory,
		item.LocationStash,
		item.LocationSharedStash,
	)
	if !foundHel {
		ctx.Logger.Warn("Runeword reroll: cannot unsocket runeword; no Hel rune found")
		return false, "No Hel rune"
	}

	// Grab a loose Scroll of Town Portal (tomes eat the extra scroll otherwise).
	scroll, foundScroll := ensureLooseTownPortalScroll()
	if !foundScroll {
		ctx.Logger.Warn("Runeword reroll: cannot unsocket runeword; no loose TP scroll available")
		return false, "No loose TP scroll"
	}

	ctx.Logger.Info("Runeword reroll: attempting to unsocket runeword",
		"itemName", string(itm.Name),
		"runewordName", string(itm.RunewordName),
	)

	// Kick off the cube recipe.
	if err := CubeAddItems(itm, helRune, scroll); err != nil {
		ctx.Logger.Warn("Runeword reroll: failed to add items to cube for unsocket",
			"error", err,
		)
		return false, "Failed to add to cube"
	}

	if err := CubeTransmute(); err != nil {
		ctx.Logger.Warn("Runeword reroll: cube transmute failed while unsocketing runeword",
			"error", err,
		)
		return false, "Cube transmute failed"
	}

	ctx.Logger.Info("Runeword reroll: successfully unsocketed runeword item")

	// Refresh inventory and immediately hand control back to the maker so it can rebuild the item.
	ctx.RefreshGameData()

	if err := MakeRunewords(); err != nil {
		ctx.Logger.Warn("Runeword reroll: MakeRunewords failed after unsocket",
			"error", err,
		)
		return false, "MakeRunewords failed"
	} else {
		ctx.Logger.Info("Runeword reroll: invoked MakeRunewords after successful unsocket")
	}

	return true, ""
}

// RerollRunewords looks for configured reroll rules, checks owned items, and unsockets one that misses every applicable rule.
func RerollRunewords() {
	ctx := context.Get()

	if !ctx.CharacterCfg.Game.RunewordMaker.Enabled {
		ctx.Logger.Debug("Runeword reroll: runeword maker disabled; skipping")
		return
	}

	// Skip reroll logic for leveling characters so their streamlined flow stays untouched.
	if _, isLevelingChar := ctx.Char.(context.LevelingCharacter); isLevelingChar {
		ctx.Logger.Debug("Runeword reroll: skipping reroll logic for leveling character")
		return
	}

	rulesByRuneword := ctx.CharacterCfg.Game.RunewordRerollRules
	if len(rulesByRuneword) == 0 {
		ctx.Logger.Debug("Runeword reroll: no reroll rules configured; skipping")
		return
	}

	// Look for candidates only in stash/shared/inventory; avoid equipped items.
	ctx.Logger.Debug("Runeword reroll: scanning inventory/stash/shared for reroll candidates")
	items := ctx.Data.Inventory.ByLocation(
		item.LocationInventory,
		item.LocationStash,
		item.LocationSharedStash,
	)

	for _, recipe := range Runewords {
		name := string(recipe.Name)
		rules, ok := rulesByRuneword[name]
		if !ok || len(rules) == 0 {
			continue
		}

		// Collect runeword instances we actually own for this recipe.
		candidates := make([]data.Item, 0)
		for _, itm := range items {
			if !itm.IsRuneword || itm.RunewordName != recipe.Name {
				continue
			}
			candidates = append(candidates, itm)
		}

		if len(candidates) == 0 {
			continue
		}

		// Ensure we have enough runes to unsocket and recreate this runeword.
		if !hasRunesForReroll(ctx, recipe) {
			ctx.Logger.Debug("Runeword reroll: skipping runeword due to insufficient runes",
				"runeword", name,
			)
			continue
		}

		for _, itm := range candidates {
			applicableRuleFound, meetsAnyRule, historyRule := evaluateRunewordRules(ctx, itm, rules, name)

			// If no rule applied to this base, skip it (no reroll decision).
			if !applicableRuleFound {
				continue
			}

			// If it meets at least one applicable rule, do not reroll.
			if meetsAnyRule {
				continue
			}

			// At this point, the item failed all rules that apply to its base;
			// attempt to unsocket once, then stop for this tick.
			var targetSummary string
			var actualSummary string
			if historyRule != nil {
				targetSummary, actualSummary = buildRerollHistorySummary(itm, *historyRule)
			} else {
				targetSummary = "Unknown"
				actualSummary = "n/a"
			}

			excludeIDs := buildExcludedRunewordUnitIDs(itm, recipe)
			success, failureReason := unsocketRuneword(itm)
			if success {
				remade, found := findRerolledRunewordItem(itm, recipe, excludeIDs)
				if !found {
					success = false
					failureReason = "Remade item not found"
				} else {
					_, meetsAnyRuleAfter, _ := evaluateRunewordRules(ctx, remade, rules, name)
					if meetsAnyRuleAfter {
						success = true
						failureReason = ""
					} else {
						success = false
						failureReason = "Target not met"
					}

					if historyRule != nil {
						targetSummary, actualSummary = buildRerollHistorySummary(remade, *historyRule)
					}
				}
			}
			if success {
				ctx.Logger.Info("Runeword reroll: target satisfied",
					"runeword", string(itm.RunewordName),
					"targetStats", targetSummary,
					"actualStats", actualSummary,
				)
			}
			event.Send(event.RunewordReroll(
				event.Text(ctx.Name, "Runeword reroll"),
				string(itm.RunewordName),
				targetSummary,
				actualSummary,
				success,
				failureReason,
			))
			return
		}
	}
}
