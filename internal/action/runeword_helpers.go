package action

import (
	"fmt"
	"math"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
)

// PrettyRunewordStatLabel maps the raw stat ID + layer combo to the label shown in the reroll UI.
func PrettyRunewordStatLabel(id stat.ID, layer int) string {
	switch id {
	case stat.EnhancedDamage, stat.EnhancedDamageMin:
		return "Enhanced Damage"
	case stat.EnhancedDefense:
		return "Enhanced Defense"
	case stat.MinDamage:
		return "Minimum Damage"
	case stat.MaxLife, stat.LifePerLevel:
		return "Life"
	case stat.MaxMana:
		return "Mana"
	case stat.Defense:
		return "Defense"
	case stat.IncreasedAttackSpeed:
		return "Increased Attack Speed (IAS)"
	case stat.FasterCastRate:
		return "Faster Cast Rate (FCR)"
	case stat.FasterHitRecovery:
		return "Faster Hit Recovery (FHR)"
	case stat.AttackRatingPercent:
		return "Attack Rating %"
	case stat.DemonDamagePercent:
		return "Damage vs Demons %"
	case stat.UndeadDamagePercent:
		return "Damage vs Undead %"
	case stat.EnemyLightningResist:
		return "Enemy Lightning Resist %"
	case stat.EnemyPoisonResist:
		return "Enemy Poison Resist %"
	case stat.EnemyFireResist:
		return "Enemy Fire Resist %"
	case stat.EnemyColdResist:
		return "Enemy Cold Resist %"
	case stat.FireSkillDamage:
		return "Fire Skill Damage %"
	case stat.ColdSkillDamage:
		return "Cold Skill Damage %"
	case stat.LightningSkillDamage:
		return "Lightning Skill Damage %"
	case stat.PoisonSkillDamage:
		return "Poison Skill Damage %"
	case stat.LifeSteal:
		return "Life Leech %"
	case stat.ManaSteal:
		return "Mana Leech %"
	case stat.LifeAfterEachKill:
		return "Life after Each Kill"
	case stat.MagicFind:
		return "Magic Find %"
	case stat.GoldFind:
		return "Gold Find %"
	case stat.AllSkills:
		return "+ All Skills"
	case stat.SingleSkill:
		// Layer carries the target skill; expose a couple of known values.
		switch layer {
		case 32:
			return "+ Valkyrie"
		default:
			if layer != 0 {
				return fmt.Sprintf("+ Single Skill (layer %d)", layer)
			}
			return "+ Single Skill"
		}
	case stat.NonClassSkill:
		// Some frequent layer values from popular runewords.
		switch layer {
		case 9:
			return "To Critical Strike"
		case 155:
			return "+ Battle Command"
		case 149:
			return "+ Battle Orders"
		case 146:
			return "+ Battle Cry"
		default:
			if layer != 0 {
				return fmt.Sprintf("+ Skill (layer %d)", layer)
			}
			return "+ Skill"
		}
	case stat.Aura:
		// Same idea for aura rolls.
		switch layer {
		case 100:
			return "Resist Fire Aura"
		case 103:
			return "Thorns Aura"
		case 104:
			return "Defiance Aura"
		case 109:
			return "Cleansing Aura"
		case 113:
			return "Concentration Aura"
		case 119:
			return "Sanctuary Aura"
		case 120:
			return "Meditation Aura"
		case 122:
			return "Fanaticism Aura"
		case 124:
			return "Redemption Aura"
		default:
			if layer != 0 {
				return fmt.Sprintf("Aura (layer %d)", layer)
			}
			return "Aura"
		}
	case stat.FireResist:
		return "Fire Resist %"
	case stat.ColdResist:
		return "Cold Resist %"
	case stat.LightningResist:
		return "Lightning Resist %"
	case stat.PoisonResist:
		return "Poison Resist %"
	case stat.AbsorbFire:
		return "Fire Absorb"
	case stat.AbsorbCold:
		return "Cold Absorb"
	case stat.AbsorbLightning:
		return "Lightning Absorb"
	case stat.AbsorbMagic:
		return "Magic Absorb"
	case stat.CrushingBlow:
		return "Crushing Blow %"
	case stat.Strength:
		return "Strength"
	case stat.Dexterity:
		return "Dexterity"
	case stat.Vitality:
		return "Vitality"
	case stat.ManaRecoveryBonus:
		return "Mana Regeneration %"
	}

	// Fallback to the stat ID.
	return fmt.Sprintf("%v", id)
}

// RunewordUIRoll mirrors how a stat roll is displayed in the UI, including synthetic “All Res” and “All Attributes” entries.
type RunewordUIRoll struct {
	StatID stat.ID
	Layer  int
	Min    float64
	Max    float64
	Label  string
	Group  string
}

const (
	rerollGroupSingle         = "single"
	rerollGroupAllResistances = "allResistances"
	rerollGroupAllAttributes  = "allAttributes"
)

var (
	groupedResistStatIDs    = []stat.ID{stat.FireResist, stat.ColdResist, stat.LightningResist, stat.PoisonResist}
	groupedAttributeStatIDs = []stat.ID{stat.Strength, stat.Energy, stat.Dexterity, stat.Vitality}
)

// BuildRunewordUIRolls reshapes the raw rolls from a recipe into the UI-friendly list (grouping resist/all-attr when possible).
func BuildRunewordUIRolls(rw Runeword) []RunewordUIRoll {
	var resistRolls []RunewordStatRolls
	var attrRolls []RunewordStatRolls
	var otherRolls []RunewordStatRolls

	for _, rRoll := range rw.Rolls {
		switch rRoll.StatID {
		case stat.FireResist, stat.ColdResist, stat.LightningResist, stat.PoisonResist:
			resistRolls = append(resistRolls, rRoll)
		case stat.Strength, stat.Energy, stat.Dexterity, stat.Vitality:
			attrRolls = append(attrRolls, rRoll)
		default:
			otherRolls = append(otherRolls, rRoll)
		}
	}

	var uiRolls []RunewordUIRoll

	// Plain rolls pass through untouched.
	for _, rRoll := range otherRolls {
		label := PrettyRunewordStatLabel(rRoll.StatID, rRoll.Layer)
		uiRolls = append(uiRolls, RunewordUIRoll{
			StatID: rRoll.StatID,
			Layer:  rRoll.Layer,
			Min:    rRoll.Min,
			Max:    rRoll.Max,
			Label:  label,
			Group:  rerollGroupSingle,
		})
	}

	// Merge fire/cold/light/poison res into a single entry when they share a range.
	if len(resistRolls) > 0 {
		if ok, minVal, maxVal := groupedStatRange(rw, groupedResistStatIDs); ok {
			uiRolls = append(uiRolls, RunewordUIRoll{
				StatID: 0,
				Layer:  0,
				Min:    minVal,
				Max:    maxVal,
				Label:  "All Resistances",
				Group:  rerollGroupAllResistances,
			})
		} else {
			for _, rr := range resistRolls {
				label := PrettyRunewordStatLabel(rr.StatID, rr.Layer)
				uiRolls = append(uiRolls, RunewordUIRoll{
					StatID: rr.StatID,
					Layer:  rr.Layer,
					Min:    rr.Min,
					Max:    rr.Max,
					Label:  label,
					Group:  rerollGroupSingle,
				})
			}
		}
	}

	// Same for STR/ENE/DEX/VIT.
	if len(attrRolls) > 0 {
		if ok, minVal, maxVal := groupedStatRange(rw, groupedAttributeStatIDs); ok {
			uiRolls = append(uiRolls, RunewordUIRoll{
				StatID: 0,
				Layer:  0,
				Min:    minVal,
				Max:    maxVal,
				Label:  "All Attributes",
				Group:  rerollGroupAllAttributes,
			})
		} else {
			for _, ar := range attrRolls {
				label := PrettyRunewordStatLabel(ar.StatID, ar.Layer)
				uiRolls = append(uiRolls, RunewordUIRoll{
					StatID: ar.StatID,
					Layer:  ar.Layer,
					Min:    ar.Min,
					Max:    ar.Max,
					Label:  label,
					Group:  rerollGroupSingle,
				})
			}
		}
	}

	return uiRolls
}

func groupedStatRange(rw Runeword, ids []stat.ID) (bool, float64, float64) {
	stats := make(map[stat.ID]RunewordStatRolls, len(ids))
	for _, roll := range rw.Rolls {
		for _, id := range ids {
			if roll.StatID == id {
				stats[id] = roll
				break
			}
		}
	}

	if len(stats) != len(ids) {
		return false, 0, 0
	}

	min := stats[ids[0]].Min
	max := stats[ids[0]].Max
	for _, id := range ids[1:] {
		rr := stats[id]
		if rr.Min != min || rr.Max != max {
			return false, 0, 0
		}
	}

	return true, min, max
}

func runewordSupportsGroupedResists(rw Runeword) bool {
	ok, _, _ := groupedStatRange(rw, groupedResistStatIDs)
	return ok
}

func runewordSupportsGroupedAttributes(rw Runeword) bool {
	ok, _, _ := groupedStatRange(rw, groupedAttributeStatIDs)
	return ok
}

// PrettyRunewordBaseTypeLabel translates the raw item type code into the dropdown label.
func PrettyRunewordBaseTypeLabel(code string) string {
	switch code {
	case item.TypeArmor:
		return "Armor"
	case item.TypeShield:
		return "Shield"
	case item.TypeVoodooHeads:
		return "Necromancer Head"
	case item.TypeAuricShields:
		return "Paladin Shield"
	case item.TypeGrimoire:
		return "Grimoire"
	case item.TypeAmazonItem, item.TypeAmazonBow:
		return "Amazon Weapon"
	case item.TypeBow:
		return "Bow"
	case item.TypeCrossbow:
		return "Crossbow"
	case item.TypeSword:
		return "Sword"
	case item.TypeAxe:
		return "Axe"
	case item.TypeMace, item.TypeClub:
		return "Mace / Club"
	case item.TypeHammer:
		return "Hammer"
	case item.TypeScepter:
		return "Scepter"
	case item.TypeWand:
		return "Wand"
	case item.TypeKnife:
		return "Dagger / Knife"
	case item.TypeSpear:
		return "Spear"
	case item.TypePolearm:
		return "Polearm"
	case item.TypeStaff:
		return "Staff"
	case item.TypeHelm:
		return "Helm"
	case item.TypePelt:
		return "Druid Helm"
	case item.TypePrimalHelm:
		return "Barbarian Helm"
	case item.TypeCirclet:
		return "Circlet"
	case item.TypeHandtoHand, item.TypeHandtoHand2:
		return "Claw"
	}

	// Fallback: best-effort title case of the raw code.
	if code == "" {
		return "Unknown"
	}
	return strings.Title(code)
}

func findRunewordArmorEDPercentExact(it data.Item) (value int, ok bool) {
	if ed, found := it.Stats.FindStat(stat.EnhancedDefense, 0); found {
		return ed.Value, true
	}
	if ed, found := findStatAnyLayerInItem(it, stat.EnhancedDefense); found {
		return ed.Value, true
	}
	return 0, false
}

func getRunewordRollRange(runeword item.RunewordName, statID stat.ID, layer int) (min int, max int, ok bool) {
	for _, rw := range Runewords {
		if rw.Name != runeword {
			continue
		}
		for _, roll := range rw.Rolls {
			if roll.StatID != statID {
				continue
			}
			if roll.Layer != layer {
				continue
			}

			min = int(math.Ceil(roll.Min))
			max = int(math.Floor(roll.Max))
			if min > max {
				return 0, 0, false
			}
			return min, max, true
		}
	}

	return 0, 0, false
}

func getRunewordByName(runeword item.RunewordName) (Runeword, bool) {
	for _, rw := range Runewords {
		if rw.Name == runeword {
			return rw, true
		}
	}
	return Runeword{}, false
}

func flatDefenseFromRunewordRunes(it data.Item) int {
	rw, ok := getRunewordByName(it.RunewordName)
	if !ok {
		return 0
	}

	// Rune socket bonuses depend on the base; for ED math we only care about flat defense from armor-ish items.
	isArmorLike := it.Type().IsType(item.TypeArmor) ||
		it.Type().IsType(item.TypeHelm) ||
		it.Type().IsType(item.TypePelt) ||
		it.Type().IsType(item.TypePrimalHelm) ||
		it.Type().IsType(item.TypeCirclet) ||
		it.Type().IsType(item.TypeShield) ||
		it.Type().IsType(item.TypeAuricShields)

	if !isArmorLike {
		return 0
	}

	flat := 0
	for _, r := range rw.Runes {
		// El rune grants +15 Defense in armor/helm/shield.
		if r == "ElRune" {
			flat += 15
		}
	}

	return flat
}

func findItemEDPercent(it data.Item) (value int, ok bool) {
	// Prefer layer 0 when present.
	if ed, found := it.FindStat(stat.EnhancedDamageMin, 0); found {
		return ed.Value, true
	}
	if ed, found := it.FindStat(stat.EnhancedDamage, 0); found {
		return ed.Value, true
	}
	if ed, found := it.FindStat(stat.DamagePercent, 0); found {
		return ed.Value, true
	}

	// Some items store ED under a non-zero layer; scan all layers (stats + basestats).
	if ed, found := findStatAnyLayerInItem(it, stat.EnhancedDamageMin); found {
		return ed.Value, true
	}
	if ed, found := findStatAnyLayerInItem(it, stat.EnhancedDamage); found {
		return ed.Value, true
	}
	if ed, found := findStatAnyLayerInItem(it, stat.DamagePercent); found {
		return ed.Value, true
	}

	// Some runewords (and occasionally other items) do not expose ED% as an explicit stat entry,
	// but the resulting weapon damage stats still reflect the roll. As a fallback, try to derive
	// ED% from base vs current damage.
	if minED, _, exact, derived := deriveWeaponEDPercentFromDamageStats(it); derived {
		if exact {
			return minED, true
		}
		// Best-effort: collapse ranges to the lower bound (more conservative / pessimistic).
		// This keeps the output stable (no "206-207"), while ensuring we don't overestimate ED.
		return minED, true
	}

	return 0, false
}

// deriveWeaponEDPercentFromDamageStats tries to infer on-weapon ED% by comparing base damage
// (BaseStats/Desc) to current damage (Stats/BaseStats).
func deriveWeaponEDPercentFromDamageStats(it data.Item) (min int, max int, exact bool, ok bool) {
	// Prefer 2H damage when present.
	baseMax, curMax, baseMin, curMin, has2H := getDamagePair(it, stat.TwoHandedMaxDamage, stat.TwoHandedMinDamage, true)
	if !has2H {
		baseMax, curMax, baseMin, curMin, _ = getDamagePair(it, stat.MaxDamage, stat.MinDamage, false)
	}

	if baseMax <= 0 || curMax <= 0 {
		return 0, 0, false, false
	}

	minMax, maxMax, okMax := edPercentRangeFromBaseCurrent(baseMax, curMax)
	if !okMax {
		return 0, 0, false, false
	}
	min = minMax
	max = maxMax

	// If we can also compute from min damage, intersect. If intersection is empty (common when the
	// item adds flat min/max damage), fall back to the tighter of the two ranges.
	if baseMin > 0 && curMin > 0 {
		minMin, maxMin, okMin := edPercentRangeFromBaseCurrent(baseMin, curMin)
		if okMin {
			iMin := min
			if minMin > iMin {
				iMin = minMin
			}
			iMax := max
			if maxMin < iMax {
				iMax = maxMin
			}
			if iMin <= iMax {
				min, max = iMin, iMax
			} else {
				// Pick tighter range.
				widthMax := maxMax - minMax
				widthMin := maxMin - minMin
				if widthMin < widthMax {
					min, max = minMin, maxMin
				} else {
					min, max = minMax, maxMax
				}
			}
		}
	}

	exact = min == max
	return min, max, exact, true
}

func getDamagePair(it data.Item, maxID, minID stat.ID, preferDesc2H bool) (baseMax, curMax, baseMin, curMin int, ok bool) {
	// Base: prefer BaseStats, fallback to Stats.
	if s, found := it.BaseStats.FindStat(maxID, 0); found {
		baseMax = s.Value
	} else if s, found := it.Stats.FindStat(maxID, 0); found {
		baseMax = s.Value
	}
	if s, found := it.BaseStats.FindStat(minID, 0); found {
		baseMin = s.Value
	} else if s, found := it.Stats.FindStat(minID, 0); found {
		baseMin = s.Value
	}

	// Current: prefer Stats, fallback to BaseStats.
	if s, found := it.Stats.FindStat(maxID, 0); found {
		curMax = s.Value
	} else if s, found := it.BaseStats.FindStat(maxID, 0); found {
		curMax = s.Value
	}
	if s, found := it.Stats.FindStat(minID, 0); found {
		curMin = s.Value
	} else if s, found := it.BaseStats.FindStat(minID, 0); found {
		curMin = s.Value
	}

	// If base damage is missing, fall back to the item description (template/base values).
	// This is not perfect for superior/eth bases, but it's better than failing completely.
	if baseMax <= 0 {
		d := it.Desc()
		if preferDesc2H && d.TwoHandMaxDamage > 0 {
			baseMax = d.TwoHandMaxDamage
		} else if d.MaxDamage > 0 {
			baseMax = d.MaxDamage
		}
	}
	if baseMin <= 0 {
		d := it.Desc()
		if preferDesc2H && d.TwoHandMinDamage > 0 {
			baseMin = d.TwoHandMinDamage
		} else if d.MinDamage > 0 {
			baseMin = d.MinDamage
		}
	}

	// Current max is required to do anything useful.
	ok = baseMax > 0 && curMax > 0
	return baseMax, curMax, baseMin, curMin, ok
}

// edPercentRangeFromBaseCurrent returns the possible ED% integer range that could result in the
// observed current value given a base value, using integer truncation semantics.
func edPercentRangeFromBaseCurrent(base, current int) (minED int, maxED int, ok bool) {
	if base <= 0 || current <= 0 {
		return 0, 0, false
	}

	// x = 100 + ed
	// current = floor(base * x / 100)
	// => current <= base*x/100 < current+1
	// => 100*current/base <= x < 100*(current+1)/base
	// We need integer x, so:
	//   minX = ceil(100*current/base)
	//   maxX = floor((100*(current+1)-1)/base)
	// ed = x - 100
	numerMin := int64(100) * int64(current)
	minX := int((numerMin + int64(base) - 1) / int64(base))

	numerMax := int64(100)*(int64(current)+1) - 1
	maxX := int(numerMax / int64(base))

	minED = minX - 100
	maxED = maxX - 100
	if minED < 0 {
		minED = 0
	}
	if maxED < 0 {
		maxED = 0
	}
	if minED > maxED {
		return 0, 0, false
	}
	return minED, maxED, true
}

func findStatAnyLayerInItem(it data.Item, id stat.ID) (stat.Data, bool) {
	for _, s := range it.Stats {
		if s.ID == id {
			return s, true
		}
	}
	for _, s := range it.BaseStats {
		if s.ID == id {
			return s, true
		}
	}
	return stat.Data{}, false
}

// GetRunewordWeaponDamageEDPercentRange returns a best-effort weapon ED% value.
// Historically this returned an ED% range derived from base/current damage.
// To avoid unstable UI outputs like "206-207", we always collapse to a single value (min==max),
// picking the lower bound when only a range can be derived.
func GetRunewordWeaponDamageEDPercentRange(it data.Item) (min int, max int, exact bool, ok bool) {
	if ed, found := findItemEDPercent(it); found {
		return ed, ed, true, true
	}
	return 0, 0, false, false
}

// GetRunewordArmorDefenseEDPercentRange derives the armor ED% range, accounting for any flat +defense rolls.
func GetRunewordArmorDefenseEDPercentRange(it data.Item) (min int, max int, exact bool, ok bool) {
	// If explicit ED is present anywhere, it's exact.
	if ed, found := findRunewordArmorEDPercentExact(it); found {
		return ed, ed, true, true
	}

	baseDef, okBase := it.BaseStats.FindStat(stat.Defense, 0)
	curDef, okCur := it.Stats.FindStat(stat.Defense, 0)
	if !okBase || !okCur || baseDef.Value == 0 {
		return 0, 0, false, false
	}

	// Pull out the +defense provided by the runes used to make the runeword (El in Fortitude, etc.).
	runeFlat := flatDefenseFromRunewordRunes(it)

	// If the runeword has its own flat roll, union the feasible ranges across that span.
	flatMin, flatMax, hasFlat := getRunewordRollRange(it.RunewordName, stat.Defense, 0)
	if hasFlat {
		foundAny := false
		for flat := flatMin; flat <= flatMax; flat++ {
			adjusted := curDef.Value - runeFlat - flat
			if adjusted < 0 {
				continue
			}
			m, x, okOne := edPercentRangeFromBaseCurrent(baseDef.Value, adjusted)
			if !okOne {
				continue
			}
			if !foundAny {
				min, max = m, x
				foundAny = true
				continue
			}
			if m < min {
				min = m
			}
			if x > max {
				max = x
			}
		}
		if !foundAny {
			return 0, 0, false, false
		}
		exact = (min == max)
		return min, max, exact, true
	}

	min, max, ok = edPercentRangeFromBaseCurrent(baseDef.Value, curDef.Value-runeFlat)
	if !ok {
		return 0, 0, false, false
	}
	exact = (min == max)
	return min, max, exact, true
}

// GetRunewordArmorFlatDefenseRange backs into the flat-defense roll range that matches the observed stats.
// The reroll logic only marks a rule as satisfied when that entire range is inside the rule's bounds.
func GetRunewordArmorFlatDefenseRange(it data.Item) (min int, max int, exact bool, ok bool) {
	baseDef, okBase := it.BaseStats.FindStat(stat.Defense, 0)
	curDef, okCur := it.Stats.FindStat(stat.Defense, 0)
	if !okBase || !okCur || baseDef.Value == 0 {
		return 0, 0, false, false
	}

	// Only meaningful for runewords that actually roll flat +Defense.
	flatMin, flatMax, hasFlat := getRunewordRollRange(it.RunewordName, stat.Defense, 0)
	if !hasFlat {
		return 0, 0, false, false
	}

	runeFlat := flatDefenseFromRunewordRunes(it)

	foundAny := false
	for flat := flatMin; flat <= flatMax; flat++ {
		adjusted := curDef.Value - runeFlat - flat
		if adjusted < 0 {
			continue
		}
		_, _, okOne := edPercentRangeFromBaseCurrent(baseDef.Value, adjusted)
		if !okOne {
			continue
		}
		if !foundAny {
			min, max = flat, flat
			foundAny = true
			continue
		}
		if flat < min {
			min = flat
		}
		if flat > max {
			max = flat
		}
	}

	if !foundAny {
		return 0, 0, false, false
	}

	exact = (min == max)
	return min, max, exact, true
}
