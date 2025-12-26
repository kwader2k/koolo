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
	case item.TypeAuricShields:
		return "Paladin Shield"
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

func findStatAnyLayer(stats stat.Stats, id stat.ID) (stat.Data, bool) {
	for _, s := range stats {
		if s.ID == id {
			return s, true
		}
	}
	return stat.Data{}, false
}

func findRunewordWeaponEDPercentExact(it data.Item) (value int, ok bool) {
	// Prefer layer 0 if present.
	if ed, found := it.Stats.FindStat(stat.EnhancedDamageMin, 0); found {
		return ed.Value, true
	}
	if ed, found := it.Stats.FindStat(stat.EnhancedDamage, 0); found {
		return ed.Value, true
	}
	if ed, found := it.Stats.FindStat(stat.DamagePercent, 0); found {
		return ed.Value, true
	}

	// Some stats may appear under a non-zero layer; scan all layers.
	if ed, found := findStatAnyLayer(it.Stats, stat.EnhancedDamageMin); found {
		return ed.Value, true
	}
	if ed, found := findStatAnyLayer(it.Stats, stat.EnhancedDamage); found {
		return ed.Value, true
	}
	if ed, found := findStatAnyLayer(it.Stats, stat.DamagePercent); found {
		return ed.Value, true
	}

	return 0, false
}

func findRunewordArmorEDPercentExact(it data.Item) (value int, ok bool) {
	if ed, found := it.Stats.FindStat(stat.EnhancedDefense, 0); found {
		return ed.Value, true
	}
	if ed, found := findStatAnyLayer(it.Stats, stat.EnhancedDefense); found {
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

func edPercentRangeFromBaseCurrent(base, current int) (minED int, maxED int, ok bool) {
	if base <= 0 || current < 0 {
		return 0, 0, false
	}

	// current = floor(base * (100 + ED) / 100)
	// => 100*current/base <= (100+ED) < 100*(current+1)/base
	// Integer ED lives in [ceil(...), floor(...)] - 100.
	a := int64(100) * int64(current)
	b := int64(base)
	minX := (a + b - 1) / b
	maxX := (int64(100)*int64(current+1) - 1) / b
	minED = int(minX - 100)
	maxED = int(maxX - 100)
	if minED > maxED {
		return 0, 0, false
	}
	return minED, maxED, true
}

// GetRunewordWeaponDamageEDPercentRange derives the weapon ED% range implied by base/current dmg.
func GetRunewordWeaponDamageEDPercentRange(it data.Item) (min int, max int, exact bool, ok bool) {
	// If explicit ED is present anywhere, it's exact.
	if ed, found := findRunewordWeaponEDPercentExact(it); found {
		return ed, ed, true, true
	}

	// Prefer 2H rolls when available.
	baseMax, okBaseMax := it.BaseStats.FindStat(stat.TwoHandedMaxDamage, 0)
	curMax, okCurMax := it.Stats.FindStat(stat.TwoHandedMaxDamage, 0)
	baseMin, okBaseMin := it.BaseStats.FindStat(stat.TwoHandedMinDamage, 0)
	curMin, okCurMin := it.Stats.FindStat(stat.TwoHandedMinDamage, 0)

	if !okBaseMax || !okCurMax {
		baseMax, okBaseMax = it.BaseStats.FindStat(stat.MaxDamage, 0)
		curMax, okCurMax = it.Stats.FindStat(stat.MaxDamage, 0)
		baseMin, okBaseMin = it.BaseStats.FindStat(stat.MinDamage, 0)
		curMin, okCurMin = it.Stats.FindStat(stat.MinDamage, 0)
	}

	if !okBaseMax || !okCurMax || baseMax.Value == 0 {
		return 0, 0, false, false
	}

	minMax, maxMax, okMax := edPercentRangeFromBaseCurrent(baseMax.Value, curMax.Value)
	if !okMax {
		return 0, 0, false, false
	}

	min = minMax
	max = maxMax

	// Intersect with the min-damage range when we can.
	if okBaseMin && okCurMin && baseMin.Value != 0 {
		minMin, maxMin, okMin := edPercentRangeFromBaseCurrent(baseMin.Value, curMin.Value)
		if okMin {
			if minMin > min {
				min = minMin
			}
			if maxMin < max {
				max = maxMin
			}
		}
	}

	if min > max {
		return 0, 0, false, false
	}

	exact = (min == max)
	return min, max, exact, true
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
