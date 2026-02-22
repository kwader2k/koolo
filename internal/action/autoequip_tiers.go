package action

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/difficulty"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/context"
)

const (
	BaseScore     = 1.0
	BeltBaseSlots = 4
)

var (
	skillWeights = map[stat.ID]float64{
		stat.AllSkills:      200.0,
		stat.AddClassSkills: 175.0,
		stat.AddSkillTab:    125.0,
		stat.SingleSkill:    40.0,
	}

	resistWeightsMain = map[stat.ID]float64{
		stat.FireResist:      3.0,
		stat.ColdResist:      2.0,
		stat.LightningResist: 3.0,
		stat.PoisonResist:    1.0,
	}

	resistWeightsOther = map[stat.ID]float64{
		stat.MaxFireResist:          8.0,
		stat.MaxLightningResist:     8.0,
		stat.MaxColdResist:          6.0,
		stat.MaxPoisonResist:        4.0,
		stat.AbsorbFire:             2.0,
		stat.AbsorbLightning:        2.0,
		stat.AbsorbMagic:            2.0,
		stat.AbsorbCold:             2.0,
		stat.AbsorbFirePercent:      4.0,
		stat.AbsorbLightningPercent: 4.0,
		stat.AbsorbMagicPercent:     4.0,
		stat.AbsorbColdPercent:      4.0,
		stat.DamageReduced:          2.0,
		stat.DamagePercent:          3.0,
		stat.MagicDamageReduction:   2.0,
		stat.MagicResist:            2.0,
	}

	generalWeights = map[stat.ID]float64{
		stat.CannotBeFrozen:       25.0,
		stat.FasterHitRecovery:    3.0,
		stat.FasterRunWalk:        2.0,
		stat.FasterBlockRate:      2.0,
		stat.FasterCastRate:       5.0,
		stat.IncreasedAttackSpeed: 0.0,
		stat.ChanceToBlock:        2.5,
		stat.MagicFind:            1.0,
		stat.GoldFind:             0.1,
		stat.Defense:              0.05,
		stat.ManaRecovery:         2.5,
		stat.Strength:             1.0,
		stat.Dexterity:            1.0,
		stat.Vitality:             1.5,
		stat.Energy:               0.5,
		stat.MaxLife:              0.5,
		stat.MaxMana:              0.25,
		stat.ReplenishQuantity:    0.0,
		stat.ReplenishLife:        2.0,
		stat.LifePerLevel:         3.0,
		stat.ManaPerLevel:         2.0,
	}

	uniqueItemScores = map[item.Name]float64{
		item.Name(item.ThudergodsVigor):    3000.0,
		item.Name(item.SkinoftheVipermagi): 2000.0,
		item.Name(item.RunewordSmoke):      1000.0,
		item.Name(item.ArachnidMesh):       3000.0,
		item.Name(item.NosferatusCoil):     3000.0,
		item.Name(item.VerdugosHeartyCord): 3000.0,
		item.Name(item.Bladebuckle):        3000.0,
		item.Name(item.StringofEars):       3000.0,
		item.Name(item.Razortail):          3000.0,
		item.Name(item.Gloomstrap):         3000.0,
		item.Name(item.Snowclash):          3000.0,
		item.Name(item.Nightsmoke):         2000.0,
		item.Name(item.Goldwrap):           2000.0,
		item.Name(item.Snakecord):          1000.0,
		item.Name(item.LenymsCord):         0.0,
		item.Name(item.RunewordInsight):    0.0,
	}

	classWeightModifiers = map[data.Class]map[stat.ID]float64{
		data.Amazon: {
			stat.CannotBeFrozen:       75.0,
			stat.IncreasedAttackSpeed: 4.0,
			stat.FireResist:           1.0,
			stat.ColdResist:           1.0,
			stat.LightningResist:      1.0,
			stat.PoisonResist:         0.5,
			stat.FasterCastRate:       0,
			stat.ManaRecovery:         0,
			stat.Dexterity:            2.0,
			stat.ReplenishQuantity:    50.0,
		},
		data.Sorceress: {
			stat.FasterCastRate:    5.0,
			stat.Defense:           0.1,
			stat.FireResist:        1.0,
			stat.ColdResist:        1.0,
			stat.LightningResist:   1.0,
			stat.PoisonResist:      0.5,
			stat.MaxLife:           1.0,
			stat.MaxMana:           1.0,
			stat.FasterHitRecovery: 2.0,
			stat.Energy:            1.5,
		},
		data.Necromancer: {
			stat.FasterCastRate: 4.0,
			stat.MaxMana:        1.5,
			stat.Strength:       0.5,
			stat.Vitality:       1.0,
		},
		data.Paladin: {
			stat.FasterCastRate:       3.0,
			stat.IncreasedAttackSpeed: 2.0,
			stat.ChanceToBlock:        3.0,
			stat.Defense:              0.2,
		},
		data.Barbarian: {
			stat.CannotBeFrozen:       75.0,
			stat.IncreasedAttackSpeed: 4.0,
			stat.FasterCastRate:       -4.5,
			stat.ManaRecovery:         -1.5,
			stat.Strength:             2.0,
			stat.MaxLife:              2.0,
		},
		data.Druid: {
			stat.FasterCastRate:    5.0,
			stat.Defense:           0.1,
			stat.FireResist:        1.0,
			stat.ColdResist:        1.0,
			stat.LightningResist:   1.0,
			stat.PoisonResist:      0.5,
			stat.MaxLife:           1.0,
			stat.MaxMana:           1.0,
			stat.FasterHitRecovery: 2.0,
			stat.Energy:            1.5,
		},
		data.Assassin: {
			stat.CannotBeFrozen:       75.0,
			stat.IncreasedAttackSpeed: 4.0,
			stat.FasterCastRate:       -4.5,
			stat.ManaRecovery:         -1.5,
		},
	}

	mercWeights = map[stat.ID]float64{
		stat.IncreasedAttackSpeed:   3.5,
		stat.MinDamage:              3.0,
		stat.MaxDamage:              3.0,
		stat.TwoHandedMinDamage:     3.0,
		stat.TwoHandedMaxDamage:     3.0,
		stat.AttackRating:           0.1,
		stat.CrushingBlow:           3.0,
		stat.OpenWounds:             3.0,
		stat.LifeSteal:              20.0,
		stat.ReplenishLife:          2.0,
		stat.FasterHitRecovery:      3.0,
		stat.Defense:                0.05,
		stat.Strength:               1.5,
		stat.Dexterity:              1.5,
		stat.FireResist:             2.0,
		stat.ColdResist:             1.5,
		stat.LightningResist:        2.0,
		stat.PoisonResist:           1.0,
		stat.DamageReduced:          2.0,
		stat.MagicResist:            3.0,
		stat.AbsorbFirePercent:      2.7,
		stat.AbsorbColdPercent:      2.7,
		stat.AbsorbLightningPercent: 2.7,
		stat.AbsorbMagicPercent:     2.7,
	}

	beltSizes = map[string]int{
		"lbl": 2,
		"vbl": 2,
		"mbl": 3,
		"tbl": 3,
	}
)

type mercCTCWeights struct {
	StatID stat.ID
	Weight float64
	Layer  int
}

type ResistStats struct {
	Fire      int
	Cold      int
	Lightning int
	Poison    int
}

var mercCTCWeight = []mercCTCWeights{
	{StatID: stat.SkillOnAttack, Weight: 5.0, Layer: 4227},     // Amp Damage
	{StatID: stat.SkillOnAttack, Weight: 10.0, Layer: 5572},    // Decrepify
	{StatID: stat.SkillOnHit, Weight: 3.0, Layer: 4225},        // Amp Damage
	{StatID: stat.SkillOnHit, Weight: 8.0, Layer: 5572},        // Decrepify
	{StatID: stat.SkillOnGetHit, Weight: 1000.0, Layer: 17103}, // Fade
	{StatID: stat.Aura, Weight: 1000.0, Layer: 123},            // Infinity
	{StatID: stat.Aura, Weight: 100.0, Layer: 120},             // Insight
}

// Can't find a way to get this from txt files - needed for + to fire skills from Magefists, Leaf, Flickering Flame etc
var fireSkills = []skill.ID{
	// Amazon
	skill.FireArrow,
	skill.ExplodingArrow,
	skill.ImmolationArrow,
	// Assassin
	skill.FistsOfFire,
	skill.FireBlast,
	skill.WakeOfFire,
	skill.WakeOfInferno,
	// Druid
	skill.FireClaws,
	skill.Firestorm,
	skill.MoltenBoulder,
	skill.Fissure,
	skill.Volcano,
	skill.Armageddon,
	// Necromancer
	skill.FireGolem,
	skill.CorpseExplosion,
	// Paladin
	skill.HolyFire,
}

var resPenalty = map[difficulty.Difficulty]int{
	difficulty.Normal:    0,
	difficulty.Nightmare: 40,
	difficulty.Hell:      100,
}

// PlayerScore calculates overall item tier score
func PlayerScore(itm data.Item) map[item.LocationType]float64 {
	//ctx := context.Get()

	bodyLocs := itm.Desc().GetType().BodyLocs
	if len(bodyLocs) == 0 {
		return make(map[item.LocationType]float64)
	}

	// Should move valid location checks here maybe to avoid unneccessary calcs
	scores := make(map[item.LocationType]float64)

	for _, loc := range bodyLocs {
		generalScore := calculateGeneralScore(itm)
		resistScore := calculateResistScore(itm, loc)
		skillScore := calculateSkillScore(itm)

		totalScore := BaseScore + generalScore + resistScore + skillScore

		if itm.IsBroken && itm.Ethereal {
			totalScore -= 10000
		}

		scores[loc] = totalScore
	}
	//ctx.Logger.Debug(fmt.Sprintf("Item %s score: %v", itm.IdentifiedName, scores))
	return scores
}
func calculateGeneralScore(itm data.Item) float64 {

	itemName := itm.Name
	if itm.IsRuneword {
		itemName = item.Name(itm.RunewordName)
	}
	ctx := context.Get()

	// Unique item override
	if score, found := uniqueItemScores[itemName]; found {
		return score
	}

	score := BaseScore

	tierRule, _ := ctx.CharacterCfg.Runtime.Rules.EvaluateTiers(itm, ctx.CharacterCfg.Runtime.TierRules)
	if tierRule.Tier() > 0 {
		score = tierRule.Tier()
	}
	// Handle Cannot Be Frozen
	//if !ctx.Data.CanTeleport() && itm.FindStat(stat.CannotbeFrozen, 0) {
	//	if <add logic to check if another item has CBF> {
	//		score += GeneralWeights[stat.CannotbeFrozen]
	//	}
	//}

	if itm.Desc().Type == "belt" {
		beltScore := calculateBeltScore(itm)
		//ctx.Logger.Debug(fmt.Sprintf("Belt score for %s: %.1f", itm.IdentifiedName, beltScore))
		score += beltScore
	}

	// Handle sockets - this might be a bad idea becauase we won't properly use the sockets
	//if !itm.IsRuneword && !itm.HasSocketedItems() {
	//	if sockets, found := itm.FindStat(stat.NumSockets, 0); found {
	//		socketScore := float64(sockets.Value * 1)
	//		//ctx.Logger.Debug(fmt.Sprintf("Socket score for %s (%d sockets): %.1f", itm.IdentifiedName, sockets.Value, socketScore))
	//		score += socketScore
	//	}
	//}

	perLevelScore := calculatePerLevelStats(itm)
	baseStatsScore := calculateBaseStats(itm)

	score += perLevelScore + baseStatsScore
	//if score > 0 {
	//	ctx.Logger.Debug(fmt.Sprintf("Final general score for %s: %.1f (per-level: %.1f, base stats: %.1f)", itm.IdentifiedName, score, perLevelScore, baseStatsScore))
	//}
	return score
}

// Belt-specific scoring so we don't lose belt slots
func calculateBeltScore(itm data.Item) float64 {
	// Score belts based on their base type name to prioritize slots above all else.
	// The scores are set in high tiers to ensure no combination of other stats can beat a higher slot belt.
	switch itm.Name {
	case "PlatedBelt": // 16 slots
		return 3000.0
	case "Belt", "HeavyBelt": // 12 slots
		return 2000.0
	case "LightBelt": // 8 slots
		return 1000.0
	case "Sash": // 4 slots
		return 0.0
	}

	// Fallback for exceptional/elite belts which all have 16 slots
	if s, found := itm.FindStat(54, 0); found {
		if s.Value >= 16 {
			return 3000.0
		}
		if s.Value >= 12 {
			return 2000.0
		}
		if s.Value >= 8 {
			return 1000.0
		}
	}

	return 0.0
}

func getBeltSize(itm data.Item) int {
	if size := beltSizes[itm.Desc().Code]; size > 0 {
		return size
	}
	return BeltBaseSlots
}

func getCurrentBeltSize() int {
	ctx := context.Get()
	for _, item := range ctx.Data.Inventory.ByLocation(item.LocationEquipped) {
		if item.Desc().Type == "belt" {
			return beltSizes[item.Desc().Code]
		}
	}
	return 0
}

func calculatePerLevelStats(itm data.Item) float64 {
	ctx := context.Get()
	charLevel, _ := ctx.Data.PlayerUnit.FindStat(stat.Level, 0)

	lifePerlvl, _ := itm.FindStat(stat.LifePerLevel, 0)
	manaPerlvl, _ := itm.FindStat(stat.ManaPerLevel, 0)

	lifeScore := (float64(lifePerlvl.Value) / 2048) * float64(charLevel.Value) * generalWeights[stat.LifePerLevel]
	manaScore := (float64(manaPerlvl.Value) / 2048) * float64(charLevel.Value) * generalWeights[stat.ManaPerLevel]

	totalScore := lifeScore + manaScore
	//if totalScore > 0 {
	//	ctx.Logger.Debug(fmt.Sprintf("Per-level stats score for %s: %.1f (life: %.1f, mana: %.1f)", itm.IdentifiedName, totalScore, lifeScore, manaScore))
	//}
	return totalScore
}

func calculateBaseStats(itm data.Item) float64 {
	//ctx := context.Get()
	score := 0.0
	class := context.Get().Data.PlayerUnit.Class

	for statID, baseWeight := range generalWeights {
		if statData, found := itm.FindStat(statID, 0); found {
			weight := baseWeight

			// Apply class-specific modifier if it exists
			if modifiers, hasClassModifiers := classWeightModifiers[class]; hasClassModifiers {
				if modifier, hasStatModifier := modifiers[statID]; hasStatModifier {
					weight += modifier
				}
			}

			statScore := float64(statData.Value) * weight
			//ctx.Logger.Debug(fmt.Sprintf("Item: %s, Stat: %s, Value: %d, Score: %.1f",
			//	itm.IdentifiedName, statID, statData.Value, statScore))
			score += statScore
		}
	}

	//ctx.Logger.Debug(fmt.Sprintf("Total base stats score for %s: %.1f", itm.IdentifiedName, score))
	return score
}

// Resists

// calculateResistScore evaluates item resistance values and returns a weighted score
func calculateResistScore(itm data.Item, bodyloc item.LocationType) float64 {
	//ctx := context.Get()
	newResists := getItemMainResists(itm)
	mainScore := 0.0
	if newResists.Fire == 0 && newResists.Cold == 0 && newResists.Lightning == 0 && newResists.Poison == 0 {
		return 0.0
	}

	//ctx.Logger.Debug(fmt.Sprintf("(%s) New item resists - Fire: %d, Cold: %d, Lightning: %d, Poison: %d", itm.IdentifiedName, newResists.Fire, newResists.Cold, newResists.Lightning, newResists.Poison))

	// get item resists stats from olditem currently equipped on body location
	oldResists := getEquippedResists(bodyloc)
	//ctx.Logger.Debug(fmt.Sprintf("(%s) Old equipped item resists - Fire: %d, Cold: %d, Lightning: %d, Poison: %d", itm.IdentifiedName, oldResists.Fire, oldResists.Cold, oldResists.Lightning, oldResists.Poison))

	// Base resists returns what our resists would be without the equipped item (including difficulty penalty)
	baseResists := getBaseResists(oldResists)
	//ctx.Logger.Debug(fmt.Sprintf("(%s) Base resists after removing equipped item - Fire: %d, Cold: %d, Lightning: %d, Poison: %d", itm.IdentifiedName, baseResists.Fire, baseResists.Cold, baseResists.Lightning, baseResists.Poison))

	// subtract olditem resists from current total resists
	effectiveResists := calculateEffectiveResists(newResists, baseResists)
	//ctx.Logger.Debug(fmt.Sprintf("(%s) Effective resists - Fire: %d, Cold: %d, Lightning: %d, Poison: %d", itm.IdentifiedName, effectiveResists.Fire, effectiveResists.Cold, effectiveResists.Lightning, effectiveResists.Poison))

	mainScore = calculateMainResistScore(effectiveResists)

	otherScore := calculateOtherResistScore(itm)

	totalScore := mainScore + otherScore
	//ctx.Logger.Debug(fmt.Sprintf("%v - %s Total resist score: %.1f (main: %.1f, other: %.1f)", bodyloc, itm.IdentifiedName, totalScore, mainScore, otherScore))

	return totalScore
}

func getItemMainResists(itm data.Item) ResistStats {
	fr, _ := itm.FindStat(stat.FireResist, 0)
	cr, _ := itm.FindStat(stat.ColdResist, 0)
	lr, _ := itm.FindStat(stat.LightningResist, 0)
	pr, _ := itm.FindStat(stat.PoisonResist, 0)

	return ResistStats{
		Fire:      fr.Value,
		Cold:      cr.Value,
		Lightning: lr.Value,
		Poison:    pr.Value,
	}
}

func getEquippedResists(bodyloc item.LocationType) ResistStats {
	ctx := context.Get()
	var resists ResistStats
	for _, equippedItem := range ctx.Data.Inventory.ByLocation(item.LocationEquipped) {
		if equippedItem.Location.BodyLocation == bodyloc {
			fr, _ := equippedItem.FindStat(stat.FireResist, 0)
			resists.Fire = fr.Value
			cr, _ := equippedItem.FindStat(stat.ColdResist, 0)
			resists.Cold = cr.Value
			lr, _ := equippedItem.FindStat(stat.LightningResist, 0)
			resists.Lightning = lr.Value
			pr, _ := equippedItem.FindStat(stat.PoisonResist, 0)
			resists.Poison = pr.Value

			return resists
		}

	}
	return resists
}

func getBaseResists(equipped ResistStats) ResistStats {
	ctx := context.Get()

	fr, _ := ctx.Data.PlayerUnit.FindStat(stat.FireResist, 0)
	cr, _ := ctx.Data.PlayerUnit.FindStat(stat.ColdResist, 0)
	lr, _ := ctx.Data.PlayerUnit.FindStat(stat.LightningResist, 0)
	pr, _ := ctx.Data.PlayerUnit.FindStat(stat.PoisonResist, 0)

	baseRes := ResistStats{
		Fire:      fr.Value - resPenalty[ctx.CharacterCfg.Game.Difficulty] - equipped.Fire,
		Cold:      cr.Value - resPenalty[ctx.CharacterCfg.Game.Difficulty] - equipped.Cold,
		Lightning: lr.Value - resPenalty[ctx.CharacterCfg.Game.Difficulty] - equipped.Lightning,
		Poison:    pr.Value - resPenalty[ctx.CharacterCfg.Game.Difficulty] - equipped.Poison,
	}

	return baseRes
}

func calculateEffectiveResists(new, base ResistStats) ResistStats {
	const maxResist = 75

	// This prevents scoring if we're already at max res (doesn't account for items that increase max res)
	limit := ResistStats{
		Fire:      max(maxResist-base.Fire, 0),
		Cold:      max(maxResist-base.Cold, 0),
		Lightning: max(maxResist-base.Lightning, 0),
		Poison:    max(maxResist-base.Poison, 0),
	}

	effectiveRes := ResistStats{
		Fire:      min(new.Fire, limit.Fire),
		Cold:      min(new.Cold, limit.Cold),
		Lightning: min(new.Lightning, limit.Lightning),
		Poison:    min(new.Poison, limit.Poison),
	}

	return effectiveRes
}

func calculateMainResistScore(resists ResistStats) float64 {
	fireScore := float64(resists.Fire) * resistWeightsMain[stat.FireResist]
	coldScore := float64(resists.Cold) * resistWeightsMain[stat.ColdResist]
	lightScore := float64(resists.Lightning) * resistWeightsMain[stat.LightningResist]
	poisonScore := float64(resists.Poison) * resistWeightsMain[stat.PoisonResist]

	totalScore := fireScore + coldScore + lightScore + poisonScore

	//context.Get().Logger.Debug(fmt.Sprintf("Main resist score components - Fire: %.1f, Cold: %.1f, Lightning: %.1f, Poison: %.1f, Total: %.1f", fireScore, coldScore, lightScore, poisonScore, totalScore))

	return totalScore
}

func calculateOtherResistScore(itm data.Item) float64 {
	//ctx := context.Get()
	var score float64

	for statID, weight := range resistWeightsOther {
		if statData, found := itm.FindStat(statID, 0); found {
			statScore := float64(statData.Value) * weight
			//ctx.Logger.Debug(fmt.Sprintf("Item: %s, Other resist %s: value %d, weight %.1f, score %.1f", itm.IdentifiedName, statID, statData.Value, weight, statScore))
			score += statScore
		}
	}
	//if score > 0 {
	//	ctx.Logger.Debug(fmt.Sprintf("Total other resist score for %s: %.1f", itm.IdentifiedName, score))
	//}
	return score
}

// Skill calcs

func calculateSkillScore(itm data.Item) float64 {
	ctx := context.Get()
	score := 0.0

	if statData, found := itm.FindStat(stat.AllSkills, 0); found {
		allSkillScore := float64(statData.Value) * skillWeights[statData.ID]
		//ctx.Logger.Debug(fmt.Sprintf("Item: %s, +All skills: %d, weight: %.1f, score: %.1f", itm.IdentifiedName, statData.Value, skillWeights[statData.ID], allSkillScore))
		score += allSkillScore
	}

	if classSkillsStat, found := itm.FindStat(stat.AddClassSkills, int(ctx.Data.PlayerUnit.Class)); found {
		classSkillScore := float64(classSkillsStat.Value) * skillWeights[classSkillsStat.ID]
		//ctx.Logger.Debug(fmt.Sprintf("Item: %s, +Class skills: %d, weight: %.1f, score: %.1f", itm.IdentifiedName, classSkillsStat.Value, skillWeights[classSkillsStat.ID], classSkillScore))
		score += classSkillScore
	}

	tabskill := int(ctx.Data.PlayerUnit.Class)*8 + (getMaxSkillTabPage() - 1)
	if tabSkillsStat, found := itm.FindStat(stat.AddSkillTab, tabskill); found {
		tabSkillScore := float64(tabSkillsStat.Value) * skillWeights[tabSkillsStat.ID]
		//ctx.Logger.Debug(fmt.Sprintf("Item: %s, +Tab skills (tab %d): %d, weight: %.1f, score: %.1f", itm.IdentifiedName, getMaxSkillTabPage(), tabSkillsStat.Value, skillWeights[tabSkillsStat.ID], tabSkillScore))
		score += tabSkillScore
	}

	usedSkills := make([]skill.ID, 0)

	//Let's ignore 1 point wonders unless we're below level 4
	for sk, pts := range ctx.Data.PlayerUnit.Skills {
		if pts.Level > 1 {
			usedSkills = append(usedSkills, sk)
		} else if lvl, _ := ctx.Data.PlayerUnit.FindStat(stat.Level, 0); lvl.Value < 4 {
			usedSkills = append(usedSkills, sk)
		}
	}

	for _, usedSkill := range usedSkills {
		if usedSkillsStat, found := itm.FindStat(stat.SingleSkill, int(usedSkill)); found {
			usedSkillScore := float64(usedSkillsStat.Value) * skillWeights[usedSkillsStat.ID]
			//ctx.Logger.Debug(fmt.Sprintf("Item: %s, +%d to %s, weight: %.1f, score: %.1f", itm.IdentifiedName, usedSkillsStat.Value, usedSkill.Desc().Name, skillWeights[usedSkillsStat.ID], usedSkillScore))
			score += usedSkillScore
		}
	}

	if fireSkillsStat, found := itm.FindStat(stat.FireSkills, 1); found {
		// Non-Sorcs
		for sk := range ctx.Data.PlayerUnit.Skills {
			for _, fireSkill := range fireSkills {
				if sk == fireSkill {
					const fireSkillWeight = 40.0
					fireSkillScore := float64(fireSkillsStat.Value) * fireSkillWeight
					//ctx.Logger.Debug(fmt.Sprintf("Item: %s, +%d to Fire Skills, weight: %.1f, score: %.1f", itm.IdentifiedName, fireSkillsStat.Value, fireSkillWeight, fireSkillScore))
					score += fireSkillScore
				}
			}
		}
		if ctx.Data.PlayerUnit.Class == data.Sorceress && getMaxSkillTabPage() == 1 { // Sorc using Fire tree
			fireSkillScore := float64(fireSkillsStat.Value) * skillWeights[stat.AddSkillTab] // Consider it the same as '+x to Fire Skills (Sorceress only)'
			//ctx.Logger.Debug(fmt.Sprintf("Item: %s, +%d to Fire Skills, weight: %.1f, score: %.1f", itm.IdentifiedName, fireSkillsStat.Value, skillWeights[stat.AddSkillTab], fireSkillScore))
			score += fireSkillScore
		}
	}
	//if score > 0 {
	//	ctx.Logger.Debug(fmt.Sprintf("Total skill score for %s: %.1f", itm.IdentifiedName, score))
	//}
	return score
}

// MercScore calculates mercenary-specific item score
func MercScore(itm data.Item) map[item.LocationType]float64 {
	// Get all possible body locations for this item
	bodyLocs := itm.Desc().GetType().BodyLocs
	if len(bodyLocs) == 0 {
		return make(map[item.LocationType]float64)
	}

	// Should move valid location checks here maybe to avoid unneccessary calcs
	scores := make(map[item.LocationType]float64)

	for _, loc := range bodyLocs {
		score, isMetaItem := getMercenaryMetaItemScore(itm)

		if isMetaItem {
			scores[loc] = score
			continue
		}

		totalScore := BaseScore + sumElementalDamage(itm)*2.0

		// Base stats
		for statID, weight := range mercWeights {
			if statData, found := itm.FindStat(statID, 0); found {
				mercStatScore := float64(statData.Value) * weight
				totalScore += mercStatScore
			}
		}

		// Chance-to-cast
		for _, ctc := range mercCTCWeight {
			if ctcStat, found := itm.FindStat(ctc.StatID, ctc.Layer); found {
				mercCTCScore := float64(ctcStat.Value) * ctc.Weight
				totalScore += mercCTCScore
			}
		}

		scores[loc] = totalScore
	}
	//ctx.Logger.Debug(fmt.Sprintf("Item %s MERC score: %v", itm.IdentifiedName, scores))
	return scores
}

// Helper functions
func sumElementalDamage(itm data.Item) float64 {
	return sumDamageType(itm, stat.FireMinDamage, stat.FireMaxDamage) +
		sumDamageType(itm, stat.LightningMinDamage, stat.LightningMaxDamage) +
		sumDamageType(itm, stat.ColdMinDamage, stat.ColdMaxDamage) +
		sumDamageType(itm, stat.MagicMinDamage, stat.MagicMaxDamage) +
		calculatePoisonDamage(itm)
}

func sumDamageType(itm data.Item, minStat, maxStat stat.ID) float64 {
	min, _ := itm.FindStat(minStat, 0)
	max, _ := itm.FindStat(maxStat, 0)
	return float64(min.Value + max.Value)
}

func calculatePoisonDamage(itm data.Item) float64 {
	poisonMin, _ := itm.FindStat(stat.PoisonMinDamage, 0)
	return float64(poisonMin.Value) * 125.0 / 256.0
}

func getMaxSkillTabPage() int {
	ctx := context.Get()

	tabCounts := make(map[int]int)
	maxCount := 0
	maxPage := 0
	for pskill, pts := range ctx.Data.PlayerUnit.Skills {
		if page := pskill.Desc().Page; page > 0 {
			tabCounts[page] += int(pts.Level)
			if tabCounts[page] > maxCount {
				maxCount = tabCounts[page]
				maxPage = page
			}
		}
	}

	return maxPage
}

// wrapper for barb leveling
func CalculateItemScore(itm data.Item) float64 {
	return calculateGeneralScore(itm)
}
