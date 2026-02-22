package action

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/context"
)

func getMercenaryMetaItemScore(it data.Item) (float64, bool) {

	name := getItemNameForScore(it)

	ctx := context.Get()

	totalScore := 0.0

	_, tierRule := ctx.CharacterCfg.Runtime.Rules.EvaluateTiers(it, ctx.CharacterCfg.Runtime.TierRules)
	if tierRule.MercTier() > 0 {
		totalScore = tierRule.MercTier()
	}

	if score, found := MercenaryMetaHelmetScore[name]; found {
		totalScore += score
	}

	if score, found := MercenaryMetaArmorScore[name]; found {
		totalScore += score
	}

	if score, found := MercenaryMetaWeaponScore[name]; found {
		totalScore += score
	}

	if bonusCalculators, found := itemBonusScoreCalculator[name]; found {
		for _, calculator := range bonusCalculators {
			totalScore += calculator(it)
		}
	}

	if totalScore > 0 {
		return totalScore, true
	}

	return 0.0, false
}

func getItemNameForScore(it data.Item) item.Name {
	if it.IsRuneword {
		return item.Name(it.RunewordName)
	}

	if it.IdentifiedName != "" {
		return item.Name(it.IdentifiedName)
	}

	return it.Name
}

var (
	MercenaryMetaHelmetScore = map[item.Name]float64{
		item.Name(item.AndarielsVisage):        2000.0,
		item.Name(item.Stealskull):             1900.0,
		item.Name(item.CrownofThieves):         1800.0,
		item.Name(item.TalRashasHoradricCrest): 1700.0,
		item.Name(item.Vampiregaze):            1600.0,
		item.Name(item.RunewordBulwark):        1500.0,
		item.Name(item.UndeadCrown):            1400.0,
		item.Name(item.Wormskull):              1300.0,
	}

	MercenaryMetaArmorScore = map[item.Name]float64{
		item.Name(item.RunewordTreachery): 2000.0,
		item.Name(item.RunewordHustle):    1900.0,
		item.Name(item.RunewordSmoke):     1800.0,
		item.Name(item.DurielsShell):      1700.0,
	}

	MercenaryMetaWeaponScore = map[item.Name]float64{
		item.Name(item.RunewordInsight): 2000.0,
	}

	PolearmTypeScore = map[string]float64{
		// Elite
		"GiantThresher":  150.0,
		"Thresher":       140.0,
		"GreatPoleaxe":   130.0,
		"CrypticAxe":     120.0,
		"ColossusVoulge": 110.0,

		// Exceptional
		"GrimScythe":   100.0,
		"BattleScythe": 90.0,
		"BecDeCorbin":  80.0,
		"Partizan":     70.0,
		"Bill":         60.0,

		// Normal
		"WarScythe": 50.0,
		"Scythe":    40.0,
		"Halberd":   30.0,
		"Poleaxe":   20.0,
		"Voulge":    10.0,
	}

	itemBonusScoreCalculator = map[item.Name][]func(it data.Item) float64{
		item.Name(item.RunewordInsight): {getInsightDamageScore, getInsightAuraScore, getInsightWeaponTypeScore},
	}
)

func getInsightWeaponTypeScore(it data.Item) float64 {
	if score, found := PolearmTypeScore[string(it.Name)]; found {
		return score
	}

	return 0.0
}

func getInsightDamageScore(it data.Item) float64 {
	score := 0.0
	baseMaxDamageData, foundBaseMaxDamageData := it.BaseStats.FindStat(stat.TwoHandedMaxDamage, 0)
	maxDamageData, foundMaxDamageData := it.Stats.FindStat(stat.TwoHandedMaxDamage, 0)

	if foundBaseMaxDamageData && foundMaxDamageData {
		baseMaxDamage := baseMaxDamageData.Value
		maxDamage := maxDamageData.Value
		score = (float64(maxDamage) - float64(baseMaxDamage)) / float64(baseMaxDamage)
	}

	return score
}

func getInsightAuraScore(it data.Item) float64 {
	score := 0.0
	data, found := it.Stats.FindStat(stat.Aura, 120)

	if found {
		score = float64(data.Value) / 10
	}

	return score
}
