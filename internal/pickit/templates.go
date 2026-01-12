package pickit

import "fmt"

// GetRuleTemplates returns pre-built rule templates
func GetRuleTemplates() []RuleTemplate {
	return []RuleTemplate{
		// Ring Templates
		{
			ID:          "fcr_ring",
			Name:        "FCR Ring",
			Category:    "Rings",
			Description: "Ring with Faster Cast Rate, useful for casters",
			Rule: PickitRule{
				ItemName: "FCR Ring",
				ItemID:   "ring",
				LeftConditions: []Condition{
					{Property: "type", Operator: "==", Value: "ring"},
					{Property: "quality", Operator: "==", Value: "rare"},
				},
				RightConditions: []Condition{
					{Property: "fcr", Operator: ">=", Value: 10},
				},
				Enabled:  true,
				Priority: 70,
			},
			Examples: []string{"Rare ring with 10+ FCR"},
		},
		{
			ID:          "ar_life_ring",
			Name:        "AR + Life Ring",
			Category:    "Rings",
			Description: "Ring with Attack Rating and Life for melee characters",
			Rule: PickitRule{
				ItemName: "AR/Life Ring",
				ItemID:   "ring",
				LeftConditions: []Condition{
					{Property: "type", Operator: "==", Value: "ring"},
					{Property: "quality", Operator: "==", Value: "rare"},
				},
				RightConditions: []Condition{
					{Property: "tohit", Operator: ">=", Value: 100},
					{Property: "maxhp", Operator: ">=", Value: 40},
				},
				Enabled:  true,
				Priority: 70,
			},
			Examples: []string{"Rare ring with 100+ AR and 40+ Life"},
		},
		{
			ID:          "dual_leech_ring",
			Name:        "Dual Leech Ring",
			Category:    "Rings",
			Description: "Ring with both Life and Mana leech",
			Rule: PickitRule{
				ItemName: "Dual Leech Ring",
				ItemID:   "ring",
				LeftConditions: []Condition{
					{Property: "type", Operator: "==", Value: "ring"},
					{Property: "quality", Operator: "==", Value: "rare"},
				},
				RightConditions: []Condition{
					{Property: "lifeleech", Operator: ">=", Value: 3},
					{Property: "manaleech", Operator: ">=", Value: 3},
				},
				Enabled:  true,
				Priority: 80,
			},
			Examples: []string{"Rare ring with 3%+ LL and 3%+ ML"},
		},

		// Amulet Templates
		{
			ID:          "casting_amulet",
			Name:        "Casting Amulet",
			Category:    "Amulets",
			Description: "Amulet with FCR and resistances",
			Rule: PickitRule{
				ItemName: "Casting Amulet",
				ItemID:   "amulet",
				LeftConditions: []Condition{
					{Property: "type", Operator: "==", Value: "amulet"},
					{Property: "quality", Operator: "==", Value: "rare"},
				},
				RightConditions: []Condition{
					{Property: "fcr", Operator: ">=", Value: 10},
				},
				IsScored:       true,
				ScoreThreshold: 40,
				ScoreWeights: map[string]float64{
					"fcr":          2.0,
					"strength":     1.0,
					"fireresist":   0.5,
					"coldresist":   0.5,
					"lightresist":  0.5,
					"poisonresist": 0.5,
				},
				Enabled:  true,
				Priority: 75,
			},
			Examples: []string{"Rare amulet with FCR and good stats"},
		},

		// Charm Templates
		{
			ID:          "skill_gc",
			Name:        "Skill Grand Charm",
			Category:    "Charms",
			Description: "Grand charm with +1 to skill tree",
			Rule: PickitRule{
				ItemName: "Skill GC",
				ItemID:   "grandcharm",
				LeftConditions: []Condition{
					{Property: "name", Operator: "==", Value: "grandcharm"},
					{Property: "quality", Operator: "==", Value: "magic"},
				},
				RightConditions: []Condition{
					{Property: "coldskilltab", Operator: ">=", Value: 1},
				},
				Enabled:  true,
				Priority: 90,
				Comments: "Adjust skill tab as needed",
			},
			Examples: []string{"Cold Skills GC", "Lightning Skills GC", "Fire Skills GC"},
		},
		{
			ID:          "life_gc",
			Name:        "Life Grand Charm",
			Category:    "Charms",
			Description: "Grand charm with significant life bonus",
			Rule: PickitRule{
				ItemName: "Life GC",
				ItemID:   "grandcharm",
				LeftConditions: []Condition{
					{Property: "name", Operator: "==", Value: "grandcharm"},
					{Property: "quality", Operator: "==", Value: "magic"},
				},
				RightConditions: []Condition{
					{Property: "maxhp", Operator: ">=", Value: 35},
				},
				Enabled:  true,
				Priority: 80,
			},
			Examples: []string{"35+ Life GC"},
		},
		{
			ID:          "res_life_sc",
			Name:        "Resist + Life Small Charm",
			Category:    "Charms",
			Description: "Small charm with resistance and life",
			Rule: PickitRule{
				ItemName: "Res/Life SC",
				ItemID:   "smallcharm",
				LeftConditions: []Condition{
					{Property: "name", Operator: "==", Value: "smallcharm"},
					{Property: "quality", Operator: "==", Value: "magic"},
				},
				IsScored:       true,
				ScoreThreshold: 30,
				ScoreWeights: map[string]float64{
					"fireresist":   1.0,
					"coldresist":   1.0,
					"lightresist":  1.0,
					"poisonresist": 1.0,
					"maxhp":        0.5,
				},
				Enabled:  true,
				Priority: 70,
			},
			Examples: []string{"5/20 SC", "11 Fire Res SC"},
		},

		// Jewel Templates
		{
			ID:          "ias_ed_jewel",
			Name:        "IAS/ED Jewel",
			Category:    "Jewels",
			Description: "Jewel with Attack Speed and Enhanced Damage",
			Rule: PickitRule{
				ItemName: "IAS/ED Jewel",
				ItemID:   "jewel",
				LeftConditions: []Condition{
					{Property: "type", Operator: "==", Value: "jewel"},
					{Property: "quality", Operator: "==", Value: "rare"},
				},
				RightConditions: []Condition{
					{Property: "ias", Operator: ">=", Value: 15},
					{Property: "enhanceddamage", Operator: ">=", Value: 30},
				},
				Enabled:  true,
				Priority: 85,
			},
			Examples: []string{"15 IAS / 30 ED Jewel"},
		},

		// Base Item Templates
		{
			ID:          "monarch_4os",
			Name:        "Monarch 4 Socket",
			Category:    "Bases",
			Description: "4 socket Monarch for Spirit runeword",
			Rule: PickitRule{
				ItemName: "Monarch 4os",
				ItemID:   "monarchbase",
				LeftConditions: []Condition{
					{Property: "name", Operator: "==", Value: "monarch"},
					{Property: "quality", Operator: "==", Value: "normal"},
				},
				RightConditions: []Condition{
					{Property: "sockets", Operator: "==", Value: 4},
				},
				Enabled:     true,
				Priority:    90,
				MaxQuantity: 3,
			},
			Examples: []string{"White 4os Monarch"},
		},
		{
			ID:          "eth_thresher",
			Name:        "Ethereal Thresher",
			Category:    "Bases",
			Description: "Ethereal Thresher for Infinity/Insight",
			Rule: PickitRule{
				ItemName: "Eth Thresher",
				ItemID:   "thresher",
				LeftConditions: []Condition{
					{Property: "name", Operator: "==", Value: "thresher"},
					{Property: "flag", Operator: "==", Value: "ethereal"},
				},
				Enabled:     true,
				Priority:    95,
				MaxQuantity: 2,
			},
			Examples: []string{"Ethereal Thresher"},
		},
	}
}

// GetStatPresets returns common stat preset combinations
func GetStatPresets() []StatPreset {
	return []StatPreset{
		{
			ID:          "caster_ring",
			Name:        "Caster Ring",
			ItemType:    "ring",
			Description: "Ideal caster ring stats",
			Stats: map[string]float64{
				"fcr":      10,
				"strength": 10,
				"maxhp":    40,
				"maxmana":  40,
			},
			Weights: map[string]float64{
				"fcr":      20,
				"strength": 8,
				"maxhp":    4,
				"maxmana":  4,
			},
			Examples: []string{"10 FCR, 10 Str, 40 Life ring"},
		},
		{
			ID:          "melee_ring",
			Name:        "Melee Ring",
			ItemType:    "ring",
			Description: "Ideal melee ring stats",
			Stats: map[string]float64{
				"tohit":     120,
				"maxdamage": 9,
				"lifeleech": 4,
				"strength":  10,
			},
			Weights: map[string]float64{
				"tohit":     0.5,
				"maxdamage": 10,
				"lifeleech": 15,
				"strength":  8,
			},
			Examples: []string{"AR, Max Dmg, Leech ring"},
		},
		{
			ID:          "all_res_amulet",
			Name:        "All Resist Amulet",
			ItemType:    "amulet",
			Description: "High resistance amulet",
			Stats: map[string]float64{
				"fireresist":   15,
				"coldresist":   15,
				"lightresist":  15,
				"poisonresist": 15,
			},
			Weights: map[string]float64{
				"fireresist":   1,
				"coldresist":   1,
				"lightresist":  1,
				"poisonresist": 1,
			},
			Examples: []string{"60+ all res amulet"},
		},
	}
}

// GetAutoSuggestions generates automatic suggestions for a rule
func GetAutoSuggestions(rule *PickitRule) []AutoSuggestion {
	suggestions := []AutoSuggestion{}

	// Suggest adding quality filter if missing
	hasQuality := false
	for _, cond := range rule.LeftConditions {
		if cond.Property == "quality" {
			hasQuality = true
			break
		}
	}

	if !hasQuality {
		suggestions = append(suggestions, AutoSuggestion{
			Type:        "add_condition",
			Title:       "Add Quality Filter",
			Description: "Consider specifying item quality (magic, rare, unique, etc.)",
			Action:      "add_quality",
			Priority:    80,
		})
	}

	// Suggest maxquantity for common items
	if rule.MaxQuantity == 0 {
		suggestions = append(suggestions, AutoSuggestion{
			Type:        "add_limit",
			Title:       "Set Quantity Limit",
			Description: "Consider setting a max quantity to avoid filling your stash",
			Action:      "add_maxquantity",
			Priority:    60,
		})
	}

	// Suggest stat requirements for broad rules
	if len(rule.LeftConditions) <= 2 && len(rule.RightConditions) == 0 && !rule.IsScored {
		suggestions = append(suggestions, AutoSuggestion{
			Type:        "add_stats",
			Title:       "Add Stat Requirements",
			Description: "This rule is very broad. Add stat requirements to be more selective",
			Action:      "add_stat_conditions",
			Priority:    90,
		})
	}

	// Suggest using scored rules for complex conditions
	if len(rule.RightConditions) > 3 && !rule.IsScored {
		suggestions = append(suggestions, AutoSuggestion{
			Type:        "convert_to_scored",
			Title:       "Convert to Scored Rule",
			Description: "Consider using a scored rule for better flexibility",
			Action:      "convert_scored",
			Priority:    70,
		})
	}

	return suggestions
}

// DetectConflicts detects conflicts between rules
func DetectConflicts(rules []PickitRule) []ConflictDetection {
	conflicts := []ConflictDetection{}

	// Check for duplicate rules
	seen := make(map[string][]string)
	for _, rule := range rules {
		key := rule.ItemName
		seen[key] = append(seen[key], rule.ID)
	}

	for itemName, ruleIDs := range seen {
		if len(ruleIDs) > 1 {
			conflicts = append(conflicts, ConflictDetection{
				Type:        "duplicate",
				Rules:       ruleIDs,
				Severity:    "warning",
				Description: fmt.Sprintf("Multiple rules for '%s'", itemName),
				Suggestion:  "Consider merging these rules or adjusting priorities",
			})
		}
	}

	return conflicts
}
