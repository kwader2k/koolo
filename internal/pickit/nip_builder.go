package pickit

import (
	"fmt"
	"strings"
	"time"
)

// NIPBuilder handles conversion between visual rules and NIP syntax
type NIPBuilder struct{}

// NewNIPBuilder creates a new NIP builder
func NewNIPBuilder() *NIPBuilder {
	return &NIPBuilder{}
}

// GenerateNIP converts a PickitRule to NIP syntax
func (b *NIPBuilder) GenerateNIP(rule *PickitRule) (string, error) {
	if rule == nil {
		return "", fmt.Errorf("rule cannot be nil")
	}

	var parts []string

	// Build left side (before #)
	leftSide := b.buildLeftConditions(rule.LeftConditions)
	if leftSide == "" {
		return "", fmt.Errorf("rule must have at least one left condition")
	}
	parts = append(parts, leftSide)

	// Build right side (after #) - stats/scored conditions
	if len(rule.RightConditions) > 0 || rule.IsScored {
		parts = append(parts, "#")

		if rule.IsScored {
			// Build scored rule
			scoredSyntax := b.buildScoredConditions(rule)
			parts = append(parts, scoredSyntax)
		} else {
			// Build regular stat conditions
			rightSide := b.buildRightConditions(rule.RightConditions)
			if rightSide != "" {
				parts = append(parts, rightSide)
			}
		}
	}

	// Add max quantity if specified
	if rule.MaxQuantity > 0 {
		parts = append(parts, fmt.Sprintf("# [maxquantity] == %d", rule.MaxQuantity))
	}

	nipLine := strings.Join(parts, " ")

	// Add comments if any
	if rule.Comments != "" {
		nipLine = nipLine + " // " + rule.Comments
	}

	return nipLine, nil
}

// buildLeftConditions builds the left side of NIP rule (before #)
func (b *NIPBuilder) buildLeftConditions(conditions []Condition) string {
	if len(conditions) == 0 {
		return ""
	}

	var parts []string
	for _, cond := range conditions {
		syntax := b.conditionToNIP(cond)
		if syntax != "" {
			parts = append(parts, syntax)
		}
	}

	return strings.Join(parts, " && ")
}

// buildRightConditions builds the right side of NIP rule (after #)
func (b *NIPBuilder) buildRightConditions(conditions []Condition) string {
	if len(conditions) == 0 {
		return ""
	}

	var parts []string
	for _, cond := range conditions {
		syntax := b.conditionToNIP(cond)
		if syntax != "" {
			parts = append(parts, syntax)
		}
	}

	return strings.Join(parts, " && ")
}

// buildScoredConditions builds scored rule syntax
func (b *NIPBuilder) buildScoredConditions(rule *PickitRule) string {
	if len(rule.ScoreWeights) == 0 {
		return ""
	}

	var scoreParts []string
	for stat, weight := range rule.ScoreWeights {
		statType := GetStatTypeByID(stat)
		if statType != nil {
			scoreParts = append(scoreParts, fmt.Sprintf("%s*%.1f", statType.NipProperty, weight))
		}
	}

	if len(scoreParts) == 0 {
		return ""
	}

	// Build: ([stat1]*weight1 + [stat2]*weight2 + ...) >= threshold
	scoreFormula := fmt.Sprintf("(%s) >= %.0f", strings.Join(scoreParts, " + "), rule.ScoreThreshold)
	return scoreFormula
}

// conditionToNIP converts a single condition to NIP syntax
func (b *NIPBuilder) conditionToNIP(cond Condition) string {
	property := cond.Property
	operator := cond.Operator
	value := cond.Value

	// Compatibility: older templates/UI used "eddmg" for Enhanced Damage.
	// The NIP engine expects [enhanceddamage], but we keep accepting/saving "eddmg" internally.
	if property == "eddmg" {
		property = "enhanceddamage"
	}
	// Handle special properties
	switch property {
	case "name", "type", "quality", "flag":
		// String properties
		return fmt.Sprintf("[%s] %s %v", property, operator, value)
	default:
		// Numeric properties
		return fmt.Sprintf("[%s] %s %v", property, operator, value)
	}
}

// ParseNIP parses a NIP line into a PickitRule
func (b *NIPBuilder) ParseNIP(nipLine string) (*PickitRule, error) {
	if nipLine == "" {
		return nil, fmt.Errorf("empty NIP line")
	}

	// Remove comments
	parts := strings.Split(nipLine, "//")
	nipLine = strings.TrimSpace(parts[0])
	comments := ""
	if len(parts) > 1 {
		comments = strings.TrimSpace(parts[1])
	}

	// Split by #
	sections := strings.Split(nipLine, "#")

	rule := &PickitRule{
		ID:        generateRuleID(),
		Enabled:   true,
		Priority:  50,
		Comments:  comments,
		CreatedAt: time.Now().Format(time.RFC3339),
		UpdatedAt: time.Now().Format(time.RFC3339),
	}

	// Parse left side (before first #)
	if len(sections) > 0 {
		leftConditions, err := b.parseConditions(sections[0])
		if err != nil {
			return nil, fmt.Errorf("error parsing left conditions: %w", err)
		}
		rule.LeftConditions = leftConditions

		// Extract item name from conditions
		for _, cond := range leftConditions {
			if cond.Property == "name" {
				rule.ItemName = fmt.Sprintf("%v", cond.Value)
				break
			}
		}
	}

	// Parse right side (stats - between first and second #)
	if len(sections) > 1 {
		rightSection := strings.TrimSpace(sections[1])

		// Check if it's a scored rule
		if strings.Contains(rightSection, "(") && strings.Contains(rightSection, "*") {
			// Scored rule
			rule.IsScored = true
			// TODO: Parse scored conditions
		} else {
			// Regular stat conditions
			rightConditions, err := b.parseConditions(rightSection)
			if err != nil {
				return nil, fmt.Errorf("error parsing right conditions: %w", err)
			}
			rule.RightConditions = rightConditions
		}
	}

	// Parse max quantity (after second #)
	if len(sections) > 2 {
		maxQtySection := strings.TrimSpace(sections[2])
		if strings.Contains(maxQtySection, "maxquantity") {
			// Parse: [maxquantity] == 5
			var qty int
			fmt.Sscanf(maxQtySection, "[maxquantity] == %d", &qty)
			rule.MaxQuantity = qty
		}
	}

	rule.GeneratedNIP = nipLine
	return rule, nil
}

// parseConditions parses conditions from a section
func (b *NIPBuilder) parseConditions(section string) ([]Condition, error) {
	section = strings.TrimSpace(section)
	if section == "" {
		return []Condition{}, nil
	}

	// Split by && for AND conditions
	conditionParts := strings.Split(section, "&&")

	var conditions []Condition
	for _, part := range conditionParts {
		part = strings.TrimSpace(part)
		if part == "" {
			continue
		}

		cond, err := b.parseCondition(part)
		if err != nil {
			return nil, err
		}
		conditions = append(conditions, cond)
	}

	return conditions, nil
}

// parseCondition parses a single condition
func (b *NIPBuilder) parseCondition(condStr string) (Condition, error) {
	condStr = strings.TrimSpace(condStr)

	// Extract property, operator, value
	// Format: [property] operator value

	// Find property (between [ and ])
	startBracket := strings.Index(condStr, "[")
	endBracket := strings.Index(condStr, "]")

	if startBracket == -1 || endBracket == -1 {
		return Condition{}, fmt.Errorf("invalid condition format: %s", condStr)
	}

	property := condStr[startBracket+1 : endBracket]
	rest := strings.TrimSpace(condStr[endBracket+1:])

	// Parse operator and value
	var operator string
	var value interface{}

	// Try different operators
	operators := []string{">=", "<=", "==", "!=", ">", "<"}
	for _, op := range operators {
		if strings.Contains(rest, op) {
			parts := strings.SplitN(rest, op, 2)
			operator = op
			value = strings.TrimSpace(parts[1])
			break
		}
	}

	if operator == "" {
		return Condition{}, fmt.Errorf("no operator found in condition: %s", condStr)
	}

	return Condition{
		Property:  property,
		Operator:  operator,
		Value:     value,
		NipSyntax: condStr,
	}, nil
}

// ValidateRule validates a pickit rule
func (b *NIPBuilder) ValidateRule(rule *PickitRule) ValidationResult {
	result := ValidationResult{
		Valid:       true,
		Errors:      []string{},
		Warnings:    []string{},
		Suggestions: []string{},
	}

	// Check if rule has left conditions
	if len(rule.LeftConditions) == 0 {
		result.Valid = false
		result.Errors = append(result.Errors, "Rule must have at least one condition")
	}

	// Check if scored rule has weights
	if rule.IsScored {
		if len(rule.ScoreWeights) == 0 {
			result.Valid = false
			result.Errors = append(result.Errors, "Scored rule must have at least one weighted stat")
		}
		if rule.ScoreThreshold <= 0 {
			result.Valid = false
			result.Errors = append(result.Errors, "Scored rule must have a positive threshold")
		}
	}

	// Warning: Very broad rules
	if len(rule.LeftConditions) == 1 && len(rule.RightConditions) == 0 && !rule.IsScored {
		result.Warnings = append(result.Warnings, "This rule is very broad and may pick up many items")
		result.Suggestions = append(result.Suggestions, "Consider adding more specific conditions or stat requirements")
	}

	// Warning: MaxQuantity on unique items
	hasUniqueQuality := false
	for _, cond := range rule.LeftConditions {
		if cond.Property == "quality" && cond.Value == "unique" {
			hasUniqueQuality = true
			break
		}
	}

	if hasUniqueQuality && rule.MaxQuantity == 0 {
		result.Warnings = append(result.Warnings, "Unique items typically don't need quantity limits")
		result.Suggestions = append(result.Suggestions, "Consider setting a reasonable max quantity (e.g., 1-5)")
	}

	// Validate stat properties exist
	for _, cond := range append(rule.LeftConditions, rule.RightConditions...) {
		if !b.isValidProperty(cond.Property) {
			result.Warnings = append(result.Warnings, fmt.Sprintf("Unknown property: [%s]", cond.Property))
		}
	}

	return result
}

// isValidProperty checks if a property name is valid
func (b *NIPBuilder) isValidProperty(property string) bool {
	validProps := []string{
		"name", "type", "quality", "flag", "sockets", "defense", "enhanceddefense",
	}

	// Check if it's in valid props
	for _, prop := range validProps {
		if prop == property {
			return true
		}
	}

	// Check if it's a stat
	statType := GetStatTypeByID(property)
	return statType != nil
}

// generateRuleID generates a unique rule ID
func generateRuleID() string {
	return fmt.Sprintf("rule_%d", time.Now().UnixNano())
}

// ExportToNIP exports rules to .nip file format
func (b *NIPBuilder) ExportToNIP(rules []PickitRule, options ExportOptions) (string, error) {
	var lines []string

	// Add header comment
	if options.IncludeComments {
		lines = append(lines, "// Koolo Pickit Rules")
		lines = append(lines, fmt.Sprintf("// Generated: %s", time.Now().Format(time.RFC3339)))
		lines = append(lines, "")
	}

	// Filter and sort rules
	filteredRules := rules
	if options.OnlyEnabled {
		filteredRules = []PickitRule{}
		for _, rule := range rules {
			if rule.Enabled {
				filteredRules = append(filteredRules, rule)
			}
		}
	}

	// Generate NIP lines
	for _, rule := range filteredRules {
		nipLine, err := b.GenerateNIP(&rule)
		if err != nil {
			return "", fmt.Errorf("error generating NIP for rule %s: %w", rule.ID, err)
		}
		lines = append(lines, nipLine)
	}

	return strings.Join(lines, "\n"), nil
}
