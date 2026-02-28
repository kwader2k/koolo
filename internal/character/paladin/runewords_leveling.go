package paladin

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"hash"
	"hash/fnv"
	"log/slog"
	"sort"
	"strconv"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/nip"
	"github.com/hectorgimenez/koolo/internal/config"
)

const paladinLevelingDynamicRuleSource = "dynamic:paladin_leveling"

type paladinLevelingRunewordConfig struct {
	Facts []StateFact `json:"facts,omitempty"`
	Plan  StatePlan   `json:"plan"`
}

//go:embed runewords_leveling_config.json
var paladinLevelingRunewordConfigRaw []byte

var paladinLevelingRunewordsConfig = mustLoadPaladinLevelingRunewordsConfig()

func mustLoadPaladinLevelingRunewordsConfig() paladinLevelingRunewordConfig {
	cfg := paladinLevelingRunewordConfig{}
	if err := json.Unmarshal(paladinLevelingRunewordConfigRaw, &cfg); err != nil {
		panic(fmt.Sprintf("failed to parse paladin leveling runeword config: %v", err))
	}

	if err := ValidateStatePlan(cfg.Plan); err != nil {
		panic(fmt.Sprintf("invalid paladin leveling runeword config: %v", err))
	}
	if err := ValidateFacts(cfg.Facts); err != nil {
		panic(fmt.Sprintf("invalid paladin leveling runeword facts: %v", err))
	}
	if err := ValidateStateReferences(cfg.Facts, cfg.Plan); err != nil {
		panic(fmt.Sprintf("invalid paladin leveling runeword condition wiring: %v", err))
	}

	return cfg
}

func (p *PaladinLeveling) updateLevelingRunewords() {
	p.CharacterCfg.Game.RunewordMaker.Enabled = true
	if p.runewordRuleInjector == nil {
		p.runewordRuleInjector = NewDynamicRuleInjector(paladinLevelingDynamicRuleSource)
	}

	baseState, err := EvaluateFacts(
		FactContext{
			CanUseSkill:     p.CanUseSkill,
			Inventory:       p.Data.Inventory.AllItems,
			RuntimeRules:    p.CharacterCfg.Runtime.Rules,
			TierRuleIndexes: p.CharacterCfg.Runtime.TierRules,
		},
		paladinLevelingRunewordsConfig.Facts,
	)
	if err != nil {
		p.Logger.Error("Failed to evaluate runeword facts", "error", err)
		baseState = State{}
	}

	evaluation := EvaluateStatePlan(paladinLevelingRunewordsConfig.Plan, baseState)

	p.CharacterCfg.Game.RunewordMaker.EnabledRecipes = evaluation.EnabledRecipes
	p.runewordRuleInjector.Apply(p.CharacterCfg, p.Logger, evaluation.DynamicRuleLines)
}

type State map[string]bool

type RuleSelection string

const (
	SelectAllValid   RuleSelection = "all_valid"
	SelectFirstValid RuleSelection = "first_valid"
)

type StateCondition struct {
	All  []string `json:"all,omitempty"`
	Any  []string `json:"any,omitempty"`
	None []string `json:"none,omitempty"`
}

func (c StateCondition) Matches(state State) bool {
	for _, key := range c.All {
		if !state[key] {
			return false
		}
	}

	if len(c.Any) > 0 {
		anyMatched := false
		for _, key := range c.Any {
			if state[key] {
				anyMatched = true
				break
			}
		}
		if !anyMatched {
			return false
		}
	}

	for _, key := range c.None {
		if state[key] {
			return false
		}
	}

	return true
}

type StateRule struct {
	RuleLine string         `json:"ruleLine"`
	When     StateCondition `json:"when,omitempty"`
}

type StateRunewordTarget struct {
	Runeword  item.RunewordName `json:"runeword"`
	When      StateCondition    `json:"when,omitempty"`
	BaseRules []StateRule       `json:"baseRules,omitempty"`
	Selection RuleSelection     `json:"selection,omitempty"`
}

type StatePlan struct {
	Targets    []StateRunewordTarget `json:"targets,omitempty"`
	ExtraRules []StateRule           `json:"extraRules,omitempty"`
}

type Evaluation struct {
	EnabledRecipes   []string
	DynamicRuleLines []string
}

func EvaluateStatePlan(plan StatePlan, state State) Evaluation {
	enabledRecipes := make([]string, 0, len(plan.Targets))
	enabledRunewordsSet := make(map[string]struct{}, len(plan.Targets))

	appendEnabledRecipe := func(name item.RunewordName) {
		recipeName := string(name)
		if recipeName == "" {
			return
		}
		if _, exists := enabledRunewordsSet[recipeName]; exists {
			return
		}
		enabledRunewordsSet[recipeName] = struct{}{}
		enabledRecipes = append(enabledRecipes, recipeName)
	}

	dynamicRuleLines := make([]string, 0, len(plan.Targets)+len(plan.ExtraRules))
	dynamicRuleSet := make(map[string]struct{}, len(plan.Targets)+len(plan.ExtraRules))

	appendRuleLine := func(line string) {
		line = strings.TrimSpace(line)
		if line == "" {
			return
		}
		if _, exists := dynamicRuleSet[line]; exists {
			return
		}
		dynamicRuleSet[line] = struct{}{}
		dynamicRuleLines = append(dynamicRuleLines, line)
	}

	for _, target := range plan.Targets {
		if !target.When.Matches(state) {
			continue
		}

		appendEnabledRecipe(target.Runeword)

		for _, baseRule := range target.BaseRules {
			if !baseRule.When.Matches(state) {
				continue
			}

			appendRuleLine(baseRule.RuleLine)
			if normalizeSelection(target.Selection) == SelectFirstValid {
				break
			}
		}
	}

	for _, extraRule := range plan.ExtraRules {
		if !extraRule.When.Matches(state) {
			continue
		}
		appendRuleLine(extraRule.RuleLine)
	}

	return Evaluation{
		EnabledRecipes:   enabledRecipes,
		DynamicRuleLines: dynamicRuleLines,
	}
}

func ValidateStatePlan(plan StatePlan) error {
	for i, target := range plan.Targets {
		selection := normalizeSelection(target.Selection)
		if selection != SelectAllValid && selection != SelectFirstValid {
			return fmt.Errorf("invalid selection for target %d (%s): %s", i, target.Runeword, target.Selection)
		}
	}
	return nil
}

func ValidateStateReferences(facts []StateFact, plan StatePlan) error {
	known := make(map[string]struct{}, len(facts))

	for _, fact := range facts {
		known[fact.Name] = struct{}{}
	}

	referenced := make(map[string]struct{})
	addRefs := func(condition StateCondition) {
		for _, ref := range condition.All {
			ref = strings.TrimSpace(ref)
			if ref != "" {
				referenced[ref] = struct{}{}
			}
		}
		for _, ref := range condition.Any {
			ref = strings.TrimSpace(ref)
			if ref != "" {
				referenced[ref] = struct{}{}
			}
		}
		for _, ref := range condition.None {
			ref = strings.TrimSpace(ref)
			if ref != "" {
				referenced[ref] = struct{}{}
			}
		}
	}

	for _, target := range plan.Targets {
		addRefs(target.When)
		for _, baseRule := range target.BaseRules {
			addRefs(baseRule.When)
		}
	}
	for _, extraRule := range plan.ExtraRules {
		addRefs(extraRule.When)
	}

	for ref := range referenced {
		if _, exists := known[ref]; !exists {
			return fmt.Errorf("condition %q is referenced but not defined", ref)
		}
	}

	unusedFacts := make([]string, 0)
	for _, fact := range facts {
		if _, used := referenced[fact.Name]; !used {
			unusedFacts = append(unusedFacts, fact.Name)
		}
	}

	if len(unusedFacts) == 0 {
		return nil
	}

	sort.Strings(unusedFacts)
	return fmt.Errorf("unused facts: %s", strings.Join(unusedFacts, ", "))
}

type DynamicRuleInjector struct {
	source    string
	signature uint64
}

func NewDynamicRuleInjector(source string) *DynamicRuleInjector {
	return &DynamicRuleInjector{source: strings.TrimSpace(source)}
}

func (i *DynamicRuleInjector) Apply(cfg *config.CharacterCfg, logger *slog.Logger, ruleLines []string) bool {
	if cfg == nil {
		return false
	}

	source := i.source
	if source == "" {
		source = "dynamic:leveling_runewords"
	}

	baseRules := make(nip.Rules, 0, len(cfg.Runtime.Rules))
	foundDynamicRules := false
	signature := fnv.New64a()

	for _, rule := range cfg.Runtime.Rules {
		if rule.Filename == source {
			// Replace previously injected dynamic rules as a full set.
			foundDynamicRules = true
			continue
		}

		baseRules = append(baseRules, rule)
		hashPart(signature, rule.Filename)
		hashPart(signature, rule.RawLine)
	}

	filteredRuleLines := make([]string, 0, len(ruleLines))
	seenRuleLines := make(map[string]struct{}, len(ruleLines))
	for _, line := range ruleLines {
		line = strings.TrimSpace(line)
		if line == "" {
			continue
		}
		if _, exists := seenRuleLines[line]; exists {
			continue
		}
		seenRuleLines[line] = struct{}{}
		filteredRuleLines = append(filteredRuleLines, line)
		hashPart(signature, line)
	}

	combinedSignature := signature.Sum64()
	// Skip runtime rebuild when both static rule-set and dynamic lines are unchanged.
	if foundDynamicRules && combinedSignature == i.signature {
		return false
	}

	updatedRules := make(nip.Rules, 0, len(baseRules)+len(filteredRuleLines))
	updatedRules = append(updatedRules, baseRules...)

	for idx, line := range filteredRuleLines {
		rule, err := nip.NewRule(line, source, idx+1)
		if err != nil {
			if logger != nil {
				logger.Error("Failed to build leveling runeword pickit rule", "rule", line, "error", err)
			}
			return false
		}
		updatedRules = append(updatedRules, rule)
	}

	updatedTierRules := make([]int, 0, len(updatedRules))
	for idx, rule := range updatedRules {
		if rule.Tier() > 0 || rule.MercTier() > 0 {
			updatedTierRules = append(updatedTierRules, idx)
		}
	}

	cfg.Runtime.Rules = updatedRules
	cfg.Runtime.TierRules = updatedTierRules
	i.signature = combinedSignature

	if logger != nil {
		logger.Debug("Dynamic leveling runeword rules updated", "source", source, "rules", len(filteredRuleLines))
	}

	return true
}

func hashPart(h hash.Hash64, text string) {
	_, _ = h.Write([]byte(text))
	_, _ = h.Write([]byte{0})
}

func normalizeSelection(selection RuleSelection) RuleSelection {
	if selection == "" {
		return SelectAllValid
	}
	return selection
}

type FactType string

const (
	FactTypeSkillAvailable FactType = "skill_available"
	FactTypeInventoryMatch FactType = "inventory_match"
)

type StateFact struct {
	Name  string         `json:"name"`
	Type  FactType       `json:"type"`
	Skill string         `json:"skill,omitempty"`
	Query InventoryQuery `json:"query,omitempty"`
}

type InventoryQuery struct {
	Locations        []item.LocationType `json:"locations,omitempty"`
	BodyLocations    []item.LocationType `json:"bodyLocations,omitempty"`
	Runeword         item.RunewordName   `json:"runeword,omitempty"`
	Types            []string            `json:"types,omitempty"`
	ItemNames        []string            `json:"itemNames,omitempty"`
	UniqueNames      []string            `json:"uniqueNames,omitempty"`
	Quality          string              `json:"quality,omitempty"`
	QualityAtMost    string              `json:"qualityAtMost,omitempty"`
	QualityAtLeast   string              `json:"qualityAtLeast,omitempty"`
	Tier             string              `json:"tier,omitempty"`
	TierAtMost       string              `json:"tierAtMost,omitempty"`
	TierAtLeast      string              `json:"tierAtLeast,omitempty"`
	TierScoreSource  string              `json:"tierScoreSource,omitempty"`
	TierScore        *float64            `json:"tierScore,omitempty"`
	TierScoreAtMost  *float64            `json:"tierScoreAtMost,omitempty"`
	TierScoreAtLeast *float64            `json:"tierScoreAtLeast,omitempty"`
	Sockets          *int                `json:"sockets,omitempty"`
	Ethereal         *bool               `json:"ethereal,omitempty"`
	IsRuneword       *bool               `json:"isRuneword,omitempty"`
	HasSocketedItems *bool               `json:"hasSocketedItems,omitempty"`
	MinCount         int                 `json:"minCount,omitempty"`
}

type FactContext struct {
	CanUseSkill func(skill.ID) bool
	Inventory   []data.Item
	// RuntimeRules/TierRuleIndexes are required for tier-score based queries.
	RuntimeRules    nip.Rules
	TierRuleIndexes []int
}

func ValidateFacts(facts []StateFact) error {
	seen := make(map[string]struct{}, len(facts))

	for i, fact := range facts {
		name := strings.TrimSpace(fact.Name)
		if name == "" {
			return fmt.Errorf("fact at index %d has empty name", i)
		}

		if _, exists := seen[name]; exists {
			return fmt.Errorf("duplicate fact name: %s", name)
		}
		seen[name] = struct{}{}

		switch fact.Type {
		case FactTypeSkillAvailable:
			if _, ok := resolveSkillID(fact.Skill); !ok {
				return fmt.Errorf("fact %q: unknown skill %q", name, fact.Skill)
			}
		case FactTypeInventoryMatch:
			if _, err := prepareInventoryQuery(fact.Query); err != nil {
				return fmt.Errorf("fact %q: %w", name, err)
			}
		default:
			return fmt.Errorf("fact %q has unsupported type %q", name, fact.Type)
		}
	}

	return nil
}

func EvaluateFacts(ctx FactContext, facts []StateFact) (State, error) {
	state := make(State, len(facts))

	for i, fact := range facts {
		name := strings.TrimSpace(fact.Name)
		if name == "" {
			return nil, fmt.Errorf("fact at index %d has empty name", i)
		}

		switch fact.Type {
		case FactTypeSkillAvailable:
			id, ok := resolveSkillID(fact.Skill)
			if !ok {
				return nil, fmt.Errorf("fact %q: unknown skill %q", name, fact.Skill)
			}

			if ctx.CanUseSkill == nil {
				state[name] = false
				continue
			}
			state[name] = ctx.CanUseSkill(id)
		case FactTypeInventoryMatch:
			prepared, err := prepareInventoryQuery(fact.Query)
			if err != nil {
				return nil, fmt.Errorf("fact %q: %w", name, err)
			}
			state[name] = prepared.matchesAny(ctx.Inventory, ctx.RuntimeRules, ctx.TierRuleIndexes)
		default:
			return nil, fmt.Errorf("fact %q has unsupported type %q", name, fact.Type)
		}
	}

	return state, nil
}

type preparedInventoryQuery struct {
	locationSet      map[item.LocationType]struct{}
	bodyLocationSet  map[item.LocationType]struct{}
	runeword         item.RunewordName
	typeSet          map[string]struct{}
	itemNameSet      map[string]struct{}
	uniqueIDSet      map[int]struct{}
	qualityExact     *item.Quality
	qualityMin       *item.Quality
	qualityMax       *item.Quality
	tierExact        *item.Tier
	tierMin          *item.Tier
	tierMax          *item.Tier
	tierScoreSource  TierScoreSource
	tierScoreExact   *float64
	tierScoreMin     *float64
	tierScoreMax     *float64
	sockets          *int
	ethereal         *bool
	isRuneword       *bool
	hasSocketedItems *bool
	minCount         int
}

func prepareInventoryQuery(query InventoryQuery) (preparedInventoryQuery, error) {
	pq := preparedInventoryQuery{
		runeword:         item.RunewordName(strings.TrimSpace(string(query.Runeword))),
		sockets:          query.Sockets,
		ethereal:         query.Ethereal,
		isRuneword:       query.IsRuneword,
		hasSocketedItems: query.HasSocketedItems,
		minCount:         query.MinCount,
	}

	if pq.minCount <= 0 {
		pq.minCount = 1
	}

	if pq.sockets != nil && *pq.sockets < 0 {
		return preparedInventoryQuery{}, fmt.Errorf("sockets must be >= 0")
	}

	if pq.runeword != "" {
		if _, ok := validRunewordNames[pq.runeword]; !ok {
			return preparedInventoryQuery{}, fmt.Errorf("unknown runeword %q", query.Runeword)
		}
	}

	locationSet, err := buildLocationSet(query.Locations)
	if err != nil {
		return preparedInventoryQuery{}, err
	}
	pq.locationSet = locationSet

	bodyLocationSet, err := buildBodyLocationSet(query.BodyLocations)
	if err != nil {
		return preparedInventoryQuery{}, err
	}
	pq.bodyLocationSet = bodyLocationSet

	if len(query.Types) > 0 {
		pq.typeSet = make(map[string]struct{}, len(query.Types))
		for _, typ := range query.Types {
			normalizedType := strings.ToLower(strings.TrimSpace(typ))
			if normalizedType == "" {
				return preparedInventoryQuery{}, fmt.Errorf("types contains empty value")
			}
			pq.typeSet[normalizedType] = struct{}{}
		}
	}

	if len(query.ItemNames) > 0 {
		pq.itemNameSet = make(map[string]struct{}, len(query.ItemNames))
		for _, name := range query.ItemNames {
			normalizedName := normalizeToken(name)
			if normalizedName == "" {
				return preparedInventoryQuery{}, fmt.Errorf("itemNames contains empty value")
			}
			pq.itemNameSet[normalizedName] = struct{}{}
		}
	}

	if len(query.UniqueNames) > 0 {
		pq.uniqueIDSet = make(map[int]struct{}, len(query.UniqueNames))
		for _, uniqueName := range query.UniqueNames {
			uniqueID, ok := uniqueNameToID[normalizeToken(uniqueName)]
			if !ok {
				return preparedInventoryQuery{}, fmt.Errorf("unknown unique name %q", uniqueName)
			}
			pq.uniqueIDSet[uniqueID] = struct{}{}
		}
	}

	if strings.TrimSpace(query.Quality) != "" && (strings.TrimSpace(query.QualityAtLeast) != "" || strings.TrimSpace(query.QualityAtMost) != "") {
		return preparedInventoryQuery{}, fmt.Errorf("quality cannot be combined with qualityAtLeast/qualityAtMost")
	}
	if strings.TrimSpace(query.Quality) != "" {
		quality, err := parseQuality(query.Quality)
		if err != nil {
			return preparedInventoryQuery{}, err
		}
		pq.qualityExact = &quality
	} else {
		if strings.TrimSpace(query.QualityAtLeast) != "" {
			qualityMin, err := parseQuality(query.QualityAtLeast)
			if err != nil {
				return preparedInventoryQuery{}, fmt.Errorf("qualityAtLeast: %w", err)
			}
			pq.qualityMin = &qualityMin
		}
		if strings.TrimSpace(query.QualityAtMost) != "" {
			qualityMax, err := parseQuality(query.QualityAtMost)
			if err != nil {
				return preparedInventoryQuery{}, fmt.Errorf("qualityAtMost: %w", err)
			}
			pq.qualityMax = &qualityMax
		}
	}

	if strings.TrimSpace(query.Tier) != "" && (strings.TrimSpace(query.TierAtLeast) != "" || strings.TrimSpace(query.TierAtMost) != "") {
		return preparedInventoryQuery{}, fmt.Errorf("tier cannot be combined with tierAtLeast/tierAtMost")
	}
	if strings.TrimSpace(query.Tier) != "" {
		tier, err := parseTier(query.Tier)
		if err != nil {
			return preparedInventoryQuery{}, err
		}
		pq.tierExact = &tier
	} else {
		if strings.TrimSpace(query.TierAtLeast) != "" {
			tierMin, err := parseTier(query.TierAtLeast)
			if err != nil {
				return preparedInventoryQuery{}, fmt.Errorf("tierAtLeast: %w", err)
			}
			pq.tierMin = &tierMin
		}
		if strings.TrimSpace(query.TierAtMost) != "" {
			tierMax, err := parseTier(query.TierAtMost)
			if err != nil {
				return preparedInventoryQuery{}, fmt.Errorf("tierAtMost: %w", err)
			}
			pq.tierMax = &tierMax
		}
	}

	if query.TierScore != nil && (query.TierScoreAtLeast != nil || query.TierScoreAtMost != nil) {
		return preparedInventoryQuery{}, fmt.Errorf("tierScore cannot be combined with tierScoreAtLeast/tierScoreAtMost")
	}

	tierScoreSource, err := parseTierScoreSource(query.TierScoreSource)
	if err != nil {
		return preparedInventoryQuery{}, err
	}
	pq.tierScoreSource = tierScoreSource
	pq.tierScoreExact = query.TierScore
	pq.tierScoreMin = query.TierScoreAtLeast
	pq.tierScoreMax = query.TierScoreAtMost

	return pq, nil
}

func (pq preparedInventoryQuery) matchesAny(items []data.Item, runtimeRules nip.Rules, tierRuleIndexes []int) bool {
	matchCount := 0
	for _, itm := range items {
		if !pq.matchesItem(itm, runtimeRules, tierRuleIndexes) {
			continue
		}

		matchCount++
		if matchCount >= pq.minCount {
			return true
		}
	}

	return false
}

func (pq preparedInventoryQuery) matchesItem(itm data.Item, runtimeRules nip.Rules, tierRuleIndexes []int) bool {
	if len(pq.locationSet) > 0 {
		if _, ok := pq.locationSet[itm.Location.LocationType]; !ok {
			return false
		}
	}

	if len(pq.bodyLocationSet) > 0 {
		if _, ok := pq.bodyLocationSet[itm.Location.BodyLocation]; !ok {
			return false
		}
	}

	if pq.runeword != "" && itm.RunewordName != pq.runeword {
		return false
	}

	if len(pq.typeSet) > 0 {
		typ := strings.ToLower(strings.TrimSpace(itm.Type().Code))
		if _, ok := pq.typeSet[typ]; !ok {
			return false
		}
	}

	if len(pq.itemNameSet) > 0 {
		normalizedInternalName := normalizeToken(string(itm.Name))
		normalizedDisplayName := normalizeToken(string(itm.Desc().Name))
		if _, ok := pq.itemNameSet[normalizedInternalName]; !ok {
			if _, ok := pq.itemNameSet[normalizedDisplayName]; !ok {
				return false
			}
		}
	}

	if len(pq.uniqueIDSet) > 0 {
		if itm.Quality != item.QualityUnique {
			return false
		}

		if _, ok := pq.uniqueIDSet[int(itm.UniqueSetID)]; !ok {
			return false
		}
	}

	if pq.qualityExact != nil {
		if itm.Quality != *pq.qualityExact {
			return false
		}
	}
	if pq.qualityMin != nil && itm.Quality < *pq.qualityMin {
		return false
	}
	if pq.qualityMax != nil && itm.Quality > *pq.qualityMax {
		return false
	}

	currentTier := itm.Desc().Tier()
	if pq.tierExact != nil {
		if currentTier != *pq.tierExact {
			return false
		}
	}
	if pq.tierMin != nil && currentTier < *pq.tierMin {
		return false
	}
	if pq.tierMax != nil && currentTier > *pq.tierMax {
		return false
	}

	if pq.tierScoreExact != nil || pq.tierScoreMin != nil || pq.tierScoreMax != nil {
		tierScore := resolveTierScore(itm, pq.tierScoreSource, runtimeRules, tierRuleIndexes)
		if pq.tierScoreExact != nil && tierScore != *pq.tierScoreExact {
			return false
		}
		if pq.tierScoreMin != nil && tierScore < *pq.tierScoreMin {
			return false
		}
		if pq.tierScoreMax != nil && tierScore > *pq.tierScoreMax {
			return false
		}
	}

	if pq.sockets != nil {
		currentSockets := 0
		if socketsStat, found := itm.FindStat(stat.NumSockets, 0); found {
			currentSockets = socketsStat.Value
		}

		if currentSockets != *pq.sockets {
			return false
		}
	}

	if pq.ethereal != nil && itm.Ethereal != *pq.ethereal {
		return false
	}
	if pq.isRuneword != nil && itm.IsRuneword != *pq.isRuneword {
		return false
	}
	if pq.hasSocketedItems != nil && itm.HasSocketedItems() != *pq.hasSocketedItems {
		return false
	}

	return true
}

type TierScoreSource string

const (
	TierScoreSourcePlayer TierScoreSource = "player"
	TierScoreSourceMerc   TierScoreSource = "merc"
)

func parseTierScoreSource(value string) (TierScoreSource, error) {
	switch normalizeToken(value) {
	case "", "player":
		return TierScoreSourcePlayer, nil
	case "merc", "mercenary":
		return TierScoreSourceMerc, nil
	default:
		return "", fmt.Errorf("unknown tierScoreSource %q", value)
	}
}

func resolveTierScore(itm data.Item, source TierScoreSource, runtimeRules nip.Rules, tierRuleIndexes []int) float64 {
	if len(runtimeRules) == 0 || len(tierRuleIndexes) == 0 {
		return 0
	}

	// Tier values are sourced from evaluated NIP tier rules (not item base tier).
	playerRule, mercRule := runtimeRules.EvaluateTiers(itm, tierRuleIndexes)
	if source == TierScoreSourceMerc {
		return mercRule.MercTier()
	}

	return playerRule.Tier()
}

var qualityNameMap = map[string]item.Quality{
	"lowquality": item.QualityLowQuality,
	"normal":     item.QualityNormal,
	"superior":   item.QualitySuperior,
	"magic":      item.QualityMagic,
	"set":        item.QualitySet,
	"rare":       item.QualityRare,
	"unique":     item.QualityUnique,
	"crafted":    item.QualityCrafted,
}

func parseQuality(value string) (item.Quality, error) {
	normalized := normalizeToken(value)
	if quality, ok := qualityNameMap[normalized]; ok {
		return quality, nil
	}

	return item.QualityLowQuality, fmt.Errorf("unknown quality %q", value)
}

var tierNameMap = map[string]item.Tier{
	"normal":      item.TierNormal,
	"exceptional": item.TierExceptional,
	"elite":       item.TierElite,
}

func parseTier(value string) (item.Tier, error) {
	normalized := normalizeToken(value)
	if tier, ok := tierNameMap[normalized]; ok {
		return tier, nil
	}

	return item.TierNormal, fmt.Errorf("unknown tier %q", value)
}

func normalizeToken(value string) string {
	value = strings.ToLower(strings.TrimSpace(value))
	if value == "" {
		return ""
	}

	var b strings.Builder
	b.Grow(len(value))
	for _, r := range value {
		if r >= 'a' && r <= 'z' {
			b.WriteRune(r)
			continue
		}
		if r >= '0' && r <= '9' {
			b.WriteRune(r)
			continue
		}
	}

	return b.String()
}

var validLocationTypes = map[item.LocationType]struct{}{
	item.LocationUnknown:      {},
	item.LocationInventory:    {},
	item.LocationStash:        {},
	item.LocationSharedStash:  {},
	item.LocationBelt:         {},
	item.LocationCube:         {},
	item.LocationVendor:       {},
	item.LocationGround:       {},
	item.LocationSocket:       {},
	item.LocationCursor:       {},
	item.LocationEquipped:     {},
	item.LocationMercenary:    {},
	item.LocationGemsTab:      {},
	item.LocationMaterialsTab: {},
	item.LocationRunesTab:     {},
}

func buildLocationSet(locations []item.LocationType) (map[item.LocationType]struct{}, error) {
	if len(locations) == 0 {
		return nil, nil
	}

	set := make(map[item.LocationType]struct{}, len(locations))
	for _, location := range locations {
		if _, ok := validLocationTypes[location]; !ok {
			return nil, fmt.Errorf("unsupported location %q", location)
		}
		set[location] = struct{}{}
	}

	return set, nil
}

var validBodyLocationTypes = map[item.LocationType]struct{}{
	item.LocNone:              {},
	item.LocHead:              {},
	item.LocNeck:              {},
	item.LocTorso:             {},
	item.LocLeftArm:           {},
	item.LocRightArm:          {},
	item.LocLeftRing:          {},
	item.LocRightRing:         {},
	item.LocBelt:              {},
	item.LocFeet:              {},
	item.LocGloves:            {},
	item.LocLeftArmSecondary:  {},
	item.LocRightArmSecondary: {},
}

func buildBodyLocationSet(bodyLocations []item.LocationType) (map[item.LocationType]struct{}, error) {
	if len(bodyLocations) == 0 {
		return nil, nil
	}

	set := make(map[item.LocationType]struct{}, len(bodyLocations))
	for _, location := range bodyLocations {
		if _, ok := validBodyLocationTypes[location]; !ok {
			return nil, fmt.Errorf("unsupported body location %q", location)
		}
		set[location] = struct{}{}
	}

	return set, nil
}

var validRunewordNames = buildRunewordSet()

func buildRunewordSet() map[item.RunewordName]struct{} {
	set := map[item.RunewordName]struct{}{
		item.RunewordNone: {},
	}

	for _, rw := range item.RunewordIDMap {
		set[rw] = struct{}{}
	}

	return set
}

var skillIDLookup = buildSkillLookup()

func buildSkillLookup() map[string]skill.ID {
	lookup := make(map[string]skill.ID, len(skill.SkillNames)*2)

	for id, rawSkillName := range skill.SkillNames {
		lookup[normalizeToken(rawSkillName)] = id
	}

	for id, details := range skill.Skills {
		lookup[normalizeToken(details.Name)] = id
		lookup[normalizeToken(string(details.SkillDesc))] = id
	}

	return lookup
}

func resolveSkillID(skillRef string) (skill.ID, bool) {
	skillRef = strings.TrimSpace(skillRef)
	if skillRef == "" {
		return skill.Unset, false
	}

	if value, err := strconv.Atoi(skillRef); err == nil {
		id := skill.ID(value)
		if _, ok := skill.Skills[id]; ok {
			return id, true
		}
	}

	id, ok := skillIDLookup[normalizeToken(skillRef)]
	return id, ok
}

var uniqueNameToID = buildUniqueNameToIDMap()

func buildUniqueNameToIDMap() map[string]int {
	lookup := make(map[string]int, len(item.UniqueItems)*2)
	for uniqueName, details := range item.UniqueItems {
		lookup[normalizeToken(string(uniqueName))] = details.ID
		lookup[normalizeToken(details.Name)] = details.ID
	}

	return lookup
}
