package core

import (
	"log/slog"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/mode"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

//region Constants

const (
	LegacyScreenRange     = 13
	ModernScreenRange     = 18
	ScreenRange           = ModernScreenRange
	MeleeRange            = 3
	DefaultMainSkillRange = 12
	CastCompleteTimeout   = 3 * time.Second
	CastCompleteMinGap    = 150 * time.Millisecond
	CastCompletePoll      = 16 * time.Millisecond
	TargetTimeout         = 10 * time.Second
)

//endregion Constants

//region Types

// CharacterBase holds shared combat behavior and helpers used by all builds.
// Concrete builds embed it and override only what they need (e.g. KillMonsterSequence).
type CharacterBase struct {
	*context.Context
}

//endregion Types

func (bc CharacterBase) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

func (bc CharacterBase) MainSkillRange() int {
	return DefaultMainSkillRange
}

func ResolveScreenRange(legacyGraphics bool) int {
	if legacyGraphics {
		return LegacyScreenRange
	}

	return ModernScreenRange
}

func (bc CharacterBase) ScreenRange() int {
	if bc.Data == nil {
		return ScreenRange
	}

	return ResolveScreenRange(bc.Data.LegacyGraphics)
}

//region Boss Helpers

func (bc CharacterBase) KillCountess() error {
	return bc.KillMonsterByName(npc.DarkStalker, data.MonsterTypeSuperUnique, nil)
}

func (bc CharacterBase) KillAndariel() error {
	return bc.KillMonsterByName(npc.Andariel, data.MonsterTypeUnique, nil)
}

func (bc CharacterBase) KillSummoner() error {
	return bc.KillMonsterByName(npc.Summoner, data.MonsterTypeUnique, nil)
}

func (bc CharacterBase) KillDuriel() error {
	return bc.KillMonsterByName(npc.Duriel, data.MonsterTypeUnique, nil)
}

func (bc CharacterBase) KillCouncil() error {
	return bc.Context.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		// Prioritize the closest council member to keep the fight localized.
		var councilMembers []data.Monster
		for _, m := range d.Monsters {
			if (m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3) && m.Stats[stat.Life] > 0 {
				councilMembers = append(councilMembers, m)
			}
		}

		sort.Slice(councilMembers, func(i, j int) bool {
			return bc.PathFinder.DistanceFromMe(councilMembers[i].Position) < bc.PathFinder.DistanceFromMe(councilMembers[j].Position)
		})

		for _, m := range councilMembers {
			return m.UnitID, true
		}

		return 0, false
	}, nil)
}

func (bc CharacterBase) KillMephisto() error {
	return bc.KillMonsterByName(npc.Mephisto, data.MonsterTypeUnique, nil)
}

func (bc CharacterBase) KillIzual() error {
	return bc.KillMonsterByName(npc.Izual, data.MonsterTypeUnique, nil)
}

func (bc CharacterBase) KillDiablo() error {
	timeout := 20 * time.Second
	startTime := time.Now()
	diabloFound := false

	for {
		if time.Since(startTime) > timeout && !diabloFound {
			bc.Logger.Error("Diablo was not found, timeout reached")
			return nil
		}

		diablo, found := bc.Data.Monsters.FindOne(npc.Diablo, data.MonsterTypeUnique)
		if !found || diablo.Stats[stat.Life] <= 0 {
			if diabloFound {
				return nil
			}
			time.Sleep(200 * time.Millisecond)
			continue
		}

		diabloFound = true
		bc.Logger.Info("Diablo detected, attacking")

		return bc.KillMonsterByName(npc.Diablo, data.MonsterTypeUnique, nil)
	}
}

func (bc CharacterBase) KillPindle() error {
	return bc.KillMonsterByName(npc.DefiledWarrior, data.MonsterTypeSuperUnique, bc.CharacterCfg.Game.Pindleskin.SkipOnImmunities)
}

func (bc CharacterBase) KillNihlathak() error {
	return bc.KillMonsterByName(npc.Nihlathak, data.MonsterTypeSuperUnique, nil)
}

func (bc CharacterBase) KillBaal() error {
	return bc.KillMonsterByName(npc.BaalCrab, data.MonsterTypeUnique, nil)
}

func (bc CharacterBase) KillLilith() error {
	return bc.KillMonsterByName(npc.Lilith, data.MonsterTypeUnique, nil)
}

func (bc CharacterBase) KillUberDuriel() error {
	return bc.KillMonsterByName(npc.UberDuriel, data.MonsterTypeUnique, nil)
}

func (bc CharacterBase) KillUberIzual() error {
	return bc.KillMonsterByName(npc.UberIzual, data.MonsterTypeUnique, nil)
}

func (bc CharacterBase) KillUberMephisto() error {
	return bc.KillMonsterByName(npc.UberMephisto, data.MonsterTypeUnique, nil)
}

func (bc CharacterBase) KillUberDiablo() error {
	return bc.KillMonsterByName(npc.UberDiablo, data.MonsterTypeUnique, nil)
}

func (bc CharacterBase) KillUberBaal() error {
	return bc.KillMonsterByName(npc.UberBaal, data.MonsterTypeUnique, nil)
}

// KillMonsterByName exposes the generic boss kill loop to other packages.
func (bc CharacterBase) KillMonsterByName(npcID npc.ID, monsterType data.MonsterType, skipOnImmunities []stat.Resist) error {
	for {
		m, found := bc.Data.Monsters.FindOne(npcID, monsterType)
		if !found || m.Stats[stat.Life] <= 0 {
			return nil
		}

		if err := bc.Context.Char.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			if m, found := d.Monsters.FindOne(npcID, monsterType); found {
				return m.UnitID, true
			}
			return 0, false
		}, skipOnImmunities); err != nil {
			return err
		}
	}
}

//endregion Boss Helpers

//region Combat Filters

func (bc CharacterBase) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

// PreBattleChecks applies safety gates before we start a combat loop.
func (bc CharacterBase) PreBattleChecks(id data.UnitID, skipOnImmunities []stat.Resist) bool {
	monster, found := bc.Data.Monsters.FindByID(id)
	if !found {
		return false
	}

	// Skip dead targets early.
	if monster.Stats[stat.Life] <= 0 {
		return false
	}

	// Special case: Vizier can spawn on weird/off-grid tiles in Chaos Sanctuary.
	isVizier := monster.Type == data.MonsterTypeSuperUnique && monster.Name == npc.StormCaster

	// Filter "underwater/off-grid" targets that exist in data but are not actually attackable/reachable.
	// Apply only for nearby targets to avoid changing long-range targeting behavior.
	const sanityRangeYards = 60
	if !isVizier && bc.PathFinder.DistanceFromMe(monster.Position) <= sanityRangeYards {
		if !bc.Data.AreaData.IsWalkable(monster.Position) {
			return false
		}

		// If we cannot teleport, ensure the target is reachable by pathing.
		if !bc.Data.CanTeleport() {
			_, _, pathFound := bc.PathFinder.GetPath(monster.Position)
			if !pathFound {
				return false
			}
		}
	}

	// TODO: Is it really used? Seems everyone calls it with nil value
	for _, i := range skipOnImmunities {
		if monster.IsImmune(i) {
			bc.Logger.Info("Monster is immune! skipping", slog.String("immuneTo", string(i)))
			return false
		}
	}

	return true
}

// RetargetIfBlockedBeyondRange selects a closer visible enemy when the target is out of range and blocked by monsters.
func (bc CharacterBase) RetargetIfBlockedBeyondRange(target data.Monster, mainSkillRange int) (data.Monster, bool) {
	// Do not retarget if we can teleport or on invalid mainSkillRange
	if bc.Data.CanTeleport() || mainSkillRange <= 0 {
		return data.Monster{}, false
	}

	// Do not retarget if the target is in range
	targetDistance := bc.PathFinder.DistanceFromMe(target.Position)
	if targetDistance < mainSkillRange {
		return data.Monster{}, false
	}

	const pathBlockerPadding = 3
	enemies := bc.Data.Monsters.Enemies()
	playerPos := bc.Data.PlayerUnit.Position
	targetPos := target.Position

	// Check if we are blocked by an enemy
	blocked := false
	lineDX := targetPos.X - playerPos.X
	lineDY := targetPos.Y - playerPos.Y
	lineLenSq := int64(lineDX*lineDX + lineDY*lineDY)
	// Check if there is any enemy in the direct line to target
	if lineLenSq > 0 {
		for _, candidate := range enemies {
			if candidate.UnitID == target.UnitID {
				continue
			}
			if candidate.Stats[stat.Life] <= 0 {
				continue
			}
			candidateDistance := bc.PathFinder.DistanceFromMe(candidate.Position)
			if candidateDistance >= targetDistance {
				continue
			}

			offsetX := candidate.Position.X - playerPos.X
			offsetY := candidate.Position.Y - playerPos.Y
			dot := int64(offsetX*lineDX + offsetY*lineDY)
			if dot <= 0 || dot >= lineLenSq {
				continue
			}

			cross := int64(offsetX*lineDY - offsetY*lineDX)
			padding := int64(pathBlockerPadding)
			if cross*cross <= padding*padding*lineLenSq {
				blocked = true
				break
			}
		}
	}
	// Check if there is any enemy in the path to target
	if !blocked {
		path, _, found := bc.PathFinder.GetPath(targetPos)
		if found && len(path) > 0 {
			for _, candidate := range enemies {
				if candidate.UnitID == target.UnitID {
					continue
				}
				if candidate.Stats[stat.Life] <= 0 {
					continue
				}
				if path.Intersects(*bc.Data, candidate.Position, pathBlockerPadding) {
					blocked = true
					break
				}
			}
		}
	}
	if !blocked {
		return data.Monster{}, false
	}

	// Pick closest enemy in LoS
	closestFound := false
	closestDistance := 0
	closest := data.Monster{}
	for _, candidate := range enemies {
		if candidate.Stats[stat.Life] <= 0 {
			continue
		}
		if !bc.PathFinder.LineOfSight(playerPos, candidate.Position) {
			continue
		}
		candidateDistance := bc.PathFinder.DistanceFromMe(candidate.Position)
		if !closestFound || candidateDistance < closestDistance {
			closestFound = true
			closestDistance = candidateDistance
			closest = candidate
		}
	}
	if !closestFound {
		return data.Monster{}, false
	}

	return closest, true
}

func (bc CharacterBase) HasRaiserNearby(maxRange int) bool {
	for _, m := range bc.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}
		if !m.IsMonsterRaiser() {
			continue
		}
		if bc.PathFinder.DistanceFromMe(m.Position) <= maxRange {
			return true
		}
	}
	return false
}

var unusableCorpseStates = []state.State{
	state.CorpseNoselect,
	state.CorpseNodraw,
	state.Revive,
	state.Redeemed,
	state.Shatter,
	state.Freeze,
	state.Restinpeace,
}

func (bc CharacterBase) HasRedeemableCorpseNearby(maxRange int) bool {
	for _, c := range bc.Data.Corpses {
		if c.IsMerc() {
			continue
		}
		if bc.PathFinder.DistanceFromMe(c.Position) > maxRange {
			continue
		}
		skipCorpse := false
		for _, unusableState := range unusableCorpseStates {
			if c.States.HasState(unusableState) {
				skipCorpse = true
				break
			}
		}
		if skipCorpse {
			continue
		}
		return true
	}
	return false
}

func (bc CharacterBase) HasUberMephistoNearby(maxRange int) bool {
	for _, m := range bc.Data.Monsters.Enemies() {
		if m.Stats[stat.Life] <= 0 {
			continue
		}
		if m.Name != npc.UberMephisto {
			continue
		}
		if bc.PathFinder.DistanceFromMe(m.Position) <= maxRange {
			return true
		}
	}
	return false
}

//endregion Combat Filters

//region Casting Helpers

func (bc CharacterBase) WaitForCastComplete() bool {
	startTime := time.Now()
	lastCastAt := bc.Context.LastCastAt

	for time.Since(startTime) < CastCompleteTimeout {
		bc.Context.RefreshGameData()

		// Use LastCastAt (updated by cast/attack actions) to avoid re-triggering mid-animation.
		if bc.Data.PlayerUnit.Mode != mode.CastingSkill && time.Since(lastCastAt) > CastCompleteMinGap {
			return true
		}

		time.Sleep(CastCompletePoll)
	}

	return false
}

//endregion Casting Helpers

//region Skill Helpers

func (bc CharacterBase) BaseSkillLevel(id skill.ID) int {
	if s, found := bc.Data.PlayerUnit.Skills[id]; found {
		return int(s.Level)
	}

	return 0
}

func (bc CharacterBase) NonClassSkillLevel(id skill.ID) int {
	for _, item := range bc.Data.Inventory.ByLocation(item.LocationEquipped) {
		if s, found := item.FindStat(stat.NonClassSkill, int(id)); found {
			return int(s.Value)
		}
	}

	return 0
}

func (bc CharacterBase) CanUseSkill(id skill.ID) bool {
	return bc.BaseSkillLevel(id) > 0
}

func (bc CharacterBase) CanUseNonClassSkill(id skill.ID) bool {
	return bc.NonClassSkillLevel(id) > 0
}

func (bc CharacterBase) AppendUniqueSkill(skills []skill.ID, id skill.ID) []skill.ID {
	if id <= 0 {
		return skills
	}
	for _, existing := range skills {
		if existing == id {
			return skills
		}
	}

	return append(skills, id)
}

func (bc CharacterBase) MissingKeyBindings(required []skill.ID) []skill.ID {
	missing := make([]skill.ID, 0)
	for _, cskill := range required {
		if cskill <= 0 {
			continue
		}
		if _, found := bc.Data.KeyBindings.KeyBindingForSkill(cskill); !found {
			missing = append(missing, cskill)
		}
	}

	if len(missing) > 0 {
		bc.Logger.Debug("There are missing required key bindings.", slog.Any("Bindings", missing))
	}

	return missing
}

func (bc CharacterBase) SetUseTeleport(enabled bool) {
	bc.CharacterCfg.Character.UseTeleport = enabled
	bc.Data.CharacterCfg.Character.UseTeleport = enabled
}

//endregion Skill Helpers
