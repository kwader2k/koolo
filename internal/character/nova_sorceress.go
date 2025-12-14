package character

import (
	"log/slog"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

const (
	NovaMinDistance = 6
	NovaMaxDistance = 9

	// Aggressive positioning – closer to the pack
	NovaAggroMinDistance = 3
	NovaAggroMaxDistance = 7

	StaticMinDistance    = 13
	StaticMaxDistance    = 22
	NovaMaxAttacksLoop   = 10
	StaticFieldThreshold = 67 // Cast Static Field if monster HP is above this percentage
)

type NovaSorceress struct {
	BaseCharacter
}

// gridDistance returns Chebyshev distance on the tile grid (max of |dx|,|dy|).
func gridDistance(a, b data.Position) int {
	dx := a.X - b.X
	if dx < 0 {
		dx = -dx
	}
	dy := a.Y - b.Y
	if dy < 0 {
		dy = -dy
	}
	if dx > dy {
		return dx
	}
	return dy
}

// findBestAggressiveNovaPosition tries to find a position around the current pack
// from which Nova will hit as many monsters as possible, while not drifting too
// far from the player or the main target.
func (s NovaSorceress) findBestAggressiveNovaPosition(target data.Monster) (data.Position, bool) {
	ctx := context.Get()
	playerPos := ctx.Data.PlayerUnit.Position

	enemies := ctx.Data.Monsters.Enemies()
	if len(enemies) == 0 {
		return data.Position{}, false
	}

	// Treat as pack all monsters near the main target.
	const packRadius = 15

	pack := make([]data.Monster, 0, len(enemies))
	for _, m := range enemies {
		if m.Stats[stat.Life] <= 0 {
			continue
		}
		if gridDistance(target.Position, m.Position) <= packRadius {
			pack = append(pack, m)
		}
	}
	if len(pack) == 0 {
		pack = enemies
	}

	// Candidate positions: tiles around each monster in the pack.
	candidates := make([]data.Position, 0, len(pack)*9)
	for _, m := range pack {
		for dx := -1; dx <= 1; dx++ {
			for dy := -1; dy <= 1; dy++ {
				pos := data.Position{X: m.Position.X + dx, Y: m.Position.Y + dy}
				if !ctx.Data.AreaData.IsWalkable(pos) {
					continue
				}
				candidates = append(candidates, pos)
			}
		}
	}

	if len(candidates) == 0 {
		return data.Position{}, false
	}

	var bestPos data.Position
	bestScore := -1e9

	novaRadius := NovaAggroMaxDistance

	for _, pos := range candidates {
		monstersInRange := 0
		minMonsterDist := 999

		for _, m := range pack {
			if m.Stats[stat.Life] <= 0 {
				continue
			}
			d := gridDistance(pos, m.Position)
			if d < minMonsterDist {
				minMonsterDist = d
			}
			if d <= novaRadius {
				monstersInRange++
			}
		}

		if monstersInRange == 0 {
			continue
		}

		distFromPlayer := gridDistance(playerPos, pos)
		distFromTarget := gridDistance(target.Position, pos)

		// Score:
		// - main reward: number of monsters hit by Nova
		// - light penalty: distance from target and player
		// - small penalty: standing literally on top of monsters
		score := float64(monstersInRange)*5.0 -
			float64(distFromTarget)*0.3 -
			float64(distFromPlayer)*0.2

		if minMonsterDist < 2 {
			score -= 2.0
		}

		if score > bestScore {
			bestScore = score
			bestPos = pos
		}
	}

	if bestScore <= -1e8 {
		return data.Position{}, false
	}

	return bestPos, true
}

func (s NovaSorceress) CheckKeyBindings() []skill.ID {
	requiredKeybindings := []skill.ID{
		skill.Nova,
		skill.Teleport,
		skill.TomeOfTownPortal,
		skill.StaticField,
	}

	missingKeybindings := make([]skill.ID, 0)

	for _, cskill := range requiredKeybindings {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(cskill); !found {
			missingKeybindings = append(missingKeybindings, cskill)
		}
	}

	// Check for one of the armor skills.
	armorSkills := []skill.ID{
		skill.FrozenArmor,
		skill.ShiverArmor,
		skill.ChillingArmor,
	}

	hasArmor := false
	for _, armor := range armorSkills {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(armor); found {
			hasArmor = true
			break
		}
	}

	if !hasArmor {
		missingKeybindings = append(missingKeybindings, skill.FrozenArmor)
	}

	if len(missingKeybindings) > 0 {
		s.Logger.Debug("There are missing required key bindings.", slog.Any("Bindings", missingKeybindings))
	}

	return missingKeybindings
}

func (s NovaSorceress) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	ctx := context.Get()
	completedAttackLoops := 0
	staticFieldCast := false

	var lastTargetID data.UnitID
	positionedForTarget := false

	for {
		ctx.PauseIfNotPriority()

		id, found := monsterSelector(*s.Data)
		if !found {
			return nil
		}

		// Reset aggressive positioning when target changes.
		if id != lastTargetID {
			lastTargetID = id
			positionedForTarget = false
		}

		if !s.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found || monster.Stats[stat.Life] <= 0 {
			return nil
		}

		// Aggressive Nova: try to find a better starting position inside the pack.
		if ctx.CharacterCfg.Character.NovaSorceress.AggressiveNovaPositioning && !positionedForTarget {
			if bestPos, ok := s.findBestAggressiveNovaPosition(monster); ok {
				if err := step.MoveTo(bestPos); err != nil {
					s.Logger.Debug("Aggressive Nova reposition failed", slog.String("error", err.Error()))
				}
			}
			// Do not try to reposition repeatedly for the same target.
			positionedForTarget = true
		}

		// Cast Static Field first if needed.
		if !staticFieldCast && s.shouldCastStaticField(monster) {
			staticOpts := []step.AttackOption{
				step.RangedDistance(StaticMinDistance, StaticMaxDistance),
			}

			if err := step.SecondaryAttack(skill.StaticField, monster.UnitID, 1, staticOpts...); err == nil {
				staticFieldCast = true
				continue
			}
		}

		// Choose Nova distance based on config (aggressive / normal).
		novaMin := NovaMinDistance
		novaMax := NovaMaxDistance
		if ctx.CharacterCfg.Character.NovaSorceress.AggressiveNovaPositioning {
			novaMin = NovaAggroMinDistance
			novaMax = NovaAggroMaxDistance
		}

		novaOpts := []step.AttackOption{
			step.RangedDistance(novaMin, novaMax),
		}

		if err := step.SecondaryAttack(skill.Nova, monster.UnitID, 1, novaOpts...); err == nil {
			completedAttackLoops++
		}

		if completedAttackLoops >= NovaMaxAttacksLoop {
			completedAttackLoops = 0
			staticFieldCast = false
		}
	}
}

func (s NovaSorceress) shouldCastStaticField(monster data.Monster) bool {
	// Only cast Static Field if monster HP is above threshold.
	maxLife := float64(monster.Stats[stat.MaxLife])
	if maxLife == 0 {
		return false
	}

	hpPercentage := (float64(monster.Stats[stat.Life]) / maxLife) * 100

	return hpPercentage > StaticFieldThreshold
}

func (s NovaSorceress) killBossWithStatic(bossID npc.ID, monsterType data.MonsterType) error {
	ctx := context.Get()

	for {
		ctx.PauseIfNotPriority()

		boss, found := s.Data.Monsters.FindOne(bossID, monsterType)
		if !found || boss.Stats[stat.Life] <= 0 {
			return nil
		}

		bossHPPercent := (float64(boss.Stats[stat.Life]) / float64(boss.Stats[stat.MaxLife])) * 100
		thresholdFloat := float64(ctx.CharacterCfg.Character.NovaSorceress.BossStaticThreshold)

		// Cast Static Field until boss HP is below threshold.
		if bossHPPercent > thresholdFloat {
			staticOpts := []step.AttackOption{
				step.Distance(StaticMinDistance, StaticMaxDistance),
			}

			err := step.SecondaryAttack(skill.StaticField, boss.UnitID, 1, staticOpts...)
			if err != nil {
				s.Logger.Warn("Failed to cast Static Field", slog.String("error", err.Error()))
			}

			continue
		}

		// Switch to Nova once boss HP is low enough.
		return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			return boss.UnitID, true
		}, nil)
	}
}

func (s NovaSorceress) killMonsterByName(id npc.ID, monsterType data.MonsterType, skipOnImmunities []stat.Resist) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		if m, found := d.Monsters.FindOne(id, monsterType); found {
			return m.UnitID, true
		}
		return 0, false
	}, skipOnImmunities)
}

func (s NovaSorceress) BuffSkills() []skill.ID {
	skillsList := make([]skill.ID, 0)

	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.EnergyShield); found {
		skillsList = append(skillsList, skill.EnergyShield)
	}

	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.ThunderStorm); found {
		skillsList = append(skillsList, skill.ThunderStorm)
	}

	// Add one of the armor skills.
	for _, armor := range []skill.ID{skill.ChillingArmor, skill.ShiverArmor, skill.FrozenArmor} {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(armor); found {
			skillsList = append(skillsList, armor)
			break
		}
	}

	return skillsList
}

func (s NovaSorceress) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

// ShouldIgnoreMonster implements context.Character and
// contains logic for skipping "trash" leftover monsters when aggressive Nova is enabled.
func (s NovaSorceress) ShouldIgnoreMonster(m data.Monster) bool {
	ctx := context.Get()

	// If aggressive Nova is not enabled, never ignore.
	if !ctx.CharacterCfg.Character.NovaSorceress.AggressiveNovaPositioning {
		return false
	}

	// Never ignore elites / bosses / important monsters.
	if m.IsElite() {
		return false
	}

	// Dead or invalid monsters do not matter here.
	if m.Stats[stat.Life] <= 0 || m.Stats[stat.MaxLife] <= 0 {
		return false
	}

	// Count how many normal (non-elite) monsters are within Nova radius around this monster.
	radius := NovaAggroMaxDistance
	normalCount := 0

	for _, other := range ctx.Data.Monsters.Enemies() {
		if other.Stats[stat.Life] <= 0 || other.Stats[stat.MaxLife] <= 0 {
			continue
		}
		if other.IsElite() {
			continue
		}
		if gridDistance(m.Position, other.Position) <= radius {
			normalCount++
		}
	}

	// If there are fewer than 3 normal monsters in Nova range around this one,
	// treat it as a small/irrelevant pack and allow higher-level logic to skip it.
	if normalCount < 3 {
		return true
	}

	return false
}

func (s NovaSorceress) KillAndariel() error {
	return s.killBossWithStatic(npc.Andariel, data.MonsterTypeUnique)
}

func (s NovaSorceress) KillDuriel() error {
	return s.killBossWithStatic(npc.Duriel, data.MonsterTypeUnique)
}

func (s NovaSorceress) KillMephisto() error {
	return s.killBossWithStatic(npc.Mephisto, data.MonsterTypeUnique)
}

func (s NovaSorceress) KillDiablo() error {
	timeout := time.Second * 20
	startTime := time.Now()
	diabloFound := false

	for {
		if time.Since(startTime) > timeout && !diabloFound {
			s.Logger.Error("Diablo was not found, timeout reached")
			return nil
		}

		diablo, found := s.Data.Monsters.FindOne(npc.Diablo, data.MonsterTypeUnique)
		if !found || diablo.Stats[stat.Life] <= 0 {
			if diabloFound {
				return nil
			}

			time.Sleep(200 * time.Millisecond)
			continue
		}

		diabloFound = true
		s.Logger.Info("Diablo detected, attacking")

		return s.killBossWithStatic(npc.Diablo, data.MonsterTypeUnique)
	}
}

func (s NovaSorceress) KillBaal() error {
	return s.killBossWithStatic(npc.BaalCrab, data.MonsterTypeUnique)
}

func (s NovaSorceress) KillCountess() error {
	return s.killMonsterByName(npc.DarkStalker, data.MonsterTypeSuperUnique, nil)
}

func (s NovaSorceress) KillSummoner() error {
	return s.killMonsterByName(npc.Summoner, data.MonsterTypeUnique, nil)
}

func (s NovaSorceress) KillIzual() error {
	return s.killBossWithStatic(npc.Izual, data.MonsterTypeUnique)
}

func (s NovaSorceress) KillCouncil() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		for _, m := range d.Monsters.Enemies() {
			if m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3 {
				return m.UnitID, true
			}
		}

		return 0, false
	}, nil)
}

func (s NovaSorceress) KillPindle() error {
	return s.killMonsterByName(
		npc.DefiledWarrior,
		data.MonsterTypeSuperUnique,
		s.CharacterCfg.Game.Pindleskin.SkipOnImmunities,
	)
}

func (s NovaSorceress) KillNihlathak() error {
	return s.killMonsterByName(npc.Nihlathak, data.MonsterTypeSuperUnique, nil)
}
