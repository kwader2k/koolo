package character

import (
	"fmt"
	"log/slog"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/utils"
)

var _ context.LevelingCharacter = (*WarlockLeveling)(nil)

const (
	warlockMaxAttacksLoop = 3
	warlockMinDistance    = 10
	warlockMaxDistance    = 15
	warlockDangerDistance = 4
	warlockSafeDistance   = 6
)

type WarlockLeveling struct {
	BaseCharacter
}

func (s WarlockLeveling) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s WarlockLeveling) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{}
	missingKeybindings := []skill.ID{}

	for _, cskill := range requireKeybindings {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(cskill); !found {
			missingKeybindings = append(missingKeybindings, cskill)
		}
	}

	if len(missingKeybindings) > 0 {
		s.Logger.Debug("There are missing required key bindings.", slog.Any("Bindings", missingKeybindings))
	}

	return missingKeybindings
}

func (s WarlockLeveling) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	var lastReposition time.Time
	var lastConsume time.Time

	for {
		context.Get().PauseIfNotPriority()

		if s.Context.Data.PlayerUnit.IsDead() {
			return nil
		}

		id, found := monsterSelector(*s.Data)
		if !found {
			return nil
		}
		if previousUnitID != int(id) {
			completedAttackLoops = 0
		}

		if !s.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		if completedAttackLoops >= warlockMaxAttacksLoop {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			s.Logger.Info("Monster not found", slog.String("monster", fmt.Sprintf("%v", monster)))
			return nil
		}

		lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
		mana, _ := s.Data.PlayerUnit.FindStat(stat.Mana, 0)
		onCooldown := s.Data.PlayerUnit.States.HasState(state.Cooldown)

		canReposition := lvl.Value > 12 && time.Since(lastReposition) > time.Second*4
		if canReposition {
			isAnyEnemyNearby, _ := action.IsAnyEnemyAroundPlayer(warlockDangerDistance)
			if isAnyEnemyNearby {
				if safePos, found := action.FindSafePosition(monster, warlockDangerDistance, warlockSafeDistance, warlockMinDistance, warlockMaxDistance); found {
					step.MoveTo(safePos, step.WithIgnoreMonsters())
					lastReposition = time.Now()
				}
			}
		}

		if lvl.Value < 49 {
			// Pre-respec: Fire-focused build
			if onCooldown {
				if s.Data.PlayerUnit.Skills[skill.MiasmaBolt].Level > 0 && mana.Value > 2 {
					step.SecondaryAttack(skill.MiasmaBolt, id, 4, step.Distance(warlockMinDistance, warlockMaxDistance))
				} else {
					step.PrimaryAttack(id, 1, true, step.Distance(1, 3))
				}
			} else if lvl.Value >= 30 && s.Data.PlayerUnit.Skills[skill.Apocalypse].Level > 0 && mana.Value > 15 {
				step.SecondaryAttack(skill.Apocalypse, id, 3, step.Distance(5, 10))
			} else if lvl.Value >= 18 && s.Data.PlayerUnit.Skills[skill.FlameWave].Level > 0 && mana.Value > 8 {
				step.SecondaryAttack(skill.FlameWave, id, 4, step.Distance(8, 13))
			} else if lvl.Value >= 6 && s.Data.PlayerUnit.Skills[skill.RingOfFire].Level > 0 && mana.Value > 5 {
				step.SecondaryAttack(skill.RingOfFire, id, 5, step.Distance(3, 7))
			} else if s.Data.PlayerUnit.Skills[skill.MiasmaBolt].Level > 0 && mana.Value > 2 {
				step.SecondaryAttack(skill.MiasmaBolt, id, 4, step.Distance(warlockMinDistance, warlockMaxDistance))
			} else {
				step.PrimaryAttack(id, 1, true, step.Distance(1, 3))
			}
		} else {
			// Post-respec: Magic-focused build
			if time.Since(lastConsume) > time.Second && mana.Value > 6 && s.castConsumeOnNearbyCorpse(10) {
				lastConsume = time.Now()
			}

			opts := []step.AttackOption{step.Distance(warlockMinDistance, warlockMaxDistance)}
			if onCooldown {
				if s.Data.PlayerUnit.Skills[skill.MiasmaChains].Level > 0 && mana.Value > 5 {
					step.SecondaryAttack(skill.MiasmaChains, id, 3, opts...)
				} else if s.Data.PlayerUnit.Skills[skill.MiasmaBolt].Level > 0 && mana.Value > 2 {
					step.SecondaryAttack(skill.MiasmaBolt, id, 3, opts...)
				} else {
					step.PrimaryAttack(id, 1, true, step.Distance(1, 3))
				}
			} else if s.Data.PlayerUnit.Skills[skill.Abyss].Level > 0 && mana.Value > 15 {
				step.SecondaryAttack(skill.Abyss, id, 3, opts...)
			} else if s.Data.PlayerUnit.Skills[skill.MiasmaChains].Level > 0 && mana.Value > 5 {
				step.SecondaryAttack(skill.MiasmaChains, id, 3, opts...)
			} else if s.Data.PlayerUnit.Skills[skill.MiasmaBolt].Level > 0 && mana.Value > 2 {
				step.SecondaryAttack(skill.MiasmaBolt, id, 3, opts...)
			} else {
				step.PrimaryAttack(id, 1, true, step.Distance(1, 3))
			}
		}

		completedAttackLoops++
		previousUnitID = int(id)
		time.Sleep(time.Millisecond * 100)
	}
}

func (s WarlockLeveling) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}
		return m.UnitID, true
	}, nil)
}

func (s WarlockLeveling) BuffSkills() []skill.ID {
	return []skill.ID{}
}

func (s WarlockLeveling) PreCTABuffSkills() []skill.ID {
	maxSummons := min(1+action.GetSkillTotalLevel(skill.DemonicMastery)/10, 3)

	currentSummons := uint(0)
	for _, m := range s.Data.Monsters {
		if m.States.HasState(state.BindDemon) {
			currentSummons++
		}
		if m.IsPet() {
			switch m.Name {
			case npc.WarGoatman, npc.Tainted3, npc.WarDefiler:
				currentSummons++
			}
		}
	}

	if currentSummons >= maxSummons {
		return nil
	}

	remaining := maxSummons - currentSummons
	for _, summon := range []skill.ID{skill.SummonDefiler, skill.SummonTainted, skill.SummonGoatman} {
		if action.GetSkillTotalLevel(summon) <= 0 {
			continue
		}
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(summon); !found {
			continue
		}
		skills := make([]skill.ID, remaining)
		for i := range skills {
			skills[i] = summon
		}
		return skills
	}

	return nil
}

func (s WarlockLeveling) ShouldResetSkills() bool {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
	if lvl.Value >= 49 && s.Data.PlayerUnit.Skills[skill.RingOfFire].Level > 10 {
		s.Logger.Info("Resetting skills: Level 49+ and Ring of Fire level > 10")
		return true
	}
	return false
}

func (s WarlockLeveling) SkillsToBind() (skill.ID, []skill.ID) {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	mainSkill := skill.AttackSkill
	skillBindings := []skill.ID{}

	if miasmaBolt, found := s.Data.PlayerUnit.Skills[skill.MiasmaBolt]; found && miasmaBolt.Level > 0 {
		skillBindings = append(skillBindings, skill.MiasmaBolt)
	}

	if lvl.Value >= 6 {
		skillBindings = append(skillBindings, skill.RingOfFire)
	}

	if lvl.Value >= 18 {
		skillBindings = append(skillBindings, skill.FlameWave)
	}

	if lvl.Value >= 30 {
		skillBindings = append(skillBindings, skill.Apocalypse)
	}

	if lvl.Value >= 49 {
		// Post-respec: Magic build with summons
		mainSkill = skill.AttackSkill
		skillBindings = []skill.ID{
			skill.Abyss,
			skill.MiasmaChains,
			skill.MiasmaBolt,
			skill.SummonGoatman,
			skill.SummonTainted,
			skill.SummonDefiler,
			skill.Consume,
		}
	}

	if s.Data.PlayerUnit.Skills[skill.BattleCommand].Level > 0 {
		skillBindings = append(skillBindings, skill.BattleCommand)
	}
	if s.Data.PlayerUnit.Skills[skill.BattleOrders].Level > 0 {
		skillBindings = append(skillBindings, skill.BattleOrders)
	}

	_, found := s.Data.Inventory.Find(item.TomeOfTownPortal, item.LocationInventory)
	if found {
		skillBindings = append(skillBindings, skill.TomeOfTownPortal)
	}

	s.Logger.Info("Skills bound", "mainSkill", mainSkill, "skillBindings", skillBindings)
	return mainSkill, skillBindings
}

func (s WarlockLeveling) StatPoints() []context.StatAllocation {
	stats := []context.StatAllocation{
		{Stat: stat.Vitality, Points: 30},
		{Stat: stat.Energy, Points: 35},
		{Stat: stat.Vitality, Points: 45},
		{Stat: stat.Strength, Points: 30},
		{Stat: stat.Vitality, Points: 85},
		{Stat: stat.Strength, Points: 35},
		{Stat: stat.Vitality, Points: 90},
		{Stat: stat.Strength, Points: 40},
		{Stat: stat.Vitality, Points: 999},
	}
	s.Logger.Debug("Stat point allocation plan", "stats", stats)
	return stats
}

func (s WarlockLeveling) SkillPoints() []skill.ID {
	lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)

	var skillSequence []skill.ID

	if lvl.Value < 49 {
		skillSequence = []skill.ID{
			// Levels 2-5: MiasmaBolt
			skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt,
			skill.MiasmaBolt, // Den of Evil
			// Level 6-15: RingOfFire
			skill.RingOfFire, skill.RingOfFire, skill.RingOfFire, skill.RingOfFire, skill.RingOfFire,
			skill.RingOfFire, skill.RingOfFire, skill.RingOfFire, skill.RingOfFire, skill.RingOfFire,
			skill.SigilLethargy, // Radament reward
			// Level 16-18: RingOfFire
			skill.RingOfFire, skill.RingOfFire,
			// Level 18-23: FlameWave
			skill.FlameWave, skill.FlameWave, skill.FlameWave,
			skill.FlameWave, // Lam Essens
			skill.FlameWave, skill.FlameWave, skill.FlameWave,
			skill.SigilRancor, // Izual
			skill.SigilDeath,  // Izual
			// Level 24-36: Max FlameWave
			skill.FlameWave, skill.FlameWave, skill.FlameWave, skill.FlameWave, skill.FlameWave,
			skill.FlameWave, skill.FlameWave, skill.FlameWave, skill.FlameWave, skill.FlameWave,
			skill.FlameWave, skill.FlameWave,
			skill.FlameWave, // Radament NM
			// Level 37-Respec (49): Apocalypse
			skill.Apocalypse, skill.Apocalypse, skill.Apocalypse, skill.Apocalypse, skill.Apocalypse,
			skill.Apocalypse, skill.Apocalypse, skill.Apocalypse, skill.Apocalypse, skill.Apocalypse,
			skill.Apocalypse, skill.Apocalypse,
		}
	} else {
		// Post-respec: Magic build (Miasma/Abyss) with demon summoning
		skillSequence = []skill.ID{
			// Summoning prereqs and utility
			skill.SummonGoatman, skill.DemonicMastery, skill.BloodOath,
			skill.SummonTainted, skill.SummonDefiler,
			skill.Consume,
			// Main damage: MiasmaBolt → MiasmaChains → EnhancedEntropy → Abyss
			skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt,
			skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt,
			skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains,
			skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains,
			skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains,
			skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains, skill.MiasmaChains,
			skill.EnhancedEntropy,
			skill.Abyss, skill.Abyss, skill.Abyss, skill.Abyss, skill.Abyss,
			skill.Abyss, skill.Abyss, skill.Abyss, skill.Abyss, skill.Abyss,
			skill.Abyss, skill.Abyss, skill.Abyss, skill.Abyss, skill.Abyss,
			skill.Abyss, skill.Abyss, skill.Abyss, skill.Abyss, skill.Abyss,
			// Remaining points: more MiasmaBolt synergy
			skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt,
			skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt, skill.MiasmaBolt,
			skill.MiasmaBolt,
			skill.EnhancedEntropy, skill.EnhancedEntropy, skill.EnhancedEntropy, skill.EnhancedEntropy, skill.EnhancedEntropy,
			skill.EnhancedEntropy, skill.EnhancedEntropy, skill.EnhancedEntropy, skill.EnhancedEntropy, skill.EnhancedEntropy,
			skill.EnhancedEntropy, skill.EnhancedEntropy, skill.EnhancedEntropy, skill.EnhancedEntropy, skill.EnhancedEntropy,
		}
	}

	return skillSequence
}

func (s WarlockLeveling) killBoss(bossNPC npc.ID, timeout time.Duration) error {
	s.Logger.Info(fmt.Sprintf("Starting kill sequence for %v...", bossNPC))
	startTime := time.Now()
	lastConsume := time.Time{}

	for {
		context.Get().PauseIfNotPriority()

		if time.Since(startTime) > timeout {
			s.Logger.Error(fmt.Sprintf("Timed out waiting for %v.", bossNPC))
			return fmt.Errorf("%v timeout", bossNPC)
		}

		if s.Context.Data.PlayerUnit.IsDead() {
			s.Logger.Info("Player detected as dead, stopping boss kill sequence.")
			return nil
		}

		boss, found := s.Data.Monsters.FindOne(bossNPC, data.MonsterTypeUnique)
		if !found {
			time.Sleep(time.Second)
			continue
		}

		if boss.Stats[stat.Life] <= 0 {
			s.Logger.Info(fmt.Sprintf("%v has been defeated.", bossNPC))
			if bossNPC == npc.BaalCrab {
				s.Logger.Info("Waiting...")
				time.Sleep(time.Second * 1)
			}
			return nil
		}

		lvl, _ := s.Data.PlayerUnit.FindStat(stat.Level, 0)
		mana, _ := s.Data.PlayerUnit.FindStat(stat.Mana, 0)
		onCooldown := s.Data.PlayerUnit.States.HasState(state.Cooldown)

		if lvl.Value < 49 {
			if onCooldown {
				if s.Data.PlayerUnit.Skills[skill.MiasmaBolt].Level > 0 && mana.Value > 2 {
					step.SecondaryAttack(skill.MiasmaBolt, boss.UnitID, 4, step.Distance(10, 15))
				} else {
					step.PrimaryAttack(boss.UnitID, 1, true, step.Distance(1, 3))
				}
			} else if s.Data.PlayerUnit.Skills[skill.FlameWave].Level > 0 && mana.Value > 8 {
				step.SecondaryAttack(skill.FlameWave, boss.UnitID, 4, step.Distance(8, 13))
			} else if s.Data.PlayerUnit.Skills[skill.RingOfFire].Level > 0 && mana.Value > 5 {
				step.SecondaryAttack(skill.RingOfFire, boss.UnitID, 5, step.Distance(3, 7))
			} else if s.Data.PlayerUnit.Skills[skill.MiasmaBolt].Level > 0 && mana.Value > 2 {
				step.SecondaryAttack(skill.MiasmaBolt, boss.UnitID, 4, step.Distance(10, 15))
			} else {
				step.PrimaryAttack(boss.UnitID, 1, true, step.Distance(1, 3))
			}
		} else {
			if time.Since(lastConsume) > time.Second && mana.Value > 6 && s.castConsumeOnNearbyCorpse(10) {
				lastConsume = time.Now()
			}

			if onCooldown {
				if s.Data.PlayerUnit.Skills[skill.MiasmaChains].Level > 0 && mana.Value > 5 {
					step.SecondaryAttack(skill.MiasmaChains, boss.UnitID, 3, step.Distance(10, 15))
				} else if s.Data.PlayerUnit.Skills[skill.MiasmaBolt].Level > 0 && mana.Value > 2 {
					step.SecondaryAttack(skill.MiasmaBolt, boss.UnitID, 4, step.Distance(10, 15))
				} else {
					step.PrimaryAttack(boss.UnitID, 1, true, step.Distance(1, 3))
				}
			} else if s.Data.PlayerUnit.Skills[skill.Abyss].Level > 0 && mana.Value > 15 {
				step.SecondaryAttack(skill.Abyss, boss.UnitID, 3, step.Distance(10, 15))
			} else if s.Data.PlayerUnit.Skills[skill.MiasmaChains].Level > 0 && mana.Value > 5 {
				step.SecondaryAttack(skill.MiasmaChains, boss.UnitID, 3, step.Distance(10, 15))
			} else if s.Data.PlayerUnit.Skills[skill.MiasmaBolt].Level > 0 && mana.Value > 2 {
				step.SecondaryAttack(skill.MiasmaBolt, boss.UnitID, 4, step.Distance(10, 15))
			} else {
				step.PrimaryAttack(boss.UnitID, 1, true, step.Distance(1, 3))
			}
		}

		time.Sleep(time.Millisecond * 100)
	}
}

func (s WarlockLeveling) castConsumeOnNearbyCorpse(maxDistance int) bool {
	if s.Data.PlayerUnit.Skills[skill.Consume].Level <= 0 {
		return false
	}

	consumeKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Consume)
	if !found {
		return false
	}

	for _, corpse := range s.Data.Corpses {
		if corpse.States.HasState(state.CorpseNoselect) || corpse.States.HasState(state.CorpseNodraw) {
			continue
		}
		if s.PathFinder.DistanceFromMe(corpse.Position) > maxDistance {
			continue
		}
		if !s.PathFinder.LineOfSight(s.Data.PlayerUnit.Position, corpse.Position) {
			continue
		}

		if s.Data.PlayerUnit.RightSkill != skill.Consume {
			s.HID.PressKeyBinding(consumeKey)
			utils.Sleep(50)
		}

		s.HID.KeyDown(s.Data.KeyBindings.StandStill)
		x, y := s.PathFinder.GameCoordsToScreenCords(corpse.Position.X, corpse.Position.Y)
		s.HID.Click(game.RightButton, x, y)
		s.HID.KeyUp(s.Data.KeyBindings.StandStill)
		utils.Sleep(50)
		return true
	}

	return false
}

func (s WarlockLeveling) killMonsterByName(id npc.ID, monsterType data.MonsterType, skipOnImmunities []stat.Resist) error {
	s.Logger.Info(fmt.Sprintf("Starting persistent kill sequence for %v...", id))

	for {
		monster, found := s.Data.Monsters.FindOne(id, monsterType)
		if !found {
			s.Logger.Info(fmt.Sprintf("%v not found, assuming dead.", id))
			return nil
		}

		if monster.Stats[stat.Life] <= 0 {
			s.Logger.Info(fmt.Sprintf("%v is dead.", id))
			return nil
		}

		err := s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
			m, found := d.Monsters.FindOne(id, monsterType)
			if !found {
				return 0, false
			}
			return m.UnitID, true
		}, skipOnImmunities)

		if err != nil {
			s.Logger.Warn(fmt.Sprintf("Error during KillMonsterSequence for %v: %v", id, err))
		}

		time.Sleep(time.Millisecond * 250)
	}
}

func (s WarlockLeveling) KillCountess() error {
	return s.killMonsterByName(npc.DarkStalker, data.MonsterTypeSuperUnique, nil)
}

func (s WarlockLeveling) KillAndariel() error {
	return s.killBoss(npc.Andariel, time.Second*220)
}

func (s WarlockLeveling) KillSummoner() error {
	return s.killMonsterByName(npc.Summoner, data.MonsterTypeUnique, nil)
}

func (s WarlockLeveling) KillDuriel() error {
	return s.killBoss(npc.Duriel, time.Second*220)
}

func (s WarlockLeveling) KillCouncil() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		var councilMembers []data.Monster
		for _, m := range d.Monsters {
			if m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3 {
				councilMembers = append(councilMembers, m)
			}
		}

		sort.Slice(councilMembers, func(i, j int) bool {
			distanceI := s.PathFinder.DistanceFromMe(councilMembers[i].Position)
			distanceJ := s.PathFinder.DistanceFromMe(councilMembers[j].Position)
			return distanceI < distanceJ
		})

		if len(councilMembers) > 0 {
			return councilMembers[0].UnitID, true
		}

		return 0, false
	}, nil)
}

func (s WarlockLeveling) KillMephisto() error {
	return s.killBoss(npc.Mephisto, time.Second*220)
}

func (s WarlockLeveling) KillIzual() error {
	return s.killBoss(npc.Izual, time.Second*220)
}

func (s WarlockLeveling) KillDiablo() error {
	return s.killBoss(npc.Diablo, time.Second*220)
}

func (s WarlockLeveling) KillPindle() error {
	return s.killMonsterByName(npc.DefiledWarrior, data.MonsterTypeSuperUnique, nil)
}

func (s WarlockLeveling) KillAncients() error {
	originalBackToTownCfg := s.CharacterCfg.BackToTown
	s.CharacterCfg.BackToTown.NoHpPotions = false
	s.CharacterCfg.BackToTown.NoMpPotions = false
	s.CharacterCfg.BackToTown.EquipmentBroken = false
	s.CharacterCfg.BackToTown.MercDied = false

	for _, m := range s.Data.Monsters.Enemies(data.MonsterEliteFilter()) {
		foundMonster, found := s.Data.Monsters.FindOne(m.Name, data.MonsterTypeSuperUnique)
		if !found {
			continue
		}
		step.MoveTo(data.Position{X: 10062, Y: 12639})
		s.killMonster(foundMonster.Name, data.MonsterTypeSuperUnique)
	}

	s.CharacterCfg.BackToTown = originalBackToTownCfg
	s.Logger.Info("Restored original back-to-town checks after Ancients fight.")
	return nil
}

func (s WarlockLeveling) KillNihlathak() error {
	return s.killMonsterByName(npc.Nihlathak, data.MonsterTypeSuperUnique, nil)
}

func (s WarlockLeveling) KillBaal() error {
	return s.killBoss(npc.BaalCrab, time.Second*240)
}

func (s WarlockLeveling) GetAdditionalRunewords() []string {
	return action.GetCastersCommonRunewords()
}

func (s WarlockLeveling) InitialCharacterConfigSetup() {
}

func (s WarlockLeveling) AdjustCharacterConfig() {
}
