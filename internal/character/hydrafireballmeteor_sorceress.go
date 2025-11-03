package character

import (
	"fmt"
	"log/slog"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
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

const (
	hfm_sorceressMinDistance = 0
	hfm_sorceressMaxDistance = 30
)

type HydraFireballMeteorSorceress struct {
	BaseCharacter
}

func (s HydraFireballMeteorSorceress) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s HydraFireballMeteorSorceress) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{skill.Meteor, skill.Hydra, skill.Teleport, skill.TomeOfTownPortal, skill.FrozenArmor, skill.StaticField}
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

func (s HydraFireballMeteorSorceress) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	hydraCount := 0

	for {
		context.Get().PauseIfNotPriority()

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

		if completedAttackLoops >= sorceressMaxAttacksLoop {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			s.Logger.Info("Monster not found", slog.String("monster", fmt.Sprintf("%v", monster)))
			return nil
		}

		opts := step.Distance(hfm_sorceressMinDistance, hfm_sorceressMaxDistance)

		step.SecondaryAttack(skill.Meteor, id, 1, opts)

		// Cast Hydra for the first 6 attack loops to spawn max hydras
		if hydraCount < 6 {
			hydraCount++
			time.Sleep(50 * time.Millisecond)
			step.SecondaryAttack(skill.Hydra, id, 1, opts)
		}

		if s.Data.PlayerUnit.States.HasState(state.Cooldown) {
			time.Sleep(50 * time.Millisecond)
			step.PrimaryAttack(id, 2, true, opts)
		}

		completedAttackLoops++
		previousUnitID = int(id)
	}
}

func (s HydraFireballMeteorSorceress) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}

		return m.UnitID, true
	}, nil)
}

func (s HydraFireballMeteorSorceress) killMonsterByName(id npc.ID, monsterType data.MonsterType, _ int, _ bool, skipOnImmunities []stat.Resist) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		if m, found := d.Monsters.FindOne(id, monsterType); found {
			return m.UnitID, true
		}

		return 0, false
	}, skipOnImmunities)
}

func (s HydraFireballMeteorSorceress) BuffSkills() []skill.ID {
	skillsList := make([]skill.ID, 0)
	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.EnergyShield); found {
		skillsList = append(skillsList, skill.EnergyShield)
	}

	armors := []skill.ID{skill.ChillingArmor, skill.ShiverArmor, skill.FrozenArmor}
	for _, armor := range armors {
		if _, found := s.Data.KeyBindings.KeyBindingForSkill(armor); found {
			skillsList = append(skillsList, armor)
			return skillsList
		}
	}

	return skillsList
}

func (s HydraFireballMeteorSorceress) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

func (s HydraFireballMeteorSorceress) KillCountess() error {
	return s.killMonsterByName(npc.DarkStalker, data.MonsterTypeSuperUnique, hfm_sorceressMaxDistance, false, nil)
}

func (s HydraFireballMeteorSorceress) KillAndariel() error {
	return s.killMonsterByName(npc.Andariel, data.MonsterTypeUnique, hfm_sorceressMaxDistance, false, nil)
}

func (s HydraFireballMeteorSorceress) KillSummoner() error {
	return s.killMonsterByName(npc.Summoner, data.MonsterTypeUnique, hfm_sorceressMaxDistance, false, nil)
}

func (s HydraFireballMeteorSorceress) KillDuriel() error {
	return s.killMonsterByName(npc.Duriel, data.MonsterTypeUnique, hfm_sorceressMaxDistance, true, nil)
}

func (s HydraFireballMeteorSorceress) KillCouncil() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		// Exclude monsters that are not council members
		var councilMembers []data.Monster
		var veryImmunes []data.Monster
		for _, m := range d.Monsters.Enemies() {
			if m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3 {
				if m.IsImmune(stat.ColdImmune) && m.IsImmune(stat.FireImmune) {
					veryImmunes = append(veryImmunes, m)
				} else {
					councilMembers = append(councilMembers, m)
				}
			}
		}

		councilMembers = append(councilMembers, veryImmunes...)

		for _, m := range councilMembers {
			return m.UnitID, true
		}

		return 0, false
	}, nil)
}

func (s HydraFireballMeteorSorceress) KillMephisto() error {
	if !s.CharacterCfg.Character.HydraFireballMeteorSorceress.UseMoatTrick {
		return s.killMonsterByName(npc.Mephisto, data.MonsterTypeUnique, hfm_sorceressMaxDistance, true, nil)
	}

	// Moat trick positioning
	err := step.MoveTo(data.Position{X: 17563, Y: 8072}, step.WithIgnoreMonsters())
	if err != nil {
		return err
	}

	utils.Sleep(350)

	type positionAndWaitTime struct {
		x        int
		y        int
		duration int
	}

	initialPositions := []positionAndWaitTime{
		{17575, 8086, 350}, {17584, 8088, 1200},
		{17600, 8090, 550}, {17609, 8090, 2500},
	}

	for _, pos := range initialPositions {
		err := step.MoveTo(data.Position{X: pos.x, Y: pos.y}, step.WithIgnoreMonsters())
		if err != nil {
			return err
		}
		utils.Sleep(pos.duration)
	}

	err = action.ClearAreaAroundPosition(data.Position{X: 17609, Y: 8090}, 10, data.MonsterAnyFilter())
	if err != nil {
		return err
	}

	err = step.MoveTo(data.Position{X: 17609, Y: 8090}, step.WithIgnoreMonsters())
	if err != nil {
		return err
	}

	// Attack from moat position
	// Cannot reuse KillMonsterSequence because moat trick requires:
	// 1. Much longer attack range (15-80 vs 0-30)
	// 2. More attack loops (100 vs sorceressMaxAttacksLoop)
	// 3. ForceAttack enabled to attack through the moat
	ctx := context.Get()
	opts := step.Distance(15, 80)
	ctx.ForceAttack = true
	defer func() {
		ctx.ForceAttack = false
	}()

	maxAttack := 100
	attackCount := 0

	for attackCount < maxAttack {
		ctx.PauseIfNotPriority()

		monster, found := s.Data.Monsters.FindOne(npc.Mephisto, data.MonsterTypeUnique)
		if !found {
			return nil
		}

		if s.Data.PlayerUnit.States.HasState(state.Cooldown) {
			step.SecondaryAttack(skill.Hydra, monster.UnitID, 1, opts)
			step.PrimaryAttack(monster.UnitID, 2, true, opts)
			utils.Sleep(50)
		}

		step.SecondaryAttack(skill.Meteor, monster.UnitID, 1, opts)
		utils.Sleep(100)
		attackCount++
	}

	return nil
}

func (s HydraFireballMeteorSorceress) KillIzual() error {
	m, _ := s.Data.Monsters.FindOne(npc.Izual, data.MonsterTypeUnique)
	_ = step.SecondaryAttack(skill.StaticField, m.UnitID, 4, step.Distance(5, 8))

	return s.killMonster(npc.Izual, data.MonsterTypeUnique)
}

func (s HydraFireballMeteorSorceress) KillDiablo() error {
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
			// Already dead
			if diabloFound {
				return nil
			}

			// Keep waiting...
			time.Sleep(200)
			continue
		}

		diabloFound = true
		s.Logger.Info("Diablo detected, attacking")

		_ = step.SecondaryAttack(skill.StaticField, diablo.UnitID, 5, step.Distance(3, 8))

		return s.killMonster(npc.Diablo, data.MonsterTypeUnique)
	}
}

func (s HydraFireballMeteorSorceress) KillPindle() error {
	return s.killMonsterByName(npc.DefiledWarrior, data.MonsterTypeSuperUnique, hfm_sorceressMaxDistance, false, s.CharacterCfg.Game.Pindleskin.SkipOnImmunities)
}

func (s HydraFireballMeteorSorceress) KillNihlathak() error {
	return s.killMonsterByName(npc.Nihlathak, data.MonsterTypeSuperUnique, hfm_sorceressMaxDistance, false, nil)
}

func (s HydraFireballMeteorSorceress) KillBaal() error {
	m, _ := s.Data.Monsters.FindOne(npc.BaalCrab, data.MonsterTypeUnique)
	step.SecondaryAttack(skill.StaticField, m.UnitID, 5, step.Distance(5, 8))

	return s.killMonster(npc.BaalCrab, data.MonsterTypeUnique)
}
