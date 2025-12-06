package character

import (
	"fmt"
	"log/slog"
	"sort"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/utils"
)

const (
	maxJavazonAttackLoops = 10
	minJavazonDistance    = 10
	maxJavazonDistance    = 30
)

type Javazon struct {
	BaseCharacter
}

func (s Javazon) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

func (s Javazon) CheckKeyBindings() []skill.ID {
	requireKeybindings := []skill.ID{skill.LightningFury, skill.ChargedStrike, skill.TomeOfTownPortal}
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

func (s Javazon) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	const numOfAttacks = 5

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

		if completedAttackLoops >= maxJavazonAttackLoops {
			return nil
		}

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			s.Logger.Info("Monster not found", slog.String("monster", fmt.Sprintf("%v", monster)))
			return nil
		}

		closeMonsters := 0
		for _, mob := range s.Data.Monsters {
			if mob.IsPet() || mob.IsMerc() || mob.IsGoodNPC() || mob.IsSkip() || monster.Stats[stat.Life] <= 0 && mob.UnitID != monster.UnitID {
				continue
			}
			if pather.DistanceFromPoint(mob.Position, monster.Position) <= 15 {
				closeMonsters++
			}
			if closeMonsters >= 3 {
				break
			}
		}

		if closeMonsters >= 3 {
			step.SecondaryAttack(skill.LightningFury, id, numOfAttacks, step.Distance(minJavazonDistance, maxJavazonDistance))
		} else {
			if s.Data.PlayerUnit.Skills[skill.ChargedStrike].Level > 0 {
				for i := 0; i < numOfAttacks; i++ {
					s.chargedStrike(id)
				}
			} else {
				step.PrimaryAttack(id, numOfAttacks, false, step.Distance(1, 1))
			}
		}

		completedAttackLoops++
		previousUnitID = int(id)
	}
}

func (s Javazon) KillBossSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),
	skipOnImmunities []stat.Resist,
) error {
	completedAttackLoops := 0
	previousUnitID := 0
	const numOfAttacks = 5

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

		monster, found := s.Data.Monsters.FindByID(id)
		if !found {
			return nil
		}

		if monster.Stats[stat.Life] <= 0 {
			return nil
		}

		if completedAttackLoops >= maxJavazonAttackLoops {
			completedAttackLoops = 0
			continue
		}

		if s.Data.PlayerUnit.Skills[skill.ChargedStrike].Level > 0 {
			for i := 0; i < numOfAttacks; i++ {
				s.chargedStrike(id)
			}
		} else {
			step.PrimaryAttack(id, numOfAttacks, false, step.Distance(1, 1))
		}

		completedAttackLoops++
		previousUnitID = int(id)
	}
}

func (s Javazon) chargedStrike(monsterID data.UnitID) {
	ctx := context.Get()
	ctx.PauseIfNotPriority()

	monster, found := s.Data.Monsters.FindByID(monsterID)
	if !found {
		return
	}

	distance := ctx.PathFinder.DistanceFromMe(monster.Position)
	if distance > 5 {
		_ = action.MoveToCoords(monster.Position, step.WithDistanceToFinish(3))
		ctx.RefreshGameData()
		monster, found = s.Data.Monsters.FindByID(monsterID)
		if !found {
			return
		}
	}

	csKey, found := s.Data.KeyBindings.KeyBindingForSkill(skill.ChargedStrike)
	if found && s.Data.PlayerUnit.RightSkill != skill.ChargedStrike {
		ctx.HID.PressKeyBinding(csKey)
		utils.Sleep(50)
	}

	screenX, screenY := ctx.PathFinder.GameCoordsToScreenCords(monster.Position.X, monster.Position.Y)
	ctx.HID.Click(game.RightButton, screenX, screenY)
}

func (s Javazon) killMonster(npc npc.ID, t data.MonsterType) error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}

		return m.UnitID, true
	}, nil)
}

func (s Javazon) killBoss(npc npc.ID, t data.MonsterType) error {
	return s.KillBossSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc, t)
		if !found {
			return 0, false
		}

		return m.UnitID, true
	}, nil)
}

func (s Javazon) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

func (s Javazon) shouldSummonValkyrie() bool {
	if s.Data.PlayerUnit.Area.IsTown() {
		return false
	}

	if _, found := s.Data.KeyBindings.KeyBindingForSkill(skill.Valkyrie); found {
		needsValkyrie := true

		for _, monster := range s.Data.Monsters {
			if monster.IsPet() {
				switch monster.Name {
				case npc.Valkyrie:
					needsValkyrie = false
				}
			}
		}
		return needsValkyrie
	}

	return false
}

func (s Javazon) BuffSkills() []skill.ID {
	if s.shouldSummonValkyrie() {
		return []skill.ID{skill.Valkyrie}
	}
	return []skill.ID{}
}

func (s Javazon) KillCountess() error {
	return s.killMonster(npc.DarkStalker, data.MonsterTypeSuperUnique)
}

func (s Javazon) KillAndariel() error {
	return s.killBoss(npc.Andariel, data.MonsterTypeUnique)
}

func (s Javazon) KillSummoner() error {
	return s.killMonster(npc.Summoner, data.MonsterTypeUnique)
}

func (s Javazon) KillDuriel() error {
	return s.killBoss(npc.Duriel, data.MonsterTypeUnique)
}

func (s Javazon) KillCouncil() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		// Exclude monsters that are not council members
		var councilMembers []data.Monster
		for _, m := range d.Monsters {
			if m.Name == npc.CouncilMember || m.Name == npc.CouncilMember2 || m.Name == npc.CouncilMember3 {
				councilMembers = append(councilMembers, m)
			}
		}

		// Order council members by distance
		sort.Slice(councilMembers, func(i, j int) bool {
			distanceI := s.PathFinder.DistanceFromMe(councilMembers[i].Position)
			distanceJ := s.PathFinder.DistanceFromMe(councilMembers[j].Position)

			return distanceI < distanceJ
		})

		for _, m := range councilMembers {
			return m.UnitID, true
		}

		return 0, false
	}, nil)
}

func (s Javazon) KillMephisto() error {
	return s.killBoss(npc.Mephisto, data.MonsterTypeUnique)
}

func (s Javazon) KillIzual() error {
	return s.killBoss(npc.Izual, data.MonsterTypeUnique)
}

func (s Javazon) KillDiablo() error {
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

		return s.killMonster(npc.Diablo, data.MonsterTypeUnique)
	}
}

func (s Javazon) KillPindle() error {
	return s.KillMonsterSequence(func(d game.Data) (data.UnitID, bool) {
		m, found := d.Monsters.FindOne(npc.DefiledWarrior, data.MonsterTypeSuperUnique)
		if !found {
			return 0, false
		}
		return m.UnitID, true
	}, nil)
}

func (s Javazon) KillNihlathak() error {
	return s.killBoss(npc.Nihlathak, data.MonsterTypeSuperUnique)
}

func (s Javazon) KillBaal() error {
	return s.killBoss(npc.BaalCrab, data.MonsterTypeUnique)
}

func (f Javazon) KillUberDuriel() error {
	return f.killBoss(npc.UberDuriel, data.MonsterTypeUnique)
}

func (f Javazon) KillUberIzual() error {
	return f.killBoss(npc.UberIzual, data.MonsterTypeUnique)
}

func (f Javazon) KillLilith() error {
	return f.killBoss(npc.Lilith, data.MonsterTypeUnique)
}

func (f Javazon) KillUberMephisto() error {
	return f.killBoss(npc.UberMephisto, data.MonsterTypeUnique)
}

func (f Javazon) KillUberDiablo() error {
	return f.killBoss(npc.UberDiablo, data.MonsterTypeUnique)
}

func (f Javazon) KillUberBaal() error {
	return f.killBoss(npc.UberBaal, data.MonsterTypeUnique)
}
