package character

import (
	"fmt"
	"log/slog"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
)

// MuleCharacter is a placeholder character that does nothing.
// It's used for mule profiles to prevent keybinding errors and allow the bot to load.
type MuleCharacter struct {
	BaseCharacter
}

func (s MuleCharacter) ShouldIgnoreMonster(m data.Monster) bool {
	return false
}

// CheckKeyBindings returns an empty list, as mules do not require any specific skills to be bound.
func (m MuleCharacter) CheckKeyBindings() []skill.ID {
	return []skill.ID{}
}

// Buff does nothing, as mules do not need to cast any buffs.
func (m MuleCharacter) Buff() error {
	return nil
}

// BuffSkills returns an empty list, as mules do not have buff skills.
func (m MuleCharacter) BuffSkills() []skill.ID {
	return []skill.ID{}
}

func (m MuleCharacter) PreCTABuffSkills() []skill.ID {
	return []skill.ID{}
}

// KillMonster does nothing. This function will not be called for a mule.
func (m MuleCharacter) KillMonster(monsterName string, monsterType int) error {
	return nil
}

func (m MuleCharacter) KillMonsterSequence(
	monsterSelector func(d game.Data) (data.UnitID, bool),

	skipOnImmunities []stat.Resist,
) error {

	completedAttackLoops := 0
	previousUnitID := 0

	lsOpts := step.Distance(fireballSorceressLSMinDistance, fireballSorceressLSMaxDistance)

	for {
		context.Get().PauseIfNotPriority()

		id, found := monsterSelector(*m.Data)
		if !found {
			return nil

		}
		if previousUnitID != int(id) {
			completedAttackLoops = 0
		}

		if !m.preBattleChecks(id, skipOnImmunities) {
			return nil
		}

		if completedAttackLoops >= fireballSorceressMaxAttacksLoop {
			return nil
		}

		monster, found := m.Data.Monsters.FindByID(id)
		if !found {
			m.Logger.Info("Monster not found", slog.String("monster", fmt.Sprintf("%v", monster)))
			return nil
		}

		if m.Data.PlayerUnit.States.HasState(state.Cooldown) {
			step.PrimaryAttack(id, 2, true, lsOpts)
		}

		step.SecondaryAttack(skill.Meteor, id, 1, lsOpts)

		completedAttackLoops++
		previousUnitID = int(id)
	}
}

// --- Boss Killers (Placeholders) ---

func (m MuleCharacter) KillAndariel() error {
	return nil
}

func (m MuleCharacter) KillBaal() error {
	return nil
}

func (m MuleCharacter) KillCouncil() error {
	return nil
}

func (m MuleCharacter) KillCountess() error {
	return nil
}

func (m MuleCharacter) KillDiablo() error {
	return nil
}

func (m MuleCharacter) KillDuriel() error {
	return nil
}

func (m MuleCharacter) KillIzual() error {
	return nil
}

func (m MuleCharacter) KillMephisto() error {
	return nil
}

func (m MuleCharacter) KillNihlathak() error {
	return nil
}

func (m MuleCharacter) KillPindle() error {
	return nil
}

func (m MuleCharacter) KillSummoner() error {
	return nil
}
