package context

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/game"
)

// Character interfaces live in context to avoid import cycles with implementations.
type Character interface {
	CheckKeyBindings() []skill.ID
	BuffSkills() []skill.ID
	PreCTABuffSkills() []skill.ID
	MainSkillRange() int
	KillCountess() error
	KillAndariel() error
	KillSummoner() error
	KillDuriel() error
	KillCouncil() error
	KillMephisto() error
	KillIzual() error
	KillDiablo() error
	KillPindle() error
	KillNihlathak() error
	KillBaal() error
	KillLilith() error
	KillUberDuriel() error
	KillUberIzual() error
	KillUberMephisto() error
	KillUberDiablo() error
	KillUberBaal() error
	KillMonsterSequence(
		monsterSelector func(d game.Data) (data.UnitID, bool),
		skipOnImmunities []stat.Resist,
	) error
	ShouldIgnoreMonster(m data.Monster) bool
}

type StatAllocation struct {
	Stat   stat.ID
	Points int
}

type LevelingCharacter interface {
	Character
	// StatPoints Stats will be assigned in the order they are returned by this function.
	StatPoints() []StatAllocation
	SkillPoints() []skill.ID
	SkillsToBind() (skill.ID, []skill.ID)
	ShouldResetSkills() bool
	GetAdditionalRunewords() []string
	InitialCharacterConfigSetup()
	AdjustCharacterConfig()
	KillAncients() error
}
