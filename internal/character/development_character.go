package character

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/game"
)

// DevelopmentCharacter is a passive character implementation used by the Tool run.
// All hooks are no-ops so the bot can attach to the game without performing actions.
// This should be reworked to not be a character at all in the future, but requires a larger refactor.
type DevelopmentCharacter struct {
	BaseCharacter
}

func (DevelopmentCharacter) CheckKeyBindings() []skill.ID { return nil }
func (DevelopmentCharacter) BuffSkills() []skill.ID       { return nil }
func (DevelopmentCharacter) PreCTABuffSkills() []skill.ID { return nil }
func (DevelopmentCharacter) KillCountess() error          { return nil }
func (DevelopmentCharacter) KillAndariel() error          { return nil }
func (DevelopmentCharacter) KillSummoner() error          { return nil }
func (DevelopmentCharacter) KillDuriel() error            { return nil }
func (DevelopmentCharacter) KillMephisto() error          { return nil }
func (DevelopmentCharacter) KillPindle() error            { return nil }
func (DevelopmentCharacter) KillNihlathak() error         { return nil }
func (DevelopmentCharacter) KillCouncil() error           { return nil }
func (DevelopmentCharacter) KillDiablo() error            { return nil }
func (DevelopmentCharacter) KillIzual() error             { return nil }
func (DevelopmentCharacter) KillBaal() error              { return nil }
func (DevelopmentCharacter) KillUberIzual() error         { return nil }
func (DevelopmentCharacter) KillUberDuriel() error        { return nil }
func (DevelopmentCharacter) KillLilith() error            { return nil }
func (DevelopmentCharacter) KillUberMephisto() error      { return nil }
func (DevelopmentCharacter) KillUberDiablo() error        { return nil }
func (DevelopmentCharacter) KillUberBaal() error          { return nil }
func (DevelopmentCharacter) KillMonsterSequence(func(game.Data) (data.UnitID, bool), []stat.Resist) error {
	return nil
}
func (DevelopmentCharacter) ShouldIgnoreMonster(data.Monster) bool { return true }
