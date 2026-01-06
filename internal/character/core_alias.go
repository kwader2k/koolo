package character

import (
	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/character/core"
)

// BaseCharacter wraps the core CharacterBase to keep legacy code untouched.
type BaseCharacter struct {
	core.CharacterBase
}

// castingTimeout keeps older build code working inside the character package.
const castingTimeout = core.CastCompleteTimeout

// preBattleChecks keeps older build code working inside the character package.
func (bc BaseCharacter) preBattleChecks(id data.UnitID, skipOnImmunities []stat.Resist) bool {
	return bc.CharacterBase.PreBattleChecks(id, skipOnImmunities)
}
