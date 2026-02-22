package chicken

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data/state"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/health"
)

const RangeForScaryAura = 25

func CheckForScaryAuraAndCurse() {
	ctx := context.Get()
	cursesCfg := ctx.CharacterCfg.ChickenOnCurses
	aurasCfg := ctx.CharacterCfg.ChickenOnAuras

	if cursesCfg.AmplifyDamage && ctx.Data.PlayerUnit.States.HasState(state.Amplifydamage) {
		panic(fmt.Errorf("%w: Player has amplify damage curse", health.ErrChicken))
	}

	if cursesCfg.Decrepify && ctx.Data.PlayerUnit.States.HasState(state.Decrepify) {
		panic(fmt.Errorf("%w: Player has decrepify curse", health.ErrChicken))
	}

	if cursesCfg.LowerResist && ctx.Data.PlayerUnit.States.HasState(state.Lowerresist) {
		panic(fmt.Errorf("%w: Player has lower resist curse", health.ErrChicken))
	}

	if cursesCfg.BloodMana && ctx.Data.PlayerUnit.States.HasState(state.BloodMana) {
		panic(fmt.Errorf("%w: Player has blood mana curse", health.ErrChicken))
	}

	for _, m := range ctx.Data.Monsters.Enemies() {
		if ctx.PathFinder.DistanceFromMe(m.Position) <= RangeForScaryAura {
			var scaryAura string

			if aurasCfg.Fanaticism && m.States.HasState(state.Fanaticism) {
				scaryAura = "Fanaticism"
			}

			if aurasCfg.Might && m.States.HasState(state.Might) {
				scaryAura = "Might"
			}

			if aurasCfg.Conviction && m.States.HasState(state.Conviction) {
				scaryAura = "Conviction"
			}

			if aurasCfg.HolyFire && m.States.HasState(state.Holyfire) {
				scaryAura = "Holy Fire"
			}

			if aurasCfg.BlessedAim && m.States.HasState(state.Blessedaim) {
				scaryAura = "Blessed Aim"
			}

			if aurasCfg.HolyFreeze && m.States.HasState(state.Holywindcold) {
				scaryAura = "Holy Freeze"
			}

			if aurasCfg.HolyShock && m.States.HasState(state.Holyshock) {
				scaryAura = "Holy Shock"
			}

			if scaryAura != "" {
				message := fmt.Errorf("%w: Mob has %s aura", health.ErrChicken, scaryAura)
				ctx.Logger.Debug(message.Error())
				panic(message)
			}
		}
	}
}
