package run

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/koolo/internal/action"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
)

type Utility struct {
	ctx *context.Status
}

func NewUtility() *Utility {
	return &Utility{
		ctx: context.Get(),
	}
}

func (u *Utility) Name() string {
	return string(config.UtilityRun)
}

func (u *Utility) CheckConditions(parameters *RunParameters) SequencerResult {
	return SequencerError
}

func (u *Utility) Run(parameters *RunParameters) error {
	targetAct := u.ctx.CharacterCfg.Game.Utility.ParkingAct

	u.ctx.Logger.Info(fmt.Sprintf("Park it like it's hot in Act %d", targetAct))

	var targetArea = area.ThePandemoniumFortress
	switch targetAct {
	case 1:
		targetArea = area.RogueEncampment
	case 2:
		targetArea = area.LutGholein
	case 3:
		targetArea = area.KurastDocks
	case 4:
		targetArea = area.ThePandemoniumFortress
	case 5:
		targetArea = area.Harrogath
	default:
		return fmt.Errorf("invalid parking act: %d", targetAct)
	}

	err := action.WayPoint(targetArea)
	if err != nil {
		return err
	}

	u.ctx.Logger.Info(fmt.Sprintf("Character parked in Act %d", targetAct))
	return nil
}
