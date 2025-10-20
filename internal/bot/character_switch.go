package bot

import (
	"time"

	"github.com/hectorgimenez/koolo/internal/event"
)

// handleCharacterSwitch is a specialized event handler for character switching
func (mng *SupervisorManager) handleCharacterSwitch(evt event.Event) {
	if evt.Message() == "Switching character for muling" {
		currentSupervisor := evt.Supervisor()
		nextCharacter := mng.supervisors[currentSupervisor].GetContext().CurrentGame.SwitchToCharacter

		// Wait for the current supervisor to fully stop
		time.Sleep(5 * time.Second)

		// Start the new character
		if err := mng.Start(nextCharacter, false); err != nil {
			mng.logger.Error("Failed to start next character",
				"from", currentSupervisor,
				"to", nextCharacter,
				"error", err.Error())
		}
	}
}
