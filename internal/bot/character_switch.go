package bot

import (
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/utils"
)

// handleCharacterSwitch is a specialized event handler for character switching
func (mng *SupervisorManager) handleCharacterSwitch(evt event.Event) {
	if evt.Message() == "Switching character for muling" {
		currentSupervisor := evt.Supervisor()
		nextCharacter := mng.supervisors[currentSupervisor].GetContext().CurrentGame.SwitchToCharacter

		// Wait for the current supervisor to fully stop
		utils.Sleep(5000, 2000) // Wait 5 seconds with some jitter

		// Start the new character
		if err := mng.Start(nextCharacter, false, false); err != nil {
			mng.logger.Error("Failed to start next character",
				"from", currentSupervisor,
				"to", nextCharacter,
				"error", err.Error())
		}
	}
}
