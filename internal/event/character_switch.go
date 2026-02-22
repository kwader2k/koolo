package event

// CharacterSwitchEvent represents a request to switch to a different character
type CharacterSwitchEvent struct {
	BaseEvent
	CurrentCharacter string
	NextCharacter    string
}

func CharacterSwitch(be BaseEvent, currentCharacter string, nextCharacter string) CharacterSwitchEvent {
	return CharacterSwitchEvent{
		BaseEvent:        be,
		CurrentCharacter: currentCharacter,
		NextCharacter:    nextCharacter,
	}
}
