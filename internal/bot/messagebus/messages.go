package messagebus

import "time"

// BaseMessage provides common fields for all messages
type BaseMessage struct {
	sender    string
	topic     Topic
	Timestamp time.Time
}

func (m BaseMessage) Sender() string {
	return m.sender
}

func (m BaseMessage) Topic() Topic {
	return m.topic
}

// NewBaseMessage creates a new base message
func NewBaseMessage(sender string, topic Topic) BaseMessage {
	return BaseMessage{
		sender:    sender,
		topic:     topic,
		Timestamp: time.Now(),
	}
}

// LeaderLeftGameMessage is sent when a follow bot detects the leader has left the game
type LeaderLeftGameMessage struct {
	BaseMessage
	LeaderName string
	GameName   string
}

// NewLeaderLeftGameMessage creates a new leader left game message
func NewLeaderLeftGameMessage(sender, leaderName, gameName string) LeaderLeftGameMessage {
	return LeaderLeftGameMessage{
		BaseMessage: NewBaseMessage(sender, TopicLeaderStatus),
		LeaderName:  leaderName,
		GameName:    gameName,
	}
}

// LeaderJoinedGameMessage is sent when a follow bot detects the leader has joined a game
type LeaderJoinedGameMessage struct {
	BaseMessage
	LeaderName string
	GameName   string
}

// NewLeaderJoinedGameMessage creates a new leader joined game message
func NewLeaderJoinedGameMessage(sender, leaderName, gameName string) LeaderJoinedGameMessage {
	return LeaderJoinedGameMessage{
		BaseMessage: NewBaseMessage(sender, TopicLeaderStatus),
		LeaderName:  leaderName,
		GameName:    gameName,
	}
}

// GameAvailableMessage is sent by the scout bot when it successfully joins the leader's new game
type GameAvailableMessage struct {
	BaseMessage
	LeaderName   string
	GameName     string
	GamePassword string
}

// NewGameAvailableMessage creates a new game available message
func NewGameAvailableMessage(sender, leaderName, gameName, gamePassword string) GameAvailableMessage {
	return GameAvailableMessage{
		BaseMessage:  NewBaseMessage(sender, TopicGameCoordination),
		LeaderName:   leaderName,
		GameName:     gameName,
		GamePassword: gamePassword,
	}
}

// GameNotFoundMessage is sent when a scout bot cannot find the leader's game
type GameNotFoundMessage struct {
	BaseMessage
	LeaderName string
	Reason     string
}

// NewGameNotFoundMessage creates a new game not found message
func NewGameNotFoundMessage(sender, leaderName, reason string) GameNotFoundMessage {
	return GameNotFoundMessage{
		BaseMessage: NewBaseMessage(sender, TopicGameCoordination),
		LeaderName:  leaderName,
		Reason:      reason,
	}
}

// BotJoinedGameMessage is sent when a follow bot successfully joins a game
type BotJoinedGameMessage struct {
	BaseMessage
	BotName    string
	GameName   string
	LeaderName string
}

// NewBotJoinedGameMessage creates a new bot joined game message
func NewBotJoinedGameMessage(sender, botName, gameName, leaderName string) BotJoinedGameMessage {
	return BotJoinedGameMessage{
		BaseMessage: NewBaseMessage(sender, TopicBotStatus),
		BotName:     botName,
		GameName:    gameName,
		LeaderName:  leaderName,
	}
}

// BotLeftGameMessage is sent when a follow bot leaves a game
type BotLeftGameMessage struct {
	BaseMessage
	BotName  string
	GameName string
	Reason   string
}

// NewBotLeftGameMessage creates a new bot left game message
func NewBotLeftGameMessage(sender, botName, gameName, reason string) BotLeftGameMessage {
	return BotLeftGameMessage{
		BaseMessage: NewBaseMessage(sender, TopicBotStatus),
		BotName:     botName,
		GameName:    gameName,
		Reason:      reason,
	}
}

// RequestScoutMessage is sent to determine which bot should be the scout
type RequestScoutMessage struct {
	BaseMessage
	LeaderName string
	Priority   int // Lower priority = more likely to be scout
}

// NewRequestScoutMessage creates a new request scout message
func NewRequestScoutMessage(sender, leaderName string, priority int) RequestScoutMessage {
	return RequestScoutMessage{
		BaseMessage: NewBaseMessage(sender, TopicGameCoordination),
		LeaderName:  leaderName,
		Priority:    priority,
	}
}

// ScoutClaimedMessage is sent when a bot claims the scout role
type ScoutClaimedMessage struct {
	BaseMessage
	ScoutName  string
	LeaderName string
}

// NewScoutClaimedMessage creates a new scout claimed message
func NewScoutClaimedMessage(sender, scoutName, leaderName string) ScoutClaimedMessage {
	return ScoutClaimedMessage{
		BaseMessage: NewBaseMessage(sender, TopicGameCoordination),
		ScoutName:   scoutName,
		LeaderName:  leaderName,
	}
}

// FollowerStartMessage is sent by a leader to instruct a follower to start following
type FollowerStartMessage struct {
	BaseMessage
	FollowerName string // Target follower supervisor name
	LeaderName   string // Name of the leader character
	GamePattern  string // Game name pattern (e.g., "run-")
	GamePassword string // Password for games
	JoinDelayMin int    // Minimum join delay in ms
	JoinDelayMax int    // Maximum join delay in ms
}

// NewFollowerStartMessage creates a new follower start message
func NewFollowerStartMessage(sender, followerName, leaderName, gamePattern, gamePassword string, joinDelayMin, joinDelayMax int) FollowerStartMessage {
	return FollowerStartMessage{
		BaseMessage:  NewBaseMessage(sender, TopicGameCoordination),
		FollowerName: followerName,
		LeaderName:   leaderName,
		GamePattern:  gamePattern,
		GamePassword: gamePassword,
		JoinDelayMin: joinDelayMin,
		JoinDelayMax: joinDelayMax,
	}
}

// FollowerStopMessage is sent by a leader to instruct a follower to stop following
type FollowerStopMessage struct {
	BaseMessage
	FollowerName string
	LeaderName   string
}

// NewFollowerStopMessage creates a new follower stop message
func NewFollowerStopMessage(sender, followerName, leaderName string) FollowerStopMessage {
	return FollowerStopMessage{
		BaseMessage:  NewBaseMessage(sender, TopicGameCoordination),
		FollowerName: followerName,
		LeaderName:   leaderName,
	}
}

