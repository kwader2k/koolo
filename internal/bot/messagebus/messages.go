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

// LeecherCommandType represents the type of leecher command
type LeecherCommandType string

const (
	LeecherCmdCome LeecherCommandType = "come" // Signal leechers to enter portal
	LeecherCmdStay LeecherCommandType = "stay" // Signal leechers to wait
	LeecherCmdTP   LeecherCommandType = "tp"   // Signal leechers to use town portal
	LeecherCmdExit LeecherCommandType = "exit" // Signal leechers to exit game
)

// LeecherCommandMessage is sent to control leecher behavior
type LeecherCommandMessage struct {
	BaseMessage
	Command    LeecherCommandType
	LeaderName string
}

// NewLeecherCommandMessage creates a new leecher command message
func NewLeecherCommandMessage(sender string, command LeecherCommandType, leaderName string) LeecherCommandMessage {
	return LeecherCommandMessage{
		BaseMessage: NewBaseMessage(sender, TopicLeecherCommand),
		Command:     command,
		LeaderName:  leaderName,
	}
}

// LeecherComeMessage is sent to signal leechers to enter the portal
type LeecherComeMessage struct {
	BaseMessage
	LeaderName string
}

// NewLeecherComeMessage creates a new leecher come message
func NewLeecherComeMessage(sender, leaderName string) LeecherComeMessage {
	return LeecherComeMessage{
		BaseMessage: NewBaseMessage(sender, TopicLeecherCommand),
		LeaderName:  leaderName,
	}
}

// LeecherStayMessage is sent to signal leechers to wait
type LeecherStayMessage struct {
	BaseMessage
	LeaderName string
}

// NewLeecherStayMessage creates a new leecher stay message
func NewLeecherStayMessage(sender, leaderName string) LeecherStayMessage {
	return LeecherStayMessage{
		BaseMessage: NewBaseMessage(sender, TopicLeecherCommand),
		LeaderName:  leaderName,
	}
}

// LeecherTPMessage is sent to signal leechers to use town portal
type LeecherTPMessage struct {
	BaseMessage
	LeaderName string
}

// NewLeecherTPMessage creates a new leecher TP message
func NewLeecherTPMessage(sender, leaderName string) LeecherTPMessage {
	return LeecherTPMessage{
		BaseMessage: NewBaseMessage(sender, TopicLeecherCommand),
		LeaderName:  leaderName,
	}
}

// LeecherExitMessage is sent to signal leechers to exit the game
type LeecherExitMessage struct {
	BaseMessage
	LeaderName string
}

// NewLeecherExitMessage creates a new leecher exit message
func NewLeecherExitMessage(sender, leaderName string) LeecherExitMessage {
	return LeecherExitMessage{
		BaseMessage: NewBaseMessage(sender, TopicLeecherCommand),
		LeaderName:  leaderName,
	}
}

// LeecherStartMessage is sent by a leader to instruct a leecher to start
type LeecherStartMessage struct {
	BaseMessage
	LeecherName       string // Target leecher supervisor name
	LeaderName        string // Name of the leader character
	GamePattern       string // Game name pattern
	GamePassword      string // Password for games
	JoinDelayMin      int    // Minimum join delay in ms
	JoinDelayMax      int    // Maximum join delay in ms
	PollInterval      int    // How often to poll (ms)
	PortalEntryDelay  int    // Seconds to wait after "come"
	MaxLeaderDistance int    // Max distance for XP
	UseLegacyGraphics bool   // Use legacy graphics
}

// NewLeecherStartMessage creates a new leecher start message
func NewLeecherStartMessage(sender, leecherName, leaderName, gamePattern, gamePassword string,
	joinDelayMin, joinDelayMax, pollInterval, portalEntryDelay, maxLeaderDistance int, useLegacyGraphics bool) LeecherStartMessage {
	return LeecherStartMessage{
		BaseMessage:       NewBaseMessage(sender, TopicGameCoordination),
		LeecherName:       leecherName,
		LeaderName:        leaderName,
		GamePattern:       gamePattern,
		GamePassword:      gamePassword,
		JoinDelayMin:      joinDelayMin,
		JoinDelayMax:      joinDelayMax,
		PollInterval:      pollInterval,
		PortalEntryDelay:  portalEntryDelay,
		MaxLeaderDistance: maxLeaderDistance,
		UseLegacyGraphics: useLegacyGraphics,
	}
}

// LeecherStopMessage is sent by a leader to instruct a leecher to stop
type LeecherStopMessage struct {
	BaseMessage
	LeecherName string
	LeaderName  string
}

// NewLeecherStopMessage creates a new leecher stop message
func NewLeecherStopMessage(sender, leecherName, leaderName string) LeecherStopMessage {
	return LeecherStopMessage{
		BaseMessage: NewBaseMessage(sender, TopicGameCoordination),
		LeecherName: leecherName,
		LeaderName:  leaderName,
	}
}

