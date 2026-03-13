package bot

import (
	"log/slog"
	"sync"
	"time"
)

// PartyRegistry tracks all party members in the current game session.
// It allows members to signal when they've finished their runs, so the
// leader can wait for everyone before exiting the game.
//
// It also stores the active game info so companions can rejoin after
// a crash/chicken without waiting for a new event from the leader.
//
// Enable/disable is controlled per-character via Companion.WaitForParty config.
type PartyRegistry struct {
	mu       sync.Mutex
	logger   *slog.Logger
	members  map[string]*partyMember // key = supervisor name
	gameID   string                  // current game identifier (prevents cross-game confusion)
	gameInfo *ActiveGameInfo         // current active game connection info
}

type partyMember struct {
	name       string
	done       bool
	registedAt time.Time
}

// ActiveGameInfo holds the connection details for the current party game.
// Companions use this to rejoin after crash/chicken.
type ActiveGameInfo struct {
	GameName   string
	Password   string
	LeaderName string
	CreatedAt  time.Time
}

var (
	partyOnce     sync.Once
	partyInstance *PartyRegistry
)

// GetPartyRegistry returns the global PartyRegistry singleton.
func GetPartyRegistry() *PartyRegistry {
	partyOnce.Do(func() {
		partyInstance = &PartyRegistry{
			members: make(map[string]*partyMember),
		}
	})
	return partyInstance
}

// SetLogger sets the logger (called once during manager init).
func (pr *PartyRegistry) SetLogger(logger *slog.Logger) {
	pr.mu.Lock()
	defer pr.mu.Unlock()
	pr.logger = logger
}

func (pr *PartyRegistry) log() *slog.Logger {
	if pr.logger != nil {
		return pr.logger
	}
	return slog.Default()
}

// SetActiveGame stores the current game info so companions can rejoin.
// Called by the leader when creating a new game.
func (pr *PartyRegistry) SetActiveGame(gameName, password, leaderName string) {
	pr.mu.Lock()
	defer pr.mu.Unlock()
	pr.gameInfo = &ActiveGameInfo{
		GameName:   gameName,
		Password:   password,
		LeaderName: leaderName,
		CreatedAt:  time.Now(),
	}
	pr.log().Info("Party registry: active game set",
		slog.String("game", gameName),
		slog.String("leader", leaderName))
}

// ClearActiveGame removes the active game info. Called when leader exits.
func (pr *PartyRegistry) ClearActiveGame() {
	pr.mu.Lock()
	defer pr.mu.Unlock()
	if pr.gameInfo != nil {
		pr.log().Info("Party registry: active game cleared",
			slog.String("game", pr.gameInfo.GameName))
	}
	pr.gameInfo = nil
}

// GetActiveGame returns the current active game info, or nil if no game is active.
// Companions call this to check if there's a game to rejoin.
func (pr *PartyRegistry) GetActiveGame() *ActiveGameInfo {
	pr.mu.Lock()
	defer pr.mu.Unlock()
	if pr.gameInfo == nil {
		return nil
	}
	// Return a copy to avoid races
	info := *pr.gameInfo
	return &info
}

// RegisterMember adds a supervisor to the party for a given game.
// If the game changed, the registry is reset automatically.
func (pr *PartyRegistry) RegisterMember(name string, gameID string) {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	if pr.gameID != gameID {
		// New game — reset all members
		pr.log().Info("Party registry: new game detected, resetting",
			slog.String("oldGame", pr.gameID),
			slog.String("newGame", gameID))
		pr.members = make(map[string]*partyMember)
		pr.gameID = gameID
	}

	pr.members[name] = &partyMember{
		name:       name,
		done:       false,
		registedAt: time.Now(),
	}
	pr.log().Info("Party registry: member registered",
		slog.String("member", name),
		slog.String("game", gameID),
		slog.Int("totalMembers", len(pr.members)))
}

// UnregisterMember removes a supervisor from the party (crash, stop, etc.).
// This prevents other members from waiting forever for a crashed member.
func (pr *PartyRegistry) UnregisterMember(name string) {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	if _, ok := pr.members[name]; ok {
		delete(pr.members, name)
		pr.log().Info("Party registry: member unregistered",
			slog.String("member", name),
			slog.Int("remainingMembers", len(pr.members)))
	}
}

// MarkDone signals that a member has finished all their runs.
func (pr *PartyRegistry) MarkDone(name string) {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	if m, ok := pr.members[name]; ok {
		m.done = true
		pr.log().Info("Party registry: member marked done",
			slog.String("member", name))
	}
}

// AllDone returns true if all registered members are done (or registry is empty).
func (pr *PartyRegistry) AllDone() bool {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	if len(pr.members) == 0 {
		return true
	}
	for _, m := range pr.members {
		if !m.done {
			return false
		}
	}
	return true
}

// MemberCount returns the current number of registered members.
func (pr *PartyRegistry) MemberCount() int {
	pr.mu.Lock()
	defer pr.mu.Unlock()
	return len(pr.members)
}

// WaitForAll blocks until all members are done or timeout is reached.
// Returns true if all members finished, false if timeout occurred.
func (pr *PartyRegistry) WaitForAll(timeout time.Duration) bool {
	deadline := time.Now().Add(timeout)
	ticker := time.NewTicker(1 * time.Second)
	defer ticker.Stop()

	for {
		if pr.AllDone() {
			return true
		}
		if time.Now().After(deadline) {
			pr.log().Warn("Party registry: wait timeout reached",
				slog.Duration("timeout", timeout),
				slog.Int("members", pr.MemberCount()))
			return false
		}
		<-ticker.C
	}
}

// Reset clears the entire registry (members + game info). Called when leader exits game.
func (pr *PartyRegistry) Reset() {
	pr.mu.Lock()
	defer pr.mu.Unlock()
	pr.members = make(map[string]*partyMember)
	pr.gameID = ""
	pr.gameInfo = nil
	pr.log().Info("Party registry: reset")
}

// Status returns a snapshot of all members and their done state (for logging/debug).
func (pr *PartyRegistry) Status() map[string]bool {
	pr.mu.Lock()
	defer pr.mu.Unlock()
	status := make(map[string]bool, len(pr.members))
	for name, m := range pr.members {
		status[name] = m.done
	}
	return status
}
