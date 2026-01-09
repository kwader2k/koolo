---
name: Leader-Leecher Run
overview: Implement a new "leader_leecher" run that allows bots to follow a human or bot leader through dungeons, waiting for portal signals and maintaining XP range distance. This requires adding chat message reading to d2go and creating the new run logic in koolo.
todos:
  - id: chat-memory
    content: Research and implement chat message reading in d2go (memory pattern)
    status: pending
  - id: config-struct
    content: Add LeaderLeecher config struct and run constant
    status: pending
  - id: follow-actions
    content: Create follow.go with player following utilities
    status: pending
  - id: leecher-run
    content: Implement leader_leecher.go run logic
    status: pending
  - id: leecher-supervisor
    content: Add runLeecherMode to single_supervisor.go
    status: pending
  - id: message-bus
    content: Add LeecherComeMessage to message bus
    status: pending
  - id: ui-template
    content: Add UI configuration template for leader_leecher run
    status: pending
  - id: http-handler
    content: Add form handling in http_server.go
    status: pending
---

# Leader-Leecher Run Implementation

## Overview

Create a new run type where bots follow a leader (human or bot) through portals, maintaining distance for XP sharing. The leecher waits at TP areas for portals, enters on a "come" chat message (or message bus signal for bot leaders), and follows the leader at max XP distance.

## Architecture

```mermaid
flowchart TD
    subgraph game_join [Game Join Phase]
        A[Join game via pattern] --> B[Find leader in roster]
        B --> C[Join leader's party]
    end
    
    subgraph act_sync [Act Synchronization]
        C --> D{Leader in accessible act?}
        D -->|Yes| E[Travel to leader's act town]
        D -->|No| F[Wait in current town]
        F --> D
    end
    
    subgraph portal_wait [Portal Wait Phase]
        E --> G[Go to TP waiting area]
        G --> H{Leader's portal visible?}
        H -->|No| I[Poll for portal]
        I --> H
        H -->|Yes| J{Come message received?}
        J -->|No| K[Wait for message]
        K --> J
        J -->|Yes| L[Enter portal]
    end
    
    subgraph follow_phase [Follow Phase]
        L --> M[Follow leader position]
        M --> N{Within max XP distance?}
        N -->|No| O[Move closer to leader]
        O --> M
        N -->|Yes| P{New portal detected?}
        P -->|Yes| Q[Return to town]
        Q --> G
        P -->|No| R{Leader left game?}
        R -->|Yes| S[Exit game, rejoin next]
        R -->|No| M
    end
```



## Components to Implement

### 1. Chat Message Reading (d2go)

Add memory reading for in-game chat messages to detect the "come" signal.**Files:**

- `d2go-personal/pkg/memory/chat.go` (new) - Chat message memory reading
- `d2go-personal/pkg/memory/offset.go` - Add chat offset
- `d2go-personal/pkg/memory/game_reader.go` - Add GetChatMessages()
- `d2go-personal/pkg/data/data.go` - Add ChatMessages field

### 2. New Run Configuration

Add config options for the leader-leecher run.**File:** [`internal/config/config.go`](internal/config/config.go)

```go
LeaderLeecher struct {
    LeaderName       string   // Character name to follow
    GameNamePattern  string   // Game pattern (e.g., "run-")
    GamePassword     string
    ComeMessage      string   // Chat trigger (e.g., "come", "go", "tp")
    MaxLeaderDistance int     // Max distance from leader for XP (default ~40)
    JoinDelayMin     int
    JoinDelayMax     int
    UseLegacyGraphics bool
} `yaml:"leaderLeecher"`
```



### 3. New Run Type

Create the leader-leecher run.**Files:**

- [`internal/config/runs.go`](internal/config/runs.go) - Add `LeaderLeecherRun` constant
- `internal/run/leader_leecher.go` (new) - Main run logic
- [`internal/run/run.go`](internal/run/run.go) - Register in BuildRun

### 4. Leecher Supervisor Mode

Similar to follower mode, handle leecher behavior in supervisor.**File:** [`internal/bot/single_supervisor.go`](internal/bot/single_supervisor.go)

- Add `runLeecherMode()` function
- Add leecher registry similar to follower registry

### 5. Following Logic

Implement leader position tracking and distance maintenance.**File:** `internal/action/follow.go` (new)

```go
func FollowPlayer(playerName string, maxDistance int) error
func GetPlayerPosition(playerName string) (data.Position, area.ID, bool)
func IsWithinDistance(playerName string, maxDist int) bool
```

Uses existing `data.Roster.FindByName()` which provides player position and area.

### 6. Portal Detection and Entry

Leverage existing portal logic from [`internal/action/tp_actions.go`](internal/action/tp_actions.go):

- `UsePortalFrom(owner string)` - Already exists
- `TPWaitingArea()` - Already defined per act in `internal/town/`

### 7. Message Bus Extensions

Add "come" message type for bot-to-bot signaling.**File:** [`internal/bot/messagebus/messages.go`](internal/bot/messagebus/messages.go)

```go
type LeecherComeMessage struct {
    BaseMessage
    LeaderName string
}
```



### 8. UI Template

Add configuration panel for the run.**File:** [`internal/server/templates/run_settings_components.gohtml`](internal/server/templates/run_settings_components.gohtml)

### 9. HTTP Server Form Handling

**File:** [`internal/server/http_server.go`](internal/server/http_server.go)

## Key Implementation Details

### XP Distance

D2R experience sharing range is approximately 2 screens (~40-50 game units). Default `MaxLeaderDistance` to 35 for safety margin.

### Act Accessibility Check

Check if leecher has waypoints or can walk to leader's act. If not, wait in current town and poll leader's area from roster.

### Chat Message Detection

Will need to find the memory pattern for chat messages in D2R. This is the most uncertain part - may require research/testing.

## Execution Order

1. Implement chat reading in d2go (if pattern can be found)
2. Add config and run constant
3. Implement follow action utilities
4. Create leader_leecher.go run
5. Add leecher mode to supervisor
6. Add message bus message type
7. Add UI template and form handling