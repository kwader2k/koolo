# agents.md

## Role

You are an expert Go maintainer operating as an improvement agent.

Your responsibility is to **analyze, review, and safely improve** this repository while preserving correctness, performance, and backward compatibility.

You are conservative by default and biased toward **clarity, safety, and operational correctness** over stylistic or architectural rewrites.

---

## Scope of Work

- Behavior changes are allowed only when explicitly intended, clearly documented, and justified by correctness or safety.
- Do **not** introduce new external dependencies (unless requested/approved).
- Do **not** add new tests beyond those already present (unless requested).
- When in doubt, prefer not making a change and document the concern instead.
- All changes must:
  - compile
  - be `gofmt`-formatted
  - respect existing naming, style, linting, and logging conventions

---

## Primary Objectives (Priority Order)

### 1. Correctness & Behavioral Safety

You must fully understand the behavioral impact of the code changes.

Check for:
- Behavior changes (explicit or accidental)
- Broken invariants or assumptions
- Nil / zero-value handling
- Boundary conditions and edge cases
- Error paths and error propagation
- Context cancellation, timeouts, retries
- Concurrency safety (data races, goroutine leaks, deadlocks)
- Resource lifecycle issues (files, connections, timers)
- Backward compatibility of exported APIs and observable behavior

If a change could affect callers, it **must be explicitly identified and documented**.

---

### 2. Targeted Code Improvements (Safe Only)

Within the **touched code only**, you may apply small, justified improvements:

#### Clarity & Maintainability
- Fix spelling and typos in:
  - identifiers
  - comments
  - log messages
  - docstrings
  - error strings (only if safe)
- Simplify control flow:
  - prefer early returns
  - reduce nesting
- Refactor only when it clearly improves readability or correctness

#### Helper Function Policy
- Extract helpers **only** if logic is reused in multiple places.
- Keep helpers in the same file when private and local.
- Do **not** create tiny one-use helpers.
- Inline helpers introduced in the same change if they add indirection without clear value.

#### Performance
- Avoid unnecessary allocations and repeated work.
- Avoid premature micro-optimizations.
- Optimize only obvious hot paths or correctness-related inefficiencies.

---

### 3. Logging Quality

Logging must be intentional and operationally useful.

- Add logs only for:
  - early returns
  - non-obvious branches
  - retries, fallbacks, degraded paths
  - state or flow changes
- Avoid logs in tight loops or per-item processing unless debug-guarded and justified.
- Include sufficient context (IDs, counts, durations, key parameters).
- Never log secrets, credentials, or PII.
- Match the repository’s existing logging style (structured vs unstructured).

---

### 4. Commit Message

- Use **Conventional Commits**:
  - `type(scope): imperative summary`
  - Optional body: explain **what and why**, not how
  - Optional footer for breaking changes or references

---

## Guiding Principles

- Be conservative.
- Prefer small, safe improvements.
- Never trade correctness for cleverness.
- Document behavior changes clearly.
- Leave the codebase better than you found it — but only where you touched it.

## Project Overview

**Koolo** is a Windows-only Diablo 2 bot written in Go 1.23. It drives the game by reading process memory and sending HID (keyboard/mouse) input. The module path is `github.com/hectorgimenez/koolo`.

### Technology Stack

- **Go 1.23**, `gofmt`-formatted.
- **Windows-only**: uses `github.com/lxn/win`, `golang.org/x/sys/windows`, Win32 APIs. Do not introduce cross-platform abstractions.
- **Game data types**: all Diablo 2 entities (areas, monsters, NPCs, skills, items, stats, quests, objects) come from `github.com/hectorgimenez/d2go/pkg/data/...`. The `go.mod` `replace` directive currently points this to a private fork (`github.com/crazywh1t3/d2go`). Never inline or duplicate types from this package.
- **Configuration**: YAML files under `config/`, parsed by `gopkg.in/yaml.v3`. Global config is `config.Koolo`; per-character config is `ctx.CharacterCfg`.
- **Logging**: `log/slog` with structured key-value pairs via `ctx.Logger`. Never use `fmt.Print*` or `log.*` (standard library) for runtime logging.

### Key External Packages

| Import path | Purpose |
|---|---|
| `github.com/hectorgimenez/d2go/pkg/data` | Core game data structures (`Item`, `Monster`, `Position`, …) |
| `github.com/hectorgimenez/d2go/pkg/data/area` | Area IDs |
| `github.com/hectorgimenez/d2go/pkg/data/npc` | NPC IDs |
| `github.com/hectorgimenez/d2go/pkg/data/skill` | Skill IDs |
| `github.com/hectorgimenez/d2go/pkg/data/stat` | Stat/resist IDs |
| `github.com/hectorgimenez/d2go/pkg/data/item` | Item names, locations, body slots |
| `github.com/hectorgimenez/d2go/pkg/data/object` | Object IDs (shrines, portals, chests) |
| `github.com/hectorgimenez/d2go/pkg/data/quest` | Quest IDs |
| `github.com/hectorgimenez/d2go/pkg/nip` | NIP rule evaluation (pickit) |

---

## Package Structure

```
internal/
  action/         High-level bot actions (move, pickup, fight, stash, buff, …)
    step/         Low-level atomic primitives (attack, cast, interact, move step)
  bot/            Supervisor, manager, scheduler, bot loop, stuck detection
  buildnoise/     Audio noise generation utilities
  character/      Per-class character implementations
    core/         Shared character helpers (BaseCharacter, preBattleChecks)
    paladin/      Paladin variants (Hammerdin, FoH, Dragon, leveling)
  chicken/        Flee-on-low-health logic (scary auras/curses)
  config/         Config loading, CharacterCfg, KooloCfg, run/build constants
  context/        Goroutine-local execution context; the central data carrier
  drop/           Per-supervisor drop tracking, filtering, and coordination
  event/          Event broadcast system (Discord, Telegram, internal listeners)
  game/           Memory reader/injector, HID (keyboard/mouse), game manager, crash detector
  health/         Belt manager, health manager, ping monitor
  mule/           Mule (item transfer) orchestration
  packet/         Packet-based game interaction (alternative to HID)
  pather/         A* pathfinding, map rendering, collision grid
    astar/        A* algorithm implementation
  pickit/         NIP rule compilation, item evaluation, pickit editor API
  remote/         Remote control integrations
    discord/      Discord webhook/bot notifications
    droplog/      Drop log file writer
    ngrok/        Ngrok tunnel for remote access
    telegram/     Telegram bot notifications
  run/            Run implementations (one file per boss/area/quest)
  server/         HTTP server for the web overlay, WebSocket status broadcast
  terrorzone/     Terror zone detection, tier scoring, and selection
  town/           Act-specific town routines (A1.go – A5.go)
  ui/             UI interaction helpers (menus, inventory coordinates)
  updater/        Auto-update, cherry-pick, rollback logic
  utils/          Sleep helpers, math, randomness, Windows utilities
    winproc/      Windows process utilities
```

---

## Core Patterns

### The Context Pattern

Every goroutine that executes bot logic retrieves its context with:

```go
ctx := context.Get()
```

`context.Get()` is keyed by **goroutine ID** (via `runtime.Stack`). Each action function must call `ctx.SetLastAction("FunctionName")` at entry for stuck-detection and debug tracing. Steps call `ctx.SetLastStep("StepName")` analogously.

The `context.Status` struct wraps `*Context` with a `Priority` field:

```go
type Status struct {
    *Context
    Priority Priority
}
```

The `context.Context` struct is the single source of truth for:
- `ctx.Data` — last-read game state (`*game.Data`); refresh with `ctx.RefreshGameData()`
- `ctx.CharacterCfg` — per-character YAML config (`*config.CharacterCfg`)
- `ctx.Logger` — structured `*slog.Logger`
- `ctx.HID` — keyboard/mouse input driver (`*game.HID`)
- `ctx.GameReader` — memory reader (`*game.MemoryReader`)
- `ctx.MemoryInjector` — memory injector for hooking USER32.dll (`*game.MemoryInjector`)
- `ctx.PathFinder` — A* pathfinder (`*pather.PathFinder`)
- `ctx.BeltManager` — potion belt manager (`*health.BeltManager`)
- `ctx.HealthManager` — health/mana/chicken manager (`*health.Manager`)
- `ctx.Char` — the active `Character` or `LevelingCharacter` implementation
- `ctx.CurrentGame` — per-game transient state (`*CurrentGameHelper`)
- `ctx.PacketSender` — packet-based game interaction (`*game.PacketSender`)
- `ctx.Drop` — per-supervisor drop manager (`*drop.Manager`)
- `ctx.EventListener` — event broadcast system (`*event.Listener`)
- `ctx.Manager` — game lifecycle manager (`*game.Manager`)

### Context Lifecycle

```go
// Creation (in buildSupervisor):
ctx := context.NewContext(supervisorName)  // attaches to current goroutine

// In spawned goroutines:
ctx.AttachRoutine(priority)  // registers goroutine with given priority
defer ctx.Detach()           // removes goroutine from context map

// Retrieval (anywhere in bot logic):
ctx := context.Get()         // returns *Status for current goroutine
```

**Never pass `*context.Context` through call arguments where `context.Get()` is available.** Use `context.Status` (the goroutine-local wrapper) when you need priority metadata.

### Priority System

```go
const (
    PriorityHigh       = 0   // chicken, health emergencies, item pickup
    PriorityNormal     = 1   // standard bot operation (run execution)
    PriorityBackground = 5   // background health monitoring, data refresh
    PriorityPause      = 10  // game paused
    PriorityStop       = 100 // supervisor stopping (triggers panic inside loops)
)
```

**Every loop** inside a run or character kill sequence **must** call `ctx.PauseIfNotPriority()` at the top of each iteration. This is the pause/stop mechanism. Omitting it causes the bot to ignore pause/stop commands.

`PauseIfNotPriority()` blocks when the goroutine's priority does not match `ExecutionPriority`. When `ExecutionPriority == PriorityStop`, it panics with `"Bot is stopped"` to unwind the call stack.

### Game Data Refresh

`ctx.Data` is a snapshot; it goes stale as the game advances. Call `ctx.RefreshGameData()` before reading positional or state-sensitive data. In tight loops, refresh at most once per iteration.

```go
func (ctx *Context) RefreshGameData()   // full refresh: all game state
func (ctx *Context) RefreshInventory()  // inventory-only refresh (lighter)
```

### Sleep Utilities

| Function | When to use |
|---|---|
| `utils.Sleep(ms)` | Simple fixed wait with ±30% jitter (randomizes between 0.7× and 1.3×) |
| `utils.PingSleep(utils.Light, ms)` | Polling / lightweight checks — adds 1× ping to base delay |
| `utils.PingSleep(utils.Medium, ms)` | UI interactions, clicks, movement waits — adds 2× ping to base delay |
| `utils.PingSleep(utils.Critical, ms)` | State transitions, portal enters, game joins — adds 4× ping to base delay |
| `utils.RetryDelay(attempt, basePing, minMs)` | Escalating retry delays — `minMs + basePing × ping × attempt` |

All ping-adaptive functions cap at 5000ms. Never use `time.Sleep` directly in bot logic — it ignores network latency. Use the utilities above.

The ping getter is initialized in `NewContext()` via `utils.SetPingGetter()` to read `ctx.Data.Game.Ping`, defaulting to 50ms when unavailable.

### Action vs. Step

- **`internal/action/`** — composite actions that may call multiple steps, refresh game data, loop, and handle errors. Named functions like `action.MoveToArea(...)`, `action.ItemPickup(...)`. These are **top-level functions** (not methods) that call `context.Get()` internally.
- **`internal/action/step/`** — single atomic operations that interact with the game once or in a tight loop. `step.Attack(...)`, `step.InteractNPC(...)`, `step.MoveTo(...)`. Steps do not call other steps.

#### Step Files (17 files):

| File | Purpose |
|---|---|
| `attack.go` | `PrimaryAttack()`, `SecondaryAttack()` with functional options |
| `cast.go` | Skill casting |
| `close_all_menus.go` | Close all open game menus |
| `interact_entrance.go` | Area entrance interaction |
| `interact_entrance_packet.go` | Packet-based entrance interaction |
| `interact_npc.go` | NPC interaction |
| `interact_object.go` | Object interaction (shrines, chests, etc.) |
| `interact_object_packet.go` | Packet-based object interaction |
| `interrupt.go` | Drop interrupt handling |
| `move.go` | `MoveTo()` with pathfinding, stuck detection, area transitions |
| `open_inventory.go` | Open inventory screen |
| `open_portal.go` | Open Town Portal |
| `pickup_item.go` | Pick up a single item |
| `pickup_item_packet.go` | Packet-based item pickup |
| `set_skill.go` | Set active skill |
| `skill_selection.go` | Skill selection logic |
| `swap_weapon.go` | CTA weapon swap |

#### Action Files (48 files):

Key action files include: `item_pickup.go` (complex multi-attempt pickup with blacklisting), `stash.go` (gold/inventory stashing with tab management), `autoequip.go` (automated equipment evaluation with tier scoring), `runeword_maker.go` (runeword crafting with reroll support), `cube_recipes.go` (Horadric Cube recipe automation), `gambling.go` (gold gambling at NPCs), `shopping_action.go` (vendor shopping with NIP rules), `move.go` (high-level movement), `clear_area.go`/`clear_level.go` (area clearing), `buff.go` (skill buffing), `town.go` (town routines), `tp_actions.go` (Town Portal management).

---

## Bot Main Loop (`internal/bot/bot.go`)

The `Bot.Run()` method orchestrates four concurrent goroutines via `errgroup`:

1. **Background goroutine** (`PriorityBackground`): Calls `RefreshGameData()` every 100ms, tracks position changes for idle detection.

2. **Health goroutine** (`PriorityBackground`): Calls `HandleHealthAndMana()` every 100ms. Also checks global idle (2min no movement → quit game) and max game length enforcement. Skipped during Drop runs.

3. **High-priority goroutine** (`PriorityHigh`): Handles item pickup, rebuffing, belt refill, return-to-town decisions (low gold, low potions, broken equipment, merc died), area corrections, legacy mode, and portrait/chat cleanup. Uses check-then-lock pattern.

4. **Normal-priority goroutine** (`PriorityNormal`): Iterates through configured runs, calls `PreRun`/`Run`/`PostRun`. Tracks analytics (XP per run). Handles Drop interrupts (`drop.ErrInterrupt`). Recovers from chicken panics.

### Bot Struct

```go
type Bot struct {
    ctx                   *botCtx.Context
    lastActivityTime      time.Time
    lastActivityTimeMux   sync.RWMutex
    lastKnownPosition     data.Position
    lastPositionCheckTime time.Time
    analyticsManager      *AnalyticsManager
    MuleManager           interface {
        ShouldMule(stashFull bool, characterName string) (bool, string)
    }
}
```

---

## Supervisor System (`internal/bot/supervisor.go`, `manager.go`)

### Supervisor Interface

```go
type Supervisor interface {
    Start() error
    Name() string
    Stop()
    Stats() Stats
    TogglePause()
    SetWindowPosition(x, y int)
    GetData() *game.Data
    GetContext() *ct.Context
    GetAnalyticsManager() *AnalyticsManager
}
```

The `baseSupervisor` struct provides the common implementation. `SinglePlayerSupervisor` (not shown) extends it.

### SupervisorManager

```go
type SupervisorManager struct {
    logger         *slog.Logger
    supervisors    map[string]Supervisor
    crashDetectors map[string]*game.CrashDetector
    eventListener  *event.Listener
    Drop           *drop.Service
}
```

Key methods:
- `Start(supervisorName, attachToExisting, manualMode, pidHwnd)` — reloads config, creates supervisor via `buildSupervisor()`, starts crash detector
- `Stop(supervisorName)` / `StopAll()` — stops supervisors
- `ReloadConfig()` — clears NIP cache and reloads configs; applies to running supervisors
- `TogglePause(supervisorName)` — pause/resume via `MemoryInjector`

### Stats & Status Tracking

```go
type SupervisorStatus string
const (
    NotStarted SupervisorStatus = "Not Started"
    Starting   SupervisorStatus = "Starting"
    InGame     SupervisorStatus = "In game"
    Paused     SupervisorStatus = "Paused"
    Crashed    SupervisorStatus = "Crashed"
)

type Stats struct {
    StartedAt           time.Time
    SupervisorStatus    SupervisorStatus
    Details             string
    Drops               []data.Drop
    Games               []GameStats
    IsCompanionFollower bool
    UI                  CharacterOverview
    MuleEnabled         bool
    ManualModeActive    bool
}

type GameStats struct {
    StartedAt  time.Time
    FinishedAt time.Time
    Reason     event.FinishReason
    Runs       []RunStats
}

type RunStats struct {
    Name        string
    Reason      event.FinishReason
    StartedAt   time.Time
    Items       []data.Item
    FinishedAt  time.Time
    UsedPotions []event.UsedPotionEvent
}

type CharacterOverview struct {
    Class, Difficulty, Area string
    Level, Ping, Life, MaxLife, Mana, MaxMana int
    Experience, LastExp, NextExp uint64
    MagicFind, GoldFind int
    FireResist, ColdResist, LightningResist, PoisonResist int
    Gold int
}
```

---

## Configuration System (`internal/config/config.go`)

### Global Config

```go
var Koolo *KooloCfg
var Characters map[string]*CharacterCfg
var Version = "dev"
```

#### KooloCfg

```go
type KooloCfg struct {
    Debug struct {
        Log, Screenshots, RenderMap bool
    }
    FirstRun                bool
    UseCustomSettings       bool
    GameWindowArrangement   bool
    D2LoDPath, D2RPath      string
    CentralizedPickitPath   string
    WindowWidth, WindowHeight int
    Discord   struct { Enabled bool; ChannelID, Token, WebhookURL string; ... }
    Telegram  struct { Enabled bool; Token, ChatID string; ... }
    Ngrok     struct { Enabled bool; AuthToken string }
    PingMonitor struct { ... }
    AutoStart   struct { Enabled bool; SupervisorNames []string }
    Analytics   struct { Enabled bool; HistoryDays int; MaxNotableDrops int; TrackAllItems bool }
    RunewordFavoriteRecipes []string
    RunFavoriteRuns         []string
    LogSaveDirectory        string
}
```

#### CharacterCfg (per-supervisor)

```go
type CharacterCfg struct {
    MaxGameLength       int
    Username, Password  string
    AuthMethod          string   // "TokenAuth" or empty
    AuthToken           string
    Realm               string
    CharacterName       string
    AutoCreateCharacter bool
    KillD2OnStop        bool
    ClassicMode         bool
    UseCentralizedPickit bool
    PacketCasting       struct { Entrance, Item, TP, Teleport, Entity, Skill bool }
    Scheduler           Scheduler
    Health              struct {
        HealingPotionAt, ManaPotionAt, RejuvPotionAtLife, RejuvPotionAtMana float64
        MercHealingPotionAt, MercRejuvPotionAt float64
        ChickenAt, MercChickenAt float64
    }
    ChickenOnCurses []string   // e.g. "amplify_damage", "decrepify"
    ChickenOnAuras  []string   // e.g. "fanaticism", "conviction"
    Inventory       struct {
        InventoryLock [4][10]int   // 0 = free, 1 = locked
        BeltColumns   BeltColumns
    }
    Character struct {
        Class          string    // e.g. "hammerdin", "sorceress", "trapsin"
        UseMerc        bool
        StashToShared  int       // 0 = personal, 1-3 = shared stash tab
        UseTeleport    bool
        ClearPathDist  int
        AutoStatSkill  AutoStatSkillConfig
        // Per-class sub-configs: BerserkerBarb, WhirlwindBarb, BlizzardSorceress, etc.
    }
    Game struct {
        MinGoldPickup         int
        UseCainIdentify       bool
        InteractWithShrines   bool
        InteractWithChests    bool
        Difficulty            string  // "normal", "nightmare", "hell"
        Runs                  []string
        // Per-run configs:
        Pindleskin  struct { SkipOnImmunities []string }
        Pit         struct { FocusBosses bool }
        Diablo      struct { ... }
        Baal        struct { ... }
        TerrorZone  struct { ... }
        Leveling    struct { ... }
        RunewordMaker struct { Enabled bool; ... }
    }
    Companion struct { Enabled bool; Leader bool; GameNameTemplate string }
    Gambling  struct { Enabled bool; Items []string }
    Muling    struct { Enabled bool; SwitchToMule string; ReturnTo string; MuleProfiles []string }
    CubeRecipes struct { Enabled bool; EnabledRecipes []string; SkipPerfects bool }
    BackToTown  struct { ... }
    Shopping    ShoppingConfig
    Runtime     struct { Rules nip.Rules; TierRules; Drops }
}
```

### Constants

```go
const GameVersionReignOfTheWarlock = "reign_of_the_warlock"
const GameVersionExpansion = "expansion"
```

### Key Config Functions

```go
func Load() error
func GetCharacter(name string) (*CharacterCfg, error)
func GetCharacters() map[string]*CharacterCfg
func CreateFromTemplate(name string) error
func ValidateAndSaveConfig(cfg *CharacterCfg) error
func SaveSupervisorConfig(name string, cfg *CharacterCfg) error
func SaveKooloConfig(cfg *KooloCfg) error
func ClearNIPCache()
```

### Run Constants (`internal/config/runs.go`)

70+ run constants of type `Run string`:

```go
type Run string
const (
    CountessRun       Run = "countess"
    AndarielRun       Run = "andariel"
    SummonerRun       Run = "summoner"
    DurielRun         Run = "duriel"
    MephistoRun       Run = "mephisto"
    DiabloRun         Run = "diablo"
    BaalRun           Run = "baal"
    PindleskinRun     Run = "pindleskin"
    NihlathakRun      Run = "nihlathak"
    TravincalRun      Run = "travincal"
    PitRun            Run = "pit"
    AncientTunnelsRun Run = "ancient_tunnels"
    CowsRun           Run = "cows"
    TerrorZoneRun     Run = "terror_zone"
    LevelingRun       Run = "leveling"
    LevelingSequenceRun Run = "leveling_sequence"
    QuestsRun         Run = "quests"
    ShoppingRun       Run = "shopping"
    MuleRun           Run = "mule"
    // ... 50+ more
)
```

Supporting types:
- `AvailableRuns map[Run]string` — display names
- `SequencerQuests []LevelingRunInfo` — Act 1–5 quest sequence with mandatory flags
- `SequencerRuns []Run` — ordered runs for the leveling sequencer
- `LevelingRunInfo struct { Run, Act int, IsMandatory bool }`

### Scheduler Config

```go
type Scheduler struct {
    Enabled          bool
    Mode             string  // "timeSlots" or "duration"
    Days             map[Day][]TimeRange
    DurationSchedule DurationSchedule
}

type DurationSchedule struct {
    WakeUpTime   string     // "08:00"
    PlayHours    float64    // e.g. 8.5
    MealBreak    bool
    ShortBreaks  bool
}
```

---

## Run System (`internal/run/`)

### Interfaces

```go
type Run interface {
    Name() string
    Run(parameters *RunParameters) error
    CheckConditions(parameters *RunParameters) SequencerResult
}

type TownRoutineSkipper interface {
    SkipTownRoutines() bool
}

type SequencerResult int
const (
    SequencerSkip  SequencerResult = iota  // skip this run, continue
    SequencerStop                          // stop the sequencer
    SequencerOk                            // run is ready
    SequencerError                         // error, may retry
)
```

### Adding a New Run

1. Create `internal/run/<runname>.go` implementing the `Run` interface:
   ```go
   type MyRun struct { ctx *context.Status }
   func NewMyRun() *MyRun { return &MyRun{ctx: context.Get()} }
   func (r MyRun) Name() string { return string(config.MyRunRun) }
   func (r MyRun) CheckConditions(p *RunParameters) SequencerResult { ... }
   func (r MyRun) Run(p *RunParameters) error { ... }
   ```
2. Add the constant to `internal/config/runs.go`:
   ```go
   MyRunRun Run = "my_run"
   ```
3. Add a `case` to `BuildRun()` in `internal/run/run.go`.
4. If the run requires specific conditions (quests, difficulty), check them in `CheckConditions` and return `SequencerSkip` or `SequencerError` as appropriate.

### Run Interface Contracts

- `CheckConditions` must be side-effect free and cheap (no game interaction).
- `Run` must return a non-nil error only when recovery is possible; if the run is structurally impossible, return a descriptive error.
- Implement `TownRoutineSkipper` (return `SkipTownRoutines() bool`) only if the run explicitly manages its own pre/post town logic.

### Run Directory (78 files)

One file per run. Includes boss runs (andariel, mephisto, diablo, baal), area runs (pit, ancient_tunnels, cows), quest runs (den, rescue_cain, staff, duriel, izual, ancients, hellforge), uber runs (uber_organs, uber_torch, uber_lilith, uber_duriel, uber_izual), leveling (leveling.go, leveling_act1–5.go, leveling_sequence.go), utility (mule, shopping, development, utility, threshsocket, frozen_aura_merc, tristram_early_goldfarm), plus helpers.go and uber_helper.go.

---

## Character System (`internal/character/`, `internal/context/character.go`)

### Character Interface

```go
type Character interface {
    CheckKeyBindings() []skill.ID
    BuffSkills() []skill.ID
    PreCTABuffSkills() []skill.ID
    MainSkillRange() int
    KillCountess() error
    KillAndariel() error
    KillSummoner() error
    KillDuriel() error
    KillCouncil() error
    KillMephisto() error
    KillIzual() error
    KillDiablo() error
    KillPindle() error
    KillNihlathak() error
    KillBaal() error
    KillLilith() error
    KillUberDuriel() error
    KillUberIzual() error
    KillUberMephisto() error
    KillUberDiablo() error
    KillUberBaal() error
    KillMonsterSequence(
        monsterSelector func(d game.Data) (data.UnitID, bool),
        skipOnImmunities []stat.Resist,
    ) error
    ShouldIgnoreMonster(m data.Monster) bool
}
```

### LevelingCharacter Interface

```go
type LevelingCharacter interface {
    Character
    StatPoints() []StatAllocation
    SkillPoints() []skill.ID
    SkillsToBind() (skill.ID, []skill.ID)
    ShouldResetSkills() bool
    GetAdditionalRunewords() []string
    InitialCharacterConfigSetup()
    AdjustCharacterConfig()
    KillAncients() error
}

type StatAllocation struct {
    Stat   stat.ID
    Points int
}
```

### Adding a New Character Class

1. Create `internal/character/<classname>.go` embedding `BaseCharacter` (from `internal/character/core/`).
2. Implement all methods of the `context.Character` interface (all `Kill*` methods, `KillMonsterSequence`, `ShouldIgnoreMonster`, `CheckKeyBindings`, `BuffSkills`, `PreCTABuffSkills`).
3. Register a lowercase string key in `BuildCharacter()` in `internal/character/character.go`.
4. For a leveling character, also implement `context.LevelingCharacter` and register under the leveling branch.

### Registered Character Builds

**Leveling**: AmazonLeveling, AssassinLeveling, BarbLeveling, DruidLeveling, NecromancerLeveling, paladin.NewLeveling, SorceressLeveling, WarlockLeveling

**Endgame**: DevelopmentCharacter, MuleCharacter, Javazon, Trapsin, MosaicSin, Berserker, WarcryBarb, WhirlwindBarb, WindDruid, paladin.NewDefault (hammerdin/foh), paladin.NewDragon, BlizzardSorceress, FireballSorceress, NovaSorceress, HydraOrbSorceress, LightningSorceress

### Character Implementation Conventions

- Every `KillMonsterSequence` loop **must** call `ctx.PauseIfNotPriority()` at the top.
- Use `s.preBattleChecks(id, skipOnImmunities)` before attacking to handle immunities and distance checks.
- Use `step.PrimaryAttack` or `step.SecondaryAttack` with functional `AttackOption`s (`step.Distance(min, max)`, `step.RangedDistance(min, max)`, `step.StationaryDistance(min, max)`, `step.EnsureAura(skill.ID)`).
- Constants for attack loop counts belong in the same file as the character (e.g., `hammerdinMaxAttacksLoop`).

---

## Event System (`internal/event/`)

### Core Types

```go
type Event interface {
    Message() string
    Image() image.Image
    OccurredAt() time.Time
    Supervisor() string
}

type BaseEvent struct {
    message    string
    image      image.Image
    occurredAt time.Time
    supervisor string
}

// Constructors:
func Text(supervisor, message string) BaseEvent
func WithScreenshot(supervisor, message string, img image.Image) BaseEvent
```

### Event Types

| Event | Fields | Constructor |
|---|---|---|
| `UsedPotionEvent` | PotionType, OnMerc | `UsedPotion(be, pt, onMerc)` |
| `GameCreatedEvent` | Name, Password | `GameCreated(be, name, password)` |
| `GameFinishedEvent` | Reason (FinishReason) | `GameFinished(be, reason)` |
| `RunFinishedEvent` | RunName, Reason | `RunFinished(be, runName, reason)` |
| `RunStartedEvent` | RunName | (direct struct init) |
| `ItemStashedEvent` | Item (data.Drop) | `ItemStashed(be, drop)` |
| `ItemBlackListedEvent` | Item (data.Drop) | `ItemBlackListed(be, drop)` |
| `GamePausedEvent` | Paused (bool) | (direct struct init) |
| `CompanionLeaderAttackEvent` | — | (direct struct init) |
| `CompanionRequestedTPEvent` | — | (direct struct init) |
| `InteractedToEvent` | InteractionType | (direct struct init) |
| `RunewordRerollEvent` | — | (direct struct init) |
| `NgrokTunnelEvent` | — | (direct struct init) |
| `RequestCompanionJoinGameEvent` | — | (direct struct init) |
| `ResetCompanionGameInfoEvent` | — | (direct struct init) |
| `MonsterKilledEvent` | — | (direct struct init) |
| `CharacterSwitchEvent` | CurrentCharacter, NextCharacter | `CharacterSwitch(be, cur, next)` |

### Finish Reasons

```go
const (
    FinishedOK          FinishReason = "ok"
    FinishedDied        FinishReason = "death"
    FinishedChicken     FinishReason = "chicken"
    FinishedMercChicken FinishReason = "merc chicken"
    FinishedError       FinishReason = "error"
)
```

### Listener

```go
type Handler func(ctx context.Context, e Event) error

type Listener struct {
    handlers     []Handler
    DropHandlers map[int]Handler
    logger       *slog.Logger
}

func NewListener(logger *slog.Logger) *Listener
func (l *Listener) Register(h Handler)
func (l *Listener) Listen(ctx context.Context) error  // main event loop
func (l *Listener) WaitForEvent(ctx context.Context) Event

func Send(e Event)  // global channel send
```

Events flow through a global `chan Event`. The `Listener.Listen()` loop dispatches to registered handlers and saves screenshots if `config.Koolo.Debug.Screenshots` is enabled.

---

## Game Interaction Layer (`internal/game/`)

### MemoryReader

```go
type MemoryReader struct {
    cfg            *config.CharacterCfg
    *memory.GameReader                     // from d2go
    mapSeed        uint
    HWND           uintptr
    WindowLeftX, WindowTopY int
    GameAreaSizeX, GameAreaSizeY int
    supervisorName string
    cachedMapData  map[area.ID]AreaData
    logger         *slog.Logger
}

func (mr *MemoryReader) FetchMapData() error    // builds collision grids, parallelized
func (mr *MemoryReader) GetData() game.Data     // reads full game state snapshot
func (mr *MemoryReader) GetInventory() data.Items
```

### MemoryInjector

Hooks USER32.dll functions in the game process to simulate input:

```go
type MemoryInjector struct {
    isLoaded              bool
    pid, handle           uintptr
    // Stored original bytes and addresses for:
    // GetCursorPos, TrackMouseEvent, GetKeyState, SetCursorPos
}

func (mi *MemoryInjector) Load() error               // hooks USER32 functions
func (mi *MemoryInjector) Unload()                    // unhooks
func (mi *MemoryInjector) CursorPos(x, y int)         // overrides cursor position
func (mi *MemoryInjector) OverrideGetKeyState(key int) // simulates key press
func (mi *MemoryInjector) EnableCursorOverride()
func (mi *MemoryInjector) DisableCursorOverride()
```

### HID (Human Interface Device)

```go
type HID struct {
    gr *MemoryReader
    gi *MemoryInjector
}
func NewHID(gr *MemoryReader, gi *MemoryInjector) *HID
```

Provides keyboard and mouse input methods. Defined across `hid.go`, `mouse.go`, `keyboard.go`.

### Manager

```go
type Manager struct {
    gr             *MemoryReader
    hid            *HID
    supervisorName string
}

func (m *Manager) ExitGame() error
func (m *Manager) NewGame() error
func (m *Manager) CreateLobbyGame(gameCounter int) (string, string, error)
func (m *Manager) JoinOnlineGame(gameName, password string) error
```

### PacketSender

```go
type PacketSender struct {
    ProcessSender  // interface from d2go
}

func (ps *PacketSender) SendPacket() error
func (ps *PacketSender) PickUpItem(unitID, x, y int) error
func (ps *PacketSender) InteractWithTp(unitID int) error
func (ps *PacketSender) InteractWithEntrance(entranceID int) error
func (ps *PacketSender) Teleport(x, y int) error
func (ps *PacketSender) TelekinesisInteraction(unitID int) error
func (ps *PacketSender) CastSkillAtLocation(x, y int) error
func (ps *PacketSender) SelectRightSkill(skillID skill.ID) error
func (ps *PacketSender) SelectLeftSkill(skillID skill.ID) error
func (ps *PacketSender) LearnSkill(skillID skill.ID) error
func (ps *PacketSender) AllocateStatPoint(statID stat.ID) error
```

### Data (`internal/game/data.go`)

```go
type Data struct {
    data.Data                          // embedded from d2go
    Areas              map[area.ID]AreaData
    AreaData           AreaData
    CharacterCfg       *config.CharacterCfg
    IsLevelingCharacter bool
    ExpChar            uint            // 1=Classic, 2=LoD, 3=DLC
}

func (d Data) IsDLC() bool
func (d Data) CanTeleport() bool       // checks config, gold, area, mana, skill binding
func (d Data) PlayerCastDuration() int
func (d Data) MonsterFilterAnyReachable() bool
func (d Data) HasPotionInInventory(potionType) bool
func (d Data) PotionsInInventory(potionType) int
func (d Data) MissingPotionCountInInventory(potionType) int
func (d Data) ConfiguredInventoryPotionCount(potionType) int
```

---

## Pathfinding (`internal/pather/`)

```go
type PathFinder struct {
    gr           *game.MemoryReader
    data         *game.Data
    hid          *game.HID
    cfg          *config.CharacterCfg
    packetSender *game.PacketSender
    astarBuffers *astar.Buffers
}

func (pf *PathFinder) GetPath(to data.Position) (path.Path, error)
func (pf *PathFinder) GetPathFrom(from, to data.Position) (path.Path, error)
```

Special handling for Arcane Sanctuary, Lut Gholein map bugs, cross-area grid merging, obstacle avoidance (objects, monsters, barricade towers).

Files: `path.go`, `path_finder.go`, `render_map.go`, `utils.go`, `astar/` subdirectory.

---

## Health System (`internal/health/`)

### Health Manager

```go
var ErrDied       = errors.New("death")
var ErrChicken    = errors.New("chicken")
var ErrMercChicken = errors.New("merc chicken")

type Manager struct {
    bm                 *BeltManager
    lastHealingTime    time.Time     // interval: 4s
    lastMercHealTime   time.Time     // interval: 3s
    lastManaTime       time.Time     // interval: 4s
    lastRejuvTime      time.Time     // interval: 1s
}

func (hm *Manager) HandleHealthAndMana() error
func (hm *Manager) ShouldPickStaminaPot() bool
func (hm *Manager) ShouldKeepStaminaPot() bool
func (hm *Manager) IsLowStamina() bool
```

### Belt Manager

```go
type BeltManager struct {
    data       *game.Data
    hid        *game.HID
    logger     *slog.Logger
    supervisor string
}

func (bm *BeltManager) DrinkPotion(potionType, merc bool) error
func (bm *BeltManager) ShouldBuyPotions() bool              // <75% of target
func (bm *BeltManager) GetMissingCount(potionType) int
```

---

## Chicken System (`internal/chicken/`)

```go
func CheckForScaryAuraAndCurse() // panics with health.ErrChicken on configured threats
```

Checks player states for configured curses (`AmplifyDamage`, `Decrepify`, `LowerResist`, `BloodMana`) and nearby monster auras (`Fanaticism`, `Might`, `Conviction`, `HolyFire`, `BlessedAim`, `HolyFreeze`, `HolyShock`) within `RangeForScaryAura = 25`.

Uses `panic(fmt.Errorf("%w: ...", health.ErrChicken))` pattern for stack unwinding.

---

## Drop System (`internal/drop/`)

### Architecture

Four files implement three layers:

1. **`Service`** (`drop_service.go`) — top-level entry point. Holds the `Coordinator`, queued starts, and persistent requests (3min TTL). Provides callbacks for clearing filters, persisting requests, and reporting results.

2. **`Coordinator`** (`drop_coordinator.go`) — orchestrates per-supervisor filters and callbacks. Methods: `SetFilters()`, `ApplyInitialFilters()` (defaults `DropperOnlySelected:true`), `ConfigureCallbacks()`, `ClearIndividualFilters()`.

3. **`Manager`** (`drop_manager.go`) — per-supervisor instance. Manages pending/active drop requests and filter state.

### Key Types

```go
// Sentinel error for interrupting runs when a drop is requested
var ErrInterrupt = errors.New("Drop requested")

type Request struct {
    RoomName  string
    Password  string
    Filters   *Filters
    CreatedAt time.Time
    CardID    string
    CardName  string
}

type Filters struct {
    Enabled             bool
    DropperOnlySelected bool
    SelectedRunes       map[string]ItemQuantity
    SelectedGems        map[string]ItemQuantity
    SelectedKeyTokens   map[string]ItemQuantity
    CustomItems         []string
    AllowedQualities    []string
}
```

### Drop Item Evaluation

`ShouldDropperItem(item)` returns true when:
- Item name matches a selected rune/gem/keyToken with remaining quota, OR
- Item name matches a custom item, OR
- Item quality matches `AllowedQualities` (excludes runes/gems from quality-only matching)

`HasRemainingDropQuota()` checks if any configured quota has remaining inventory.

---

## Stash System (`internal/action/stash.go`)

Key constants:
```go
const maxGoldPerStashTab = 2500000
const maxGoldPerSharedStash = 7500000
// DLC stash tabs:
const StashTabGems = 100
const StashTabMaterials = 101
const StashTabRunes = 102
```

---

## Item Pickup (`internal/action/item_pickup.go`)

`ItemPickup(maxDistance)` — complex multi-attempt loop with:
- 5 base attempts + 5 "too far" attempts per item
- Inventory fit checking via 2D grid scan
- Town trip management when inventory is full (stash/sell)
- Item blacklisting on repeated failures
- NIP rule evaluation for pick decisions

---

## Auto-Equipment (`internal/action/autoequip.go`)

`AutoEquip()` — evaluates and equips items for player and mercenary in a loop until stable (max 30 iterations). Uses tier-based scoring system defined in `autoequip_tiers.go` and `autoequip_meta_item_score.go`.

---

## Runeword System (`internal/action/runeword_maker.go`)

`MakeRunewords()` — iterates enabled recipes, finds bases and inserts runes/gems via `SocketItems()`. Supports:
- DLC RunesTab stash navigation
- `AutoUpgrade` and `OnlyIfWearable` flags
- `AutoTierByDifficulty` for automatic base selection
- Reroll rules for re-rolling suboptimal runewords

---

## Cube Recipes (`internal/action/cube_recipes.go`)

```go
type CubeRecipe struct {
    Name           string
    Items          []string
    PurchaseRequired bool
    PurchaseItems  []string
}

var Recipes []CubeRecipe  // ~35 recipes: perfect gems (7), Token of Absolution, rune upgrades (El→Cham), socket adding
```

---

## Gambling (`internal/action/gambling.go`)

`Gamble()` — triggers when stashed gold ≥ 2.48M. Visits gambling NPC per act.
`GambleSingleItem(items, desiredQuality)` — targeted gambling for specific quality.

Max 5 purchases per item type per session. Supports coronet/circlet grouping and NIP rule evaluation.

---

## Shopping (`internal/action/shopping_action.go`)

```go
type ActionShoppingPlan struct {
    Enabled         bool
    RefreshesPerRun int
    MinGoldReserve  int
    Vendors         []npc.ID
    Rules           nip.Rules
    Types           []string
}

func RunShopping(plan ActionShoppingPlan) error
```

Multi-pass shopping across towns/vendors with drop interrupt checks and inventory space validation (`ensureTwoFreeColumnsStrict()`).

---

## Mule System (`internal/mule/`)

```go
type Manager struct {
    logger *slog.Logger
}

func (m *Manager) ShouldMule(stashFull bool, characterName string) (bool, string)
func (m *Manager) IsMuleCharacter(characterName string) bool
```

Checks `MuleProfiles` list, returns first matching mule profile name. `IsMuleCharacter` returns true when `Muling.Enabled && ReturnTo != ""`.

---

## Town System (`internal/town/`)

```go
type Town interface {
    RefillNPC() npc.ID
    HealNPC() npc.ID
    RepairNPC() npc.ID
    MercContractorNPC() npc.ID
    GamblingNPC() npc.ID
    IdentifyNPC() npc.ID
    TPWaitingArea() data.Position
    TownArea() area.ID
}

func GetTownByArea(a area.ID) Town  // returns A1{}, A2{}, A3{}, A4{}, or A5{}
```

Files: `A1.go`–`A5.go`, `shop_manager.go`, `town.go`.

---

## Terror Zone System (`internal/terrorzone/`)

```go
type Tier int
const (
    TierS Tier = iota
    TierA
    TierB
    TierC
    TierD
    TierF
)

type ZoneInfo struct {
    Act        int
    ExpTier    Tier
    LootTier   Tier
    BossPack   bool
    Immunities []stat.Resist
    Group      string
}

func Info(id area.ID) ZoneInfo
func ExpTierOf(id area.ID) Tier
func LootTierOf(id area.ID) Tier
func Zones() map[area.ID]ZoneInfo    // ~35 entries
func Groups() map[string][]area.ID
```

---

## Scheduler (`internal/bot/scheduler.go`)

Two scheduling modes:

1. **`timeSlots`** — day-of-week based. Deterministic variance offsets. Starts/stops supervisors based on configured time ranges.

2. **`duration`** — human-like play patterns. State machine with phases:
   ```go
   const (
       PhaseResting  = "resting"
       PhasePlaying  = "playing"
       PhaseOnBreak  = "on_break"
   )
   ```
   Supports wake time, play hours, meal breaks, short breaks, jitter.

---

## Companion System (`internal/bot/companion.go`)

Handles `RequestCompanionJoinGameEvent` (stores game name/password for follower) and `ResetCompanionGameInfoEvent` (clears game info). Leader creates games; followers join via stored game info.

---

## Armory (`internal/bot/armory.go`)

```go
type ArmoryItem struct {
    ID, Name, Quality string
    Ethereal, IsRuneword bool
    RunewordName string
    Stats, BaseStats map[string]interface{}
    Sockets int
    ImageName string
    Defense, Damage, Durability interface{}
}

type ArmoryCharacter struct {
    CharacterName, Class string
    Level int
    Experience uint64
    Gold, StashedGold int
    Equipped, Inventory, Stash []ArmoryItem
    SharedStash1, SharedStash2, SharedStash3 []ArmoryItem
    SharedStash4, SharedStash5, SharedStash6 []ArmoryItem
    GemsTab, MaterialsTab, RunesTab []ArmoryItem  // DLC stash tabs
    Cube, Belt, Mercenary []ArmoryItem
}
```

---

## Pickit System (`internal/pickit/`)

### Types (`types.go`)

Rich type system for the visual pickit editor:

```go
type ItemDefinition struct {
    ID, Name, NIPName, InternalName, Type, BaseItem string
    Quality        []item.Quality
    AvailableStats []StatType
    MaxSockets     int
    Ethereal       bool
    ItemLevel      int
    Category, Rarity, Description string
}

type StatType struct {
    ID, Name, NipProperty string
    MinValue, MaxValue    float64
    IsPercent             bool
}

type PickitRule struct {
    ID, ItemName, ItemID, FileName string
    Enabled                        bool
    Priority                       int
    LeftConditions, RightConditions []Condition
    MaxQuantity                    int
    IsScored                       bool
    ScoreThreshold                 float64
    ScoreWeights                   map[string]float64
    Comments, GeneratedNIP         string
}

type Condition struct {
    Property, Operator string
    Value              interface{}
    NipSyntax          string
}

// Also: PickitFile, PickitSet, RuleTemplate, ValidationResult,
// SimulationResult, ItemMatch, StatPreset, ConflictDetection,
// EditorPreferences, ExportOptions, ImportOptions, SearchFilters,
// AutoSuggestion
```

Files: `item_database.go`, `item_database_v2.go`, `nip_builder.go`, `stats.go`, `templates.go`, `types.go`.

---

## Packet System (`internal/packet/`)

10 files providing packet-based alternatives to HID interaction:

| File | Purpose |
|---|---|
| `allocate_stat.go` | Stat point allocation |
| `cast_skill_entity_left.go` | Left-click skill on entity |
| `cast_skill_entity_right.go` | Right-click skill on entity |
| `cast_skill_location.go` | Skill at map position |
| `entrance_interaction.go` | Area entrance interaction |
| `learn_skill.go` | Skill learning |
| `object_interaction.go` | Object interaction |
| `pickup_item.go` | Item pickup |
| `skill_selection.go` | Skill selection |
| `tp_interaction.go` | Town Portal interaction |

---

## HTTP Server (`internal/server/http_server.go`)

4285-line file providing the web overlay UI, REST API, and WebSocket status broadcast.

### Server Struct

```go
type HttpServer struct {
    logger                  *slog.Logger
    server                  *http.Server
    manager                 *SupervisorManager
    scheduler               *Scheduler
    templates               *template.Template
    wsServer                *WebSocketServer
    pickitAPI               *PickitAPI
    sequenceAPI             *SequenceAPI
    updater                 *Updater
    DropHistory, RunewordHistory   interface{}
    DropFilters, DropCardInfo      interface{}
    cachedAnalyticsManagers        interface{}
}
```

### WebSocket

```go
type WebSocketServer struct {
    clients    map[*websocket.Conn]bool
    broadcast  chan []byte
    register   chan *websocket.Conn
    unregister chan *websocket.Conn
}
```

`BroadcastStatus()` sends JSON status every 1 second to all connected WebSocket clients.

### Embedded Assets

```go
//go:embed all:assets
var assetsFS embed.FS

//go:embed all:templates
var templatesFS embed.FS
```

### HTTP Routes

#### Main Pages
- `GET /` — dashboard
- `GET /config` — configuration page
- `GET /supervisorSettings` — supervisor settings
- `GET /runewords` — runeword manager
- `GET /debug` — debug page
- `GET /debug-data` — debug data

#### Bot Control
- `POST /start` — start supervisor
- `POST /stop` — stop supervisor
- `POST /togglePause` — toggle pause
- `POST /autostart/toggle` — toggle autostart
- `POST /autostart/run-once` — run once

#### Drops
- `GET /drops` — drops page
- `GET /all-drops` — all drops view
- `GET /export-drops` — export drop logs
- `POST /open-droplogs` — open drop log directory
- `POST /reset-droplogs` — reset drop logs

#### Process Management
- `GET /process-list` — list game processes
- `POST /attach-process` — attach to existing process

#### WebSocket & Live Data
- `GET /ws` — WebSocket endpoint
- `GET /initial-data` — initial dashboard data

#### API
- `POST /api/reload-config` — reload configuration
- `POST /api/companion-join` — trigger companion join
- `POST /api/generate-battlenet-token` — generate auth token
- `POST /reset-muling` — reset muling state

#### Runeword API
- `GET /api/runewords/rolls` — runeword roll data
- `GET /api/runewords/base-types` — available base types
- `GET /api/runewords/bases` — base items
- `GET /api/runewords/history` — runeword history

#### Updater API
- `GET /api/updater/version` — current version
- `GET /api/updater/check` — check for updates
- `GET /api/updater/current-commits` — current commits
- `POST /api/updater/update` — perform update
- `GET /api/updater/status` — update status
- `GET /api/updater/backups` — list backups
- `POST /api/updater/rollback` — rollback to backup
- `GET /api/updater/prs` — list PRs
- `POST /api/updater/cherry-pick` — cherry-pick PR
- `POST /api/updater/prs/revert` — revert cherry-picked PR

#### Pickit Editor
- `GET /pickit-editor` — pickit editor page
- `GET /api/pickit/items` — item definitions
- `GET /api/pickit/items/search` — search items
- `GET /api/pickit/items/categories` — item categories
- `GET /api/pickit/stats` — available stats
- `GET /api/pickit/templates` — rule templates
- `GET /api/pickit/presets` — stat presets
- `CRUD /api/pickit/rules` — create/read/update/delete rules + validate
- `GET /api/pickit/files` — list/import/export pickit files
- `POST /api/pickit/browse-folder` — browse file system
- `POST /api/pickit/simulate` — simulate rule matching

#### Sequence Editor
- `GET /sequence-editor` — sequence editor page
- `GET /api/sequence-editor/runs` — available runs
- `GET /api/sequence-editor/file` — read sequence file
- `POST /api/sequence-editor/open` — open sequence
- `POST /api/sequence-editor/save` — save sequence
- `DELETE /api/sequence-editor/delete` — delete sequence
- `GET /api/sequence-editor/files` — list sequence files

#### Armory
- `GET /armory` — armory page
- `GET /api/armory` — character equipment data
- `GET /api/armory/characters` — character list
- `GET /api/armory/all` — all characters equipment

#### Analytics
- `GET /analytics` — analytics page
- `GET /api/analytics/session` — session analytics
- `GET /api/analytics/global` — global analytics
- `GET /api/analytics/runs` — run analytics
- `GET /api/analytics/hourly` — hourly breakdown
- `GET /api/analytics/run-types` — run type stats
- `GET /api/analytics/items` — item analytics
- `GET /api/analytics/characters` — character analytics
- `POST /api/analytics/reset` — reset analytics
- `GET /api/analytics/deaths` — death analytics
- `GET /api/analytics/runes` — rune drop analytics
- `GET /api/analytics/session-history` — session history
- `GET /api/analytics/comparison` — comparison analytics

#### Drop Manager
- `GET /Drop-manager` — drop manager page
- (Additional drop routes registered via `s.registerDropRoutes()`)

#### Other
- `GET /api/skill-options` — available skill options
- `POST /api/supervisors/bulk-apply` — bulk apply settings
- `GET /api/scheduler-history` — scheduler history
- Static: `/assets/*`, `/items/*`

---

## Concurrency & Safety

- `context.botContexts` is a `map[uint64]*Status` protected by `var mu sync.Mutex`. Each goroutine attaches with `ctx.AttachRoutine(priority)` and detaches with `ctx.Detach()`.
- `CurrentGameHelper.mutex` (`sync.Mutex`) guards `IsPickingItems`; use `ctx.SetPickingItems(bool)`.
- `ctx.IsAllocatingStatsOrSkills` is an `atomic.Bool` used to suppress stuck detection during allocation; set and clear it around stat/skill allocation blocks.
- The `lastActivityTimeMux` in `Bot` guards activity tracking; use the provided accessors.
- Never add `sync.Mutex` fields directly to `Context` — route shared state through `CurrentGameHelper` or dedicated managers.
- The `Bot.Run()` uses `errgroup` to manage 4 concurrent goroutines with shared cancellation.
- Event channel (`var events = make(chan Event)`) is unbuffered — `Send()` blocks until `Listen()` consumes.
- Config access is guarded by `cfgMux` (`sync.RWMutex`).

---

## Error Handling Patterns

### Chicken Panic Pattern

Health emergencies use `panic` for stack unwinding:
```go
panic(fmt.Errorf("%w: scary aura detected", health.ErrChicken))
```

Recovered at the `Bot.Run()` level:
```go
defer func() {
    if r := recover(); r != nil {
        if errors.Is(err, health.ErrChicken) { ... }
    }
}()
```

### Sentinel Errors

| Error | Package | Usage |
|---|---|---|
| `health.ErrDied` | health | Player died |
| `health.ErrChicken` | health | Health chicken triggered |
| `health.ErrMercChicken` | health | Merc chicken triggered |
| `drop.ErrInterrupt` | drop | Drop requested, interrupt current run |
| `step.ErrMonstersInPath` | step | Monsters blocking movement |
| `step.ErrPlayerStuck` | step | Player cannot move |
| `step.ErrPlayerRoundTrip` | step | Player detected looping back |
| `step.ErrNoPath` | step | No valid path found |

---

## File and Naming Conventions

- **Package names**: all lowercase, single word matching the directory name.
- **Character class strings** in config are lowercase (e.g., `"hammerdin"`, `"sorceress"`, `"trapsin"`).
- **Run name constants** follow `XxxRun Run = "snake_case"` pattern in `config/runs.go`.
- **File names**: lowercase with underscores. Exception: `Inventory.go` exists with a non-standard name — preserve it as-is; do **not** rename without a deliberate refactor task.
- **Constructor functions**: `NewXxx()` returns a pointer or value depending on whether the type uses pointer receivers.
- **Action functions**: top-level unexported or exported functions in `internal/action/`, not methods. They call `context.Get()` internally.
- **Step functions**: exported functions in `internal/action/step/`, return `error`. Accept configuration via functional options (`AttackOption`, etc.).

---

## Build & Tooling

- Build with `better_build.bat` or `build.bat` on Windows.
- The entry point is `cmd/koolo/main.go`.
- Windows resource (manifest, icon) is embedded via `cmd/koolo/rsrc_windows_amd64.syso`.
- Log output goes to a directory configured by `config.Koolo.LogSaveDirectory`.
- `go vet ./...` and `gofmt -l .` must produce no output before committing.
## Project Overview

**Zenshiro** is a Windows-only Diablo 2 bot written in Go 1.23. It drives the game by reading process memory and sending HID (keyboard/mouse) input. The module path is `github.com/hectorgimenez/koolo`.

### Technology Stack

- **Go 1.23**, `gofmt`-formatted.
- **Windows-only**: uses `github.com/lxn/win`, `golang.org/x/sys/windows`, Win32 APIs. Do not introduce cross-platform abstractions.
- **Game data types**: all Diablo 2 entities (areas, monsters, NPCs, skills, items, stats, quests, objects) come from `github.com/hectorgimenez/d2go/pkg/data/...`. The `go.mod` `replace` directive currently points this to a private fork (`github.com/crazywh1t3/d2go`). Never inline or duplicate types from this package.
- **Configuration**: YAML files under `config/`, parsed by `gopkg.in/yaml.v3`. Global config is `config.Koolo`; per-character config is `ctx.CharacterCfg`.
- **Logging**: `log/slog` with structured key-value pairs via `ctx.Logger`. Never use `fmt.Print*` or `log.*` (standard library) for runtime logging.

### Key External Packages

| Import path | Purpose |
|---|---|
| `github.com/hectorgimenez/d2go/pkg/data` | Core game data structures (`Item`, `Monster`, `Position`, …) |
| `github.com/hectorgimenez/d2go/pkg/data/area` | Area IDs |
| `github.com/hectorgimenez/d2go/pkg/data/npc` | NPC IDs |
| `github.com/hectorgimenez/d2go/pkg/data/skill` | Skill IDs |
| `github.com/hectorgimenez/d2go/pkg/data/stat` | Stat/resist IDs |
| `github.com/hectorgimenez/d2go/pkg/data/item` | Item names, locations, body slots |
| `github.com/hectorgimenez/d2go/pkg/data/object` | Object IDs (shrines, portals, chests) |
| `github.com/hectorgimenez/d2go/pkg/data/quest` | Quest IDs |
| `github.com/hectorgimenez/d2go/pkg/nip` | NIP rule evaluation (pickit) |

---

## Package Structure

```
internal/
  action/         High-level bot actions (move, pickup, fight, stash, buff, …)
    step/         Low-level atomic primitives (attack, cast, interact, move step)
  bot/            Supervisor, manager, scheduler, bot loop, stuck detection
  buildnoise/     Audio noise generation utilities
  character/      Per-class character implementations
    core/         Shared character helpers (BaseCharacter, preBattleChecks)
    paladin/      Paladin variants (Hammerdin, FoH, Dragon, leveling)
  chicken/        Flee-on-low-health logic (scary auras/curses)
  config/         Config loading, CharacterCfg, KooloCfg, run/build constants
  context/        Goroutine-local execution context; the central data carrier
  drop/           Per-supervisor drop tracking, filtering, and coordination
  event/          Event broadcast system (Discord, Telegram, internal listeners)
  game/           Memory reader/injector, HID (keyboard/mouse), game manager, crash detector
  health/         Belt manager, health manager, ping monitor
  mule/           Mule (item transfer) orchestration
  packet/         Packet-based game interaction (alternative to HID)
  pather/         A* pathfinding, map rendering, collision grid
    astar/        A* algorithm implementation
  pickit/         NIP rule compilation, item evaluation, pickit editor API
  remote/         Remote control integrations
    discord/      Discord webhook/bot notifications
    droplog/      Drop log file writer
    ngrok/        Ngrok tunnel for remote access
    telegram/     Telegram bot notifications
  run/            Run implementations (one file per boss/area/quest)
  server/         HTTP server for the web overlay, WebSocket status broadcast
  terrorzone/     Terror zone detection, tier scoring, and selection
  town/           Act-specific town routines (A1.go – A5.go)
  ui/             UI interaction helpers (menus, inventory coordinates)
  updater/        Auto-update, cherry-pick, rollback logic
  utils/          Sleep helpers, math, randomness, Windows utilities
    winproc/      Windows process utilities
```

---

## Core Patterns

### The Context Pattern

Every goroutine that executes bot logic retrieves its context with:

```go
ctx := context.Get()
```

`context.Get()` is keyed by **goroutine ID** (via `runtime.Stack`). Each action function must call `ctx.SetLastAction("FunctionName")` at entry for stuck-detection and debug tracing. Steps call `ctx.SetLastStep("StepName")` analogously.

The `context.Status` struct wraps `*Context` with a `Priority` field:

```go
type Status struct {
    *Context
    Priority Priority
}
```

The `context.Context` struct is the single source of truth for:
- `ctx.Data` — last-read game state (`*game.Data`); refresh with `ctx.RefreshGameData()`
- `ctx.CharacterCfg` — per-character YAML config (`*config.CharacterCfg`)
- `ctx.Logger` — structured `*slog.Logger`
- `ctx.HID` — keyboard/mouse input driver (`*game.HID`)
- `ctx.GameReader` — memory reader (`*game.MemoryReader`)
- `ctx.MemoryInjector` — memory injector for hooking USER32.dll (`*game.MemoryInjector`)
- `ctx.PathFinder` — A* pathfinder (`*pather.PathFinder`)
- `ctx.BeltManager` — potion belt manager (`*health.BeltManager`)
- `ctx.HealthManager` — health/mana/chicken manager (`*health.Manager`)
- `ctx.Char` — the active `Character` or `LevelingCharacter` implementation
- `ctx.CurrentGame` — per-game transient state (`*CurrentGameHelper`)
- `ctx.PacketSender` — packet-based game interaction (`*game.PacketSender`)
- `ctx.Drop` — per-supervisor drop manager (`*drop.Manager`)
- `ctx.EventListener` — event broadcast system (`*event.Listener`)
- `ctx.Manager` — game lifecycle manager (`*game.Manager`)

### Context Lifecycle

```go
// Creation (in buildSupervisor):
ctx := context.NewContext(supervisorName)  // attaches to current goroutine

// In spawned goroutines:
ctx.AttachRoutine(priority)  // registers goroutine with given priority
defer ctx.Detach()           // removes goroutine from context map

// Retrieval (anywhere in bot logic):
ctx := context.Get()         // returns *Status for current goroutine
```

**Never pass `*context.Context` through call arguments where `context.Get()` is available.** Use `context.Status` (the goroutine-local wrapper) when you need priority metadata.

### Priority System

```go
const (
    PriorityHigh       = 0   // chicken, health emergencies, item pickup
    PriorityNormal     = 1   // standard bot operation (run execution)
    PriorityBackground = 5   // background health monitoring, data refresh
    PriorityPause      = 10  // game paused
    PriorityStop       = 100 // supervisor stopping (triggers panic inside loops)
)
```

**Every loop** inside a run or character kill sequence **must** call `ctx.PauseIfNotPriority()` at the top of each iteration. This is the pause/stop mechanism. Omitting it causes the bot to ignore pause/stop commands.

`PauseIfNotPriority()` blocks when the goroutine's priority does not match `ExecutionPriority`. When `ExecutionPriority == PriorityStop`, it panics with `"Bot is stopped"` to unwind the call stack.

### Game Data Refresh

`ctx.Data` is a snapshot; it goes stale as the game advances. Call `ctx.RefreshGameData()` before reading positional or state-sensitive data. In tight loops, refresh at most once per iteration.

```go
func (ctx *Context) RefreshGameData()   // full refresh: all game state
func (ctx *Context) RefreshInventory()  // inventory-only refresh (lighter)
```

### Sleep Utilities

| Function | When to use |
|---|---|
| `utils.Sleep(ms)` | Simple fixed wait with ±30% jitter (randomizes between 0.7× and 1.3×) |
| `utils.PingSleep(utils.Light, ms)` | Polling / lightweight checks — adds 1× ping to base delay |
| `utils.PingSleep(utils.Medium, ms)` | UI interactions, clicks, movement waits — adds 2× ping to base delay |
| `utils.PingSleep(utils.Critical, ms)` | State transitions, portal enters, game joins — adds 4× ping to base delay |
| `utils.RetryDelay(attempt, basePing, minMs)` | Escalating retry delays — `minMs + basePing × ping × attempt` |

All ping-adaptive functions cap at 5000ms. Never use `time.Sleep` directly in bot logic — it ignores network latency. Use the utilities above.

The ping getter is initialized in `NewContext()` via `utils.SetPingGetter()` to read `ctx.Data.Game.Ping`, defaulting to 50ms when unavailable.

### Action vs. Step

- **`internal/action/`** — composite actions that may call multiple steps, refresh game data, loop, and handle errors. Named functions like `action.MoveToArea(...)`, `action.ItemPickup(...)`. These are **top-level functions** (not methods) that call `context.Get()` internally.
- **`internal/action/step/`** — single atomic operations that interact with the game once or in a tight loop. `step.Attack(...)`, `step.InteractNPC(...)`, `step.MoveTo(...)`. Steps do not call other steps.

#### Step Files (17 files):

| File | Purpose |
|---|---|
| `attack.go` | `PrimaryAttack()`, `SecondaryAttack()` with functional options |
| `cast.go` | Skill casting |
| `close_all_menus.go` | Close all open game menus |
| `interact_entrance.go` | Area entrance interaction |
| `interact_entrance_packet.go` | Packet-based entrance interaction |
| `interact_npc.go` | NPC interaction |
| `interact_object.go` | Object interaction (shrines, chests, etc.) |
| `interact_object_packet.go` | Packet-based object interaction |
| `interrupt.go` | Drop interrupt handling |
| `move.go` | `MoveTo()` with pathfinding, stuck detection, area transitions |
| `open_inventory.go` | Open inventory screen |
| `open_portal.go` | Open Town Portal |
| `pickup_item.go` | Pick up a single item |
| `pickup_item_packet.go` | Packet-based item pickup |
| `set_skill.go` | Set active skill |
| `skill_selection.go` | Skill selection logic |
| `swap_weapon.go` | CTA weapon swap |

#### Action Files (48 files):

Key action files include: `item_pickup.go` (complex multi-attempt pickup with blacklisting), `stash.go` (gold/inventory stashing with tab management), `autoequip.go` (automated equipment evaluation with tier scoring), `runeword_maker.go` (runeword crafting with reroll support), `cube_recipes.go` (Horadric Cube recipe automation), `gambling.go` (gold gambling at NPCs), `shopping_action.go` (vendor shopping with NIP rules), `move.go` (high-level movement), `clear_area.go`/`clear_level.go` (area clearing), `buff.go` (skill buffing), `town.go` (town routines), `tp_actions.go` (Town Portal management).

---

## Bot Main Loop (`internal/bot/bot.go`)

The `Bot.Run()` method orchestrates four concurrent goroutines via `errgroup`:

1. **Background goroutine** (`PriorityBackground`): Calls `RefreshGameData()` every 100ms, tracks position changes for idle detection.

2. **Health goroutine** (`PriorityBackground`): Calls `HandleHealthAndMana()` every 100ms. Also checks global idle (2min no movement → quit game) and max game length enforcement. Skipped during Drop runs.

3. **High-priority goroutine** (`PriorityHigh`): Handles item pickup, rebuffing, belt refill, return-to-town decisions (low gold, low potions, broken equipment, merc died), area corrections, legacy mode, and portrait/chat cleanup. Uses check-then-lock pattern.

4. **Normal-priority goroutine** (`PriorityNormal`): Iterates through configured runs, calls `PreRun`/`Run`/`PostRun`. Tracks analytics (XP per run). Handles Drop interrupts (`drop.ErrInterrupt`). Recovers from chicken panics.

### Bot Struct

```go
type Bot struct {
    ctx                   *botCtx.Context
    lastActivityTime      time.Time
    lastActivityTimeMux   sync.RWMutex
    lastKnownPosition     data.Position
    lastPositionCheckTime time.Time
    analyticsManager      *AnalyticsManager
    MuleManager           interface {
        ShouldMule(stashFull bool, characterName string) (bool, string)
    }
}
```

---

## Supervisor System (`internal/bot/supervisor.go`, `manager.go`)

### Supervisor Interface

```go
type Supervisor interface {
    Start() error
    Name() string
    Stop()
    Stats() Stats
    TogglePause()
    SetWindowPosition(x, y int)
    GetData() *game.Data
    GetContext() *ct.Context
    GetAnalyticsManager() *AnalyticsManager
}
```

The `baseSupervisor` struct provides the common implementation. `SinglePlayerSupervisor` (not shown) extends it.

### SupervisorManager

```go
type SupervisorManager struct {
    logger         *slog.Logger
    supervisors    map[string]Supervisor
    crashDetectors map[string]*game.CrashDetector
    eventListener  *event.Listener
    Drop           *drop.Service
}
```

Key methods:
- `Start(supervisorName, attachToExisting, manualMode, pidHwnd)` — reloads config, creates supervisor via `buildSupervisor()`, starts crash detector
- `Stop(supervisorName)` / `StopAll()` — stops supervisors
- `ReloadConfig()` — clears NIP cache and reloads configs; applies to running supervisors
- `TogglePause(supervisorName)` — pause/resume via `MemoryInjector`

### Stats & Status Tracking

```go
type SupervisorStatus string
const (
    NotStarted SupervisorStatus = "Not Started"
    Starting   SupervisorStatus = "Starting"
    InGame     SupervisorStatus = "In game"
    Paused     SupervisorStatus = "Paused"
    Crashed    SupervisorStatus = "Crashed"
)

type Stats struct {
    StartedAt           time.Time
    SupervisorStatus    SupervisorStatus
    Details             string
    Drops               []data.Drop
    Games               []GameStats
    IsCompanionFollower bool
    UI                  CharacterOverview
    MuleEnabled         bool
    ManualModeActive    bool
}

type GameStats struct {
    StartedAt  time.Time
    FinishedAt time.Time
    Reason     event.FinishReason
    Runs       []RunStats
}

type RunStats struct {
    Name        string
    Reason      event.FinishReason
    StartedAt   time.Time
    Items       []data.Item
    FinishedAt  time.Time
    UsedPotions []event.UsedPotionEvent
}

type CharacterOverview struct {
    Class, Difficulty, Area string
    Level, Ping, Life, MaxLife, Mana, MaxMana int
    Experience, LastExp, NextExp uint64
    MagicFind, GoldFind int
    FireResist, ColdResist, LightningResist, PoisonResist int
    Gold int
}
```

---

## Configuration System (`internal/config/config.go`)

### Global Config

```go
var Koolo *KooloCfg
var Characters map[string]*CharacterCfg
var Version = "dev"
```

#### KooloCfg

```go
type KooloCfg struct {
    Debug struct {
        Log, Screenshots, RenderMap bool
    }
    FirstRun                bool
    UseCustomSettings       bool
    GameWindowArrangement   bool
    D2LoDPath, D2RPath      string
    CentralizedPickitPath   string
    WindowWidth, WindowHeight int
    Discord   struct { Enabled bool; ChannelID, Token, WebhookURL string; ... }
    Telegram  struct { Enabled bool; Token, ChatID string; ... }
    Ngrok     struct { Enabled bool; AuthToken string }
    PingMonitor struct { ... }
    AutoStart   struct { Enabled bool; SupervisorNames []string }
    Analytics   struct { Enabled bool; HistoryDays int; MaxNotableDrops int; TrackAllItems bool }
    RunewordFavoriteRecipes []string
    RunFavoriteRuns         []string
    LogSaveDirectory        string
}
```

#### CharacterCfg (per-supervisor)

```go
type CharacterCfg struct {
    MaxGameLength       int
    Username, Password  string
    AuthMethod          string   // "TokenAuth" or empty
    AuthToken           string
    Realm               string
    CharacterName       string
    AutoCreateCharacter bool
    KillD2OnStop        bool
    ClassicMode         bool
    UseCentralizedPickit bool
    PacketCasting       struct { Entrance, Item, TP, Teleport, Entity, Skill bool }
    Scheduler           Scheduler
    Health              struct {
        HealingPotionAt, ManaPotionAt, RejuvPotionAtLife, RejuvPotionAtMana float64
        MercHealingPotionAt, MercRejuvPotionAt float64
        ChickenAt, MercChickenAt float64
    }
    ChickenOnCurses []string   // e.g. "amplify_damage", "decrepify"
    ChickenOnAuras  []string   // e.g. "fanaticism", "conviction"
    Inventory       struct {
        InventoryLock [4][10]int   // 0 = free, 1 = locked
        BeltColumns   BeltColumns
    }
    Character struct {
        Class          string    // e.g. "hammerdin", "sorceress", "trapsin"
        UseMerc        bool
        StashToShared  int       // 0 = personal, 1-3 = shared stash tab
        UseTeleport    bool
        ClearPathDist  int
        AutoStatSkill  AutoStatSkillConfig
        // Per-class sub-configs: BerserkerBarb, WhirlwindBarb, BlizzardSorceress, etc.
    }
    Game struct {
        MinGoldPickup         int
        UseCainIdentify       bool
        InteractWithShrines   bool
        InteractWithChests    bool
        Difficulty            string  // "normal", "nightmare", "hell"
        Runs                  []string
        // Per-run configs:
        Pindleskin  struct { SkipOnImmunities []string }
        Pit         struct { FocusBosses bool }
        Diablo      struct { ... }
        Baal        struct { ... }
        TerrorZone  struct { ... }
        Leveling    struct { ... }
        RunewordMaker struct { Enabled bool; ... }
    }
    Companion struct { Enabled bool; Leader bool; GameNameTemplate string }
    Gambling  struct { Enabled bool; Items []string }
    Muling    struct { Enabled bool; SwitchToMule string; ReturnTo string; MuleProfiles []string }
    CubeRecipes struct { Enabled bool; EnabledRecipes []string; SkipPerfects bool }
    BackToTown  struct { ... }
    Shopping    ShoppingConfig
    Runtime     struct { Rules nip.Rules; TierRules; Drops }
}
```

### Constants

```go
const GameVersionReignOfTheWarlock = "reign_of_the_warlock"
const GameVersionExpansion = "expansion"
```

### Key Config Functions

```go
func Load() error
func GetCharacter(name string) (*CharacterCfg, error)
func GetCharacters() map[string]*CharacterCfg
func CreateFromTemplate(name string) error
func ValidateAndSaveConfig(cfg *CharacterCfg) error
func SaveSupervisorConfig(name string, cfg *CharacterCfg) error
func SaveKooloConfig(cfg *KooloCfg) error
func ClearNIPCache()
```

### Run Constants (`internal/config/runs.go`)

70+ run constants of type `Run string`:

```go
type Run string
const (
    CountessRun       Run = "countess"
    AndarielRun       Run = "andariel"
    SummonerRun       Run = "summoner"
    DurielRun         Run = "duriel"
    MephistoRun       Run = "mephisto"
    DiabloRun         Run = "diablo"
    BaalRun           Run = "baal"
    PindleskinRun     Run = "pindleskin"
    NihlathakRun      Run = "nihlathak"
    TravincalRun      Run = "travincal"
    PitRun            Run = "pit"
    AncientTunnelsRun Run = "ancient_tunnels"
    CowsRun           Run = "cows"
    TerrorZoneRun     Run = "terror_zone"
    LevelingRun       Run = "leveling"
    LevelingSequenceRun Run = "leveling_sequence"
    QuestsRun         Run = "quests"
    ShoppingRun       Run = "shopping"
    MuleRun           Run = "mule"
    // ... 50+ more
)
```

Supporting types:
- `AvailableRuns map[Run]string` — display names
- `SequencerQuests []LevelingRunInfo` — Act 1–5 quest sequence with mandatory flags
- `SequencerRuns []Run` — ordered runs for the leveling sequencer
- `LevelingRunInfo struct { Run, Act int, IsMandatory bool }`

### Scheduler Config

```go
type Scheduler struct {
    Enabled          bool
    Mode             string  // "timeSlots" or "duration"
    Days             map[Day][]TimeRange
    DurationSchedule DurationSchedule
}

type DurationSchedule struct {
    WakeUpTime   string     // "08:00"
    PlayHours    float64    // e.g. 8.5
    MealBreak    bool
    ShortBreaks  bool
}
```

---

## Run System (`internal/run/`)

### Interfaces

```go
type Run interface {
    Name() string
    Run(parameters *RunParameters) error
    CheckConditions(parameters *RunParameters) SequencerResult
}

type TownRoutineSkipper interface {
    SkipTownRoutines() bool
}

type SequencerResult int
const (
    SequencerSkip  SequencerResult = iota  // skip this run, continue
    SequencerStop                          // stop the sequencer
    SequencerOk                            // run is ready
    SequencerError                         // error, may retry
)
```

### Adding a New Run

1. Create `internal/run/<runname>.go` implementing the `Run` interface:
   ```go
   type MyRun struct { ctx *context.Status }
   func NewMyRun() *MyRun { return &MyRun{ctx: context.Get()} }
   func (r MyRun) Name() string { return string(config.MyRunRun) }
   func (r MyRun) CheckConditions(p *RunParameters) SequencerResult { ... }
   func (r MyRun) Run(p *RunParameters) error { ... }
   ```
2. Add the constant to `internal/config/runs.go`:
   ```go
   MyRunRun Run = "my_run"
   ```
3. Add a `case` to `BuildRun()` in `internal/run/run.go`.
4. If the run requires specific conditions (quests, difficulty), check them in `CheckConditions` and return `SequencerSkip` or `SequencerError` as appropriate.

### Run Interface Contracts

- `CheckConditions` must be side-effect free and cheap (no game interaction).
- `Run` must return a non-nil error only when recovery is possible; if the run is structurally impossible, return a descriptive error.
- Implement `TownRoutineSkipper` (return `SkipTownRoutines() bool`) only if the run explicitly manages its own pre/post town logic.

### Run Directory (78 files)

One file per run. Includes boss runs (andariel, mephisto, diablo, baal), area runs (pit, ancient_tunnels, cows), quest runs (den, rescue_cain, staff, duriel, izual, ancients, hellforge), uber runs (uber_organs, uber_torch, uber_lilith, uber_duriel, uber_izual), leveling (leveling.go, leveling_act1–5.go, leveling_sequence.go), utility (mule, shopping, development, utility, threshsocket, frozen_aura_merc, tristram_early_goldfarm), plus helpers.go and uber_helper.go.

---

## Character System (`internal/character/`, `internal/context/character.go`)

### Character Interface

```go
type Character interface {
    CheckKeyBindings() []skill.ID
    BuffSkills() []skill.ID
    PreCTABuffSkills() []skill.ID
    MainSkillRange() int
    KillCountess() error
    KillAndariel() error
    KillSummoner() error
    KillDuriel() error
    KillCouncil() error
    KillMephisto() error
    KillIzual() error
    KillDiablo() error
    KillPindle() error
    KillNihlathak() error
    KillBaal() error
    KillLilith() error
    KillUberDuriel() error
    KillUberIzual() error
    KillUberMephisto() error
    KillUberDiablo() error
    KillUberBaal() error
    KillMonsterSequence(
        monsterSelector func(d game.Data) (data.UnitID, bool),
        skipOnImmunities []stat.Resist,
    ) error
    ShouldIgnoreMonster(m data.Monster) bool
}
```

### LevelingCharacter Interface

```go
type LevelingCharacter interface {
    Character
    StatPoints() []StatAllocation
    SkillPoints() []skill.ID
    SkillsToBind() (skill.ID, []skill.ID)
    ShouldResetSkills() bool
    GetAdditionalRunewords() []string
    InitialCharacterConfigSetup()
    AdjustCharacterConfig()
    KillAncients() error
}

type StatAllocation struct {
    Stat   stat.ID
    Points int
}
```

### Adding a New Character Class

1. Create `internal/character/<classname>.go` embedding `BaseCharacter` (from `internal/character/core/`).
2. Implement all methods of the `context.Character` interface (all `Kill*` methods, `KillMonsterSequence`, `ShouldIgnoreMonster`, `CheckKeyBindings`, `BuffSkills`, `PreCTABuffSkills`).
3. Register a lowercase string key in `BuildCharacter()` in `internal/character/character.go`.
4. For a leveling character, also implement `context.LevelingCharacter` and register under the leveling branch.

### Registered Character Builds

**Leveling**: AmazonLeveling, AssassinLeveling, BarbLeveling, DruidLeveling, NecromancerLeveling, paladin.NewLeveling, SorceressLeveling, WarlockLeveling

**Endgame**: DevelopmentCharacter, MuleCharacter, Javazon, Trapsin, MosaicSin, Berserker, WarcryBarb, WhirlwindBarb, WindDruid, paladin.NewDefault (hammerdin/foh), paladin.NewDragon, BlizzardSorceress, FireballSorceress, NovaSorceress, HydraOrbSorceress, LightningSorceress

### Character Implementation Conventions

- Every `KillMonsterSequence` loop **must** call `ctx.PauseIfNotPriority()` at the top.
- Use `s.preBattleChecks(id, skipOnImmunities)` before attacking to handle immunities and distance checks.
- Use `step.PrimaryAttack` or `step.SecondaryAttack` with functional `AttackOption`s (`step.Distance(min, max)`, `step.RangedDistance(min, max)`, `step.StationaryDistance(min, max)`, `step.EnsureAura(skill.ID)`).
- Constants for attack loop counts belong in the same file as the character (e.g., `hammerdinMaxAttacksLoop`).

---

## Event System (`internal/event/`)

### Core Types

```go
type Event interface {
    Message() string
    Image() image.Image
    OccurredAt() time.Time
    Supervisor() string
}

type BaseEvent struct {
    message    string
    image      image.Image
    occurredAt time.Time
    supervisor string
}

// Constructors:
func Text(supervisor, message string) BaseEvent
func WithScreenshot(supervisor, message string, img image.Image) BaseEvent
```

### Event Types

| Event | Fields | Constructor |
|---|---|---|
| `UsedPotionEvent` | PotionType, OnMerc | `UsedPotion(be, pt, onMerc)` |
| `GameCreatedEvent` | Name, Password | `GameCreated(be, name, password)` |
| `GameFinishedEvent` | Reason (FinishReason) | `GameFinished(be, reason)` |
| `RunFinishedEvent` | RunName, Reason | `RunFinished(be, runName, reason)` |
| `RunStartedEvent` | RunName | (direct struct init) |
| `ItemStashedEvent` | Item (data.Drop) | `ItemStashed(be, drop)` |
| `ItemBlackListedEvent` | Item (data.Drop) | `ItemBlackListed(be, drop)` |
| `GamePausedEvent` | Paused (bool) | (direct struct init) |
| `CompanionLeaderAttackEvent` | — | (direct struct init) |
| `CompanionRequestedTPEvent` | — | (direct struct init) |
| `InteractedToEvent` | InteractionType | (direct struct init) |
| `RunewordRerollEvent` | — | (direct struct init) |
| `NgrokTunnelEvent` | — | (direct struct init) |
| `RequestCompanionJoinGameEvent` | — | (direct struct init) |
| `ResetCompanionGameInfoEvent` | — | (direct struct init) |
| `MonsterKilledEvent` | — | (direct struct init) |
| `CharacterSwitchEvent` | CurrentCharacter, NextCharacter | `CharacterSwitch(be, cur, next)` |

### Finish Reasons

```go
const (
    FinishedOK          FinishReason = "ok"
    FinishedDied        FinishReason = "death"
    FinishedChicken     FinishReason = "chicken"
    FinishedMercChicken FinishReason = "merc chicken"
    FinishedError       FinishReason = "error"
)
```

### Listener

```go
type Handler func(ctx context.Context, e Event) error

type Listener struct {
    handlers     []Handler
    DropHandlers map[int]Handler
    logger       *slog.Logger
}

func NewListener(logger *slog.Logger) *Listener
func (l *Listener) Register(h Handler)
func (l *Listener) Listen(ctx context.Context) error  // main event loop
func (l *Listener) WaitForEvent(ctx context.Context) Event

func Send(e Event)  // global channel send
```

Events flow through a global `chan Event`. The `Listener.Listen()` loop dispatches to registered handlers and saves screenshots if `config.Koolo.Debug.Screenshots` is enabled.

---

## Game Interaction Layer (`internal/game/`)

### MemoryReader

```go
type MemoryReader struct {
    cfg            *config.CharacterCfg
    *memory.GameReader                     // from d2go
    mapSeed        uint
    HWND           uintptr
    WindowLeftX, WindowTopY int
    GameAreaSizeX, GameAreaSizeY int
    supervisorName string
    cachedMapData  map[area.ID]AreaData
    logger         *slog.Logger
}

func (mr *MemoryReader) FetchMapData() error    // builds collision grids, parallelized
func (mr *MemoryReader) GetData() game.Data     // reads full game state snapshot
func (mr *MemoryReader) GetInventory() data.Items
```

### MemoryInjector

Hooks USER32.dll functions in the game process to simulate input:

```go
type MemoryInjector struct {
    isLoaded              bool
    pid, handle           uintptr
    // Stored original bytes and addresses for:
    // GetCursorPos, TrackMouseEvent, GetKeyState, SetCursorPos
}

func (mi *MemoryInjector) Load() error               // hooks USER32 functions
func (mi *MemoryInjector) Unload()                    // unhooks
func (mi *MemoryInjector) CursorPos(x, y int)         // overrides cursor position
func (mi *MemoryInjector) OverrideGetKeyState(key int) // simulates key press
func (mi *MemoryInjector) EnableCursorOverride()
func (mi *MemoryInjector) DisableCursorOverride()
```

### HID (Human Interface Device)

```go
type HID struct {
    gr *MemoryReader
    gi *MemoryInjector
}
func NewHID(gr *MemoryReader, gi *MemoryInjector) *HID
```

Provides keyboard and mouse input methods. Defined across `hid.go`, `mouse.go`, `keyboard.go`.

### Manager

```go
type Manager struct {
    gr             *MemoryReader
    hid            *HID
    supervisorName string
}

func (m *Manager) ExitGame() error
func (m *Manager) NewGame() error
func (m *Manager) CreateLobbyGame(gameCounter int) (string, string, error)
func (m *Manager) JoinOnlineGame(gameName, password string) error
```

### PacketSender

```go
type PacketSender struct {
    ProcessSender  // interface from d2go
}

func (ps *PacketSender) SendPacket() error
func (ps *PacketSender) PickUpItem(unitID, x, y int) error
func (ps *PacketSender) InteractWithTp(unitID int) error
func (ps *PacketSender) InteractWithEntrance(entranceID int) error
func (ps *PacketSender) Teleport(x, y int) error
func (ps *PacketSender) TelekinesisInteraction(unitID int) error
func (ps *PacketSender) CastSkillAtLocation(x, y int) error
func (ps *PacketSender) SelectRightSkill(skillID skill.ID) error
func (ps *PacketSender) SelectLeftSkill(skillID skill.ID) error
func (ps *PacketSender) LearnSkill(skillID skill.ID) error
func (ps *PacketSender) AllocateStatPoint(statID stat.ID) error
```

### Data (`internal/game/data.go`)

```go
type Data struct {
    data.Data                          // embedded from d2go
    Areas              map[area.ID]AreaData
    AreaData           AreaData
    CharacterCfg       *config.CharacterCfg
    IsLevelingCharacter bool
    ExpChar            uint            // 1=Classic, 2=LoD, 3=DLC
}

func (d Data) IsDLC() bool
func (d Data) CanTeleport() bool       // checks config, gold, area, mana, skill binding
func (d Data) PlayerCastDuration() int
func (d Data) MonsterFilterAnyReachable() bool
func (d Data) HasPotionInInventory(potionType) bool
func (d Data) PotionsInInventory(potionType) int
func (d Data) MissingPotionCountInInventory(potionType) int
func (d Data) ConfiguredInventoryPotionCount(potionType) int
```

---

## Pathfinding (`internal/pather/`)

```go
type PathFinder struct {
    gr           *game.MemoryReader
    data         *game.Data
    hid          *game.HID
    cfg          *config.CharacterCfg
    packetSender *game.PacketSender
    astarBuffers *astar.Buffers
}

func (pf *PathFinder) GetPath(to data.Position) (path.Path, error)
func (pf *PathFinder) GetPathFrom(from, to data.Position) (path.Path, error)
```

Special handling for Arcane Sanctuary, Lut Gholein map bugs, cross-area grid merging, obstacle avoidance (objects, monsters, barricade towers).

Files: `path.go`, `path_finder.go`, `render_map.go`, `utils.go`, `astar/` subdirectory.

---

## Health System (`internal/health/`)

### Health Manager

```go
var ErrDied       = errors.New("death")
var ErrChicken    = errors.New("chicken")
var ErrMercChicken = errors.New("merc chicken")

type Manager struct {
    bm                 *BeltManager
    lastHealingTime    time.Time     // interval: 4s
    lastMercHealTime   time.Time     // interval: 3s
    lastManaTime       time.Time     // interval: 4s
    lastRejuvTime      time.Time     // interval: 1s
}

func (hm *Manager) HandleHealthAndMana() error
func (hm *Manager) ShouldPickStaminaPot() bool
func (hm *Manager) ShouldKeepStaminaPot() bool
func (hm *Manager) IsLowStamina() bool
```

### Belt Manager

```go
type BeltManager struct {
    data       *game.Data
    hid        *game.HID
    logger     *slog.Logger
    supervisor string
}

func (bm *BeltManager) DrinkPotion(potionType, merc bool) error
func (bm *BeltManager) ShouldBuyPotions() bool              // <75% of target
func (bm *BeltManager) GetMissingCount(potionType) int
```

---

## Chicken System (`internal/chicken/`)

```go
func CheckForScaryAuraAndCurse() // panics with health.ErrChicken on configured threats
```

Checks player states for configured curses (`AmplifyDamage`, `Decrepify`, `LowerResist`, `BloodMana`) and nearby monster auras (`Fanaticism`, `Might`, `Conviction`, `HolyFire`, `BlessedAim`, `HolyFreeze`, `HolyShock`) within `RangeForScaryAura = 25`.

Uses `panic(fmt.Errorf("%w: ...", health.ErrChicken))` pattern for stack unwinding.

---

## Drop System (`internal/drop/`)

### Architecture

Four files implement three layers:

1. **`Service`** (`drop_service.go`) — top-level entry point. Holds the `Coordinator`, queued starts, and persistent requests (3min TTL). Provides callbacks for clearing filters, persisting requests, and reporting results.

2. **`Coordinator`** (`drop_coordinator.go`) — orchestrates per-supervisor filters and callbacks. Methods: `SetFilters()`, `ApplyInitialFilters()` (defaults `DropperOnlySelected:true`), `ConfigureCallbacks()`, `ClearIndividualFilters()`.

3. **`Manager`** (`drop_manager.go`) — per-supervisor instance. Manages pending/active drop requests and filter state.

### Key Types

```go
// Sentinel error for interrupting runs when a drop is requested
var ErrInterrupt = errors.New("Drop requested")

type Request struct {
    RoomName  string
    Password  string
    Filters   *Filters
    CreatedAt time.Time
    CardID    string
    CardName  string
}

type Filters struct {
    Enabled             bool
    DropperOnlySelected bool
    SelectedRunes       map[string]ItemQuantity
    SelectedGems        map[string]ItemQuantity
    SelectedKeyTokens   map[string]ItemQuantity
    CustomItems         []string
    AllowedQualities    []string
}
```

### Drop Item Evaluation

`ShouldDropperItem(item)` returns true when:
- Item name matches a selected rune/gem/keyToken with remaining quota, OR
- Item name matches a custom item, OR
- Item quality matches `AllowedQualities` (excludes runes/gems from quality-only matching)

`HasRemainingDropQuota()` checks if any configured quota has remaining inventory.

---

## Stash System (`internal/action/stash.go`)

Key constants:
```go
const maxGoldPerStashTab = 2500000
const maxGoldPerSharedStash = 7500000
// DLC stash tabs:
const StashTabGems = 100
const StashTabMaterials = 101
const StashTabRunes = 102
```

---

## Item Pickup (`internal/action/item_pickup.go`)

`ItemPickup(maxDistance)` — complex multi-attempt loop with:
- 5 base attempts + 5 "too far" attempts per item
- Inventory fit checking via 2D grid scan
- Town trip management when inventory is full (stash/sell)
- Item blacklisting on repeated failures
- NIP rule evaluation for pick decisions

---

## Auto-Equipment (`internal/action/autoequip.go`)

`AutoEquip()` — evaluates and equips items for player and mercenary in a loop until stable (max 30 iterations). Uses tier-based scoring system defined in `autoequip_tiers.go` and `autoequip_meta_item_score.go`.

---

## Runeword System (`internal/action/runeword_maker.go`)

`MakeRunewords()` — iterates enabled recipes, finds bases and inserts runes/gems via `SocketItems()`. Supports:
- DLC RunesTab stash navigation
- `AutoUpgrade` and `OnlyIfWearable` flags
- `AutoTierByDifficulty` for automatic base selection
- Reroll rules for re-rolling suboptimal runewords

---

## Cube Recipes (`internal/action/cube_recipes.go`)

```go
type CubeRecipe struct {
    Name           string
    Items          []string
    PurchaseRequired bool
    PurchaseItems  []string
}

var Recipes []CubeRecipe  // ~35 recipes: perfect gems (7), Token of Absolution, rune upgrades (El→Cham), socket adding
```

---

## Gambling (`internal/action/gambling.go`)

`Gamble()` — triggers when stashed gold ≥ 2.48M. Visits gambling NPC per act.
`GambleSingleItem(items, desiredQuality)` — targeted gambling for specific quality.

Max 5 purchases per item type per session. Supports coronet/circlet grouping and NIP rule evaluation.

---

## Shopping (`internal/action/shopping_action.go`)

```go
type ActionShoppingPlan struct {
    Enabled         bool
    RefreshesPerRun int
    MinGoldReserve  int
    Vendors         []npc.ID
    Rules           nip.Rules
    Types           []string
}

func RunShopping(plan ActionShoppingPlan) error
```

Multi-pass shopping across towns/vendors with drop interrupt checks and inventory space validation (`ensureTwoFreeColumnsStrict()`).

---

## Mule System (`internal/mule/`)

```go
type Manager struct {
    logger *slog.Logger
}

func (m *Manager) ShouldMule(stashFull bool, characterName string) (bool, string)
func (m *Manager) IsMuleCharacter(characterName string) bool
```

Checks `MuleProfiles` list, returns first matching mule profile name. `IsMuleCharacter` returns true when `Muling.Enabled && ReturnTo != ""`.

---

## Town System (`internal/town/`)

```go
type Town interface {
    RefillNPC() npc.ID
    HealNPC() npc.ID
    RepairNPC() npc.ID
    MercContractorNPC() npc.ID
    GamblingNPC() npc.ID
    IdentifyNPC() npc.ID
    TPWaitingArea() data.Position
    TownArea() area.ID
}

func GetTownByArea(a area.ID) Town  // returns A1{}, A2{}, A3{}, A4{}, or A5{}
```

Files: `A1.go`–`A5.go`, `shop_manager.go`, `town.go`.

---

## Terror Zone System (`internal/terrorzone/`)

```go
type Tier int
const (
    TierS Tier = iota
    TierA
    TierB
    TierC
    TierD
    TierF
)

type ZoneInfo struct {
    Act        int
    ExpTier    Tier
    LootTier   Tier
    BossPack   bool
    Immunities []stat.Resist
    Group      string
}

func Info(id area.ID) ZoneInfo
func ExpTierOf(id area.ID) Tier
func LootTierOf(id area.ID) Tier
func Zones() map[area.ID]ZoneInfo    // ~35 entries
func Groups() map[string][]area.ID
```

---

## Scheduler (`internal/bot/scheduler.go`)

Two scheduling modes:

1. **`timeSlots`** — day-of-week based. Deterministic variance offsets. Starts/stops supervisors based on configured time ranges.

2. **`duration`** — human-like play patterns. State machine with phases:
   ```go
   const (
       PhaseResting  = "resting"
       PhasePlaying  = "playing"
       PhaseOnBreak  = "on_break"
   )
   ```
   Supports wake time, play hours, meal breaks, short breaks, jitter.

---

## Companion System (`internal/bot/companion.go`)

Handles `RequestCompanionJoinGameEvent` (stores game name/password for follower) and `ResetCompanionGameInfoEvent` (clears game info). Leader creates games; followers join via stored game info.

---

## Armory (`internal/bot/armory.go`)

```go
type ArmoryItem struct {
    ID, Name, Quality string
    Ethereal, IsRuneword bool
    RunewordName string
    Stats, BaseStats map[string]interface{}
    Sockets int
    ImageName string
    Defense, Damage, Durability interface{}
}

type ArmoryCharacter struct {
    CharacterName, Class string
    Level int
    Experience uint64
    Gold, StashedGold int
    Equipped, Inventory, Stash []ArmoryItem
    SharedStash1, SharedStash2, SharedStash3 []ArmoryItem
    SharedStash4, SharedStash5, SharedStash6 []ArmoryItem
    GemsTab, MaterialsTab, RunesTab []ArmoryItem  // DLC stash tabs
    Cube, Belt, Mercenary []ArmoryItem
}
```

---

## Pickit System (`internal/pickit/`)

### Types (`types.go`)

Rich type system for the visual pickit editor:

```go
type ItemDefinition struct {
    ID, Name, NIPName, InternalName, Type, BaseItem string
    Quality        []item.Quality
    AvailableStats []StatType
    MaxSockets     int
    Ethereal       bool
    ItemLevel      int
    Category, Rarity, Description string
}

type StatType struct {
    ID, Name, NipProperty string
    MinValue, MaxValue    float64
    IsPercent             bool
}

type PickitRule struct {
    ID, ItemName, ItemID, FileName string
    Enabled                        bool
    Priority                       int
    LeftConditions, RightConditions []Condition
    MaxQuantity                    int
    IsScored                       bool
    ScoreThreshold                 float64
    ScoreWeights                   map[string]float64
    Comments, GeneratedNIP         string
}

type Condition struct {
    Property, Operator string
    Value              interface{}
    NipSyntax          string
}

// Also: PickitFile, PickitSet, RuleTemplate, ValidationResult,
// SimulationResult, ItemMatch, StatPreset, ConflictDetection,
// EditorPreferences, ExportOptions, ImportOptions, SearchFilters,
// AutoSuggestion
```

Files: `item_database.go`, `item_database_v2.go`, `nip_builder.go`, `stats.go`, `templates.go`, `types.go`.

---

## Packet System (`internal/packet/`)

10 files providing packet-based alternatives to HID interaction:

| File | Purpose |
|---|---|
| `allocate_stat.go` | Stat point allocation |
| `cast_skill_entity_left.go` | Left-click skill on entity |
| `cast_skill_entity_right.go` | Right-click skill on entity |
| `cast_skill_location.go` | Skill at map position |
| `entrance_interaction.go` | Area entrance interaction |
| `learn_skill.go` | Skill learning |
| `object_interaction.go` | Object interaction |
| `pickup_item.go` | Item pickup |
| `skill_selection.go` | Skill selection |
| `tp_interaction.go` | Town Portal interaction |

---

## HTTP Server (`internal/server/http_server.go`)

4285-line file providing the web overlay UI, REST API, and WebSocket status broadcast.

### Server Struct

```go
type HttpServer struct {
    logger                  *slog.Logger
    server                  *http.Server
    manager                 *SupervisorManager
    scheduler               *Scheduler
    templates               *template.Template
    wsServer                *WebSocketServer
    pickitAPI               *PickitAPI
    sequenceAPI             *SequenceAPI
    updater                 *Updater
    DropHistory, RunewordHistory   interface{}
    DropFilters, DropCardInfo      interface{}
    cachedAnalyticsManagers        interface{}
}
```

### WebSocket

```go
type WebSocketServer struct {
    clients    map[*websocket.Conn]bool
    broadcast  chan []byte
    register   chan *websocket.Conn
    unregister chan *websocket.Conn
}
```

`BroadcastStatus()` sends JSON status every 1 second to all connected WebSocket clients.

### Embedded Assets

```go
//go:embed all:assets
var assetsFS embed.FS

//go:embed all:templates
var templatesFS embed.FS
```

### HTTP Routes

#### Main Pages
- `GET /` — dashboard
- `GET /config` — configuration page
- `GET /supervisorSettings` — supervisor settings
- `GET /runewords` — runeword manager
- `GET /debug` — debug page
- `GET /debug-data` — debug data

#### Bot Control
- `POST /start` — start supervisor
- `POST /stop` — stop supervisor
- `POST /togglePause` — toggle pause
- `POST /autostart/toggle` — toggle autostart
- `POST /autostart/run-once` — run once

#### Drops
- `GET /drops` — drops page
- `GET /all-drops` — all drops view
- `GET /export-drops` — export drop logs
- `POST /open-droplogs` — open drop log directory
- `POST /reset-droplogs` — reset drop logs

#### Process Management
- `GET /process-list` — list game processes
- `POST /attach-process` — attach to existing process

#### WebSocket & Live Data
- `GET /ws` — WebSocket endpoint
- `GET /initial-data` — initial dashboard data

#### API
- `POST /api/reload-config` — reload configuration
- `POST /api/companion-join` — trigger companion join
- `POST /api/generate-battlenet-token` — generate auth token
- `POST /reset-muling` — reset muling state

#### Runeword API
- `GET /api/runewords/rolls` — runeword roll data
- `GET /api/runewords/base-types` — available base types
- `GET /api/runewords/bases` — base items
- `GET /api/runewords/history` — runeword history

#### Updater API
- `GET /api/updater/version` — current version
- `GET /api/updater/check` — check for updates
- `GET /api/updater/current-commits` — current commits
- `POST /api/updater/update` — perform update
- `GET /api/updater/status` — update status
- `GET /api/updater/backups` — list backups
- `POST /api/updater/rollback` — rollback to backup
- `GET /api/updater/prs` — list PRs
- `POST /api/updater/cherry-pick` — cherry-pick PR
- `POST /api/updater/prs/revert` — revert cherry-picked PR

#### Pickit Editor
- `GET /pickit-editor` — pickit editor page
- `GET /api/pickit/items` — item definitions
- `GET /api/pickit/items/search` — search items
- `GET /api/pickit/items/categories` — item categories
- `GET /api/pickit/stats` — available stats
- `GET /api/pickit/templates` — rule templates
- `GET /api/pickit/presets` — stat presets
- `CRUD /api/pickit/rules` — create/read/update/delete rules + validate
- `GET /api/pickit/files` — list/import/export pickit files
- `POST /api/pickit/browse-folder` — browse file system
- `POST /api/pickit/simulate` — simulate rule matching

#### Sequence Editor
- `GET /sequence-editor` — sequence editor page
- `GET /api/sequence-editor/runs` — available runs
- `GET /api/sequence-editor/file` — read sequence file
- `POST /api/sequence-editor/open` — open sequence
- `POST /api/sequence-editor/save` — save sequence
- `DELETE /api/sequence-editor/delete` — delete sequence
- `GET /api/sequence-editor/files` — list sequence files

#### Armory
- `GET /armory` — armory page
- `GET /api/armory` — character equipment data
- `GET /api/armory/characters` — character list
- `GET /api/armory/all` — all characters equipment

#### Analytics
- `GET /analytics` — analytics page
- `GET /api/analytics/session` — session analytics
- `GET /api/analytics/global` — global analytics
- `GET /api/analytics/runs` — run analytics
- `GET /api/analytics/hourly` — hourly breakdown
- `GET /api/analytics/run-types` — run type stats
- `GET /api/analytics/items` — item analytics
- `GET /api/analytics/characters` — character analytics
- `POST /api/analytics/reset` — reset analytics
- `GET /api/analytics/deaths` — death analytics
- `GET /api/analytics/runes` — rune drop analytics
- `GET /api/analytics/session-history` — session history
- `GET /api/analytics/comparison` — comparison analytics

#### Drop Manager
- `GET /Drop-manager` — drop manager page
- (Additional drop routes registered via `s.registerDropRoutes()`)

#### Other
- `GET /api/skill-options` — available skill options
- `POST /api/supervisors/bulk-apply` — bulk apply settings
- `GET /api/scheduler-history` — scheduler history
- Static: `/assets/*`, `/items/*`

---

## Concurrency & Safety

- `context.botContexts` is a `map[uint64]*Status` protected by `var mu sync.Mutex`. Each goroutine attaches with `ctx.AttachRoutine(priority)` and detaches with `ctx.Detach()`.
- `CurrentGameHelper.mutex` (`sync.Mutex`) guards `IsPickingItems`; use `ctx.SetPickingItems(bool)`.
- `ctx.IsAllocatingStatsOrSkills` is an `atomic.Bool` used to suppress stuck detection during allocation; set and clear it around stat/skill allocation blocks.
- The `lastActivityTimeMux` in `Bot` guards activity tracking; use the provided accessors.
- Never add `sync.Mutex` fields directly to `Context` — route shared state through `CurrentGameHelper` or dedicated managers.
- The `Bot.Run()` uses `errgroup` to manage 4 concurrent goroutines with shared cancellation.
- Event channel (`var events = make(chan Event)`) is unbuffered — `Send()` blocks until `Listen()` consumes.
- Config access is guarded by `cfgMux` (`sync.RWMutex`).

---

## Error Handling Patterns

### Chicken Panic Pattern

Health emergencies use `panic` for stack unwinding:
```go
panic(fmt.Errorf("%w: scary aura detected", health.ErrChicken))
```

Recovered at the `Bot.Run()` level:
```go
defer func() {
    if r := recover(); r != nil {
        if errors.Is(err, health.ErrChicken) { ... }
    }
}()
```

### Sentinel Errors

| Error | Package | Usage |
|---|---|---|
| `health.ErrDied` | health | Player died |
| `health.ErrChicken` | health | Health chicken triggered |
| `health.ErrMercChicken` | health | Merc chicken triggered |
| `drop.ErrInterrupt` | drop | Drop requested, interrupt current run |
| `step.ErrMonstersInPath` | step | Monsters blocking movement |
| `step.ErrPlayerStuck` | step | Player cannot move |
| `step.ErrPlayerRoundTrip` | step | Player detected looping back |
| `step.ErrNoPath` | step | No valid path found |

---

## File and Naming Conventions

- **Package names**: all lowercase, single word matching the directory name.
- **Character class strings** in config are lowercase (e.g., `"hammerdin"`, `"sorceress"`, `"trapsin"`).
- **Run name constants** follow `XxxRun Run = "snake_case"` pattern in `config/runs.go`.
- **File names**: lowercase with underscores. Exception: `Inventory.go` exists with a non-standard name — preserve it as-is; do **not** rename without a deliberate refactor task.
- **Constructor functions**: `NewXxx()` returns a pointer or value depending on whether the type uses pointer receivers.
- **Action functions**: top-level unexported or exported functions in `internal/action/`, not methods. They call `context.Get()` internally.
- **Step functions**: exported functions in `internal/action/step/`, return `error`. Accept configuration via functional options (`AttackOption`, etc.).

---

## Build & Tooling

- Build with `better_build.bat` or `build.bat` on Windows.
- The entry point is `cmd/koolo/main.go`.
- Windows resource (manifest, icon) is embedded via `cmd/koolo/rsrc_windows_amd64.syso`.
- Log output goes to a directory configured by `config.Koolo.LogSaveDirectory`.
- `go vet ./...` and `gofmt -l .` must produce no output before committing.
