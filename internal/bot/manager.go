package bot

import (
	"fmt"
	"log/slog"
	"strconv"
	"sync"
	"syscall"
	"time"
	"unsafe"

	"github.com/hectorgimenez/koolo/cmd/koolo/log"
	"github.com/hectorgimenez/koolo/internal/character"
	"github.com/hectorgimenez/koolo/internal/config"
	ct "github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/event"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/health"
	"github.com/hectorgimenez/koolo/internal/mule"
	"github.com/hectorgimenez/koolo/internal/pather"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/hectorgimenez/koolo/internal/utils/winproc"
	"github.com/lxn/win"
)

type SupervisorManager struct {
	logger         *slog.Logger
	supervisors    map[string]Supervisor
	crashDetectors map[string]*game.CrashDetector
	eventListener  *event.Listener
	startMux       sync.Mutex
	startQueue     []string
	isStarting     bool
}

type startParams struct {
	supervisorName   string
	attachToExisting bool
	pidHwnd          []uint32
}

func NewSupervisorManager(logger *slog.Logger, eventListener *event.Listener) *SupervisorManager {

	return &SupervisorManager{
		logger:         logger,
		supervisors:    make(map[string]Supervisor),
		crashDetectors: make(map[string]*game.CrashDetector),
		eventListener:  eventListener,
		startQueue:     make([]string, 0),
	}
}

func (mng *SupervisorManager) AvailableSupervisors() []string {
	availableSupervisors := make([]string, 0)
	for name := range config.GetCharacters() {
		if name != "template" {
			availableSupervisors = append(availableSupervisors, name)
		}
	}

	return availableSupervisors
}

// Start handles the queuing logic.
func (mng *SupervisorManager) Start(supervisorName string, attachToExisting bool, pidHwnd ...uint32) error {
	mng.startMux.Lock()
	defer mng.startMux.Unlock()

	// 1. Check if the bot is already running (original check)
	if _, exists := mng.supervisors[supervisorName]; exists {
		return fmt.Errorf("supervisor %s is already running", supervisorName)
	}

	// 2. Check if a bot is currently in the starting phase (the global lock)
	if mng.isStarting {
		// A bot is currently in the startup phase. Queue the request.
		for _, name := range mng.startQueue {
			if name == supervisorName {
				mng.logger.Debug(fmt.Sprintf("Supervisor %s already in queue. Ignoring duplicate click.", supervisorName))
				return nil
			}
		}

		mng.startQueue = append(mng.startQueue, supervisorName)
		mng.logger.Info(fmt.Sprintf("Supervisor %s queued. Queue size: %d", supervisorName, len(mng.startQueue)))
		return nil
	}

	// 3. No bot is starting. Start immediately.
	mng.isStarting = true // Acquire the starting lock

	// Start the actual execution in a goroutine
	go mng.handleStartExecution(startParams{
		supervisorName:   supervisorName,
		attachToExisting: attachToExisting,
		pidHwnd:          pidHwnd,
	})

	return nil
}

// handleStartExecution manages the startup of a bot until it is InGame, and then manages the queue state.
func (mng *SupervisorManager) handleStartExecution(params startParams) {
	// 1. Channel signals whether the supervisor reached the InGame state (true) or failed (false)
	gameStartedCh := make(chan bool, 1)

	// Run the bot's lifecycle in a separate goroutine
	go mng.runSupervisorLifecycle(params, gameStartedCh)

	// Wait for the in-game signal. This signifies the end of the "starting" phase.
	successfullyStarted := <-gameStartedCh

	if successfullyStarted {
		mng.logger.Info(fmt.Sprintf("Supervisor %s reached in-game state. Checking queue.", params.supervisorName))
	} else {
		mng.logger.Warn(fmt.Sprintf("Supervisor %s failed to start. Checking queue.", params.supervisorName))
	}

	// CRITICAL FIX: Atomic queue check and start lock management.
	mng.startMux.Lock()

	if len(mng.startQueue) == 0 {
		// Queue is empty. Release the global start lock completely.
		mng.isStarting = false
		mng.startMux.Unlock()
		mng.logger.Debug("Queue empty. Global start lock released.")
		return
	}

	// Queue is NOT empty. Dequeue the next bot and keep the isStarting lock set to true.
	nextSupervisorName := mng.startQueue[0]
	mng.startQueue = mng.startQueue[1:] // Dequeue

	// Unlock the main mutex *before* launching the next goroutine.
	mng.startMux.Unlock()

	mng.logger.Info(fmt.Sprintf("Starting next bot from queue: %s", nextSupervisorName))

	// Launch the next queued bot (this starts the next lifecycle).
	go mng.handleStartExecution(startParams{
		supervisorName:   nextSupervisorName,
		attachToExisting: false,
		pidHwnd:          nil,
	})
}

func (mng *SupervisorManager) ReloadConfig() error {

	// Load fresh configs
	if err := config.Load(); err != nil {
		return err
	}

	// Apply new configs to running supervisors
	for name, sup := range mng.supervisors {
		newCfg, exists := config.GetCharacter(name)
		if !exists {
			continue
		}

		ctx := sup.GetContext()
		if ctx == nil {
			continue
		}

		// Update the config
		*ctx.CharacterCfg = *newCfg
	}

	return nil
}

func (mng *SupervisorManager) runSupervisorLifecycle(params startParams, gameStartedCh chan<- bool) {
	// Reload config for the latest changes before execution
	err := config.Load()
	if err != nil {
		mng.logger.Error(fmt.Sprintf("error loading config for %s: %v", params.supervisorName, err))
		gameStartedCh <- false // Signal failure to the manager
		return
	}

	supervisorLogger, err := log.NewLogger(config.Koolo.Debug.Log, config.Koolo.LogSaveDirectory, params.supervisorName)
	if err != nil {
		gameStartedCh <- false
		return
	}

	var optionalPID uint32
	var optionalHWND win.HWND

	if params.attachToExisting {
		if len(params.pidHwnd) == 2 {
			mng.logger.Info("Attaching to existing game", "pid", params.pidHwnd[0], "hwnd", params.pidHwnd[1])
			optionalPID = params.pidHwnd[0]
			optionalHWND = win.HWND(params.pidHwnd[1])
		} else {
			mng.logger.Error(fmt.Sprintf("pid and hwnd are required when attaching to an existing game for %s", params.supervisorName))
			gameStartedCh <- false
			return
		}
	}

	supervisor, crashDetector, err := mng.buildSupervisor(params.supervisorName, supervisorLogger, params.attachToExisting, optionalPID, optionalHWND)
	if err != nil {
		mng.logger.Error(fmt.Sprintf("error building supervisor %s: %s", params.supervisorName, err.Error()))
		gameStartedCh <- false
		return
	}

	// Add the supervisor to the map
	mng.startMux.Lock()
	if oldCrashDetector, exists := mng.crashDetectors[params.supervisorName]; exists {
		oldCrashDetector.Stop()
	}
	mng.supervisors[params.supervisorName] = supervisor
	mng.crashDetectors[params.supervisorName] = crashDetector
	mng.startMux.Unlock()

	if config.Koolo.GameWindowArrangement {
		go func() {
			time.Sleep(time.Second * 5)
			mng.rearrangeWindows()
		}()
	}

	go crashDetector.Start()

	// This blocks for the bot's entire lifecycle
	err = supervisor.Start(gameStartedCh)
	if err != nil {
		mng.logger.Error(fmt.Sprintf("error running supervisor %s: %s", params.supervisorName, err.Error()))
	}

	// Bot has finished its lifecycle. Clean up.
	mng.logger.Info(fmt.Sprintf("Supervisor %s has finished its lifecycle. Cleaning up and checking exit reason.", params.supervisorName))

	// Call the function that cleans up and checks the exit condition.
	mng.finishSupervisorLifecycle(params.supervisorName)
}

// finishSupervisorLifecycle handles cleanup and prevents unwanted restarts.
func (mng *SupervisorManager) finishSupervisorLifecycle(supervisorName string) {
	mng.startMux.Lock()
	defer mng.startMux.Unlock()

	s, found := mng.supervisors[supervisorName]

	// 1. Stop crash detector and delete from maps (standard cleanup)
	if cd, ok := mng.crashDetectors[supervisorName]; ok {
		cd.Stop()
		delete(mng.crashDetectors, supervisorName)
	}

	// 2. Deleting from the supervisor map.
	if found {
		delete(mng.supervisors, supervisorName)
	}

	// 3. Crucial check: If the bot was manually stopped by the user, the context priority will be PriorityStop.
	if found && s.GetContext().ExecutionPriority == ct.PriorityStop {
		mng.logger.Info(fmt.Sprintf("Supervisor %s was intentionally stopped. Cleanup complete.", supervisorName))
		return // Exit early, preventing crash detector's restartFunc from being called/restarting bot logic.
	}

	// If we reach here, the bot crashed or naturally finished runs.
	mng.logger.Info(fmt.Sprintf("Supervisor %s finished run/crashed. Cleanup complete.", supervisorName))
}

// Stop is simplified to be fast and non-blocking, setting the priority flag first.
func (mng *SupervisorManager) Stop(supervisorName string) {
	mng.startMux.Lock()
	s, found := mng.supervisors[supervisorName]

	if found {
		// CRITICAL: Set the priority while holding the lock to ensure a crash/stop race doesn't miss the flag.
		s.GetContext().SwitchPriority(ct.PriorityStop)
	}
	mng.startMux.Unlock() // Release the lock immediately to prevent deadlocks

	if found {
		// Calling s.Stop() breaks the loop in single_supervisor.go, causing
		// supervisor.Start() in runSupervisorLifecycle to return, which then
		// triggers the final finishSupervisorLifecycle cleanup.
		s.Stop()
		mng.logger.Info(fmt.Sprintf("Sent stop signal to supervisor %s. Cleanup initiated.", supervisorName))
	} else {
		mng.logger.Warn(fmt.Sprintf("Stop failed: Supervisor %s not found or already stopped.", supervisorName))
	}
}

func (mng *SupervisorManager) StopAll() {
	for _, s := range mng.supervisors {
		// Set priority first, then stop
		s.GetContext().SwitchPriority(ct.PriorityStop)
		s.Stop()
	}
}

func (mng *SupervisorManager) TogglePause(supervisor string) {
	s, found := mng.supervisors[supervisor]
	if found {
		s.TogglePause()
	}
}

func (mng *SupervisorManager) Status(characterName string) Stats {
	for name, supervisor := range mng.supervisors {
		if name == characterName {
			return supervisor.Stats()
		}
	}

	return Stats{}
}

func (mng *SupervisorManager) GetData(characterName string) *game.Data {
	for name, supervisor := range mng.supervisors {
		if name == characterName {
			return supervisor.GetData()
		}
	}

	return nil
}

func (mng *SupervisorManager) GetContext(characterName string) *ct.Context {
	for name, supervisor := range mng.supervisors {
		if name == characterName {
			return supervisor.GetContext()
		}
	}

	return nil
}

func (mng *SupervisorManager) buildSupervisor(supervisorName string, logger *slog.Logger, attach bool, optionalPID uint32, optionalHWND win.HWND) (Supervisor, *game.CrashDetector, error) {
	cfg, found := config.GetCharacter(supervisorName)
	if !found {
		return nil, nil, fmt.Errorf("character %s not found", supervisorName)
	}

	var pid uint32
	var hwnd win.HWND

	if attach {
		if optionalPID != 0 && optionalHWND != 0 {
			pid = optionalPID
			hwnd = optionalHWND
		} else {
			return nil, nil, fmt.Errorf("pid and hwnd are required when attaching to an existing game")
		}
	} else {
		var err error
		pid, hwnd, err = game.StartGame(cfg.Username, cfg.Password, cfg.AuthMethod, cfg.AuthToken, cfg.Realm, cfg.CommandLineArgs, config.Koolo.UseCustomSettings)
		if err != nil {
			return nil, nil, fmt.Errorf("error starting game: %w", err)
		}
	}

	gr, err := game.NewGameReader(cfg, supervisorName, pid, hwnd, logger)
	if err != nil {
		return nil, nil, fmt.Errorf("error creating game reader: %w", err)
	}

	gi, err := game.InjectorInit(logger, gr.GetPID())
	if err != nil {
		return nil, nil, fmt.Errorf("error creating game injector: %w", err)
	}

	ctx := ct.NewContext(supervisorName)

	hidM := game.NewHID(gr, gi)
	pf := pather.NewPathFinder(gr, ctx.Data, hidM, cfg)

	bm := health.NewBeltManager(ctx.Data, hidM, logger, supervisorName)
	hm := health.NewHealthManager(bm, ctx.Data)

	ctx.CharacterCfg = cfg
	ctx.EventListener = mng.eventListener
	ctx.HID = hidM
	ctx.PacketSender = game.NewPacketSender(gr.Process)
	ctx.Logger = logger
	ctx.Manager = game.NewGameManager(gr, hidM, supervisorName)
	ctx.GameReader = gr
	ctx.MemoryInjector = gi
	ctx.PathFinder = pf
	ctx.BeltManager = bm
	ctx.HealthManager = hm
	char, err := character.BuildCharacter(ctx.Context)
	if err != nil {
		return nil, nil, fmt.Errorf("error creating character: %w", err)
	}
	ctx.Char = char

	muleManager := mule.NewManager(logger)
	bot := NewBot(ctx.Context, muleManager)

	statsHandler := NewStatsHandler(supervisorName, logger)
	mng.eventListener.Register(statsHandler.Handle)
	supervisor, err := NewSinglePlayerSupervisor(supervisorName, bot, statsHandler)

	if err != nil {
		return nil, nil, err
	}

	supervisor.GetContext().StopSupervisorFn = supervisor.Stop

	// This function will be used to restart the client - passed to the crashDetector
	restartFunc := func() {

		ctx := supervisor.GetContext()

		// CRITICAL FIX: Abort restart if the bot was manually stopped.
		if ctx.ExecutionPriority == ct.PriorityStop {
			mng.logger.Info("Restart aborted: Supervisor was already intentionally stopped by user (PriorityStop).", slog.String("supervisor", supervisorName))
			return
		}

		if ctx.CleanStopRequested {
			if ctx.RestartWithCharacter != "" {
				mng.logger.Info("Supervisor requested restart with different character",
					slog.String("from", supervisorName),
					slog.String("to", ctx.RestartWithCharacter))
				nextCharacter := ctx.RestartWithCharacter
				mng.Stop(supervisorName)
				time.Sleep(5 * time.Second) // Wait before starting new character
				if err := mng.Start(nextCharacter, false); err != nil {
					mng.logger.Error("Failed to start next character",
						slog.String("character", nextCharacter),
						slog.String("error", err.Error()))
				}
				return
			}
			mng.logger.Info("Supervisor stopped cleanly by game logic. Preventing restart.", slog.String("supervisor", supervisorName))
			mng.Stop(supervisorName)
			return
		}

		mng.logger.Info("Restarting supervisor after crash", slog.String("supervisor", supervisorName))

		// Ensure the crashing client is fully killed and removed from maps (sets PriorityStop, triggers cleanup)
		mng.Stop(supervisorName)
		time.Sleep(5 * time.Second) // Wait a bit before restarting

		// Get a list of all available Supervisors
		supervisorList := mng.AvailableSupervisors()

		for {

			// Set the default state
			tokenAuthStarting := false

			// Get the current supervisor's config
			supCfg, _ := config.GetCharacter(supervisorName)

			for _, sup := range supervisorList {

				// If the current don't check against the one we're trying to launch
				if sup == supervisorName {
					continue
				}

				if mng.GetSupervisorStats(sup).SupervisorStatus == Starting {
					if supCfg.AuthMethod == "TokenAuth" {
						tokenAuthStarting = true
						mng.logger.Info("Waiting before restart as another client is already starting and we're using token auth", slog.String("supervisor", sup))
						break
					}

					sCfg, found := config.GetCharacter(sup)
					if found {
						if sCfg.AuthMethod == "TokenAuth" {
							// A client that uses token auth is currently starting, hold off restart
							tokenAuthStarting = true
							mng.logger.Info("Waiting before restart as a client that's using token auth is already starting", slog.String("supervisor", sup))
							break
						}
					}
				}
			}

			if !tokenAuthStarting {
				break
			}

			// Wait 5 seconds before checking again
			utils.Sleep(5000)
		}

		gameTitle := "D2R - [" + strconv.FormatInt(int64(pid), 10) + "] - " + supervisorName + " - " + cfg.Realm
		winproc.SetWindowText.Call(uintptr(hwnd), uintptr(unsafe.Pointer(syscall.StringToUTF16Ptr(gameTitle))))

		err := mng.Start(supervisorName, false)
		if err != nil {
			mng.logger.Error("Failed to restart supervisor", slog.String("supervisor", supervisorName), slog.String("Error: ", err.Error()))
		}
	}

	gameTitle := "D2R - [" + strconv.FormatInt(int64(pid), 10) + "] - " + supervisorName + " - " + cfg.Realm
	winproc.SetWindowText.Call(uintptr(hwnd), uintptr(unsafe.Pointer(syscall.StringToUTF16Ptr(gameTitle))))
	crashDetector := game.NewCrashDetector(supervisorName, int32(pid), uintptr(hwnd), mng.logger, restartFunc)

	return supervisor, crashDetector, nil
}

func (mng *SupervisorManager) GetSupervisorStats(supervisor string) Stats {
	if mng.supervisors[supervisor] == nil {
		return Stats{}
	}
	return mng.supervisors[supervisor].Stats()
}

func (mng *SupervisorManager) rearrangeWindows() {
	width := win.GetSystemMetrics(0)
	height := win.GetSystemMetrics(1)
	var windowBorderX int32 = 2
	var windowBorderY int32 = 40
	var windowOffsetX int32 = -10
	maxColumns := width / (1280 + windowBorderX)
	maxRows := height / (720 + windowBorderY)

	mng.logger.Debug(
		"Arranging windows",
		slog.String("displaywidth", strconv.FormatInt(int64(width), 10)),
		slog.String("displayheight", strconv.FormatInt(int64(height), 10)),
		slog.String("max columns", strconv.FormatInt(int64(maxColumns+1), 10)),
		slog.String("max rows", strconv.FormatInt(int64(maxRows+1), 10)),
	)

	var column, row int32
	for _, sp := range mng.supervisors {
		if column > maxColumns {
			column = 0
			row++
		}

		if row <= maxRows {
			sp.SetWindowPosition(int(column*(1280+windowBorderX)+windowOffsetX), int(row*(720+windowBorderY)))
			mng.logger.Debug(
				"Window Positions",
				slog.String("supervisor", sp.Name()),
				slog.String("column", strconv.FormatInt(int64(column), 10)),
				slog.String("row", strconv.FormatInt(int64(row), 10)),
				slog.String("position", strconv.FormatInt(int64(column*(1280+windowBorderX)+windowOffsetX), 10)+"x"+strconv.FormatInt(int64(row*(720+windowBorderY)), 10)),
			)
			column++
		} else {
			mng.logger.Debug("Window position of supervisor " + sp.Name() + " was not changed, no free space for it")
		}
	}
}
