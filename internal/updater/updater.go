package updater

import (
	"fmt"
	"log/slog"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"time"

	"github.com/google/uuid"
)

type UpdaterStatus struct {
	State       string // "idle", "checking", "updating", "building", "rollback", "cherry-pick", "done", "error"
	Progress    int    // 0-100
	CurrentStep string
	Logs        []string
	Error       string
}

type Updater struct {
	logger         *slog.Logger
	status         UpdaterStatus
	statusMux      sync.RWMutex
	logCallback    func(message string)
	logCallbackMux sync.RWMutex
	opMux          sync.Mutex
	opRunning      bool
	opName         string
}

func NewUpdater(logger *slog.Logger) *Updater {
	return &Updater{
		logger: logger,
		status: UpdaterStatus{
			State:    "idle",
			Progress: 0,
			Logs:     []string{},
		},
	}
}

// GetStatus returns the current updater status (thread-safe)
func (u *Updater) GetStatus() UpdaterStatus {
	u.statusMux.RLock()
	status := u.status
	if len(status.Logs) > 0 {
		logsCopy := make([]string, len(status.Logs))
		copy(logsCopy, status.Logs)
		status.Logs = logsCopy
	}
	u.statusMux.RUnlock()
	return status
}

// SetLogCallback sets a callback function to receive log messages
func (u *Updater) SetLogCallback(callback func(message string)) {
	u.logCallbackMux.Lock()
	u.logCallback = callback
	u.logCallbackMux.Unlock()
}

func (u *Updater) log(message string) {
	u.statusMux.Lock()
	u.status.Logs = append(u.status.Logs, message)
	// Keep only last 100 log lines
	if len(u.status.Logs) > 100 {
		u.status.Logs = u.status.Logs[len(u.status.Logs)-100:]
	}
	u.statusMux.Unlock()

	u.logger.Info(message)
	u.logCallbackMux.RLock()
	callback := u.logCallback
	u.logCallbackMux.RUnlock()
	if callback != nil {
		callback(message)
	}
}

func (u *Updater) updateProgress(state string, progress int, step string) {
	u.statusMux.Lock()
	u.status.State = state
	u.status.Progress = progress
	u.status.CurrentStep = step
	u.statusMux.Unlock()
	u.log(step)
}

func (u *Updater) setError(err error) {
	u.statusMux.Lock()
	u.status.State = "error"
	u.status.Error = err.Error()
	u.statusMux.Unlock()
	u.logger.Error("Updater error", slog.Any("error", err))
}

func (u *Updater) resetStatus(state string) {
	u.statusMux.Lock()
	u.status = UpdaterStatus{
		State:    state,
		Progress: 0,
		Logs:     []string{},
	}
	u.statusMux.Unlock()
}

func (u *Updater) TryStartOperation(name string) bool {
	u.opMux.Lock()
	defer u.opMux.Unlock()
	if u.opRunning {
		return false
	}
	u.opRunning = true
	u.opName = name
	return true
}

func (u *Updater) EndOperation() {
	u.opMux.Lock()
	u.opRunning = false
	u.opName = ""
	u.opMux.Unlock()
}

// ExecuteUpdate performs the full update process
func (u *Updater) ExecuteUpdate(autoRestart bool) error {
	u.resetStatus("updating")
	u.updateProgress("updating", 10, "[1/5] Preparing update...")

	ctx, err := resolveRepoContext()
	if err != nil {
		u.setError(err)
		return err
	}

	// Step 1: Git pull
	u.updateProgress("updating", 20, "[2/5] Updating repository...")
	err = PerformUpdate(ctx, func(step, message string) {
		u.log(message)
	})
	if err != nil {
		u.setError(err)
		return err
	}

	// Step 2: Backup old executables
	u.updateProgress("updating", 40, "[3/5] Backing up old executables...")
	if err := u.backupOldExecutables(ctx.InstallDir, "update"); err != nil {
		u.setError(err)
		return err
	}

	// Step 3: Build new version
	u.updateProgress("building", 50, "[4/5] Building new version (this may take 1-2 minutes)...")
	if err := u.buildNewVersion(ctx); err != nil {
		u.setError(err)
		return err
	}

	// Step 4: Prepare restart
	u.updateProgress("done", 90, "[5/5] Update completed successfully!")

	if autoRestart {
		u.updateProgress("done", 95, "Preparing to restart...")
		time.Sleep(2 * time.Second)
		if err := u.restartApplication("update"); err != nil {
			u.setError(err)
			return err
		}
	} else {
		if err := u.scheduleMoveOnExit("update"); err != nil {
			u.setError(err)
			return err
		}
	}

	u.updateProgress("done", 100, "Update complete! Please restart the application.")
	return nil
}

// ExecuteBuild performs a build/restart without pulling new commits.
func (u *Updater) ExecuteBuild(autoRestart bool, backupTag string) error {
	u.resetStatus("building")
	u.updateProgress("building", 10, "[1/3] Preparing build...")

	ctx, err := resolveRepoContext()
	if err != nil {
		u.setError(err)
		return err
	}

	u.updateProgress("building", 35, "[2/3] Backing up old executables...")
	if err := u.backupOldExecutables(ctx.InstallDir, backupTag); err != nil {
		u.setError(err)
		return err
	}

	u.updateProgress("building", 60, "[3/3] Building new version (this may take 1-2 minutes)...")
	if err := u.buildNewVersion(ctx); err != nil {
		u.setError(err)
		return err
	}

	u.updateProgress("done", 90, "Build completed successfully!")

	if autoRestart {
		u.updateProgress("done", 95, "Preparing to restart...")
		time.Sleep(2 * time.Second)
		if err := u.restartApplication(backupTag); err != nil {
			u.setError(err)
			return err
		}
	} else {
		if err := u.scheduleMoveOnExit(backupTag); err != nil {
			u.setError(err)
			return err
		}
	}

	u.updateProgress("done", 100, "Build complete! Please restart the application.")
	return nil
}

func (u *Updater) backupOldExecutables(installDir string, tag string) error {
	oldDir := filepath.Join(installDir, "old_versions")
	currentExe := ""
	if exePath, err := os.Executable(); err == nil {
		if absExe, absErr := filepath.Abs(exePath); absErr == nil {
			currentExe = absExe
		}
	}
	// Check if install directory exists
	if _, err := os.Stat(installDir); os.IsNotExist(err) {
		u.log("No install directory found, skipping backup")
		return nil
	}

	// Find all .exe files in install directory
	files, err := filepath.Glob(filepath.Join(installDir, "*.exe"))
	if err != nil {
		return fmt.Errorf("failed to find exe files: %w", err)
	}

	if len(files) == 0 {
		u.log("No old executables found, skipping backup")
		return nil
	}

	// Create old_versions directory if it doesn't exist
	if err := os.MkdirAll(oldDir, 0755); err != nil {
		return fmt.Errorf("failed to create old_versions directory: %w", err)
	}

	// Move all exe files to old_versions
	backedUp := 0
	skipped := 0
	for _, file := range files {
		filename := filepath.Base(file)
		absFile := file
		if absPath, absErr := filepath.Abs(file); absErr == nil {
			absFile = absPath
		}
		if currentExe != "" && strings.EqualFold(absFile, currentExe) {
			u.log(fmt.Sprintf("Skipping running executable: %s", filename))
			skipped++
			continue
		}
		dest := filepath.Join(oldDir, filename)
		u.log(fmt.Sprintf("Moving %s to old_versions/", filename))
		if err := os.Rename(file, dest); err != nil {
			if copyErr := copyFile(file, dest); copyErr != nil {
				return fmt.Errorf("failed to move %s: %w", filename, err)
			}
			u.log(fmt.Sprintf("Copied %s to old_versions/ (file in use)", filename))
			backedUp++
			continue
		}
		backedUp++
	}

	if backedUp == 0 {
		if skipped > 0 {
			u.log("Running executable will be moved after exit")
			return nil
		}
		u.log("No old executables found, skipping backup")
		return nil
	}
	if skipped > 0 {
		u.log(fmt.Sprintf("Backed up %d old executable(s); running executable will be moved after exit", backedUp))
		if err := pruneOldVersions(oldDir, maxOldVersionBackups, u.log); err != nil {
			u.log(fmt.Sprintf("Backup cleanup skipped: %v", err))
		}
		return nil
	}
	u.log(fmt.Sprintf("Backed up %d old executable(s)", backedUp))
	if err := pruneOldVersions(oldDir, maxOldVersionBackups, u.log); err != nil {
		u.log(fmt.Sprintf("Backup cleanup skipped: %v", err))
	}
	return nil
}

func (u *Updater) buildNewVersion(ctx repoContext) error {
	// Check Go installation
	u.log("Checking Go installation...")
	goCmd := newCommand("go", "version")
	if output, err := goCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("Go is not installed: %w", err)
	} else {
		u.log(fmt.Sprintf("Go version: %s", strings.TrimSpace(string(output))))
	}

	// Check Garble installation
	u.log("Checking Garble installation...")
	garbleCmd := newCommand("garble", "version")
	if output, err := garbleCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("Garble is not installed: %w\nPlease install with: go install mvdan.cc/garble@v0.14.2", err)
	} else {
		u.log(fmt.Sprintf("Garble version: %s", strings.TrimSpace(string(output))))
	}

	// Generate build identifiers
	buildID := uuid.New().String()
	buildTime := time.Now().Format(time.RFC3339)
	outputExe := filepath.Join(ctx.InstallDir, buildID+".exe")

	u.log("Starting Garble build...")
	u.log(fmt.Sprintf("Build ID: %s", buildID))

	// Set environment variables for Garble
	restoreEnv := func(key string, val string, ok bool) {
		if ok {
			_ = os.Setenv(key, val)
			return
		}
		_ = os.Unsetenv(key)
	}

	gogarblePrev, gogarbleSet := os.LookupEnv("GOGARBLE")
	gocachePrev, gocacheSet := os.LookupEnv("GOCACHE")
	gotmpPrev, gotmpSet := os.LookupEnv("GOTMPDIR")
	defer func() {
		restoreEnv("GOGARBLE", gogarblePrev, gogarbleSet)
		restoreEnv("GOCACHE", gocachePrev, gocacheSet)
		restoreEnv("GOTMPDIR", gotmpPrev, gotmpSet)
	}()

	if err := os.Setenv("GOGARBLE", "*,!github.com/hectorgimenez/koolo/internal/server*,!github.com/hectorgimenez/koolo/internal/event*,!github.com/inkeliz/gowebview*"); err != nil {
		return fmt.Errorf("failed to set GOGARBLE: %w", err)
	}

	// Create build directory
	staticBuildDir, err := os.MkdirTemp(ctx.InstallDir, "koolo-build-")
	if err != nil {
		staticBuildDir, err = os.MkdirTemp("", "koolo-build-")
		if err != nil {
			return fmt.Errorf("failed to create build directory: %w", err)
		}
	}
	defer os.RemoveAll(staticBuildDir)

	if err := os.Setenv("GOCACHE", filepath.Join(staticBuildDir, "gocache")); err != nil {
		return fmt.Errorf("failed to set GOCACHE: %w", err)
	}
	if err := os.Setenv("GOTMPDIR", staticBuildDir); err != nil {
		return fmt.Errorf("failed to set GOTMPDIR: %w", err)
	}

	// Build ldflags
	ldflags := fmt.Sprintf("-s -w -H windowsgui -X 'main.buildID=%s' -X 'main.buildTime=%s' -X 'github.com/hectorgimenez/koolo/internal/config.Version=dev'", buildID, buildTime)

	// Execute Garble build
	cmd := newCommand("garble",
		"-literals=false",
		"-seed=random",
		"build",
		"-a",
		"-trimpath",
		"-tags", "static",
		"--ldflags", ldflags,
		"-o", outputExe,
		"./cmd/koolo",
	)
	cmd.Dir = ctx.RepoDir

	// Capture output
	output, err := cmd.CombinedOutput()
	if err != nil {
		u.log(fmt.Sprintf("Build failed: %s", string(output)))
		return fmt.Errorf("garble build failed: %w\n%s", err, output)
	}

	// Verify executable was created
	if _, err := os.Stat(outputExe); os.IsNotExist(err) {
		return fmt.Errorf("executable was not created at %s", outputExe)
	}

	u.log(fmt.Sprintf("Build successful: %s.exe", buildID))

	// Copy config files
	u.log("Copying configuration files...")
	if err := u.copyConfigFiles(ctx); err != nil {
		return err
	}

	u.log("Build process completed successfully!")
	return nil
}

func (u *Updater) copyConfigFiles(ctx repoContext) error {
	// Copy tools folder
	u.log("Copying tools folder...")
	if err := copyDir(filepath.Join(ctx.RepoDir, "tools"), filepath.Join(ctx.InstallDir, "tools")); err != nil {
		return fmt.Errorf("failed to copy tools: %w", err)
	}

	// Create config directory
	os.MkdirAll(filepath.Join(ctx.InstallDir, "config"), 0755)

	// Copy Settings.json if it doesn't exist
	settingsDest := filepath.Join(ctx.InstallDir, "config", "Settings.json")
	if _, err := os.Stat(settingsDest); os.IsNotExist(err) {
		u.log("Copying Settings.json...")
		if err := copyFile(filepath.Join(ctx.RepoDir, "config", "Settings.json"), settingsDest); err != nil {
			return fmt.Errorf("failed to copy Settings.json: %w", err)
		}
	}

	// Copy koolo.yaml.dist if koolo.yaml doesn't exist
	yamlDest := filepath.Join(ctx.InstallDir, "config", "koolo.yaml")
	if _, err := os.Stat(yamlDest); os.IsNotExist(err) {
		u.log("Copying koolo.yaml.dist...")
		if err := copyFile(filepath.Join(ctx.RepoDir, "config", "koolo.yaml.dist"), yamlDest); err != nil {
			return fmt.Errorf("failed to copy koolo.yaml: %w", err)
		}
	}

	// Copy template folder
	u.log("Copying template folder...")
	if err := copyDir(filepath.Join(ctx.RepoDir, "config", "template"), filepath.Join(ctx.InstallDir, "config", "template")); err != nil {
		return fmt.Errorf("failed to copy templates: %w", err)
	}

	// Copy README.md
	if err := copyFile(filepath.Join(ctx.RepoDir, "README.md"), filepath.Join(ctx.InstallDir, "README.md")); err != nil {
		u.log(fmt.Sprintf("Warning: failed to copy README.md: %v", err))
	}

	return nil
}

func (u *Updater) restartApplication(tag string) error {
	installDir, err := resolveInstallDir()
	if err != nil {
		return err
	}

	currentExe := ""
	if exePath, err := os.Executable(); err == nil {
		if absExe, absErr := filepath.Abs(exePath); absErr == nil {
			currentExe = absExe
		}
	}
	oldDir := filepath.Join(installDir, "old_versions")
	prefix := "pre_update_"
	switch strings.ToLower(strings.TrimSpace(tag)) {
	case "pr":
		prefix = "pre_PR_"
	case "build":
		prefix = "pre_build_"
	case "update":
		prefix = "pre_update_"
	}
	backupName := ""
	if currentExe != "" {
		backupName = fmt.Sprintf("%s%s_%s", prefix, time.Now().Format("20060102_150405"), filepath.Base(currentExe))
	}
	backupDest := ""
	if backupName != "" {
		backupDest = filepath.Join(oldDir, backupName)
	}

	// Find the newly built executable
	files, err := filepath.Glob(filepath.Join(installDir, "*.exe"))
	if err != nil || len(files) == 0 {
		return fmt.Errorf("no executable found after build")
	}

	// Get the most recent exe
	var newestExe string
	var newestTime time.Time
	for _, file := range files {
		info, err := os.Stat(file)
		if err != nil {
			continue
		}
		if info.ModTime().After(newestTime) {
			newestTime = info.ModTime()
			newestExe, _ = filepath.Abs(file)
		}
	}

	if newestExe == "" {
		return fmt.Errorf("could not find newest executable")
	}

	u.log(fmt.Sprintf("Restarting with: %s", newestExe))

	// Create restart script
	script := fmt.Sprintf(`@echo off
cd /d "%s"
if not exist "%s" mkdir "%s"
:WAIT_LOOP
timeout /t 1 /nobreak >nul
if "%s"=="" goto START_NEW
if exist "%s" (
    move /y "%s" "%s" >nul
    if exist "%s" goto WAIT_LOOP
)
:START_NEW
timeout /t 3 /nobreak >nul
start "" /D "%s" "%s"
del "%%~f0"
`, installDir, oldDir, oldDir, currentExe, currentExe, currentExe, backupDest, currentExe, installDir, newestExe)

	restartScript, err := writeRestartScript(installDir, "restart_koolo_*.bat", script)
	if err != nil {
		return err
	}

	if err := startScript(restartScript); err != nil {
		return err
	}

	// Exit current process
	os.Exit(0)
	return nil
}

func (u *Updater) scheduleMoveOnExit(tag string) error {
	installDir, err := resolveInstallDir()
	if err != nil {
		return err
	}

	currentExe := ""
	if exePath, err := os.Executable(); err == nil {
		if absExe, absErr := filepath.Abs(exePath); absErr == nil {
			currentExe = absExe
		}
	}
	if currentExe == "" {
		return nil
	}

	oldDir := filepath.Join(installDir, "old_versions")
	prefix := "pre_update_"
	switch strings.ToLower(strings.TrimSpace(tag)) {
	case "pr":
		prefix = "pre_PR_"
	case "build":
		prefix = "pre_build_"
	case "update":
		prefix = "pre_update_"
	}
	backupName := fmt.Sprintf("%s%s_%s", prefix, time.Now().Format("20060102_150405"), filepath.Base(currentExe))
	backupDest := filepath.Join(oldDir, backupName)

	script := fmt.Sprintf(`@echo off
cd /d "%s"
if not exist "%s" mkdir "%s"
:WAIT_LOOP
timeout /t 1 /nobreak >nul
if exist "%s" (
    move /y "%s" "%s" >nul
    if exist "%s" goto WAIT_LOOP
)
del "%%~f0"
`, installDir, oldDir, oldDir, currentExe, currentExe, backupDest, currentExe)

	moveScript, err := writeRestartScript(installDir, "move_koolo_*.bat", script)
	if err != nil {
		return err
	}

	return startScript(moveScript)
}

// Helper functions for file operations
func copyFile(src, dst string) error {
	data, err := os.ReadFile(src)
	if err != nil {
		return err
	}
	return os.WriteFile(dst, data, 0644)
}

func copyDir(src, dst string) error {
	// Remove destination if it exists
	os.RemoveAll(dst)

	// Create destination directory
	if err := os.MkdirAll(dst, 0755); err != nil {
		return err
	}

	// Read source directory
	entries, err := os.ReadDir(src)
	if err != nil {
		return err
	}

	for _, entry := range entries {
		srcPath := filepath.Join(src, entry.Name())
		dstPath := filepath.Join(dst, entry.Name())

		if entry.IsDir() {
			if err := copyDir(srcPath, dstPath); err != nil {
				return err
			}
		} else {
			if err := copyFile(srcPath, dstPath); err != nil {
				return err
			}
		}
	}

	return nil
}

func writeRestartScript(preferredDir, pattern, script string) (string, error) {
	dirs := []string{}
	if preferredDir != "" {
		dirs = append(dirs, preferredDir)
	}
	dirs = append(dirs, os.TempDir())

	for _, dir := range dirs {
		f, err := os.CreateTemp(dir, pattern)
		if err != nil {
			continue
		}
		if _, err := f.Write([]byte(script)); err != nil {
			f.Close()
			_ = os.Remove(f.Name())
			continue
		}
		if err := f.Close(); err != nil {
			_ = os.Remove(f.Name())
			continue
		}
		return f.Name(), nil
	}

	return "", fmt.Errorf("failed to create restart script")
}

func startScript(scriptPath string) error {
	cmd := newCommand("cmd", "/c", scriptPath)
	if err := cmd.Start(); err != nil {
		return fmt.Errorf("failed to start restart script: %w", err)
	}
	return nil
}
