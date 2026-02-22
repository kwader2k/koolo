package updater

import (
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"
)

type BackupVersion struct {
	Filename  string    `json:"filename"`
	FilePath  string    `json:"filePath"`
	CreatedAt time.Time `json:"createdAt"`
	Size      int64     `json:"size"`
	IsCurrent bool      `json:"isCurrent"`
}

// GetBackupVersions returns a list of backup executables sorted by creation time (newest first)
func GetBackupVersions(limit int) ([]BackupVersion, error) {
	installDir, err := resolveInstallDir()
	if err != nil {
		return nil, err
	}

	oldVersionsDir := filepath.Join(installDir, "old_versions")

	// Check if directory exists
	if _, err := os.Stat(oldVersionsDir); os.IsNotExist(err) {
		return []BackupVersion{}, nil
	}

	_ = pruneOldVersions(oldVersionsDir, maxOldVersionBackups, nil)

	entries, err := os.ReadDir(oldVersionsDir)
	if err != nil {
		return nil, fmt.Errorf("failed to list backup files: %w", err)
	}

	// Get current running executable to mark it
	currentExe, _ := os.Executable()
	currentExePath, _ := filepath.Abs(currentExe)

	versions := make([]BackupVersion, 0)
	for _, entry := range entries {
		if entry.IsDir() {
			continue
		}
		name := entry.Name()
		if !strings.HasSuffix(strings.ToLower(name), ".exe") {
			continue
		}

		file := filepath.Join(oldVersionsDir, name)
		info, err := entry.Info()
		if err != nil {
			continue
		}

		absPath, _ := filepath.Abs(file)
		version := BackupVersion{
			Filename:  name,
			FilePath:  file,
			CreatedAt: info.ModTime(),
			Size:      info.Size(),
			IsCurrent: absPath == currentExePath,
		}

		if limit <= 0 {
			versions = append(versions, version)
			continue
		}

		inserted := false
		for i, existing := range versions {
			if version.CreatedAt.After(existing.CreatedAt) {
				versions = append(versions, BackupVersion{})
				copy(versions[i+1:], versions[i:])
				versions[i] = version
				inserted = true
				break
			}
		}
		if !inserted {
			versions = append(versions, version)
		}
		if len(versions) > limit {
			versions = versions[:limit]
		}
	}

	if limit <= 0 {
		// Sort by creation time (newest first)
		sort.Slice(versions, func(i, j int) bool {
			return versions[i].CreatedAt.After(versions[j].CreatedAt)
		})
	}

	return versions, nil
}

// GetCurrentExecutable returns information about the currently running executable
func GetCurrentExecutable() (*BackupVersion, error) {
	installDir, err := resolveInstallDir()
	if err != nil {
		return nil, err
	}

	if exePath, err := os.Executable(); err == nil {
		if absPath, absErr := filepath.Abs(exePath); absErr == nil {
			if info, statErr := os.Stat(absPath); statErr == nil {
				return &BackupVersion{
					Filename:  filepath.Base(absPath),
					FilePath:  absPath,
					CreatedAt: info.ModTime(),
					Size:      info.Size(),
					IsCurrent: true,
				}, nil
			}
		}
	}

	// Find current exe in install directory
	files, err := filepath.Glob(filepath.Join(installDir, "*.exe"))
	if err != nil || len(files) == 0 {
		return nil, fmt.Errorf("no executable found in install directory")
	}

	// Get the most recent one (should be current)
	var currentExe string
	var currentTime time.Time
	for _, file := range files {
		info, err := os.Stat(file)
		if err != nil {
			continue
		}
		if info.ModTime().After(currentTime) {
			currentTime = info.ModTime()
			currentExe = file
		}
	}

	if currentExe == "" {
		return nil, fmt.Errorf("no current executable found")
	}

	info, err := os.Stat(currentExe)
	if err != nil {
		return nil, err
	}

	return &BackupVersion{
		Filename:  filepath.Base(currentExe),
		FilePath:  currentExe,
		CreatedAt: info.ModTime(),
		Size:      info.Size(),
		IsCurrent: true,
	}, nil
}

// RollbackToVersion restores a backup version and restarts the application
func (u *Updater) RollbackToVersion(backupFilePath string) error {
	u.resetStatus("rollback")
	u.log(fmt.Sprintf("Starting rollback to: %s", filepath.Base(backupFilePath)))

	installDir, err := resolveInstallDir()
	if err != nil {
		return err
	}

	oldVersionsDir := filepath.Join(installDir, "old_versions")
	absOldVersions, err := filepath.Abs(oldVersionsDir)
	if err != nil {
		return fmt.Errorf("failed to resolve old_versions path: %w", err)
	}
	if err := os.MkdirAll(absOldVersions, 0o755); err != nil {
		return fmt.Errorf("failed to create old_versions directory: %w", err)
	}
	absBackup, err := filepath.Abs(backupFilePath)
	if err != nil {
		return fmt.Errorf("failed to resolve backup path: %w", err)
	}
	if !isPathWithinDir(absOldVersions, absBackup) {
		return fmt.Errorf("backup file must be inside %s", oldVersionsDir)
	}

	// Verify backup file exists
	if _, err := os.Stat(absBackup); os.IsNotExist(err) {
		return fmt.Errorf("backup file not found: %s", backupFilePath)
	}

	// Resolve current exe path
	currentExe, _ := os.Executable()
	absCurrentExe, _ := filepath.Abs(currentExe)

	if absCurrentExe != "" {
		if same, err := filesSameContent(absCurrentExe, absBackup); err == nil && same {
			u.log("Selected version matches current executable; rollback skipped.")
			return nil
		}
	}

	// Copy backup to install directory via script after exit
	destPath := filepath.Join(installDir, filepath.Base(absBackup))
	u.log(fmt.Sprintf("Restoring backup to: %s", destPath))

	u.log("Preparing to restart application...")
	backupName := fmt.Sprintf("pre_rollback_%s.exe", time.Now().Format("20060102_150405"))
	backupDest := filepath.Join(absOldVersions, backupName)
	pid := os.Getpid()
	script := fmt.Sprintf(`@echo off
setlocal enabledelayedexpansion
if not exist "%s" mkdir "%s"
:WAIT_LOOP
timeout /t 1 /nobreak >nul
if exist "%s" (
    move /y "%s" "%s" >nul
    if exist "%s" goto WAIT_LOOP
)
copy /y "%s" "%s"
set /a PORT_WAIT=0
:WAIT_PORT
netstat -ano | findstr /R /C:":8087 .*LISTENING" >nul
if %%ERRORLEVEL%%==0 (
    set /a PORT_WAIT+=1
    if !PORT_WAIT! GEQ 60 goto START_APP
    timeout /t 1 /nobreak >nul
    goto WAIT_PORT
)
:WAIT_PID
tasklist /FI "PID eq %d" 2>nul | findstr /R /C:" %d " >nul
if %%ERRORLEVEL%%==0 (
    timeout /t 1 /nobreak >nul
    goto WAIT_PID
)
:START_APP
start "" "%s"
del "%%~f0"
`, absOldVersions, absOldVersions, absCurrentExe, absCurrentExe, backupDest, absCurrentExe, absBackup, destPath, pid, pid, destPath)

	restartScript, err := writeRestartScript(installDir, "restart_koolo_rollback_*.bat", script)
	if err != nil {
		return err
	}

	cmd := newCommand("cmd", "/c", restartScript)
	if err := cmd.Start(); err != nil {
		return fmt.Errorf("failed to start rollback script: %w", err)
	}

	if err := pruneOldVersions(absOldVersions, maxOldVersionBackups, u.log); err != nil {
		u.log(fmt.Sprintf("Backup cleanup skipped: %v", err))
	}

	time.Sleep(1 * time.Second)
	os.Exit(0)
	return nil
}

func isPathWithinDir(baseDir, targetPath string) bool {
	base := strings.TrimRight(baseDir, string(os.PathSeparator)) + string(os.PathSeparator)
	target := targetPath
	if base == "" || target == "" {
		return false
	}
	return strings.HasPrefix(strings.ToLower(target), strings.ToLower(base))
}
