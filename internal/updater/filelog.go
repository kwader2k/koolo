package updater

import (
	"fmt"
	"os"
	"path/filepath"
	"sync"
	"time"
)

var logFileMu sync.Mutex

const maxOperationLogBytes = 5 * 1024 * 1024

func logFileName(op string) string {
	switch op {
	case "cherry-pick":
		return "updater_pr_apply.log"
	case "revert":
		return "updater_pr_revert.log"
	case "rollback":
		return "updater_rollback.log"
	case "update":
		return "updater_update.log"
	default:
		return "updater.log"
	}
}

func appendOperationLog(op, message string) {
	installDir, err := resolveInstallDir()
	if err != nil {
		return
	}
	logDir := filepath.Join(installDir, "logs")
	if err := os.MkdirAll(logDir, 0o755); err != nil {
		return
	}
	logPath := filepath.Join(logDir, logFileName(op))

	timestamp := time.Now().Format(time.RFC3339)
	entry := fmt.Sprintf("[%s] %s\n", timestamp, message)

	logFileMu.Lock()
	defer logFileMu.Unlock()
	if info, err := os.Stat(logPath); err == nil && info.Size() > maxOperationLogBytes {
		truncateMsg := fmt.Sprintf("[%s] log truncated (exceeded %d bytes)\n", time.Now().Format(time.RFC3339), maxOperationLogBytes)
		_ = os.WriteFile(logPath, []byte(truncateMsg), 0o644)
	}
	file, err := os.OpenFile(logPath, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0o644)
	if err != nil {
		return
	}
	_, _ = file.WriteString(entry)
	_ = file.Close()
}
