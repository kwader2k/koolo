package droplog

import (
	"bufio"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"log/slog"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/event"
)

// Record is the persisted representation of a stashed drop including metadata for aggregation.
type Record struct {
	Time       time.Time `json:"time"`
	Supervisor string    `json:"supervisor"`
	Character  string    `json:"character"` // in-game character name
	Profile    string    `json:"profile"`   // config folder name
	Drop       data.Drop `json:"drop"`
}

type Writer struct {
	logDir string
	logger *slog.Logger
}

func NewWriter(logDir string, logger *slog.Logger) *Writer {
	return &Writer{logDir: logDir, logger: logger}
}

// Handle subscribes to the event bus and persists ItemStashedEvent to a daily JSONL file.
func (w *Writer) Handle(_ context.Context, e event.Event) error {
	ist, ok := e.(event.ItemStashedEvent)
	if !ok {
		return nil
	}

	// Resolve metadata
	sup := e.Supervisor()
	charName := ""
	profile := ""
	if cfg, found := config.GetCharacter(sup); found && cfg != nil { 
		charName = cfg.CharacterName
		profile = cfg.ConfigFolderName
	}

	rec := Record{
		Time:       e.OccurredAt(),
		Supervisor: sup,
		Character:  charName,
		Profile:    profile,
		Drop:       ist.Item,
	}

	// Ensure directory exists
	if err := os.MkdirAll(w.logDir, 0o755); err != nil {
		w.logger.Error("Failed to create droplog directory", slog.Any("error", err), slog.String("dir", w.logDir))
		return nil // don't break the bot because of logging errors
	}

	// Daily rotation by date
	file := filepath.Join(w.logDir, fmt.Sprintf("droplog-%s.jsonl", time.Now().Format("2006-01-02")))
	f, err := os.OpenFile(file, os.O_CREATE|os.O_APPEND|os.O_WRONLY, 0o644)
	if err != nil {
		w.logger.Error("Failed to open droplog file", slog.Any("error", err), slog.String("file", file))
		return nil
	}
	defer f.Close()

	enc, err := json.Marshal(rec)
	if err != nil {
		w.logger.Error("Failed to encode droplog record", slog.Any("error", err))
		return nil
	}
	if _, err = f.Write(append(enc, '\n')); err != nil {
		w.logger.Error("Failed to write droplog record", slog.Any("error", err))
	}

	return nil
}

// ReadAll scans the log directory for droplog-*.jsonl files, parses them, and returns all records.
func ReadAll(logDir string) ([]Record, error) {
	files, err := filepath.Glob(filepath.Join(logDir, "droplog-*.jsonl"))
	if err != nil {
		return nil, err
	}

	// Sort by name (date) ascending
	sortStrings(files)

	var out []Record
	for _, fpath := range files {
		f, err := os.Open(fpath)
		if err != nil {
			continue
		}
		r := bufio.NewReader(f)
		for {
			line, err := r.ReadString('\n')
			if len(line) > 0 {
				var rec Record
				if err := json.Unmarshal([]byte(strings.TrimSpace(line)), &rec); err == nil {
					out = append(out, rec)
				}
			}
			if err != nil {
				if err == io.EOF {
					break
				}
				break
			}
		}
		f.Close()
	}
	return out, nil
}

// minimal local sort to avoid pulling extra deps
func sortStrings(a []string) {
	for i := 0; i < len(a); i++ {
		for j := i + 1; j < len(a); j++ {
			if a[j] < a[i] {
				a[i], a[j] = a[j], a[i]
			}
		}
	}
}
