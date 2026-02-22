package updater

import (
	"crypto/sha256"
	"encoding/hex"
	"io"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"
)

const maxOldVersionBackups = 20

type backupFile struct {
	path    string
	modTime time.Time
	size    int64
	hash    string
}

func fileHash(path string) (string, error) {
	f, err := os.Open(path)
	if err != nil {
		return "", err
	}
	defer f.Close()

	hasher := sha256.New()
	if _, err := io.Copy(hasher, f); err != nil {
		return "", err
	}
	return hex.EncodeToString(hasher.Sum(nil)), nil
}

func filesSameContent(a, b string) (bool, error) {
	infoA, err := os.Stat(a)
	if err != nil {
		return false, err
	}
	infoB, err := os.Stat(b)
	if err != nil {
		return false, err
	}
	if infoA.Size() != infoB.Size() {
		return false, nil
	}
	hashA, err := fileHash(a)
	if err != nil {
		return false, err
	}
	hashB, err := fileHash(b)
	if err != nil {
		return false, err
	}
	return hashA == hashB, nil
}

func pruneOldVersions(oldDir string, maxKeep int, logf func(string)) error {
	if maxKeep <= 0 {
		return nil
	}
	entries, err := os.ReadDir(oldDir)
	if err != nil {
		return err
	}

	files := make([]backupFile, 0)
	for _, entry := range entries {
		if entry.IsDir() {
			continue
		}
		name := entry.Name()
		if !strings.HasSuffix(strings.ToLower(name), ".exe") {
			continue
		}
		info, err := entry.Info()
		if err != nil {
			continue
		}
		files = append(files, backupFile{
			path:    filepath.Join(oldDir, name),
			modTime: info.ModTime(),
			size:    info.Size(),
		})
	}

	if len(files) <= 1 {
		return nil
	}

	sort.Slice(files, func(i, j int) bool {
		return files[i].modTime.After(files[j].modTime)
	})

	kept := make([]backupFile, 0, len(files))
	seen := make(map[string]struct{})
	for _, file := range files {
		hash, err := fileHash(file.path)
		if err != nil {
			kept = append(kept, file)
			continue
		}
		file.hash = hash
		if _, exists := seen[hash]; exists {
			if logf != nil {
				logf("Removing duplicate backup: " + filepath.Base(file.path))
			}
			_ = os.Remove(file.path)
			continue
		}
		seen[hash] = struct{}{}
		kept = append(kept, file)
	}

	if len(kept) <= maxKeep {
		return nil
	}

	sort.Slice(kept, func(i, j int) bool {
		return kept[i].modTime.After(kept[j].modTime)
	})

	for i := maxKeep; i < len(kept); i++ {
		if logf != nil {
			logf("Removing old backup: " + filepath.Base(kept[i].path))
		}
		_ = os.Remove(kept[i].path)
	}

	return nil
}
