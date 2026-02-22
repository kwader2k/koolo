package updater

import (
	"os"
	"path/filepath"
	"strings"
	"time"
)

// buildCommitHash is injected at build time via -ldflags.
var buildCommitHash string

// buildCommitTime is injected at build time via -ldflags (RFC3339).
var buildCommitTime string

type VersionInfo struct {
	CommitHash string
	CommitDate time.Time
	CommitMsg  string
	Branch     string

	commitHashFull string
}

// GetCurrentVersion returns the current build or local Git version info.
func GetCurrentVersion() (*VersionInfo, error) {
	if embedded := getEmbeddedVersion(); embedded != nil {
		return embedded, nil
	}

	ctx, err := resolveRepoContext()
	if err != nil {
		return nil, err
	}

	return getCurrentVersion(ctx.RepoDir)
}

// GetCurrentVersionNoClone returns the current build or local Git version info
// without cloning a repository if none exists.
func GetCurrentVersionNoClone() (*VersionInfo, error) {
	if embedded := getEmbeddedVersion(); embedded != nil {
		return embedded, nil
	}

	workDir, err := os.Getwd()
	if err != nil {
		return nil, err
	}

	if root, ok := findGitRoot(workDir); ok {
		return getCurrentVersion(root)
	}

	repoDir := filepath.Join(workDir, sourceDirName)
	isRepo, err := isGitRepo(repoDir)
	if err != nil {
		return nil, err
	}
	if !isRepo {
		return nil, nil
	}

	return getCurrentVersion(repoDir)
}

func (v *VersionInfo) fullHash() string {
	if v == nil {
		return ""
	}
	if v.commitHashFull != "" {
		return v.commitHashFull
	}
	return v.CommitHash
}

func getEmbeddedVersion() *VersionInfo {
	commitHash := strings.TrimSpace(buildCommitHash)
	if commitHash == "" {
		return nil
	}

	var commitDate time.Time
	if buildCommitTime != "" {
		if parsed, err := time.Parse(time.RFC3339, strings.TrimSpace(buildCommitTime)); err == nil {
			commitDate = parsed
		}
	}

	return &VersionInfo{
		CommitHash:     shortHash(commitHash),
		CommitDate:     commitDate,
		CommitMsg:      "",
		Branch:         "unknown",
		commitHashFull: commitHash,
	}
}

func getCurrentVersion(repoDir string) (*VersionInfo, error) {
	// Prefer current HEAD so the UI reflects the active branch/version.
	hashCmd := gitCmd(repoDir, "rev-parse", "HEAD")
	hashOut, err := hashCmd.Output()
	if err != nil {
		// If HEAD isn't available, try main
		hashCmd = gitCmd(repoDir, "rev-parse", "main")
		hashOut, err = hashCmd.Output()
		if err != nil {
			// Last resort: use origin/main
			hashCmd = gitCmd(repoDir, "rev-parse", "origin/main")
			hashOut, err = hashCmd.Output()
			if err != nil {
				return nil, err
			}
		}
	}
	commitHash := strings.TrimSpace(string(hashOut))

	// Get commit date
	dateCmd := gitCmd(repoDir, "show", "-s", "--format=%ci", commitHash)
	dateOut, err := dateCmd.Output()
	if err != nil {
		return nil, err
	}
	commitDate, err := time.Parse("2006-01-02 15:04:05 -0700", strings.TrimSpace(string(dateOut)))
	if err != nil {
		return nil, err
	}

	// Get commit message (first line only)
	msgCmd := gitCmd(repoDir, "show", "-s", "--format=%s", commitHash)
	msgOut, err := msgCmd.Output()
	if err != nil {
		return nil, err
	}
	commitMsg := strings.TrimSpace(string(msgOut))

	// Get current branch (fallback to HEAD if detached)
	branchCmd := gitCmd(repoDir, "rev-parse", "--abbrev-ref", "HEAD")
	branchOut, err := branchCmd.Output()
	if err != nil {
		return nil, err
	}
	branch := strings.TrimSpace(string(branchOut))
	if branch == "" {
		branch = "HEAD"
	}

	return &VersionInfo{
		CommitHash:     shortHash(commitHash),
		CommitDate:     commitDate,
		CommitMsg:      commitMsg,
		Branch:         branch,
		commitHashFull: commitHash,
	}, nil
}

func shortHash(hash string) string {
	if len(hash) > 7 {
		return hash[:7]
	}
	return hash
}
