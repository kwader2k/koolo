package updater

import (
	"strings"
	"time"
)

type VersionInfo struct {
	CommitHash string
	CommitDate time.Time
	CommitMsg  string
	Branch     string
}

// GetCurrentVersion returns the current local Git version info.
func GetCurrentVersion() (*VersionInfo, error) {
	ctx, err := resolveRepoContext()
	if err != nil {
		return nil, err
	}

	return getCurrentVersion(ctx.RepoDir)
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
		CommitHash: commitHash[:7], // Short hash
		CommitDate: commitDate,
		CommitMsg:  commitMsg,
		Branch:     branch,
	}, nil
}
