package updater

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"
)

type CommitInfo struct {
	Hash    string
	Date    time.Time
	Message string
}

type UpdateCheckResult struct {
	HasUpdates     bool
	CommitsAhead   int
	CommitsBehind  int
	AheadCommits   []CommitInfo
	NewCommits     []CommitInfo
	CurrentVersion *VersionInfo
}

// CheckForUpdates checks if there are any new commits available from upstream
func CheckForUpdates() (*UpdateCheckResult, error) {
	// Check if git is installed
	if err := checkGitInstalled(); err != nil {
		return nil, err
	}

	ctx, err := resolveRepoContext()
	if err != nil {
		return nil, err
	}

	// Get current version
	currentVersion := getEmbeddedVersion()
	if currentVersion == nil {
		currentVersion, err = getCurrentVersion(ctx.RepoDir)
		if err != nil {
			return nil, fmt.Errorf("failed to get current version: %w", err)
		}
	}

	// Ensure upstream remote is configured
	if err := ensureUpstreamRemote(ctx.RepoDir); err != nil {
		return nil, err
	}

	// Fetch latest from upstream
	fetchCmd := gitCmd(ctx.RepoDir, "fetch", "upstream", "main")
	if err := fetchCmd.Run(); err != nil {
		return nil, fmt.Errorf("git fetch upstream failed: %w", err)
	}

	result := &UpdateCheckResult{
		CurrentVersion: currentVersion,
	}

	// Get ahead/behind counts
	baseRef := comparisonBaseRef(ctx.RepoDir, currentVersion)
	countCmd := gitCmd(ctx.RepoDir, "rev-list", "--left-right", "--count", fmt.Sprintf("%s...upstream/main", baseRef))
	countOut, err := countCmd.Output()
	if err != nil {
		return nil, fmt.Errorf("failed to compare with upstream: %w", err)
	}
	fmt.Sscanf(string(countOut), "%d %d", &result.CommitsAhead, &result.CommitsBehind)

	result.HasUpdates = result.CommitsBehind > 0

	if result.CommitsAhead > 0 {
		aheadLogCmd := gitCmd(ctx.RepoDir, "log", "--pretty=format:%H|%ci|%s", fmt.Sprintf("upstream/main..%s", baseRef), "-n", "10")
		aheadLogOut, err := aheadLogCmd.Output()
		if err == nil {
			lines := strings.Split(strings.TrimSpace(string(aheadLogOut)), "\n")
			for _, line := range lines {
				if line == "" {
					continue
				}
				parts := strings.SplitN(line, "|", 3)
				if len(parts) != 3 {
					continue
				}

				commitDate, _ := time.Parse("2006-01-02 15:04:05 -0700", parts[1])
				hash := parts[0]
				if len(hash) > 7 {
					hash = hash[:7]
				}
				commit := CommitInfo{
					Hash:    hash,
					Date:    commitDate,
					Message: parts[2],
				}
				result.AheadCommits = append(result.AheadCommits, commit)
			}
		}
	}

	if !result.HasUpdates {
		return result, nil
	}

	// Get list of new commits (up to 10)
	logCmd := gitCmd(ctx.RepoDir, "log", "--pretty=format:%H|%ci|%s", fmt.Sprintf("%s..upstream/main", baseRef), "-n", "10")
	logOut, err := logCmd.Output()
	if err != nil {
		return result, nil // Return partial result
	}

	lines := strings.Split(strings.TrimSpace(string(logOut)), "\n")
	for _, line := range lines {
		if line == "" {
			continue
		}
		parts := strings.SplitN(line, "|", 3)
		if len(parts) != 3 {
			continue
		}

		commitDate, _ := time.Parse("2006-01-02 15:04:05 -0700", parts[1])
		hash := parts[0]
		if len(hash) > 7 {
			hash = hash[:7]
		}
		commit := CommitInfo{
			Hash:    hash, // Short hash
			Date:    commitDate,
			Message: parts[2],
		}
		result.NewCommits = append(result.NewCommits, commit)
	}

	return result, nil
}

func comparisonBaseRef(repoDir string, current *VersionInfo) string {
	if current == nil {
		return "HEAD"
	}
	fullHash := current.fullHash()
	if fullHash == "" {
		return "HEAD"
	}
	if err := gitCmd(repoDir, "cat-file", "-e", fmt.Sprintf("%s^{commit}", fullHash)).Run(); err != nil {
		return "HEAD"
	}
	return fullHash
}

// BootstrapRepoForUpdates initializes the updater repo when missing and writes .commit.
func BootstrapRepoForUpdates() (bool, error) {
	if err := checkGitInstalled(); err != nil {
		return false, err
	}

	installDir, err := resolveInstallDir()
	if err != nil {
		return false, err
	}

	commitPath := filepath.Join(installDir, ".commit")
	if _, err := os.Stat(commitPath); err == nil {
		return false, nil
	} else if err != nil && !os.IsNotExist(err) {
		return false, err
	}

	ctx, err := resolveExistingRepoContext()
	if err != nil {
		if !strings.Contains(err.Error(), "no git repository found") {
			return false, err
		}
		ctx, err = resolveRepoContext()
		if err != nil {
			return false, err
		}
	}

	if err := ensureUpstreamRemote(ctx.RepoDir); err != nil {
		return true, err
	}

	fetchCmd := gitCmd(ctx.RepoDir, "fetch", "upstream", "main")
	if output, err := fetchCmd.CombinedOutput(); err != nil {
		return true, fmt.Errorf("git fetch upstream failed: %w\n%s", err, output)
	}

	hashOut, err := gitCmd(ctx.RepoDir, "rev-parse", "upstream/main").Output()
	if err != nil {
		return true, fmt.Errorf("failed to resolve upstream/main commit: %w", err)
	}

	if err := updateCommitFile(ctx, strings.TrimSpace(string(hashOut))); err != nil {
		return true, fmt.Errorf("failed to write .commit: %w", err)
	}

	return true, nil
}

// GetCurrentCommits returns recent commits from the current HEAD.
func GetCurrentCommits(limit int) ([]CommitInfo, error) {
	if limit <= 0 || limit > 50 {
		limit = 10
	}

	if err := checkGitInstalled(); err != nil {
		return nil, err
	}

	ctx, err := resolveRepoContext()
	if err != nil {
		return nil, err
	}

	logCmd := gitCmd(ctx.RepoDir, "log", "--pretty=format:%H|%ci|%s", "HEAD", "-n", fmt.Sprintf("%d", limit))
	logOut, err := logCmd.Output()
	if err != nil {
		return nil, err
	}

	lines := strings.Split(strings.TrimSpace(string(logOut)), "\n")
	commits := make([]CommitInfo, 0, len(lines))
	for _, line := range lines {
		if line == "" {
			continue
		}
		parts := strings.SplitN(line, "|", 3)
		if len(parts) != 3 {
			continue
		}
		commitDate, _ := time.Parse("2006-01-02 15:04:05 -0700", parts[1])
		hash := parts[0]
		if len(hash) > 7 {
			hash = hash[:7]
		}
		commits = append(commits, CommitInfo{
			Hash:    hash,
			Date:    commitDate,
			Message: parts[2],
		})
	}

	return commits, nil
}

// PerformUpdate executes a fetch+merge update from upstream and returns update progress
func PerformUpdate(ctx repoContext, progressCallback func(step, message string)) error {
	if progressCallback == nil {
		progressCallback = func(step, message string) {}
	}

	// Check git installation
	progressCallback("check", "Checking Git installation...")
	if err := checkGitInstalled(); err != nil {
		return err
	}

	// Ensure upstream remote is configured
	if err := ensureUpstreamRemote(ctx.RepoDir); err != nil {
		return err
	}

	// Verify current branch
	progressCallback("branch", "Verifying current branch...")
	branchCmd := gitCmd(ctx.RepoDir, "rev-parse", "--abbrev-ref", "HEAD")
	branchOut, err := branchCmd.Output()
	if err != nil {
		return fmt.Errorf("failed to determine current branch: %w", err)
	}
	currentBranch := strings.TrimSpace(string(branchOut))
	if currentBranch == "" || currentBranch == "HEAD" {
		return fmt.Errorf("detached HEAD state detected; please switch to the main branch before updating")
	}
	if currentBranch != "main" {
		progressCallback("branch", fmt.Sprintf("Current branch is %q; updating against upstream/main...", currentBranch))
	}

	// Check for local changes
	progressCallback("status", "Checking local changes...")
	statusCmd := gitCmd(ctx.RepoDir, "status", "--porcelain")
	statusOut, err := statusCmd.Output()
	if err != nil {
		return fmt.Errorf("failed to check working tree: %w", err)
	}
	dirty := strings.TrimSpace(string(statusOut)) != ""
	if dirty {
		return newDiscardLocalChangesError("Local changes detected. Force sync will discard them before updating.")
	}

	// Fetch latest changes from upstream
	progressCallback("fetch", "Fetching latest changes from upstream/main...")
	fetchCmd := gitCmd(ctx.RepoDir, "fetch", "upstream", "main")
	if output, err := fetchCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("git fetch upstream failed: %w\n%s", err, output)
	}

	tmpRoot := filepath.Join(ctx.RepoDir, ".tmp", "update-worktrees")
	if err := os.MkdirAll(tmpRoot, 0o755); err != nil {
		return fmt.Errorf("failed to create update worktree directory: %w", err)
	}

	worktreeName := fmt.Sprintf("koolo-update-%d", time.Now().UnixNano())
	worktreeDir := filepath.Join(tmpRoot, worktreeName)
	progressCallback("worktree", "Creating isolated worktree for update preview...")
	worktreeCmd := gitCmd(ctx.RepoDir, "worktree", "add", "-b", worktreeName, worktreeDir, "HEAD")
	if output, err := worktreeCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("failed to create worktree: %w\n%s", err, output)
	}
	defer func() {
		_ = gitCmd(ctx.RepoDir, "worktree", "remove", "--force", worktreeDir).Run()
		_ = gitCmd(ctx.RepoDir, "branch", "-D", worktreeName).Run()
		_ = os.RemoveAll(worktreeDir)
		cleanupTmpRoot(tmpRoot)
	}()

	progressCallback("merge", "Testing merge with upstream/main...")
	mergeCmd := gitCmd(worktreeDir, "merge", "--no-edit", "upstream/main")
	mergeOutput, mergeErr := mergeCmd.CombinedOutput()
	if mergeErr != nil {
		outputStr := string(mergeOutput)
		conflict := strings.Contains(outputStr, "CONFLICT") || strings.Contains(outputStr, "Automatic merge failed")
		if !conflict {
			if lsOut, lsErr := gitCmd(worktreeDir, "ls-files", "-u").Output(); lsErr == nil {
				if strings.TrimSpace(string(lsOut)) != "" {
					conflict = true
				}
			}
		}
		if conflict {
			_ = gitCmd(worktreeDir, "merge", "--abort").Run()
			return newDiscardLocalChangesError("Update conflict detected. Force sync will discard local changes and reset to upstream/main.")
		}
		return fmt.Errorf("git merge upstream/main failed: %w\n%s", mergeErr, outputStr)
	}

	headOut, err := gitCmd(worktreeDir, "rev-parse", "HEAD").Output()
	if err != nil {
		return fmt.Errorf("failed to resolve worktree HEAD: %w", err)
	}
	worktreeHead := strings.TrimSpace(string(headOut))
	if worktreeHead == "" {
		return fmt.Errorf("failed to resolve worktree HEAD commit")
	}

	progressCallback("update", fmt.Sprintf("Updating %s to merged upstream/main...", currentBranch))
	checkoutCmd := gitCmd(ctx.RepoDir, "checkout", "-B", currentBranch, worktreeHead)
	if output, err := checkoutCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("git checkout -B %s failed: %w\n%s", currentBranch, err, output)
	}
	resetCmd := gitCmd(ctx.RepoDir, "reset", "--hard", worktreeHead)
	if output, err := resetCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("git reset --hard failed: %w\n%s", err, output)
	}
	cleanCmd := gitCmd(ctx.RepoDir, "clean", "-fd")
	if output, err := cleanCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("git clean -fd failed: %w\n%s", err, output)
	}

	progressCallback("commit", "Updating .commit to upstream/main...")
	hashOut, err := gitCmd(ctx.RepoDir, "rev-parse", "upstream/main").Output()
	if err != nil {
		return fmt.Errorf("failed to resolve upstream/main commit: %w", err)
	}
	if err := updateCommitFile(ctx, strings.TrimSpace(string(hashOut))); err != nil {
		return fmt.Errorf("failed to write .commit: %w", err)
	}

	progressCallback("complete", "Git update completed successfully")
	return nil
}

// ForceSyncToUpstream discards local changes and hard-resets to upstream/main.
func ForceSyncToUpstream(ctx repoContext, progressCallback func(step, message string)) error {
	if progressCallback == nil {
		progressCallback = func(step, message string) {}
	}

	// Check git installation
	progressCallback("check", "Checking Git installation...")
	if err := checkGitInstalled(); err != nil {
		return err
	}

	// Ensure upstream remote is configured
	if err := ensureUpstreamRemote(ctx.RepoDir); err != nil {
		return err
	}

	// Prepare branch (force sync can recover from detached or empty HEAD)
	progressCallback("branch", "Preparing branch for force sync...")
	branchCmd := gitCmd(ctx.RepoDir, "rev-parse", "--abbrev-ref", "HEAD")
	if branchOut, err := branchCmd.Output(); err == nil {
		currentBranch := strings.TrimSpace(string(branchOut))
		if currentBranch == "" || currentBranch == "HEAD" {
			progressCallback("branch", "No active branch detected; creating main from upstream/main...")
		} else if currentBranch != "main" {
			progressCallback("branch", fmt.Sprintf("Current branch is %q; syncing against upstream/main...", currentBranch))
		}
	}

	// Fetch latest changes from upstream
	progressCallback("fetch", "Fetching latest changes from upstream/main...")
	fetchCmd := gitCmd(ctx.RepoDir, "fetch", "upstream", "main")
	if output, err := fetchCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("git fetch upstream failed: %w\n%s", err, output)
	}

	// Force reset to upstream/main
	progressCallback("reset", "Forcing sync with upstream/main (discarding local changes)...")
	cleanCmd := gitCmd(ctx.RepoDir, "clean", "-fd")
	if output, err := cleanCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("git clean -fd failed: %w\n%s", err, output)
	}
	checkoutCmd := gitCmd(ctx.RepoDir, "checkout", "-f", "-B", "main", "upstream/main")
	if output, err := checkoutCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("git checkout -B main upstream/main failed: %w\n%s", err, output)
	}
	resetCmd := gitCmd(ctx.RepoDir, "reset", "--hard", "upstream/main")
	if output, err := resetCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("git reset --hard failed: %w\n%s", err, output)
	}
	cleanCmd = gitCmd(ctx.RepoDir, "clean", "-fd")
	if output, err := cleanCmd.CombinedOutput(); err != nil {
		return fmt.Errorf("git clean -fd failed: %w\n%s", err, output)
	}

	progressCallback("commit", "Updating .commit to upstream/main...")
	hashOut, err := gitCmd(ctx.RepoDir, "rev-parse", "upstream/main").Output()
	if err != nil {
		return fmt.Errorf("failed to resolve upstream/main commit: %w", err)
	}
	if err := updateCommitFile(ctx, strings.TrimSpace(string(hashOut))); err != nil {
		return fmt.Errorf("failed to write .commit: %w", err)
	}

	progressCallback("complete", "Force sync completed")
	return nil
}

// CanRevertCommits checks whether all commit SHAs are reachable from HEAD.
func CanRevertCommits(commits []string) bool {
	if len(commits) == 0 {
		return false
	}
	if err := checkGitInstalled(); err != nil {
		return false
	}
	ctx, err := resolveExistingRepoContext()
	if err != nil {
		return false
	}
	for _, sha := range commits {
		sha = strings.TrimSpace(sha)
		if sha == "" {
			continue
		}
		if err := gitCmd(ctx.RepoDir, "merge-base", "--is-ancestor", sha, "HEAD").Run(); err != nil {
			return false
		}
	}
	return true
}

func checkGitInstalled() error {
	cmd := newCommand("git", "--version")
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("git is not installed or not in PATH")
	}
	return nil
}

// ensureUpstreamRemote checks if upstream remote exists and adds it if not
func ensureUpstreamRemote(repoDir string) error {
	// Check if upstream remote exists
	checkCmd := gitCmd(repoDir, "remote", "get-url", "upstream")
	output, err := checkCmd.Output()

	if err != nil {
		// upstream doesn't exist, add it
		upstreamURL := upstreamRemoteURL()
		addCmd := gitCmd(repoDir, "remote", "add", "upstream", upstreamURL)
		if output, err := addCmd.CombinedOutput(); err != nil {
			return fmt.Errorf("failed to add upstream remote: %w\nOutput: %s", err, string(output))
		}
		return nil
	}

	// Verify it's pointing to the correct URL
	currentURL := strings.TrimSpace(string(output))
	expectedURL := upstreamRemoteURL()
	expectedSlug := upstreamSlug()

	if expectedSlug != "" && !strings.Contains(currentURL, expectedSlug) {
		// Update to correct URL
		setCmd := gitCmd(repoDir, "remote", "set-url", "upstream", expectedURL)
		if output, err := setCmd.CombinedOutput(); err != nil {
			return fmt.Errorf("failed to update upstream remote URL: %w\nOutput: %s", err, string(output))
		}
	}

	return nil
}
