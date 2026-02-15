package updater

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"
)

type RevertResult struct {
	PRNumber  int      `json:"prNumber"`
	Success   bool     `json:"success"`
	Error     string   `json:"error,omitempty"`
	Reverted  []string `json:"reverted,omitempty"`
	Conflicts []string `json:"conflicted,omitempty"`
}

// RevertPR reverts commits previously applied for a PR (most recent first).
func (u *Updater) RevertPR(prNumber int, allowDiscardLocalChanges bool, progressCallback func(message string)) (result *RevertResult, err error) {
	if progressCallback == nil {
		progressCallback = func(message string) {}
	}
	originalProgress := progressCallback
	progressCallback = func(message string) {
		originalProgress(message)
		appendOperationLog("revert", message)
	}

	result = &RevertResult{
		PRNumber: prNumber,
		Reverted: make([]string, 0),
	}

	if prNumber <= 0 {
		return nil, fmt.Errorf("invalid PR number: %d", prNumber)
	}

	ctx, err := resolveRepoContext()
	if err != nil {
		return nil, err
	}

	statusCmd := gitCmd(ctx.RepoDir, "status", "--porcelain")
	statusOut, err := statusCmd.Output()
	if err != nil {
		return nil, fmt.Errorf("failed to check working tree: %w", err)
	}
	dirty := strings.TrimSpace(string(statusOut)) != ""
	if dirty && !allowDiscardLocalChanges {
		return nil, newDiscardLocalChangesError("Reverting PRs requires a clean repository.")
	}
	if dirty && allowDiscardLocalChanges {
		progressCallback("Discarding local changes before revert...")
		resetCmd := gitCmd(ctx.RepoDir, "reset", "--hard", "HEAD")
		if output, err := resetCmd.CombinedOutput(); err != nil {
			return nil, fmt.Errorf("git reset --hard failed: %w\n%s", err, output)
		}
		_ = gitCmd(ctx.RepoDir, "clean", "-fd").Run()
	}

	info, ok, err := ResolveAppliedPRInfoInRepo(ctx.RepoDir, prNumber)
	if err != nil {
		return nil, err
	}
	if !ok || len(info.Commits) == 0 {
		return nil, fmt.Errorf("no applied commits found for PR #%d", prNumber)
	}

	progressCallback(fmt.Sprintf("Reverting PR #%d...", prNumber))

	tmpRoot := filepath.Join(ctx.RepoDir, ".tmp", "revert-worktrees")
	if err := os.MkdirAll(tmpRoot, 0o755); err != nil {
		return nil, fmt.Errorf("failed to create worktree directory: %w", err)
	}
	worktreeName := fmt.Sprintf("koolo-revert-%d-%d", prNumber, time.Now().UnixNano())
	worktreeDir := filepath.Join(tmpRoot, worktreeName)
	progressCallback("Creating isolated worktree for revert...")
	worktreeCmd := gitCmd(ctx.RepoDir, "worktree", "add", "-b", worktreeName, worktreeDir, "HEAD")
	if output, err := worktreeCmd.CombinedOutput(); err != nil {
		return nil, fmt.Errorf("failed to create worktree: %w\n%s", err, output)
	}
	defer func() {
		_ = gitCmd(ctx.RepoDir, "worktree", "remove", "--force", worktreeDir).Run()
		_ = gitCmd(ctx.RepoDir, "branch", "-D", worktreeName).Run()
		_ = os.RemoveAll(worktreeDir)
		cleanupTmpRoot(tmpRoot)
	}()

	for i := len(info.Commits) - 1; i >= 0; i-- {
		sha := info.Commits[i]
		shortSHA := sha
		if len(shortSHA) > 7 {
			shortSHA = shortSHA[:7]
		}

		progressCallback(fmt.Sprintf("Reverting %s...", shortSHA))
		revertCmd := gitCmd(worktreeDir, "revert", "--no-edit", sha)
		output, err := revertCmd.CombinedOutput()
		outputStr := string(output)

		if err != nil {
			if strings.Contains(outputStr, "merge but no -m option was given") {
				_ = gitCmd(worktreeDir, "revert", "--abort").Run()
				result.Success = false
				result.Error = fmt.Sprintf("Cannot revert merge commit %s without -m", shortSHA)
				progressCallback(result.Error)
				return result, fmt.Errorf(result.Error)
			}

			if strings.Contains(outputStr, "nothing to commit") ||
				strings.Contains(outputStr, "The previous cherry-pick is now empty") ||
				strings.Contains(outputStr, "The previous cherry-pick is empty") ||
				strings.Contains(outputStr, "nothing added to commit but untracked files present") {
				_ = gitCmd(worktreeDir, "revert", "--skip").Run()
				progressCallback(fmt.Sprintf("Skipped %s (already reverted)", shortSHA))
				continue
			}

			if strings.Contains(outputStr, "conflict") || strings.Contains(outputStr, "CONFLICT") {
				_ = gitCmd(worktreeDir, "revert", "--abort").Run()
				result.Conflicts = append(result.Conflicts, shortSHA)
				result.Success = false
				result.Error = fmt.Sprintf("Conflict while reverting %s", shortSHA)
				progressCallback(result.Error)
				return result, fmt.Errorf(result.Error)
			}

			_ = gitCmd(worktreeDir, "revert", "--abort").Run()
			result.Success = false
			result.Error = fmt.Sprintf("Failed to revert %s: %v\nOutput: %s", shortSHA, err, outputStr)
			return result, fmt.Errorf(result.Error)
		}

		result.Reverted = append(result.Reverted, shortSHA)
		progressCallback(fmt.Sprintf("Reverted %s", shortSHA))
	}

	headOut, err := gitCmd(worktreeDir, "rev-parse", "HEAD").Output()
	if err != nil {
		return nil, fmt.Errorf("failed to resolve worktree HEAD: %w", err)
	}
	worktreeHead := strings.TrimSpace(string(headOut))
	if worktreeHead == "" {
		return nil, fmt.Errorf("failed to resolve worktree HEAD commit")
	}

	checkoutCmd := gitCmd(ctx.RepoDir, "checkout", "-B", "main", worktreeHead)
	if output, err := checkoutCmd.CombinedOutput(); err != nil {
		return nil, fmt.Errorf("failed to update main branch: %w\n%s", err, output)
	}
	resetCmd := gitCmd(ctx.RepoDir, "reset", "--hard", worktreeHead)
	if output, err := resetCmd.CombinedOutput(); err != nil {
		return nil, fmt.Errorf("git reset --hard failed: %w\n%s", err, output)
	}
	_ = gitCmd(ctx.RepoDir, "clean", "-fd").Run()

	_ = RemoveAppliedPR(prNumber)

	result.Success = true
	progressCallback(fmt.Sprintf("Successfully reverted PR #%d", prNumber))
	return result, nil
}
