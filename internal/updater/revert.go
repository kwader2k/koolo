package updater

import (
	"fmt"
	"strings"
)

type RevertResult struct {
	PRNumber  int      `json:"prNumber"`
	Success   bool     `json:"success"`
	Error     string   `json:"error,omitempty"`
	Reverted  []string `json:"reverted,omitempty"`
	Conflicts []string `json:"conflicted,omitempty"`
}

// RevertPR reverts commits previously applied for a PR (most recent first).
func (u *Updater) RevertPR(prNumber int, progressCallback func(message string)) (result *RevertResult, err error) {
	if progressCallback == nil {
		progressCallback = func(message string) {}
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
	stashCreated := false
	if strings.TrimSpace(string(statusOut)) != "" {
		progressCallback("Working tree has local changes; stashing before revert")
		stashCmd := gitCmd(ctx.RepoDir, "stash", "push", "-u", "-m", "koolo-updater-revert")
		stashOut, stashErr := stashCmd.CombinedOutput()
		if stashErr != nil {
			return nil, fmt.Errorf("failed to stash local changes: %w\n%s", stashErr, string(stashOut))
		}
		outLower := strings.ToLower(string(stashOut))
		stashCreated = !strings.Contains(outLower, "no local changes")
	}
	defer func() {
		if !stashCreated {
			return
		}
		popCmd := gitCmd(ctx.RepoDir, "stash", "pop")
		popOut, popErr := popCmd.CombinedOutput()
		if popErr == nil {
			return
		}
		popMsg := fmt.Errorf("failed to restore local changes: %w\n%s", popErr, string(popOut))
		if err == nil {
			err = popMsg
		} else {
			err = fmt.Errorf("%v; %w", err, popMsg)
		}
	}()

	applied, err := LoadAppliedPRs()
	if err != nil {
		return nil, err
	}
	info, ok := applied[prNumber]
	if !ok || len(info.Commits) == 0 {
		return nil, fmt.Errorf("no applied commits recorded for PR #%d", prNumber)
	}

	progressCallback(fmt.Sprintf("Reverting PR #%d...", prNumber))

	for i := len(info.Commits) - 1; i >= 0; i-- {
		sha := info.Commits[i]
		shortSHA := sha
		if len(shortSHA) > 7 {
			shortSHA = shortSHA[:7]
		}

		progressCallback(fmt.Sprintf("Reverting %s...", shortSHA))
		revertCmd := gitCmd(ctx.RepoDir, "revert", "--no-edit", sha)
		output, err := revertCmd.CombinedOutput()
		outputStr := string(output)

		if err != nil {
			if strings.Contains(outputStr, "merge but no -m option was given") {
				_ = gitCmd(ctx.RepoDir, "revert", "--abort").Run()
				return nil, fmt.Errorf("cannot revert merge commit %s without -m", shortSHA)
			}

			if strings.Contains(outputStr, "nothing to commit") ||
				strings.Contains(outputStr, "The previous cherry-pick is now empty") ||
				strings.Contains(outputStr, "The previous cherry-pick is empty") ||
				strings.Contains(outputStr, "nothing added to commit but untracked files present") {
				_ = gitCmd(ctx.RepoDir, "revert", "--skip").Run()
				progressCallback(fmt.Sprintf("Skipped %s (already reverted)", shortSHA))
				continue
			}

			if strings.Contains(outputStr, "conflict") || strings.Contains(outputStr, "CONFLICT") {
				_ = gitCmd(ctx.RepoDir, "revert", "--abort").Run()
				result.Conflicts = append(result.Conflicts, shortSHA)
				result.Success = false
				result.Error = fmt.Sprintf("Conflict while reverting %s", shortSHA)
				progressCallback(result.Error)
				return result, fmt.Errorf(result.Error)
			}

			_ = gitCmd(ctx.RepoDir, "revert", "--abort").Run()
			return nil, fmt.Errorf("failed to revert %s: %v\nOutput: %s", shortSHA, err, outputStr)
		}

		result.Reverted = append(result.Reverted, shortSHA)
		progressCallback(fmt.Sprintf("Reverted %s", shortSHA))
	}

	if err := RemoveAppliedPR(prNumber); err != nil {
		progressCallback(fmt.Sprintf("Warning: failed to update applied PRs: %v", err))
	}

	result.Success = true
	progressCallback(fmt.Sprintf("Successfully reverted PR #%d", prNumber))
	return result, nil
}
