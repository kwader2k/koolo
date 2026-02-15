package updater

import (
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"time"
)

var githubHTTPClient = &http.Client{Timeout: 15 * time.Second}

const discardLocalChangesPrefix = "CONFIRM_DISCARD_LOCAL_CHANGES"
const confirmResetUpstreamPrefix = "CONFIRM_RESET_UPSTREAM"

type PullRequest struct {
	Number    int       `json:"number"`
	Title     string    `json:"title"`
	State     string    `json:"state"`
	CreatedAt time.Time `json:"created_at"`
	UpdatedAt time.Time `json:"updated_at"`
	User      struct {
		Login string `json:"login"`
	} `json:"user"`
	Head struct {
		SHA string `json:"sha"`
	} `json:"head"`
	Commits   int  `json:"commits"`
	Applied   bool `json:"applied"`
	CanRevert bool `json:"canRevert"`
}

type PRCommit struct {
	SHA    string `json:"sha"`
	Commit struct {
		Message string `json:"message"`
	} `json:"commit"`
}

type CherryPickResult struct {
	PRNumber   int      `json:"prNumber"`
	Success    bool     `json:"success"`
	Error      string   `json:"error,omitempty"`
	Applied    []string `json:"applied,omitempty"`    // Successfully applied commits
	Conflicted []string `json:"conflicted,omitempty"` // Commits that had conflicts
}

type CherryPickOptions struct {
	AllowDiscardLocalChanges bool
}

type discardLocalChangesError struct {
	message string
}

func (e discardLocalChangesError) Error() string {
	if e.message == "" {
		return discardLocalChangesPrefix
	}
	return fmt.Sprintf("%s: %s", discardLocalChangesPrefix, e.message)
}

func newDiscardLocalChangesError(message string) error {
	return discardLocalChangesError{message: message}
}

type confirmResetUpstreamError struct {
	message string
}

func (e confirmResetUpstreamError) Error() string {
	if e.message == "" {
		return confirmResetUpstreamPrefix
	}
	return fmt.Sprintf("%s: %s", confirmResetUpstreamPrefix, e.message)
}

func newConfirmResetUpstreamError(message string) error {
	return confirmResetUpstreamError{message: message}
}

// GetUpstreamPRs fetches open pull requests from upstream repository
func GetUpstreamPRs(state string, limit int) ([]PullRequest, error) {
	if state == "" {
		state = "open"
	}
	if limit <= 0 || limit > 100 {
		limit = 30
	}

	url := fmt.Sprintf("https://api.github.com/repos/%s/%s/pulls?state=%s&per_page=%d&sort=updated&direction=desc",
		upstreamOwner(), upstreamRepo(), state, limit)

	req, err := http.NewRequest(http.MethodGet, url, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to build PR request: %w", err)
	}
	req.Header.Set("User-Agent", "koolo-updater")
	applyGitHubAuth(req)

	resp, err := githubHTTPClient.Do(req)
	if err != nil {
		return nil, fmt.Errorf("failed to fetch PRs: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		body, _ := io.ReadAll(io.LimitReader(resp.Body, 2048))
		return nil, formatGitHubAPIError(resp.StatusCode, body)
	}

	var prs []PullRequest
	if err := json.NewDecoder(resp.Body).Decode(&prs); err != nil {
		return nil, fmt.Errorf("failed to decode PR list: %w", err)
	}

	return prs, nil
}

// GetPR fetches a single pull request by number.
func GetPR(prNumber int) (PullRequest, error) {
	if prNumber <= 0 {
		return PullRequest{}, fmt.Errorf("invalid PR number: %d", prNumber)
	}

	url := fmt.Sprintf("https://api.github.com/repos/%s/%s/pulls/%d",
		upstreamOwner(), upstreamRepo(), prNumber)

	req, err := http.NewRequest(http.MethodGet, url, nil)
	if err != nil {
		return PullRequest{}, fmt.Errorf("failed to build PR request: %w", err)
	}
	req.Header.Set("User-Agent", "koolo-updater")
	applyGitHubAuth(req)

	resp, err := githubHTTPClient.Do(req)
	if err != nil {
		return PullRequest{}, fmt.Errorf("failed to fetch PR: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		body, _ := io.ReadAll(io.LimitReader(resp.Body, 2048))
		return PullRequest{}, formatGitHubAPIError(resp.StatusCode, body)
	}

	var pr PullRequest
	if err := json.NewDecoder(resp.Body).Decode(&pr); err != nil {
		return PullRequest{}, fmt.Errorf("failed to decode PR: %w", err)
	}

	return pr, nil
}

// GetPRCommits fetches all commits for a specific PR
func GetPRCommits(prNumber int) ([]PRCommit, error) {
	url := fmt.Sprintf("https://api.github.com/repos/%s/%s/pulls/%d/commits",
		upstreamOwner(), upstreamRepo(), prNumber)

	req, err := http.NewRequest(http.MethodGet, url, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to build commit request: %w", err)
	}
	req.Header.Set("User-Agent", "koolo-updater")
	applyGitHubAuth(req)

	resp, err := githubHTTPClient.Do(req)
	if err != nil {
		return nil, fmt.Errorf("failed to fetch PR commits: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		body, _ := io.ReadAll(io.LimitReader(resp.Body, 2048))
		return nil, formatGitHubAPIError(resp.StatusCode, body)
	}

	var commits []PRCommit
	if err := json.NewDecoder(resp.Body).Decode(&commits); err != nil {
		return nil, fmt.Errorf("failed to decode commit list: %w", err)
	}

	return commits, nil
}

type githubAPIError struct {
	Message          string `json:"message"`
	DocumentationURL string `json:"documentation_url"`
}

func formatGitHubAPIError(statusCode int, body []byte) error {
	bodyText := strings.TrimSpace(string(body))
	if bodyText != "" {
		var payload githubAPIError
		if err := json.Unmarshal([]byte(bodyText), &payload); err == nil && payload.Message != "" {
			if payload.DocumentationURL != "" {
				return fmt.Errorf("GitHub API returned status %d: %s (%s)", statusCode, payload.Message, payload.DocumentationURL)
			}
			return fmt.Errorf("GitHub API returned status %d: %s", statusCode, payload.Message)
		}
	}
	return fmt.Errorf("GitHub API returned status %d", statusCode)
}

func applyGitHubAuth(req *http.Request) {
	token := strings.TrimSpace(os.Getenv("KOOLO_GITHUB_TOKEN"))
	if token == "" {
		token = strings.TrimSpace(os.Getenv("GITHUB_TOKEN"))
	}
	if token != "" {
		req.Header.Set("Authorization", "Bearer "+token)
	}
}

// CherryPickPR applies commits from a PR using git cherry-pick.
func (u *Updater) CherryPickPR(prNumber int, opts CherryPickOptions, progressCallback func(message string)) (*CherryPickResult, error) {
	result := &CherryPickResult{
		PRNumber:   prNumber,
		Applied:    make([]string, 0),
		Conflicted: make([]string, 0),
	}

	if progressCallback == nil {
		progressCallback = func(message string) {}
	}
	originalProgress := progressCallback
	progressCallback = func(message string) {
		originalProgress(message)
		appendOperationLog("cherry-pick", message)
	}

	// Ensure upstream remote is configured
	ctx, err := resolveRepoContext()
	if err != nil {
		return nil, err
	}

	if err := ensureUpstreamRemote(ctx.RepoDir); err != nil {
		return nil, err
	}

	// Fetch latest from upstream
	progressCallback(fmt.Sprintf("Fetching PR #%d from upstream...", prNumber))
	fetchCmd := gitCmd(ctx.RepoDir, "fetch", "upstream", "main")
	if err := fetchCmd.Run(); err != nil {
		return nil, fmt.Errorf("git fetch failed: %w", err)
	}

	statusCmd := gitCmd(ctx.RepoDir, "status", "--porcelain")
	statusOut, err := statusCmd.Output()
	if err != nil {
		return nil, fmt.Errorf("failed to check working tree: %w", err)
	}
	dirty := strings.TrimSpace(string(statusOut)) != ""

	if dirty && !opts.AllowDiscardLocalChanges {
		return nil, newDiscardLocalChangesError("Applying PRs requires a clean repository.")
	}

	if dirty && opts.AllowDiscardLocalChanges {
		progressCallback("Discarding local changes before applying PRs...")
		resetCmd := gitCmd(ctx.RepoDir, "reset", "--hard", "HEAD")
		if output, err := resetCmd.CombinedOutput(); err != nil {
			return nil, fmt.Errorf("git reset --hard failed: %w\n%s", err, output)
		}
		_ = gitCmd(ctx.RepoDir, "clean", "-fd").Run()
	}

	// Fetch PR head to ensure commits are available (supports forked PRs)
	progressCallback(fmt.Sprintf("Fetching PR #%d head ref...", prNumber))
	prFetchCmd := gitCmd(ctx.RepoDir, "fetch", "upstream", fmt.Sprintf("pull/%d/head", prNumber))
	if output, err := prFetchCmd.CombinedOutput(); err != nil {
		return nil, fmt.Errorf("failed to fetch PR #%d head: %w\nOutput: %s", prNumber, err, string(output))
	}

	// Get PR commits
	progressCallback(fmt.Sprintf("Getting commit list for PR #%d...", prNumber))
	commits, err := GetPRCommits(prNumber)
	if err != nil {
		return nil, err
	}

	if len(commits) == 0 {
		return nil, fmt.Errorf("no commits found in PR #%d", prNumber)
	}

	progressCallback(fmt.Sprintf("Found %d commit(s) to apply", len(commits)))

	tmpRoot := filepath.Join(ctx.RepoDir, ".tmp", "pr-worktrees")
	if err := os.MkdirAll(tmpRoot, 0o755); err != nil {
		return nil, fmt.Errorf("failed to create worktree directory: %w", err)
	}
	worktreeName := fmt.Sprintf("koolo-pr-%d-%d", prNumber, time.Now().UnixNano())
	worktreeDir := filepath.Join(tmpRoot, worktreeName)

	progressCallback("Creating isolated worktree for PR apply...")
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

	progressCallback("Syncing worktree with upstream/main...")
	mergeCmd := gitCmd(worktreeDir, "merge", "--no-edit", "upstream/main")
	mergeOutput, mergeErr := mergeCmd.CombinedOutput()
	if mergeErr != nil {
		mergeOutStr := string(mergeOutput)
		if strings.Contains(mergeOutStr, "CONFLICT") || strings.Contains(mergeOutStr, "Automatic merge failed") {
			if !opts.AllowDiscardLocalChanges {
				_ = gitCmd(worktreeDir, "merge", "--abort").Run()
				return nil, newDiscardLocalChangesError("Syncing with upstream/main would discard local changes.")
			}

			progressCallback("Merge conflict with upstream/main; resetting worktree to upstream/main...")
			_ = gitCmd(worktreeDir, "merge", "--abort").Run()
			resetCmd := gitCmd(worktreeDir, "reset", "--hard", "upstream/main")
			if output, err := resetCmd.CombinedOutput(); err != nil {
				return nil, fmt.Errorf("git reset --hard failed: %w\n%s", err, output)
			}
			_ = gitCmd(worktreeDir, "clean", "-fd").Run()
			if commitOut, err := gitCmd(ctx.RepoDir, "rev-parse", "upstream/main").Output(); err != nil {
				progressCallback(fmt.Sprintf("Warning: failed to resolve upstream/main for .commit update: %v", err))
			} else if err := updateCommitFile(ctx, strings.TrimSpace(string(commitOut))); err != nil {
				progressCallback(fmt.Sprintf("Warning: failed to update .commit: %v", err))
			} else {
				progressCallback("Updated .commit to upstream/main after conflict reset.")
			}
		} else {
			return nil, fmt.Errorf("git merge upstream/main failed: %w\n%s", mergeErr, mergeOutStr)
		}
	}

	// Apply each commit in the isolated worktree (retry once after reset)
	appliedCommits := make([]string, 0, len(commits))
applyAttempt:
	for attempt := 0; attempt < 2; attempt++ {
		if attempt > 0 {
			progressCallback("Retrying cherry-pick after upstream reset...")
		}
		appliedCommits = appliedCommits[:0]
		result.Applied = result.Applied[:0]
		result.Conflicted = result.Conflicted[:0]
		for i, commit := range commits {
			shortSHA := commit.SHA[:7]
			shortMsg := commit.Commit.Message
			if len(shortMsg) > 60 {
				shortMsg = shortMsg[:60] + "..."
			}
			firstLine := strings.Split(shortMsg, "\n")[0]

			progressCallback(fmt.Sprintf("[%d/%d] Cherry-picking %s: %s", i+1, len(commits), shortSHA, firstLine))

			runCherryPick := func() (string, error) {
				cherryPickCmd := gitCmd(worktreeDir, "cherry-pick", "-x", commit.SHA)
				output, err := cherryPickCmd.CombinedOutput()
				return string(output), err
			}

			outputStr, err := runCherryPick()
			if err != nil {
				if strings.Contains(outputStr, "merge but no -m option was given") {
					_ = gitCmd(worktreeDir, "cherry-pick", "--abort").Run()
					progressCallback(fmt.Sprintf("Skipped merge commit %s", shortSHA))
					continue
				}

				if strings.Contains(outputStr, "nothing to commit") ||
					strings.Contains(outputStr, "empty commit") ||
					strings.Contains(outputStr, "The previous cherry-pick is now empty") {
					progressCallback(fmt.Sprintf("Skipped %s (already applied)", shortSHA))
					_ = gitCmd(worktreeDir, "cherry-pick", "--skip").Run()
					continue
				}

				conflictFiles := ""
				if filesOut, filesErr := gitCmd(worktreeDir, "diff", "--name-only", "--diff-filter=U").Output(); filesErr == nil {
					conflictFiles = strings.TrimSpace(string(filesOut))
					if conflictFiles != "" {
						progressCallback(fmt.Sprintf("Conflicted files:\n%s", conflictFiles))
					}
				}

				writeLog := func(kind string, pickErr error) {
					detail := fmt.Sprintf("PR #%d %s on commit %s: %s", prNumber, kind, shortSHA, firstLine)
					if conflictFiles != "" {
						detail += fmt.Sprintf("\nConflicted files:\n%s", conflictFiles)
					}
					if strings.TrimSpace(outputStr) != "" {
						detail += fmt.Sprintf("\nCherry-pick output:\n%s", strings.TrimSpace(outputStr))
					}
					detail += fmt.Sprintf("\nError: %v", pickErr)
					appendOperationLog("cherry-pick", detail)
				}

				if strings.Contains(outputStr, "conflict") || strings.Contains(outputStr, "CONFLICT") || conflictFiles != "" {
					progressCallback(fmt.Sprintf("Conflict detected on %s, aborting...", shortSHA))
					writeLog("conflict", err)
					_ = gitCmd(worktreeDir, "cherry-pick", "--abort").Run()

					if !opts.AllowDiscardLocalChanges {
						return nil, newConfirmResetUpstreamError("Cherry-pick conflict detected. Reset to upstream/main and retry?")
					}

					if attempt == 0 {
						progressCallback("Resetting worktree to upstream/main after conflict...")
						resetCmd := gitCmd(worktreeDir, "reset", "--hard", "upstream/main")
						if output, resetErr := resetCmd.CombinedOutput(); resetErr != nil {
							return nil, fmt.Errorf("git reset --hard failed after cherry-pick conflict: %w\n%s", resetErr, output)
						}
						_ = gitCmd(worktreeDir, "clean", "-fd").Run()
						continue applyAttempt
					}

					result.Conflicted = append(result.Conflicted, shortSHA)
					result.Success = false
					if conflictFiles != "" {
						result.Error = fmt.Sprintf("Conflict on commit %s: %s\n%s", shortSHA, firstLine, conflictFiles)
					} else {
						result.Error = fmt.Sprintf("Conflict on commit %s: %s", shortSHA, firstLine)
					}
					return result, nil
				}

				writeLog("error", err)
				_ = gitCmd(worktreeDir, "cherry-pick", "--abort").Run()
				result.Success = false
				result.Error = fmt.Sprintf("Failed to cherry-pick %s: %v\nOutput: %s", shortSHA, err, outputStr)
				return result, nil
			}

			appliedSHA := commit.SHA
			if headOut, err := gitCmd(worktreeDir, "rev-parse", "HEAD").Output(); err == nil {
				headSHA := strings.TrimSpace(string(headOut))
				if headSHA != "" {
					appliedSHA = headSHA
				} else {
					progressCallback(fmt.Sprintf("Warning: unable to resolve applied commit for %s", shortSHA))
				}
			} else {
				progressCallback(fmt.Sprintf("Warning: failed to resolve applied commit for %s: %v", shortSHA, err))
			}
			appliedCommits = append(appliedCommits, appliedSHA)
			result.Applied = append(result.Applied, shortSHA)
			progressCallback(fmt.Sprintf("Applied %s", shortSHA))
		}

		headOut, err := gitCmd(worktreeDir, "rev-parse", "HEAD").Output()
		if err != nil {
			return nil, fmt.Errorf("failed to resolve worktree HEAD: %w", err)
		}
		worktreeHead := strings.TrimSpace(string(headOut))
		if worktreeHead == "" {
			return nil, fmt.Errorf("failed to resolve worktree HEAD commit")
		}

		progressCallback("Updating main branch with applied PR commits...")
		checkoutCmd := gitCmd(ctx.RepoDir, "checkout", "-B", "main", worktreeHead)
		if output, err := checkoutCmd.CombinedOutput(); err != nil {
			return nil, fmt.Errorf("failed to update main branch: %w\n%s", err, output)
		}
		resetCmd := gitCmd(ctx.RepoDir, "reset", "--hard", worktreeHead)
		if output, err := resetCmd.CombinedOutput(); err != nil {
			return nil, fmt.Errorf("git reset --hard failed applying PR result: %w\n%s", err, output)
		}
		_ = gitCmd(ctx.RepoDir, "clean", "-fd").Run()

		result.Success = true
		progressCallback(fmt.Sprintf("Successfully applied all %d commits from PR #%d", len(commits), prNumber))
		if err := MarkPRApplied(prNumber, appliedCommits); err != nil {
			progressCallback(fmt.Sprintf("Warning: failed to record PR #%d as applied: %v", prNumber, err))
		}

		return result, nil
	}

	return nil, fmt.Errorf("cherry-pick attempts exhausted")
}

// CherryPickMultiplePRs applies multiple PRs in sequence.
func (u *Updater) CherryPickMultiplePRs(prNumbers []int, opts CherryPickOptions, progressCallback func(message string)) ([]CherryPickResult, error) {
	if progressCallback == nil {
		progressCallback = func(message string) {}
	}

	u.resetStatus("cherry-pick")

	results := make([]CherryPickResult, 0)

	for i, prNumber := range prNumbers {
		progressCallback(fmt.Sprintf("\n=== Processing PR #%d (%d/%d) ===", prNumber, i+1, len(prNumbers)))

		result, err := u.CherryPickPR(prNumber, opts, progressCallback)
		if err != nil {
			// Fatal error (not a conflict)
			return results, err
		}

		results = append(results, *result)

		if !result.Success {
			progressCallback(fmt.Sprintf("Skipping PR #%d due to conflicts", prNumber))
		}
	}

	return results, nil
}
