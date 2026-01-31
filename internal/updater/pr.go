package updater

import (
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"strings"
	"time"
)

const (
	upstreamOwner = "kwader2k"
	upstreamRepo  = "koolo"
)

var githubHTTPClient = &http.Client{Timeout: 15 * time.Second}

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

// GetUpstreamPRs fetches open pull requests from upstream repository
func GetUpstreamPRs(state string, limit int) ([]PullRequest, error) {
	if state == "" {
		state = "open"
	}
	if limit <= 0 || limit > 100 {
		limit = 30
	}

	url := fmt.Sprintf("https://api.github.com/repos/%s/%s/pulls?state=%s&per_page=%d&sort=updated&direction=desc",
		upstreamOwner, upstreamRepo, state, limit)

	req, err := http.NewRequest(http.MethodGet, url, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to build PR request: %w", err)
	}
	req.Header.Set("User-Agent", "koolo-updater")

	resp, err := githubHTTPClient.Do(req)
	if err != nil {
		return nil, fmt.Errorf("failed to fetch PRs: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		body, _ := io.ReadAll(io.LimitReader(resp.Body, 2048))
		bodyText := strings.TrimSpace(string(body))
		if bodyText != "" {
			return nil, fmt.Errorf("GitHub API returned status %d: %s", resp.StatusCode, bodyText)
		}
		return nil, fmt.Errorf("GitHub API returned status %d", resp.StatusCode)
	}

	var prs []PullRequest
	if err := json.NewDecoder(resp.Body).Decode(&prs); err != nil {
		return nil, fmt.Errorf("failed to decode PR list: %w", err)
	}

	return prs, nil
}

// GetPRCommits fetches all commits for a specific PR
func GetPRCommits(prNumber int) ([]PRCommit, error) {
	url := fmt.Sprintf("https://api.github.com/repos/%s/%s/pulls/%d/commits",
		upstreamOwner, upstreamRepo, prNumber)

	req, err := http.NewRequest(http.MethodGet, url, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to build commit request: %w", err)
	}
	req.Header.Set("User-Agent", "koolo-updater")

	resp, err := githubHTTPClient.Do(req)
	if err != nil {
		return nil, fmt.Errorf("failed to fetch PR commits: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		body, _ := io.ReadAll(io.LimitReader(resp.Body, 2048))
		bodyText := strings.TrimSpace(string(body))
		if bodyText != "" {
			return nil, fmt.Errorf("GitHub API returned status %d: %s", resp.StatusCode, bodyText)
		}
		return nil, fmt.Errorf("GitHub API returned status %d", resp.StatusCode)
	}

	var commits []PRCommit
	if err := json.NewDecoder(resp.Body).Decode(&commits); err != nil {
		return nil, fmt.Errorf("failed to decode commit list: %w", err)
	}

	return commits, nil
}

// CherryPickPR applies commits from a PR using git cherry-pick
func (u *Updater) CherryPickPR(prNumber int, progressCallback func(message string)) (*CherryPickResult, error) {
	result := &CherryPickResult{
		PRNumber:   prNumber,
		Applied:    make([]string, 0),
		Conflicted: make([]string, 0),
	}

	if progressCallback == nil {
		progressCallback = func(message string) {}
	}

	// Ensure upstream remote is configured
	ctx, err := resolveRepoContext()
	if err != nil {
		return nil, err
	}

	if err := ensureUpstreamRemote(ctx.RepoDir); err != nil {
		return nil, err
	}

	statusCmd := gitCmd(ctx.RepoDir, "status", "--porcelain")
	statusOut, err := statusCmd.Output()
	if err != nil {
		return nil, fmt.Errorf("failed to check working tree: %w", err)
	}
	if strings.TrimSpace(string(statusOut)) != "" {
		progressCallback("Working tree has local changes; proceeding with cherry-pick")
	}

	progressCallback("Using current branch for cherry-pick...")

	// Fetch latest from upstream
	progressCallback(fmt.Sprintf("Fetching PR #%d from upstream...", prNumber))
	fetchCmd := gitCmd(ctx.RepoDir, "fetch", "upstream")
	if err := fetchCmd.Run(); err != nil {
		return nil, fmt.Errorf("git fetch failed: %w", err)
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

	// Apply each commit
	appliedCommits := make([]string, 0, len(commits))
	for i, commit := range commits {
		shortSHA := commit.SHA[:7]
		shortMsg := commit.Commit.Message
		if len(shortMsg) > 60 {
			shortMsg = shortMsg[:60] + "..."
		}
		firstLine := strings.Split(shortMsg, "\n")[0]

		progressCallback(fmt.Sprintf("[%d/%d] Cherry-picking %s: %s", i+1, len(commits), shortSHA, firstLine))

		// Try to cherry-pick
		cherryPickCmd := gitCmd(ctx.RepoDir, "cherry-pick", commit.SHA)
		output, err := cherryPickCmd.CombinedOutput()
		outputStr := string(output)

		if err != nil {
			// Skip merge commits (e.g. merge-from-main) that cannot be cherry-picked without -m
			if strings.Contains(outputStr, "merge but no -m option was given") {
				_ = gitCmd(ctx.RepoDir, "cherry-pick", "--abort").Run()
				progressCallback(fmt.Sprintf("Skipped merge commit %s", shortSHA))
				continue
			}

			// Check if it's already applied (empty commit)
			if strings.Contains(outputStr, "nothing to commit") ||
				strings.Contains(outputStr, "empty commit") ||
				strings.Contains(outputStr, "The previous cherry-pick is now empty") {
				progressCallback(fmt.Sprintf("Skipped %s (already applied)", shortSHA))

				// Skip this commit but continue with others
				skipCmd := gitCmd(ctx.RepoDir, "cherry-pick", "--skip")
				skipCmd.Run()
				continue
			}

			// Check if it's a conflict
			if strings.Contains(outputStr, "conflict") || strings.Contains(outputStr, "CONFLICT") {
				progressCallback(fmt.Sprintf("Conflict detected on %s, aborting...", shortSHA))

				// Abort the cherry-pick
				abortCmd := gitCmd(ctx.RepoDir, "cherry-pick", "--abort")
				abortCmd.Run()

				result.Conflicted = append(result.Conflicted, shortSHA)
				result.Success = false
				result.Error = fmt.Sprintf("Conflict on commit %s: %s", shortSHA, firstLine)

				// Stop processing this PR
				return result, nil
			}

			// Other error - show detailed message
			_ = gitCmd(ctx.RepoDir, "cherry-pick", "--abort").Run()
			result.Success = false
			result.Error = fmt.Sprintf("Failed to cherry-pick %s: %v\nOutput: %s", shortSHA, err, outputStr)
			return result, nil
		}

		appliedSHA := commit.SHA
		if headOut, err := gitCmd(ctx.RepoDir, "rev-parse", "HEAD").Output(); err == nil {
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

	result.Success = true
	progressCallback(fmt.Sprintf("Successfully applied all %d commits from PR #%d", len(commits), prNumber))
	if err := MarkPRApplied(prNumber, appliedCommits); err != nil {
		progressCallback(fmt.Sprintf("Warning: failed to record PR #%d as applied: %v", prNumber, err))
	}

	return result, nil
}

// CherryPickMultiplePRs applies multiple PRs in sequence
func (u *Updater) CherryPickMultiplePRs(prNumbers []int, progressCallback func(message string)) ([]CherryPickResult, error) {
	if progressCallback == nil {
		progressCallback = func(message string) {}
	}

	u.resetStatus("cherry-pick")

	results := make([]CherryPickResult, 0)

	for i, prNumber := range prNumbers {
		progressCallback(fmt.Sprintf("\n=== Processing PR #%d (%d/%d) ===", prNumber, i+1, len(prNumbers)))

		result, err := u.CherryPickPR(prNumber, progressCallback)
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
