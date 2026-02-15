package updater

import (
	"bytes"
	"fmt"
	"strings"
)

type AppliedPRInfo struct {
	Commits []string
}

// ResolveAppliedPRs computes applied PRs by comparing PR commits against the current Git history.
func ResolveAppliedPRs(prs []PullRequest) (map[int]AppliedPRInfo, error) {
	if len(prs) == 0 {
		return map[int]AppliedPRInfo{}, nil
	}

	ctx, err := resolveExistingRepoContext()
	if err != nil {
		return map[int]AppliedPRInfo{}, nil
	}
	return resolveAppliedPRsInRepo(ctx.RepoDir, prs)
}

// ResolveAppliedPRInfo computes applied commit SHAs for a single PR.
func ResolveAppliedPRInfo(prNumber int) (AppliedPRInfo, bool, error) {
	if prNumber <= 0 {
		return AppliedPRInfo{}, false, fmt.Errorf("invalid PR number: %d", prNumber)
	}

	ctx, err := resolveExistingRepoContext()
	if err != nil {
		return AppliedPRInfo{}, false, err
	}
	return ResolveAppliedPRInfoInRepo(ctx.RepoDir, prNumber)
}

// ResolveAppliedPRsForNumbers resolves applied PRs for a list of PR numbers.
func ResolveAppliedPRsForNumbers(prNumbers []int) (map[int]AppliedPRInfo, error) {
	if len(prNumbers) == 0 {
		return map[int]AppliedPRInfo{}, nil
	}

	ctx, err := resolveExistingRepoContext()
	if err != nil {
		return map[int]AppliedPRInfo{}, nil
	}

	return ResolveAppliedPRsForNumbersInRepo(ctx.RepoDir, prNumbers)
}

// ResolveAppliedPRsForNumbersInRepo resolves applied PRs for a list of PR numbers in the provided repo.
func ResolveAppliedPRsForNumbersInRepo(repoDir string, prNumbers []int) (map[int]AppliedPRInfo, error) {
	if len(prNumbers) == 0 {
		return map[int]AppliedPRInfo{}, nil
	}

	prs := make([]PullRequest, 0, len(prNumbers))
	for _, prNumber := range prNumbers {
		if prNumber <= 0 {
			continue
		}
		pr, err := GetPR(prNumber)
		if err != nil {
			continue
		}
		prs = append(prs, pr)
	}

	return resolveAppliedPRsInRepo(repoDir, prs)
}

// ResolveAppliedPRInfoInRepo computes applied commit SHAs for a single PR in the provided repo.
func ResolveAppliedPRInfoInRepo(repoDir string, prNumber int) (AppliedPRInfo, bool, error) {
	if prNumber <= 0 {
		return AppliedPRInfo{}, false, fmt.Errorf("invalid PR number: %d", prNumber)
	}
	if err := checkGitInstalled(); err != nil {
		return AppliedPRInfo{}, false, err
	}
	if strings.TrimSpace(repoDir) == "" {
		return AppliedPRInfo{}, false, fmt.Errorf("repository directory is required")
	}

	if err := ensureUpstreamRemote(repoDir); err != nil {
		return AppliedPRInfo{}, false, err
	}
	if err := gitCmd(repoDir, "fetch", "upstream", "main").Run(); err != nil {
		return AppliedPRInfo{}, false, err
	}

	resolver, err := newAppliedPRResolver(repoDir)
	if err != nil {
		return AppliedPRInfo{}, false, err
	}

	pr, err := GetPR(prNumber)
	if err != nil {
		return AppliedPRInfo{}, false, err
	}

	return resolver.resolve(pr)
}

func resolveAppliedPRsInRepo(repoDir string, prs []PullRequest) (map[int]AppliedPRInfo, error) {
	if len(prs) == 0 {
		return map[int]AppliedPRInfo{}, nil
	}
	if err := checkGitInstalled(); err != nil {
		return map[int]AppliedPRInfo{}, nil
	}
	if strings.TrimSpace(repoDir) == "" {
		return map[int]AppliedPRInfo{}, nil
	}

	if err := ensureUpstreamRemote(repoDir); err != nil {
		return nil, err
	}
	if err := gitCmd(repoDir, "fetch", "upstream", "main").Run(); err != nil {
		return nil, err
	}

	resolver, err := newAppliedPRResolver(repoDir)
	if err != nil {
		return nil, err
	}

	result := make(map[int]AppliedPRInfo)
	for _, pr := range prs {
		info, ok, err := resolver.resolve(pr)
		if err != nil {
			continue
		}
		if ok {
			result[pr.Number] = info
		}
	}

	return result, nil
}

type appliedPRResolver struct {
	repoDir       string
	patchToCommit map[string]string
	cherryPickMap map[string]string
	reverted      map[string]struct{}
}

func newAppliedPRResolver(repoDir string) (*appliedPRResolver, error) {
	aheadCommits, err := listAheadCommits(repoDir)
	if err != nil {
		return nil, err
	}

	patchToCommit := make(map[string]string, len(aheadCommits))
	cherryPickMap := make(map[string]string, len(aheadCommits))
	reverted := make(map[string]struct{}, len(aheadCommits))
	for _, sha := range aheadCommits {
		if patchID, err := patchIDForCommit(repoDir, sha); err == nil && patchID != "" {
			if _, exists := patchToCommit[patchID]; !exists {
				patchToCommit[patchID] = sha
			}
		}

		if msg, err := commitMessageForCommit(repoDir, sha); err == nil && msg != "" {
			if original := extractCherryPickedFrom(msg); original != "" {
				key := strings.ToLower(original)
				if _, exists := cherryPickMap[key]; !exists {
					cherryPickMap[key] = sha
				}
			}
			if revertedSHA := extractRevertedCommit(msg); revertedSHA != "" {
				reverted[strings.ToLower(revertedSHA)] = struct{}{}
			}
		}
	}

	return &appliedPRResolver{
		repoDir:       repoDir,
		patchToCommit: patchToCommit,
		cherryPickMap: cherryPickMap,
		reverted:      reverted,
	}, nil
}

func (r *appliedPRResolver) resolve(pr PullRequest) (AppliedPRInfo, bool, error) {
	if pr.Number <= 0 {
		return AppliedPRInfo{}, false, nil
	}

	commits, err := prCommitSHAs(r.repoDir, pr)
	if err != nil {
		return AppliedPRInfo{}, false, err
	}
	if len(commits) == 0 {
		return AppliedPRInfo{}, false, nil
	}

	applied := make([]string, 0, len(commits))
	relevant := 0
	for _, sha := range commits {
		sha = strings.TrimSpace(sha)
		if sha == "" {
			continue
		}
		if localSHA, ok := r.cherryPickMap[strings.ToLower(sha)]; ok {
			if r.isReverted(localSHA) {
				return AppliedPRInfo{}, false, nil
			}
			applied = append(applied, localSHA)
			relevant++
			continue
		}

		patchID, err := patchIDForCommit(r.repoDir, sha)
		if err != nil {
			return AppliedPRInfo{}, false, nil
		}
		if patchID == "" {
			// Ignore merge/empty commits that have no diff.
			continue
		}
		relevant++
		localSHA, ok := r.patchToCommit[patchID]
		if !ok {
			return AppliedPRInfo{}, false, nil
		}
		if r.isReverted(localSHA) {
			return AppliedPRInfo{}, false, nil
		}
		applied = append(applied, localSHA)
	}

	if relevant == 0 {
		return AppliedPRInfo{}, false, nil
	}

	return AppliedPRInfo{Commits: applied}, true, nil
}

func (r *appliedPRResolver) isReverted(sha string) bool {
	if sha == "" {
		return false
	}
	_, ok := r.reverted[strings.ToLower(sha)]
	return ok
}

func listAheadCommits(repoDir string) ([]string, error) {
	out, err := gitCmd(repoDir, "rev-list", "upstream/main..HEAD").Output()
	if err != nil {
		return nil, err
	}
	lines := strings.Split(strings.TrimSpace(string(out)), "\n")
	commits := make([]string, 0, len(lines))
	for _, line := range lines {
		line = strings.TrimSpace(line)
		if line == "" {
			continue
		}
		commits = append(commits, line)
	}
	return commits, nil
}

func patchIDForCommit(repoDir, sha string) (string, error) {
	sha = strings.TrimSpace(sha)
	if sha == "" {
		return "", nil
	}

	showCmd := gitCmd(repoDir, "show", "--pretty=format:", sha)
	showOut, err := showCmd.Output()
	if err != nil {
		return "", err
	}
	if len(showOut) == 0 {
		return "", nil
	}

	patchCmd := gitCmd(repoDir, "patch-id", "--stable")
	patchCmd.Stdin = bytes.NewReader(showOut)
	patchOut, err := patchCmd.Output()
	if err != nil {
		return "", err
	}

	fields := strings.Fields(string(patchOut))
	if len(fields) == 0 {
		return "", nil
	}
	return fields[0], nil
}

func commitMessageForCommit(repoDir, sha string) (string, error) {
	sha = strings.TrimSpace(sha)
	if sha == "" {
		return "", nil
	}

	msgOut, err := gitCmd(repoDir, "show", "-s", "--format=%B", sha).Output()
	if err != nil {
		return "", err
	}
	return strings.TrimSpace(string(msgOut)), nil
}

func extractCherryPickedFrom(message string) string {
	const needle = "cherry picked from commit "
	lower := strings.ToLower(message)
	idx := strings.Index(lower, needle)
	if idx == -1 {
		return ""
	}
	start := idx + len(needle)
	if start >= len(message) {
		return ""
	}

	rest := message[start:]
	hash := make([]rune, 0, 40)
	for _, r := range rest {
		if (r >= '0' && r <= '9') || (r >= 'a' && r <= 'f') || (r >= 'A' && r <= 'F') {
			hash = append(hash, r)
			continue
		}
		break
	}

	if len(hash) < 7 {
		return ""
	}
	return string(hash)
}

func extractRevertedCommit(message string) string {
	const needle = "this reverts commit "
	lower := strings.ToLower(message)
	idx := strings.Index(lower, needle)
	if idx == -1 {
		return ""
	}
	start := idx + len(needle)
	if start >= len(message) {
		return ""
	}

	rest := message[start:]
	hash := make([]rune, 0, 40)
	for _, r := range rest {
		if (r >= '0' && r <= '9') || (r >= 'a' && r <= 'f') || (r >= 'A' && r <= 'F') {
			hash = append(hash, r)
			continue
		}
		break
	}

	if len(hash) < 7 {
		return ""
	}
	return string(hash)
}

func prCommitSHAs(repoDir string, pr PullRequest) ([]string, error) {
	fetchCmd := gitCmd(repoDir, "fetch", "upstream", fmt.Sprintf("pull/%d/head", pr.Number))
	if output, err := fetchCmd.CombinedOutput(); err != nil {
		return nil, fmt.Errorf("failed to fetch PR #%d head: %w\n%s", pr.Number, err, strings.TrimSpace(string(output)))
	}

	headSHA := strings.TrimSpace(pr.Head.SHA)
	if headSHA == "" {
		headOut, err := gitCmd(repoDir, "rev-parse", "FETCH_HEAD").Output()
		if err != nil {
			return nil, fmt.Errorf("failed to resolve PR #%d head: %w", pr.Number, err)
		}
		headSHA = strings.TrimSpace(string(headOut))
	}
	if headSHA == "" {
		return nil, fmt.Errorf("failed to resolve PR #%d head commit", pr.Number)
	}

	baseOut, err := gitCmd(repoDir, "merge-base", "upstream/main", headSHA).Output()
	if err != nil {
		return nil, fmt.Errorf("failed to resolve merge base for PR #%d: %w", pr.Number, err)
	}
	baseSHA := strings.TrimSpace(string(baseOut))
	if baseSHA == "" {
		return nil, fmt.Errorf("failed to resolve merge base for PR #%d", pr.Number)
	}

	revCmd := gitCmd(repoDir, "rev-list", "--reverse", "--no-merges", fmt.Sprintf("%s..%s", baseSHA, headSHA))
	revOut, err := revCmd.Output()
	if err != nil {
		return nil, fmt.Errorf("failed to list commits for PR #%d: %w", pr.Number, err)
	}

	lines := strings.Split(strings.TrimSpace(string(revOut)), "\n")
	commits := make([]string, 0, len(lines))
	for _, line := range lines {
		line = strings.TrimSpace(line)
		if line == "" {
			continue
		}
		commits = append(commits, line)
	}
	return commits, nil
}

// MarkPRApplied is a no-op; applied PRs are derived from Git history.
func MarkPRApplied(prNumber int, commits []string) error {
	return nil
}

// RemoveAppliedPR is a no-op; applied PRs are derived from Git history.
func RemoveAppliedPR(prNumber int) error {
	return nil
}

// PruneAppliedPRsForCommit is a no-op; applied PRs are derived from Git history.
func PruneAppliedPRsForCommit(repoDir, targetCommit string) error {
	return nil
}

// backupAppliedPRs is a no-op; applied PRs are derived from Git history.
func backupAppliedPRs(backupPath string, logFn func(string)) {}

// restoreAppliedPRsFromBackup is a no-op; applied PRs are derived from Git history.
func restoreAppliedPRsFromBackup(backupPath string) (bool, error) {
	return false, nil
}
