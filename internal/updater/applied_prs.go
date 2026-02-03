package updater

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strconv"
)

type appliedPRState struct {
	Applied []int                      `json:"applied,omitempty"`
	PRs     map[string]appliedPRRecord `json:"prs,omitempty"`
}

type appliedPRRecord struct {
	Commits []string `json:"commits,omitempty"`
}

type AppliedPRInfo struct {
	Commits []string
}

func appliedPRsPath() (string, error) {
	installDir, err := resolveInstallDir()
	if err != nil {
		return "", err
	}
	return filepath.Join(installDir, "applied_prs.json"), nil
}

// LoadAppliedPRs returns PRs recorded as applied along with commit SHAs when available.
func LoadAppliedPRs() (map[int]AppliedPRInfo, error) {
	path, err := appliedPRsPath()
	if err != nil {
		return nil, err
	}
	data, err := os.ReadFile(path)
	if err != nil {
		if os.IsNotExist(err) {
			return map[int]AppliedPRInfo{}, nil
		}
		return nil, err
	}

	var state appliedPRState
	if err := json.Unmarshal(data, &state); err != nil {
		return nil, err
	}

	result := make(map[int]AppliedPRInfo)
	for key, record := range state.PRs {
		n, err := strconv.Atoi(key)
		if err != nil || n <= 0 {
			continue
		}
		result[n] = AppliedPRInfo{Commits: append([]string(nil), record.Commits...)}
	}

	if len(result) == 0 && len(state.Applied) > 0 {
		for _, n := range state.Applied {
			if n > 0 {
				result[n] = AppliedPRInfo{}
			}
		}
	}

	return result, nil
}

// MarkPRApplied records a PR number as applied with the provided commit SHAs.
func MarkPRApplied(prNumber int, commits []string) error {
	if prNumber <= 0 {
		return fmt.Errorf("invalid PR number: %d", prNumber)
	}
	prs, err := LoadAppliedPRs()
	if err != nil {
		return err
	}
	info := prs[prNumber]
	if len(commits) > 0 {
		info.Commits = append([]string(nil), commits...)
	}
	prs[prNumber] = info
	return saveAppliedPRs(prs)
}

// RemoveAppliedPR removes a PR from the applied list.
func RemoveAppliedPR(prNumber int) error {
	if prNumber <= 0 {
		return fmt.Errorf("invalid PR number: %d", prNumber)
	}
	prs, err := LoadAppliedPRs()
	if err != nil {
		return err
	}
	if _, ok := prs[prNumber]; !ok {
		return nil
	}
	delete(prs, prNumber)
	return saveAppliedPRs(prs)
}

func saveAppliedPRs(prs map[int]AppliedPRInfo) error {
	path, err := appliedPRsPath()
	if err != nil {
		return err
	}

	nums := make([]int, 0, len(prs))
	for n := range prs {
		nums = append(nums, n)
	}
	sort.Ints(nums)

	state := appliedPRState{
		Applied: nums,
		PRs:     make(map[string]appliedPRRecord, len(prs)),
	}
	for _, n := range nums {
		info := prs[n]
		state.PRs[strconv.Itoa(n)] = appliedPRRecord{Commits: info.Commits}
	}

	payload, err := json.Marshal(state)
	if err != nil {
		return err
	}

	tmp := path + ".tmp"
	if err := os.WriteFile(tmp, payload, 0o644); err != nil {
		return err
	}
	return os.Rename(tmp, path)
}
