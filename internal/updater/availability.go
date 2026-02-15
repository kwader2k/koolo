package updater

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
)

type UpdaterGate struct {
	ForceSyncOnly bool
	Reason        string
}

func GetUpdaterGate() UpdaterGate {
	commitPath, err := resolveCommitPath()
	if err != nil {
		if os.IsNotExist(err) {
			return UpdaterGate{
				ForceSyncOnly: true,
				Reason:        "Updater is locked: .commit is missing. Enable Force Sync to initialize from upstream/main.",
			}
		}
		return UpdaterGate{
			ForceSyncOnly: true,
			Reason:        fmt.Sprintf("Updater is locked: failed to resolve .commit (%v). Enable Force Sync to initialize.", err),
		}
	}

	commitBytes, err := os.ReadFile(commitPath)
	if err != nil {
		return UpdaterGate{
			ForceSyncOnly: true,
			Reason:        fmt.Sprintf("Updater is locked: failed to read .commit (%v). Enable Force Sync to initialize.", err),
		}
	}

	commitHash := strings.TrimSpace(string(commitBytes))
	if commitHash == "" {
		return UpdaterGate{
			ForceSyncOnly: true,
			Reason:        "Updater is locked: .commit is empty. Enable Force Sync to initialize from upstream/main.",
		}
	}

	ctx, err := resolveExistingRepoContext()
	if err != nil {
		return UpdaterGate{
			ForceSyncOnly: true,
			Reason:        "Updater is locked: git repository is missing. Enable Force Sync to initialize from upstream/main.",
		}
	}

	if err := gitCmd(ctx.RepoDir, "rev-parse", "--verify", "HEAD").Run(); err != nil {
		return UpdaterGate{
			ForceSyncOnly: true,
			Reason:        "Updater is locked: local repository has no commits. Enable Force Sync to initialize from upstream/main.",
		}
	}

	return UpdaterGate{}
}

func writeCommitFile(ctx repoContext, commitHash string) error {
	commitHash = strings.TrimSpace(commitHash)
	if commitHash == "" {
		return fmt.Errorf("empty commit hash")
	}

	commitPath := filepath.Join(ctx.InstallDir, ".commit")
	if ctx.RepoDir != "" {
		if ok, err := isGitRepo(ctx.RepoDir); err == nil && ok {
			commitPath = filepath.Join(ctx.RepoDir, ".commit")
		}
	}
	return os.WriteFile(commitPath, []byte(commitHash+"\n"), 0o644)
}

func updateCommitFile(ctx repoContext, commitHash string) error {
	commitHash = strings.TrimSpace(commitHash)
	if commitHash == "" {
		return fmt.Errorf("empty commit hash")
	}

	target := ctx
	repoOK := false
	if target.RepoDir != "" {
		if ok, err := isGitRepo(target.RepoDir); err == nil && ok {
			repoOK = true
		}
	}
	if !repoOK {
		if repoCtx, err := resolveExistingRepoContext(); err == nil {
			target.RepoDir = repoCtx.RepoDir
		} else if root, ok := findGitRoot(target.InstallDir); ok {
			target.RepoDir = root
		}
	}

	// Allow updating .commit even if it's marked skip-worktree.
	_ = setCommitSkipWorktree(target.RepoDir, false)

	if err := writeCommitFile(target, commitHash); err != nil {
		return err
	}

	_ = setCommitSkipWorktree(target.RepoDir, true)
	return nil
}

func maybeEnableCommitSkipWorktree(repoDir string) {
	_ = setCommitSkipWorktree(repoDir, true)
}

func setCommitSkipWorktree(repoDir string, enable bool) error {
	tracked, err := commitFileTracked(repoDir)
	if err != nil || !tracked {
		return err
	}

	args := []string{"update-index"}
	if enable {
		args = append(args, "--skip-worktree")
	} else {
		args = append(args, "--no-skip-worktree")
	}
	args = append(args, ".commit")

	if output, err := gitCmd(repoDir, args...).CombinedOutput(); err != nil {
		return fmt.Errorf("git %s failed: %w\n%s", strings.Join(args, " "), err, strings.TrimSpace(string(output)))
	}

	return nil
}

func commitFileTracked(repoDir string) (bool, error) {
	if repoDir == "" {
		return false, nil
	}

	ok, err := isGitRepo(repoDir)
	if err != nil || !ok {
		return false, err
	}

	commitPath := filepath.Join(repoDir, ".commit")
	if _, err := os.Stat(commitPath); err != nil {
		if os.IsNotExist(err) {
			return false, nil
		}
		return false, err
	}

	if err := gitCmd(repoDir, "ls-files", "--error-unmatch", ".commit").Run(); err != nil {
		return false, nil
	}

	return true, nil
}
