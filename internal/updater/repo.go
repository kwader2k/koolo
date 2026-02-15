package updater

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
)

const sourceDirName = ".koolo-src"

type repoContext struct {
	RepoDir    string
	InstallDir string
}

func resolveRepoContext() (repoContext, error) {
	return resolveRepoContextInternal(true)
}

func resolveExistingRepoContext() (repoContext, error) {
	return resolveRepoContextInternal(false)
}

// ResolveExistingRepoDir returns the current repository directory if it exists.
func ResolveExistingRepoDir() (string, error) {
	ctx, err := resolveExistingRepoContext()
	if err != nil {
		return "", err
	}
	return ctx.RepoDir, nil
}

func resolveRepoContextInternal(allowClone bool) (repoContext, error) {
	workDir, err := os.Getwd()
	if err != nil {
		return repoContext{}, fmt.Errorf("error getting current working directory: %w", err)
	}

	installDir, err := resolveInstallDir()
	if err != nil {
		return repoContext{}, err
	}

	if root, ok := findGitRoot(installDir); ok {
		return newRepoContext(root, installDir), nil
	}

	if root, ok := findGitRoot(workDir); ok {
		return newRepoContext(root, installDir), nil
	}

	repoDir := filepath.Join(installDir, sourceDirName)
	isRepo, err := isGitRepo(repoDir)
	if err != nil {
		return repoContext{}, err
	}
	if isRepo {
		return newRepoContext(repoDir, installDir), nil
	}

	fallbackRepoDir := filepath.Join(workDir, sourceDirName)
	isRepo, err = isGitRepo(fallbackRepoDir)
	if err != nil {
		return repoContext{}, err
	}
	if isRepo {
		return newRepoContext(fallbackRepoDir, installDir), nil
	}

	if !allowClone {
		return repoContext{}, fmt.Errorf("no git repository found")
	}

	if err := ensureCloneDirAvailable(repoDir); err != nil {
		return repoContext{}, err
	}

	if err := checkGitInstalled(); err != nil {
		return repoContext{}, err
	}

	cloneCmd := newCommand("git", "clone", upstreamRemoteURL(), repoDir)
	cloneCmd.Dir = installDir
	if output, err := cloneCmd.CombinedOutput(); err != nil {
		return repoContext{}, fmt.Errorf("failed to clone upstream repository: %w\nOutput: %s", err, strings.TrimSpace(string(output)))
	}

	return newRepoContext(repoDir, installDir), nil
}

func newRepoContext(repoDir, installDir string) repoContext {
	ctx := repoContext{
		RepoDir:    repoDir,
		InstallDir: installDir,
	}
	maybeEnableCommitSkipWorktree(ctx.RepoDir)
	return ctx
}

func resolveInstallDir() (string, error) {
	exePath, err := os.Executable()
	if err == nil {
		absPath, absErr := filepath.Abs(exePath)
		if absErr == nil {
			return filepath.Dir(absPath), nil
		}
	}

	workDir, err := os.Getwd()
	if err != nil {
		return "", fmt.Errorf("error getting install directory: %w", err)
	}
	return workDir, nil
}

func findGitRoot(startDir string) (string, bool) {
	dir := startDir
	for {
		if _, err := os.Stat(filepath.Join(dir, ".git")); err == nil {
			return dir, true
		}

		parent := filepath.Dir(dir)
		if parent == dir {
			break
		}
		dir = parent
	}

	return "", false
}

func isGitRepo(repoDir string) (bool, error) {
	gitPath := filepath.Join(repoDir, ".git")
	if _, err := os.Stat(gitPath); err == nil {
		return true, nil
	} else if os.IsNotExist(err) {
		return false, nil
	} else {
		return false, fmt.Errorf("failed to inspect %s: %w", gitPath, err)
	}
}

func ensureCloneDirAvailable(repoDir string) error {
	info, err := os.Stat(repoDir)
	if err != nil {
		if os.IsNotExist(err) {
			return nil
		}
		return fmt.Errorf("failed to access %s: %w", repoDir, err)
	}
	if !info.IsDir() {
		return fmt.Errorf("source path exists and is not a directory: %s", repoDir)
	}

	entries, err := os.ReadDir(repoDir)
	if err != nil {
		return fmt.Errorf("failed to read %s: %w", repoDir, err)
	}
	if len(entries) > 0 {
		return fmt.Errorf("source directory is not empty: %s", repoDir)
	}

	return nil
}

func resolveCommitPath() (string, error) {
	installDir, err := resolveInstallDir()
	if err != nil {
		return "", err
	}

	if ctx, err := resolveExistingRepoContext(); err == nil {
		commitPath := filepath.Join(ctx.RepoDir, ".commit")
		if _, err := os.Stat(commitPath); err == nil {
			return commitPath, nil
		} else if err != nil && !os.IsNotExist(err) {
			return "", err
		}
	}

	commitPath := filepath.Join(installDir, ".commit")
	if _, err := os.Stat(commitPath); err == nil {
		return commitPath, nil
	} else if err != nil && !os.IsNotExist(err) {
		return "", err
	}

	return "", os.ErrNotExist
}

func cleanupTmpRoot(tmpRoot string) {
	if tmpRoot == "" {
		return
	}

	removeDirIfEmpty(tmpRoot)

	parent := filepath.Dir(tmpRoot)
	if filepath.Base(parent) == ".tmp" {
		removeDirIfEmpty(parent)
	}
}

func removeDirIfEmpty(dir string) {
	if dir == "" {
		return
	}

	entries, err := os.ReadDir(dir)
	if err != nil {
		return
	}
	if len(entries) != 0 {
		return
	}
	_ = os.Remove(dir)
}

func gitCmd(repoDir string, args ...string) *exec.Cmd {
	cmd := newCommand("git", args...)
	cmd.Dir = repoDir
	return cmd
}
