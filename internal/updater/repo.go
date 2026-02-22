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
	WorkDir    string
}

func resolveRepoContext() (repoContext, error) {
	workDir, err := os.Getwd()
	if err != nil {
		return repoContext{}, fmt.Errorf("error getting current working directory: %w", err)
	}

	installDir, err := resolveInstallDir()
	if err != nil {
		return repoContext{}, err
	}

	if root, ok := findGitRoot(workDir); ok {
		return repoContext{
			RepoDir:    root,
			InstallDir: installDir,
			WorkDir:    workDir,
		}, nil
	}

	repoDir := filepath.Join(workDir, sourceDirName)
	isRepo, err := isGitRepo(repoDir)
	if err != nil {
		return repoContext{}, err
	}
	if isRepo {
		return repoContext{
			RepoDir:    repoDir,
			InstallDir: installDir,
			WorkDir:    workDir,
		}, nil
	}

	if err := ensureCloneDirAvailable(repoDir); err != nil {
		return repoContext{}, err
	}

	if err := checkGitInstalled(); err != nil {
		return repoContext{}, err
	}

	cloneCmd := newCommand("git", "clone", "https://github.com/kwader2k/koolo.git", repoDir)
	cloneCmd.Dir = workDir
	if output, err := cloneCmd.CombinedOutput(); err != nil {
		return repoContext{}, fmt.Errorf("failed to clone upstream repository: %w\nOutput: %s", err, strings.TrimSpace(string(output)))
	}

	return repoContext{
		RepoDir:    repoDir,
		InstallDir: installDir,
		WorkDir:    workDir,
	}, nil
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

func gitCmd(repoDir string, args ...string) *exec.Cmd {
	cmd := newCommand("git", args...)
	cmd.Dir = repoDir
	return cmd
}
