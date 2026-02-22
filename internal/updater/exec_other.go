//go:build !windows

package updater

import "os/exec"

func hideWindow(cmd *exec.Cmd) {}
