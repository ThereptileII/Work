//go:build !windows && !linux

package helm

import "errors"

func openPlatformDisplay(string) (Display, error) {
	return nil, errors.New("SKAGER Helm captures the screen on Windows and Linux only")
}
