//go:build !windows

package launcher

import "errors"

func newPlatform() (platform, error) {
	return nil, errors.New("installed launcher requires native Windows")
}

// ReportFailure has no graphical implementation outside the supported host.
func ReportFailure() {}
