//go:build !windows

package brokerenroll

import "errors"

func newServiceBackend() (serviceBackend, error) {
	return nil, errors.New("native Windows SCM enrollment required")
}
