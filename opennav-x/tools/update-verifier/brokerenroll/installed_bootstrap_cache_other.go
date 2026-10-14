//go:build !windows

package brokerenroll

import (
	"errors"
	"io"
)

func bindOriginalBootstrapCacheNative(enrollmentFiles, io.Closer) error {
	return errors.New("original native Windows bootstrap cache custody required")
}
