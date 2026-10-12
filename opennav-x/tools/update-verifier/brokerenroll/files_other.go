//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
	"io"

	"example.com/opennav-update-verifier/releasepolicy"
)

func createFreshFiles(context.Context, io.Reader, releasepolicy.Artifact) (enrollmentFiles, error) {
	return nil, errors.New("fresh broker enrollment requires native Windows")
}

func holdOperatorDirectories(string, string) (io.Closer, error) {
	return nil, errors.New("operator directory custody requires native Windows")
}
