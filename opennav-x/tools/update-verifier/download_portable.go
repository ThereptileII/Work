//go:build !windows

package verifier

import "os"

// This retains a read-only handle, but Unix does not enforce Windows share modes.
func openDownloadedArtifact(path string) (*os.File, error) { return os.Open(path) }
