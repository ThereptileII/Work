//go:build !windows

package brokerenroll

import (
	"bytes"
	"context"
	"testing"
)

func TestEnrollmentFilesRequireNativeWindows(t *testing.T) {
	if files, err := createFreshFiles(context.Background(), bytes.NewBufferString("broker"), imageArtifact([]byte("broker"))); files != nil || err == nil {
		t.Fatal("non-Windows machine creation accepted")
	}
	if holder, err := holdOperatorDirectories("/tmp/cache", "/tmp/artifacts"); holder != nil || err == nil {
		t.Fatal("non-Windows input custody accepted")
	}
}
