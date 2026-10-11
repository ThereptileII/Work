//go:build !windows

package brokerstore

import (
	"errors"
	"testing"
)

func TestPortableBackendRefuses(t *testing.T) {
	for _, root := range []string{"", t.TempDir(), `C:\ProgramData\SKAGER\UpdateBroker`} {
		s, e := Open(root)
		if s != nil || !errors.Is(e, ErrNativeRequired) {
			t.Fatalf("%v %v", s, e)
		}
	}
}
