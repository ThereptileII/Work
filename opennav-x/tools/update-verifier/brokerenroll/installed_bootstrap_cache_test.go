package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"testing"
)

func TestInstalledCompletionOriginalCacheVersionCompatibility(t *testing.T) {
	legacy := installedCompletionRecordFixture(t)
	original, err := marshalInstalledCompletion(legacy)
	if err != nil {
		t.Fatal(err)
	}
	if bytes.Contains(original, []byte("bootstrapCache")) {
		t.Fatal("schema1 bytes changed")
	}
	got, err := parseInstalledCompletion(original)
	if err != nil || got.Schema != 1 || got.BootstrapCache != nil {
		t.Fatal("schema1 compatibility lost", err)
	}
	binding := &installedBootstrapCacheBinding{Path: `C:\ProtectedCache`, Directory: legacy.Parent}
	v2 := legacy
	v2.Schema, v2.BootstrapCache = 2, binding
	raw, err := marshalInstalledCompletion(v2)
	if err != nil {
		t.Fatal(err)
	}
	got, err = parseInstalledCompletion(raw)
	if err != nil || got.Schema != 2 || got.BootstrapCache == nil || *got.BootstrapCache != *binding {
		t.Fatal("bound original cache lost", err)
	}
	for _, fault := range []string{"missing", "legacy-with-binding", "relative", "alias", "stream", "zero-id", "descriptor"} {
		t.Run(fault, func(t *testing.T) {
			changed := v2
			cache := *binding
			changed.BootstrapCache = &cache
			switch fault {
			case "missing":
				changed.BootstrapCache = nil
			case "legacy-with-binding":
				changed.Schema = 1
			case "relative":
				cache.Path = `ProtectedCache`
			case "alias":
				cache.Path = `C:\Cache\..\ProtectedCache`
			case "stream":
				cache.Path = `C:\ProtectedCache:other`
			case "zero-id":
				cache.Directory.FileID = [16]byte{}
			case "descriptor":
				cache.Directory.DescriptorSHA256 = ""
			}
			invalid, _ := json.Marshal(changed)
			if _, err := parseInstalledCompletion(invalid); err == nil {
				t.Fatal("invalid cache binding admitted", fault)
			}
		})
	}
}

func TestEnrollmentOriginalCacheBindingPrecedesServiceInitialization(t *testing.T) {
	for _, fail := range []bool{false, true} {
		f := newEnrollmentFixture(t)
		called := false
		f.op.bindOriginalInputs = func(files enrollmentFiles, inputs io.Closer) error {
			if called || files != f.files || inputs != f.inputs || f.inputs.closed || f.files.closed {
				t.Fatal("original inputs not retained")
			}
			called = true
			if fail {
				return errors.New("original cache binding refused")
			}
			return nil
		}
		initialize := f.op.initialize
		initialized := false
		f.op.initialize = func(ctx context.Context, files enrollmentFiles) error {
			if !called {
				t.Fatal("initialization before binding")
			}
			initialized = true
			return initialize(ctx, files)
		}
		err := run(context.Background(), f.req, f.op)
		if (err != nil) != fail || !called || initialized == fail {
			t.Fatal("cache failure reached initialization", err)
		}
	}
}
