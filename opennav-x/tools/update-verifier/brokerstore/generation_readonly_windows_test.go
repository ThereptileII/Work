package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"runtime"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/baselineevidence"
	"golang.org/x/sys/windows"
)

// Use the actual inert test executable as a valid PE input. Mapping this copied
// image checks Windows SEC_IMAGE semantics; no entrypoint or helper is executed.
func generationImageData(t *testing.T, f baselineFixtureData) baselineFixtureData {
	t.Helper()
	path, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	info, err := os.Stat(path)
	if err != nil || !info.Mode().IsRegular() || info.Size() > 128<<20 {
		t.Fatal("bounded own test image required", err)
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	f.payload["app/opencpn.exe"] = raw
	var product map[string]any
	if err = json.Unmarshal(f.payload["docs/PRODUCT_BUILD.json"], &product); err != nil {
		t.Fatal(err)
	}
	product["executable_sha256"] = registrationDigest(raw)
	f.payload["docs/PRODUCT_BUILD.json"], err = json.Marshal(product)
	if err != nil {
		t.Fatal(err)
	}
	var manifest map[string]any
	if err = json.Unmarshal(f.manifest, &manifest); err != nil {
		t.Fatal(err)
	}
	for _, entry := range manifest["files"].([]any) {
		item := entry.(map[string]any)
		name := item["path"].(string)
		item["sha256"] = registrationDigest(f.payload[name])
	}
	f.manifest, err = json.Marshal(manifest)
	if err != nil {
		t.Fatal(err)
	}
	var inventory map[string]string
	if err = json.Unmarshal(f.inventory, &inventory); err != nil {
		t.Fatal(err)
	}
	observed := map[string]string{}
	for name, raw := range f.payload {
		observed[name] = registrationDigest(raw)
		inventory[name] = observed[name]
	}
	inventory["installer/package.json"] = registrationDigest(f.manifest)
	f.inventory, err = json.Marshal(inventory)
	if err != nil {
		t.Fatal(err)
	}
	f.expected.PackageSHA256 = registrationDigest(f.manifest)
	f.expected.InventorySHA256 = registrationDigest(f.inventory)
	f.expected.ProductBuildSHA256 = observed["docs/PRODUCT_BUILD.json"]
	f.expected.ExecutableSHA256 = observed["app/opencpn.exe"]
	f.identity, _, err = baselineevidence.Validate(context.Background(), f.inventory, f.manifest, f.payload["docs/PRODUCT_BUILD.json"], observed, f.expected)
	if err != nil {
		t.Fatal(err)
	}
	return f
}
func TestNativeGenerationReadOnlyMapsImageAndPreservesOriginalCustody(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, p, _, f := nativeGenerationFixture(t)
	f = generationImageData(t, f)
	c := nativeGenerationSource(t, s, f, strings.Repeat("9", 32))
	g, err := s.MaterializeCandidateFresh(context.Background(), p, c)
	if err != nil {
		t.Fatal(err)
	}
	n := g.backend.(*nativeInstalledGeneration)
	before, err := g.Completion()
	if err != nil {
		t.Fatal(err)
	}
	identities := map[string]diskIdentity{}
	for name, file := range n.members {
		identities[name] = file.identity
	}
	imagePath := physical(n.members["app/opencpn.exe"].pin.name)
	if err = nativeAttemptFile(t, imagePath, windows.GENERIC_READ|windows.GENERIC_EXECUTE); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatal("fixture writer was already loader-compatible", err)
	}
	r, err := g.TransitionReadOnly(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	after, err := r.Completion()
	if err != nil || after != before {
		t.Fatal("completion changed", err)
	}
	for name, file := range n.members {
		if file.identity != identities[name] {
			t.Fatal("physical original replaced", name)
		}
		h, err := r.OpenMember(name)
		if err != nil {
			t.Fatal(err)
		}
		id, err := h.Identity()
		if err != nil {
			t.Fatal(err)
		}
		raw := make([]byte, id.Length)
		var off int64
		for off < id.Length {
			end := off + maxGenerationMemberRead
			if end > id.Length {
				end = id.Length
			}
			count, err := h.ReadAt(raw[off:end], off)
			if err != nil {
				t.Fatal(name, err)
			}
			off += int64(count)
		}
		if registrationDigest(raw) != id.SHA256 {
			t.Fatal("owned original bytes changed", name)
		}
		h.Close()
		if _, err = file.File.WriteAt([]byte("X"), 0); err == nil {
			t.Fatal("final handle retained writer access", name)
		}
		for _, access := range []uint32{windows.GENERIC_WRITE, windows.DELETE} {
			if err = nativeAttemptFile(t, physical(file.pin.name), access); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
				t.Fatal("readonly custody permits writer/delete", name, err)
			}
		}
	}
	path, err := windows.UTF16PtrFromString(imagePath)
	if err != nil {
		t.Fatal(err)
	}
	handle, err := windows.CreateFile(path, windows.GENERIC_READ|windows.GENERIC_EXECUTE, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_ATTRIBUTE_NORMAL, 0)
	if err != nil {
		t.Fatal("loader read/execute open", err)
	}
	defer windows.CloseHandle(handle)
	const secImage = 0x01000000
	mapping, err := windows.CreateFileMapping(handle, nil, windows.PAGE_READONLY|secImage, 0, 0, nil)
	if err != nil {
		t.Fatal("SEC_IMAGE mapping", err)
	}
	defer windows.CloseHandle(mapping)
	view, err := windows.MapViewOfFile(mapping, windows.FILE_MAP_READ, 0, 0, 0)
	if err != nil {
		t.Fatal("SEC_IMAGE view", err)
	}
	if err = windows.UnmapViewOfFile(view); err != nil {
		t.Fatal(err)
	}
	if err = r.Validate(); err != nil {
		t.Fatal(err)
	}
	if err = r.Close(); err != nil {
		t.Fatal(err)
	}
	if err = g.Close(); err != nil {
		t.Fatal(err)
	}
}

// Cancellation observes an actual closed native writer, rather than depending
// on a count of context checks. It exercises the close/reopen gap without hooks
// in production or any native application/service action.
type generationWriterClosedContext struct {
	context.Context
	writer *os.File
}

func (c generationWriterClosedContext) Err() error {
	if c.writer.Fd() == ^uintptr(0) {
		return context.Canceled
	}
	return c.Context.Err()
}
func TestNativeGenerationReadOnlyCancellationGapRetainsCompletedBytes(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, p, _, f := nativeGenerationFixture(t)
	c := nativeGenerationSource(t, s, f, strings.Repeat("9", 32))
	g, err := s.MaterializeCandidateFresh(context.Background(), p, c)
	if err != nil {
		t.Fatal(err)
	}
	n := g.backend.(*nativeInstalledGeneration)
	old := n.members[g.names()[0]]
	markerPath := physical(n.marker.pin.name)
	recordPath := physical(n.record.pin.name)
	raw := bytes.Clone(g.raw)
	r, err := g.TransitionReadOnly(generationWriterClosedContext{context.Background(), old.File})
	if r != nil || !errors.Is(err, context.Canceled) || !errors.Is(err, ErrInstalledGenerationUncertain) {
		t.Fatal("cancelled transition accepted or failure lost", err)
	}
	if old.File.Fd() != ^uintptr(0) {
		t.Fatal("writer not consumed")
	}
	if _, err = g.TransitionReadOnly(context.Background()); err == nil {
		t.Fatal("failed owner retried")
	}
	first := g.Close()
	if first == nil || g.Close() != first {
		t.Fatal("cleanup failure not cached")
	}
	actual, err := os.ReadFile(recordPath)
	if err != nil || !bytes.Equal(actual, raw) {
		t.Fatal("completed bytes deleted or rewritten", err)
	}
	marker, err := os.ReadFile(markerPath)
	if err != nil || !bytes.Equal(marker, n.markerBytes) {
		t.Fatal("permanent marker lost", err)
	}
}
func TestNativeGenerationReadOnlyRefusesChangedOriginalBeforeTransfer(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, scenario := range []string{"payload", "private-history", "extra-namespace"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, _, f := nativeGenerationFixture(t)
			c := nativeGenerationSource(t, s, f, strings.Repeat("9", 32))
			g, err := s.MaterializeCandidateFresh(context.Background(), p, c)
			if err != nil {
				t.Fatal(err)
			}
			n := g.backend.(*nativeInstalledGeneration)
			switch scenario {
			case "payload":
				_, err = n.members["app/opencpn.exe"].File.WriteAt([]byte("X"), 0)
			case "private-history":
				_, err = n.record.File.WriteAt([]byte("X"), 0)
			case "extra-namespace":
				extra, e := baselineFile(n.tree.directories["app"], "unknown.dat", n.tree.securityFile, true)
				if e != nil {
					t.Fatal(e)
				}
				err = extra.Close()
			}
			if err != nil {
				t.Fatal(err)
			}
			r, err := g.TransitionReadOnly(context.Background())
			if r != nil || !errors.Is(err, ErrInstalledGenerationUncertain) {
				t.Fatal("changed generation transferred", err)
			}
			if n.readOnlyAttempted {
				t.Fatal("changed original reached close/reopen backend")
			}
			if _, err = g.TransitionReadOnly(context.Background()); err == nil {
				t.Fatal("failed generation reusable")
			}
			if !errors.Is(g.Close(), ErrInstalledGenerationUncertain) {
				t.Fatal("uncertainty not retained")
			}
		})
	}
}
