package brokerhost

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

// Inert lower-boundary fixture only: actual NTFS source files/metadata/ancestry,
// memory archive storage, no real KnownFolder/profile/service/process activity.
// Production Open uses the fixed Store readonly reader and KnownFolder factory.
func nativeReopenCaptureFixture(t *testing.T) (string, *reopenMemoryArchive, brokerstore.ExecutionJournalSnapshot, profileCaptureManifest) {
	t.Helper()
	root, source := nativeProfileCaptureFixture(t)
	p, writer := nativeCompleteCapture(t, source)
	snapshot, err := p.Snapshot(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	var m profileCaptureManifest
	if err = json.Unmarshal(snapshot.OriginalManifest, &m); err != nil {
		t.Fatal(err)
	}
	j := brokerstore.ExecutionJournalSnapshot{ClientSID: m.ClientSID, ExecutionID: m.ExecutionID, Generation: m.Generation, CompletionSHA256: m.CompletionSHA256, OriginalFence: []byte("fixture original retained journal fence")}
	a := &reopenMemoryArchive{raw: bytes.Clone(writer.raw), snapshot: brokerstore.ExecutionProfileBackupSnapshot{ClientSID: j.ClientSID, ExecutionID: j.ExecutionID, Generation: j.Generation, CompletionSHA256: j.CompletionSHA256, JournalFenceSHA256: captureDigest(j.OriginalFence), Bytes: int64(len(writer.raw)), SHA256: writer.digest, OriginalMarker: []byte("fixture readonly original storage marker")}}
	if err = p.Close(); err != nil {
		t.Fatal(err)
	}
	return root, a, j, m
}
func nativeReopenFixtureFactory(root string) func(context.Context, profileCaptureManifest) (profileCaptureSource, error) {
	return func(ctx context.Context, m profileCaptureManifest) (profileCaptureSource, error) {
		source, err := captureProfileTreePath(ctx, root, false)
		if err != nil {
			return nil, err
		}
		return bindReopenedProfileSource(ctx, m, source)
	}
}
func TestNativeExecutionProfileCaptureReopenInspectionOnly(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	root, a, j, _ := nativeReopenCaptureFixture(t)
	p, err := openProfileCaptureArchive(context.Background(), a, j, nativeReopenFixtureFactory(root))
	if err != nil {
		t.Fatal(err)
	}
	defer p.Close()
	if err = p.VerifyRestoredOriginal(context.Background()); err != nil {
		t.Fatal(err)
	}
	changed := []byte("[Connections]\r\nCOM8=new-unreviewed-navigation-change\r\n")
	if err = os.WriteFile(filepath.Join(root, "opencpn.ini"), changed, 0600); err != nil {
		t.Fatal("readonly reopened inspection incorrectly kept old source writer exclusion", err)
	}
	if err = p.VerifyRestoredOriginal(context.Background()); !errors.Is(err, ErrExecutionProfileRestoreReviewRequired) {
		t.Fatal("changed restart profile inferred restored", err)
	}
	if _, ok := p.state.source.(profileRestoreFactory); ok {
		t.Fatal("restart inspection exposed native writer factory")
	}
	actual, err := os.ReadFile(filepath.Join(root, "opencpn.ini"))
	if err != nil || !bytes.Equal(actual, changed) {
		t.Fatal("inspection changed current navigation bytes", err)
	}
	if err = p.state.source.close(); err != nil {
		t.Fatal(err)
	}
	if err = p.Validate(context.Background()); err == nil {
		t.Fatal("native ancestry owner closure did not revoke reopened capture")
	}
}
func TestNativeExecutionProfileCaptureReopenAncestryAndMutationRefuse(t *testing.T) {
	for _, kind := range []string{"rebound-parent", "hardlink-current", "archive-mutation"} {
		t.Run(kind, func(t *testing.T) {
			runtime.LockOSThread()
			defer runtime.UnlockOSThread()
			root, a, j, m := nativeReopenCaptureFixture(t)
			switch kind {
			case "rebound-parent":
				id, err := parseProfileCaptureID(m.Ancestors[0])
				if err != nil {
					t.Fatal(err)
				}
				id.file[15] ^= 0x55
				m.Ancestors[0] = id.wire()
				payload := bytes.Clone(a.raw[8+lenMustMarshal(t, m):])
				resetReopenArchive(t, a, m, payload)
			case "hardlink-current":
				if err := os.Link(filepath.Join(root, "opencpn.ini"), filepath.Join(root, "unreviewed-link.ini")); err != nil {
					t.Fatal(err)
				}
			case "archive-mutation":
				a.raw[len(a.raw)-1] ^= 1
			}
			p, err := openProfileCaptureArchive(context.Background(), a, j, nativeReopenFixtureFactory(root))
			if err == nil {
				p.Close()
				t.Fatal("rebound/mutated native restart custody accepted")
			}
			if a.closes != 1 {
				t.Fatal("failed native reopen lost archive cleanup", a.closes)
			}
		})
	}
}
func lenMustMarshal(t *testing.T, m profileCaptureManifest) int {
	t.Helper()
	raw, err := json.Marshal(m)
	if err != nil {
		t.Fatal(err)
	}
	return len(raw)
}
