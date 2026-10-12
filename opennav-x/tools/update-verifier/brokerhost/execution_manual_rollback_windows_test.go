//go:build windows

package brokerhost

import (
	"bytes"
	"context"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
	"golang.org/x/sys/windows"
)

// Temporary source trees only. This exercises actual retained NTFS current-tree
// identities, deny-writer sharing and mutation watch; no installed path, process,
// SCM object, registration, trust or real navigation data is touched.
func nativeManualRollbackProfileFixture(t *testing.T) (string, *nativeManualRollbackProfile) {
	t.Helper()
	root, source := nativeProfileCaptureFixture(t)
	_, _, b, _ := manualRollbackFixture(t)
	p := &nativeManualRollbackProfile{source: source, snapshot: brokerstore.ExecutionJournalSnapshot{
		ClientSID: b.ClientSID, ExecutionID: b.ExecutionID, Generation: b.Generation, CompletionSHA256: b.CompletionSHA256}}
	t.Cleanup(func() {
		if err := p.close(); err != nil {
			t.Error(err)
		}
	})
	return root, p
}

func TestNativeExecutionManualRollbackRetainsCurrentProfileThroughReadback(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	root, p := nativeManualRollbackProfileFixture(t)
	ctx := context.Background()
	before, err := p.validate(ctx)
	if err != nil || !hashPattern.MatchString(before.contents) || !hashPattern.MatchString(before.identity) {
		t.Fatal("actual current profile binding missing", before, err)
	}
	name := filepath.Join(root, "navigation", "navobj.xml")
	raw, err := os.ReadFile(name)
	if err != nil {
		t.Fatal(err)
	}
	borrowed := borrowedManualRollbackProfile{p}
	if err = borrowed.close(); err != nil {
		t.Fatal(err)
	}
	if err = os.WriteFile(name, []byte("must not overwrite current voyage"), 0600); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatal("writer borrowed lifetime opened profile gap", err)
	}
	if err = os.Remove(name); !errors.Is(err, windows.ERROR_SHARING_VIOLATION) {
		t.Fatal("current profile allowed deletion", err)
	}
	after, err := borrowed.validate(ctx)
	if err != nil || before != after {
		t.Fatal("current profile changed during readback", before, after, err)
	}
	current, err := os.ReadFile(name)
	if err != nil || !bytes.Equal(raw, current) {
		t.Fatal("rollback observation changed current navigation", err)
	}
	if err = p.close(); err != nil {
		t.Fatal(err)
	}
	if _, err = p.validate(ctx); err == nil {
		t.Fatal("closed profile owner retained authority")
	}
	// Only the inert fixture writes after all authority is revoked.
	if err = os.WriteFile(name, []byte("fixture post-close write"), 0600); err != nil {
		t.Fatal(err)
	}
}

func TestNativeExecutionManualRollbackNewNavigationAdditionRefusesWithoutErasingIt(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	root, p := nativeManualRollbackProfileFixture(t)
	if _, err := p.validate(context.Background()); err != nil {
		t.Fatal(err)
	}
	name := filepath.Join(root, "navigation", "new-user-data.xml")
	raw := []byte("new current user data must survive refusal")
	if err := os.WriteFile(name, raw, 0600); err != nil {
		t.Fatal(err)
	}
	if _, err := p.validate(context.Background()); err == nil {
		t.Fatal("new profile namespace acquired unchanged proof")
	}
	current, err := os.ReadFile(name)
	if err != nil || !bytes.Equal(raw, current) {
		t.Fatal("refusal erased newer navigation data", err)
	}
}
