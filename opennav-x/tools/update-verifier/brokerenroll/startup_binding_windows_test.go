//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"os"
	"path/filepath"
	"testing"
	"time"

	"example.com/opennav-update-verifier/brokerregistration"
)

// Uses only disposable inert bytes and the existing elevated native fixture.
// No installed menu, SCM service, executable or profile is touched.
func startupCompletedBindingFixture(t *testing.T) (*nativeStartupPublication, *enrollmentPin, *enrollmentPin, *brokerregistration.Request) {
	t.Helper()
	owner, menu, recovery, sd := startupThreeLinkFixture(t)
	_, _, inert, _ := startupFlowFixture(t)
	raw := bytes.Replace(inert.OriginalBytes(), []byte(inertClientSID), []byte(owner.clientSID), 1)
	if bytes.Equal(raw, inert.OriginalBytes()) {
		t.Fatal("native SID was not placed in inert request")
	}
	parsed, err := brokerregistration.ParseApprovedRequest(raw, digest(raw), time.Now().UTC())
	if err != nil {
		t.Fatal(err)
	}
	owner.image, err = startupWriteFile(context.Background(), owner.files, recovery, "inert-launcher.exe", []byte("INERT-LAUNCHER"), sd, false)
	if err != nil {
		t.Fatal(err)
	}
	owner.imageSHA = digest([]byte("INERT-LAUNCHER"))
	owner.manifest, err = startupOpenFile(owner.files, recovery, "publication.json", false, maxStartupBindingBytes)
	if err != nil {
		t.Fatal(err)
	}
	manifest, err := os.ReadFile(filepath.Join(recovery.dos, "publication.json"))
	if err != nil {
		t.Fatal(err)
	}
	owner.manifestSHA = digest(manifest)
	if err := startupPublishAllRenames(context.Background(), owner, menu, recovery); err != nil {
		t.Fatal(err)
	}
	return owner, menu, recovery, &parsed
}

func TestNativeCompletedBindingAfterThreeMoves(t *testing.T) {
	owner, menu, recovery, request := startupCompletedBindingFixture(t)
	if err := completeStartupPublicationBinding(context.Background(), owner, menu, recovery, request, nil); err == nil {
		t.Fatal("missing security descriptor accepted")
	}
	// The writer receives the exact protected descriptor from the original
	// fixture. Recreate it from the retained recovery directory descriptor.
	if err := completeStartupPublicationBinding(context.Background(), owner, menu, recovery, request, recovery.descriptor); err != nil {
		t.Fatal(err)
	}
	if err := owner.Check(context.Background()); err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(recovery.dos, startupBindingName))
	if err != nil {
		t.Fatal(err)
	}
	approved, err := ParseApprovedStartupPublication(raw, digest(raw), request.OriginalBytes(), request.RequestSHA256)
	if err != nil || !bytes.Equal(approved.OriginalBytes(), raw) {
		t.Fatalf("original completed binding did not round trip: %v", err)
	}
	if owner.binding.id == owner.manifest.id || owner.binding.id == owner.original.id {
		t.Fatal("completed receipt reused another native identity")
	}
	if err := completeStartupPublicationBinding(context.Background(), owner, menu, recovery, request, recovery.descriptor); err == nil {
		t.Fatal("completion record could be overwritten")
	}
}

func TestNativeBindingRecordsExistingMenuDescriptor(t *testing.T) {
	owner, menu, recovery, request := startupCompletedBindingFixture(t)
	// pinDirectory captures an existing menu with a nil expected descriptor.
	// Its live handle and security policy still have to pass the write gate.
	menu.descriptor = nil
	if err := completeStartupPublicationBinding(context.Background(), owner, menu, recovery, request, recovery.descriptor); err != nil {
		t.Fatal(err)
	}
	if err := owner.Check(context.Background()); err != nil {
		t.Fatal(err)
	}
}

func TestNativeBindingFailureRetainsMovedEvidence(t *testing.T) {
	owner, menu, recovery, request := startupCompletedBindingFixture(t)
	owner.manifestSHA = digest([]byte("different prepublication bytes"))
	if err := completeStartupPublicationBinding(context.Background(), owner, menu, recovery, request, recovery.descriptor); err == nil {
		t.Fatal("changed prepublication evidence accepted")
	}
	if _, err := os.Stat(filepath.Join(recovery.dos, startupBindingName)); !errors.Is(err, os.ErrNotExist) {
		t.Fatalf("failed completion left a success record: %v", err)
	}
	for _, child := range append([]*nativeStartupPublication{owner}, owner.additional...) {
		spec, _ := child.mode.spec()
		if err := startupHashPin(context.Background(), child.link, child.linkSHA); err != nil || child.link.physical != menu.physical+`\`+spec.name {
			t.Fatalf("published link custody lost after failed binding: %v", err)
		}
	}
	if err := startupHashPin(context.Background(), owner.original, owner.originalSHA); err != nil {
		t.Fatalf("original backup custody lost: %v", err)
	}
}

func TestNativeBindingDoesNotAdoptExistingCompletionName(t *testing.T) {
	owner, menu, recovery, request := startupCompletedBindingFixture(t)
	foreign := []byte("INERT-FOREIGN-COMPLETION")
	if _, err := startupWriteFile(context.Background(), owner.files, recovery, startupBindingName, foreign, recovery.descriptor, false); err != nil {
		t.Fatal(err)
	}
	if err := completeStartupPublicationBinding(context.Background(), owner, menu, recovery, request, recovery.descriptor); err == nil {
		t.Fatal("existing completion file was adopted or replaced")
	}
	raw, err := os.ReadFile(filepath.Join(recovery.dos, startupBindingName))
	if err != nil || !bytes.Equal(raw, foreign) || owner.binding != nil {
		t.Fatalf("existing completion evidence changed: %v", err)
	}
	if err := startupHashPin(context.Background(), owner.original, owner.originalSHA); err != nil {
		t.Fatalf("original backup lost after collision: %v", err)
	}
}
