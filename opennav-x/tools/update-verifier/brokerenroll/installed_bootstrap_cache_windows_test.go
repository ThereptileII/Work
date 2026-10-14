//go:build windows

package brokerenroll

import (
	"context"
	"testing"
)

// Inert native directory identities only, not installed TUF/cache migration or SCM.
func TestNativeInstalledBootstrapCacheMissingAndReplacementIdentityRefuse(t *testing.T) {
	owner, original, replacement, _ := startupNativeFixture(t)
	id, err := startupDirectoryIdentity(original)
	if err != nil {
		t.Fatal(err)
	}
	b := &installedBootstrapCacheBinding{Path: original.dos, Directory: id}
	if err := matchInstalledBootstrapCachePinNative(original, b); err != nil {
		t.Fatal(err)
	}
	foreign := *b
	foreign.Path = replacement.dos // Even if spelling were approved, identity differs.
	if matchInstalledBootstrapCachePinNative(replacement, &foreign) == nil {
		t.Fatal("replacement directory adopted")
	}
	if matchInstalledBootstrapCachePinNative(nil, b) == nil {
		t.Fatal("missing cache accepted")
	}
	for _, field := range []string{"id", "creation", "descriptor", "path"} {
		changed := *b
		switch field {
		case "id":
			changed.Directory.FileID[0] ^= 1
		case "creation":
			changed.Directory.Creation++
		case "descriptor":
			changed.Directory.DescriptorSHA256 = digest([]byte("other descriptor"))
		case "path":
			changed.Path += `\other`
		}
		if matchInstalledBootstrapCachePinNative(original, &changed) == nil {
			t.Fatal("tampered identity accepted", field)
		}
	}
	inputs := &nativeEnrollmentFiles{bootstrapCache: original}
	// Borrow the real fixture ancestry owner, without transferring its cleanup.
	owner.files.bootstrapCache = original
	f := &nativeEnrollmentFiles{bootstrapCache: original, bootstrapCacheOwner: owner.files}
	if _, err := originalBootstrapCacheBindingNative(f); err != nil {
		t.Fatal(err)
	}
	if bindOriginalBootstrapCacheNative(f, inputs) == nil {
		t.Fatal("existing cache binding replaced")
	}
	f.bootstrapCache = nil
	if _, err := originalBootstrapCacheBindingNative(f); err == nil {
		t.Fatal("missing original binding accepted")
	}
}

func TestNativeInstalledCompletionCannotPublishWithoutOriginalCacheCustody(t *testing.T) {
	s, f, request, source := installedCompletionNativeFixture(t)
	f.bootstrapCacheOwner = nil
	owner, err := s.prepareInstalledCompletion(context.Background(), f, request, source)
	if owner != nil {
		owner.Close()
	}
	if err == nil {
		t.Fatal("new completion silently omitted original cache")
	}
}
