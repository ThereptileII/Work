//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"crypto/rand"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"io"
	"reflect"
	"sort"
	"testing"
	"time"

	"golang.org/x/sys/windows"
	"golang.org/x/sys/windows/registry"
)

// These tests own only a fresh private key below HKU/currentSID/Software. They
// never open the installed product registration, service or machine namespace.
// Transactions, original native key handles and visibility checks are real.
type installedRegistryFixture struct {
	sid, path, base string
	want            map[string][]byte
	key, sibling    registry.Key
}

func newInstalledRegistryFixture(t *testing.T) *installedRegistryFixture {
	t.Helper()
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	var nonce [16]byte
	if _, err := rand.Read(nonce[:]); err != nil {
		t.Fatal(err)
	}
	f := &installedRegistryFixture{sid: user.User.Sid.String(), base: `Software\SKAGER-Registry-Fixture-` + hex.EncodeToString(nonce[:])}
	f.path = f.base + `\Original`
	f.want = installedRegistryValues(`C:\SKAGER-Inert-Registry\Integration`, `C:\SKAGER-Inert-Registry\Integration\generations\aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa`, "0.4.0-fixture")
	t.Cleanup(func() {
		if f.key != 0 {
			if err := f.key.Close(); err != nil {
				t.Error("private original key close", err)
			}
		}
		if f.sibling != 0 {
			if err := f.sibling.Close(); err != nil {
				t.Error("private sibling key close", err)
			}
		}
		deleteInstalledRegistryFixture(t, f.sid+`\`+f.base)
	})
	f.key, _, err = registry.CreateKey(registry.USERS, f.sid+`\`+f.path, registry.ALL_ACCESS|registry.WOW64_32KEY)
	if err != nil {
		t.Fatal(err)
	}
	for name, raw := range f.want {
		units := make([]uint16, len(raw)/2)
		for i := range units {
			units[i] = binary.LittleEndian.Uint16(raw[i*2:])
		}
		if err := f.key.SetStringValue(name, windows.UTF16ToString(units)); err != nil {
			t.Fatal(err)
		}
	}
	f.sibling, _, err = registry.CreateKey(registry.USERS, f.sid+`\`+f.base+`\Sibling`, registry.ALL_ACCESS|registry.WOW64_32KEY)
	if err != nil {
		t.Fatal(err)
	}
	if err := f.sibling.SetStringValue("Keep", "unrelated private fixture registration"); err != nil {
		t.Fatal(err)
	}
	return f
}

func deleteInstalledRegistryFixture(t *testing.T, path string) {
	t.Helper()
	key, err := registry.OpenKey(registry.USERS, path, registry.READ|registry.WOW64_32KEY)
	if errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		return
	}
	if err != nil {
		t.Error("private fixture cleanup open", err)
		return
	}
	children, err := key.ReadSubKeyNames(16)
	closeErr := key.Close()
	if err != nil && !errors.Is(err, io.EOF) || closeErr != nil || len(children) == 16 {
		t.Error("private fixture cleanup bounds", err, closeErr)
		return
	}
	for _, child := range children {
		deleteInstalledRegistryFixture(t, path+`\`+child)
	}
	if err := registry.DeleteKey(registry.USERS, path); err != nil {
		t.Error("private fixture cleanup delete", err)
	}
}

func (f *installedRegistryFixture) owner(t *testing.T, ctx context.Context) *installedRegistryOwner {
	t.Helper()
	n, err := openInstalledRegistryAt(ctx, f.sid, f.path, f.want)
	if n != nil {
		t.Cleanup(func() {
			if err := n.close(); err != nil {
				t.Error("original transaction owner close", err)
			}
		})
	}
	if err != nil || n == nil {
		t.Fatal("original private registry transaction unavailable", err)
	}
	return n
}

type installedRegistryFixtureValue struct {
	Kind uint32
	Raw  []byte
}
type installedRegistryFixtureSnapshot struct {
	Values   map[string]installedRegistryFixtureValue
	Children []string
}

func installedRegistrySnapshot(t *testing.T, key registry.Key) installedRegistryFixtureSnapshot {
	t.Helper()
	info, err := key.Stat()
	if err != nil || info.ValueCount > 32 || info.SubKeyCount > 16 || info.MaxValueLen > 32768 {
		t.Fatal("private registry fixture snapshot bounds", err)
	}
	names, err := key.ReadValueNames(33)
	if err != nil && !errors.Is(err, io.EOF) {
		t.Fatal(err)
	}
	result := installedRegistryFixtureSnapshot{Values: make(map[string]installedRegistryFixtureValue, len(names))}
	for _, name := range names {
		buffer := make([]byte, 32768)
		used, kind, err := key.GetValue(name, buffer)
		if err != nil {
			t.Fatal(err)
		}
		result.Values[name] = installedRegistryFixtureValue{kind, bytes.Clone(buffer[:used])}
	}
	result.Children, err = key.ReadSubKeyNames(17)
	if err != nil && !errors.Is(err, io.EOF) {
		t.Fatal(err)
	}
	sort.Strings(result.Children)
	return result
}

func (f *installedRegistryFixture) requireOriginal(t *testing.T, expected installedRegistryFixtureSnapshot) {
	t.Helper()
	key, err := registry.OpenKey(registry.USERS, f.sid+`\`+f.path, registry.READ|registry.WOW64_32KEY)
	if err != nil {
		t.Fatal("original fixed private registration disappeared", err)
	}
	defer key.Close()
	if actual := installedRegistrySnapshot(t, key); !reflect.DeepEqual(actual, expected) {
		t.Fatal("original private registration bytes/subkeys changed", actual, expected)
	}
	if actual, _, err := f.sibling.GetStringValue("Keep"); err != nil || actual != "unrelated private fixture registration" {
		t.Fatal("unrelated sibling registration changed", err)
	}
}

func stageInstalledRegistryFixtureDelete(t *testing.T, n *installedRegistryOwner) {
	t.Helper()
	if err := installedDeleteKey.Find(); err != nil {
		t.Fatal(err)
	}
	status, _, _ := installedDeleteKey.Call(uintptr(n.transacted))
	if int32(status) < 0 {
		t.Fatalf("same original transacted key delete refused NTSTATUS %#x", uint32(status))
	}
}

func rollbackInstalledRegistryFixture(t *testing.T, n *installedRegistryOwner) {
	t.Helper()
	if outcome, err := n.outcome(); err != nil || outcome != 1 && outcome != 3 {
		t.Fatal("private registry transaction cannot be safely rolled back", outcome, err)
	} else if outcome == 3 {
		return // A native conflict/timeout may already have aborted this owner.
	}
	rollback := windows.NewLazySystemDLL("ktmw32.dll").NewProc("RollbackTransaction")
	if err := rollback.Find(); err != nil {
		t.Fatal(err)
	}
	if ok, _, err := rollback.Call(uintptr(n.transaction)); ok == 0 {
		t.Fatal("private registry transaction rollback failed", err)
	}
	if outcome, err := n.outcome(); err != nil || outcome != 3 {
		t.Fatal("private registry transaction did not abort", outcome, err)
	}
}

func TestNativeInstalledRegistryRetirementDeleteIsHiddenBeforeCommitAndCloseRollsBack(t *testing.T) {
	f := newInstalledRegistryFixture(t)
	before := installedRegistrySnapshot(t, f.key)
	n := f.owner(t, context.Background())
	if n.original.key == 0 || n.transacted == 0 || n.transaction == 0 || n.original.key == n.transacted {
		t.Fatal("separate retained original and transaction handles required")
	}
	stageInstalledRegistryFixtureDelete(t, n)
	if _, err := registry.Key(n.transacted).Stat(); !errors.Is(err, windows.ERROR_KEY_DELETED) && !errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		t.Fatal("staged deletion remained visible inside its transaction", err)
	}
	if outcome, err := n.outcome(); err != nil || outcome != 1 {
		t.Fatal("staged deletion unexpectedly committed or aborted", outcome, err)
	}
	// This independent nontransacted path observation must still see all exact
	// original bytes. NtDeleteKey on the held transaction must not delete early.
	f.requireOriginal(t, before)
	if err := n.close(); err != nil {
		t.Fatal(err)
	}
	f.requireOriginal(t, before)
}

func TestNativeInstalledRegistryDecommissionCommitDeletesOnlyOriginalKey(t *testing.T) {
	f := newInstalledRegistryFixture(t)
	n := f.owner(t, context.Background())
	if err := n.remove(context.Background()); err != nil {
		t.Fatal("original registry retirement failed", err)
	}
	if !n.committed || n.failed || n.check(context.Background()) != nil {
		t.Fatal("committed original registry custody differs")
	}
	key, err := registry.OpenKey(registry.USERS, f.sid+`\`+f.path, registry.READ|registry.WOW64_32KEY)
	if key != 0 {
		key.Close()
	}
	if !errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		t.Fatal("original fixed registration name remains after commit", err)
	}
	if value, _, err := f.sibling.GetStringValue("Keep"); err != nil || value != "unrelated private fixture registration" {
		t.Fatal("registry commit affected a sibling", err)
	}
	if err := n.remove(context.Background()); err == nil {
		t.Fatal("original registry deletion retried")
	}
}

func TestNativeInstalledRegistryRetirementConflictingNontransactedMutationPreservesOriginal(t *testing.T) {
	f := newInstalledRegistryFixture(t)
	before := installedRegistrySnapshot(t, f.key)
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	n := f.owner(t, ctx)
	writeDone := make(chan error, 1)
	go func() { writeDone <- f.key.SetStringValue("DisplayName", "completed outside mutation") }()
	var writeErr error
	select {
	case writeErr = <-writeDone:
	case <-time.After(5 * time.Second):
		// Release only this private transaction so fixture cleanup cannot leave
		// a blocked writer or accidentally erase the initial failure evidence.
		rollbackInstalledRegistryFixture(t, n)
		select {
		case <-writeDone:
		case <-time.After(time.Second):
			t.Fatal("outside private registry writer remained blocked after rollback")
		}
		t.Fatal("native transaction did not bound conflicting outside writer")
	}
	if writeErr != nil {
		if !errors.Is(writeErr, windows.ERROR_TRANSACTIONAL_CONFLICT) {
			t.Fatal("outside mutation failed for an unexpected reason", writeErr)
		}
		t.Log("native transaction excluded conflicting outside mutation; rollback must preserve exact original")
		rollbackInstalledRegistryFixture(t, n)
	} else {
		t.Log("outside mutation completed; original retirement must refuse without losing changed bytes")
		before = installedRegistrySnapshot(t, f.key)
	}
	if err := n.remove(context.Background()); err == nil || n.committed || !n.failed {
		t.Fatal("conflicting outside mutation did not revoke retirement", err)
	}
	if err := n.close(); err != nil {
		t.Fatal(err)
	}
	f.requireOriginal(t, before)
	if err := n.remove(context.Background()); err == nil {
		t.Fatal("conflicting registry deletion retried")
	}
}

func TestNativeInstalledRegistryDecommissionRejectsUnknownChangedAndAliasedValues(t *testing.T) {
	for _, fault := range []string{"extra value", "default value", "subkey", "changed value", "wrong value type", "case alias"} {
		t.Run(fault, func(t *testing.T) {
			f := newInstalledRegistryFixture(t)
			var err error
			switch fault {
			case "extra value":
				err = f.key.SetStringValue("UnknownFixture", "preserve")
			case "default value":
				err = f.key.SetStringValue("", "preserve")
			case "subkey":
				child, _, childErr := registry.CreateKey(f.key, "UnknownChild", registry.ALL_ACCESS|registry.WOW64_32KEY)
				err = childErr
				if child != 0 {
					err = errors.Join(err, child.Close())
				}
			case "changed value":
				err = f.key.SetStringValue("DisplayVersion", "changed original")
			case "wrong value type":
				err = f.key.SetDWordValue("DisplayVersion", 7)
			case "case alias":
				err = f.key.DeleteValue("DisplayName")
				if err == nil {
					err = f.key.SetStringValue("displayname", "SKAGER")
				}
			}
			if err != nil {
				t.Fatal(err)
			}
			before := installedRegistrySnapshot(t, f.key)
			n, err := openInstalledRegistryAt(context.Background(), f.sid, f.path, f.want)
			if n != nil {
				defer n.close()
			}
			if err == nil || n != nil && (n.committed || !n.closed) {
				t.Fatal("unknown/changed private registration acquired retirement authority", err)
			}
			f.requireOriginal(t, before)
		})
	}
}

func TestNativeInstalledRegistryRetirementUnsupportedAndCancelledNeverPlainDelete(t *testing.T) {
	for _, fault := range []string{"unsupported open", "unsupported delete", "unsupported commit", "cancelled open", "cancelled remove", "aborted transaction"} {
		t.Run(fault, func(t *testing.T) {
			f := newInstalledRegistryFixture(t)
			before := installedRegistrySnapshot(t, f.key)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			missing := windows.NewLazySystemDLL("ktmw32.dll").NewProc("SkagerRegistryFixtureMissingExport")
			if fault == "unsupported open" {
				original := installedOpenKeyTransacted
				installedOpenKeyTransacted = missing
				defer func() { installedOpenKeyTransacted = original }()
			}
			if fault == "cancelled open" {
				cancel()
			}
			n, err := openInstalledRegistryAt(ctx, f.sid, f.path, f.want)
			if n != nil {
				t.Cleanup(func() {
					if err := n.close(); err != nil {
						t.Error(err)
					}
				})
			}
			if fault == "unsupported open" || fault == "cancelled open" {
				if err == nil || n != nil {
					t.Fatal("unsupported/cancelled native transaction fell back to plain key ownership", err)
				}
				f.requireOriginal(t, before)
				return
			}
			if err != nil || n == nil {
				t.Fatal(err)
			}
			switch fault {
			case "unsupported delete":
				original := installedDeleteKey
				installedDeleteKey = missing
				defer func() { installedDeleteKey = original }()
			case "unsupported commit":
				original := installedCommitTransaction
				installedCommitTransaction = missing
				defer func() { installedCommitTransaction = original }()
			case "cancelled remove":
				cancel()
			case "aborted transaction":
				rollbackInstalledRegistryFixture(t, n)
			}
			if err := n.remove(ctx); err == nil || n.committed || !n.failed || !n.removeAttempted {
				t.Fatal("failed native registry transaction fell back to plain deletion", err)
			}
			if err := n.close(); err != nil {
				t.Fatal(err)
			}
			f.requireOriginal(t, before)
			if err := n.remove(context.Background()); err == nil {
				t.Fatal("failed native registry transaction retried")
			}
		})
	}
}

func TestNativeInstalledRegistryDecommissionTimeoutAbortsAndPreservesOriginal(t *testing.T) {
	f := newInstalledRegistryFixture(t)
	before := installedRegistrySnapshot(t, f.key)
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	n := f.owner(t, ctx)
	deadline := time.Now().Add(5 * time.Second)
	for {
		outcome, err := n.outcome()
		if err != nil {
			t.Fatal(err)
		}
		if outcome == 3 {
			break
		}
		if outcome != 1 || time.Now().After(deadline) {
			t.Fatal("bounded private registry transaction did not abort", outcome)
		}
		time.Sleep(10 * time.Millisecond)
	}
	if err := n.remove(context.Background()); err == nil || n.committed || !n.failed {
		t.Fatal("expired native transaction deleted original key", err)
	}
	if err := n.close(); err != nil {
		t.Fatal(err)
	}
	f.requireOriginal(t, before)
	if err := n.remove(context.Background()); err == nil {
		t.Fatal("expired native registry transaction retried")
	}
}

func TestNativeInstalledRegistryRetirementSecurityDistinguishesNotifyFromMutation(t *testing.T) {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil {
		t.Fatal(err)
	}
	sid := user.User.Sid.String()
	for _, tc := range []struct {
		name, rights string
		allowed      bool
	}{
		{"read with notification", "KR", true},
		{"set value", "0x2", false},
		{"create subkey", "0x4", false},
		{"create registry link", "0x20", false},
		{"delete", "SD", false},
		{"change DACL", "WD", false},
		{"change owner", "WO", false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sd, err := windows.SecurityDescriptorFromString("O:" + sid + "D:P(A;;KA;;;" + sid + ")(A;;" + tc.rights + ";;;AU)")
			if err != nil {
				t.Fatal(err)
			}
			if err := installedRegistrySecurity(sd, sid); (err == nil) != tc.allowed {
				t.Fatal("registry rights were confused with file rights", err)
			}
		})
	}
}
