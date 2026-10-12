package launcher

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"runtime"
	"testing"

	"golang.org/x/sys/windows"
)

// Private inert fixture: collect trustees from the original actual ACL for the
// existing strict parser, and use an unassigned service SID. No SCM/account
// creation or production fixed-broker lookup is replaced. Only this test's own
// original primary token object is modified; fixture cleanup restores its exact
// prefixture descriptor through a separately retained control handle.
func maintenanceFolderAccessFixture(t *testing.T) *brokerQueryAccess {
	t.Helper()
	process, err := windows.OpenProcess(windows.PROCESS_QUERY_INFORMATION|windows.PROCESS_QUERY_LIMITED_INFORMATION|windows.SYNCHRONIZE|windows.READ_CONTROL|windows.WRITE_DAC, false, windows.GetCurrentProcessId())
	if err != nil {
		t.Fatal(err)
	}
	access := &brokerQueryAccess{process: process}
	t.Cleanup(func() { access.Close() })
	if err = windows.OpenProcessToken(process, windows.TOKEN_QUERY|windows.READ_CONTROL|windows.WRITE_DAC, &access.token); err != nil {
		t.Fatal(err)
	}
	access.proof, err = brokerQueryIdentityOf(process, access.token)
	if err != nil {
		t.Fatal("own native primary token fixture", err)
	}
	access.processSD, err = brokerQueryReadSD(process)
	if err != nil {
		t.Fatal(err)
	}
	original, err := brokerQueryReadSD(windows.Handle(access.token))
	if err != nil {
		t.Fatal(err)
	}
	var control windows.Token
	if err = windows.OpenProcessToken(process, windows.TOKEN_QUERY|windows.READ_CONTROL|windows.WRITE_DAC, &control); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		runtime.LockOSThread()
		defer runtime.UnlockOSThread()
		defer control.Close()
		acl, _, err := original.DACL()
		if err != nil {
			t.Error(err)
			return
		}
		if err = windows.SetSecurityInfo(windows.Handle(control), windows.SE_KERNEL_OBJECT, windows.DACL_SECURITY_INFORMATION, nil, nil, acl, nil); err != nil {
			t.Error("restore owned inert fixture", err)
			return
		}
		actual, err := brokerQueryReadSD(windows.Handle(control))
		if err != nil || !brokerQuerySameSD(actual, original) {
			t.Error("inert fixture restore readback", err)
		}
	})
	raw, err := brokerQueryACLBytes(original)
	if err != nil {
		t.Fatal(err)
	}
	pos := 8
	for index := 0; index < int(binary.LittleEndian.Uint16(raw[4:6])); index++ {
		if pos+8 > len(raw) {
			t.Fatal("actual fixture ACL truncated")
		}
		size := int(binary.LittleEndian.Uint16(raw[pos+2 : pos+4]))
		if size < 16 || pos+size > len(raw) {
			t.Fatal("actual fixture ACE truncated")
		}
		access.trusted = append(access.trusted, bytes.Clone(raw[pos+8:pos+size]))
		pos += size
	}
	access.broker, err = windows.StringToSid("S-1-5-80-17171-27272-37373-47474-57575")
	if err != nil {
		t.Fatal(err)
	}
	access.tokenSD, err = brokerQueryGrant(windows.Handle(access.token), access.proof.sid, access.broker, brokerQueryTokenMask, access.trusted)
	if err != nil {
		t.Fatal("strict own-token query fixture", err)
	}
	return access
}

func TestNativeMaintenanceFolderAccessExactGrantAndRestore(t *testing.T) {
	if maintenanceFolderTokenMask != windows.TOKEN_QUERY|windows.TOKEN_IMPERSONATE || maintenanceFolderTokenMask&(windows.TOKEN_QUERY_SOURCE|windows.TOKEN_DUPLICATE|windows.TOKEN_ADJUST_PRIVILEGES|windows.TOKEN_ADJUST_GROUPS|windows.TOKEN_ADJUST_DEFAULT|windows.TOKEN_ADJUST_SESSIONID) != 0 {
		t.Fatal("maintenance mask is not exactly QUERY|IMPERSONATE")
	}

	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	access := maintenanceFolderAccessFixture(t)
	original := access.tokenSD
	process, token := access.process, access.token
	owner, err := prepareMaintenanceFolderAccessNative(context.Background(), access)
	if maintenanceFolderAdmin(access.token) != nil {
		if err == nil || owner != nil {
			t.Fatal("non-admin/filtered primary token received maintenance grant")
		}
		actual, readErr := brokerQueryReadSD(windows.Handle(token))
		if readErr != nil || !brokerQuerySameSD(original, actual) {
			t.Fatal("ineligible operator token mutated", readErr)
		}
		return
	}
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	if owner.process != process || owner.token != token || access.process != process || access.token != token {
		t.Fatal("original process/token custody replaced")
	}
	if err = access.Validate(context.Background()); err != nil {
		t.Fatal("strict ordinary query owner lost temporary expected descriptor", err)
	}
	if err = owner.Validate(context.Background()); err != nil {
		t.Fatal(err)
	}
	originalRaw, _ := brokerQueryACLBytes(original)
	expectedRaw, err := maintenanceFolderACL(originalRaw, brokerQuerySIDBytes(access.broker), access.trusted)
	if err != nil {
		t.Fatal(err)
	}
	actual, err := brokerQueryReadSD(windows.Handle(token))
	actualRaw, rawErr := brokerQueryACLBytes(actual)
	if err != nil || rawErr != nil || !bytes.Equal(expectedRaw, actualRaw) {
		t.Fatal("actual folder rights differ from exact mask transform", err, rawErr)
	}
	if err = owner.Close(); err != nil {
		t.Fatal(err)
	}
	actual, err = brokerQueryReadSD(windows.Handle(token))
	if err != nil || !brokerQuerySameSD(original, actual) || !brokerQuerySameSD(original, access.tokenSD) {
		t.Fatal("query-only descriptor not restored exactly", err)
	}
	if err = access.Validate(context.Background()); err != nil {
		t.Fatal("restored query owner refused", err)
	}
	if err = owner.Close(); err != nil {
		t.Fatal("repeat cleanup", err)
	}
}

func TestNativeMaintenanceFolderAccessCancellationStillRestores(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	access := maintenanceFolderAccessFixture(t)
	if maintenanceFolderAdmin(access.token) != nil {
		t.Skip("actual elevated enabled-administrator token required for restoration; refusal covered separately")
	}
	original := access.tokenSD
	owner, err := prepareMaintenanceFolderAccessNative(context.Background(), access)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if err = owner.Validate(ctx); !errors.Is(err, context.Canceled) {
		t.Fatal("cancellation not preserved", err)
	}
	if err = owner.Close(); !errors.Is(err, context.Canceled) {
		t.Fatal("cleanup lost uncertainty", err)
	}
	actual, err := brokerQueryReadSD(windows.Handle(access.token))
	if err != nil || !brokerQuerySameSD(original, actual) || !brokerQuerySameSD(original, access.tokenSD) {
		t.Fatal("cancellation prevented exact native-valid restoration", err)
	}
}

func TestNativeMaintenanceFolderAccessDescriptorDriftRefusesBlindRestore(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	access := maintenanceFolderAccessFixture(t)
	if maintenanceFolderAdmin(access.token) != nil {
		t.Skip("actual elevated enabled-administrator token required for restoration; refusal covered separately")
	}
	owner, err := prepareMaintenanceFolderAccessNative(context.Background(), access)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	raw, err := brokerQueryACLBytes(access.tokenSD)
	if err != nil {
		t.Fatal(err)
	}
	// Test-controlled, zero-right grant to the same inert broker SID: deliberately
	// differs from the exact temporary descriptor, so cleanup must not overwrite it.
	pos := 8
	for index := 0; index < int(binary.LittleEndian.Uint16(raw[4:6])); index++ {
		size := int(binary.LittleEndian.Uint16(raw[pos+2 : pos+4]))
		if bytes.Equal(raw[pos+8:pos+size], brokerQuerySIDBytes(access.broker)) {
			binary.LittleEndian.PutUint32(raw[pos+4:pos+8], 0)
			break
		}
		pos += size
	}
	drift, err := maintenanceFolderDescriptor(access.tokenSD, raw)
	if err != nil {
		t.Fatal(err)
	}
	acl, _, err := drift.DACL()
	if err != nil {
		t.Fatal(err)
	}
	if err = windows.SetSecurityInfo(windows.Handle(access.token), windows.SE_KERNEL_OBJECT, windows.DACL_SECURITY_INFORMATION, nil, nil, acl, nil); err != nil {
		t.Fatal(err)
	}
	if err = owner.Close(); err == nil {
		t.Fatal("descriptor drift blindly overwritten")
	}
	actual, readErr := brokerQueryReadSD(windows.Handle(access.token))
	if readErr != nil || !brokerQuerySameSD(drift, actual) {
		t.Fatal("cleanup overwrote unexpected token descriptor", readErr)
	}
	if access.Validate(context.Background()) == nil {
		t.Fatal("uncertain query owner revived")
	}
}

func TestNativeMaintenanceFolderAccessOriginalCustodyMismatchRefusesRestore(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	access := maintenanceFolderAccessFixture(t)
	if maintenanceFolderAdmin(access.token) != nil {
		t.Skip("actual elevated enabled-administrator token required; refusal covered separately")
	}
	owner, err := prepareMaintenanceFolderAccessNative(context.Background(), access)
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	expected := access.tokenSD
	access.proof.tokenID.LowPart++ // same SID/logon cannot replace exact original TokenID
	if err = owner.Close(); err == nil {
		t.Fatal("changed original token proof authorized restoration")
	}
	actual, readErr := brokerQueryReadSD(windows.Handle(access.token))
	if readErr != nil || !brokerQuerySameSD(expected, actual) {
		t.Fatal("lost custody blindly restored token DACL", readErr)
	}
}
