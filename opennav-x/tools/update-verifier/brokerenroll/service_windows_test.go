//go:build windows

package brokerenroll

import (
	"errors"
	"testing"
	"unsafe"

	"golang.org/x/sys/windows"
)

func TestServiceMutationDiagnosticPreservesOriginalDescriptorAndBoundary(t *testing.T) {
	sd, err := windows.SecurityDescriptorFromString("O:SYD:P(A;;0xf003f;;;SY)(A;CI;0x2;;;AU)")
	if err != nil {
		t.Fatal(err)
	}
	mutation := uint32(windows.DELETE | windows.WRITE_DAC | windows.WRITE_OWNER | windows.SC_MANAGER_CREATE_SERVICE)
	for _, boundary := range []struct {
		location serviceSecurityLocation
		name     string
	}{
		{serviceSecurityLocation{role: serviceSecurityManager}, "sc-manager"},
		{serviceSecurityLocation{role: serviceSecurityRegistryAncestor, ancestor: 4}, "registry-ancestor"},
		{serviceSecurityLocation{role: serviceSecurityInitialService}, "initial-service"},
	} {
		refusal := serviceDescriptorSafeAt(sd, mutation, boundary.location)
		var observed *serviceMutationError
		if !errors.As(errors.Join(refusal, errors.New("fixture cleanup")), &observed) ||
			refusal.Error() != "untrusted machine mutation permission" || observed.location != boundary.location ||
			observed.location.role.String() != boundary.name || observed.ownerSID != "S-1-5-18" ||
			observed.aceSID != "S-1-5-11" || observed.mask != 2 || observed.flags != windows.CONTAINER_INHERIT_ACE || observed.mutation != mutation {
			t.Fatal("original descriptor rejection facts or sanitized error changed", refusal)
		}
	}
	for _, sddl := range []string{
		"O:SYD:P(A;;0xf003f;;;SY)(A;;0x20015;;;AU)",
		"O:SYD:P(A;;0xf003f;;;SY)(A;CIIO;0x2;;;AU)",
		"O:SYD:P(A;;0xf003f;;;SY)(D;;0x2;;;AU)",
	} {
		accepted, err := windows.SecurityDescriptorFromString(sddl)
		if err != nil {
			t.Fatal(err)
		}
		if err := serviceDescriptorSafeAt(accepted, mutation, serviceSecurityLocation{role: serviceSecurityManager}); err != nil {
			t.Fatal("diagnostic changed existing acceptance", err)
		}
	}
	badOwner, err := windows.SecurityDescriptorFromString("O:AUD:P(A;;0x2;;;AU)")
	if err != nil {
		t.Fatal(err)
	}
	refusal := serviceDescriptorSafeAt(badOwner, mutation, serviceSecurityLocation{role: serviceSecurityManager})
	var observed *serviceMutationError
	if refusal == nil || errors.As(refusal, &observed) || refusal.Error() != "trusted machine owner required" {
		t.Fatal("owner failure incorrectly relabeled as observed ACE rejection", refusal)
	}
}

func TestServiceManagerAndServiceMutationRightsRemainDistinct(t *testing.T) {
	common := uint32(windows.DELETE | windows.WRITE_DAC | windows.WRITE_OWNER | windows.GENERIC_ALL | windows.GENERIC_WRITE | windows.GENERIC_EXECUTE)
	manager := common | windows.SC_MANAGER_CREATE_SERVICE | windows.SC_MANAGER_LOCK | windows.SC_MANAGER_MODIFY_BOOT_CONFIG
	service := common | windows.SERVICE_CHANGE_CONFIG | windows.SERVICE_START | windows.SERVICE_STOP | windows.SERVICE_PAUSE_CONTINUE
	for _, test := range []struct {
		name     string
		role     serviceSecurityRole
		mutation uint32
		mask     string
		refuse   bool
	}{
		{"manager-query-lock", serviceSecurityManager, manager, "0x10", false},
		{"service-start", serviceSecurityInitialService, service, "0x10", true},
		{"manager-lock", serviceSecurityManager, manager, "0x8", true},
		{"service-enumerate-dependents", serviceSecurityInitialService, service, "0x8", false},
		{"manager-connect", serviceSecurityManager, manager, "0x1", false},
		{"manager-enumerate", serviceSecurityManager, manager, "0x4", false},
		{"manager-read-control", serviceSecurityManager, manager, "0x20000", false},
		{"manager-local-default", serviceSecurityManager, manager, "0x20015", false},
		{"manager-create", serviceSecurityManager, manager, "0x2", true},
		{"manager-boot-config", serviceSecurityManager, manager, "0x20", true},
		{"manager-delete", serviceSecurityManager, manager, "0x10000", true},
		{"manager-write-dacl", serviceSecurityManager, manager, "0x40000", true},
		{"manager-write-owner", serviceSecurityManager, manager, "0x80000", true},
		{"manager-generic-write", serviceSecurityManager, manager, "GW", true},
		{"manager-generic-execute", serviceSecurityManager, manager, "GX", true},
		{"manager-generic-all", serviceSecurityManager, manager, "GA", true},
	} {
		t.Run(test.name, func(t *testing.T) {
			sd, err := windows.SecurityDescriptorFromString("O:SYD:P(A;;0xf003f;;;SY)(A;;" + test.mask + ";;;AU)")
			if err != nil {
				t.Fatal(err)
			}
			refusal := serviceDescriptorSafeAt(sd, test.mutation, serviceSecurityLocation{role: test.role})
			if (refusal != nil) != test.refuse {
				t.Fatal("object-specific mutation policy changed", refusal)
			}
			if test.refuse {
				var observed *serviceMutationError
				if !errors.As(refusal, &observed) || observed.location.role != test.role || observed.mutation != test.mutation {
					t.Fatal("object-specific rejection diagnostic changed", refusal)
				}
			}
		})
	}
}

// Inert descriptor parsing only. These tests never open SCM, create a service,
// change machine security, run a child or qualify native enrollment.
func TestServiceCreationDescriptorRejectsUntrustedMutation(t *testing.T) {
	mutation := uint32(windows.SERVICE_CHANGE_CONFIG | windows.SERVICE_START | windows.SERVICE_STOP | windows.SERVICE_PAUSE_CONTINUE | windows.DELETE | windows.WRITE_DAC | windows.WRITE_OWNER | windows.GENERIC_ALL | windows.GENERIC_WRITE | windows.GENERIC_EXECUTE)
	for _, sddl := range []string{
		"O:SYD:(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x2018d;;;AU)",
		"O:BAD:(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20005;;;AU)",
	} {
		sd, err := windows.SecurityDescriptorFromString(sddl)
		if err != nil {
			t.Fatal(err)
		}
		if err := serviceDescriptorSafe(sd, mutation); err != nil {
			t.Fatalf("safe service default rights rejected: %v", err)
		}
	}
	for _, sddl := range []string{
		"O:AUD:(A;;0xf01ff;;;SY)", "O:SYD:NO_ACCESS_CONTROL",
		"O:SYD:(A;;0x10;;;AU)", "O:SYD:(A;;0x20;;;AU)",
		"O:SYD:(A;;0x2;;;AU)", "O:SYD:(A;;0x10000;;;AU)",
		"O:SYD:(A;;0x40000;;;AU)", "O:SYD:(A;;0x80000;;;AU)",
		"O:SYD:(A;;GW;;;AU)", "O:SYD:(A;;GX;;;AU)",
	} {
		sd, err := windows.SecurityDescriptorFromString(sddl)
		if err != nil {
			t.Fatal(err)
		}
		if serviceDescriptorSafe(sd, mutation) == nil {
			t.Fatalf("unsafe descriptor accepted: %s", sddl)
		}
	}
}

func TestFinalServiceDescriptorIsExactAndQueryOnly(t *testing.T) {
	base := "D:P(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20005;;;AU)"
	expected, err := windows.SecurityDescriptorFromString(base)
	if err != nil {
		t.Fatal(err)
	}
	good, err := windows.SecurityDescriptorFromString("O:BA" + base)
	if err != nil {
		t.Fatal(err)
	}
	if err := finalServiceDescriptor(good, expected); err != nil {
		t.Fatal(err)
	}
	for _, bad := range []string{
		"O:AU" + base,
		"O:BAD:(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20005;;;AU)",
		"O:BAD:P(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20105;;;AU)",
		"O:BAD:P(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20015;;;AU)",
		"O:BAD:P(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)",
		"O:BAD:P(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20005;;;AU)(A;;0x20005;;;WD)",
	} {
		sd, err := windows.SecurityDescriptorFromString(bad)
		if err != nil {
			t.Fatal(err)
		}
		if finalServiceDescriptor(sd, expected) == nil {
			t.Fatalf("changed final descriptor accepted: %s", bad)
		}
	}
}

func TestServiceRegistryRequiresExactResolvedControlSetAncestry(t *testing.T) {
	held := []string{`\REGISTRY\MACHINE\SYSTEM`, `\REGISTRY\MACHINE\SYSTEM\Select`, `\REGISTRY\MACHINE\SYSTEM\ControlSet001`, `\REGISTRY\MACHINE\SYSTEM\ControlSet001\Services`}
	for i, path := range []string{`SYSTEM`, `SYSTEM\Select`, `SYSTEM\CurrentControlSet`, `SYSTEM\CurrentControlSet\Services`} {
		if !fixedRegistryName(path, held[i], held) {
			t.Fatalf("expected registry ancestry refused: %s", path)
		}
	}
	if !fixedRegistryName(`SYSTEM\CurrentControlSet\Services\`+serviceName, held[3]+`\`+serviceName, held) {
		t.Fatal("exact native service registry key refused")
	}
	for _, v := range [][2]string{
		{`SYSTEM`, `\REGISTRY\USER\SYSTEM`},
		{`SYSTEM\CurrentControlSet`, `\REGISTRY\MACHINE\SYSTEM\CurrentControlSet`},
		{`SYSTEM\CurrentControlSet`, `\REGISTRY\MACHINE\SYSTEM\ControlSet001\Elsewhere`},
		{`SYSTEM\CurrentControlSet\Services`, `\REGISTRY\MACHINE\SYSTEM\ControlSet002\Services`},
		{`SYSTEM\CurrentControlSet\Services\` + serviceName, held[3] + `\OtherService`},
	} {
		if fixedRegistryName(v[0], v[1], held) {
			t.Fatalf("substituted registry ancestry accepted: %v", v)
		}
	}
}

func TestServiceRejectsNormalOrAliasedStartBeforeSCMCall(t *testing.T) {
	n := &nativeServiceBackend{}
	for _, mode := range []string{"", "serve", "Initialize-store", "verify-store extra"} {
		if n.start(mode) == nil {
			t.Fatalf("unexpected service mode accepted: %q", mode)
		}
	}
}

func TestServiceConfigurationStringsStayInsideReturnedBuffer(t *testing.T) {
	buffer := []byte{'o', 0, 'k', 0, 0, 0}
	p := (*uint16)(unsafe.Pointer(&buffer[0]))
	if value, err := boundedServiceString(buffer, p); err != nil || value != "ok" {
		t.Fatalf("bounded native string: %q %v", value, err)
	}
	other := []uint16{'x', 0}
	if _, err := boundedServiceString(buffer, &other[0]); err == nil {
		t.Fatal("external native string pointer accepted")
	}
	if _, err := boundedServiceString(buffer, (*uint16)(unsafe.Pointer(&buffer[1]))); err == nil {
		t.Fatal("unaligned native string pointer accepted")
	}
	if _, err := boundedServiceString(buffer[:4], p); err == nil {
		t.Fatal("unterminated native string accepted")
	}
	if value, err := boundedServiceString(buffer, nil); err != nil || value != "" {
		t.Fatal("absent optional native string refused")
	}
}
