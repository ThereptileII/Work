package brokerenroll

import (
	"context"
	"encoding/json"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerregistration"
)

func TestInstalledUninstallRetirementInventoryCannotAdoptMutableOwnership(t *testing.T) {
	s := &installedInventorySink{members: map[string]installedApprovedMember{}}
	ctx := context.Background()
	body := "independently authenticated payload"
	if err := s.WriteFile(ctx, "app/opencpn.exe", strings.NewReader(body), int64(len(body)), digest([]byte(body))); err != nil {
		t.Fatal(err)
	}
	helper := []byte("approved original Lifecycle helper")
	if err := s.WriteEvidence(ctx, "installer/engine/Lifecycle.ps1", helper, digest(helper)); err != nil {
		t.Fatal(err)
	}
	manifest := []byte("original approved package evidence")
	if err := s.WriteEvidence(ctx, "installer/package.json", manifest, digest(manifest)); err != nil {
		t.Fatal(err)
	}
	for _, name := range []string{"ownership.json", "owner.json", "state.json", "Maintain.exe", "maintenance/payload.zip", "installer/engine/foreign.ps1"} {
		if err := s.WriteEvidence(ctx, name, helper, digest(helper)); err == nil {
			t.Fatal("mutable/unapproved input extended deletion inventory", name)
		}
		if _, ok := s.members[name]; ok {
			t.Fatal("unapproved member adopted", name)
		}
	}
	for _, name := range []string{"app/../../profile", "app\\foreign", "app//extra", "profile/opencpn.ini", "app/OPENNAV_PORTABLE_PREVIEW"} {
		if err := s.WriteFile(ctx, name, strings.NewReader(body), int64(len(body)), digest([]byte(body))); err == nil {
			t.Fatal("unsafe/non-installed namespace adopted", name)
		}
	}
	if len(s.members) != 3 || s.members["Lifecycle.ps1"].sha != digest(helper) || s.members["maintenance/package.json"].sha != digest(manifest) {
		t.Fatal("closed original mapping changed", s.members)
	}
}

func TestInstalledUninstallRetirementRejectsSelfDeclaredBaselineBytes(t *testing.T) {
	f := newMaintenanceFixture(t)
	client, err := brokerregistration.ParseRetainedApprovedRequest(f.req.ClientRegistrationRequest, f.req.ApprovedClientRequestSHA256)
	if err != nil {
		t.Fatal(err)
	}
	baseline := newBaselineFixture()
	if inventory, err := collectInstalledInventory(context.Background(), maintenanceAuthority{client: client}, baseline); err == nil || inventory != nil {
		t.Fatal("mutable/self-declared archive granted removal authority")
	}
	if baseline.closed {
		t.Fatal("collector prematurely released caller baseline custody")
	}
}

func TestInstalledUninstallRetirementStateOnlyLocatesApprovedTargets(t *testing.T) {
	f := newMaintenanceFixture(t)
	client, err := brokerregistration.ParseRetainedApprovedRequest(f.req.ClientRegistrationRequest, f.req.ApprovedClientRequestSHA256)
	if err != nil {
		t.Fatal(err)
	}
	state := map[string]any{"owner": installedIntegrationOwner, "schema": 1, "current": strings.Repeat("2", 32), "previous": "", "stock": map[string]any{"path": client.Stock.Path, "sha256": client.Stock.ExecutableSHA256, "version": client.Stock.Version, "arch": client.Stock.Arch}, "shortcutModes": []string{"xnav", "legacy", "safe"}}
	raw, _ := json.Marshal(state)
	parsed, err := parseInstalledState(raw, client)
	if err != nil || parsed.Current == client.Installed.Generation {
		t.Fatal("LocalAppData locator was incorrectly tied to independent protected GUID", err)
	}
	for _, bad := range []string{
		strings.Replace(string(raw), `"schema":1`, `"schema":1,"schema":1`, 1),
		strings.Replace(string(raw), `"current":`, `"Current":`, 1),
		strings.Replace(string(raw), strings.Repeat("2", 32), "../profile", 1),
		strings.TrimSuffix(string(raw), "}") + `,"managedFiles":[{"path":"profile/opencpn.ini","sha256":"` + strings.Repeat("a", 64) + `"}]}`,
		strings.Replace(string(raw), client.Stock.ExecutableSHA256, strings.Repeat("a", 64), 1),
		string(raw) + `{}`,
	} {
		if _, err := parseInstalledState([]byte(bad), client); err == nil {
			t.Fatal("unapproved state extended authority", bad)
		}
	}
}
