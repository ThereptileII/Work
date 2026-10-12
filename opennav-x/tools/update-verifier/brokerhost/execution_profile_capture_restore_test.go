package brokerhost

import (
	"bytes"
	"context"
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

const restoreINIOriginal = "[Settings]\r\nConfigVersionString=Version 5.12.3 Build old\r\nNavMessageShown=0\r\n[Connections]\r\nCOM8=OUTPUT;pilot-binding\r\n[PlugIns]\r\nPilot=enabled\r\n[Routes]\r\nCurrent=voyage\r\n[Settings/NMEADataSource]\r\nDataConnections=original-output\r\n"
const restoreINICurrent = "[Settings]\r\nConfigVersionString=Version 5.12.4 Build new\r\nNavMessageShown=1\r\n[Connections]\r\nCOM8=OUTPUT;pilot-binding\r\n[PlugIns]\r\nPilot=enabled\r\n[Routes]\r\nCurrent=voyage\r\n[Settings/NMEADataSource]\r\nDataConnections=original-output\r\n"

func TestExecutionProfileRestoreSourceDerivedINIAllowlist(t *testing.T) {
	for _, tc := range []struct {
		name, original, current string
		allow                   bool
	}{
		{"exact-reviewed-startup", restoreINIOriginal, restoreINICurrent, true},
		{"unchanged", restoreINIOriginal, restoreINIOriginal, true},
		{"line-ending-canonicalization", restoreINIOriginal, strings.ReplaceAll(restoreINICurrent, "\r\n", "\n"), true},
		{"connection", restoreINIOriginal, strings.ReplaceAll(restoreINICurrent, "COM8=OUTPUT", "COM8=INPUT"), false},
		{"route", restoreINIOriginal, strings.ReplaceAll(restoreINICurrent, "Current=voyage", "Current=another-voyage"), false},
		{"plugin-pilot", restoreINIOriginal, strings.ReplaceAll(restoreINICurrent, "Pilot=enabled", "Pilot=disabled"), false},
		{"nmea-pilot-binding", restoreINIOriginal, strings.ReplaceAll(restoreINICurrent, "DataConnections=original-output", "DataConnections=rewritten"), false},
		{"unknown-key", restoreINIOriginal, restoreINICurrent + "UnreviewedStartup=1\r\n", false},
		{"unknown-key-change", restoreINIOriginal + "Unknown=1\r\n", restoreINICurrent + "Unknown=2\r\n", false},
		{"comment-erased", restoreINIOriginal + ";original comment\r\n", restoreINICurrent, false},
		{"duplicate-key", restoreINIOriginal, restoreINICurrent + "DataConnections=duplicate\r\n", false},
		{"allowed-key-case-alias", restoreINIOriginal, strings.ReplaceAll(restoreINICurrent, "NavMessageShown", "navmessageshown"), false},
		{"duplicate-section", restoreINIOriginal, restoreINICurrent + "[Settings]\r\nNavMessageShown=1\r\n", false},
		{"unchanged-zero-nag", restoreINIOriginal, strings.ReplaceAll(restoreINICurrent, "NavMessageShown=1", "NavMessageShown=0"), true},
		{"unsupported-nag-value", restoreINIOriginal, strings.ReplaceAll(restoreINICurrent, "NavMessageShown=1", "NavMessageShown=2"), false},
		{"deleted-startup-key", restoreINIOriginal, strings.ReplaceAll(restoreINICurrent, "NavMessageShown=1\r\n", ""), false},
		{"invalid-version", restoreINIOriginal, strings.ReplaceAll(restoreINICurrent, "Version 5.12.4 Build new", "unreviewed"), false},
		{"malformed", restoreINIOriginal, restoreINICurrent + "broken-line\r\n", false},
		{"version-escape", restoreINIOriginal, strings.ReplaceAll(restoreINICurrent, "Build new", "Build new\\nInjected"), false},
		{"nul", restoreINIOriginal, restoreINICurrent + "\x00", false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			err := automaticINIChanges([]byte(tc.original), []byte(tc.current))
			if (err == nil) != tc.allow {
				t.Fatal("closed INI policy", err)
			}
		})
	}
}

type memoryProfileRestoreSource struct {
	*memoryProfileCaptureSource
	nextRestore                   *memoryProfileRestoreSource
	events                        *[]string
	applyFault, closeFaultRestore error
	onApply                       func()
	writes                        int
}

func (s *memoryProfileRestoreSource) inspectWritable(ctx context.Context) (profileRestoreSource, error) {
	*s.events = append(*s.events, "inspect")
	return s.nextRestore, ctx.Err()
}
func (s *memoryProfileRestoreSource) readRestoreFile(ctx context.Context, name string, limit int64) ([]byte, error) {
	if int64(len(s.files[name])) > limit {
		return nil, ErrExecutionProfileRestoreReviewRequired
	}
	return bytes.Clone(s.files[name]), ctx.Err()
}
func (s *memoryProfileRestoreSource) applyRestore(ctx context.Context, plan profileRestorePlan, check func() error) error {
	*s.events = append(*s.events, "apply")
	if err := check(); err != nil {
		return err
	}
	if s.onApply != nil {
		s.onApply()
	}
	if s.applyFault != nil {
		return s.applyFault
	}
	for _, op := range plan.Files {
		s.writes++
		s.files[op.Original.Path] = bytes.Clone(op.Bytes)
		for i, e := range s.entries {
			if e.Path == op.Original.Path {
				id := e.Identity
				s.entries[i] = op.Original
				s.entries[i].Identity = id
			}
		}
	}
	for _, op := range plan.Directories {
		for i, e := range s.entries {
			if e.Path == op.Path {
				s.entries[i] = op
			}
		}
	}
	return ctx.Err()
}
func (s *memoryProfileRestoreSource) close() error { s.closes++; return s.closeFaultRestore }

type memoryProfileRestoreAuthority struct {
	events                     *[]string
	binding                    profileRestoreBinding
	intent                     profileRestoreIntent
	validateFault, intentFault error
	onIntent                   func()
	onValidate                 func() error
}

func (a *memoryProfileRestoreAuthority) validateProfileRestore(ctx context.Context, b profileRestoreBinding) error {
	if a.binding != (profileRestoreBinding{}) && a.binding != b {
		return profileCaptureTestFault
	}
	if a.onValidate != nil {
		if err := a.onValidate(); err != nil {
			return err
		}
	}
	return errors.Join(ctx.Err(), a.validateFault)
}
func (a *memoryProfileRestoreAuthority) recordProfileRestoreIntent(ctx context.Context, intent profileRestoreIntent) error {
	*a.events = append(*a.events, "intent")
	a.intent = intent
	if a.onIntent != nil {
		a.onIntent()
	}
	return errors.Join(ctx.Err(), a.intentFault)
}
func memoryRestoreEntries(s *memoryProfileCaptureSource) {
	for i, e := range s.entries {
		if !e.Directory {
			e.Bytes = int64(len(s.files[e.Path]))
			e.SHA256 = captureDigest(s.files[e.Path])
			s.entries[i] = e
		}
	}
}
func profileRestoreFixture(t *testing.T) (*ExecutionProfileCapture, *memoryProfileRestoreSource, *memoryProfileRestoreAuthority, *memoryProfileCaptureArchive, *[]string) {
	return profileRestoreFixtureINI(t, restoreINIOriginal, restoreINICurrent)
}
func profileRestoreFixtureINI(t *testing.T, originalINI, currentINI string) (*ExecutionProfileCapture, *memoryProfileRestoreSource, *memoryProfileRestoreAuthority, *memoryProfileCaptureArchive, *[]string) {
	t.Helper()
	events := []string{}
	old := profileCaptureTestSource()
	old.files["opencpn.ini"] = []byte(originalINI)
	old.files["opencpn.log"] = []byte("original operational log\n")
	old.entries = append(old.entries, profileCaptureEntry{Path: "opencpn.log", Identity: "original-log-id", Attributes: 32, Created: 1, LastAccess: 2, LastWrite: 3, Security: "original owner/group/DACL"})
	memoryRestoreEntries(old)
	source := &memoryProfileRestoreSource{memoryProfileCaptureSource: old, events: &events}
	archive := &memoryProfileCaptureArchive{}
	p, err := completeProfileCapture(context.Background(), source, archive, brokerstore.ExecutionJournalSnapshot{ClientSID: "S-1-5-21-1-2-3-1001", ExecutionID: strings.Repeat("a", 32), Generation: strings.Repeat("b", 32), CompletionSHA256: strings.Repeat("c", 64)})
	if err != nil {
		t.Fatal(err)
	}
	now := profileCaptureTestSource()
	now.entries = append([]profileCaptureEntry(nil), old.entries...)
	now.files = map[string][]byte{}
	for name, raw := range old.files {
		now.files[name] = bytes.Clone(raw)
	}
	now.files["opencpn.ini"] = []byte(currentINI)
	now.files["opencpn.log"] = append(now.files["opencpn.log"], []byte("startup and close\n")...)
	memoryRestoreEntries(now)
	for i := range now.entries {
		if now.entries[i].Path == "opencpn.ini" {
			now.entries[i].Identity = "wx-atomic-replacement-full-id"
			now.entries[i].Created = 9
			now.entries[i].LastWrite = 10
		}
	}
	current := &memoryProfileRestoreSource{memoryProfileCaptureSource: now, events: &events}
	source.nextRestore = current
	if err = p.ReleaseForQualification(context.Background()); err != nil {
		t.Fatal(err)
	}
	return p, current, &memoryProfileRestoreAuthority{events: &events}, archive, &events
}
func TestExecutionProfileRestoreDurableIntentBeforeOriginalBytes(t *testing.T) {
	p, current, authority, archive, events := profileRestoreFixture(t)
	originalArchive := bytes.Clone(archive.raw)
	digest, err := p.restoreOriginalNative(context.Background(), authority)
	if err != nil {
		t.Fatal(err)
	}
	if digest != profileTreeDigest(p.state.manifest.Entries) || !bytes.Equal(current.files["opencpn.ini"], []byte(restoreINIOriginal)) || !bytes.Equal(current.files["opencpn.log"], []byte("original operational log\n")) || !bytes.Equal(originalArchive, archive.raw) {
		t.Fatal("original full tree or retained archive lost")
	}
	if strings.Join(*events, ",") != "inspect,intent,apply" || authority.intent.Binding.ExecutionID != strings.Repeat("a", 32) || authority.intent.PlanSHA256 == "" || authority.intent.CurrentManifestSHA256 == "" {
		t.Fatal("durable closed intent/order lost", *events)
	}
	if _, err = p.restoreOriginalNative(context.Background(), authority); err == nil {
		t.Fatal("one-use restore repeated")
	}
	if err = p.Close(); err != nil {
		t.Fatal(err)
	}
	if current.closes != 1 || archive.closes != 1 {
		t.Fatal("cleanup repeated")
	}
}
func TestExecutionProfileRestoreRefusesBeforeAnyOverwrite(t *testing.T) {
	for _, fault := range []string{"navigation", "connection", "pilot-binding", "unknown-ini", "log-rewrite", "log-rotation", "missing", "extra", "security", "attributes", "closed-child", "wrong-binding", "cancel"} {
		t.Run(fault, func(t *testing.T) {
			p, current, a, archive, events := profileRestoreFixture(t)
			defer p.Close()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "navigation":
				current.files["navigation/navobj.xml"] = []byte("new voyage")
			case "connection":
				current.files["opencpn.ini"] = bytes.ReplaceAll(current.files["opencpn.ini"], []byte("COM8=OUTPUT"), []byte("COM8=INPUT"))
			case "pilot-binding":
				current.files["opencpn.ini"] = bytes.ReplaceAll(current.files["opencpn.ini"], []byte("Pilot=enabled"), []byte("Pilot=disabled"))
			case "unknown-ini":
				current.files["opencpn.ini"] = append(current.files["opencpn.ini"], []byte("Unknown=1\r\n")...)
			case "log-rewrite":
				current.files["opencpn.log"] = []byte("rewritten log")
			case "log-rotation":
				current.entries = append(current.entries, profileCaptureEntry{Path: "opencpn.log.log"})
			case "missing":
				current.entries = current.entries[:len(current.entries)-1]
			case "extra":
				current.entries = append(current.entries, profileCaptureEntry{Path: "new-voyage.gpx"})
			case "security":
				current.entries[0].Security = "changed ACL"
			case "attributes":
				current.entries[0].Attributes = 999
			case "closed-child":
				a.validateFault = profileCaptureTestFault
			case "wrong-binding":
				a.binding = profileRestoreBinding{ExecutionID: strings.Repeat("f", 32)}
			case "cancel":
				cancel()
			}
			memoryRestoreEntries(current.memoryProfileCaptureSource)
			before := bytes.Clone(archive.raw)
			if _, err := p.restoreOriginalNative(ctx, a); err == nil {
				t.Fatal("unreviewed change accepted")
			}
			if current.writes != 0 || !bytes.Equal(archive.raw, before) || strings.Contains(strings.Join(*events, ","), "intent") {
				t.Fatal("refusal mutated bytes or published intent", *events)
			}
		})
	}
}
func TestExecutionProfileRestorePartialEffectsRemainUncertain(t *testing.T) {
	for _, fault := range []string{"intent", "cancel-after-intent", "apply", "post-apply-cancel", "close"} {
		t.Run(fault, func(t *testing.T) {
			p, current, a, archive, _ := profileRestoreFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "intent":
				a.intentFault = profileCaptureTestFault
			case "cancel-after-intent":
				a.onIntent = cancel
			case "apply":
				current.applyFault = profileCaptureTestFault
			case "post-apply-cancel":
				current.onApply = cancel
			case "close":
				current.closeFaultRestore = profileCaptureTestFault
			}
			before := bytes.Clone(archive.raw)
			_, err := p.restoreOriginalNative(ctx, a)
			if !errors.Is(err, ErrExecutionProfileRestoreUncertain) {
				t.Fatal("attempt uncertainty lost", err)
			}
			if (fault == "intent" || fault == "cancel-after-intent") && current.writes != 0 {
				t.Fatal("write before durable intent/after cancellation")
			}
			if !bytes.Equal(archive.raw, before) {
				t.Fatal("original archive altered")
			}
			if _, err = p.restoreOriginalNative(context.Background(), a); err == nil {
				t.Fatal("partial retried")
			}
			first := p.Close()
			if first == nil || p.Close() != first || current.closes != 1 || archive.closes != 1 {
				t.Fatal("cleanup not cached", first)
			}
		})
	}
}

// Real version-change source path (pinned37fd): ocpn_app.cpp1444-1447 calls
// SetUpgradeOptions. OCPNPlatform.cpp1522-1555 forces CompatOS*, Audio booleans
// and ToolbarX; GL-capable upgrade633-646 forces the exact8 graphics values.
// navutil.cpp2076-2077,2145,2162-2170,2234,2381-2383 persists them, then2617
// Flush commits. These customized prior settings are legitimate; no connection,
// route, pilot or plugin setting is changed by the reviewed fixture.
const restoreForcedINIOriginal = `[Settings]
;retain original owner comment
ConfigVersionString=Version 5.12.3 Build old
NavMessageShown=0
CompatOS=prior-custom-platform
CompatOsVersion=prior-custom-version
CmdSoundString=original-custom-command-never-executed
ToolbarX=137
OpenGL=0
UseAcceleratedPanning=0
GPUTextureCompression=0
GPUTextureCompressionCaching=0
GPUTextureDimension=1024
GPUTextureMemSize=128
PolygonSmoothing=0
LineSmoothing=0
UnreviewedSetting=unchanged
[Settings/Audio]
bAIS_GCPA_AlertAudio=0
bAIS_SART_AlertAudio=0
bAIS_DSC_AlertAudio=0
bAnchorAlertAudio=0
[Connections]
COM8=OUTPUT;pilot-binding
[PlugIns]
Pilot=enabled
[Routes]
Current=voyage
[Settings/NMEADataSource]
DataConnections=original-output
`
const restoreForcedINICurrent = `[Settings]
;retain original owner comment
ConfigVersionString=Version 5.12.4 Build new
NavMessageShown=1
CompatOS=
CompatOsVersion=
CmdSoundString=original-custom-command-never-executed
ToolbarX=-1
OpenGL=1
UseAcceleratedPanning=1
GPUTextureCompression=1
GPUTextureCompressionCaching=1
GPUTextureDimension=512
GPUTextureMemSize=64
PolygonSmoothing=1
LineSmoothing=1
UnreviewedSetting=unchanged
[Settings/Audio]
bAIS_GCPA_AlertAudio=1
bAIS_SART_AlertAudio=1
bAIS_DSC_AlertAudio=1
bAnchorAlertAudio=0
[Connections]
COM8=OUTPUT;pilot-binding
[PlugIns]
Pilot=enabled
[Routes]
Current=voyage
[Settings/NMEADataSource]
DataConnections=original-output
`

func TestExecutionProfileRestoreForcedUpgradeSettingsOriginalBytes(t *testing.T) {
	p, current, a, archive, events := profileRestoreFixtureINI(t, restoreForcedINIOriginal, restoreForcedINICurrent)
	originalArchive := bytes.Clone(archive.raw)
	digest, err := p.restoreOriginalNative(context.Background(), a)
	if err != nil {
		t.Fatal(err)
	}
	if digest != profileTreeDigest(p.state.manifest.Entries) || !bytes.Equal(current.files["opencpn.ini"], []byte(restoreForcedINIOriginal)) || !bytes.Equal(archive.raw, originalArchive) || strings.Join(*events, ",") != "inspect,intent,apply" {
		t.Fatal("customized original settings/backup/order not preserved")
	}
	if err = p.Close(); err != nil {
		t.Fatal(err)
	}
}
func TestExecutionProfileRestoreForcedSettingsRejectUnexpectedValuesAndAliases(t *testing.T) {
	for _, tc := range []struct{ name, from, to string }{
		{"compat-not-reset", "CompatOS=\n", "CompatOS=another-platform\n"},
		{"compat-version-not-reset", "CompatOsVersion=\n", "CompatOsVersion=another-version\n"},
		{"toolbar-not-forced", "ToolbarX=-1", "ToolbarX=0"},
		{"opengl-not-forced", "OpenGL=1", "OpenGL=2"},
		{"panning-not-forced", "UseAcceleratedPanning=1", "UseAcceleratedPanning=2"},
		{"texture-not-forced", "GPUTextureCompression=1", "GPUTextureCompression=2"},
		{"texture-cache-not-forced", "GPUTextureCompressionCaching=1", "GPUTextureCompressionCaching=2"},
		{"texture-dimension-not-forced", "GPUTextureDimension=512", "GPUTextureDimension=2048"},
		{"texture-memory-not-forced", "GPUTextureMemSize=64", "GPUTextureMemSize=256"},
		{"polygon-not-forced", "PolygonSmoothing=1", "PolygonSmoothing=true"},
		{"line-not-forced", "LineSmoothing=1", "LineSmoothing=enabled"},
		{"ais-audio-not-forced", "bAIS_GCPA_AlertAudio=1", "bAIS_GCPA_AlertAudio=true"},
		{"sart-audio-not-forced", "bAIS_SART_AlertAudio=1", "bAIS_SART_AlertAudio=2"},
		{"dsc-audio-not-forced", "bAIS_DSC_AlertAudio=1", "bAIS_DSC_AlertAudio=yes"},
		{"anchor-audio-still-unreviewed", "bAnchorAlertAudio=0", "bAnchorAlertAudio=1"},
		{"sound-command-serialization-unproven", "CmdSoundString=original-custom-command-never-executed", "CmdSoundString=PowerShell unproven-serialized-default"},
		{"field-case-alias", "CompatOS=", "compatos="},
		{"audio-field-case-alias", "bAIS_GCPA_AlertAudio=1", "bais_gcpa_alertaudio=1"},
		{"group-case-alias", "[Settings/Audio]", "[Settings/audio]"},
		{"unknown-key", "UnreviewedSetting=unchanged", "UnreviewedSetting=changed"},
		{"pilot-binding", "COM8=OUTPUT;pilot-binding", "COM8=INPUT;pilot-binding"},
		{"pilot-plugin", "Pilot=enabled", "Pilot=disabled"},
		{"route-data", "Current=voyage", "Current=other-voyage"},
		{"comments-preserved", ";retain original owner comment", ";unreviewed changed comment"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			changed := strings.Replace(restoreForcedINICurrent, tc.from, tc.to, 1)
			if changed == restoreForcedINICurrent {
				t.Fatal("fixture replacement missing")
			}
			p, current, a, archive, events := profileRestoreFixtureINI(t, restoreForcedINIOriginal, changed)
			defer p.Close()
			before := bytes.Clone(current.files["opencpn.ini"])
			backup := bytes.Clone(archive.raw)
			if _, err := p.restoreOriginalNative(context.Background(), a); !errors.Is(err, ErrExecutionProfileRestoreReviewRequired) {
				t.Fatal("unreviewed value/alias accepted", err)
			}
			if current.writes != 0 || !bytes.Equal(before, current.files["opencpn.ini"]) || !bytes.Equal(backup, archive.raw) || strings.Contains(strings.Join(*events, ","), "intent") {
				t.Fatal("closed-policy refusal overwrote originals/current bytes")
			}
		})
	}
}
