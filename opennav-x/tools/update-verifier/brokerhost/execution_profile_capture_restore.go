package brokerhost

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"example.com/opennav-update-verifier/brokerstore"
	"io"
	"sort"
	"strings"
	"unicode/utf8"
)

const maxAutomaticProfileFileBytes = 8 << 20

var ErrExecutionProfileRestoreUncertain = errors.New("profile restore attempted; original archive retained, no retry")

// These private capabilities are supplied only by the native host after its
// authenticated ORIGINAL child process handle is signaled. Neither a report nor
// this binding/intent data can construct teardown or restoration authority.
type profileRestoreBinding struct {
	ClientSID              string `json:"clientSid"`
	ExecutionID            string `json:"executionId"`
	Generation             string `json:"generation"`
	CompletionSHA256       string `json:"completionSha256"`
	OriginalManifestSHA256 string `json:"originalManifestSha256"`
}
type profileRestoreIntent struct {
	Schema                int                   `json:"schema"`
	Purpose               string                `json:"purpose"`
	Binding               profileRestoreBinding `json:"binding"`
	CurrentManifestSHA256 string                `json:"currentManifestSha256"`
	PlanSHA256            string                `json:"planSha256"`
}
type executionProfileRestoreAuthority interface {
	validateProfileRestore(context.Context, profileRestoreBinding) error
	recordProfileRestoreIntent(context.Context, profileRestoreIntent) error
}
type profileRestoreFile struct {
	Original, Current profileCaptureEntry
	Bytes             []byte
}
type profileRestorePlan struct {
	Files                                 []profileRestoreFile
	Directories                           []profileCaptureEntry
	OriginalTreeSHA256, CurrentTreeSHA256 string
}
type profileRestoreSource interface {
	profileCaptureSource
	readRestoreFile(context.Context, string, int64) ([]byte, error)
	applyRestore(context.Context, profileRestorePlan, func() error) error
}
type profileRestoreFactory interface {
	inspectWritable(context.Context) (profileRestoreSource, error)
}

// Pinned startup policy: OpenCPN37fd0cddb7334fe489e9f18aa163977a9c5c84f7.
// ocpn_app.cpp:1937-1957 sets NavMessageShown=1 and ConfigVersionString.
// OCPNPlatform.cpp:1495-1555 (version change) resets CompatOS/CompatOsVersion
// to empty, the three AIS-specific audio booleans to true and ToolbarX=-1.
// OCPNPlatform.cpp:633-646 (GL-capable upgrade) forces OpenGL plus seven GL
// defaults below. navutil.cpp:2076-2081,2145,2162-2170,2234,2381-2383 persists
// these reviewed values. Later toolbar.cpp:433-457 recalculated coordinates,
// canvas geometry/default normalization and all other startup writes REFUSE.
// CmdSoundString is not admitted yet: SoundConfig.cmake:10-28 selects a build-
// dependent command; its actual Windows INI serialization must be pinned first.
// base_platform.cpp:671-679 rotates logs above1,000,000 bytes (refused here);
// logger.cpp:83-84 opens out|app, so only exact original-prefix append is allowed.
// No navigation/connection/pilot/plugin fields are approved by these changes.
func automaticStartupINIValues() map[string]string {
	// Fresh private map; callers cannot mutate a global policy or supply values.
	return map[string]string{
		"/Settings/ConfigVersionString":          "", // separately checked version shape
		"/Settings/NavMessageShown":              "1",
		"/Settings/CompatOS":                     "",
		"/Settings/CompatOsVersion":              "",
		"/Settings/ToolbarX":                     "-1",
		"/Settings/OpenGL":                       "1",
		"/Settings/UseAcceleratedPanning":        "1",
		"/Settings/GPUTextureCompression":        "1",
		"/Settings/GPUTextureCompressionCaching": "1",
		"/Settings/GPUTextureDimension":          "512",
		"/Settings/GPUTextureMemSize":            "64",
		"/Settings/PolygonSmoothing":             "1",
		"/Settings/LineSmoothing":                "1",
		"/Settings/Audio/bAIS_GCPA_AlertAudio":   "1",
		"/Settings/Audio/bAIS_SART_AlertAudio":   "1",
		"/Settings/Audio/bAIS_DSC_AlertAudio":    "1",
	}
}
func automaticProfileINI(raw []byte) (map[string]string, []string, error) {
	if len(raw) > maxAutomaticProfileFileBytes || !utf8.Valid(raw) || bytes.ContainsRune(raw, 0) {
		return nil, nil, ErrExecutionProfileRestoreReviewRequired
	}
	raw = bytes.TrimPrefix(raw, []byte{0xef, 0xbb, 0xbf})
	values := map[string]string{}
	canonicalKeys := map[string]string{}
	for key := range automaticStartupINIValues() {
		canonicalKeys[strings.ToLower(key)] = key
	}
	known := map[string]bool{}
	sections := map[string]bool{}
	var fixed []string
	group := ""
	for _, line := range strings.Split(string(raw), "\n") {
		line = strings.TrimSuffix(line, "\r")
		if strings.ContainsRune(line, '\r') {
			return nil, nil, ErrExecutionProfileRestoreReviewRequired
		}
		trimmed := strings.TrimSpace(line)
		if trimmed == "" {
			continue
		}
		if strings.HasPrefix(trimmed, "[") {
			if line != trimmed || !strings.HasSuffix(line, "]") || strings.Count(line, "[") != 1 || strings.Count(line, "]") != 1 {
				return nil, nil, ErrExecutionProfileRestoreReviewRequired
			}
			group = line[1 : len(line)-1]
			lower := strings.ToLower(group)
			if group == "" || sections[lower] || strings.ContainsAny(group, "\\\x00") {
				return nil, nil, ErrExecutionProfileRestoreReviewRequired
			}
			sections[lower] = true
			fixed = append(fixed, "section:"+group)
			continue
		}
		if strings.HasPrefix(trimmed, "#") || strings.HasPrefix(trimmed, ";") {
			fixed = append(fixed, "comment:"+group+":"+line)
			continue
		}
		key, value, ok := strings.Cut(line, "=")
		if !ok || key == "" || key != strings.TrimSpace(key) || strings.ContainsAny(key, "/\\[]") {
			return nil, nil, ErrExecutionProfileRestoreReviewRequired
		}
		path := "/" + group + "/" + key
		lower := strings.ToLower(path)
		if known[lower] {
			return nil, nil, ErrExecutionProfileRestoreReviewRequired
		}
		known[lower] = true
		if canonical, ok := canonicalKeys[lower]; ok {
			if path != canonical {
				return nil, nil, ErrExecutionProfileRestoreReviewRequired
			}
			values[path] = value
		} else {
			fixed = append(fixed, "entry:"+path+"="+value)
		}
	}
	sort.Strings(fixed)
	return values, fixed, nil
}
func automaticINIChanges(original, current []byte) error {
	before, oldFixed, err := automaticProfileINI(original)
	if err != nil {
		return err
	}
	after, newFixed, err := automaticProfileINI(current)
	if err != nil {
		return err
	}
	if !equalProfileStrings(oldFixed, newFixed) {
		return ErrExecutionProfileRestoreReviewRequired
	}
	for key := range before {
		if _, ok := after[key]; !ok {
			return ErrExecutionProfileRestoreReviewRequired
		}
	}
	forced := automaticStartupINIValues()
	for key, value := range after {
		if old, ok := before[key]; ok && old == value {
			continue
		}
		switch key {
		case "/Settings/NavMessageShown":
			if value != "1" || (before[key] != "" && before[key] != "0") {
				return ErrExecutionProfileRestoreReviewRequired
			}
		case "/Settings/ConfigVersionString":
			if len(value) > 256 || strings.ContainsAny(value, "\\[]") || !strings.HasPrefix(value, "Version ") || !strings.Contains(value, " Build ") {
				return ErrExecutionProfileRestoreReviewRequired
			}
			for _, r := range value {
				if r < 32 || r > 126 {
					return ErrExecutionProfileRestoreReviewRequired
				}
			}
		default:
			expected, ok := forced[key]
			if !ok || value != expected {
				return ErrExecutionProfileRestoreReviewRequired
			}
		}
	}
	return nil
}
func equalProfileStrings(a, b []string) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}
func profileTreeDigest(entries []profileCaptureEntry) string {
	copyEntries := append([]profileCaptureEntry(nil), entries...)
	for i := range copyEntries {
		copyEntries[i].Identity = ""
		copyEntries[i].LastAccess = 0
	}
	sort.Slice(copyEntries, func(i, j int) bool { return copyEntries[i].Path < copyEntries[j].Path })
	raw, _ := json.Marshal(struct {
		Purpose string
		Entries []profileCaptureEntry
	}{"restored-original-profile-tree-v1", copyEntries})
	return captureDigest(raw)
}
func (s *profileCaptureState) originalFile(ctx context.Context, name string) ([]byte, error) {
	offset := int64(8 + len(s.original))
	for _, e := range s.manifest.Entries {
		if e.Directory {
			continue
		}
		if e.Path != name {
			offset += e.Bytes
			continue
		}
		if e.Bytes < 0 || e.Bytes > maxAutomaticProfileFileBytes {
			return nil, ErrExecutionProfileRestoreReviewRequired
		}
		raw := make([]byte, int(e.Bytes))
		for count := 0; count < len(raw); {
			end := count + (64 << 10)
			if end > len(raw) {
				end = len(raw)
			}
			expected := end - count
			n, err := s.archive.ReadAt(ctx, raw[count:end], offset+int64(count))
			count += n
			if err != nil && !errors.Is(err, io.EOF) {
				return nil, err
			}
			if n != expected {
				return nil, io.ErrUnexpectedEOF
			}
		}
		if captureDigest(raw) != e.SHA256 {
			return nil, ErrExecutionProfileCaptureRefused
		}
		return raw, nil
	}
	return nil, ErrExecutionProfileRestoreReviewRequired
}
func buildAutomaticProfileRestore(ctx context.Context, s *profileCaptureState, current profileRestoreSource) (profileRestorePlan, error) {
	entries, parents := current.originals()
	if !equalProfileStrings(parents, s.manifest.Ancestors) || len(entries) != len(s.manifest.Entries) {
		return profileRestorePlan{}, ErrExecutionProfileRestoreReviewRequired
	}
	observed := map[string]profileCaptureEntry{}
	for _, e := range entries {
		if _, ok := observed[e.Path]; ok {
			return profileRestorePlan{}, ErrExecutionProfileRestoreReviewRequired
		}
		observed[e.Path] = e
	}
	plan := profileRestorePlan{OriginalTreeSHA256: profileTreeDigest(s.manifest.Entries), CurrentTreeSHA256: profileTreeDigest(entries)}
	for _, original := range s.manifest.Entries {
		now, ok := observed[original.Path]
		if !ok || original.Directory != now.Directory {
			return plan, ErrExecutionProfileRestoreReviewRequired
		}
		before, after := original, now
		before.Identity = ""
		after.Identity = ""
		before.LastAccess = 0
		after.LastAccess = 0
		if before == after {
			continue
		}
		if original.Directory {
			after.LastWrite = before.LastWrite
			if before != after {
				return plan, ErrExecutionProfileRestoreReviewRequired
			}
			plan.Directories = append(plan.Directories, original)
			continue
		}
		if original.Path != "opencpn.ini" && original.Path != "opencpn.log" {
			return plan, ErrExecutionProfileRestoreReviewRequired
		}
		after.Bytes = before.Bytes
		after.SHA256 = before.SHA256
		after.Created = before.Created
		after.LastWrite = before.LastWrite
		if before != after {
			return plan, ErrExecutionProfileRestoreReviewRequired
		}
		old, err := s.originalFile(ctx, original.Path)
		if err != nil {
			return plan, err
		}
		raw, err := current.readRestoreFile(ctx, original.Path, maxAutomaticProfileFileBytes)
		if err != nil {
			return plan, err
		}
		if int64(len(raw)) != now.Bytes || captureDigest(raw) != now.SHA256 {
			return plan, ErrExecutionProfileCaptureRefused
		}
		if original.Path == "opencpn.ini" {
			if err = automaticINIChanges(old, raw); err != nil {
				return plan, err
			}
		} else if len(old) > 1000000 || !bytes.HasPrefix(raw, old) {
			return plan, ErrExecutionProfileRestoreReviewRequired
		}
		plan.Files = append(plan.Files, profileRestoreFile{original, now, old})
	}
	return plan, nil
}

// restoreOriginalNative is a one-use mutation capability boundary, not a report
// handler. It preserves the full archive on refusal/partial effects. The caller
// must supply the opaque native original-child closure owner and durable journal
// intent implementation; strings, READY and client close reports are not proof.
func (p *ExecutionProfileCapture) restoreOriginalNative(ctx context.Context, authority executionProfileRestoreAuthority) (digest string, resultErr error) {
	if p == nil || p.state == nil || authority == nil {
		return "", ErrExecutionProfileCaptureRefused
	}
	s := p.state
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.ready(ctx); err != nil {
		return "", err
	}
	if !s.released || s.restoreAttempted {
		return "", ErrExecutionProfileCaptureRefused
	}
	binding := profileRestoreBinding{s.manifest.ClientSID, s.manifest.ExecutionID, s.manifest.Generation, s.manifest.CompletionSHA256, captureDigest(s.original)}
	// Full archive validation happens before and after the mutation. Per-chunk
	// gates check revocation/ancestry/actual child proof without hashing up to
	// 2 GiB of unchanged archive for every 64 KiB output write.
	check := func() error {
		return errors.Join(ctx.Err(), s.owner.Err(), s.failure, s.source.validate(ctx, false), authority.validateProfileRestore(ctx, binding))
	}
	if err := check(); err != nil {
		return "", err
	}
	factory, ok := s.source.(profileRestoreFactory)
	if !ok {
		return "", ErrExecutionProfileCaptureRefused
	}
	current, err := factory.inspectWritable(ctx)
	if err != nil {
		return "", err
	}
	defer func() {
		resultErr = errors.Join(resultErr, current.close())
		if resultErr != nil && s.restoreAttempted {
			s.failure = errors.Join(ErrExecutionProfileRestoreUncertain, resultErr)
			resultErr = s.failure
		}
	}()
	plan, err := buildAutomaticProfileRestore(ctx, s, current)
	if err != nil {
		return "", err
	}
	if err = errors.Join(check(), current.validate(ctx, true)); err != nil {
		return "", err
	}
	raw, err := json.Marshal(plan)
	if err != nil {
		return "", err
	}
	_, currentRaw, _, err := profileManifestFromRestore(s, current)
	if err != nil {
		return "", err
	}
	intent := profileRestoreIntent{1, "automatic-original-profile-restore-v1", binding, captureDigest(currentRaw), captureDigest(raw)}
	// Once durable intent is attempted, a failure is uncertain. No reuse/retry.
	s.restoreAttempted = true
	if err = authority.recordProfileRestoreIntent(ctx, intent); err != nil {
		return "", err
	}
	if err = errors.Join(check(), current.validate(ctx, true)); err != nil {
		return "", err
	}
	if err = current.applyRestore(ctx, plan, check); err != nil {
		return "", err
	}
	if err = errors.Join(s.ready(ctx), check(), current.validate(ctx, true)); err != nil {
		return "", err
	}
	entries, _ := current.originals()
	digest = profileTreeDigest(entries)
	if digest != plan.OriginalTreeSHA256 {
		return "", ErrExecutionProfileRestoreUncertain
	}
	return digest, nil
}
func profileManifestFromRestore(s *profileCaptureState, current profileCaptureSource) (profileCaptureManifest, []byte, int64, error) {
	return profileManifest(brokerstore.ExecutionJournalSnapshot{ClientSID: s.manifest.ClientSID, ExecutionID: s.manifest.ExecutionID, Generation: s.manifest.Generation, CompletionSHA256: s.manifest.CompletionSHA256}, current)
}
