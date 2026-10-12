package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"runtime"
	"strings"

	"example.com/opennav-update-verifier/brokerregistration"
)

// GenerationPurpose describes protected creation provenance, not selection,
// commissioning, execution or health. It cannot be used as a launch permit.
type GenerationPurpose string

const (
	GenerationPurposeSignedCandidate  GenerationPurpose = "signed-candidate-generation"
	GenerationPurposeApprovedBaseline GenerationPurpose = approvedBaselineGenerationPurpose
)

// ReopenedGeneration retains independently opened readonly native file objects.
// Only fixed protected original records and full physical identities can produce
// it. Caller receipts, paths and present permissions are never reopen evidence.
// Closing its Store or this owner revokes every derived member capability.
type ReopenedGeneration struct {
	owner   *ReadOnlyGeneration
	purpose GenerationPurpose
}

func (r *ReopenedGeneration) Purpose() GenerationPurpose {
	if r == nil {
		return ""
	}
	return r.purpose
}
func (r *ReopenedGeneration) Validate() error {
	if r == nil || r.owner == nil {
		return ErrClosed
	}
	return r.owner.Validate()
}
func (r *ReopenedGeneration) OpenMember(name string) (*GenerationReadHandle, error) {
	if r == nil || r.owner == nil {
		return nil, ErrClosed
	}
	return r.owner.OpenMember(name)
}
func (r *ReopenedGeneration) Close() error {
	if r == nil || r.owner == nil {
		return nil
	}
	return r.owner.Close()
}
func (r *ReopenedGeneration) Candidate() (*ReadOnlyGeneration, error) {
	if err := r.checkPurpose(GenerationPurposeSignedCandidate); err != nil {
		return nil, err
	}
	return r.owner, nil
}
func (r *ReopenedGeneration) ApprovedBaseline() (*ReadOnlyApprovedBaselineGeneration, error) {
	if err := r.checkPurpose(GenerationPurposeApprovedBaseline); err != nil {
		return nil, err
	}
	return &ReadOnlyApprovedBaselineGeneration{owner: r.owner}, nil
}
func (r *ReopenedGeneration) checkPurpose(p GenerationPurpose) error {
	if r == nil || r.owner == nil || r.owner.generation == nil {
		return ErrClosed
	}
	g := r.owner.generation
	g.store.mu.Lock()
	defer g.store.mu.Unlock()
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := r.owner.ready(); err != nil {
		return err
	}
	if r.purpose != p || GenerationPurpose(g.record.Purpose) != p {
		return errors.New("protected generation has different creation purpose")
	}
	return nil
}

type generationReopenEvidence struct {
	completion, baselineCompletion []byte
	lineage                        generationBaselineOrigin
}
type generationReopenStoreBackend interface {
	openReadOnlyGeneration(context.Context, string, string) (generationReopenBackend, generationReopenEvidence, error)
}
type generationReopenBackend interface {
	generationBackend
	openReadOnlyMembers(context.Context, map[string]baselineMember, map[string]diskIdentity) (map[string]candidateFile, error)
}

// OpenReadOnlyGeneration reopens one completed fixed protected generation for a
// registered real user. The native authority boundary retains original completed
// registration/baseline/parent/generation records, verifies their exact hashes and
// full original physical IDs, and refuses partial or extra namespaces. It does
// not adopt a caller receipt or establish the currently selected predecessor.
// Existing signed-policy/commissioning/health checks remain separate obligations.
func (s *Store) OpenReadOnlyGeneration(ctx context.Context, clientSID, generation string) (result *ReopenedGeneration, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if ctx == nil || !brokerregistration.IsCanonicalUserSID(clientSID) || !validHex(generation, 32) {
		return nil, errors.New("live context and canonical fixed client/generation required")
	}
	ctx = registrationContext{Context: ctx, store: s}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := s.backend.check(); err != nil {
		return nil, err
	}
	live := 0
	for _, g := range s.generations {
		g.mu.Lock()
		same := g.record.ClientSID == clientSID
		failed := errors.Join(g.failure, g.closeErr)
		writer := !g.closed && (!g.transferred || !g.readOnly)
		if !g.closed {
			live++
		}
		g.mu.Unlock()
		if same && failed != nil {
			return nil, errors.Join(ErrPoisoned, failed)
		}
		if same && writer {
			return nil, ErrGenerationHistoryRequiresReadOnly
		}
	}
	if live >= MaxCandidates {
		return nil, errors.New("bounded live generation owners exhausted")
	}
	backend, ok := s.backend.(generationReopenStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	b, e, err := backend.openReadOnlyGeneration(ctx, clientSID, generation)
	if err != nil {
		return nil, err
	}
	if b == nil {
		return nil, errors.New("native readonly generation absent")
	}
	g := &InstalledGeneration{store: s, backend: b, files: map[string]candidateFile{}, complete: true, transferred: true, readOnly: true}
	defer func() {
		if resultErr != nil {
			g.failure = resultErr
			resultErr = errors.Join(resultErr, g.closeLocked())
		}
	}()
	record, err := validateGenerationReopenEvidence(e, clientSID, generation)
	if err != nil {
		return nil, err
	}
	g.record = record
	g.raw = bytes.Clone(e.completion)
	g.files, err = b.openReadOnlyMembers(ctx, record.Members, record.Directories)
	if err != nil {
		return nil, err
	}
	for name := range record.Members {
		f, ok := g.files[name].(*generationReadOnlyFile)
		if !ok || f == nil || f.reader == nil {
			return nil, errors.New("independently opened readonly member required")
		}
	}
	if err = g.validateContents(ctx); err != nil {
		return nil, err
	}
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	s.generations = append(s.generations, g)
	return &ReopenedGeneration{owner: &ReadOnlyGeneration{generation: g, children: map[*GenerationReadHandle]bool{}}, purpose: GenerationPurpose(record.Purpose)}, nil
}

// Only the native backend supplies this evidence from retained fixed originals.
// Canonical records reject unknown/duplicate/case-aliased fields by exact bytes;
// this is not an unsigned general-purpose generation import API.
func validateGenerationReopenEvidence(e generationReopenEvidence, client, stage string) (generationRecord, error) {
	var record generationRecord
	if len(e.completion) < 1 || len(e.completion) > maxGenerationRecordBytes || json.Unmarshal(e.completion, &record) != nil {
		return record, errors.New("bounded original generation completion required")
	}
	raw, err := json.Marshal(record)
	if err != nil || !bytes.Equal(raw, e.completion) || record.Schema != 1 || record.ClientSID != client || record.Generation != stage {
		return record, errors.New("exact original generation completion required")
	}
	if err = validateGenerationPurpose(record); err != nil {
		return record, err
	}
	o := e.lineage
	if !o.RegistrationIdentity.valid() || !o.RegistrationMarkerIdentity.valid() || !o.BaselineMarkerIdentity.valid() || !o.BaselineCompletionIdentity.valid() {
		return record, errors.New("original protected lineage full IDs required")
	}
	request, err := validateBaselineRegistration(o.Registration, o.RegistrationMarker, client, o.ApprovedRequestSHA256)
	if err != nil {
		return record, err
	}
	var baseline baselineRecord
	if len(e.baselineCompletion) < 1 || len(e.baselineCompletion) > maxBaselineRecordBytes || json.Unmarshal(e.baselineCompletion, &baseline) != nil {
		return record, errors.New("bounded original baseline required")
	}
	canonical, err := json.Marshal(baseline)
	if err != nil || !bytes.Equal(canonical, e.baselineCompletion) || baseline.Schema != 1 || baseline.ClientSID != client || baseline.Generation != request.Installed.Generation || baseline.ApprovedRequestSHA256 != o.ApprovedRequestSHA256 || !baselineMatches(baseline.Identity, request.Installed) || len(baseline.Members) < 1 || len(baseline.Members) > MaxFiles {
		return record, errors.New("completed baseline differs from original registration")
	}
	digest, err := completedBaselineDigest(o.BaselineMarker, client, baseline.Generation, o.ApprovedRequestSHA256)
	if err != nil || digest != registrationDigest(e.baselineCompletion) || record.PreviousCompletionSHA256 != digest || !bytes.Equal(record.PreviousCompletion, e.baselineCompletion) {
		return record, errors.Join(errors.New("generation original registered baseline lineage differs"), err)
	}
	if record.Purpose == approvedBaselineGenerationPurpose {
		a, _ := json.Marshal(record.ApprovedBaseline)
		b, _ := json.Marshal(o)
		if !bytes.Equal(a, b) {
			return record, errors.New("approved baseline original physical provenance differs")
		}
	} else if record.Acquisition.Version != record.Identity.Version || record.Acquisition.Commit != record.Identity.Commit || stage == baseline.Generation {
		return record, errors.New("signed candidate identity or distinct generation differs")
	}
	if len(record.Members) < 1 || len(record.Members) > MaxFiles || len(record.SourceMembers) != len(record.Members) || len(record.Directories) < 1 || len(record.Directories) > MaxFiles+16 || len(record.SourceDirectories) < 1 || len(record.SourceDirectories) > MaxFiles+16 {
		return record, errors.New("bounded complete generation namespace required")
	}
	names := map[string]string{}
	expectedDirs := map[string]bool{"": true}
	var total int64
	for name, m := range record.Members {
		if !baselineEvidenceName(name) {
			if err = memberPath(name); err != nil {
				return record, err
			}
		}
		if name == "app/OPENNAV_PORTABLE_PREVIEW" || m.Length < 0 || m.Length > MaxFileBytes || m.Length > MaxPayloadBytes-total || !validHex(m.SHA256, 64) || !m.Identity.valid() || !record.SourceMembers[name].valid() {
			return record, errors.New("bounded exact member originals required")
		}
		total += m.Length
		lower := strings.ToLower(name)
		if _, exists := names[lower]; exists {
			return record, errors.New("generation member case collision")
		}
		names[lower] = name
		parent, _ := splitReopenName(name)
		for parent != "" {
			expectedDirs[parent] = true
			parent, _ = splitReopenName(parent)
		}
	}
	directoryNames := map[string]bool{}
	for name, id := range record.Directories {
		if !id.valid() {
			return record, errors.New("original directory full ID required")
		}
		if strings.HasPrefix(name, "@") {
			continue
		}
		lower := strings.ToLower(name)
		if directoryNames[lower] {
			return record, errors.New("generation directory case collision")
		}
		directoryNames[lower] = true
		if !expectedDirs[name] {
			return record, errors.New("unexpected recorded generation directory")
		}
		if _, exists := names[strings.ToLower(name)]; exists {
			return record, errors.New("generation member/directory collision")
		}
		delete(expectedDirs, name)
	}
	if len(expectedDirs) != 0 {
		return record, errors.New("missing original generation directory")
	}
	for _, id := range record.SourceDirectories {
		if !id.valid() {
			return record, errors.New("original source directory full ID required")
		}
	}
	return record, nil
}

func splitReopenName(name string) (string, string) {
	if i := strings.LastIndexByte(name, '/'); i >= 0 {
		return name[:i], name[i+1:]
	}
	return "", name
}
