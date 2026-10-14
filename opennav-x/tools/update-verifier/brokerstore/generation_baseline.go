package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"runtime"
	"sort"

	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/brokerregistration"
)

const approvedBaselineGenerationPurpose = "operator-approved-baseline-generation"

type generationBaselineOrigin struct {
	ApprovedRequestSHA256      string       `json:"approvedRequestSha256"`
	Registration               []byte       `json:"registration"`
	RegistrationMarker         []byte       `json:"registrationMarker"`
	RegistrationIdentity       diskIdentity `json:"registrationIdentity"`
	RegistrationMarkerIdentity diskIdentity `json:"registrationMarkerIdentity"`
	BaselineMarker             []byte       `json:"baselineMarker"`
	BaselineMarkerIdentity     diskIdentity `json:"baselineMarkerIdentity"`
	BaselineCompletionIdentity diskIdentity `json:"baselineCompletionIdentity"`
}
type baselineGenerationStoreBackend interface {
	prepareBaselineGeneration(context.Context, installedBaselineBackend, brokerregistration.Request) (baselineGenerationBackend, error)
}
type baselineGenerationBackend interface {
	generationBackend
	baselineSource(map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, generationBaselineOrigin, error)
}

// ApprovedBaselineCompletion describes only the protected copied originals of
// an independently approved initial baseline. Its hashes are not signed update
// policy, Recovery or Setup identities. It grants no selection, commissioning,
// execution or healthy-start authority; the three retained engine helpers do not
// by themselves establish a complete commissioning closure.
type ApprovedBaselineCompletion struct {
	Schema                   int                       `json:"schema"`
	Purpose                  string                    `json:"purpose"`
	ClientSID                string                    `json:"clientSid"`
	Generation               string                    `json:"generation"`
	ApprovedRequestSHA256    string                    `json:"approvedRequestSha256"`
	RegistrationSHA256       string                    `json:"registrationSha256"`
	BaselineCompletionSHA256 string                    `json:"baselineCompletionSha256"`
	CompletionSHA256         string                    `json:"completionSha256"`
	Identity                 baselineevidence.Identity `json:"identity"`
}

// ApprovedBaselineGeneration retains a fresh public protected copy of the
// initial operator-approved baseline. It cannot be created by adopting a path or
// receipt, or interpreted as a signed candidate. Store.Close revokes custody.
type ApprovedBaselineGeneration struct{ generation *InstalledGeneration }

// ReadOnlyApprovedBaselineGeneration retains loader-readable original baseline
// bytes under the same one-way readonly lifetime as candidate generations. This
// capability remains nonselected and grants no permission to launch a process.
type ReadOnlyApprovedBaselineGeneration struct{ owner *ReadOnlyGeneration }

// MaterializeApprovedBaselineFresh derives the fixed public generation from a
// live same-Store completed baseline and original independent approval. The native
// factory must additionally retain completed fixed protected registration evidence
// binding these exact originals; rawRequest/approvedSHA alone are never authority.
// All three original engine helpers are required. Existing/partial destinations
// refuse permanently; this operation never repairs, adopts or deletes a view.
func (s *Store) MaterializeApprovedBaselineFresh(ctx context.Context, previous *InstalledBaseline, rawRequest []byte, approvedSHA string) (result *ApprovedBaselineGeneration, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if previous != nil && previous.store == s && s.hasLiveGenerationLocked(previous.record.ClientSID) {
		return nil, ErrGenerationHistoryRequiresReadOnly
	}
	return s.materializeApprovedBaselineFreshLocked(ctx, previous, rawRequest, approvedSHA, nil)
}

// Caller holds Store.mu; onMutation runs immediately before permanent reservation.
func (s *Store) materializeApprovedBaselineFreshLocked(ctx context.Context, previous *InstalledBaseline, rawRequest []byte, approvedSHA string, onMutation func()) (result *ApprovedBaselineGeneration, resultErr error) {
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if ctx == nil || previous == nil || previous.store != s {
		return nil, errors.New("live same-store completed approved baseline required")
	}
	ctx = registrationContext{Context: ctx, store: s}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	previous.mu.Lock()
	defer previous.mu.Unlock()
	if err := previous.ready(); err != nil {
		return nil, err
	}
	request, err := brokerregistration.ParseRetainedApprovedRequest(rawRequest, approvedSHA)
	if err != nil || !previous.completed || request.ClientSID != previous.record.ClientSID || request.Installed.Generation != previous.record.Generation || approvedSHA != previous.record.ApprovedRequestSHA256 || request.Installed != previous.expected || !baselineMatches(previous.record.Identity, request.Installed) {
		return nil, errors.Join(errors.New("exact completed baseline and original approved registration binding required"), err)
	}
	identity, err := previous.validateContents(ctx)
	if err != nil || identity != previous.record.Identity {
		return nil, errors.Join(errors.New("approved baseline originals changed"), err)
	}
	inventoryRaw, err := baselineGenerationRead(ctx, previous, "FILE_SHA256.json", maxBaselineEvidenceBytes)
	if err != nil {
		return nil, err
	}
	inventory := map[string]string{}
	if err = baselineevidence.DecodeInventory(inventoryRaw, &inventory); err != nil {
		return nil, err
	}
	helpers, err := baselineevidence.EngineHelpers(inventory)
	if err != nil || len(helpers) != 3 {
		return nil, errors.Join(errors.New("three original engine helpers required"), err)
	}
	previousRaw, err := json.Marshal(previous.record)
	if err != nil || len(previousRaw) > maxBaselineRecordBytes {
		return nil, errors.New("bounded original baseline completion required")
	}
	backend, ok := s.backend.(baselineGenerationStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	b, err := backend.prepareBaselineGeneration(ctx, previous.backend, request)
	if err != nil {
		return nil, err
	}
	if b == nil {
		return nil, errors.New("native baseline generation owner absent")
	}
	g := &InstalledGeneration{store: s, backend: b, files: map[string]candidateFile{}, record: generationRecord{Schema: 1, Purpose: approvedBaselineGenerationPurpose, ClientSID: request.ClientSID, Generation: request.Installed.Generation, PreviousCompletion: bytes.Clone(previousRaw), PreviousCompletionSHA256: registrationDigest(previousRaw), Identity: identity, Members: map[string]baselineMember{}}}
	defer func() {
		if resultErr != nil {
			g.failure = errors.Join(g.failure, resultErr)
			resultErr = errors.Join(resultErr, g.closeLocked())
		}
	}()
	sourceDirs, sourceFiles, origin, err := b.baselineSource(previous.record.Members)
	if err != nil {
		return nil, err
	}
	g.record.SourceDirectories = sourceDirs
	g.record.SourceMembers = sourceFiles
	g.record.ApprovedBaseline = &origin
	if err = validateApprovedBaselineGeneration(g.record); err != nil {
		return nil, err
	}
	originRaw, err := json.Marshal(g.record)
	if err != nil || len(originRaw) > maxGenerationRecordBytes {
		return nil, errors.New("bounded baseline view origin required")
	}
	if onMutation != nil {
		onMutation()
	}
	if err = b.reserve(ctx, originRaw); err != nil {
		return nil, err
	}
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	if err = b.createTree(); err != nil {
		return nil, err
	}
	var names []string
	for name := range previous.record.Members {
		names = append(names, name)
	}
	sort.Strings(names)
	// Reuse the bounded hash/sync/readback copier with the actual retained
	// baseline source check; no candidate acquisition or policy is implied.
	copier := &Candidate{store: s, backend: previous.backend}
	var total int64
	for _, name := range names {
		source := previous.record.Members[name]
		if len(g.files) >= MaxFiles || source.Length < 0 || source.Length > MaxFileBytes || source.Length > MaxPayloadBytes-total {
			return nil, errors.New("bounded complete baseline launch view required")
		}
		original := previous.files[name]
		if err = verifyFile(ctx, s, original, source.Length, source.SHA256); err != nil {
			return nil, err
		}
		if _, err = original.Seek(0, io.SeekStart); err != nil {
			return nil, err
		}
		destination, err := b.create(name)
		if err != nil {
			return nil, err
		}
		g.files[name] = destination
		if err = copier.copyFile(ctx, destination, original, source.Length, source.SHA256); err != nil {
			return nil, err
		}
		g.record.Members[name] = baselineMember{Length: source.Length, SHA256: source.SHA256}
		total += source.Length
	}
	dirs, files, err := b.identities(g.record.Members)
	if err != nil {
		return nil, err
	}
	if len(dirs) == 0 || !sameGenerationIDs(dirs, dirs) || len(files) != len(g.record.Members) {
		return nil, errors.New("complete public baseline physical identities required")
	}
	g.record.Directories = dirs
	for name, id := range files {
		m, ok := g.record.Members[name]
		if !ok || !id.valid() {
			return nil, errors.New("exact baseline destination identity set required")
		}
		m.Identity = id
		g.record.Members[name] = m
	}
	if err = g.validateContents(ctx); err != nil {
		return nil, err
	}
	if actual, err := previous.validateContents(ctx); err != nil || actual != identity {
		return nil, errors.Join(errors.New("approved source changed before completion"), err)
	}
	afterDirs, afterFiles, afterOrigin, err := b.baselineSource(previous.record.Members)
	beforeOriginRaw, _ := json.Marshal(origin)
	afterOriginRaw, _ := json.Marshal(afterOrigin)
	if err != nil || !sameGenerationIDs(afterDirs, sourceDirs) || !sameGenerationIDs(afterFiles, sourceFiles) || !bytes.Equal(beforeOriginRaw, afterOriginRaw) {
		return nil, errors.Join(errors.New("original protected baseline/registration custody changed"), err)
	}
	g.raw, err = json.Marshal(g.record)
	if err != nil || len(g.raw) > maxGenerationRecordBytes {
		return nil, errors.New("bounded exact baseline view completion required")
	}
	if err = b.publish(ctx, g.raw); err != nil {
		return nil, err
	}
	g.complete = true
	s.generations = append(s.generations, g)
	return &ApprovedBaselineGeneration{generation: g}, nil
}
func baselineGenerationRead(ctx context.Context, b *InstalledBaseline, name string, limit int64) ([]byte, error) {
	m, ok := b.record.Members[name]
	if !ok || m.Length < 1 || m.Length > limit {
		return nil, errors.New("bounded original baseline member required")
	}
	f := b.files[name]
	if _, err := f.Seek(0, io.SeekStart); err != nil {
		return nil, err
	}
	raw, err := io.ReadAll(io.LimitReader(checkedReader{ctx: ctx, store: b.store, reader: f}, m.Length+1))
	if err != nil || int64(len(raw)) != m.Length || registrationDigest(raw) != m.SHA256 {
		return nil, errors.Join(errors.New("original baseline member changed"), err)
	}
	return raw, f.check(m.Length)
}
func baselineViewCompletion(g *InstalledGeneration) (ApprovedBaselineCompletion, error) {
	if err := validateApprovedBaselineGeneration(g.record); err != nil {
		return ApprovedBaselineCompletion{}, err
	}
	o := g.record.ApprovedBaseline
	return ApprovedBaselineCompletion{1, approvedBaselineGenerationPurpose, g.record.ClientSID, g.record.Generation, o.ApprovedRequestSHA256, registrationDigest(o.Registration), g.record.PreviousCompletionSHA256, registrationDigest(g.raw), g.record.Identity}, nil
}
func (b *ApprovedBaselineGeneration) Validate() error {
	if b == nil || b.generation == nil {
		return ErrClosed
	}
	return b.generation.Validate()
}
func (b *ApprovedBaselineGeneration) Close() error {
	if b == nil || b.generation == nil {
		return nil
	}
	return b.generation.Close()
}
func (b *ApprovedBaselineGeneration) Completion() (ApprovedBaselineCompletion, error) {
	if b == nil || b.generation == nil {
		return ApprovedBaselineCompletion{}, ErrClosed
	}
	g := b.generation
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g.store.mu.Lock()
	defer g.store.mu.Unlock()
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := g.ready(); err != nil {
		return ApprovedBaselineCompletion{}, err
	}
	if g.transferred {
		return ApprovedBaselineCompletion{}, ErrGenerationTransferred
	}
	if err := g.validateContents(registrationContext{Context: context.Background(), store: g.store}); err != nil {
		g.failure = errors.Join(g.failure, err)
		return ApprovedBaselineCompletion{}, err
	}
	return baselineViewCompletion(g)
}
func (b *ApprovedBaselineGeneration) TransitionReadOnly(ctx context.Context) (*ReadOnlyApprovedBaselineGeneration, error) {
	if b == nil || b.generation == nil {
		return nil, ErrClosed
	}
	r, err := b.generation.TransitionReadOnly(ctx)
	if err != nil {
		return nil, err
	}
	return &ReadOnlyApprovedBaselineGeneration{owner: r}, nil
}
func (b *ReadOnlyApprovedBaselineGeneration) Validate() error {
	if b == nil || b.owner == nil {
		return ErrClosed
	}
	return b.owner.Validate()
}
func (b *ReadOnlyApprovedBaselineGeneration) Close() error {
	if b == nil || b.owner == nil {
		return nil
	}
	return b.owner.Close()
}
func (b *ReadOnlyApprovedBaselineGeneration) OpenMember(name string) (*GenerationReadHandle, error) {
	if b == nil || b.owner == nil {
		return nil, ErrClosed
	}
	return b.owner.OpenMember(name)
}
func (b *ReadOnlyApprovedBaselineGeneration) Completion() (ApprovedBaselineCompletion, error) {
	if b == nil || b.owner == nil || b.owner.generation == nil {
		return ApprovedBaselineCompletion{}, ErrClosed
	}
	g := b.owner.generation
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g.store.mu.Lock()
	defer g.store.mu.Unlock()
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := b.owner.ready(); err != nil {
		return ApprovedBaselineCompletion{}, err
	}
	if err := g.validateContents(registrationContext{Context: context.Background(), store: g.store}); err != nil {
		g.failure = errors.Join(g.failure, err)
		return ApprovedBaselineCompletion{}, errors.Join(ErrInstalledGenerationUncertain, err)
	}
	return baselineViewCompletion(g)
}

// These are exact producer bytes from protected registration custody, not a
// second generic registration parser accepting caller-owned declarations.
type baselineRegistrationEnvelope struct {
	Schema                int    `json:"schema"`
	ClientSID             string `json:"clientSid"`
	ApprovedRequestSHA256 string `json:"approvedRequestSha256"`
	ApprovedRequest       []byte `json:"approvedRequest"`
}

func validateBaselineRegistration(raw, marker []byte, client, approved string) (brokerregistration.Request, error) {
	var envelope baselineRegistrationEnvelope
	if len(raw) < 1 || len(raw) > MaxRegistrationBytes || json.Unmarshal(raw, &envelope) != nil {
		return brokerregistration.Request{}, errors.New("bounded protected registration required")
	}
	canonical, err := json.Marshal(envelope)
	if err != nil || !bytes.Equal(canonical, raw) || envelope.Schema != 2 || envelope.ClientSID != client || envelope.ApprovedRequestSHA256 != approved {
		return brokerregistration.Request{}, errors.New("exact completed approved registration required")
	}
	request, err := brokerregistration.ParseRetainedApprovedRequest(envelope.ApprovedRequest, approved)
	if err != nil || request.ClientSID != client {
		return brokerregistration.Request{}, errors.Join(errors.New("original protected approved request differs"), err)
	}
	if !bytes.Equal(marker, registrationMarker(client, request.Trust().Channel, approved, registrationDigest(raw), true)) {
		return brokerregistration.Request{}, errors.New("completed registration approval/JSON binding differs")
	}
	return request, nil
}
func validateApprovedBaselineGeneration(record generationRecord) error {
	o := record.ApprovedBaseline
	if record.Schema != 1 || record.Purpose != approvedBaselineGenerationPurpose || o == nil || record.Acquisition != (AcquisitionPlan{}) || !o.RegistrationIdentity.valid() || !o.RegistrationMarkerIdentity.valid() || !o.BaselineMarkerIdentity.valid() || !o.BaselineCompletionIdentity.valid() {
		return errors.New("distinct approved baseline provenance required")
	}
	request, err := validateBaselineRegistration(o.Registration, o.RegistrationMarker, record.ClientSID, o.ApprovedRequestSHA256)
	if err != nil {
		return err
	}
	var baseline baselineRecord
	if len(record.PreviousCompletion) < 1 || len(record.PreviousCompletion) > maxBaselineRecordBytes || registrationDigest(record.PreviousCompletion) != record.PreviousCompletionSHA256 || json.Unmarshal(record.PreviousCompletion, &baseline) != nil {
		return errors.New("original approved baseline completion required")
	}
	raw, err := json.Marshal(baseline)
	if err != nil || !bytes.Equal(raw, record.PreviousCompletion) || baseline.Schema != 1 || baseline.ClientSID != record.ClientSID || baseline.Generation != record.Generation || baseline.ApprovedRequestSHA256 != o.ApprovedRequestSHA256 || request.Installed.Generation != record.Generation || baseline.Identity != record.Identity || !baselineMatches(record.Identity, request.Installed) || len(baseline.Members) == 0 || len(baseline.Members) > MaxFiles {
		return errors.New("approved baseline lineage differs")
	}
	if digest, err := completedBaselineDigest(o.BaselineMarker, record.ClientSID, record.Generation, o.ApprovedRequestSHA256); err != nil || digest != record.PreviousCompletionSHA256 {
		return errors.Join(errors.New("original completed baseline marker differs"), err)
	}
	if !sameGenerationIDs(record.SourceDirectories, baseline.Directories) || len(record.SourceMembers) != len(baseline.Members) {
		return errors.New("original baseline physical namespace differs")
	}
	helperInventory := map[string]string{}
	for name, m := range baseline.Members {
		helperInventory[name] = m.SHA256
		if m.Length < 0 || m.Length > MaxFileBytes || !validHex(m.SHA256, 64) || !m.Identity.valid() || record.SourceMembers[name] != m.Identity {
			return errors.New("original baseline member identity differs")
		}
	}
	helpers, err := baselineevidence.EngineHelpers(helperInventory)
	if err != nil || len(helpers) != 3 {
		return errors.Join(errors.New("all three original baseline helpers required"), err)
	}
	if len(record.Members) > 0 {
		if len(record.Members) != len(baseline.Members) {
			return errors.New("copied baseline member set differs")
		}
		for name, m := range baseline.Members {
			copy, ok := record.Members[name]
			if !ok || copy.Length != m.Length || copy.SHA256 != m.SHA256 {
				return errors.New("copied original baseline member differs")
			}
		}
	}
	return nil
}
func validateGenerationPurpose(record generationRecord) error {
	switch record.Purpose {
	case string(GenerationPurposeRepaired):
		return validateRepairOrigin(record)
	case "signed-candidate-generation":
		if record.Repair != nil || record.ApprovedBaseline != nil || !record.Acquisition.valid() || registrationDigest(record.PreviousCompletion) != record.PreviousCompletionSHA256 {
			return errors.New("signed candidate provenance differs")
		}
		return nil
	case approvedBaselineGenerationPurpose:
		if record.Repair != nil {
			return ErrInstalledGenerationUncertain
		}
		if len(record.Members) == 0 {
			return errors.New("completed approved baseline members required")
		}
		return validateApprovedBaselineGeneration(record)
	default:
		return errors.New("unknown protected generation purpose")
	}
}
