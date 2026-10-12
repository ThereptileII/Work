package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"runtime"
	"sort"
	"sync"

	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/brokerregistration"
)

const maxGenerationRecordBytes = 16 << 20

var ErrInstalledGenerationUncertain = errors.New("installed generation publication uncertain")

// GenerationCompletion describes retained, nonselected bytes only. It grants no
// launch, installation acceptance, transaction or healthy-start authority.
type GenerationCompletion struct {
	Schema                   int                       `json:"schema"`
	ClientSID                string                    `json:"clientSid"`
	Generation               string                    `json:"generation"`
	PreviousGeneration       string                    `json:"previousGeneration"`
	PreviousCompletionSHA256 string                    `json:"previousCompletionSha256"`
	CompletionSHA256         string                    `json:"completionSha256"`
	PolicySHA256             string                    `json:"policySha256"`
	RecoverySHA256           string                    `json:"recoverySha256"`
	InstallerSHA256          string                    `json:"installerSha256"`
	Identity                 baselineevidence.Identity `json:"identity"`
}

type generationRecord struct {
	Schema                   int                       `json:"schema"`
	Purpose                  string                    `json:"purpose"`
	ApprovedBaseline         *generationBaselineOrigin `json:"approvedBaseline,omitempty"`
	ClientSID                string                    `json:"clientSid"`
	Generation               string                    `json:"generation"`
	PreviousCompletion       []byte                    `json:"previousCompletion"`
	PreviousCompletionSHA256 string                    `json:"previousCompletionSha256"`
	Acquisition              AcquisitionPlan           `json:"acquisition"`
	Identity                 baselineevidence.Identity `json:"identity"`
	SourceDirectories        map[string]diskIdentity   `json:"sourceDirectories"`
	SourceMembers            map[string]diskIdentity   `json:"sourceMembers"`
	Directories              map[string]diskIdentity   `json:"directories"`
	Members                  map[string]baselineMember `json:"members"`
}

type generationStoreBackend interface {
	prepareInstalledGeneration(installedBaselineBackend, candidateBackend, *Acquisition, string, string) (generationBackend, error)
}
type generationBackend interface {
	candidateBackend
	reserve(context.Context, []byte) error
	createTree() error
	inventory([]string, bool) error
	sourceIdentities(map[string]fileRecord) (map[string]diskIdentity, map[string]diskIdentity, error)
	identities(map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, error)
	publish(context.Context, []byte) error
}

// InstalledGeneration owns a fresh ProgramFilesX86 launch view and its private
// permanent completion evidence. No public reopen, adoption, selection or launch
// API exists. Present ACLs do not prove historical creation. Existing client
// parents require their original private completed creation record and full
// physical identities. Previous lineage refers to the approved initial baseline,
// not an assertion that this is the immediately selected or healthy generation.
type InstalledGeneration struct {
	mu                    sync.Mutex
	store                 *Store
	backend               generationBackend
	record                generationRecord
	files                 map[string]candidateFile
	raw                   []byte
	complete              bool
	transferred, readOnly bool
	closed                bool
	failure               error
	closeErr              error
}

// MaterializeCandidateFresh copies only a live frozen recovery payload, exact
// original sidecars and all three engine helpers. The actual broker host must
// obtain this candidate through signed preparation and StageRecovery; a plan is
// a consistency record, not independent authentication. Acquisition must have
// successfully sealed both exact artifacts and completed cleanup before copying.
// The prior baseline and candidate remain held throughout this call. Generation
// and client identity are derived internally; no path or approval hash is accepted.
// All partials remain permanently reserved. Actual publication uncertainty may
// coexist with durable completed bytes and never permits retry or deletion.
func (s *Store) MaterializeCandidateFresh(ctx context.Context, previous *InstalledBaseline, candidate *Candidate) (result *InstalledGeneration, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if ctx == nil || previous == nil || candidate == nil || previous.store != s || candidate.store != s {
		return nil, errors.New("live same-store approved baseline and candidate required")
	}
	ctx = registrationContext{Context: ctx, store: s}
	previous.mu.Lock()
	defer previous.mu.Unlock()
	candidate.mu.Lock()
	defer candidate.mu.Unlock()
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := previous.ready(); err != nil {
		return nil, err
	}
	if !previous.completed || !brokerregistration.IsCanonicalUserSID(candidate.clientSID) || !validHex(candidate.stageID, 32) || candidate.clientSID != previous.record.ClientSID || candidate.stageID == previous.record.Generation {
		return nil, errors.New("completed matching baseline and distinct derived generation required")
	}
	if err := candidate.ready(); err != nil {
		return nil, err
	}
	a := candidate.acquisition
	if candidate.sealed || candidate.materialized || candidate.recoveryEvidence == nil || a == nil || a.candidate != candidate || a.backend == nil || a.plan == nil || !a.plan.valid() || !a.closed || a.closeErr != nil || len(a.artifacts) != 2 || a.plan.Version != candidate.recoveryEvidence.Version || a.plan.Commit != candidate.recoveryEvidence.Commit {
		return nil, errors.New("unsealed frozen candidate and completed native acquisition required")
	}
	slots := map[ArtifactSlot]bool{}
	for _, artifact := range a.artifacts {
		if artifact == nil || artifact.acquisition != a || artifact.backend == nil || !artifact.sealed || !artifact.closed || artifact.closeErr != nil || artifact.written != artifact.spec.Size || slots[artifact.slot] {
			return nil, errors.New("two exactly sealed and cleaned acquisition artifacts required")
		}
		spec, err := a.plan.spec(artifact.slot)
		if err != nil || artifact.spec != spec {
			return nil, errors.New("acquisition artifacts differ from bound plan")
		}
		slots[artifact.slot] = true
	}
	identity, err := previous.validateContents(ctx)
	if err != nil || identity != previous.record.Identity {
		return nil, errors.Join(errors.New("retained baseline identity differs"), err)
	}
	if err := candidate.validateRecoveryEvidence(ctx, *candidate.recoveryEvidence); err != nil {
		return nil, err
	}
	inventoryRaw, err := candidate.readRecoveryMember(ctx, "FILE_SHA256.json", maxCandidateEvidenceBytes)
	if err != nil {
		return nil, err
	}
	inventory := map[string]string{}
	if err := baselineevidence.DecodeInventory(inventoryRaw, &inventory); err != nil {
		return nil, err
	}
	helpers, err := baselineevidence.EngineHelpers(inventory)
	if err != nil || len(helpers) != 3 {
		return nil, errors.Join(errors.New("all three original engine helpers required for launch view"), err)
	}
	previousRaw, err := json.Marshal(previous.record)
	if err != nil || len(previousRaw) > maxBaselineRecordBytes {
		return nil, errors.New("bounded original baseline completion required")
	}
	backend, ok := s.backend.(generationStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	b, err := backend.prepareInstalledGeneration(previous.backend, candidate.backend, a, candidate.clientSID, candidate.stageID)
	if err != nil {
		return nil, err
	}
	if b == nil {
		return nil, errors.New("native generation backend absent")
	}
	g := &InstalledGeneration{store: s, backend: b, files: map[string]candidateFile{}, record: generationRecord{Schema: 1, Purpose: "signed-candidate-generation", ClientSID: candidate.clientSID, Generation: candidate.stageID, PreviousCompletion: bytes.Clone(previousRaw), PreviousCompletionSHA256: registrationDigest(previousRaw), Acquisition: *a.plan, Identity: *candidate.recoveryEvidence, Members: map[string]baselineMember{}}}
	defer func() {
		if resultErr != nil {
			g.failure = errors.Join(g.failure, resultErr)
			resultErr = errors.Join(resultErr, g.Close())
		}
	}()
	// Consuming the source before reservation prevents retry after any attempted
	// destination creation. The native permanent reservation is authoritative.
	candidate.materialized = true
	g.record.SourceDirectories, g.record.SourceMembers, err = b.sourceIdentities(candidate.recoveryMembers)
	if err == nil && (len(g.record.SourceDirectories) == 0 || len(g.record.SourceMembers) != len(candidate.recoveryMembers) || !sameGenerationIDs(g.record.SourceDirectories, g.record.SourceDirectories) || !sameGenerationIDs(g.record.SourceMembers, g.record.SourceMembers)) {
		err = errors.New("complete native source identities required")
	}
	if err != nil {
		return nil, err
	}
	origin, err := json.Marshal(g.record)
	if err != nil || len(origin) > maxGenerationRecordBytes {
		return nil, errors.New("bounded generation origin required")
	}
	if err = b.reserve(ctx, origin); err != nil {
		return nil, err
	}
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	if err = b.createTree(); err != nil {
		return nil, err
	}
	var names []string
	for name := range candidate.recoveryMembers {
		names = append(names, name)
	}
	sort.Strings(names)
	var total int64
	for _, name := range names {
		source := candidate.recoveryMembers[name]
		if len(g.files) >= MaxFiles || source.length < 0 || source.length > MaxFileBytes || source.length > MaxPayloadBytes-total {
			return nil, errors.New("bounded complete generation payload required")
		}
		if err = verifyFile(ctx, s, source.file, source.length, source.digest); err != nil {
			return nil, err
		}
		if _, err = source.file.Seek(0, io.SeekStart); err != nil {
			return nil, err
		}
		destination, err := b.create(name)
		if err != nil {
			return nil, err
		}
		g.files[name] = destination
		if err = candidate.copyFile(ctx, destination, source.file, source.length, source.digest); err != nil {
			return nil, err
		}
		g.record.Members[name] = baselineMember{Length: source.length, SHA256: source.digest}
		total += source.length
	}
	dirs, members, err := b.identities(g.record.Members)
	if err != nil {
		return nil, err
	}
	if len(dirs) == 0 || !sameGenerationIDs(dirs, dirs) {
		return nil, errors.New("complete native destination directory identities required")
	}
	g.record.Directories = dirs
	for name, id := range members {
		m := g.record.Members[name]
		m.Identity = id
		g.record.Members[name] = m
	}
	if len(members) != len(g.record.Members) {
		return nil, errors.New("generation member identity set differs")
	}
	if err := g.validateContents(ctx); err != nil {
		return nil, err
	}
	if err := candidate.validateRecoveryEvidence(ctx, *candidate.recoveryEvidence); err != nil {
		return nil, err
	}
	if actual, err := previous.validateContents(ctx); err != nil || actual != previous.record.Identity {
		return nil, errors.Join(errors.New("baseline changed before generation completion"), err)
	}
	sourceDirs, sourceFiles, err := b.sourceIdentities(candidate.recoveryMembers)
	if err != nil || !sameGenerationIDs(sourceDirs, g.record.SourceDirectories) || !sameGenerationIDs(sourceFiles, g.record.SourceMembers) {
		return nil, errors.Join(errors.New("candidate physical identity changed during generation copy"), err)
	}
	g.raw, err = json.Marshal(g.record)
	if err != nil || len(g.raw) > maxGenerationRecordBytes {
		return nil, errors.New("bounded exact generation completion required")
	}
	if err = b.publish(ctx, g.raw); err != nil {
		return nil, err
	}
	g.complete = true
	s.generations = append(s.generations, g)
	return g, nil
}

func sameGenerationIDs(a, b map[string]diskIdentity) bool {
	if len(a) != len(b) {
		return false
	}
	for name, id := range a {
		if !id.valid() || id != b[name] {
			return false
		}
	}
	return true
}

func (g *InstalledGeneration) names() []string {
	names := make([]string, 0, len(g.files))
	for name := range g.files {
		names = append(names, name)
	}
	sort.Strings(names)
	return names
}

func (g *InstalledGeneration) validateContents(ctx context.Context) error {
	if g.record.ApprovedBaseline != nil || g.record.Purpose == approvedBaselineGenerationPurpose {
		if err := validateApprovedBaselineGeneration(g.record); err != nil {
			return err
		}
	}
	if err := g.backend.check(); err != nil {
		return err
	}
	if len(g.files) != len(g.record.Members) {
		return errors.New("generation retained member set differs")
	}
	observed := map[string]string{}
	read := func(name string, limit int64) ([]byte, error) {
		m, ok := g.record.Members[name]
		if !ok || m.Length < 1 || m.Length > limit {
			return nil, errors.New("required original generation member absent")
		}
		f := g.files[name]
		if _, err := f.Seek(0, io.SeekStart); err != nil {
			return nil, err
		}
		raw, err := io.ReadAll(io.LimitReader(checkedReader{ctx: ctx, store: g.store, reader: f}, m.Length+1))
		if err != nil || int64(len(raw)) != m.Length || registrationDigest(raw) != m.SHA256 {
			return nil, errors.Join(errors.New("generation original bytes differ"), err)
		}
		return raw, f.check(m.Length)
	}
	for name, m := range g.record.Members {
		if err := verifyFile(ctx, g.store, g.files[name], m.Length, m.SHA256); err != nil {
			return err
		}
		if !baselineSidecarName(name) {
			observed[name] = m.SHA256
		}
	}
	inventory, err := read("FILE_SHA256.json", maxBaselineEvidenceBytes)
	if err != nil {
		return err
	}
	manifest, err := read("installer/package.json", maxBaselineEvidenceBytes)
	if err != nil {
		return err
	}
	product, err := read("docs/PRODUCT_BUILD.json", 32<<10)
	if err != nil {
		return err
	}
	i := g.record.Identity
	expected := brokerregistration.Installed{Version: i.Version, Commit: i.Commit, PackageSHA256: i.LifecycleManifestSHA256, ProductBuildSHA256: i.ProductBuildSHA256, ExecutableSHA256: i.ExecutableSHA256, InventorySHA256: i.InventorySHA256}
	actual, _, err := baselineevidence.Validate(ctx, inventory, manifest, product, observed, expected)
	if err != nil || actual != i {
		return errors.Join(errors.New("generation exact original recovery identity differs"), err)
	}
	if err := g.backend.inventory(g.names(), g.complete); err != nil {
		return err
	}
	dirs, files, err := g.backend.identities(g.record.Members)
	if err != nil || !sameGenerationIDs(dirs, g.record.Directories) {
		return errors.Join(errors.New("generation directory identities changed"), err)
	}
	if len(files) != len(g.record.Members) {
		return errors.New("generation member identity set differs")
	}
	for name, m := range g.record.Members {
		if !m.Identity.valid() || files[name] != m.Identity {
			return errors.New("generation member physical identity changed")
		}
	}
	return ctx.Err()
}

func (g *InstalledGeneration) ready() error {
	if g.closed || g.store.closed.Load() {
		return ErrClosed
	}
	if g.failure != nil {
		return errors.Join(ErrPoisoned, g.failure)
	}
	if !g.complete {
		return errors.New("completed generation required")
	}
	return nil
}

func (g *InstalledGeneration) Validate() (err error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := g.ready(); err != nil {
		return err
	}
	if g.transferred {
		return ErrGenerationTransferred
	}
	err = g.validateContents(registrationContext{Context: context.Background(), store: g.store})
	if err != nil {
		g.failure = errors.Join(g.failure, err)
	}
	return err
}

func (g *InstalledGeneration) Completion() (GenerationCompletion, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	g.mu.Lock()
	defer g.mu.Unlock()
	if err := g.ready(); err != nil {
		return GenerationCompletion{}, err
	}
	if g.transferred {
		return GenerationCompletion{}, ErrGenerationTransferred
	}
	if err := g.validateContents(registrationContext{Context: context.Background(), store: g.store}); err != nil {
		g.failure = errors.Join(g.failure, err)
		return GenerationCompletion{}, err
	}
	return g.completionRecord()
}
func (g *InstalledGeneration) completionRecord() (GenerationCompletion, error) {
	if g.record.Purpose != "signed-candidate-generation" || g.record.ApprovedBaseline != nil {
		return GenerationCompletion{}, errors.New("signed candidate completion required")
	}
	var previous baselineRecord
	if err := json.Unmarshal(g.record.PreviousCompletion, &previous); err != nil {
		return GenerationCompletion{}, err
	}
	return GenerationCompletion{Schema: 1, ClientSID: g.record.ClientSID, Generation: g.record.Generation, PreviousGeneration: previous.Generation, PreviousCompletionSHA256: g.record.PreviousCompletionSHA256, CompletionSHA256: registrationDigest(g.raw), PolicySHA256: g.record.Acquisition.PolicySHA256, RecoverySHA256: g.record.Acquisition.Recovery.SHA256, InstallerSHA256: g.record.Acquisition.Installer.SHA256, Identity: g.record.Identity}, nil
}

// Close retains the first failure and cleanup result and drains exactly once.
// No partial or completed namespace is deleted or made reusable.
func (g *InstalledGeneration) Close() error {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.closeLocked()
}
func (g *InstalledGeneration) closeLocked() error {
	if g.closed {
		return g.closeErr
	}
	g.closed = true
	g.closeErr = g.failure
	for _, name := range g.names() {
		g.closeErr = errors.Join(g.closeErr, g.files[name].Close())
	}
	g.files = nil
	g.closeErr = errors.Join(g.closeErr, g.backend.close())
	if g.complete && g.closeErr != nil {
		g.closeErr = errors.Join(ErrInstalledGenerationUncertain, g.closeErr)
	}
	return g.closeErr
}
