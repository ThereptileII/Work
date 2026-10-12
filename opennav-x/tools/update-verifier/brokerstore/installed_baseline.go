package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"runtime"
	"sort"
	"strings"
	"sync"

	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/brokerregistration"
)

const baselineRecordFilename = "baseline-completion.json"
const maxBaselineRecordBytes = 8 << 20
const maxBaselineEvidenceBytes = 4 << 20
const baselineMarkerHeader = "SKAGER-installed-baseline-v1"

// ErrInstalledBaselineUncertain means completion publication was attempted but
// an I/O or custody check failed. Completed evidence may already be durable.
// Preserve all objects; this never authorizes retry, adoption, reset or launch.
var ErrInstalledBaselineUncertain = errors.New("installed baseline publication uncertain")

type installedBaselineStoreBackend interface {
	prepareInstalledBaseline(string, string, string) (installedBaselineBackend, error)
	openInstalledBaseline(context.Context, string, string, string) (installedBaselineBackend, []byte, error)
}
type installedBaselineBackend interface {
	candidateBackend
	reserve(context.Context) error
	createTree() error
	inventory([]string, bool) error
	identities(map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, error)
	openMembers(map[string]baselineMember, map[string]diskIdentity) (map[string]candidateFile, error)
	publish(context.Context, []byte) error
	directory() (string, error)
}
type baselineMember struct {
	Length   int64        `json:"length"`
	SHA256   string       `json:"sha256"`
	Identity diskIdentity `json:"identity"`
}
type baselineRecord struct {
	Schema                int                       `json:"schema"`
	ClientSID             string                    `json:"clientSid"`
	Generation            string                    `json:"generation"`
	ApprovedRequestSHA256 string                    `json:"approvedRequestSha256"`
	Identity              baselineevidence.Identity `json:"identity"`
	Directories           map[string]diskIdentity   `json:"directories"`
	Members               map[string]baselineMember `json:"members"`
}

// InstalledBaseline retains one fresh protected installed generation or one
// completed read owner. The original sidecars, exact payload set, full physical
// identities and independent request pin remain bound throughout custody. This
// is inert baseline evidence, never proof of a running installation or launch
// authority. Failure retains partial namespaces permanently for operator review.
type InstalledBaseline struct {
	mu                          sync.Mutex
	store                       *Store
	backend                     installedBaselineBackend
	record                      baselineRecord
	expected                    brokerregistration.Installed
	files                       map[string]candidateFile
	names                       map[string]string
	fileNames                   map[string]bool
	bytes                       int64
	readonly, completed, closed bool
	failure, closeErr           error
}

// BeginInstalledBaselineFresh consumes a permanent broker-private reservation
// before creating users/<canonical SID>/installed/<generation>. The client user
// tree must already exist from fresh Registration creation. Existing installed
// parents or generations are refused, never adopted or repaired. Only the actual
// non-impersonated native broker and fixed KnownFolder store support this API.
// approvedRequestSHA256 must come from independent operator approval, never an
// adjacent declaration or client-controlled request.
func (s *Store) BeginInstalledBaselineFresh(ctx context.Context, sid, generation, approvedRequestSHA256 string) (*InstalledBaseline, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if err := baselineArguments(ctx, sid, generation, approvedRequestSHA256); err != nil {
		return nil, err
	}
	ctx = registrationContext{Context: ctx, store: s}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := s.backend.check(); err != nil {
		return nil, err
	}
	backend, ok := s.backend.(installedBaselineStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	b, err := backend.prepareInstalledBaseline(sid, generation, approvedRequestSHA256)
	if err != nil {
		return nil, err
	}
	if err = beginInstalledBaseline(ctx, b); err != nil {
		return nil, err
	}
	r := newInstalledBaseline(s, b, sid, generation, approvedRequestSHA256)
	s.baselines = append(s.baselines, r)
	return r, nil
}
func baselineArguments(ctx context.Context, sid, generation, approved string) error {
	if ctx == nil || !brokerregistration.IsCanonicalUserSID(sid) || !validHex(generation, 32) || !validHex(approved, 64) {
		return errors.New("canonical client SID, generation and independent approval required")
	}
	return ctx.Err()
}
func newInstalledBaseline(s *Store, b installedBaselineBackend, sid, generation, approved string) *InstalledBaseline {
	return &InstalledBaseline{store: s, backend: b, files: map[string]candidateFile{}, names: map[string]string{}, fileNames: map[string]bool{}, record: baselineRecord{Schema: 1, ClientSID: sid, Generation: generation, ApprovedRequestSHA256: approved, Members: map[string]baselineMember{}}}
}
func beginInstalledBaseline(ctx context.Context, b installedBaselineBackend) (resultErr error) {
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, b.close())
		}
	}()
	for _, step := range []func() error{b.check, func() error { return b.reserve(ctx) }, b.check, b.createTree, b.check} {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := step(); err != nil {
			return err
		}
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	ok = true
	return nil
}

// OpenInstalledBaseline opens only completed fixed protected baseline evidence.
// It retains read-only deny-write/delete handles for every directory and member,
// checks the permanent creation record/full identities and revalidates original
// manifest/inventory/product semantics against the independently approved pins.
// Present ACLs alone are not accepted as proof of protected creation history.
func (s *Store) OpenInstalledBaseline(ctx context.Context, sid, generation, approvedRequestSHA256 string, expected brokerregistration.Installed) (result *InstalledBaseline, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if err := baselineArguments(ctx, sid, generation, approvedRequestSHA256); err != nil {
		return nil, err
	}
	if err := baselineExpected(expected, generation); err != nil {
		return nil, err
	}
	ctx = registrationContext{Context: ctx, store: s}
	if err := s.backend.check(); err != nil {
		return nil, err
	}
	backend, ok := s.backend.(installedBaselineStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	b, raw, err := backend.openInstalledBaseline(ctx, sid, generation, approvedRequestSHA256)
	if err != nil {
		return nil, err
	}
	r := newInstalledBaseline(s, b, sid, generation, approvedRequestSHA256)
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, r.Close())
		}
	}()
	if len(raw) == 0 || len(raw) > maxBaselineRecordBytes {
		return nil, errors.New("bounded completed baseline record required")
	}
	var record baselineRecord
	if err = json.Unmarshal(raw, &record); err != nil {
		return nil, err
	}
	canonical, err := json.Marshal(record)
	if err != nil || !bytes.Equal(raw, canonical) || record.Schema != 1 || record.ClientSID != sid || record.Generation != generation || record.ApprovedRequestSHA256 != approvedRequestSHA256 || len(record.Members) == 0 || len(record.Members) > MaxFiles || len(record.Directories) == 0 || len(record.Directories) > MaxFiles+2 {
		return nil, errors.New("exact completed baseline record required")
	}
	r.record = record
	r.expected = expected
	r.readonly = true
	r.completed = true
	for name, m := range record.Members {
		if err = r.reserve(name, baselineEvidenceName(name)); err != nil {
			return nil, err
		}
		if m.Length < 0 || m.Length > MaxFileBytes || !validHex(m.SHA256, 64) || !m.Identity.valid() || m.Length > MaxPayloadBytes-r.bytes {
			return nil, errors.New("completed baseline member bounds differ")
		}
		r.bytes += m.Length
	}
	r.files, err = b.openMembers(record.Members, record.Directories)
	if err != nil {
		return nil, err
	}
	identity, err := r.validateContents(ctx)
	if err != nil {
		return nil, err
	}
	if identity != record.Identity || !baselineMatches(identity, expected) {
		return nil, errors.New("completed baseline differs from approved identities")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	s.baselines = append(s.baselines, r)
	return r, nil
}
func baselineExpected(e brokerregistration.Installed, generation string) error {
	if e.Generation != generation || e.Version == "" || !validHex(e.Commit, 40) || !validHex(e.PackageSHA256, 64) || !validHex(e.ProductBuildSHA256, 64) || !validHex(e.InventorySHA256, 64) || !validHex(e.ExecutableSHA256, 64) {
		return errors.New("exact independently approved installed identities required")
	}
	return nil
}
func baselineMatches(i baselineevidence.Identity, e brokerregistration.Installed) bool {
	return i.Version == e.Version && i.Commit == e.Commit && i.ExecutableSHA256 == e.ExecutableSHA256 && i.ProductBuildSHA256 == e.ProductBuildSHA256 && i.InventorySHA256 == e.InventorySHA256 && i.LifecycleManifestAvailable && i.LifecycleManifestSHA256 == e.PackageSHA256
}
func baselineEvidenceName(name string) bool {
	return baselineSidecarName(name) || baselineevidence.IsEngineHelper(name) || baselineevidence.IsCommissioningHelper(name)
}
func baselineSidecarName(name string) bool {
	return name == "FILE_SHA256.json" || name == "installer/package.json"
}
func (r *InstalledBaseline) ready() error {
	if r.closed || r.store.closed.Load() {
		return ErrClosed
	}
	if r.failure != nil {
		return errors.Join(ErrPoisoned, r.failure)
	}
	if err := r.backend.check(); err != nil {
		r.failure = errors.Join(r.failure, err)
		return err
	}
	return nil
}
func (r *InstalledBaseline) reserve(name string, evidence bool) error {
	if evidence {
		if !baselineEvidenceName(name) {
			return errors.New("exact original baseline sidecar required")
		}
	} else {
		if err := memberPath(name); err != nil {
			return err
		}
		if name == "app/OPENNAV_PORTABLE_PREVIEW" {
			return errors.New("portable marker is not installed payload")
		}
	}
	parts := strings.Split(name, "/")
	for i := range parts {
		current := strings.Join(parts[:i+1], "/")
		key := strings.ToLower(current)
		if old, ok := r.names[key]; ok && (old != current || r.fileNames[key] || i == len(parts)-1) {
			return errors.New("baseline duplicate, alias or file-parent collision")
		}
		r.names[key] = current
		if i == len(parts)-1 {
			r.fileNames[key] = true
		}
	}
	return nil
}

// WriteFile copies one exact app/docs manifest member synchronously into a fresh
// protected object, then flushes and hashes the same retained handle. Every write
// failure poisons the owner; partial files/reservations are never removed/reused.
func (r *InstalledBaseline) WriteFile(ctx context.Context, name string, source io.Reader, size int64, digest string) error {
	return r.write(ctx, name, source, size, digest, false)
}

// WriteEvidence retains exact original bytes only for the two fixed sidecars and
// fixed engine and commissioning-v1 helpers. Their hashes keep their original meanings; package
// means the original Lifecycle manifest, never Recovery ZIP or payload.zip.
func (r *InstalledBaseline) WriteEvidence(ctx context.Context, name string, raw []byte, digest string) error {
	data := bytes.Clone(raw)
	return r.write(ctx, name, bytes.NewReader(data), int64(len(data)), digest, true)
}
func (r *InstalledBaseline) write(ctx context.Context, name string, source io.Reader, size int64, digest string, evidence bool) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	r.mu.Lock()
	defer r.mu.Unlock()
	if err := r.ready(); err != nil {
		return err
	}
	if r.readonly || r.completed {
		return errors.New("completed baseline is read-only")
	}
	defer func() {
		if resultErr != nil {
			r.failure = errors.Join(r.failure, resultErr)
		}
	}()
	if ctx == nil || source == nil || size < 0 || size > MaxFileBytes || !validHex(digest, 64) || len(r.files) >= MaxFiles || size > MaxPayloadBytes-r.bytes || (evidence && (size <= 0 || size > maxBaselineEvidenceBytes)) {
		return errors.New("bounded baseline member required")
	}
	ctx = registrationContext{Context: ctx, store: r.store}
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := r.reserve(name, evidence); err != nil {
		return err
	}
	file, err := r.backend.create(name)
	if err != nil {
		return err
	}
	r.files[name] = file
	c := Candidate{store: r.store, backend: r.backend}
	if err = c.copyFile(ctx, file, source, size, digest); err != nil {
		return errors.Join(err, ctx.Err())
	}
	r.record.Members[name] = baselineMember{Length: size, SHA256: digest}
	r.bytes += size
	return nil
}
func (r *InstalledBaseline) memberBytes(ctx context.Context, name string, bound int64) ([]byte, error) {
	m, ok := r.record.Members[name]
	if !ok || m.Length <= 0 || m.Length > bound {
		return nil, errors.New("required original baseline evidence missing or oversized")
	}
	file := r.files[name]
	if file == nil {
		return nil, errors.New("baseline evidence handle unavailable")
	}
	if err := verifyFile(ctx, r.store, file, m.Length, m.SHA256); err != nil {
		return nil, err
	}
	if _, err := file.Seek(0, io.SeekStart); err != nil {
		return nil, err
	}
	raw, err := io.ReadAll(io.LimitReader(checkedReader{ctx: ctx, store: r.store, reader: file}, m.Length+1))
	if err != nil {
		return nil, err
	}
	if int64(len(raw)) != m.Length || registrationDigest(raw) != m.SHA256 {
		return nil, errors.New("held original baseline evidence changed")
	}
	return raw, file.check(m.Length)
}
func (r *InstalledBaseline) memberNames() []string {
	names := make([]string, 0, len(r.files))
	for name := range r.files {
		names = append(names, name)
	}
	sort.Strings(names)
	return names
}
func (r *InstalledBaseline) validateContents(ctx context.Context) (baselineevidence.Identity, error) {
	var empty baselineevidence.Identity
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	if err := r.backend.check(); err != nil {
		return empty, err
	}
	if len(r.files) != len(r.record.Members) {
		return empty, errors.New("baseline retained member set differs")
	}
	observed := map[string]string{}
	for _, name := range r.memberNames() {
		m := r.record.Members[name]
		if err := verifyFile(ctx, r.store, r.files[name], m.Length, m.SHA256); err != nil {
			return empty, err
		}
		if !baselineSidecarName(name) {
			observed[name] = m.SHA256
		}
	}
	inventory, err := r.memberBytes(ctx, "FILE_SHA256.json", maxBaselineEvidenceBytes)
	if err != nil {
		return empty, err
	}
	manifest, err := r.memberBytes(ctx, "installer/package.json", maxBaselineEvidenceBytes)
	if err != nil {
		return empty, err
	}
	product, err := r.memberBytes(ctx, "docs/PRODUCT_BUILD.json", 32<<10)
	if err != nil {
		return empty, err
	}
	identity, _, err := baselineevidence.Validate(ctx, inventory, manifest, product, observed, r.expected)
	if err != nil {
		return empty, err
	}
	if err = r.backend.inventory(r.memberNames(), r.completed); err != nil {
		return empty, err
	}
	return identity, ctx.Err()
}

// Complete revalidates the exact retained original sidecars and full installed
// payload set, then publishes a creation/identity record and completed permanent
// reservation. Cancellation refuses before the final publication critical section;
// successful durable completion remains success after later cancellation. Actual
// publication failure is ErrInstalledBaselineUncertain and may already be durable.
// The supplied identity must come from ImportApprovedBaseline using the exact
// independently approved original request and retained source. Open additionally
// binds every pin to the separately reparsed approved request. There is one
// attempt, no launch, retry, reset, deletion or partial adoption.
func (r *InstalledBaseline) Complete(ctx context.Context, identity baselineevidence.Identity) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	r.mu.Lock()
	defer r.mu.Unlock()
	if err := r.ready(); err != nil {
		return err
	}
	if r.completed || r.readonly {
		return errors.New("baseline already completed")
	}
	defer func() {
		if resultErr != nil {
			r.failure = errors.Join(r.failure, resultErr)
		}
	}()
	if ctx == nil {
		return errors.New("live completion context required")
	}
	ctx = registrationContext{Context: ctx, store: r.store}
	r.expected = brokerregistration.Installed{Version: identity.Version, Commit: identity.Commit, Generation: r.record.Generation, PackageSHA256: identity.LifecycleManifestSHA256, ProductBuildSHA256: identity.ProductBuildSHA256, ExecutableSHA256: identity.ExecutableSHA256, InventorySHA256: identity.InventorySHA256}
	if err := baselineExpected(r.expected, r.record.Generation); err != nil {
		return err
	}
	actual, err := r.validateContents(ctx)
	if err != nil {
		return err
	}
	if actual != identity {
		return errors.New("baseline differs from validated import identity")
	}
	dirs, files, err := r.backend.identities(r.record.Members)
	if err != nil {
		return err
	}
	r.record.Identity = actual
	r.record.Directories = dirs
	for name, id := range files {
		m := r.record.Members[name]
		m.Identity = id
		r.record.Members[name] = m
	}
	if len(files) != len(r.record.Members) {
		return errors.New("baseline identity set differs")
	}
	raw, err := json.Marshal(r.record)
	if err != nil {
		return err
	}
	if len(raw) > maxBaselineRecordBytes {
		return errors.New("baseline completion record bound")
	}
	if err = r.backend.publish(ctx, raw); err != nil {
		return err
	}
	r.completed = true
	return nil
}

// Validate rechecks retained physical custody, exact namespace inventory and all
// baseline bytes/schema. It grants no authority to execute the resulting tree.
func (r *InstalledBaseline) Validate() (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	r.mu.Lock()
	defer r.mu.Unlock()
	if err := r.ready(); err != nil {
		return err
	}
	if !r.completed {
		return errors.New("completed baseline required")
	}
	defer func() {
		if resultErr != nil {
			r.failure = errors.Join(r.failure, resultErr)
		}
	}()
	identity, err := r.validateContents(registrationContext{Context: context.Background(), store: r.store})
	if err != nil {
		return err
	}
	if identity != r.record.Identity {
		return errors.New("retained baseline identity differs")
	}
	return nil
}

// Directory returns the corroborated completed fixed tree path while custody is
// held. A returned path is informational and confers no launch authority.
func (r *InstalledBaseline) Directory() (string, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	r.mu.Lock()
	defer r.mu.Unlock()
	if err := r.ready(); err != nil {
		return "", err
	}
	if !r.completed {
		return "", errors.New("completed baseline required")
	}
	return r.backend.directory()
}

// Close drains every retained handle exactly once and retains the original
// failure/cleanup result. It never deletes or resets permanent baseline evidence.
func (r *InstalledBaseline) Close() error {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.closed {
		return r.closeErr
	}
	r.closed = true
	r.closeErr = r.failure
	for _, name := range r.memberNames() {
		r.closeErr = errors.Join(r.closeErr, r.files[name].Close())
	}
	r.files = nil
	r.closeErr = errors.Join(r.closeErr, r.backend.close())
	return r.closeErr
}
func baselineMarker(sid, generation, approved, digest string, completed bool) []byte {
	status := "0"
	if completed {
		status = "1"
	} else {
		digest = strings.Repeat("0", 64)
	}
	return []byte(strings.Join([]string{baselineMarkerHeader, status, sid, generation, approved, digest, ""}, "\n"))
}
func completedBaselineDigest(raw []byte, sid, generation, approved string) (string, error) {
	parts := strings.Split(string(raw), "\n")
	if len(raw) > maxRegistrationMarkerBytes || len(parts) != 7 || parts[0] != baselineMarkerHeader || parts[1] != "1" || parts[2] != sid || parts[3] != generation || parts[4] != approved || !validHex(parts[5], 64) || parts[6] != "" {
		return "", errors.New("completed approved baseline reservation required")
	}
	return parts[5], nil
}
