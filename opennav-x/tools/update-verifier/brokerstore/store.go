// Package brokerstore stores inert candidates under the distinct Windows
// SkagerUpdateBroker virtual account. It neither provisions a service nor grants
// launch authority. The root must have been protected when originally created;
// validating a present DACL cannot prove that no earlier writable handle exists.
// DOS paths are corroborated at operation boundaries, not continuously pinned;
// callers must run only as the non-impersonated broker in its own DOS namespace.
// Durable partial candidates count toward a fixed cap and require explicit
// operator/recovery reconciliation. This package never evicts or adopts them.
package brokerstore

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"runtime"
	"strings"
	"sync"
	"sync/atomic"

	"example.com/opennav-update-verifier/baselineevidence"
)

const (
	BrokerAccount              = `NT SERVICE\SkagerUpdateBroker`
	ReceiptFilename            = "staging-receipt.json"
	MaxFileBytes         int64 = 512 << 20
	MaxPayloadBytes      int64 = 2 << 30
	MaxFiles                   = 12000
	MaxCandidates              = 128
	MaxReceiptBytes            = 4 << 20
	MaxRegistrationBytes       = 1 << 20
)

var (
	ErrNativeRequired = errors.New("protected broker storage requires native Windows")
	ErrPrincipal      = errors.New("actual non-impersonated SkagerUpdateBroker account required")
	ErrClosed         = errors.New("broker storage custody closed")
	ErrPoisoned       = errors.New("candidate failed or uncertain; fresh stage identity required")
)

type storeBackend interface {
	check() error
	begin(string, string) (candidateBackend, error)
	registration(string) ([]byte, error)
	verifierDirectories(string, string) (string, string, error)
	close() error
}
type candidateBackend interface {
	check() error
	create(string) (candidateFile, error)
	close() error
}
type candidateFile interface {
	io.ReadWriteSeeker
	Sync() error
	check(int64) error
	Close() error
}

// Store retains native root and ancestor handles. Close never deletes an
// abandoned candidate or registration, or resets their permanent reservations.
type Store struct {
	mu             sync.Mutex
	closeMu        sync.Mutex
	backend        storeBackend
	closed         atomic.Bool
	closeErr       error
	candidates     []*Candidate
	registrations  []*Registration
	baselines      []*InstalledBaseline
	generations    []*InstalledGeneration
	journals       []*ExecutionJournal
	journalReaders []*ExecutionJournalReader
	profileBackups []*executionProfileBackupState
}
type fileRecord struct {
	file   candidateFile
	length int64
	digest string
}

// Candidate is one new protected namespace. Client read/execute access is
// established atomically at creation, never by retrofitting a writable tree.
type Candidate struct {
	mu                       sync.Mutex
	store                    *Store
	backend                  candidateBackend
	stageID, clientSID       string
	materialized             bool
	files                    []fileRecord
	names                    map[string]string
	fileNames                map[string]bool
	bytes                    int64
	closed, poisoned, sealed bool
	closeErr                 error
	acquisition              *Acquisition
	recoveryEvidence         *baselineevidence.Identity
	recoveryMembers          map[string]fileRecord
}

func validHex(s string, n int) bool {
	if len(s) != n {
		return false
	}
	for _, c := range s {
		if !(c >= '0' && c <= '9' || c >= 'a' && c <= 'f') {
			return false
		}
	}
	return true
}
func memberPath(s string) error {
	if len(s) == 0 || len(s) > 240 || strings.Contains(s, `\`) {
		return errors.New("bounded app/docs slash-relative member required")
	}
	parts := strings.Split(s, "/")
	if len(parts) < 2 || (parts[0] != "app" && parts[0] != "docs") {
		return errors.New("only app/docs payload members allowed")
	}
	for _, p := range parts {
		if !safeComponent(p) {
			return errors.New("ambiguous Windows member")
		}
	}
	return nil
}
func safeComponent(s string) bool {
	if s == "" || s == "." || s == ".." || strings.HasSuffix(s, ".") || strings.HasSuffix(s, " ") {
		return false
	}
	for _, c := range s {
		if c < 32 || c > 126 || strings.ContainsRune(`<>:"/\|?*`, c) {
			return false
		}
	}
	base := strings.ToUpper(strings.SplitN(s, ".", 2)[0])
	base = strings.TrimRight(base, " ")
	if base == "CON" || base == "PRN" || base == "AUX" || base == "NUL" {
		return false
	}
	return !(len(base) == 4 && (strings.HasPrefix(base, "COM") || strings.HasPrefix(base, "LPT")) && base[3] >= '1' && base[3] <= '9')
}
func (s *Store) Begin(stageID, clientSID string) (*Candidate, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if !validHex(stageID, 32) {
		return nil, errors.New("lowercase 32-hex stage identity required")
	}
	if err := s.backend.check(); err != nil {
		return nil, err
	}
	backend, err := s.backend.begin(stageID, clientSID)
	if err != nil {
		return nil, err
	}
	c := &Candidate{store: s, backend: backend, stageID: stageID, clientSID: clientSID, names: map[string]string{}, fileNames: map[string]bool{}}
	s.candidates = append(s.candidates, c)
	return c, nil
}

// ReadRegistration reads only the fixed protected registrations/<SID>.json.
// It grants no authority to paths or ACLs which its JSON might contain.
func (s *Store) ReadRegistration(clientSID string) ([]byte, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if err := s.backend.check(); err != nil {
		return nil, err
	}
	return s.backend.registration(clientSID)
}

// VerifierDirectories returns only existing fixed KnownFolder ProgramData DOS
// spellings. Their physical ancestors remain held; Validate corroborates their
// resolution. This is not a continuously pinned DOS-device mapping.
func (s *Store) VerifierDirectories(clientSID, channel string) (string, string, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return "", "", ErrClosed
	}
	if channel != "beta" && channel != "stable" {
		return "", "", errors.New("fixed beta/stable trust channel required")
	}
	if err := s.backend.check(); err != nil {
		return "", "", err
	}
	return s.backend.verifierDirectories(clientSID, channel)
}

// Validate corroborates the actual broker token, held physical ancestry and
// exact protected security descriptors, plus previously returned fixed DOS paths.
func (s *Store) Validate() error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return ErrClosed
	}
	return s.backend.check()
}

// Close drains owned handles once. Concurrent and repeated callers wait for and
// receive the original cleanup result, including any retained uncertainty.
func (s *Store) Close() error {
	// Mark the store unavailable before draining children, but do not report
	// completed cleanup to another caller until that original drain finishes.
	s.closeMu.Lock()
	defer s.closeMu.Unlock()
	s.mu.Lock()
	if s.closed.Swap(true) {
		s.mu.Unlock()
		return s.closeErr
	}
	children := s.candidates
	s.candidates = nil
	registrations := s.registrations
	s.registrations = nil
	baselines := s.baselines
	s.baselines = nil
	generations := s.generations
	s.generations = nil
	profileBackups := s.profileBackups
	s.profileBackups = nil
	journalReaders := s.journalReaders
	s.journalReaders = nil
	journals := s.journals
	s.journals = nil
	s.mu.Unlock()
	var result error
	for _, p := range profileBackups {
		result = errors.Join(result, p.close())
	}
	for _, r := range journalReaders {
		result = errors.Join(result, r.Close())
	}
	for _, j := range journals {
		result = errors.Join(result, j.Close())
	}
	for _, g := range generations {
		result = errors.Join(result, g.Close())
	}
	for _, b := range baselines {
		result = errors.Join(result, b.Close())
	}
	for _, r := range registrations {
		result = errors.Join(result, r.Close())
	}
	for _, c := range children {
		result = errors.Join(result, c.Close())
	}
	s.closeErr = errors.Join(result, s.backend.close())
	return s.closeErr
}
func (c *Candidate) ready() error {
	if c.closed || c.store.closed.Load() {
		return ErrClosed
	}
	if c.poisoned {
		return ErrPoisoned
	}
	if c.sealed {
		return errors.New("candidate already sealed")
	}
	if err := c.backend.check(); err != nil {
		c.poisoned = true
		return err
	}
	return nil
}
func (c *Candidate) reserve(relative string) error {
	if err := memberPath(relative); err != nil {
		return err
	}
	parts := strings.Split(relative, "/")
	for i := range parts {
		current := strings.Join(parts[:i+1], "/")
		key := strings.ToLower(current)
		if old, ok := c.names[key]; ok && (old != current || c.fileNames[key] || i == len(parts)-1) {
			return errors.New("duplicate, case alias, or file/parent member collision")
		}
		c.names[key] = current
		if i == len(parts)-1 {
			c.fileNames[key] = true
		}
	}
	return nil
}
func (c *Candidate) WriteFile(ctx context.Context, relative string, content io.Reader, length int64, digest string) (err error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c.mu.Lock()
	defer c.mu.Unlock()
	if err = c.ready(); err != nil {
		return err
	}
	defer func() {
		if err != nil {
			c.poisoned = true
		}
	}()
	if c.recoveryEvidence != nil {
		return errors.New("recovery evidence already completed")
	}
	if ctx == nil || content == nil || length < 0 || length > MaxFileBytes || !validHex(digest, 64) || len(c.files) >= MaxFiles || length > MaxPayloadBytes-c.bytes {
		return errors.New("invalid or exhausted candidate write bounds")
	}
	if err = ctx.Err(); err != nil {
		return err
	}
	if err = c.reserve(relative); err != nil {
		return err
	}
	file, err := c.backend.create(relative)
	if err != nil {
		return err
	}
	c.files = append(c.files, fileRecord{file: file, length: length, digest: digest})
	if err = c.copyFile(ctx, file, content, length, digest); err != nil {
		return err
	}
	c.bytes += length
	if c.recoveryMembers == nil {
		c.recoveryMembers = map[string]fileRecord{}
	}
	c.recoveryMembers[relative] = fileRecord{file: file, length: length, digest: digest}
	return nil
}

type checkedReader struct {
	ctx    context.Context
	store  *Store
	reader io.Reader
}

func (r checkedReader) Read(p []byte) (int, error) {
	if err := r.ctx.Err(); err != nil {
		return 0, err
	}
	if r.store.closed.Load() {
		return 0, ErrClosed
	}
	return r.reader.Read(p)
}
func (c *Candidate) copyFile(ctx context.Context, file candidateFile, content io.Reader, length int64, digest string) error {
	reader := checkedReader{ctx: ctx, store: c.store, reader: content}
	hash := sha256.New()
	n, err := io.CopyBuffer(io.MultiWriter(file, hash), io.LimitReader(reader, length), make([]byte, 64<<10))
	if err != nil {
		return err
	}
	if n != length {
		return io.ErrUnexpectedEOF
	}
	var extra [1]byte
	nExtra, err := reader.Read(extra[:])
	if nExtra != 0 || err != io.EOF {
		return errors.New("content exceeds declared length or EOF uncertain")
	}
	if hex.EncodeToString(hash.Sum(nil)) != digest {
		return errors.New("content digest mismatch")
	}
	if err = file.Sync(); err != nil {
		return err
	}
	if err = ctx.Err(); err != nil {
		return err
	}
	if err := verifyFile(ctx, c.store, file, length, digest); err != nil {
		return err
	}
	return c.backend.check()
}
func verifyFile(ctx context.Context, store *Store, file candidateFile, length int64, digest string) error {
	if err := file.check(length); err != nil {
		return err
	}
	if _, err := file.Seek(0, io.SeekStart); err != nil {
		return err
	}
	h := sha256.New()
	n, err := io.Copy(h, io.LimitReader(checkedReader{ctx: ctx, store: store, reader: file}, length+1))
	if err != nil {
		return err
	}
	if n != length || hex.EncodeToString(h.Sum(nil)) != digest {
		return errors.New("held staged bytes differ")
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if store.closed.Load() {
		return ErrClosed
	}
	return file.check(length)
}

// Seal writes the fixed receipt after complete original recovery evidence and
// exact payload membership have been bound and rechecked through held files.
// Materialized candidates also require an independently reopened same-Store
// generation and a fresh durable seal binding the original receipt file identity.
// The broker must bind its own receipt schema to the authenticated preparation;
// this storage boundary treats receipt bytes as opaque, never launch authority.
func (c *Candidate) Seal(ctx context.Context, receipt []byte) error {
	return c.sealStage(ctx, receipt)
}

// Close releases owned handles once and retains the original cleanup result.
// It does not remove the durable stage reservation or authorize its reuse.
func (c *Candidate) Close() error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.closed {
		return c.closeErr
	}
	c.closed = true
	var result error
	if c.acquisition != nil {
		result = errors.Join(result, c.acquisition.closeLocked())
	}
	for _, entry := range c.files {
		result = errors.Join(result, entry.file.Close())
	}
	c.files = nil
	c.closeErr = errors.Join(result, c.backend.close())
	return c.closeErr
}
