package brokerstore

import (
	"context"
	"errors"
	"runtime"
	"strings"
	"sync"
)

var ErrDecommissioned = errors.New("terminal broker decommission fence present or uncertain")
var ErrDecommissionUncertain = errors.New("terminal decommission publication uncertain; preserve all state")

// DecommissionBinding contains comparisons supplied by independently retained
// host owners. These strings confer no profile, process or service authority.
// The terminal action preserves all application, profile and recovery objects.
type DecommissionBinding struct {
	RequestID             string
	ClientSID             string
	RegistrationSHA256    string
	SelectedHeadSHA256    string
	GenerationID          string
	CompletionSHA256      string
	ProfileSHA256         string
	ProfileIdentitySHA256 string
}

func (b DecommissionBinding) valid() bool {
	if !validHex(b.RequestID, 32) || !validHex(b.GenerationID, 32) || len(b.ClientSID) > 184 || !safeComponent(b.ClientSID) || !strings.HasPrefix(b.ClientSID, "S-1-") {
		return false
	}
	for _, hash := range []string{b.RegistrationSHA256, b.SelectedHeadSHA256, b.CompletionSHA256, b.ProfileSHA256, b.ProfileIdentitySHA256} {
		if !validHex(hash, 64) {
			return false
		}
	}
	return true
}
func decommissionBytes(b DecommissionBinding) []byte {
	return []byte(strings.Join([]string{"SKAGER-terminal-decommission-v1", b.RequestID, b.ClientSID, b.RegistrationSHA256, b.SelectedHeadSHA256, b.GenerationID, b.CompletionSHA256, b.ProfileSHA256, b.ProfileIdentitySHA256, ""}, "\n"))
}

type decommissionStoreBackend interface {
	singletonRegistration(context.Context, string, string) error
	prepareDecommission(context.Context, DecommissionBinding) (decommissionBackend, error)
}
type decommissionBackend interface {
	commit(context.Context) error
	check() error
	close() error
}

// Decommission is same-process, same-original-store completion custody. It cannot
// be reconstructed from the marker or a caller JSON receipt. Binding returns a
// value copy for an independently owned maintenance conversation to compare.
type Decommission struct {
	self             *Decommission
	mu               sync.Mutex
	store            *Store
	backend          decommissionBackend
	binding          DecommissionBinding
	closed, poisoned bool
	closeErr         error
}

func (d *Decommission) Binding() DecommissionBinding {
	if d == nil || d.self != d {
		return DecommissionBinding{}
	}
	return d.binding
}
func (d *Decommission) Validate(ctx context.Context) error {
	if d == nil || d.self != d || d.store == nil || d.backend == nil || ctx == nil {
		return ErrClosed
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.closed || d.store.closed.Load() {
		return ErrClosed
	}
	if d.poisoned {
		return ErrPoisoned
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := d.backend.check(); err != nil {
		d.poisoned = true
		return err
	}
	return nil
}

// Close releases custody only; it never removes or resets the terminal fence.
func (d *Decommission) Close() error {
	if d == nil {
		return nil
	}
	if d.self != d || d.backend == nil {
		return ErrNativeRequired
	}
	d.mu.Lock()
	defer d.mu.Unlock()
	if !d.closed {
		d.closed = true
		d.closeErr = d.backend.close()
	}
	return d.closeErr
}

// ValidateSingletonRegistration refuses every extra, partial or unknown entry.
// A successful read is only a preflight; CommitDecommission repeats the check.
func (s *Store) ValidateSingletonRegistration(ctx context.Context, sid, digest string) error {
	if s == nil || ctx == nil || !validHex(digest, 64) || len(sid) > 184 || !safeComponent(sid) {
		return errors.New("bounded original registration required")
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return ErrClosed
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := s.backend.check(); err != nil {
		return err
	}
	b, ok := s.backend.(decommissionStoreBackend)
	if !ok {
		return ErrNativeRequired
	}
	return b.singletonRegistration(ctx, sid, digest)
}

// CommitDecommission makes one attempt on this original Store. The original host
// guard runs WITHOUT store locks, and may call Store.Validate/ReadRegistration.
// The caller must retain its process/image/profile freeze and serialize host
// effects throughout. Private custody, exact singleton inventory and cancellation
// are rechecked immediately afterward, before CREATE_NEW. Once creation begins,
// cancellation cannot erase the fence or relabel successful durable completion.
// Any fence presence permanently denies ordinary native store use/open. There is
// no retry, adoption, deletion or profile mutation, including after service stop.
func (s *Store) CommitDecommission(ctx context.Context, binding DecommissionBinding, originalGuard func(context.Context) error) (*Decommission, error) {
	if s == nil || ctx == nil || originalGuard == nil || !binding.valid() {
		return nil, errors.New("exact bounded decommission binding and original guard required")
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	if s.closed.Load() {
		s.mu.Unlock()
		return nil, ErrClosed
	}
	if s.decommissionAttempted {
		s.mu.Unlock()
		return nil, errors.New("decommission already attempted; explicit recovery required")
	}
	if _, ok := s.backend.(decommissionStoreBackend); !ok {
		s.mu.Unlock()
		return nil, ErrNativeRequired
	}
	if err := s.backend.check(); err != nil {
		s.mu.Unlock()
		return nil, err
	}
	s.decommissionAttempted = true
	s.mu.Unlock()
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := originalGuard(ctx); err != nil {
		return nil, err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := s.backend.check(); err != nil {
		return nil, err
	}
	b, ok := s.backend.(decommissionStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	native, err := b.prepareDecommission(ctx, binding)
	if err != nil {
		return nil, err
	}
	d := &Decommission{store: s, backend: native, binding: binding}
	d.self = d
	s.decommissions = append(s.decommissions, d)
	if err = native.commit(ctx); err != nil {
		d.poisoned = true
		return nil, err
	}
	return d, nil
}

// publishDecommission owns the irreversible boundary. Once CREATE_NEW is
// attempted, every actual I/O/custody error is uncertain; cancellation alone
// cannot interrupt the bounded write/flush/exact readback or remove the fence.
func publishDecommission(ctx context.Context, create func() (candidateFile, error), raw []byte, check func() error) (file candidateFile, resultErr error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(ErrDecommissionUncertain, resultErr)
		}
	}()
	file, err := create()
	if err != nil {
		return file, err
	}
	if err = writeRegistrationBytes(context.WithoutCancel(ctx), file, 0, raw, check); err != nil {
		return file, err
	}
	return file, nil
}

// EvidenceSHA256 exposes comparison data only after validating the same original
// completed writer. It never parses or adopts a persistent fence as authority.
func (d *Decommission) EvidenceSHA256(ctx context.Context) (string, error) {
	if err := d.Validate(ctx); err != nil {
		return "", err
	}
	return registrationDigest(decommissionBytes(d.binding)), nil
}
func (*Decommission) MarshalJSON() ([]byte, error) { return nil, ErrNativeRequired }
func (*Decommission) UnmarshalJSON([]byte) error   { return ErrNativeRequired }

// CloseExecutionJournalReader releases only a clean same-client reader from this
// original Store, after private terminal inventory revalidation. Ordinary reader
// validation remains unchanged and therefore refuses the terminal store. A
// poisoned reader/receipt never gains a cleanup-success claim through this API.
func (d *Decommission) CloseExecutionJournalReader(ctx context.Context, reader *ExecutionJournalReader) error {
	if d == nil || d.self != d || d.store == nil || ctx == nil {
		return ErrNativeRequired
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	d.store.mu.Lock()
	defer d.store.mu.Unlock()
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.closeExecutionJournalReaderLocked(ctx, reader)
}

// Store.mu and receipt.mu are held, before Store.closed is set.
func (d *Decommission) closeExecutionJournalReaderLocked(ctx context.Context, reader *ExecutionJournalReader) error {
	if d.self != d || d.closed || d.poisoned || d.backend == nil || d.store.closed.Load() || ctx.Err() != nil || reader == nil || reader.state == nil {
		return ErrNativeRequired
	}
	r := reader.state
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.store != d.store || r.origin.ClientSID != d.binding.ClientSID {
		return ErrNativeRequired
	}
	if r.closed {
		return r.closeErr
	}
	if r.failure != nil {
		return r.failure
	}
	if err := d.backend.check(); err != nil {
		d.poisoned = true
		return err
	}
	r.closed = true
	r.closeErr = r.backend.close()
	if r.closeErr != nil {
		r.closeErr = errors.Join(ErrExecutionJournalUncertain, r.closeErr)
	}
	return r.closeErr
}

// Successful terminal shutdown drains retained clean readers while the original
// receipt and store lock still exist. Failed/closed custody takes the ordinary
// uncertainty-preserving cleanup path; it never fabricates successful release.
func (s *Store) drainDecommissionReadersLocked() error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, d := range s.decommissions {
		if d == nil || d.self != d {
			continue
		}
		d.mu.Lock()
		if d.closed || d.poisoned || d.backend == nil {
			d.mu.Unlock()
			continue
		}
		var result error
		for _, reader := range s.journalReaders {
			if reader == nil || reader.state == nil {
				continue
			}
			reader.state.mu.Lock()
			closed := reader.state.closed
			reader.state.mu.Unlock()
			if !closed {
				result = errors.Join(result, d.closeExecutionJournalReaderLocked(context.Background(), reader))
			}
		}
		d.mu.Unlock()
		return result
	}
	return nil
}
