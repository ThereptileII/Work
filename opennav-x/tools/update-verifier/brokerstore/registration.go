package brokerstore

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"runtime"
	"strings"
	"sync"
)

const registrationMarkerHeader = "SKAGER-client-registration-v1"
const maxRegistrationMarkerBytes = 1024

// This optional native capability does not add registration creation to ordinary
// staging backends. It accepts identities, never caller-selected storage paths.
type registrationStoreBackend interface {
	prepareRegistration(string, string, string) (registrationBackend, error)
}
type registrationBackend interface {
	check() error
	reserve(context.Context) error
	createUser() error
	createTrust() error
	createArtifacts() error
	directories() (string, string, error)
	complete(context.Context, []byte) error
	close() error
}

// Registration retains one fresh client namespace and its permanent reservation.
// Neither its paths nor completion confer launch or registration-input authority.
// The caller must independently approve the request and validate the registration
// schema, observed installation and trust configuration before Complete.
type Registration struct {
	mu                          sync.Mutex
	store                       *Store
	backend                     registrationBackend
	closed, poisoned, completed bool
	closeErr                    error
}

// BeginRegistrationFresh consumes registrations/<canonical SID>.once before
// creating the fresh broker-private client tree. Every failure retains permanent
// evidence and partial state. Existing client namespaces are never adopted.
// Only an actual, non-impersonated native broker store supports this operation.
func (s *Store) BeginRegistrationFresh(ctx context.Context, clientSID, channel, approvedRequestSHA256 string) (*Registration, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed.Load() {
		return nil, ErrClosed
	}
	if ctx == nil || clientSID == "" || (channel != "beta" && channel != "stable") || !validHex(approvedRequestSHA256, 64) {
		return nil, errors.New("bounded client identity, beta/stable channel and approved request SHA-256 required")
	}
	ctx = registrationContext{Context: ctx, store: s}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := s.backend.check(); err != nil {
		return nil, err
	}
	backend, ok := s.backend.(registrationStoreBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	b, err := backend.prepareRegistration(clientSID, channel, approvedRequestSHA256)
	if err != nil {
		return nil, err
	}
	if err = beginRegistration(ctx, b); err != nil {
		return nil, err
	}
	r := &Registration{store: s, backend: b}
	s.registrations = append(s.registrations, r)
	return r, nil
}

func beginRegistration(ctx context.Context, b registrationBackend) (resultErr error) {
	ok := false
	defer func() {
		if !ok {
			resultErr = errors.Join(resultErr, b.close())
		}
	}()
	for _, step := range []func() error{
		b.check, func() error { return b.reserve(ctx) }, b.check,
		b.createUser, b.check, b.createTrust, b.check, b.createArtifacts, b.check,
	} {
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

// The outer store can stop an in-flight bounded operation while Close waits for
// this owner's lock. All native mutation boundaries check this context explicitly.
type registrationContext struct {
	context.Context
	store *Store
}

func (c registrationContext) Err() error {
	if c.store.closed.Load() {
		return ErrClosed
	}
	return c.Context.Err()
}
func (r *Registration) ready() error {
	if r.closed || r.store.closed.Load() {
		return ErrClosed
	}
	if r.poisoned {
		return ErrPoisoned
	}
	return r.backend.check()
}

// VerifierDirectories returns a fixed absent beta/stable state path and an
// already-created private artifact directory. The caller must run InitializeState
// and ValidateState using independently approved trust before Complete. This API
// never initializes, repairs or resets trust. The owner remains held throughout.
func (r *Registration) VerifierDirectories() (state, artifacts string, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	r.mu.Lock()
	defer r.mu.Unlock()
	defer func() {
		if resultErr != nil {
			r.poisoned = true
		}
	}()
	if err := r.ready(); err != nil {
		return "", "", err
	}
	return r.backend.directories()
}

// Complete publishes exact bounded registration bytes only after the fixed
// channel directory has appeared with the broker's exact security descriptor.
// The completed permanent marker binds the approved request and registration
// SHA-256. Readers refuse started/partial markers or mismatched registration bytes.
// There is one attempt; failures retain state and require explicit operator review.
// Close must release writer custody before ReadRegistration can open the evidence.
func (r *Registration) Complete(ctx context.Context, rawRegistration []byte) (resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.closed || r.store.closed.Load() {
		return ErrClosed
	}
	if r.completed {
		return errors.New("registration already completed")
	}
	defer func() {
		if resultErr != nil {
			r.poisoned = true
		}
	}()
	if err := r.ready(); err != nil {
		return err
	}
	if ctx == nil || len(rawRegistration) == 0 || len(rawRegistration) > MaxRegistrationBytes {
		return errors.New("nonempty bounded approved registration required")
	}
	ctx = registrationContext{Context: ctx, store: r.store}
	if err := ctx.Err(); err != nil {
		return err
	}
	// Own immutable input while native writes and readback execute.
	data := append([]byte(nil), rawRegistration...)
	if err := r.backend.complete(ctx, data); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	r.completed = true
	return nil
}

// Close drains handles exactly once and retains any cleanup uncertainty. It never
// removes the permanent marker, registration, client directories or trust state.
func (r *Registration) Close() error {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.closed {
		return r.closeErr
	}
	r.closed = true
	r.closeErr = r.backend.close()
	return r.closeErr
}

// A fixed-width status and digest make completion overwrite the started marker
// without truncation or pathname replacement. A partial overwrite cannot grant
// authority unless the entire exact completed record and registration hash agree.
func registrationMarker(client, channel, request, digest string, completed bool) []byte {
	status := "0"
	if completed {
		status = "1"
	} else {
		digest = strings.Repeat("0", 64)
	}
	return []byte(strings.Join([]string{registrationMarkerHeader, status, client, channel, request, digest, ""}, "\n"))
}
func completedRegistrationDigest(data []byte, client string) (string, error) {
	parts := strings.Split(string(data), "\n")
	if len(data) > maxRegistrationMarkerBytes || len(parts) != 7 || parts[0] != registrationMarkerHeader || parts[1] != "1" || parts[2] != client ||
		(parts[3] != "beta" && parts[3] != "stable") || !validHex(parts[4], 64) || !validHex(parts[5], 64) || parts[6] != "" {
		return "", errors.New("completed exact client registration marker required")
	}
	return parts[5], nil
}
func registrationDigest(data []byte) string {
	sum := sha256.Sum256(data)
	return hex.EncodeToString(sum[:])
}

// writeRegistrationBytes verifies writes and flushes against the same retained
// file. The fixed state check brackets every mutation and verified readback.
func writeRegistrationBytes(ctx context.Context, file candidateFile, before int64, data []byte, check func() error) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := check(); err != nil {
		return err
	}
	if err := file.check(before); err != nil {
		return err
	}
	if _, err := file.Seek(0, io.SeekStart); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	n, err := file.Write(data)
	if err != nil {
		return err
	}
	if n != len(data) {
		return io.ErrShortWrite
	}
	if err = file.Sync(); err != nil {
		return err
	}
	if err = ctx.Err(); err != nil {
		return err
	}
	if err = file.check(int64(len(data))); err != nil {
		return err
	}
	if _, err = file.Seek(0, io.SeekStart); err != nil {
		return err
	}
	actual, err := io.ReadAll(io.LimitReader(file, int64(len(data))+1))
	if err != nil {
		return err
	}
	if !bytes.Equal(actual, data) {
		return errors.New("held registration bytes differ")
	}
	if err = ctx.Err(); err != nil {
		return err
	}
	if err = file.check(int64(len(data))); err != nil {
		return err
	}
	return check()
}
