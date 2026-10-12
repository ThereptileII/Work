package brokerhost

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"fmt"
	"sync"
)

const executionProfileLimit = 4 << 20

type executionProfileID struct {
	volume  uint64
	file    [16]byte
	created uint64
}

func (id executionProfileID) wire() string {
	return fmt.Sprintf("%016x %s %016x", id.volume, hex.EncodeToString(id.file[:]), id.created)
}

type executionProfileOriginal struct {
	directory, file executionProfileID
	digest          string
	raw             []byte
}

func (o executionProfileOriginal) wire() string {
	return o.directory.wire() + " " + o.file.wire() + " " + o.digest
}

type executionProfileBackend interface {
	validate(context.Context) error
	beforeLaunch(context.Context, executionProfileOriginal) error
	close() error
}
type executionColdProfileState struct {
	mu                sync.Mutex
	original          executionProfileOriginal
	backend           executionProfileBackend
	closed            bool
	failure, closeErr error
}

// ExecutionColdProfile owns a native snapshot of the fixed real Windows profile.
// It proves original bytes and physical identities, not their semantic approval,
// coldness, continuous writer exclusion, restoration or permission to launch.
// The temporary INI deny-write handle is released before the application loads.
// Only ancestry remains pinned; normal application writes are allowed afterward.
type ExecutionColdProfile struct{ state *executionColdProfileState }

// CaptureExecutionColdProfileNative accepts no path, digest or serialized proof.
// The transaction must independently enforce its cold/armed/commissioning policy.
func CaptureExecutionColdProfileNative(ctx context.Context) (*ExecutionColdProfile, error) {
	if ctx == nil || ctx.Err() != nil {
		return nil, errors.Join(ErrExecutionChildRefused, contextError(ctx))
	}
	return captureExecutionColdProfileNative(ctx)
}
func contextError(ctx context.Context) error {
	if ctx == nil {
		return ErrExecutionChildRefused
	}
	return ctx.Err()
}
func (s *executionColdProfileState) validate(ctx context.Context, before bool) error {
	if s == nil || s.closed || s.backend == nil || ctx == nil {
		return ErrExecutionChildRefused
	}
	if s.failure != nil {
		return errors.Join(ErrExecutionChildRefused, s.failure)
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	digest := sha256.Sum256(s.original.raw)
	if len(s.original.raw) == 0 || len(s.original.raw) > executionProfileLimit || hex.EncodeToString(digest[:]) != s.original.digest {
		s.failure = ErrExecutionChildRefused
		return s.failure
	}
	err := s.backend.validate(ctx)
	if err == nil && before {
		err = s.backend.beforeLaunch(ctx, s.original)
	}
	if err != nil {
		s.failure = err
	}
	return err
}

// Validate rechecks retained ancestry and owned original evidence. It deliberately
// does not require the INI to remain unchanged after the application's real load.
func (p *ExecutionColdProfile) Validate(ctx context.Context) error { return p.validate(ctx, false) }

// ValidateBeforeLaunch transiently reopens the actual fixed INI and requires its
// original identity and bytes. The read lock is released before returning.
func (p *ExecutionColdProfile) ValidateBeforeLaunch(ctx context.Context) error {
	return p.validate(ctx, true)
}
func (p *ExecutionColdProfile) validate(ctx context.Context, before bool) error {
	if p == nil || p.state == nil {
		return ErrExecutionChildRefused
	}
	s := p.state
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.validate(ctx, before)
}

// OriginalBytes returns an owned bounded copy for durable recovery storage only.
// There is no public constructor to turn this copy back into a live capability.
func (p *ExecutionColdProfile) OriginalBytes(ctx context.Context) ([]byte, error) {
	if p == nil || p.state == nil {
		return nil, ErrExecutionChildRefused
	}
	s := p.state
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.validate(ctx, false); err != nil {
		return nil, err
	}
	return append([]byte(nil), s.original.raw...), nil
}
func (p *ExecutionColdProfile) configuration(ctx context.Context) (string, error) {
	if p == nil || p.state == nil {
		return "", ErrExecutionChildRefused
	}
	s := p.state
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.validate(ctx, false); err != nil {
		return "", err
	}
	return s.original.wire(), nil
}
func (p *ExecutionColdProfile) Close() error {
	if p == nil || p.state == nil {
		return nil
	}
	s := p.state
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed {
		return s.closeErr
	}
	s.closed = true
	s.closeErr = s.backend.close()
	if s.closeErr != nil {
		s.closeErr = errors.Join(ErrExecutionChildCleanup, s.closeErr)
	}
	return s.closeErr
}
func (*ExecutionColdProfile) MarshalJSON() ([]byte, error) { return nil, ErrExecutionChildRefused }
func (*ExecutionColdProfile) UnmarshalJSON([]byte) error   { return ErrExecutionChildRefused }
