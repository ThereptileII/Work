package brokerstore

import (
	"errors"
	"os"
	"runtime"
)

// Slots are fixed broker-private filenames, never paths supplied by a caller.
type ArtifactSlot string

const (
	RecoverySlot     ArtifactSlot = "recovery"
	InstallerSlot    ArtifactSlot = "installer"
	MaxArtifactBytes int64        = 4 << 30
)

type ArtifactSpec struct {
	Size   int64
	SHA256 string
}
type AcquisitionPlan struct {
	PolicySHA256 string
	Version      string
	Commit       string
	Recovery     ArtifactSpec
	Installer    ArtifactSpec
}

func (p AcquisitionPlan) valid() bool {
	if !validHex(p.PolicySHA256, 64) || !validHex(p.Commit, 40) || len(p.Version) == 0 || len(p.Version) > 128 {
		return false
	}
	for _, c := range p.Version {
		if c < 33 || c > 126 {
			return false
		}
	}
	for _, s := range []ArtifactSpec{p.Recovery, p.Installer} {
		if s.Size <= 0 || s.Size > MaxArtifactBytes || !validHex(s.SHA256, 64) {
			return false
		}
	}
	return true
}
func (p AcquisitionPlan) spec(slot ArtifactSlot) (ArtifactSpec, error) {
	switch slot {
	case RecoverySlot:
		return p.Recovery, nil
	case InstallerSlot:
		return p.Installer, nil
	}
	return ArtifactSpec{}, errors.New("fixed Recovery or Installer acquisition slot required")
}

type acquisitionCandidateBackend interface {
	beginAcquisition() (acquisitionBackend, error)
}
type acquisitionBackend interface {
	bind(AcquisitionPlan) error
	create(ArtifactSlot, ArtifactSpec) (artifactBackend, error)
	close() error
}
type artifactBackend interface {
	Write([]byte) (int, error)
	Sync() error
	Path() string
	seal(ArtifactSpec) (*os.File, error)
	close() error
}

// Acquisition owns only disposable authenticated-download slots. Neither the
// plan nor a completed cleanup journal grants staging, adoption or launch authority.
// Cancellation is controlled by the caller's acquisition context; Close always
// attempts bounded cleanup independently of a cancelled download context.
type Acquisition struct {
	candidate *Candidate
	backend   acquisitionBackend
	plan      *AcquisitionPlan
	artifacts []*Artifact
	used      map[ArtifactSlot]bool
	closed    bool
	closeErr  error
}

// BeginAcquisition reserves one private lease before any payload writes. It is
// deliberately unsupported on a backend without native protected ownership.
func (c *Candidate) BeginAcquisition() (*Acquisition, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := c.ready(); err != nil {
		return nil, err
	}
	if c.acquisition != nil || len(c.files) != 0 {
		return nil, errors.New("one fresh acquisition lease required before payload writes")
	}
	backend, ok := c.backend.(acquisitionCandidateBackend)
	if !ok {
		return nil, ErrNativeRequired
	}
	b, err := backend.beginAcquisition()
	if err != nil {
		c.poisoned = true
		return nil, err
	}
	a := &Acquisition{candidate: c, backend: b, used: map[ArtifactSlot]bool{}}
	c.acquisition = a
	return a, nil
}
func (a *Acquisition) ready() error {
	if a.closed {
		return ErrClosed
	}
	return a.candidate.ready()
}
func (a *Acquisition) Bind(p AcquisitionPlan) error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c := a.candidate
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := a.ready(); err != nil {
		return err
	}
	if a.plan != nil || !p.valid() {
		return errors.New("one exact bounded acquisition plan required")
	}
	if err := a.backend.bind(p); err != nil {
		c.poisoned = true
		return err
	}
	a.plan = &p
	return nil
}
func (a *Acquisition) Create(slot ArtifactSlot) (*Artifact, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c := a.candidate
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := a.ready(); err != nil {
		return nil, err
	}
	if a.plan == nil || a.used[slot] {
		return nil, errors.New("bound unused artifact slot required")
	}
	spec, err := a.plan.spec(slot)
	if err != nil {
		return nil, err
	}
	a.used[slot] = true
	b, err := a.backend.create(slot, spec)
	if err != nil {
		c.poisoned = true
		return nil, err
	}
	f := &Artifact{acquisition: a, backend: b, spec: spec, slot: slot}
	a.artifacts = append(a.artifacts, f)
	return f, nil
}
func (a *Acquisition) Close() error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c := a.candidate
	c.mu.Lock()
	defer c.mu.Unlock()
	return a.closeLocked()
}
func (a *Acquisition) closeLocked() error {
	if a.closed {
		return a.closeErr
	}
	a.closed = true
	for _, f := range a.artifacts {
		a.closeErr = errors.Join(a.closeErr, f.closeLocked())
	}
	a.closeErr = errors.Join(a.closeErr, a.backend.close())
	if a.closeErr != nil {
		a.candidate.poisoned = true
	}
	return a.closeErr
}

// Artifact's returned os.File is borrowed, data-read-only and owned by Artifact.
// Never close it independently. Close deletes via its verified native handle,
// then closes custody and retains all cleanup errors. Path is diagnostic only.
type Artifact struct {
	slot           ArtifactSlot
	acquisition    *Acquisition
	backend        artifactBackend
	spec           ArtifactSpec
	written        int64
	sealed, closed bool
	closeErr       error
}

func (f *Artifact) Path() string { return f.backend.Path() }
func (f *Artifact) ready() error {
	if f.closed || f.sealed {
		return ErrClosed
	}
	return f.acquisition.ready()
}
func (f *Artifact) Write(p []byte) (int, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c := f.acquisition.candidate
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := f.ready(); err != nil {
		return 0, err
	}
	if int64(len(p)) > f.spec.Size-f.written {
		c.poisoned = true
		return 0, errors.New("artifact exceeds exact signed length")
	}
	n, err := f.backend.Write(p)
	f.written += int64(n)
	if err != nil {
		c.poisoned = true
	}
	return n, err
}
func (f *Artifact) Sync() error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c := f.acquisition.candidate
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := f.ready(); err != nil {
		return err
	}
	err := f.backend.Sync()
	if err != nil {
		c.poisoned = true
	}
	return err
}
func (f *Artifact) SealReadOnly() (*os.File, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c := f.acquisition.candidate
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := f.ready(); err != nil {
		return nil, err
	}
	if f.written != f.spec.Size {
		c.poisoned = true
		return nil, errors.New("artifact signed length incomplete")
	}
	file, err := f.backend.seal(f.spec)
	if err != nil {
		c.poisoned = true
		return nil, err
	}
	f.sealed = true
	return file, nil
}
func (f *Artifact) Close() error {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c := f.acquisition.candidate
	c.mu.Lock()
	defer c.mu.Unlock()
	return f.closeLocked()
}
func (f *Artifact) closeLocked() error {
	if f.closed {
		return f.closeErr
	}
	f.closed = true
	f.closeErr = f.backend.close()
	if f.closeErr != nil {
		f.acquisition.candidate.poisoned = true
	}
	return f.closeErr
}
