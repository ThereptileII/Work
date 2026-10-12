package brokerstore

import (
	"bytes"
	"encoding/json"
	"errors"
	"io"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func acquisitionPlan() AcquisitionPlan {
	return AcquisitionPlan{PolicySHA256: digest("policy"), Version: "1.2.3", Commit: strings.Repeat("a", 40), Recovery: ArtifactSpec{Size: 3, SHA256: digest("zip")}, Installer: ArtifactSpec{Size: 3, SHA256: digest("exe")}}
}
func journalEvent(sequence int, state string, slot ArtifactSlot, id *diskIdentity, plan *AcquisitionPlan) acquisitionEvent {
	return acquisitionEvent{Schema: 1, Sequence: sequence, State: state, StageID: stageA, ClientSID: "S-1-5-21-1-2-3-1001", Directory: diskIdentity{Volume: 1, FileID: strings.Repeat("a", 32), Created: 1}, Parent: diskIdentity{Volume: 1, FileID: strings.Repeat("d", 32), Created: 1}, Slot: slot, Identity: id, Plan: plan}
}
func encodeEvents(events ...acquisitionEvent) []byte {
	var data []byte
	for _, e := range events {
		line, _ := json.Marshal(e)
		data = append(data, line...)
		data = append(data, '\n')
	}
	return data
}
func TestAcquisitionJournalCrashStatesAndStrictBindings(t *testing.T) {
	p := acquisitionPlan()
	id := diskIdentity{Volume: 1, FileID: strings.Repeat("b", 32), Created: 2}
	events := []acquisitionEvent{journalEvent(1, "lease", "", nil, nil), journalEvent(2, "bound", "", nil, &p), journalEvent(3, "creating", RecoverySlot, nil, nil), journalEvent(4, "owned", RecoverySlot, &id, nil), journalEvent(5, "deleting", RecoverySlot, &id, nil), journalEvent(6, "deleted", RecoverySlot, &id, nil), journalEvent(7, "complete", "", nil, nil)}
	for n := 1; n <= len(events); n++ {
		j, err := parseAcquisitionIndex(encodeEvents(events[:n]...))
		if err != nil {
			t.Fatalf("valid crash prefix %d: %v", n, err)
		}
		if j.complete != (n == 7) {
			t.Fatalf("prefix %d completion changed", n)
		}
	}
	for _, tc := range []struct {
		name   string
		mutate func([]acquisitionEvent) []acquisitionEvent
	}{
		{"skip-owned", func(e []acquisitionEvent) []acquisitionEvent { e[3].State = "deleting"; return e }},
		{"identity-change", func(e []acquisitionEvent) []acquisitionEvent {
			other := id
			other.FileID = strings.Repeat("c", 32)
			e[4].Identity = &other
			return e
		}},
		{"stage-change", func(e []acquisitionEvent) []acquisitionEvent { e[3].StageID = strings.Repeat("b", 32); return e }},
		{"client-change", func(e []acquisitionEvent) []acquisitionEvent { e[3].ClientSID = "other"; return e }},
		{"parent-change", func(e []acquisitionEvent) []acquisitionEvent { e[3].Parent.Created++; return e }},
		{"directory-change", func(e []acquisitionEvent) []acquisitionEvent { e[3].Directory.Created++; return e }},
		{"repeat-plan", func(e []acquisitionEvent) []acquisitionEvent { e[2].Plan = &p; return e }},
		{"arbitrary-slot", func(e []acquisitionEvent) []acquisitionEvent { e[2].Slot = "../../victim"; return e }},
		{"premature-complete", func(e []acquisitionEvent) []acquisitionEvent {
			e[4] = journalEvent(5, "complete", "", nil, nil)
			return e[:5]
		}},
		{"reuse-after-complete", func(e []acquisitionEvent) []acquisitionEvent {
			return append(e, journalEvent(8, "creating", RecoverySlot, nil, nil))
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			copyEvents := append([]acquisitionEvent{}, events...)
			if _, err := parseAcquisitionIndex(encodeEvents(tc.mutate(copyEvents)...)); err == nil {
				t.Fatal("invalid durable transition accepted")
			}
		})
	}
	valid := encodeEvents(events...)
	for _, tc := range []struct {
		name string
		data []byte
	}{
		{"first-complete", encodeEvents(journalEvent(1, "complete", "", nil, nil))},
		{"truncated", valid[:len(valid)-1]},
		{"unknown-field", bytes.Replace(valid, []byte(`"Schema":1`), []byte(`"Unknown":0,"Schema":1`), 1)},
		{"duplicate-field", bytes.Replace(valid, []byte(`"Schema":1`), []byte(`"Schema":1,"Schema":1`), 1)},
		{"trailing-json", append(append([]byte{}, valid...), []byte("{}\n")...)},
		{"oversized", bytes.Repeat([]byte{' '}, maxAcquisitionJournal+1)},
		{"invalid-utf8", bytes.Replace(valid, []byte("S-1-5-21"), []byte{255}, 1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if _, err := parseAcquisitionIndex(tc.data); err == nil {
				t.Fatal("uncertain journal accepted")
			}
		})
	}
}

// Portable owner fixtures exercise the common lease/drain/error contract only;
// they do not emulate native ACL, sharing, identity or handle-disposition proof.
type acquisitionCandidateFixture struct {
	memoryCandidate
	lease *acquisitionFixture
}

func (c *acquisitionCandidateFixture) beginAcquisition() (acquisitionBackend, error) {
	return c.lease, nil
}

type acquisitionFixture struct {
	root      string
	binds     int
	creates   int
	closes    int
	fault     error
	artifacts []*artifactFixture
}

func (a *acquisitionFixture) bind(AcquisitionPlan) error { a.binds++; return a.fault }
func (a *acquisitionFixture) create(slot ArtifactSlot, _ ArtifactSpec) (artifactBackend, error) {
	a.creates++
	if a.fault != nil {
		return nil, a.fault
	}
	file, err := os.Create(filepath.Join(a.root, slotFilename(slot)))
	if err != nil {
		return nil, err
	}
	f := &artifactFixture{File: file, path: file.Name()}
	a.artifacts = append(a.artifacts, f)
	return f, nil
}
func (a *acquisitionFixture) close() error { a.closes++; return a.fault }

type artifactFixture struct {
	*os.File
	path   string
	closes int
	fault  error
}

func (f *artifactFixture) Path() string { return f.path }
func (f *artifactFixture) seal(ArtifactSpec) (*os.File, error) {
	_, err := f.Seek(0, io.SeekStart)
	return f.File, err
}
func (f *artifactFixture) close() error {
	f.closes++
	return errors.Join(f.File.Close(), os.Remove(f.path), f.fault)
}
func acquisitionFixtureCandidate(t *testing.T) (*Store, *Candidate, *acquisitionFixture) {
	t.Helper()
	s, _ := testStore()
	a := &acquisitionFixture{root: t.TempDir()}
	c := &Candidate{store: s, backend: &acquisitionCandidateFixture{lease: a}, names: map[string]string{}, fileNames: map[string]bool{}}
	s.candidates = append(s.candidates, c)
	t.Cleanup(func() { s.Close() })
	return s, c, a
}
func TestAcquisitionFixedSlotsBoundsAndOwnedDrain(t *testing.T) {
	s, c, b := acquisitionFixtureCandidate(t)
	a, err := c.BeginAcquisition()
	if err != nil {
		t.Fatal(err)
	}
	if _, err = c.BeginAcquisition(); err == nil {
		t.Fatal("lease reuse accepted")
	}
	if _, err = a.Create(RecoverySlot); err == nil {
		t.Fatal("unbound create accepted")
	}
	p := acquisitionPlan()
	bad := p
	bad.Recovery.Size = MaxArtifactBytes + 1
	if err = a.Bind(bad); err == nil {
		t.Fatal("oversized signed artifact accepted")
	}
	if err = a.Bind(p); err != nil {
		t.Fatal(err)
	}
	if err = a.Bind(p); err == nil {
		t.Fatal("plan rebound")
	}
	if _, err = a.Create(ArtifactSlot("caller/path")); err == nil {
		t.Fatal("arbitrary slot accepted")
	}
	f, err := a.Create(RecoverySlot)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = a.Create(RecoverySlot); err == nil {
		t.Fatal("slot reused")
	}
	if _, err = f.Write([]byte("zip")); err != nil {
		t.Fatal(err)
	}
	if err = f.Sync(); err != nil {
		t.Fatal(err)
	}
	borrowed, err := f.SealReadOnly()
	if err != nil {
		t.Fatal(err)
	}
	data, err := io.ReadAll(borrowed)
	if err != nil || string(data) != "zip" {
		t.Fatalf("borrowed bytes %q %v", data, err)
	}
	if err = s.Close(); err != nil {
		t.Fatal(err)
	}
	if err = f.Close(); err != nil {
		t.Fatal(err)
	}
	if err = a.Close(); err != nil {
		t.Fatal(err)
	}
	if b.binds != 1 || b.creates != 1 || b.closes != 1 || b.artifacts[0].closes != 1 {
		t.Fatal("ownership drain repeated or skipped")
	}
	if _, err = os.Stat(f.Path()); !errors.Is(err, os.ErrNotExist) {
		t.Fatal("owned fixture artifact not cleaned")
	}
}
func TestAcquisitionCleanupUncertaintyRetainedAcrossOwners(t *testing.T) {
	s, c, b := acquisitionFixtureCandidate(t)
	a, err := c.BeginAcquisition()
	if err != nil {
		t.Fatal(err)
	}
	if err = a.Bind(acquisitionPlan()); err != nil {
		t.Fatal(err)
	}
	f, err := a.Create(InstallerSlot)
	if err != nil {
		t.Fatal(err)
	}
	b.artifacts[0].fault = testFault
	first := f.Close()
	if !errors.Is(first, testFault) {
		t.Fatal(first)
	}
	b.artifacts[0].fault = nil
	if f.Close() != first {
		t.Fatal("artifact cleanup error forgotten")
	}
	leaseClose := a.Close()
	if !errors.Is(leaseClose, testFault) || a.Close() != leaseClose {
		t.Fatal("lease cleanup error forgotten")
	}
	if err = s.Close(); !errors.Is(err, testFault) {
		t.Fatal("store lost acquisition uncertainty", err)
	}
	if b.artifacts[0].closes != 1 || b.closes != 1 {
		t.Fatal("failed cleanup retried")
	}
}
func TestAcquisitionIncompleteAndExcessWritesPoisonCandidate(t *testing.T) {
	for _, excess := range []bool{false, true} {
		t.Run(map[bool]string{false: "incomplete", true: "excess"}[excess], func(t *testing.T) {
			_, c, _ := acquisitionFixtureCandidate(t)
			a, _ := c.BeginAcquisition()
			if err := a.Bind(acquisitionPlan()); err != nil {
				t.Fatal(err)
			}
			f, err := a.Create(RecoverySlot)
			if err != nil {
				t.Fatal(err)
			}
			if excess {
				_, err = f.Write([]byte("too long"))
			} else {
				_, err = f.SealReadOnly()
			}
			if err == nil || !c.poisoned {
				t.Fatal("invalid content did not poison")
			}
			if err = a.Close(); err != nil {
				t.Fatal(err)
			}
		})
	}
}
