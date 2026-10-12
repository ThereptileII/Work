package brokerstore

import (
	"bytes"
	"context"
	"errors"
	"os"
	"runtime"
	"strings"
	"testing"
)

// Uses only fresh temporary native fixtures. No installed paths, SCM, child
// execution, hardware, selection or commissioning are involved.
func nativeHistoryCandidate(t *testing.T, s *Store, p *InstalledBaseline, f baselineFixtureData, stage string) *Candidate {
	t.Helper()
	c, err := s.Begin(stage, p.record.ClientSID)
	if err != nil {
		t.Fatal(err)
	}
	acquireGenerationArtifacts(t, c, f)
	return c
}
func nativeHistoryOwner(t *testing.T, owner *ReadOnlyGeneration) *nativeInstalledGeneration {
	t.Helper()
	b, ok := owner.generation.backend.(interface {
		nativeHistoryGeneration() *nativeInstalledGeneration
	})
	if !ok {
		t.Fatal("native fixture backend required")
	}
	return b.nativeHistoryGeneration()
}

func TestNativeGenerationHistoryRetainedOwnersSurviveMixedPublications(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, order := range []string{"baseline-first", "candidate-first"} {
		t.Run(order, func(t *testing.T) {
			s, p, _, f, raw := nativeBaselineViewFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			var first *ReadOnlyGeneration
			var err error
			if order == "baseline-first" {
				var baseline *ReadOnlyApprovedBaselineGeneration
				baseline, err = s.MaterializeApprovedBaselineReadOnlyFresh(ctx, p, raw, registrationDigest(raw))
				if err != nil {
					t.Fatal(err)
				}
				first = baseline.owner
			} else {
				first, err = s.MaterializeCandidateReadOnlyFresh(ctx, p, nativeHistoryCandidate(t, s, p, f, strings.Repeat("9", 32)))
				if err != nil {
					t.Fatal(err)
				}
			}
			before := bytes.Clone(first.generation.raw)
			n := nativeHistoryOwner(t, first)
			beforeParents := n.parentIDs
			member, err := first.OpenMember("app/opencpn.exe")
			if err != nil {
				t.Fatal(err)
			}
			// Retained creationContext is now expired. History evolution must use
			// the new operation's context while retaining original file custody.
			cancel()
			if n.creationContext.Err() == nil {
				t.Fatal("fixture creation context did not expire")
			}
			var second *ReadOnlyGeneration
			if order == "baseline-first" {
				second, err = s.MaterializeCandidateReadOnlyFresh(context.Background(), p, nativeHistoryCandidate(t, s, p, f, strings.Repeat("9", 32)))
			} else {
				var baseline *ReadOnlyApprovedBaselineGeneration
				baseline, err = s.MaterializeApprovedBaselineReadOnlyFresh(context.Background(), p, raw, registrationDigest(raw))
				if baseline != nil {
					second = baseline.owner
				}
			}
			if err != nil {
				t.Fatal("current append context was replaced by expired creation context", err)
			}
			third, err := s.MaterializeCandidateReadOnlyFresh(context.Background(), p, nativeHistoryCandidate(t, s, p, f, strings.Repeat("8", 32)))
			if err != nil {
				t.Fatal(err)
			}
			for _, owner := range []*ReadOnlyGeneration{first, second, third} {
				if err := owner.Validate(); err != nil {
					t.Fatal("live exact sibling history refused", err)
				}
				current := nativeHistoryOwner(t, owner)
				if len(current.siblings) != 2 || !sameGenerationIDs(beforeParents, current.parentIDs) {
					t.Fatal("history or parent full IDs changed")
				}
			}
			if !bytes.Equal(before, first.generation.raw) {
				t.Fatal("append rewrote original completion")
			}
			if err := third.Close(); err != nil {
				t.Fatal(err)
			}
			if err := second.Close(); err != nil {
				t.Fatal(err)
			}
			got := make([]byte, len(f.payload["app/opencpn.exe"]))
			if _, err := member.ReadAt(got, 0); err != nil || !bytes.Equal(got, f.payload["app/opencpn.exe"]) {
				t.Fatal("successor Close revoked retained original", err)
			}
			if err := first.Validate(); err != nil {
				t.Fatal("successor Close lost independent history custody", err)
			}
			if err := member.Close(); err != nil {
				t.Fatal(err)
			}
			if err := first.Close(); err != nil {
				t.Fatal(err)
			}
		})
	}
}

func TestNativeGenerationHistoryRefusesUnknownNamespaceAndWriterInterleave(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	for _, scenario := range []string{"private-extra", "public-extra", "writer-candidate", "writer-baseline"} {
		t.Run(scenario, func(t *testing.T) {
			s, p, _, f, raw := nativeBaselineViewFixture(t)
			first, err := s.MaterializeCandidateReadOnlyFresh(context.Background(), p, nativeHistoryCandidate(t, s, p, f, strings.Repeat("9", 32)))
			if err != nil {
				t.Fatal(err)
			}
			n := nativeHistoryOwner(t, first)
			c := nativeHistoryCandidate(t, s, p, f, strings.Repeat("8", 32))
			switch scenario {
			case "private-extra":
				extra, e := baselineFile(n.registry, "unknown.json", n.privateSD, true)
				if e != nil {
					t.Fatal(e)
				}
				if e = extra.Close(); e != nil {
					t.Fatal(e)
				}
			case "public-extra":
				name := physical(n.generations.name + `\` + strings.Repeat("7", 32))
				if e := os.Mkdir(name, 0700); e != nil {
					t.Fatal(e)
				}
			}
			switch scenario {
			case "writer-candidate":
				_, err = s.MaterializeCandidateFresh(context.Background(), p, c)
			case "writer-baseline":
				_, err = s.MaterializeApprovedBaselineFresh(context.Background(), p, raw, registrationDigest(raw))
			default:
				_, err = s.MaterializeCandidateReadOnlyFresh(context.Background(), p, c)
			}
			if err == nil || c.materialized {
				t.Fatal("unknown namespace/writer interleave consumed source", err)
			}
			path := physical(n.registry.name + `\` + c.stageID + ".once")
			if _, err := os.Stat(path); !errors.Is(err, os.ErrNotExist) {
				t.Fatal("refusal created new reservation", err)
			}
			if strings.HasPrefix(scenario, "writer-") {
				if !errors.Is(err, ErrGenerationHistoryRequiresReadOnly) {
					t.Fatal(err)
				}
				if err := first.Validate(); err != nil {
					t.Fatal("writer refusal poisoned existing owner", err)
				}
			} else if first.Validate() == nil {
				t.Fatal("unknown namespace accepted")
			}
		})
	}
}

// Cancel on observing an actual newly created permanent reservation, rather
// than a fabricated metadata value or a count of context checks.
type nativeHistoryReservedContext struct {
	context.Context
	path string
}

func (c nativeHistoryReservedContext) Err() error {
	if _, err := os.Stat(c.path); err == nil {
		return context.Canceled
	}
	return c.Context.Err()
}
func TestNativeGenerationHistoryCancelledSuccessorBlocksAllRetainedOwners(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	s, p, _, f, raw := nativeBaselineViewFixture(t)
	baseline, err := s.MaterializeApprovedBaselineReadOnlyFresh(context.Background(), p, raw, registrationDigest(raw))
	if err != nil {
		t.Fatal(err)
	}
	second, err := s.MaterializeCandidateReadOnlyFresh(context.Background(), p, nativeHistoryCandidate(t, s, p, f, strings.Repeat("9", 32)))
	if err != nil {
		t.Fatal(err)
	}
	member, err := baseline.OpenMember("app/opencpn.exe")
	if err != nil {
		t.Fatal(err)
	}
	n := nativeHistoryOwner(t, baseline.owner)
	before := bytes.Clone(baseline.owner.generation.raw)
	c := nativeHistoryCandidate(t, s, p, f, strings.Repeat("8", 32))
	marker := physical(n.registry.name + `\` + c.stageID + ".once")
	owner, err := s.MaterializeCandidateReadOnlyFresh(nativeHistoryReservedContext{context.Background(), marker}, p, c)
	if owner != nil || !errors.Is(err, context.Canceled) || !errors.Is(err, ErrInstalledGenerationUncertain) {
		t.Fatal("actual reservation cancellation lost uncertainty", err)
	}
	if _, err := os.Stat(marker); err != nil {
		t.Fatal("permanent failed reservation removed", err)
	}
	if !errors.Is(baseline.Validate(), ErrPoisoned) || !errors.Is(second.Validate(), ErrPoisoned) {
		t.Fatal("uncertain namespace left live history usable")
	}
	if _, err := member.ReadAt(make([]byte, 1), 0); !errors.Is(err, ErrPoisoned) {
		t.Fatal("uncertain history left member usable", err)
	}
	actual, err := readBaselineFile(context.Background(), n.record, int64(len(before)))
	if err != nil || !bytes.Equal(actual, before) {
		t.Fatal("failed successor erased or rewrote predecessor completion", err)
	}
	if _, err := s.MaterializeCandidateReadOnlyFresh(context.Background(), p, c); err == nil {
		t.Fatal("uncertain mutation retried")
	}
}
