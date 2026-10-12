package brokerhost

import (
	"context"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

// These refusals must happen before any store operation: the private portable
// seams cannot substitute for the actual retained native source owners.
func TestNativeMaterializationAdapterRequiresRetainedOwners(t *testing.T) {
	f := newFixture()
	var missingBaseline *brokerstore.InstalledBaseline
	var missingCandidate *brokerstore.Candidate
	for name, input := range map[string]struct {
		baseline baselineOwner
		stage    candidate
	}{
		"substituted baseline":  {fakeBaseline{f}, fakeStage{f}},
		"nil native baseline":   {missingBaseline, fakeStage{f}},
		"substituted candidate": {&brokerstore.InstalledBaseline{}, fakeStage{f}},
		"nil native candidate":  {&brokerstore.InstalledBaseline{}, missingCandidate},
	} {
		t.Run(name, func(t *testing.T) {
			owner, err := (storeAdapter{}).materialize(context.Background(), input.baseline, input.stage)
			if owner != nil || err == nil || len(f.events) != 0 {
				t.Fatal("substituted source reached materialization")
			}
		})
	}
}
