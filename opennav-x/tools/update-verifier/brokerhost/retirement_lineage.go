package brokerhost

import (
	"sort"

	"example.com/opennav-update-verifier/brokerstore"
)

// retirementLineage reduces only the already authenticated original journal
// attempts retained by nativeSelectedExecution. It is comparison data for the
// Store's independent exact-set/identity checks, never authority from a reported
// generation list or a completed candidate directory alone.
func retirementLineage(attempts []executionSelectionAttempt, baseline string, expected executionSelectedVersion) ([]brokerstore.RetirementGeneration, error) {
	selected, pending, err := foldExecutionSelection(attempts, baseline)
	if err != nil || len(pending) != 0 || selected != expected {
		return nil, errDecommissionRefused
	}
	completions := make(map[string]string, len(attempts))
	for _, attempt := range attempts {
		generation, completion := attempt.begin.Generation, attempt.begin.CompletionSHA256
		if !requestPattern.MatchString(generation) || !hashPattern.MatchString(completion) {
			return nil, errDecommissionRefused
		}
		if prior, exists := completions[generation]; exists && prior != completion {
			return nil, errDecommissionRefused
		}
		// Include resolved failed candidates as well as selected ancestors. The
		// reducer above must have proved their rollback or no-spawn recovery;
		// omitting their retained roots would make installed uninstall incomplete.
		completions[generation] = completion
	}
	if !hashPattern.MatchString(completions[baseline]) || completions[selected.generation] != selected.completion {
		return nil, errDecommissionRefused
	}
	ids := make([]string, 0, len(completions)-1)
	for generation := range completions {
		if generation != baseline {
			ids = append(ids, generation)
		}
	}
	sort.Strings(ids)
	lineage := make([]brokerstore.RetirementGeneration, 0, len(completions))
	lineage = append(lineage, brokerstore.RetirementGeneration{GenerationID: baseline, CompletionSHA256: completions[baseline]})
	for _, generation := range ids {
		lineage = append(lineage, brokerstore.RetirementGeneration{GenerationID: generation, CompletionSHA256: completions[generation]})
	}
	return lineage, nil
}
