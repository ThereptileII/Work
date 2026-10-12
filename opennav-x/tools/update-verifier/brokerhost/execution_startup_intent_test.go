package brokerhost

import "testing"

func TestExecutionStartupAdmissionCannotBecomeUpdate(t *testing.T) {
	for _, operation := range []string{"start-selected", "start-current", "execute-session", "", "START-SELECTED"} {
		for _, initial := range []bool{true, false} {
			want := operation == "execute-session" || operation == "start-selected" && initial
			if got := executionQualificationIntentAllowed(operation, initial); got != want {
				t.Fatalf("operation %q empty history %v: admitted %v, want %v", operation, initial, got, want)
			}
		}
	}
}
