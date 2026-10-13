package main

import (
	"bytes"
	"encoding/json"
	"io"
	"strings"
	"testing"
)

func TestOperatorStructuredResultHasNoTrailingCompletionText(t *testing.T) {
	for _, action := range []string{"manual-rollback", "selected-history", "decommission-service", "uninstall-protected"} {
		var output bytes.Buffer
		if err := json.NewEncoder(&output).Encode(map[string]string{"state": action}); err != nil {
			t.Fatal(err)
		}
		if err := reportMaintenanceCompletion([]string{"--maintenance=" + action}, &output); err != nil {
			t.Fatal(err)
		}
		decoder := json.NewDecoder(&output)
		var result map[string]string
		if decoder.Decode(&result) != nil || result["state"] != action || decoder.Decode(&result) != io.EOF {
			t.Fatalf("structured result corrupted by trailing text: %q", output.String())
		}
	}
	var output bytes.Buffer
	if reportMaintenanceCompletion([]string{"--maintenance=quiesce-maintenance"}, &output) != nil || !strings.Contains(output.String(), "broker stop and original process exit verified") {
		t.Fatal("ordinary quiescence completion changed")
	}
}
