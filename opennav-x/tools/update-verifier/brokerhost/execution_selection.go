package brokerhost

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"sort"

	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
)

var errExecutionSelection = errors.New("protected updater selection is unavailable or uncertain")

// These records are authored only by the actual broker transaction. A generic
// journal label, client report, completed generation or baseline registration is
// never a selected version. The private schema is intentionally separate from
// generation creation lineage, which always names the original baseline.
type executionSelectionBegin struct {
	Schema             int    `json:"schema"`
	Purpose            string `json:"purpose"`
	ExecutionID        string `json:"executionId"`
	ClientSID          string `json:"clientSid"`
	Generation         string `json:"generation"`
	Version            string `json:"version"`
	Commit             string `json:"commit"`
	CompletionSHA256   string `json:"completionSha256"`
	PreviousGeneration string `json:"previousGeneration"`
	PreviousHeadSHA256 string `json:"previousHeadSha256"`
}

type executionSelectionStep struct {
	Schema         int    `json:"schema"`
	Purpose        string `json:"purpose"`
	ExecutionID    string `json:"executionId"`
	Step           string `json:"step"`
	Generation     string `json:"generation"`
	EvidenceSHA256 string `json:"evidenceSha256"`
}

type executionSelectionFact struct {
	kind     brokerstore.ExecutionFactKind
	original []byte
}

type executionSelectionAttempt struct {
	begin    executionSelectionBegin
	head     string
	terminal string
}

type executionSelectedVersion struct {
	generation, completion, head string
	installed                    releasepolicy.Installed
}

type executionPendingSelection struct {
	executionID, generation, previousGeneration string
}

// This is a reducer of already retained protected original facts, not an import
// or authentication API. Its native caller must hold every original journal and
// revalidate the selected generation, registration and history across effects.
func parseExecutionSelectionAttempt(sid, executionID, generation, completion, generationPurpose, journalDigest string, facts []executionSelectionFact) (executionSelectionAttempt, error) {
	var result executionSelectionAttempt
	refuse := func() (executionSelectionAttempt, error) { return executionSelectionAttempt{}, errExecutionSelection }
	if !brokerregistration.IsCanonicalUserSID(sid) || !requestPattern.MatchString(executionID) || !requestPattern.MatchString(generation) || !hashPattern.MatchString(completion) || !hashPattern.MatchString(journalDigest) || len(facts) == 0 || len(facts) > 64 {
		return refuse()
	}
	if facts[0].kind != brokerstore.ExecutionPreparation || exactExecutionRecord(facts[0].original, &result.begin) != nil {
		return refuse()
	}
	b := result.begin
	if b.Schema != 1 || b.Purpose != "broker-selected-transaction-v1" || b.ExecutionID != executionID || b.ClientSID != sid || b.Generation != generation || b.CompletionSHA256 != completion || !commitPattern.MatchString(b.Commit) {
		return refuse()
	}
	if _, err := releasepolicy.ParseSemVer(b.Version); err != nil || len(b.Version) > 128 {
		return refuse()
	}
	root := b.PreviousGeneration == "" && b.PreviousHeadSHA256 == ""
	if root {
		if generationPurpose != string(brokerstore.GenerationPurposeApprovedBaseline) {
			return refuse()
		}
	} else if generationPurpose != string(brokerstore.GenerationPurposeSignedCandidate) || !requestPattern.MatchString(b.PreviousGeneration) || b.PreviousGeneration == generation || !hashPattern.MatchString(b.PreviousHeadSHA256) {
		return refuse()
	}
	// A missing terminal is pending, even if every prefix fact is valid. It
	// cannot be dropped merely because an SCM restart lost its process owner.
	state := "begin"
	for _, fact := range facts[1:] {
		var step executionSelectionStep
		if exactExecutionRecord(fact.original, &step) != nil || step.Schema != 1 || step.Purpose != "broker-selected-step-v1" || step.ExecutionID != executionID || !hashPattern.MatchString(step.EvidenceSHA256) {
			return refuse()
		}
		target := generation
		expectedKind := brokerstore.ExecutionCommissioning
		allowed := false
		switch step.Step {
		case "commissioned":
			allowed = state == "begin"
		case "spawn-consumed":
			expectedKind = brokerstore.ExecutionSpawnIntent
			allowed = state == "commissioned"
		case "healthy":
			expectedKind = brokerstore.ExecutionHealthObserved
			allowed = state == "spawn-consumed"
		case "selected":
			expectedKind = brokerstore.ExecutionHealthObserved
			allowed = state == "healthy"
		case "candidate-closed":
			expectedKind = brokerstore.ExecutionTeardown
			allowed = !root && (state == "spawn-consumed" || state == "healthy")
		case "profile-restored":
			expectedKind = brokerstore.ExecutionRestoration
			allowed = state == "candidate-closed"
		case "fallback-commissioned":
			target = b.PreviousGeneration
			allowed = state == "profile-restored"
		case "fallback-spawn-consumed":
			target = b.PreviousGeneration
			expectedKind = brokerstore.ExecutionSpawnIntent
			allowed = state == "fallback-commissioned"
		case "fallback-healthy":
			target = b.PreviousGeneration
			expectedKind = brokerstore.ExecutionHealthObserved
			allowed = state == "fallback-spawn-consumed"
		case "rolled-back":
			target = b.PreviousGeneration
			expectedKind = brokerstore.ExecutionRestoration
			allowed = state == "fallback-healthy"
		}
		if !allowed || step.Generation != target || fact.kind != expectedKind {
			return refuse()
		}
		state = step.Step
	}
	if state == "selected" || state == "rolled-back" {
		result.terminal = state
	}
	result.head = journalDigest
	return result, nil
}

func exactExecutionRecord(raw []byte, target any) error {
	if len(raw) == 0 || len(raw) > 64<<10 {
		return errExecutionSelection
	}
	if err := json.Unmarshal(raw, target); err != nil {
		return errExecutionSelection
	}
	canonical, err := json.Marshal(target)
	if err != nil || !bytes.Equal(canonical, raw) {
		return errExecutionSelection
	}
	return nil
}

func executionOriginalDigest(raw []byte) string {
	h := sha256.Sum256(raw)
	return hex.EncodeToString(h[:])
}

// foldExecutionSelection requires a single authenticated baseline root and a
// nonforking sequence of terminal transactions. Failed/unfinished transactions
// stay visible as pending; a previously healthy generation does not make a
// pending attempt safe to ignore. Whole-disk rollback by a trusted administrator
// is outside this local history's guarantee.
func foldExecutionSelection(attempts []executionSelectionAttempt, baselineGeneration string) (executionSelectedVersion, []executionPendingSelection, error) {
	var selected executionSelectedVersion
	var pending []executionPendingSelection
	if !requestPattern.MatchString(baselineGeneration) || len(attempts) > 128 {
		return selected, nil, errExecutionSelection
	}
	byParent := map[string]executionSelectionAttempt{}
	heads := map[string]bool{}
	ids := map[string]bool{}
	for _, a := range attempts {
		if !requestPattern.MatchString(a.begin.ExecutionID) || !requestPattern.MatchString(a.begin.Generation) || !hashPattern.MatchString(a.head) || heads[a.head] || ids[a.begin.ExecutionID] {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		heads[a.head], ids[a.begin.ExecutionID] = true, true
		if a.terminal == "" {
			pending = append(pending, executionPendingSelection{a.begin.ExecutionID, a.begin.Generation, a.begin.PreviousGeneration})
			continue
		}
		parent := a.begin.PreviousHeadSHA256
		if _, exists := byParent[parent]; exists {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		if parent == "" && (a.begin.Generation != baselineGeneration || a.begin.PreviousGeneration != "" || a.terminal != "selected") {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		byParent[parent] = a
	}
	current := ""
	consumed := 0
	for {
		next, exists := byParent[current]
		if !exists {
			break
		}
		if consumed >= 128 {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		if current != "" && next.begin.PreviousGeneration != selected.generation {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		if next.terminal == "selected" {
			selected = executionSelectedVersion{next.begin.Generation, next.begin.CompletionSHA256, next.head, releasepolicy.Installed{Version: next.begin.Version, Commit: next.begin.Commit}}
		} else if next.terminal == "rolled-back" && current != "" {
			selected.head = next.head
		} else {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		current = next.head
		consumed++
	}
	if consumed != len(byParent) {
		return executionSelectedVersion{}, nil, errExecutionSelection
	}
	for _, a := range attempts {
		if a.terminal == "" && a.begin.PreviousHeadSHA256 != "" && !heads[a.begin.PreviousHeadSHA256] {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
	}
	if selected.generation == "" || len(pending) != 0 {
		return selected, pending, fmt.Errorf("%w: qualification or recovery required", errExecutionSelection)
	}
	return selected, nil, nil
}

// Bind the complete client namespace, not just readers captured earlier. A
// newly reserved/partial/forked attempt must invalidate an older selection.
// This is an informational binding used under engine.busy, not a disk lock.
func executionSelectionInventoryDigest(sid string, entries []brokerstore.ExecutionJournalEntry) (string, error) {
	if !brokerregistration.IsCanonicalUserSID(sid) || len(entries) > 128 {
		return "", errExecutionSelection
	}
	type item struct {
		ID    string
		Names []string
	}
	copy := make([]item, 0, len(entries))
	seen := map[string]bool{}
	for _, e := range entries {
		if !e.Readable || e.Problem != "" || !requestPattern.MatchString(e.ExecutionID) || seen[e.ExecutionID] || len(e.EntryNames) != 2 {
			return "", errExecutionSelection
		}
		seen[e.ExecutionID] = true
		names := append([]string(nil), e.EntryNames...)
		sort.Strings(names)
		prefix := sid + ".execution." + e.ExecutionID
		if names[0] != prefix+".journal" || names[1] != prefix+".once" {
			return "", errExecutionSelection
		}
		copy = append(copy, item{e.ExecutionID, names})
	}
	sort.Slice(copy, func(i, j int) bool { return copy[i].ID < copy[j].ID })
	raw, err := json.Marshal(copy)
	if err != nil {
		return "", errExecutionSelection
	}
	return executionOriginalDigest(raw), nil
}
