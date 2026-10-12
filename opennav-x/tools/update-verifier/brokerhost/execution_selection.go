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

// A recovery is a fresh journal, never an append reopening. Its prior digest
// names the complete retained original chain. Native host code must independently
// establish cold/no-spawn state and restore the retained profile before authoring
// these facts; neither this schema nor client reports confer recovery authority.
type executionSelectionRecovery struct {
	Schema             int    `json:"schema"`
	Purpose            string `json:"purpose"`
	ExecutionID        string `json:"executionId"`
	ClientSID          string `json:"clientSid"`
	Generation         string `json:"generation"`
	CompletionSHA256   string `json:"completionSha256"`
	PreviousGeneration string `json:"previousGeneration"`
	PreviousHeadSHA256 string `json:"previousHeadSha256"`
	PriorExecutionID   string `json:"priorExecutionId"`
	PriorJournalSHA256 string `json:"priorJournalSha256"`
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
	state    string
	consumed bool
	recovery *executionSelectionRecovery
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
	if facts[0].kind != brokerstore.ExecutionPreparation {
		return refuse()
	}
	// Exact re-encoding separately closes each purpose; a transaction cannot
	// smuggle recovery fields and a recovery cannot smuggle healthy/version data.
	var purpose struct {
		Purpose string `json:"purpose"`
	}
	if json.Unmarshal(facts[0].original, &purpose) != nil {
		return refuse()
	}
	if purpose.Purpose == "broker-selected-recovery-v1" {
		return parseExecutionSelectionRecovery(sid, executionID, generation, completion, generationPurpose, journalDigest, facts)
	}
	if exactExecutionRecord(facts[0].original, &result.begin) != nil {
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
		if step.Step == "spawn-consumed" || step.Step == "fallback-spawn-consumed" {
			result.consumed = true
		}
	}
	result.state = state
	if state == "selected" || state == "rolled-back" {
		result.terminal = state
	}
	result.head = journalDigest
	return result, nil
}

func parseExecutionSelectionRecovery(sid, executionID, generation, completion, generationPurpose, journalDigest string, facts []executionSelectionFact) (executionSelectionAttempt, error) {
	var recovery executionSelectionRecovery
	if exactExecutionRecord(facts[0].original, &recovery) != nil || recovery.Schema != 1 || recovery.Purpose != "broker-selected-recovery-v1" ||
		recovery.ExecutionID != executionID || recovery.ClientSID != sid || recovery.Generation != generation || recovery.CompletionSHA256 != completion ||
		!requestPattern.MatchString(recovery.PriorExecutionID) || recovery.PriorExecutionID == executionID || !hashPattern.MatchString(recovery.PriorJournalSHA256) {
		return executionSelectionAttempt{}, errExecutionSelection
	}
	root := recovery.PreviousGeneration == "" && recovery.PreviousHeadSHA256 == ""
	if root {
		if generationPurpose != string(brokerstore.GenerationPurposeApprovedBaseline) {
			return executionSelectionAttempt{}, errExecutionSelection
		}
	} else if generationPurpose != string(brokerstore.GenerationPurposeSignedCandidate) || !requestPattern.MatchString(recovery.PreviousGeneration) ||
		recovery.PreviousGeneration == generation || !hashPattern.MatchString(recovery.PreviousHeadSHA256) {
		return executionSelectionAttempt{}, errExecutionSelection
	}
	result := executionSelectionAttempt{begin: executionSelectionBegin{
		Schema: 1, Purpose: recovery.Purpose, ExecutionID: executionID, ClientSID: sid, Generation: generation,
		CompletionSHA256: completion, PreviousGeneration: recovery.PreviousGeneration, PreviousHeadSHA256: recovery.PreviousHeadSHA256},
		head: journalDigest, state: "recovery-begin", recovery: &recovery}
	expected := []struct {
		step string
		kind brokerstore.ExecutionFactKind
	}{
		{"cold-no-spawn", brokerstore.ExecutionTeardown},
		{"profile-restored", brokerstore.ExecutionRestoration},
		{"restored-no-spawn", brokerstore.ExecutionRestoration},
	}
	if len(facts) > len(expected)+1 {
		return executionSelectionAttempt{}, errExecutionSelection
	}
	for i, fact := range facts[1:] {
		var step executionSelectionStep
		if exactExecutionRecord(fact.original, &step) != nil || step.Schema != 1 || step.Purpose != "broker-selected-recovery-step-v1" ||
			step.ExecutionID != executionID || step.Generation != generation || !hashPattern.MatchString(step.EvidenceSHA256) ||
			step.Step != expected[i].step || fact.kind != expected[i].kind {
			return executionSelectionAttempt{}, errExecutionSelection
		}
		result.state = step.Step
	}
	if result.state == "restored-no-spawn" {
		result.terminal = result.state
	}
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
// nonforking sequence of terminal transactions. Unfinished transactions stay
// pending unless a fresh exact linked recovery independently proves their
// pre-spawn cold/profile-restored boundary. This reducer validates only the
// retained record linkage: actual native proof must precede record authorship.
// A healthy predecessor alone never resolves a pending attempt, and consumed
// spawn uncertainty still requires the real close/restore/fallback-health path.
// Whole-disk rollback by a trusted administrator
// is outside this local history's guarantee.
func foldExecutionSelection(attempts []executionSelectionAttempt, baselineGeneration string) (executionSelectedVersion, []executionPendingSelection, error) {
	var selected executionSelectedVersion
	var pending []executionPendingSelection
	if !requestPattern.MatchString(baselineGeneration) || len(attempts) > 128 {
		return selected, nil, errExecutionSelection
	}

	byParent := map[string]executionSelectionAttempt{}
	heads := map[string]bool{}
	byID := map[string]executionSelectionAttempt{}
	for _, a := range attempts {
		if !requestPattern.MatchString(a.begin.ExecutionID) || !requestPattern.MatchString(a.begin.Generation) || !hashPattern.MatchString(a.head) || heads[a.head] {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		if _, exists := byID[a.begin.ExecutionID]; exists {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		heads[a.head], byID[a.begin.ExecutionID] = true, a
	}
	// Every recovery (including an incomplete one) must name exactly one
	// original pre-spawn attempt. One target cannot be retried or superseded by
	// another recovery, and recovery journals can never themselves be targets.
	resolved, targeted := map[string]bool{}, map[string]bool{}
	for _, a := range attempts {
		r := a.recovery
		if r == nil {
			continue
		}
		prior, exists := byID[r.PriorExecutionID]
		if !exists || targeted[r.PriorExecutionID] || prior.recovery != nil || prior.terminal != "" || prior.consumed ||
			(prior.state != "begin" && prior.state != "commissioned") || prior.head != r.PriorJournalSHA256 ||
			prior.begin.ClientSID != r.ClientSID || prior.begin.Generation != r.Generation || prior.begin.CompletionSHA256 != r.CompletionSHA256 ||
			prior.begin.PreviousGeneration != r.PreviousGeneration || prior.begin.PreviousHeadSHA256 != r.PreviousHeadSHA256 {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		targeted[r.PriorExecutionID] = true
		if r.PreviousHeadSHA256 == "" && r.Generation != baselineGeneration {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		if a.terminal == "restored-no-spawn" {
			resolved[r.PriorExecutionID] = true
		}
	}
	for _, a := range attempts {
		if resolved[a.begin.ExecutionID] {
			continue
		}
		if a.terminal == "" {
			pending = append(pending, executionPendingSelection{a.begin.ExecutionID, a.begin.Generation, a.begin.PreviousGeneration})
			continue
		}
		parent := a.begin.PreviousHeadSHA256
		// Explicitly resolved unqualified baseline attempts grant no selected
		// root and occupy no root slot. A fresh qualification still must reach
		// native healthy/selected, independently of this recovery.
		if a.recovery != nil && parent == "" {
			continue
		}
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
	selectedAtHead := map[string]string{}
	for {
		next, exists := byParent[current]
		if !exists {
			break
		}
		if consumed >= 128 || (current != "" && next.begin.PreviousGeneration != selected.generation) {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		if next.terminal == "selected" && next.recovery == nil {
			selected = executionSelectedVersion{next.begin.Generation, next.begin.CompletionSHA256, next.head, releasepolicy.Installed{Version: next.begin.Version, Commit: next.begin.Commit}}
		} else if current != "" && (next.terminal == "rolled-back" || (next.terminal == "restored-no-spawn" && next.recovery != nil)) {
			// Recovery advances history only; selected bytes/version/completion
			// remain exactly those independently qualified by the chain.
			selected.head = next.head
		} else {
			return executionSelectedVersion{}, nil, errExecutionSelection
		}
		current = next.head
		selectedAtHead[current] = selected.generation
		consumed++
	}
	if consumed != len(byParent) {
		return executionSelectedVersion{}, nil, errExecutionSelection
	}
	for _, a := range attempts {
		if resolved[a.begin.ExecutionID] {
			continue
		}
		parent := a.begin.PreviousHeadSHA256
		if parent != "" {
			generation, reachable := selectedAtHead[parent]
			if !reachable || a.begin.PreviousGeneration != generation {
				return executionSelectedVersion{}, nil, errExecutionSelection
			}
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
