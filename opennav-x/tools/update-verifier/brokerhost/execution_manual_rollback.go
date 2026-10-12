package brokerhost

import (
	"context"
	"encoding/json"
	"errors"

	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
)

var ErrManualRollbackRefused = errors.New("manual rollback requires retained qualified ancestor and current profile custody")
var ErrManualRollbackUncertain = errors.New("manual rollback journal outcome uncertain; no automatic retry")

// ManualRollbackRequest identifies one explicit operator operation. These fields
// are comparisons against retained native owners, never authority themselves.
// No path, profile backup, replacement trust, or caller health report is accepted.
type ManualRollbackRequest struct {
	RequestID                  string
	ExpectedSelectedHeadSHA256 string
	TargetGeneration           string
	TargetCompletionSHA256     string
}

// ManualRollbackOutcome describes the observed committed head and CURRENT
// profile preserved during this transaction. It is not fresh compatibility,
// launch permission, a restorable profile or a reconstructible native owner.
type ManualRollbackOutcome struct {
	RequestID                    string
	Generation                   string
	CompletionSHA256             string
	SelectedHeadSHA256           string
	QualifiedHeadSHA256          string
	CurrentProfileSHA256         string
	CurrentProfileIdentitySHA256 string
}

// The original successful qualifier already observed health and closed its
// actual child before selecting this ancestor. A manual rollback retains that
// original journal rather than impersonating a new qualification, recovery or
// failed-candidate rollback. It never copies historical navigation data back.
type executionManualRollbackBegin struct {
	executionSelectionBegin
	QualifiedHeadSHA256          string `json:"qualifiedHeadSha256"`
	QualificationSHA256          string `json:"qualificationSha256"`
	QualificationClosedSHA256    string `json:"qualificationClosedSha256"`
	CurrentProfileSHA256         string `json:"currentProfileSha256"`
	CurrentProfileIdentitySHA256 string `json:"currentProfileIdentitySha256"`
}

type executionManualRollbackStep struct {
	Schema                       int    `json:"schema"`
	Purpose                      string `json:"purpose"`
	ExecutionID                  string `json:"executionId"`
	Step                         string `json:"step"`
	Generation                   string `json:"generation"`
	QualifiedHeadSHA256          string `json:"qualifiedHeadSha256"`
	EvidenceSHA256               string `json:"evidenceSha256"`
	CurrentProfileSHA256         string `json:"currentProfileSha256"`
	CurrentProfileIdentitySHA256 string `json:"currentProfileIdentitySha256"`
}

func validManualRollbackRequest(q ManualRollbackRequest) bool {
	return requestPattern.MatchString(q.RequestID) && requestPattern.MatchString(q.TargetGeneration) &&
		hashPattern.MatchString(q.ExpectedSelectedHeadSHA256) && hashPattern.MatchString(q.TargetCompletionSHA256)
}

func parseExecutionManualRollback(sid, id, generation, completion, generationPurpose, digest string, facts []executionSelectionFact) (executionSelectionAttempt, error) {
	var b executionManualRollbackBegin
	if len(facts) == 0 || len(facts) > 5 || facts[0].kind != brokerstore.ExecutionPreparation ||
		exactExecutionRecord(facts[0].original, &b) != nil || b.Schema != 1 || b.Purpose != "broker-manual-rollback-v1" ||
		!brokerregistration.IsCanonicalUserSID(sid) || b.ClientSID != sid || b.ExecutionID != id || b.Generation != generation || b.CompletionSHA256 != completion ||
		!requestPattern.MatchString(id) || !requestPattern.MatchString(generation) || !hashPattern.MatchString(completion) || !hashPattern.MatchString(digest) ||
		!requestPattern.MatchString(b.PreviousGeneration) || b.PreviousGeneration == generation || !hashPattern.MatchString(b.PreviousHeadSHA256) ||
		!hashPattern.MatchString(b.QualifiedHeadSHA256) || !hashPattern.MatchString(b.CurrentProfileSHA256) || !hashPattern.MatchString(b.CurrentProfileIdentitySHA256) ||
		!hashPattern.MatchString(b.QualificationSHA256) || !hashPattern.MatchString(b.QualificationClosedSHA256) ||
		!commitPattern.MatchString(b.Commit) || len(b.Version) > 128 ||
		(generationPurpose != string(brokerstore.GenerationPurposeApprovedBaseline) && generationPurpose != string(brokerstore.GenerationPurposeSignedCandidate)) {
		return executionSelectionAttempt{}, errExecutionSelection
	}
	if _, err := releasepolicy.ParseSemVer(b.Version); err != nil {
		return executionSelectionAttempt{}, errExecutionSelection
	}
	a := executionSelectionAttempt{begin: b.executionSelectionBegin, head: digest, state: "manual-begin", manual: &b}
	steps := []struct {
		name     string
		kind     brokerstore.ExecutionFactKind
		evidence string
	}{
		{"current-profile-frozen", brokerstore.ExecutionCommissioning, b.CurrentProfileIdentitySHA256},
		{"qualified-ancestor", brokerstore.ExecutionHealthObserved, b.QualificationSHA256},
		{"ancestor-qualification-closed", brokerstore.ExecutionTeardown, b.QualificationClosedSHA256},
		{"manual-selected", brokerstore.ExecutionHealthObserved, b.QualifiedHeadSHA256},
	}
	for i, fact := range facts[1:] {
		var s executionManualRollbackStep
		if exactExecutionRecord(fact.original, &s) != nil || s.Schema != 1 || s.Purpose != "broker-manual-rollback-step-v1" ||
			s.ExecutionID != id || s.Generation != generation || s.Step != steps[i].name || fact.kind != steps[i].kind ||
			s.QualifiedHeadSHA256 != b.QualifiedHeadSHA256 || s.EvidenceSHA256 != steps[i].evidence ||
			s.CurrentProfileSHA256 != b.CurrentProfileSHA256 || s.CurrentProfileIdentitySHA256 != b.CurrentProfileIdentitySHA256 {
			return executionSelectionAttempt{}, errExecutionSelection
		}
		a.state = s.Step
	}
	if a.state == "manual-selected" {
		a.terminal = a.state
	}
	return a, nil
}

// Only a successful ORIGINAL ordinary qualification transaction may be named as
// the ancestor. Failed-start rolled-back, recovery and manual records do not mint
// qualifications. The fold verifies reachability and exact identity at the edge.
func manualRollbackQualified(a executionSelectionAttempt, b *executionManualRollbackBegin) bool {
	return b != nil && a.manual == nil && a.recovery == nil && a.terminal == "selected" &&
		a.head == b.QualifiedHeadSHA256 && a.begin.ClientSID == b.ClientSID && a.begin.Generation == b.Generation &&
		a.begin.CompletionSHA256 == b.CompletionSHA256 && a.begin.Version == b.Version && a.begin.Commit == b.Commit &&
		a.health == b.QualificationSHA256 && a.closed == b.QualificationClosedSHA256
}

func ancestorInstalled(a executionSelectionAttempt) releasepolicy.Installed {
	return releasepolicy.Installed{Version: a.begin.Version, Commit: a.begin.Commit}
}

func manualRollbackAncestor(attempts []executionSelectionAttempt, baseline string, q ManualRollbackRequest) (executionSelectionAttempt, error) {
	if !validManualRollbackRequest(q) {
		return executionSelectionAttempt{}, ErrManualRollbackRefused
	}
	current, pending, err := foldExecutionSelection(attempts, baseline)
	if err != nil || len(pending) != 0 || current.head != q.ExpectedSelectedHeadSHA256 || current.generation == q.TargetGeneration {
		return executionSelectionAttempt{}, errors.Join(ErrManualRollbackRefused, err)
	}
	var target executionSelectionAttempt
	for _, a := range attempts {
		if a.begin.ExecutionID == q.RequestID {
			return executionSelectionAttempt{}, ErrManualRollbackRefused
		}
		if a.manual == nil && a.recovery == nil && a.terminal == "selected" && a.begin.Generation == q.TargetGeneration && a.begin.CompletionSHA256 == q.TargetCompletionSHA256 {
			if target.head != "" || !hashPattern.MatchString(a.health) || !hashPattern.MatchString(a.closed) {
				return executionSelectionAttempt{}, ErrManualRollbackRefused
			}
			target = a
		}
	}
	if target.head == "" {
		return target, ErrManualRollbackRefused
	}
	return target, nil
}

type manualRollbackProfileBinding struct{ contents, identity string }
type manualRollbackProfile interface {
	validate(context.Context) (manualRollbackProfileBinding, error)
	close() error
}

// Private seams are exercised with fault fixtures only. Native assembly supplies
// the original journal writer, frozen current profile and retained history. It
// supplies no spawn, restore, file removal, trust write or restart operation.
type executionManualRollbackRun struct {
	begin                              executionManualRollbackBegin
	journal                            executionHostJournal
	profile                            manualRollbackProfile
	validate                           func(context.Context) error
	qualification, qualificationClosed string
	resolve                            func(context.Context) error
	attempted                          bool
}

func (r *executionManualRollbackRun) check(ctx context.Context) error {
	if ctx == nil || ctx.Err() != nil {
		return errors.Join(ErrManualRollbackRefused, contextError(ctx))
	}
	if err := r.validate(ctx); err != nil {
		return err
	}
	p, err := r.profile.validate(ctx)
	if err != nil || p.contents != r.begin.CurrentProfileSHA256 || p.identity != r.begin.CurrentProfileIdentitySHA256 {
		return errors.Join(ErrManualRollbackRefused, err)
	}
	return ctx.Err()
}

func (r *executionManualRollbackRun) execute(ctx context.Context) (resultErr error) {
	if r == nil || r.attempted || r.journal == nil || r.profile == nil || r.validate == nil || r.resolve == nil ||
		!hashPattern.MatchString(r.qualification) || !hashPattern.MatchString(r.qualificationClosed) ||
		r.qualification != r.begin.QualificationSHA256 || r.qualificationClosed != r.begin.QualificationClosedSHA256 {
		return ErrManualRollbackRefused
	}
	r.attempted = true
	mutated := false
	defer func() {
		resultErr = errors.Join(resultErr, r.profile.close(), r.journal.Close())
		if resultErr != nil && mutated {
			resultErr = errors.Join(ErrManualRollbackUncertain, resultErr)
		}
	}()
	if err := r.check(ctx); err != nil {
		return err
	}
	raw, err := json.Marshal(r.begin)
	if err != nil {
		return err
	}
	if _, err = parseExecutionManualRollback(r.begin.ClientSID, r.begin.ExecutionID, r.begin.Generation,
		r.begin.CompletionSHA256, string(brokerstore.GenerationPurposeApprovedBaseline), r.begin.PreviousHeadSHA256,
		[]executionSelectionFact{{brokerstore.ExecutionPreparation, raw}}); err != nil {
		return err
	}
	mutated = true // ambiguous append must never authorize a retry
	if err = r.journal.AppendFact(ctx, brokerstore.ExecutionPreparation, raw); err != nil {
		return err
	}
	steps := []struct {
		name, evidence string
		kind           brokerstore.ExecutionFactKind
	}{
		{"current-profile-frozen", r.begin.CurrentProfileIdentitySHA256, brokerstore.ExecutionCommissioning},
		{"qualified-ancestor", r.qualification, brokerstore.ExecutionHealthObserved},
		{"ancestor-qualification-closed", r.qualificationClosed, brokerstore.ExecutionTeardown},
		{"manual-selected", r.begin.QualifiedHeadSHA256, brokerstore.ExecutionHealthObserved},
	}
	for _, s := range steps {
		if err = r.check(ctx); err != nil {
			return err
		}
		raw, err = json.Marshal(executionManualRollbackStep{1, "broker-manual-rollback-step-v1", r.begin.ExecutionID,
			s.name, r.begin.Generation, r.begin.QualifiedHeadSHA256, s.evidence, r.begin.CurrentProfileSHA256, r.begin.CurrentProfileIdentitySHA256})
		if err != nil {
			return err
		}
		if err = r.journal.AppendFact(ctx, s.kind, raw); err != nil {
			return err
		}
	}
	if err = r.check(ctx); err != nil {
		return err
	}
	if err = r.resolve(ctx); err != nil {
		return err
	}
	return r.check(ctx)
}
