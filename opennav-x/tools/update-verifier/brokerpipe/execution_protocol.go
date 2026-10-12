package brokerpipe

import (
	"bytes"
	"context"
	"encoding/json"
	"strings"
	"unicode/utf8"
)

// ExecutionPipeName is fixed and separate from the unchanged staging protocol.
const ExecutionPipeName = `\\.\pipe\SkagerUpdateBroker.execution.v1`
const maxExecutionPhases = 32

type ExecutionPhase string

const (
	InspectPrevious     ExecutionPhase = "inspect-previous"
	RestorePrevious     ExecutionPhase = "restore-previous"
	CommissionCandidate ExecutionPhase = "commission-candidate"
	SpawnCandidate      ExecutionPhase = "spawn-candidate"
	ObserveCandidate    ExecutionPhase = "observe-candidate"
	CloseCandidate      ExecutionPhase = "close-candidate"
	CommissionFallback  ExecutionPhase = "commission-fallback"
	SpawnFallback       ExecutionPhase = "spawn-fallback"
	ObserveFallback     ExecutionPhase = "observe-fallback"
	CloseFallback       ExecutionPhase = "close-fallback"
	RestoreProfile      ExecutionPhase = "restore-profile"
	FinishExecution     ExecutionPhase = "finish"
)

func validExecutionPhase(p ExecutionPhase) bool {
	switch p {
	case InspectPrevious, RestorePrevious, CommissionCandidate, SpawnCandidate, ObserveCandidate, CloseCandidate, CommissionFallback, SpawnFallback, ObserveFallback, CloseFallback, RestoreProfile, FinishExecution:
		return true
	}
	return false
}

// ExecutionRequest names a proposed conversation and protected stage lookup.
// Its identifiers are data, never consent, durable one-use or launch authority.
// It accepts no command, path, trust configuration or caller-selected pipe.
type ExecutionRequest struct {
	Schema    int    `json:"schema"`
	Operation string `json:"operation"`
	RequestID string `json:"requestId"`
	StageID   string `json:"stageId"`
}

// ExecutionCommand is a finite server request. The session/challenge/sequence
// envelope is minted by the serving transport, never by an execution handler.
// The host supplies actual workflow order and independently authorizes effects.
type ExecutionCommand struct {
	Schema                int            `json:"schema"`
	Kind                  string         `json:"kind"`
	RequestID             string         `json:"requestId"`
	StageID               string         `json:"stageId"`
	SessionID             string         `json:"sessionId"`
	Sequence              uint32         `json:"sequence"`
	Challenge             string         `json:"challenge"`
	Phase                 ExecutionPhase `json:"phase"`
	ApplicationGeneration string         `json:"applicationGeneration"`
	HealthPipe            string         `json:"healthPipe"`
	HealthChallenge       string         `json:"healthChallenge"`
}

// ExecutionDirective is the entire server-selected business payload. Generation
// is a canonical lookup identifier, never a path or selection authority. Only
// spawn/observe carry the fixed startup-health pipe name/challenge. Those values
// are data: the host must own the actual health endpoint/child and corroborate it.
// The transport, not this directive, supplies session/sequence/challenge fields.
type ExecutionDirective struct {
	Phase                 ExecutionPhase
	ApplicationGeneration string
	HealthPipe            string
	HealthChallenge       string
}

func validExecutionDirective(d ExecutionDirective, stageID string) bool {
	if !validExecutionPhase(d.Phase) || !id32.MatchString(stageID) {
		return false
	}
	if d.ApplicationGeneration != "" && !id32.MatchString(d.ApplicationGeneration) {
		return false
	}
	switch d.Phase {
	case FinishExecution:
		return d.ApplicationGeneration == "" && d.HealthPipe == "" && d.HealthChallenge == ""
	case CommissionCandidate, SpawnCandidate, ObserveCandidate, CloseCandidate:
		if d.ApplicationGeneration != "" && d.ApplicationGeneration != stageID {
			return false
		}
	case CommissionFallback, SpawnFallback, ObserveFallback, CloseFallback:
		if d.ApplicationGeneration != "" && d.ApplicationGeneration == stageID {
			return false
		}
	}
	switch d.Phase {
	case SpawnCandidate, ObserveCandidate, SpawnFallback, ObserveFallback:
		return id32.MatchString(d.ApplicationGeneration) && strings.HasPrefix(d.HealthPipe, "Skager.Update.") && id32.MatchString(strings.TrimPrefix(d.HealthPipe, "Skager.Update.")) && hex64.MatchString(d.HealthChallenge)
	default:
		return d.HealthPipe == "" && d.HealthChallenge == ""
	}
}
func (c ExecutionCommand) directive() ExecutionDirective {
	return ExecutionDirective{c.Phase, c.ApplicationGeneration, c.HealthPipe, c.HealthChallenge}
}

// ExecutionObservation is untrusted report data. Even reported-complete proves
// no spawn, health, restore, commissioning or durable transaction success. The
// host must corroborate native child/custody/profile evidence independently.
// Process claims are either both zero or both present; they never supply handles.
type ExecutionObservation struct {
	Status          string `json:"status"`
	EvidenceSHA256  string `json:"evidenceSha256"`
	ProcessID       uint32 `json:"processId"`
	ProcessCreation uint64 `json:"processCreation"`
}

type executionReport struct {
	ExecutionCommand
	ExecutionObservation
}

type ExecutionHandler func(context.Context, Peer, ExecutionRequest, *ExecutionSession) error
type ExecutionClientHandler func(context.Context, ExecutionCommand) (ExecutionObservation, error)

func validExecutionRequest(q ExecutionRequest) bool {
	return q.Schema == 1 && q.Operation == "execute-session" && id32.MatchString(q.RequestID) && id32.MatchString(q.StageID)
}
func encodeExecutionRequest(q ExecutionRequest) ([]byte, error) {
	if !validExecutionRequest(q) {
		return nil, ErrRefused
	}
	return json.Marshal(q)
}
func executionObject(raw []byte, keys []string) error {
	if len(raw) == 0 || len(raw) > maxFrame || !utf8.Valid(raw) {
		return ErrRefused
	}
	fields, err := exactObject(raw, keys, false)
	if err != nil {
		return err
	}
	for _, value := range fields {
		if bytes.Equal(bytes.TrimSpace(value), []byte("null")) {
			return ErrRefused
		}
	}
	return nil
}
func decodeExecutionRequest(raw []byte) (ExecutionRequest, error) {
	var q ExecutionRequest
	if executionObject(raw, []string{"schema", "operation", "requestId", "stageId"}) != nil || json.Unmarshal(raw, &q) != nil || !validExecutionRequest(q) {
		return ExecutionRequest{}, ErrRefused
	}
	return q, nil
}

var executionEnvelopeKeys = []string{"schema", "kind", "requestId", "stageId", "sessionId", "sequence", "challenge", "phase", "applicationGeneration", "healthPipe", "healthChallenge"}

func validExecutionEnvelope(c ExecutionCommand, kind string) bool {
	return c.Schema == 1 && c.Kind == kind && id32.MatchString(c.RequestID) && id32.MatchString(c.StageID) && id32.MatchString(c.SessionID) && c.Sequence > 0 && c.Sequence <= maxExecutionPhases && id32.MatchString(c.Challenge) && validExecutionDirective(c.directive(), c.StageID)
}
func decodeExecutionCommand(raw []byte, kind string) (ExecutionCommand, error) {
	var c ExecutionCommand
	if executionObject(raw, executionEnvelopeKeys) != nil || json.Unmarshal(raw, &c) != nil || !validExecutionEnvelope(c, kind) {
		return ExecutionCommand{}, ErrRefused
	}
	return c, nil
}
func validExecutionObservation(o ExecutionObservation, phase ExecutionPhase) bool {
	if o.Status != "reported-complete" && o.Status != "reported-failed" {
		return false
	}
	if !hex64.MatchString(o.EvidenceSHA256) && !(phase == FinishExecution && o.EvidenceSHA256 == "") {
		return false
	}
	return (o.ProcessID == 0) == (o.ProcessCreation == 0)
}
func encodeExecutionReport(c ExecutionCommand, o ExecutionObservation) ([]byte, error) {
	if !validExecutionEnvelope(c, "command") || !validExecutionObservation(o, c.Phase) {
		return nil, ErrRefused
	}
	c.Kind = "report"
	return json.Marshal(executionReport{c, o})
}
func decodeExecutionReport(raw []byte, expected ExecutionCommand) (ExecutionObservation, error) {
	keys := append(append([]string(nil), executionEnvelopeKeys...), "status", "evidenceSha256", "processId", "processCreation")
	var report executionReport
	if executionObject(raw, keys) != nil || json.Unmarshal(raw, &report) != nil || !validExecutionEnvelope(report.ExecutionCommand, "report") || !validExecutionObservation(report.ExecutionObservation, expected.Phase) {
		return ExecutionObservation{}, ErrRefused
	}
	report.Kind = "command"
	if report.ExecutionCommand != expected {
		return ExecutionObservation{}, ErrRefused
	}
	return report.ExecutionObservation, nil
}
func executionClosingFrame(c ExecutionCommand, kind string) ([]byte, error) {
	if c.Phase != FinishExecution {
		return nil, ErrRefused
	}
	c.Kind = kind
	if !validExecutionEnvelope(c, kind) {
		return nil, ErrRefused
	}
	return json.Marshal(c)
}
func validExecutionClosingFrame(raw []byte, expected ExecutionCommand, kind string) bool {
	got, err := decodeExecutionCommand(raw, kind)
	expected.Kind = kind
	return err == nil && got == expected && got.Phase == FinishExecution
}

// Client sequence checking is transport ordering only, not a workflow reducer.
// A repeated business phase is allowed only with a new explicit server command.
type executionClientSequence struct {
	request    ExecutionRequest
	session    string
	next       uint32
	challenges map[string]bool
	finished   bool
}

func (s *executionClientSequence) accept(c ExecutionCommand) error {
	if s.finished || !validExecutionEnvelope(c, "command") || c.RequestID != s.request.RequestID || c.StageID != s.request.StageID || c.Sequence != s.next+1 {
		return ErrRefused
	}
	if s.session != "" && c.SessionID != s.session {
		return ErrRefused
	}
	if s.challenges[c.Challenge] {
		return ErrRefused
	}
	if s.challenges == nil {
		s.challenges = map[string]bool{}
	}
	s.challenges[c.Challenge] = true
	s.session = c.SessionID
	s.next = c.Sequence
	s.finished = c.Phase == FinishExecution
	return nil
}
