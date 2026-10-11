package brokerhost

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"sync"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/releasepolicy"
)

// These private seams permit ordering/failure tests. They are not exported
// production injection points; Run constructs the concrete Windows store and
// real authenticated acquisition implementation.
type candidate interface {
	verifier.RecoverySink
	Seal(context.Context, []byte) error
	Close() error
}
type storage interface {
	Validate() error
	ReadRegistration(string) ([]byte, error)
	VerifierDirectories(string, string) (string, string, error)
	begin(string, string) (candidate, error)
}
type acquisition interface {
	StageRecovery(context.Context, verifier.RecoverySink) (verifier.RecoveryIdentity, error)
	Close() error
}
type stockOwner interface {
	io.Closer
	validate() error
}
type engine struct {
	store   storage
	observe func(context.Context, registration) (releasepolicy.OpenCpn, stockOwner, error)
	prepare func(context.Context, verifier.PrepareConfig, releasepolicy.OpenCpn, verifier.ConsentIdentity) (acquisition, error)
	busy    sync.Mutex
}

// No receipt in this operation authorizes launch. It records authenticated
// bytes staged for later qualification, not installed state or startup success.
type stagingReceipt struct {
	Schema    int                       `json:"schema"`
	State     string                    `json:"state"`
	RequestID string                    `json:"requestId"`
	ClientSID string                    `json:"clientSid"`
	Consent   verifier.ConsentIdentity  `json:"consent"`
	Recovery  verifier.RecoveryIdentity `json:"recovery"`
}

var errStageRefused = errors.New("protected update staging refused")

func (e *engine) handle(ctx context.Context, peer brokerpipe.Peer, request brokerpipe.Request) (result brokerpipe.Result, resultErr error) {
	// Do not leak paths, trust roots, server responses or caller-controlled
	// strings across the pipe. Cancellation is a refusal, never a partial success.
	defer func() {
		if resultErr != nil {
			result = brokerpipe.Result{}
			resultErr = errStageRefused
		}
	}()
	if ctx == nil || ctx.Err() != nil || e == nil || e.store == nil || e.observe == nil || e.prepare == nil ||
		peer.SID == "" || peer.ProcessID == 0 || request.Schema != 1 || request.Operation != "prepare-and-stage" ||
		!requestPattern.MatchString(request.RequestID) || !validConsent(request.Consent) {
		return brokerpipe.Result{}, errStageRefused
	}
	if !e.busy.TryLock() {
		return brokerpipe.Result{}, errStageRefused
	}
	defer e.busy.Unlock()
	ctx, cancel := context.WithTimeout(ctx, 30*time.Minute)
	defer cancel()
	if err := e.store.Validate(); err != nil {
		return brokerpipe.Result{}, err
	}
	data, err := e.store.ReadRegistration(peer.SID)
	if err != nil {
		return brokerpipe.Result{}, err
	}
	r, err := parseRegistration(data, peer.SID)
	if err != nil {
		return brokerpipe.Result{}, err
	}
	stateDir, artifactDir, err := e.store.VerifierDirectories(peer.SID, r.Trust.Channel)
	if err != nil {
		return brokerpipe.Result{}, err
	}
	observed, stock, err := e.observe(ctx, r)
	if err != nil || stock == nil || observed != stockBaseline {
		if stock != nil {
			_ = stock.Close()
		}
		return brokerpipe.Result{}, errStageRefused
	}
	defer func() {
		if stock != nil {
			resultErr = errors.Join(resultErr, stock.Close())
		}
	}()
	// CREATE_NEW reserves this exact request ID durably before network activity.
	// Failure leaves a protected partial candidate; the ID is never reused.
	stage, err := e.store.begin(request.RequestID, peer.SID)
	if err != nil {
		return brokerpipe.Result{}, err
	}
	defer func() {
		if stage != nil {
			resultErr = errors.Join(resultErr, stage.Close())
		}
	}()
	if err := e.store.Validate(); err != nil {
		return brokerpipe.Result{}, err
	}
	prepared, err := e.prepare(ctx, r.configuration(stateDir, artifactDir), observed, request.Consent)
	if err != nil || prepared == nil {
		if prepared != nil {
			_ = prepared.Close()
		}
		return brokerpipe.Result{}, errStageRefused
	}
	defer func() {
		if prepared != nil {
			resultErr = errors.Join(resultErr, prepared.Close())
		}
	}()
	if err := e.store.Validate(); err != nil {
		return brokerpipe.Result{}, err
	}
	identity, err := prepared.StageRecovery(ctx, stage)
	if err != nil {
		return brokerpipe.Result{}, err
	}
	if !validRecovery(identity, request.Consent) {
		return brokerpipe.Result{}, errStageRefused
	}
	if err := stock.validate(); err != nil {
		return brokerpipe.Result{}, err
	}
	if err := e.store.Validate(); err != nil {
		return brokerpipe.Result{}, err
	}
	// Cleanup uncertainty must not be represented by a completed staging receipt.
	closeErr := prepared.Close()
	prepared = nil
	if closeErr != nil {
		return brokerpipe.Result{}, closeErr
	}
	if err := ctx.Err(); err != nil {
		return brokerpipe.Result{}, err
	}
	if err := e.store.Validate(); err != nil {
		return brokerpipe.Result{}, err
	}
	receipt, err := json.Marshal(stagingReceipt{Schema: 1, State: stagedState, RequestID: request.RequestID,
		ClientSID: peer.SID, Consent: request.Consent, Recovery: identity})
	if err != nil {
		return brokerpipe.Result{}, err
	}
	if err := stage.Seal(ctx, receipt); err != nil {
		return brokerpipe.Result{}, err
	}
	// Release owners before reporting success; a seal is still only stage evidence.
	closeErr = stage.Close()
	stage = nil
	if closeErr != nil {
		return brokerpipe.Result{}, closeErr
	}
	closeErr = stock.Close()
	stock = nil
	if closeErr != nil {
		return brokerpipe.Result{}, closeErr
	}
	if err := ctx.Err(); err != nil {
		return brokerpipe.Result{}, err
	}
	if err := e.store.Validate(); err != nil {
		return brokerpipe.Result{}, err
	}
	return brokerpipe.Result{Schema: 1, RequestID: request.RequestID, State: stagedState, StageID: request.RequestID,
		Version: request.Consent.Version, Commit: request.Consent.Commit}, nil
}

func validConsent(c verifier.ConsentIdentity) bool {
	_, err := releasepolicy.ParseSemVer(c.Version)
	return err == nil && len(c.Version) <= 128 && commitPattern.MatchString(c.Commit) && hashPattern.MatchString(c.PolicySHA256)
}
func validRecovery(r verifier.RecoveryIdentity, c verifier.ConsentIdentity) bool {
	return r.Version == c.Version && r.Commit == c.Commit && r.LifecycleManifestAvailable &&
		hashPattern.MatchString(r.ExecutableSHA256) && hashPattern.MatchString(r.ProductBuildSHA256) &&
		hashPattern.MatchString(r.InventorySHA256) && hashPattern.MatchString(r.LifecycleManifestSHA256) &&
		hashPattern.MatchString(r.DeclaredPayloadSHA256)
}
