package launcher

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"errors"
	"sync"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/releasepolicy"
)

// The only production source of an offer is CallExecution after native SCM/pipe
// authentication. No caller file, trust JSON, receipt or local hash supplies it.
type brokerUpdateServices struct {
	conversation     func(context.Context, brokerpipe.ExecutionRequest, brokerpipe.ExecutionClientHandler) error
	prompt           func(context.Context, brokerpipe.ExecutionCommand) (bool, error)
	close            func() error
	stage            func(context.Context, brokerpipe.Request) (brokerpipe.Result, error)
	execute, current func(context.Context, string) error
}
type brokerOfferActions struct {
	mu                 sync.Mutex
	request            brokerpipe.ExecutionRequest
	prompt             func(context.Context, brokerpipe.ExecutionCommand) (bool, error)
	consumed, finished bool
	consent            verifier.ConsentIdentity
	failure            error
}

// The actual native prompt returns 0 only for Later and 10 only for Update Now.
// Initialization/parse failure, a crash, or an unknown result is not consent to
// continue startup and must not silently become the Later path.
func brokerUpdatePromptResult(code uint32) (bool, error) {
	switch code {
	case 0:
		return false, nil
	case 10:
		return true, nil
	default:
		return false, errBrokerExecutionUncertain
	}
}

func validBrokerOffer(c brokerpipe.ExecutionCommand, generation string) bool {
	_, err := releasepolicy.ParseSemVer(c.OfferVersion)
	return c.Phase == brokerpipe.OfferUpdate && c.ApplicationGeneration == generation && c.HealthPipe == "" && c.HealthChallenge == "" && err == nil && len(c.OfferVersion) <= 128 && commitPattern.MatchString(c.OfferCommit) && hashPattern.MatchString(c.OfferPolicySHA256) && c.PromptBytes > 0 && c.PromptBytes <= 512<<20 && hashPattern.MatchString(c.PromptSHA256)
}
func (a *brokerOfferActions) handle(ctx context.Context, c brokerpipe.ExecutionCommand) (o brokerpipe.ExecutionObservation, resultErr error) {
	a.mu.Lock()
	defer a.mu.Unlock()
	if ctx == nil || ctx.Err() != nil || a.finished || a.failure != nil || c.Schema != 1 || c.Kind != "command" || c.RequestID != a.request.RequestID || c.StageID != a.request.StageID {
		return o, errBrokerExecutionRefused
	}
	defer func() {
		if resultErr != nil {
			a.failure = errors.Join(a.failure, resultErr)
		}
	}()
	if c.Phase == brokerpipe.FinishExecution {
		if brokerExecutionPayload(c, a.request.StageID) != nil {
			return o, errBrokerExecutionRefused
		}
		a.finished = true
		return brokerpipe.ExecutionObservation{Status: "reported-complete"}, nil
	}
	if a.consumed || a.prompt == nil || !validBrokerOffer(c, a.request.StageID) {
		return o, errBrokerExecutionRefused
	}
	a.consumed = true // consume before entering the real native helper; no repeat.
	now, err := a.prompt(ctx, c)
	if err != nil || ctx.Err() != nil {
		return o, errors.Join(errBrokerExecutionUncertain, err, ctx.Err())
	}
	if now {
		a.consent = verifier.ConsentIdentity{Version: c.OfferVersion, Commit: c.OfferCommit, PolicySHA256: c.OfferPolicySHA256}
	}
	return brokerpipe.ExecutionObservation{Status: "reported-complete", EvidenceSHA256: c.PromptSHA256}, nil
}
func brokerUpdateNonce() (string, error) {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return "", err
	}
	return hex.EncodeToString(b[:]), nil
}
func runBrokerUpdate(ctx context.Context, generation string, s brokerUpdateServices) (resultErr error) {
	if ctx == nil || ctx.Err() != nil || !generationPattern.MatchString(generation) || s.conversation == nil || s.prompt == nil || s.close == nil || s.stage == nil || s.execute == nil || s.current == nil {
		return errBrokerExecutionRefused
	}
	closed := false
	defer func() {
		if !closed {
			resultErr = errors.Join(resultErr, s.close())
		}
	}()
	nonce, err := brokerUpdateNonce()
	if err != nil {
		return err
	}
	a := &brokerOfferActions{request: brokerpipe.ExecutionRequest{Schema: 1, Operation: "offer-current", RequestID: nonce, StageID: generation}, prompt: s.prompt}
	err = s.conversation(ctx, a.request, a.handle)
	// Drain the entire authenticated offer and native helper lifetime BEFORE opening
	// a new staging/current conversation. An uncertain finish is never a retry.
	closed = true
	err = errors.Join(err, s.close(), a.failure, ctx.Err())
	if err != nil || !a.finished {
		return errors.Join(errBrokerExecutionUncertain, err)
	}
	if a.consent == (verifier.ConsentIdentity{}) {
		return s.current(ctx, generation)
	}
	nonce, err = brokerUpdateNonce()
	if err != nil {
		return err
	}
	request := brokerpipe.Request{Schema: 1, Operation: "prepare-and-stage", RequestID: nonce, Consent: a.consent}
	staged, err := s.stage(ctx, request)
	if err != nil {
		return errors.Join(errBrokerExecutionUncertain, err)
	}
	if ctx.Err() != nil || staged.Schema != 1 || staged.RequestID != nonce || staged.StageID != nonce || staged.State != brokerpipe.Staged || staged.Version != a.consent.Version || staged.Commit != a.consent.Commit {
		return errors.Join(errBrokerExecutionUncertain, ctx.Err())
	}
	// The result is a lookup identifier only. Actual broker execution reopens the
	// original stage seal, consent, signed offer, selected chain and native peer.
	return s.execute(ctx, staged.StageID)
}

func consentFromBrokerOffer(c brokerpipe.ExecutionCommand) verifier.ConsentIdentity {
	return verifier.ConsentIdentity{Version: c.OfferVersion, Commit: c.OfferCommit, PolicySHA256: c.OfferPolicySHA256}
}
