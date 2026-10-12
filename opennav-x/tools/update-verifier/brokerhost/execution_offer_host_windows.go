package brokerhost

import (
	"context"
	"errors"
	"runtime"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/releasepolicy"
)

// This read-only execution intent uses original broker-held registration,
// selected history, stock and helper members. It neither stages nor launches.
// The five-second network budget does not include the human popup. Offline or
// ineligible metadata becomes no offer only after local custody is revalidated.
func (e *engine) handleNativeOfferExecution(ctx context.Context, peer brokerpipe.Peer, q brokerpipe.ExecutionRequest, session *brokerpipe.ExecutionSession) (resultErr error) {
	if e == nil || ctx == nil || ctx.Err() != nil || session == nil || e.observe == nil || q.Schema != 1 || q.Operation != "offer-current" || !requestPattern.MatchString(q.RequestID) || !requestPattern.MatchString(q.StageID) {
		return errExecutionCurrentRefused
	}
	adapter, ok := e.store.(storeAdapter)
	if !ok || adapter.Store == nil {
		return errExecutionCurrentRefused
	}
	capability, err := brokerpipe.AuthorizePeer(ctx, peer)
	if err != nil {
		return err
	}
	if !e.busy.TryLock() {
		return errExecutionCurrentRefused
	}
	defer e.busy.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err = adapter.Validate(); err != nil {
		return err
	}
	raw, err := adapter.ReadRegistration(peer.SID)
	if err != nil {
		return err
	}
	r, err := parseRegistration(raw, peer.SID)
	if err != nil {
		return err
	}
	selected, err := captureSelectedExecutionNative(ctx, adapter.Store, r)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, selected.Close()) }()
	if selected.selected.generation != q.StageID {
		return errExecutionCurrentRefused
	}
	executor, err := executionHostExecutor(ctx, selected.generation)
	if err != nil {
		return err
	}
	if err = session.BindExecutorImage(ctx, executor); err != nil {
		return err
	}
	observed, stock, err := e.observe(ctx, r)
	if stock != nil {
		defer func() { resultErr = errors.Join(resultErr, stock.Close()) }()
	}
	if err != nil || stock == nil || observed != stockBaseline {
		return errors.Join(errExecutionCurrentRefused, err)
	}
	validate := func() error {
		return errors.Join(ctx.Err(), adapter.Validate(), capability.Validate(ctx), selected.validate(), stock.validate())
	}
	if err = validate(); err != nil {
		return err
	}
	state, artifacts, err := adapter.VerifierDirectories(peer.SID, r.Trust.Channel)
	if err != nil {
		return err
	}
	config := r.configuration(state, artifacts)
	config.Installed = selected.installed()
	checkCtx, cancel := context.WithTimeout(ctx, 5*time.Second)
	offer, checkErr := verifier.CheckUpdate(checkCtx, config, observed)
	cancel()
	if err = validate(); err != nil {
		return err
	}
	if checkErr == nil && offer.Decision == releasepolicy.Upgrade {
		member, err := selected.generation.OpenMember("app/skager-update-prompt.exe")
		if err != nil {
			return err
		}
		defer func() { resultErr = errors.Join(resultErr, member.Close()) }()
		identity, err := member.Identity()
		if err != nil {
			return err
		}
		if identity.Length <= 0 || identity.Length > 512<<20 || !hashPattern.MatchString(identity.SHA256) {
			return errExecutionCurrentRefused
		}
		directive := brokerpipe.ExecutionDirective{Phase: brokerpipe.OfferUpdate, ApplicationGeneration: q.StageID, OfferVersion: offer.Consent.Version, OfferCommit: offer.Consent.Commit, OfferPolicySHA256: offer.Consent.PolicySHA256, PromptBytes: identity.Length, PromptSHA256: identity.SHA256}
		report, err := session.ExchangeCommand(ctx, directive)
		if err != nil {
			return err
		}
		if report.Status != "reported-complete" || report.EvidenceSHA256 != identity.SHA256 || report.ProcessID != 0 || report.ProcessCreation != 0 {
			return errExecutionCurrentRefused
		}
	}
	if err = validate(); err != nil {
		return err
	}
	_, err = session.ExchangeCommand(ctx, brokerpipe.ExecutionDirective{Phase: brokerpipe.FinishExecution})
	return err
}
