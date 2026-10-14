package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/releasepolicy"
)

func earlyIntentFixture(t *testing.T) (codeUpgradeRecord, []byte) {
	t.Helper()
	r := codeUpgradeRecordFixture(t)
	r.Broker, r.Helper = startupFileBinding{}, startupFileBinding{}
	raw, err := json.Marshal(r)
	if err != nil {
		t.Fatal(err)
	}
	return r, raw
}

func TestCodeUpgradeEarlyIntentNeverInventsSuccessorIdentity(t *testing.T) {
	r, raw := earlyIntentFixture(t)
	parsed, err := parseCodeIntent(raw)
	if err != nil || !bytes.Equal(parsed.Policy, r.Policy) {
		t.Fatal(err)
	}
	for _, change := range []func(*codeUpgradeRecord){
		func(r *codeUpgradeRecord) { r.Broker = codeUpgradeRecordFixture(t).Broker },
		func(r *codeUpgradeRecord) { r.ReservationSHA256 = strings.Repeat("a", 64) },
		func(r *codeUpgradeRecord) { r.PreviousHelper.FileID = r.PreviousBroker.FileID },
		func(r *codeUpgradeRecord) { r.OriginalCompletionSHA256 = "" },
		func(r *codeUpgradeRecord) { r.PreviousBroker.Volume++ },
	} {
		bad := r
		change(&bad)
		body, _ := json.Marshal(bad)
		if _, err := parseCodeIntent(body); err == nil {
			t.Fatal("non-original intent accepted")
		}
	}
	for _, bad := range [][]byte{nil, []byte("{"), append(bytes.Clone(raw), ' '), append(bytes.Clone(raw), raw...)} {
		if _, err := parseCodeIntent(bad); err == nil {
			t.Fatal("torn/noncanonical intent accepted")
		}
	}
}

func TestCodeUpgradeCompleteStageReceiptRequiresExactIntentArtifactAndNativeIdentity(t *testing.T) {
	r := codeUpgradeRecordFixture(t)
	p, err := releasepolicy.ParseBrokerCodeUpgrade(r.Policy)
	if err != nil {
		t.Fatal(err)
	}
	_, intent := earlyIntentFixture(t)
	receipt := codeStageReceipt{1, digest(intent), stagedBrokerName, r.Broker}
	raw, _ := json.Marshal(receipt)
	if _, err := parseCodeStageReceipt(raw, digest(intent), stagedBrokerName, p.Bootstrap.Broker, r.CodeParent); err != nil {
		t.Fatal(err)
	}
	for _, change := range []func(*codeStageReceipt){
		func(r *codeStageReceipt) { r.IntentSHA256 = strings.Repeat("e", 64) },
		func(r *codeStageReceipt) { r.Name = stagedHelperName },
		func(r *codeStageReceipt) { r.File.Bytes-- },
		func(r *codeStageReceipt) { r.File.SHA256 = strings.Repeat("f", 64) },
		func(r *codeStageReceipt) { r.File.FileID = [16]byte{} },
		func(r *codeStageReceipt) { r.File.Creation = 0 },
		func(r *codeStageReceipt) { r.File.Volume++ },
	} {
		bad := receipt
		change(&bad)
		body, _ := json.Marshal(bad)
		if _, err := parseCodeStageReceipt(body, digest(intent), stagedBrokerName, p.Bootstrap.Broker, r.CodeParent); err == nil {
			t.Fatal("unbound or partial staged code accepted")
		}
	}
}

func TestCodeUpgradeEarlyAbortClosureCannotSkipOrderedStaging(t *testing.T) {
	r := codeEarlyAbortRecord{Schema: 1, Purpose: "installed-broker-code-early-abort", OriginalCompletionSHA256: strings.Repeat("a", 64), IntentSHA256: strings.Repeat("b", 64)}
	if _, err := codeEarlyAbortBytes(r); err != nil {
		t.Fatal(err)
	}
	r.HelperStageSHA256 = strings.Repeat("c", 64)
	if _, err := codeEarlyAbortBytes(r); err == nil {
		t.Fatal("helper without completed broker accepted")
	}
	r.HelperStageSHA256 = ""
	r.ReservationSHA256 = strings.Repeat("d", 64)
	if _, err := codeEarlyAbortBytes(r); err == nil {
		t.Fatal("reservation without complete pair accepted")
	}
}

func TestCodeUpgradeEarlyAbortIsDistinctTerminalNonSuccessWithNoServiceMutation(t *testing.T) {
	ctx := context.Background()
	for _, fault := range []string{"", "query", "publication", "process"} {
		t.Run(fault, func(t *testing.T) {
			events := []string{}
			backend := &finalizeBackendFixture{state: serviceState{state: 4, processID: 17, controls: 5}, events: &events}
			publication := &finalizePublicationFixture{events: &events}
			queries := 0
			finalizer, err := openCodeFinalizerWith(ctx, backend, publication, func(_ context.Context, q brokerpipe.MaintenanceRequest) (brokerpipe.MaintenanceResult, error) {
				queries++
				if fault == "query" {
					return brokerpipe.MaintenanceResult{}, errors.New("refused")
				}
				return finalizeResponse(q), nil
			})
			if err != nil {
				t.Fatal(err)
			}
			owner := &codeUpgradeEarlyAborter{finalizer}
			if fault == "publication" {
				publication.checkErr = errors.New("changed")
			}
			if fault == "process" {
				backend.processErr = errors.New("lost original process")
			}
			err = owner.resume(ctx)
			if (fault == "") != errors.Is(err, ErrCodeUpgradeAborted) || err == nil {
				t.Fatal("abort reported upgrade success or unsupported recovery", err)
			}
			if fault == "" && (queries != 1 || publication.effects != 1 || errors.Is(err, ErrCodeUpgradeUncertain)) {
				t.Fatal("abort readiness/publication did not occur exactly once", err)
			}
			if err = owner.resume(ctx); err == nil || errors.Is(err, ErrCodeUpgradeAborted) {
				t.Fatal("terminal abort replayed", err)
			}
			if err = owner.close(); err != nil {
				t.Fatal(err)
			}
		})
	}
}

func TestCodeUpgradeEarlyAbortReachesCallerWithoutSuccessOrAutomaticContinuation(t *testing.T) {
	r, op, _, events := upgradeFixture(t)
	r.Resume = true
	op.openResume = func(context.Context, releasepolicy.BrokerCodeUpgradePolicy, []byte, []byte, string) (codeUpgradeResumeOwner, error) {
		return &earlyAbortedFixture{}, nil
	}
	err := upgradeEnrolledCodeWith(context.Background(), r, op)
	if !errors.Is(err, ErrCodeUpgradeAborted) || errors.Is(err, ErrCodeUpgradeUncertain) {
		t.Fatal("verified original recovery lost distinct non-success result", err)
	}
	for _, event := range *events {
		if event == "acquire" || event == "native-completion-service" || event == "disable-stop-exit" || event == "activate" {
			t.Fatal("abort reached new transaction or SCM continuation", *events)
		}
	}
}

func TestCodeUpgradeEarlyAbortPostEffectCustodyLossRemainsUncertain(t *testing.T) {
	r, op, _, _ := upgradeFixture(t)
	r.Resume = true
	op.openResume = func(context.Context, releasepolicy.BrokerCodeUpgradePolicy, []byte, []byte, string) (codeUpgradeResumeOwner, error) {
		return &earlyAbortedFixture{loseAfterAbort: true}, nil
	}
	err := upgradeEnrolledCodeWith(context.Background(), r, op)
	if !errors.Is(err, ErrCodeUpgradeUncertain) || errors.Is(err, ErrCodeUpgradeAborted) {
		t.Fatal("lost post-effect custody reported verified original recovery", err)
	}
}

type earlyAbortedFixture struct{ loseAfterAbort, aborted bool }

func (f *earlyAbortedFixture) check(context.Context) error {
	if f.aborted && f.loseAfterAbort {
		return errors.New("custody lost")
	}
	return nil
}
func (*earlyAbortedFixture) close() error { return nil }
func (f *earlyAbortedFixture) resume(context.Context) error {
	f.aborted = true
	return ErrCodeUpgradeAborted
}
