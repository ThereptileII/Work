//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"

	"example.com/opennav-update-verifier/launcher"
	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
)

func (n *nativeCodeUpgrade) publishStageReceipt(ctx context.Context, name string, file startupRestorationFile) error {
	intentRaw, err := json.Marshal(n.record)
	if err != nil {
		return err
	}
	if _, err = parseCodeIntent(intentRaw); err != nil {
		return err
	}
	artifact, suffix := n.policy.Bootstrap.Broker, codeBrokerStageSuffix
	if name == stagedHelperName {
		artifact, suffix = n.policy.Bootstrap.Provisioner, codeHelperStageSuffix
	} else if name != stagedBrokerName {
		return errors.New("fixed staged code role required")
	}
	identity, err := startupFileIdentity(ctx, file.pin, artifact.SHA256)
	if err != nil {
		return err
	}
	raw, err := json.Marshal(codeStageReceipt{1, digest(intentRaw), name, identity})
	if err != nil {
		return err
	}
	if _, err = parseCodeStageReceipt(raw, digest(intentRaw), name, artifact, n.record.CodeParent); err != nil {
		return err
	}
	sd, err := clientRequestDescriptor(n.backend.brokerSID, false)
	if err != nil {
		return err
	}
	_, err = n.write(ctx, n.original.requestParent, n.original.clientSID+suffix, sd, raw)
	return err
}

// This reader grants no SCM capability. Every accepted retained staged file has
// its own durable completion receipt; absence of a receipt never adopts a file.
func readCodeEarlyStateNative(ctx context.Context, c *nativeInstalledMaintenanceAuthority, original installedCompletionRecord, originalRaw []byte, recordSD, imageSD *windows.SECURITY_DESCRIPTOR) ([]byte, error) {
	for _, suffix := range []string{codeDrainSuffix, codeSuccessorSuffix, codeSuccessorPairSuffix, codeSuccessorMarkerSuffix, codeStartAttemptSuffix, codeDemandAttemptSuffix} {
		if err := startupRestorationAbsent(c.requestParent, c.clientSID+suffix); err != nil {
			return nil, errors.Join(errors.New("later code transition evidence forbids early abort"), err)
		}
	}
	for _, name := range []string{previousBrokerName, previousHelperName} {
		if err := startupRestorationAbsent(c.codeParent, name); err != nil {
			return nil, err
		}
	}
	read := func(suffix string) ([]byte, error) {
		pin, err := maintenanceHoldFile(c.files, c.requestParent, c.clientSID+suffix, recordSD, maxCodeUpgradeRecordBytes)
		if err != nil {
			return nil, err
		}
		return installedReadPin(ctx, pin, maxCodeUpgradeRecordBytes)
	}
	intentRaw, err := read(codeIntentSuffix)
	if err != nil {
		return nil, err
	}
	intent, err := parseCodeIntent(intentRaw)
	if err != nil {
		return nil, err
	}
	parent, err := startupDirectoryIdentity(c.codeParent)
	if err != nil || intent.OriginalCompletionSHA256 != digest(originalRaw) || intent.ClientRequestSHA256 != original.ClientRequestSHA256 ||
		intent.PreviousBroker != original.Broker || intent.PreviousHelper != original.MaintenanceHelper || intent.CodeParent != parent {
		return nil, errors.Join(errors.New("early intent differs from immutable original installation"), err)
	}
	for _, item := range []struct {
		name string
		file startupFileBinding
		out  **enrollmentPin
	}{{enrollmentImageName, original.Broker, &c.brokerImage}, {maintenanceImageName, original.MaintenanceHelper, &c.helperImage}} {
		pin, err := maintenanceHoldFile(c.files, c.codeParent, item.name, imageSD, maxBrokerBytes)
		if err != nil {
			return nil, err
		}
		if err = installedMatchFile(ctx, pin, item.file); err != nil {
			return nil, err
		}
		*item.out = pin
	}
	p, err := releasepolicy.ParseBrokerCodeUpgrade(intent.Policy)
	if err != nil || p.Bootstrap.TrustConfigSHA256 != c.recordTrustSHA() {
		return nil, errors.Join(errors.New("early intent trust differs from original"), err)
	}
	abort := codeEarlyAbortRecord{Schema: 1, Purpose: "installed-broker-code-early-abort", OriginalCompletionSHA256: digest(originalRaw), IntentSHA256: digest(intentRaw)}
	ids := map[[16]byte]bool{parent.FileID: true, original.Broker.FileID: true, original.MaintenanceHelper.FileID: true}
	reserved := intent
	for _, item := range []struct {
		name, suffix string
		artifact     releasepolicy.Artifact
		digest       *string
		file         *startupFileBinding
	}{{stagedBrokerName, codeBrokerStageSuffix, p.Bootstrap.Broker, &abort.BrokerStageSHA256, &reserved.Broker},
		{stagedHelperName, codeHelperStageSuffix, p.Bootstrap.Provisioner, &abort.HelperStageSHA256, &reserved.Helper}} {
		if startupRestorationAbsent(c.codeParent, item.name) == nil && startupRestorationAbsent(c.requestParent, c.clientSID+item.suffix) == nil {
			continue
		}
		raw, err := read(item.suffix)
		if err != nil {
			return nil, err
		}
		receipt, err := parseCodeStageReceipt(raw, abort.IntentSHA256, item.name, item.artifact, parent)
		if err != nil || ids[receipt.File.FileID] {
			return nil, errors.Join(errors.New("distinct original staged identity required"), err)
		}
		pin, err := maintenanceHoldFile(c.files, c.codeParent, item.name, imageSD, maxBrokerBytes)
		if err != nil {
			return nil, err
		}
		if err = installedMatchFile(ctx, pin, receipt.File); err != nil {
			return nil, err
		}
		ids[receipt.File.FileID] = true
		*item.digest, *item.file = digest(raw), receipt.File
	}
	if startupRestorationAbsent(c.requestParent, c.clientSID+codeReservationSuffix) != nil {
		raw, err := read(codeReservationSuffix)
		if err != nil {
			return nil, err
		}
		expected, err := marshalCodeUpgradeRecord(reserved)
		if err != nil || !bytes.Equal(expected, raw) {
			return nil, errors.Join(errors.New("early reservation differs from exact staged receipts"), err)
		}
		abort.ReservationSHA256 = digest(raw)
	}
	return codeEarlyAbortBytes(abort)
}

func readCodeEarlyAbortNative(ctx context.Context, c *nativeInstalledMaintenanceAuthority, original installedCompletionRecord, originalRaw []byte, recordSD, imageSD *windows.SECURITY_DESCRIPTOR) (installedCompletionRecord, error) {
	expected, err := readCodeEarlyStateNative(ctx, c, original, originalRaw, recordSD, imageSD)
	if err != nil {
		return original, err
	}
	pin, err := maintenanceHoldFile(c.files, c.requestParent, c.clientSID+codeEarlyAbortSuffix, recordSD, maxCodeUpgradeRecordBytes)
	if err != nil {
		return original, err
	}
	raw, err := installedReadPin(ctx, pin, maxCodeUpgradeRecordBytes)
	if err != nil || !bytes.Equal(raw, expected) {
		return original, errors.Join(errors.New("exact immutable early-abort closure required"), err)
	}
	return original, c.files.Check()
}

type nativeCodeEarlyAbortPublication struct {
	inner             *nativeCodeFinalizePublication
	recordSD, imageSD *windows.SECURITY_DESCRIPTOR
}

func (p *nativeCodeEarlyAbortPublication) check(ctx context.Context) error {
	if err := p.inner.check(ctx); err != nil {
		return err
	}
	c := p.inner.original
	raw, err := readCodeEarlyStateNative(ctx, c, c.record, c.originalRaw, p.recordSD, p.imageSD)
	if err != nil || !bytes.Equal(raw, p.inner.raw) {
		return errors.Join(errors.New("same early-abort closure changed"), err)
	}
	return nil
}
func (p *nativeCodeEarlyAbortPublication) publish(ctx context.Context, guard func(context.Context) error) error {
	return p.inner.publish(ctx, func(ctx context.Context) error { return errors.Join(p.check(ctx), guard(ctx)) })
}
func (p *nativeCodeEarlyAbortPublication) close() error { return p.inner.close() }

// Caller already reauthenticated the same signed target/source/trust/completion.
// Borrow its QUERY_CONFIG/QUERY_STATUS/READ_CONTROL observer; no service mutation
// capability or legacy fallback is introduced by this phase.
func openCodeEarlyAbortNative(ctx context.Context, c *nativeInstalledMaintenanceAuthority, observer *nativeCodeFinalizeBackend, p releasepolicy.BrokerCodeUpgradePolicy, policyRaw []byte) (codeUpgradeResumeOwner, error) {
	recordSD, err := clientRequestDescriptor(observer.brokerSID, false)
	if err != nil {
		return nil, err
	}
	sid, err := windows.StringToSid(observer.brokerSID)
	if err != nil {
		return nil, err
	}
	imageSD, err := enrollmentCodeDescriptor(sid, false)
	if err != nil {
		return nil, err
	}
	raw, err := readCodeEarlyStateNative(ctx, c, c.record, c.originalRaw, recordSD, imageSD)
	if err != nil {
		return nil, err
	}
	intentPin, err := maintenanceHoldFile(c.files, c.requestParent, c.clientSID+codeIntentSuffix, recordSD, maxCodeUpgradeRecordBytes)
	if err != nil {
		return nil, err
	}
	intentRaw, err := installedReadPin(ctx, intentPin, maxCodeUpgradeRecordBytes)
	if err != nil {
		return nil, err
	}
	intent, err := parseCodeIntent(intentRaw)
	if err != nil || !bytes.Equal(intent.Policy, policyRaw) || intent.PolicySHA256 != digest(policyRaw) || p.PreviousBrokerSHA256 != c.record.Broker.SHA256 || p.PreviousHelperSHA256 != c.record.MaintenanceHelper.SHA256 {
		return nil, errors.Join(errors.New("early abort differs from approved transition"), err)
	}
	observer.image = c.brokerImage
	publication := &nativeCodeEarlyAbortPublication{inner: &nativeCodeFinalizePublication{original: c, files: &nativeEnrollmentFiles{}, parent: c.requestParent,
		name: c.clientSID + codeEarlyAbortSuffix, descriptor: recordSD, raw: raw}, recordSD: recordSD, imageSD: imageSD}
	finalizer, err := openCodeFinalizerWith(ctx, observer, publication, launcher.CallMaintenanceNative)
	if err != nil {
		return nil, err
	}
	return &codeUpgradeEarlyAborter{finalizer}, nil
}
