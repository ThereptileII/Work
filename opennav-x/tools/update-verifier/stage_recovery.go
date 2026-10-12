package verifier

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"hash"
	"io"
	"sort"
	"strings"

	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/releasepolicy"
)

// RecoverySink receives validated installed-payload relative paths, exact sizes
// and SHA-256 digests. WriteFile must consume the reader synchronously and retain
// neither it nor the context. The caller owns destination protection, exclusive
// creation, resulting-byte verification and abort/cleanup on every failure.
// Implementing this interface alone does not establish protected custody.
type RecoverySink interface {
	WriteFile(context.Context, string, io.Reader, int64, string) error
}

// RecoveryEvidenceSink retains the exact authenticated original sidecars and any
// inventoried installer engine helpers alongside payload, then verifies their
// identity and complete retained membership. These
// methods alone authenticate no publisher or grant execution authority.
// StageRecovery requires this extension; RecoverySink remains the payload-only
// interface used by initial approved-baseline import.
type RecoveryEvidenceSink interface {
	RecoverySink
	WriteEvidence(context.Context, string, []byte, string) error
	CompleteRecoveryEvidence(context.Context, baselineevidence.Identity) error
}

// This private, owner-bound seal is created only after real authenticated
// acquisition succeeds. Public fields are informational, not acquisition proof.
type recoveryStageBinding struct {
	owner               *PreparedUpdateSet
	installer, recovery *VerifiedArtifact
	policy              releasepolicy.Policy
	consent             ConsentIdentity
	identity            RecoveryIdentity
}

func sealRecoveryStage(p *PreparedUpdateSet) {
	policy := p.Policy
	policy.SupportedOpenCpn = append([]releasepolicy.OpenCpn(nil), policy.SupportedOpenCpn...)
	p.stage = &recoveryStageBinding{owner: p, installer: p.Installer, recovery: p.Recovery,
		policy: policy, consent: p.Consent, identity: p.RecoveryIdentity}
}

func (p *PreparedUpdateSet) recoveryStageValid() bool {
	if p == nil || p.stage == nil || p.stage.owner != p || p.Installer != p.stage.installer || p.Recovery != p.stage.recovery ||
		p.Installer == nil || p.Recovery == nil || p.Installer.closed || p.Recovery.closed || p.Installer.file == nil || p.Recovery.file == nil ||
		p.Consent != p.stage.consent || p.RecoveryIdentity != p.stage.identity {
		return false
	}
	consent, err := policyConsent(p.Policy)
	return err == nil && consent == p.stage.consent
}

// StageRecovery rechecks the retained authenticated Recovery ZIP and all inner
// identities, requiring the exact Lifecycle manifest sidecar. It streams only
// that manifest's app/docs payload set, in lexical path order, excluding the
// portable marker, and preserves exact original FILE_SHA256.json and
// installer/package.json bytes and the fixed inventoried installer engine helpers
// as evidence. No profile is staged. Historical archives without engine helpers
// supply no helper execution authority.
//
// It selects no OS destination and grants no execution, broker or commissioning
// authority. In particular, a sidecar is not Setup embedded-resource proof.
// Retain the acquisition set and its protected directory throughout this call;
// do not mutate/copy/close it concurrently. On any failure the returned identity
// is empty; partial destination files belong to the sink's abort/cleanup path.
func (p *PreparedUpdateSet) StageRecovery(ctx context.Context, sink RecoverySink) (RecoveryIdentity, error) {
	var empty RecoveryIdentity
	if ctx == nil || sink == nil || !p.recoveryStageValid() {
		return empty, errors.New("live authenticated acquisition set and recovery sink required")
	}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	evidence, ok := sink.(RecoveryEvidenceSink)
	if !ok {
		return empty, errors.New("exact original recovery evidence sink required")
	}
	binding := p.stage
	contents, err := readRecoveryContents(ctx, binding.recovery, binding.policy)
	if canceled := ctx.Err(); canceled != nil {
		return empty, canceled
	}
	if err != nil {
		return empty, err
	}
	if !contents.identity.LifecycleManifestAvailable || contents.identity != binding.identity {
		return empty, errors.New("retained recovery identity or Lifecycle sidecar differs")
	}
	var names []string
	for name := range contents.files {
		if (strings.HasPrefix(name, "app/") || strings.HasPrefix(name, "docs/")) && name != "app/OPENNAV_PORTABLE_PREVIEW" {
			names = append(names, name)
		}
	}
	sort.Strings(names)
	for _, name := range names {
		if err := ctx.Err(); err != nil {
			return empty, err
		}
		member := contents.files[name]
		reader, err := member.Open()
		if err != nil {
			return empty, errors.New("recovery staging member unavailable")
		}
		size := int64(member.UncompressedSize64)
		stream := &recoveryStageReader{source: downloadContextReader{ctx, io.LimitReader(reader, size+1)}, digest: sha256.New()}
		writeErr := sink.WriteFile(ctx, name, stream, size, contents.inventory[name])
		// Require full consumption and EOF: the ZIP reader verifies CRC at EOF.
		// Never drain an unread member for a sink that falsely reports success.
		var tail [1]byte
		var n int
		var eof error
		if writeErr == nil && stream.count == size {
			n, eof = stream.Read(tail[:])
		}
		closeErr := reader.Close()
		if err := ctx.Err(); err != nil {
			return empty, err
		}
		if writeErr != nil {
			return empty, errors.New("recovery staging sink refused")
		}
		if closeErr != nil || stream.count != size || n != 0 || eof != io.EOF || hex.EncodeToString(stream.digest.Sum(nil)) != contents.inventory[name] {
			return empty, errors.New("recovery staging member length, hash or CRC differs")
		}
	}
	for _, original := range []struct {
		name   string
		raw    []byte
		digest string
	}{
		{"FILE_SHA256.json", contents.inventoryBytes, contents.identity.InventorySHA256},
		{"installer/package.json", contents.packageBytes, contents.identity.LifecycleManifestSHA256},
	} {
		if err := ctx.Err(); err != nil {
			return empty, err
		}
		// Pass an owned copy: a sink must never mutate the authenticated parser's
		// retained bytes or affect the next evidence write.
		if err := evidence.WriteEvidence(ctx, original.name, append([]byte(nil), original.raw...), original.digest); err != nil {
			if canceled := ctx.Err(); canceled != nil {
				return empty, canceled
			}
			return empty, errors.New("recovery staging evidence sink refused")
		}
	}
	if err := copyRecoveryEngine(ctx, contents, evidence); err != nil {
		if canceled := ctx.Err(); canceled != nil {
			return empty, canceled
		}
		return empty, errors.New("recovery staging engine evidence refused")
	}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	if !p.recoveryStageValid() {
		return empty, errors.New("acquisition set changed during recovery staging")
	}
	if err := evidence.CompleteRecoveryEvidence(ctx, contents.identity); err != nil {
		if canceled := ctx.Err(); canceled != nil {
			return empty, canceled
		}
		return empty, errors.New("recovery staging evidence binding refused")
	}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	if !p.recoveryStageValid() {
		return empty, errors.New("acquisition set changed during recovery staging")
	}
	return contents.identity, nil
}

// copyRecoveryEngine reads the fixed original ZIP members afresh through their
// retained source owner. Bounds, hash and ZIP CRC are checked before a sink sees
// bytes. It grants no execution authority and does not reconstruct a script.
func copyRecoveryEngine(ctx context.Context, contents recoveryContents, sink interface {
	WriteEvidence(context.Context, string, []byte, string) error
}) error {
	names, err := baselineevidence.EngineHelpers(contents.inventory)
	if err != nil {
		return err
	}
	for _, name := range names {
		if err := ctx.Err(); err != nil {
			return err
		}
		raw, err := recoveryMemberBytes(contents.files[name], maxRecoveryInventory)
		if err != nil || len(raw) == 0 || recoveryDigest(raw) != contents.inventory[name] {
			return errors.New("original installer engine bytes differ")
		}
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := sink.WriteEvidence(ctx, name, raw, contents.inventory[name]); err != nil {
			return err
		}
	}
	return ctx.Err()
}

type recoveryStageReader struct {
	source io.Reader
	digest hash.Hash
	count  int64
}

func (r *recoveryStageReader) Read(p []byte) (int, error) {
	n, err := r.source.Read(p)
	r.count += int64(n)
	_, _ = r.digest.Write(p[:n])
	return n, err
}
