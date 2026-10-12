package brokerstore

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"runtime"
	"sort"
	"strings"

	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/brokerregistration"
)

const maxCandidateEvidenceBytes = 4 << 20

type candidateInventoryBackend interface {
	candidateInventory(names []string, acquisition bool) error
}

// WriteEvidence creates only the two fixed original recovery sidecars and
// inventoried engine/commissioning-v1 helpers using the same exclusive, retained
// native file custody as payload. It never serializes
// a replacement inventory or manifest. Supplied hashes authenticate nothing by
// themselves; StageRecovery supplies bytes from its live signed acquisition.
func (c *Candidate) WriteEvidence(ctx context.Context, name string, raw []byte, digest string) (err error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c.mu.Lock()
	defer c.mu.Unlock()
	if err = c.ready(); err != nil {
		return err
	}
	defer func() {
		if err != nil {
			c.poisoned = true
		}
	}()
	if ctx == nil || !baselineEvidenceName(name) || len(raw) == 0 || len(raw) > maxCandidateEvidenceBytes || !validHex(digest, 64) || c.recoveryEvidence != nil || len(c.files) >= MaxFiles || int64(len(raw)) > MaxPayloadBytes-c.bytes {
		return errors.New("bounded exact original candidate evidence required")
	}
	if err = ctx.Err(); err != nil {
		return err
	}
	// The evidence paths are exact exceptions, never a relaxation of memberPath.
	if c.names == nil {
		return errors.New("candidate namespace unavailable")
	}
	parts := strings.Split(name, "/")
	for i := range parts {
		current := strings.Join(parts[:i+1], "/")
		key := strings.ToLower(current)
		if old, ok := c.names[key]; ok && (old != current || c.fileNames[key] || i == len(parts)-1) {
			return errors.New("candidate evidence duplicate, alias or parent collision")
		}
		c.names[key] = current
		if i == len(parts)-1 {
			c.fileNames[key] = true
		}
	}
	owned := bytes.Clone(raw)
	file, err := c.backend.create(name)
	if err != nil {
		return err
	}
	record := fileRecord{file: file, length: int64(len(owned)), digest: digest}
	c.files = append(c.files, record)
	if err = c.copyFile(ctx, file, bytes.NewReader(owned), record.length, digest); err != nil {
		return err
	}
	if c.recoveryMembers == nil {
		c.recoveryMembers = map[string]fileRecord{}
	}
	c.recoveryMembers[name] = record
	c.bytes += record.length
	return nil
}

// CompleteRecoveryEvidence binds the observed originals and complete payload set
// to the identity returned by authenticated StageRecovery. It is a custody and
// consistency check, not standalone signing, installation or launch authority.
// Every later Seal rechecks this same identity against held original bytes.
func (c *Candidate) CompleteRecoveryEvidence(ctx context.Context, identity baselineevidence.Identity) (err error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	c.mu.Lock()
	defer c.mu.Unlock()
	if err = c.ready(); err != nil {
		return err
	}
	defer func() {
		if err != nil {
			c.poisoned = true
		}
	}()
	if ctx == nil || c.recoveryEvidence != nil {
		return errors.New("one recovery evidence completion required")
	}
	if err = c.validateRecoveryEvidence(ctx, identity); err != nil {
		return err
	}
	c.recoveryEvidence = &identity
	return nil
}

func (c *Candidate) readRecoveryMember(ctx context.Context, name string, limit int64) ([]byte, error) {
	entry, ok := c.recoveryMembers[name]
	if !ok || entry.length <= 0 || entry.length > limit {
		return nil, errors.New("required bounded original recovery member absent")
	}
	if err := verifyFile(ctx, c.store, entry.file, entry.length, entry.digest); err != nil {
		return nil, err
	}
	if _, err := entry.file.Seek(0, io.SeekStart); err != nil {
		return nil, err
	}
	raw, err := io.ReadAll(io.LimitReader(checkedReader{ctx: ctx, store: c.store, reader: entry.file}, entry.length+1))
	if err != nil {
		return nil, err
	}
	if int64(len(raw)) != entry.length {
		return nil, errors.New("held original recovery evidence length differs")
	}
	// Recheck the bytes actually used by the parser as well as the prior readback.
	sum := sha256.Sum256(raw)
	if digest := hex.EncodeToString(sum[:]); digest != entry.digest {
		return nil, errors.New("held original recovery evidence hash differs")
	}
	if err = entry.file.check(entry.length); err != nil {
		return nil, err
	}
	return raw, nil
}

func (c *Candidate) validateRecoveryEvidence(ctx context.Context, identity baselineevidence.Identity) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if c.acquisition != nil && (c.acquisition.plan == nil || c.acquisition.plan.Version != identity.Version || c.acquisition.plan.Commit != identity.Commit) {
		return errors.New("candidate acquisition plan and recovery identity differ")
	}
	if !identity.LifecycleManifestAvailable {
		return errors.New("original Lifecycle manifest required")
	}
	inventory, err := c.readRecoveryMember(ctx, "FILE_SHA256.json", maxCandidateEvidenceBytes)
	if err != nil {
		return err
	}
	manifest, err := c.readRecoveryMember(ctx, "installer/package.json", maxCandidateEvidenceBytes)
	if err != nil {
		return err
	}
	product, err := c.readRecoveryMember(ctx, "docs/PRODUCT_BUILD.json", 32<<10)
	if err != nil {
		return err
	}
	observed := map[string]string{}
	var names []string
	for name, entry := range c.recoveryMembers {
		if err := verifyFile(ctx, c.store, entry.file, entry.length, entry.digest); err != nil {
			return err
		}
		names = append(names, name)
		if !baselineSidecarName(name) {
			observed[name] = entry.digest
		}
	}
	expected := brokerregistration.Installed{Version: identity.Version, Commit: identity.Commit,
		PackageSHA256: identity.LifecycleManifestSHA256, InventorySHA256: identity.InventorySHA256,
		ProductBuildSHA256: identity.ProductBuildSHA256, ExecutableSHA256: identity.ExecutableSHA256}
	actual, _, err := baselineevidence.Validate(ctx, inventory, manifest, product, observed, expected)
	if err != nil {
		return err
	}
	if actual != identity {
		return errors.New("candidate original recovery identity differs")
	}
	backend, ok := c.backend.(candidateInventoryBackend)
	if !ok {
		return ErrNativeRequired
	}
	sort.Strings(names)
	if err := backend.candidateInventory(names, c.acquisition != nil); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	return c.backend.check()
}
