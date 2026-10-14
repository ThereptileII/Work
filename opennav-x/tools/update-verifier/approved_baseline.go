package verifier

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"sort"
	"strings"
	"time"

	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
)

// BaselineSink retains original evidence separately from installed app/docs.
// WriteEvidence must synchronously retain exact bytes under exclusive protected
// custody, verify resulting bytes, and accept only the two original sidecars and
// the fixed inventoried engine and commissioning-v1 helper paths.
// Implementing this interface alone establishes no filesystem custody.
type BaselineSink interface {
	RecoverySink
	WriteEvidence(context.Context, string, []byte, string) error
}

// ImportApprovedBaseline reparses an independently approved original operator
// request with the current initial-enrollment clock, validates its exact sidecar
// and payload pins, and streams app/docs without the portable marker. Approval
// must come from an independently authenticated operator, never from a client or
// adjacent self-declaration. The ZIP itself is not an approved package identity.
//
// The caller retains the source and protected destination owners throughout;
// concurrent mutation is forbidden. Failed imports return an empty identity;
// partial destination cleanup belongs to the sink owner. This call neither
// completes enrollment nor authorizes launch or claims a running installation.
func ImportApprovedBaseline(ctx context.Context, rawRequest []byte, independentlyApprovedSHA256 string, source io.ReaderAt, size int64, sink BaselineSink) (RecoveryIdentity, error) {
	return importApprovedBaselineAt(ctx, rawRequest, independentlyApprovedSHA256, source, size, sink, time.Now().UTC())
}

func importApprovedBaselineAt(ctx context.Context, rawRequest []byte, independentlyApprovedSHA256 string, source io.ReaderAt, size int64, sink BaselineSink, now time.Time) (RecoveryIdentity, error) {
	var empty RecoveryIdentity
	if ctx == nil || source == nil || sink == nil || size < 22 || size > maxDownloadBytes {
		return empty, errors.New("bounded approved baseline source and sink required")
	}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	request, err := brokerregistration.ParseApprovedRequest(rawRequest, independentlyApprovedSHA256, now)
	if err != nil {
		return empty, err
	}
	contents, identity, payload, err := verifiedApprovedBaselineContents(ctx, request, source, size)
	if err != nil {
		return empty, err
	}
	names := make([]string, 0, len(payload))
	for name := range payload {
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		// Helpers are not installed app/docs payload; preserve their original
		// bytes separately below through the exact evidence namespace.
		if baselineevidence.IsEngineHelper(name) || baselineevidence.IsCommissioningHelper(name) {
			continue
		}
		if err := ctx.Err(); err != nil {
			return empty, err
		}
		member := contents.files[name]
		reader, err := member.Open()
		if err != nil {
			return empty, err
		}
		memberSize := int64(member.UncompressedSize64)
		stream := &recoveryStageReader{source: downloadContextReader{ctx, io.LimitReader(reader, memberSize+1)}, digest: sha256.New()}
		writeErr := sink.WriteFile(ctx, name, stream, memberSize, payload[name])
		var tail [1]byte
		var n int
		var eof error
		if writeErr == nil && stream.count == memberSize {
			n, eof = stream.Read(tail[:])
		}
		closeErr := reader.Close()
		if err := ctx.Err(); err != nil {
			return empty, err
		}
		if writeErr != nil {
			return empty, errors.Join(errors.New("approved baseline sink refused"), writeErr)
		}
		if closeErr != nil || stream.count != memberSize || n != 0 || eof != io.EOF || hex.EncodeToString(stream.digest.Sum(nil)) != payload[name] {
			return empty, errors.New("approved baseline member length, hash or CRC differs")
		}
	}
	for _, e := range []struct {
		name string
		raw  []byte
		hash string
	}{{"FILE_SHA256.json", contents.inventoryBytes, identity.InventorySHA256}, {"installer/package.json", contents.packageBytes, identity.LifecycleManifestSHA256}} {
		if err := ctx.Err(); err != nil {
			return empty, err
		}
		if err := sink.WriteEvidence(ctx, e.name, bytes.Clone(e.raw), e.hash); err != nil {
			return empty, err
		}
	}
	if err := copyRecoveryHelpers(ctx, contents, sink); err != nil {
		return empty, errors.Join(err, ctx.Err())
	}
	if err := ctx.Err(); err != nil {
		return empty, err
	}
	return identity, nil
}

// verifiedApprovedBaselineContents has no sink or external callback. Both callers
// establish their own request authority before sharing the same bounded archive,
// full-member hash/CRC and exact original evidence verification.
func verifiedApprovedBaselineContents(ctx context.Context, request brokerregistration.Request, source io.ReaderAt, size int64) (recoveryContents, RecoveryIdentity, map[string]string, error) {
	r := approvedBaselineReaderAt{ctx, io.NewSectionReader(source, 0, size)}
	contents, err := parseRecoveryContents(ctx, r, size, releasepolicy.Policy{Version: request.Installed.Version, Commit: request.Installed.Commit, SupportedOpenCpn: []releasepolicy.OpenCpn{request.Stock.Identity()}})
	if canceled := ctx.Err(); canceled != nil {
		return recoveryContents{}, RecoveryIdentity{}, nil, canceled
	}
	if err != nil {
		return recoveryContents{}, RecoveryIdentity{}, nil, err
	}
	observed := map[string]string{}
	for name, hash := range contents.inventory {
		if ((strings.HasPrefix(name, "app/") || strings.HasPrefix(name, "docs/")) && name != "app/OPENNAV_PORTABLE_PREVIEW") || baselineevidence.IsEngineHelper(name) || baselineevidence.IsCommissioningHelper(name) {
			observed[name] = hash
		}
	}
	identity, payload, err := baselineevidence.Validate(ctx, contents.inventoryBytes, contents.packageBytes, contents.productBytes, observed, request.Installed)
	if err != nil {
		return recoveryContents{}, RecoveryIdentity{}, nil, err
	}
	if err := ctx.Err(); err != nil {
		return recoveryContents{}, RecoveryIdentity{}, nil, err
	}
	return contents, identity, payload, nil
}

// RetainedBaselineMember is an owned value describing an authenticated original
// app/docs member or evidence/helper member. It contains no source handle,
// executable bytes, destination, or authority to mutate an installation.
type RetainedBaselineMember struct {
	Name   string
	Size   int64
	SHA256 string
}

// InspectRetainedApprovedBaseline checks original baseline bytes against a
// separately approved request retained by a completed protected registration.
// Original bootstrap expiry is irrelevant to this historical identity check;
// initial enrollment must still use ImportApprovedBaseline with its live clock.
//
// Every archive member, including excluded profile/portable members, is consumed
// and checked for exact length, hash and CRC. The returned sorted, caller-owned
// values describe only the importer's app/docs and original evidence/helper
// namespace. They grant no install, delete, launch, enrollment or trust authority.
// This function performs no writes and invokes no sink or external callback.
// The caller must retain original source custody and forbid concurrent mutation.
func InspectRetainedApprovedBaseline(ctx context.Context, rawRequest []byte, approvedSHA256 string, source io.ReaderAt, size int64) ([]RetainedBaselineMember, error) {
	if ctx == nil || source == nil || size < 22 || size > maxDownloadBytes {
		return nil, errors.New("bounded retained approved baseline source required")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	request, err := brokerregistration.ParseRetainedApprovedRequest(rawRequest, approvedSHA256)
	if err != nil {
		return nil, err
	}
	contents, identity, payload, err := verifiedApprovedBaselineContents(ctx, request, source, size)
	if err != nil {
		return nil, err
	}
	members := make([]RetainedBaselineMember, 0, len(payload)+2)
	for name, hash := range payload {
		members = append(members, RetainedBaselineMember{Name: name, Size: int64(contents.files[name].UncompressedSize64), SHA256: hash})
	}
	members = append(members,
		RetainedBaselineMember{Name: "FILE_SHA256.json", Size: int64(len(contents.inventoryBytes)), SHA256: identity.InventorySHA256},
		RetainedBaselineMember{Name: "installer/package.json", Size: int64(len(contents.packageBytes)), SHA256: identity.LifecycleManifestSHA256})
	sort.Slice(members, func(i, j int) bool { return members[i].Name < members[j].Name })
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return members, nil
}

type approvedBaselineReaderAt struct {
	ctx    context.Context
	source io.ReaderAt
}

func (r approvedBaselineReaderAt) ReadAt(p []byte, off int64) (int, error) {
	if err := r.ctx.Err(); err != nil {
		return 0, err
	}
	return r.source.ReadAt(p, off)
}
