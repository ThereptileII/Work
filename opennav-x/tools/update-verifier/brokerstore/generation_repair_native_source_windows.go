//go:build windows

package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"sort"

	"golang.org/x/sys/windows"
)

// Only the production factory resolves the real distinct client account and
// fixed Applications root. Native temporary fixtures supply those two factories
// here; source metadata, stage seals, full IDs, hashing and ownership use this
// same body. This private seam cannot be selected by an installed caller.
type nativeRepairGenerationOpener func(context.Context, string, string) (generationReopenBackend, generationReopenEvidence, error)

func openNativeRepairSource(ctx context.Context, n *nativeStore, sid *windows.SID, client, generation string, openGeneration nativeRepairGenerationOpener) (backend repairSourceBackend, current generationRecord, currentRaw []byte, original generationRecord, originalRaw []byte, resultErr error) {
	b, e, err := openGeneration(ctx, client, generation)
	if err != nil {
		return nil, current, nil, original, nil, err
	}
	g, ok := b.(*nativeReopenedGeneration)
	if !ok {
		return nil, current, nil, original, nil, errors.Join(ErrNativeRequired, b.close())
	}
	owner := &nativeRepairSource{store: n, current: g, original: g, members: map[string]candidateFile{}}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, owner.close())
		}
	}()
	current, err = validateGenerationReopenEvidence(e, client, generation)
	if err != nil {
		return nil, current, nil, original, nil, err
	}
	currentRaw = bytes.Clone(e.completion)
	original = current
	originalRaw = bytes.Clone(currentRaw)
	if current.Repair != nil {
		var source generationRecord
		if json.Unmarshal(current.Repair.SourceCompletion, &source) != nil {
			return nil, current, nil, original, nil, ErrInstalledGenerationUncertain
		}
		reopened, proof, err := openGeneration(ctx, client, source.Generation)
		if err != nil {
			return nil, current, nil, original, nil, err
		}
		actual, ok := reopened.(*nativeReopenedGeneration)
		if !ok {
			return nil, current, nil, original, nil, errors.Join(ErrNativeRequired, reopened.close())
		}
		owner.original = actual
		if !bytes.Equal(proof.completion, current.Repair.SourceCompletion) {
			return nil, current, nil, original, nil, ErrInstalledGenerationUncertain
		}
		original, err = validateGenerationReopenEvidence(proof, client, source.Generation)
		if err != nil {
			return nil, current, nil, original, nil, err
		}
		originalRaw = bytes.Clone(proof.completion)
	}
	if original.Repair != nil {
		return nil, current, nil, original, nil, ErrInstalledGenerationUncertain
	}
	owner.record = original
	sourceMembers := map[string]baselineMember{}
	for name, m := range original.Members {
		m.Identity = original.SourceMembers[name]
		sourceMembers[name] = m
	}
	if original.Purpose == approvedBaselineGenerationPurpose {
		owner.members, err = owner.original.baseline.openMembers(sourceMembers, original.SourceDirectories)
		owner.retainMembers(owner.members) // includes the partial map on failure
		if err != nil {
			return nil, current, nil, original, nil, err
		}
	} else if original.Purpose == string(GenerationPurposeSignedCandidate) {
		seal, _, _, _, err := openNativeStageSeal(ctx, owner.original, sid, original, originalRaw)
		if err != nil {
			return nil, current, nil, original, nil, err
		}
		owner.seal = seal
		native, ok := seal.(*nativeStageSeal)
		if !ok || native.source == nil || !native.attached {
			return nil, current, nil, original, nil, ErrNativeRequired
		}
		owner.candidate = native.source
		names := make([]string, 0, len(sourceMembers))
		for name := range sourceMembers {
			names = append(names, name)
		}
		sort.Strings(names)
		for _, name := range names {
			parent, component := splitBaselineName(name)
			pin := native.source.directories[parent]
			if pin == nil {
				return nil, current, nil, original, nil, ErrInstalledGenerationUncertain
			}
			file, err := baselineFile(pin, component, native.source.securityFile, false)
			if err != nil {
				return nil, current, nil, original, nil, err
			}
			owner.owned = append(owner.owned, file)
			owner.members[name] = file
			if file.identity != sourceMembers[name].Identity || file.check(sourceMembers[name].Length) != nil {
				return nil, current, nil, original, nil, ErrInstalledGenerationUncertain
			}
		}
	} else {
		return nil, current, nil, original, nil, ErrInstalledGenerationUncertain
	}
	if err = owner.check(); err != nil {
		return nil, current, nil, original, nil, err
	}
	return owner, current, currentRaw, original, originalRaw, nil
}
