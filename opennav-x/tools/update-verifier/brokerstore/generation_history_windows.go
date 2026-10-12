package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"

	"golang.org/x/sys/windows"
)

// The baseline backend embeds this native owner. Only an actual retained native
// backend can supply it; neither Completion nor a caller-selected path can.
func (g *nativeInstalledGeneration) nativeHistoryGeneration() *nativeInstalledGeneration { return g }

type nativeGenerationSiblingAppend struct {
	owner     *nativeInstalledGeneration
	stage     string
	root      *nativePin
	identity  diskIdentity
	held      []generationHeldFile
	installed bool
}

func (a *nativeGenerationSiblingAppend) install() {
	// Caller holds Store.mu and no public generation operation can race this
	// non-failing ownership transfer. Preparation already checked every key.
	a.owner.siblings[a.stage] = a.root
	a.owner.pins[a.root] = a.identity
	a.owner.tree.pins = append(a.owner.tree.pins, a.root)
	a.owner.held = append(a.owner.held, a.held...)
	a.installed = true
}
func (a *nativeGenerationSiblingAppend) close() error {
	if a.installed {
		return nil
	}
	var err error
	for _, h := range a.held {
		err = errors.Join(err, h.file.Close())
	}
	a.held = nil
	if a.root != nil {
		err = errors.Join(err, windows.CloseHandle(a.root.handle))
		a.root = nil
	}
	return err
}

func (g *nativeInstalledGeneration) prepareSibling(ctx context.Context, backend generationBackend) (result generationSiblingAppend, resultErr error) {
	if ctx == nil {
		return nil, errors.New("live sibling append context required")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	proof, ok := backend.(interface {
		nativeHistoryGeneration() *nativeInstalledGeneration
	})
	if !ok {
		return nil, errors.New("actual native successor owner required")
	}
	source := proof.nativeHistoryGeneration()
	if source == nil || source == g || source.store != g.store || source.client != g.client || source.stage == g.stage || !validHex(source.stage, 32) || !source.completed || !source.readOnlyAttempted || source.closed || !g.completed || !g.readOnlyAttempted || g.closed || source.record == nil || source.marker == nil || source.tree.directories[""] == nil {
		return nil, errors.New("live completed same-store readonly distinct sibling required")
	}
	if _, exists := g.siblings[source.stage]; exists || len(g.siblings) >= maxProtectedGenerations-1 {
		return nil, errors.New("sibling already retained or bounded history exhausted")
	}
	if !sameGenerationIDs(source.parentIDs, g.parentIDs) {
		return nil, errors.New("successor parent historical identities differ")
	}
	for _, owner := range []*nativeInstalledGeneration{g, source} {
		if err := owner.check(); err != nil {
			return nil, err
		}
		if err := owner.checkHeldDigests(); err != nil {
			return nil, err
		}
	}
	// Read the successor's original private file through its retained owner. Its
	// recorded digest is independently corroborated again by checkHeldDigests.
	var length int64
	for _, h := range source.held {
		if h.file == source.record {
			length = h.length
		}
	}
	if length < 1 || length > maxGenerationRecordBytes {
		return nil, errors.New("bounded original successor completion required")
	}
	raw, err := readBaselineFile(ctx, source.record, length)
	if err != nil {
		return nil, err
	}
	var record generationRecord
	if err := json.Unmarshal(raw, &record); err != nil {
		return nil, err
	}
	canonical, err := json.Marshal(record)
	if err != nil || !bytes.Equal(canonical, raw) || record.Schema != 1 || record.ClientSID != g.client || record.Generation != source.stage || validateGenerationPurpose(record) != nil {
		return nil, errors.New("exact original completed successor purpose/binding differs")
	}
	origin := record
	origin.Directories = nil
	origin.Members = map[string]baselineMember{}
	originRaw, err := json.Marshal(origin)
	if err != nil || registrationDigest(originRaw) != source.origin {
		return nil, errors.New("retained successor origin differs")
	}
	markerRaw, err := readBaselineFile(ctx, source.marker, source.markerLength)
	if err != nil {
		return nil, err
	}
	digest, err := generationMarkerDigest(markerRaw, g.client, source.stage, source.origin)
	if err != nil || digest != registrationDigest(raw) || !bytes.Equal(markerRaw, source.markerBytes) {
		return nil, errors.Join(errors.New("retained successor completed marker differs"), err)
	}
	for key, id := range g.parentIDs {
		if record.Directories[key] != id {
			return nil, errors.New("successor original parent lineage differs")
		}
	}
	a := &nativeGenerationSiblingAppend{owner: g, stage: source.stage}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, a.close())
		}
	}()
	// Open only the two exact completed files supplied by this live successor;
	// no rescan, unknown sibling, present ACL or serialized receipt is adopted.
	for _, member := range []struct {
		name string
		raw  []byte
	}{{source.stage + ".once", markerRaw}, {source.stage + ".json", raw}} {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		f, err := baselineFile(g.registry, member.name, g.privateSD, false)
		if err != nil {
			return nil, err
		}
		a.held = append(a.held, generationHeldFile{file: f, length: int64(len(member.raw)), digest: registrationDigest(member.raw)})
		actual, err := readBaselineFile(ctx, f, int64(len(member.raw)))
		if err != nil || !bytes.Equal(actual, member.raw) {
			return nil, errors.Join(errors.New("independent sibling original bytes differ"), err)
		}
		original := source.record
		if member.name == source.stage+".once" {
			original = source.marker
		}
		if f.identity != original.identity {
			return nil, errors.New("independent sibling private full identity differs")
		}
	}
	name := g.generations.name + `\` + source.stage
	a.root, err = openPin(physical(name), name, g.tree.securityDir)
	if err != nil {
		return nil, err
	}
	a.identity, err = fileIdentity(a.root.handle)
	if err != nil || !a.identity.valid() || a.identity != record.Directories[""] || a.identity != source.pins[source.tree.directories[""]] || a.root.info.VolumeSerialNumber != g.generations.info.VolumeSerialNumber {
		return nil, errors.Join(errors.New("independent sibling full root identity differs"), err)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	return a, nil
}
