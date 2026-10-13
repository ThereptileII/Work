package brokerstore

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"strings"
)

type decommissionInventoryOpeners struct {
	generation func(context.Context, string, string) (generationReopenBackend, generationReopenEvidence, error)
	stage      func(context.Context, generationBackend, generationRecord, []byte) (stageSealReaderBackend, []byte, []byte, []byte, error)
}
type decommissionCustodyFile struct {
	file   candidateFile
	length int64
	digest string
}

func (f decommissionCustodyFile) check() error {
	if err := f.file.check(f.length); err != nil {
		return err
	}
	if _, err := f.file.Seek(0, io.SeekStart); err != nil {
		return err
	}
	hash := sha256.New()
	count, err := io.CopyBuffer(hash, io.LimitReader(f.file, f.length+1), make([]byte, 64<<10))
	if err != nil || count != f.length || hex.EncodeToString(hash.Sum(nil)) != f.digest {
		return errors.Join(errors.New("retained original inventory bytes changed"), err)
	}
	return f.file.check(f.length)
}
func (d *nativeDecommission) retainOwnedInventory(ctx context.Context, openers decommissionInventoryOpeners) error {
	if !d.rootInventory.deployed {
		return nil
	}
	if openers.generation == nil || (len(d.rootInventory.stages) > 0 && openers.stage == nil) {
		return ErrNativeRequired
	}
	d.extraPins = map[*nativePin]diskIdentity{}
	d.extraInventories = map[*nativePin]map[string]bool{}
	current := d.binding.GenerationID
	if current == "" {
		current = d.baseline
	}
	generations := map[string]*nativeReopenedGeneration{}
	d.opened = generations
	records := map[string][]byte{}
	openGeneration := func(stage string) (*nativeReopenedGeneration, error) {
		if g := generations[stage]; g != nil {
			return g, nil
		}
		backend, e, err := openers.generation(ctx, d.binding.ClientSID, stage)
		if err != nil {
			return nil, err
		}
		g, ok := backend.(*nativeReopenedGeneration)
		if !ok || g == nil || g.store != d.store {
			if backend != nil {
				err = errors.Join(err, backend.close())
			}
			return nil, errors.Join(ErrNativeRequired, err)
		}
		d.owners = append(d.owners, g.close)
		generations[stage] = g
		records[stage] = bytes.Clone(e.completion)
		for _, held := range g.held {
			if strings.HasPrefix(held.file.pin.name, g.registry.name+`\`) && strings.HasSuffix(held.file.pin.name, ".json") {
				raw, err := decommissionReadExact(held.file, held.length, nil)
				if err != nil {
					return nil, err
				}
				var record generationRecord
				if json.Unmarshal(raw, &record) != nil {
					return nil, errors.New("original generation record unavailable")
				}
				records[record.Generation] = raw
			}
		}
		return g, nil
	}
	if _, err := openGeneration(current); err != nil {
		return err
	}
	fences := map[string][]byte{}
	for _, id := range d.rootInventory.executions {
		backend, fence, raw, err := openNativeExecutionJournal(ctx, d.store, d.binding.ClientSID, id)
		if err != nil {
			return err
		}
		reader, ok := backend.(*nativeExecutionJournalReader)
		if !ok {
			return ErrNativeRequired
		}
		d.owners = append(d.owners, reader.close)
		origin, _, err := decodeReadExecutionJournal(d.binding.ClientSID, id, fence, raw)
		if err != nil {
			return err
		}
		if !bytes.Equal(records[origin.Generation], origin.OriginalCompletion) {
			return errors.New("execution original completion is outside retained generation inventory")
		}
		fences[id] = bytes.Clone(fence)
		d.retainPair(reader.journal, int64(len(fence)), registrationDigest(fence), int64(len(raw)), registrationDigest(raw))
	}
	for _, id := range d.rootInventory.profiles {
		fence := fences[id]
		if len(fence) == 0 {
			return errors.New("profile archive has no original execution pair")
		}
		backend, marker, err := openNativeExecutionProfileBackup(ctx, d.store, d.binding.ClientSID, id)
		if err != nil {
			return err
		}
		archive, ok := backend.(*nativeExecutionProfileBackup)
		if !ok {
			return ErrNativeRequired
		}
		d.owners = append(d.owners, archive.close)
		origin, _, err := decodeExecutionProfileBackup(marker, fence)
		if err != nil || !bytes.Equal(origin.OriginalJournalFence, fence) {
			return errors.Join(errors.New("profile archive original execution fence differs"), err)
		}
		d.retainPair(archive.pair, int64(len(marker)), registrationDigest(marker), archive.size, archive.digest)
	}
	for _, stage := range d.rootInventory.stages {
		raw := records[stage]
		if len(raw) == 0 {
			return errors.New("stage seal has no original completed generation")
		}
		var record generationRecord
		if json.Unmarshal(raw, &record) != nil || record.Purpose != string(GenerationPurposeSignedCandidate) {
			return errors.New("stage seal requires original signed candidate generation")
		}
		g, err := openGeneration(stage)
		if err != nil {
			return err
		}
		stageOwner, _, _, _, err := openers.stage(ctx, g, record, raw)
		if err != nil {
			return err
		}
		nativeStage, ok := stageOwner.(*nativeStageSeal)
		if !ok || nativeStage == nil || nativeStage.generation != g || !nativeStage.attached {
			return ErrNativeRequired
		}
		// openNativeStageSeal transfers its exact fence/seal/receipt files into
		// g.held and every original source pin into g.ancestors/g.pins before
		// setting attached. The remaining pair parents borrow Store.pins. Only
		// g.close owns these transferred resources; closing stageOwner as well
		// would close the same file handles twice. The returned owner is checked
		// here so an opener cannot silently discard an unattached capability.
	}
	// Snapshot only originals already authenticated by the existing readers. Their
	// owners remain retained. After the fence, custody uses these same handles and
	// full IDs/hashes; ordinary reader/store validation remains permanently denied.
	for _, g := range generations {
		for pin, id := range g.pins {
			d.extraPins[pin] = id
		}
		for pin, id := range g.baseline.pins {
			d.extraPins[pin] = id
		}
		for _, held := range g.held {
			d.extraFiles = append(d.extraFiles, decommissionCustodyFile{held.file, held.length, held.digest})
		}
		for _, pin := range []*nativePin{g.clientRoot, g.generations, g.registry} {
			entries, err := boundedEntries(pin, maxProtectedGenerations*2+1)
			if err != nil {
				return err
			}
			names := map[string]bool{}
			for _, entry := range entries {
				names[entry.Name()] = true
			}
			d.extraInventories[pin] = names
		}
	}
	return nil
}

func (d *nativeDecommission) retainPair(pair *nativeExecutionJournal, fenceLength int64, fenceDigest string, dataLength int64, dataDigest string) {
	for name, pin := range pair.parents {
		d.extraPins[pin] = pair.ids.Parents[name]
	}
	d.extraFiles = append(d.extraFiles, decommissionCustodyFile{&nativeBaselineFile{nativeFile: pair.fence, identity: pair.ids.Fence}, fenceLength, fenceDigest}, decommissionCustodyFile{&nativeBaselineFile{nativeFile: pair.journal, identity: pair.ids.Journal}, dataLength, dataDigest})
}
