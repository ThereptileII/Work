package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"sort"
	"strings"

	"golang.org/x/sys/windows"
)

type nativeStageSeal struct {
	generation           *nativeReopenedGeneration
	source               *nativeCandidate
	pair                 *nativeExecutionJournal
	origin               stageSealOrigin
	marker, raw, receipt []byte
	receiptFile          *nativeFile
	receiptRecord        baselineMember
	client, stage        string
	attached             bool
}

func stageSealComponent(client, stage string) string { return client + ".stage." + stage }
func stageSealNames(ctx context.Context, n *nativeStore, client, stage string, fresh bool) error {
	names, err := nativeExecutionJournalNames(ctx, n)
	if err != nil {
		return err
	}
	if err = stageSealPairNames(client, stage, names, fresh); err != nil {
		return err
	}
	count := 0
	for _, name := range names {
		if strings.Contains(strings.ToLower(name), ".stage.") {
			count++
		}
	}
	if fresh && count+2 > 2*MaxCandidates {
		return errors.New("permanent stage seal capacity exhausted")
	}
	return nil
}
func stageSealNativePair(n *nativeStore) (*nativeExecutionJournal, error) {
	p := &nativeExecutionJournal{store: n, ids: executionJournalIDs{Parents: map[string]diskIdentity{}}, parents: map[string]*nativePin{}}
	for _, pin := range n.pins {
		id, err := fileIdentity(pin.handle)
		if err != nil {
			return nil, err
		}
		p.ids.Parents[pin.name] = id
		p.parents[pin.name] = pin
	}
	return p, nil
}
func (n *nativeStore) prepareStageSeal(ctx context.Context, backend generationBackend, source candidateBackend, record generationRecord, raw []byte) (stageSealWriter, error) {
	g, ok := backend.(*nativeReopenedGeneration)
	c, okc := source.(*nativeCandidate)
	if !ok || !okc || g == nil || c == nil || g.store != n || c.store != n || c.client != record.ClientSID || c.stageID != record.Generation || g.client != c.client || g.stage != c.stageID || record.Purpose != string(GenerationPurposeSignedCandidate) {
		return nil, ErrNativeRequired
	}
	if err := n.check(); err != nil {
		return nil, err
	}
	if _, err := clientSID(c.client, n.broker); err != nil {
		return nil, err
	}
	return createNativeStageSeal(ctx, g, c, record, raw)
}
func createNativeStageSeal(ctx context.Context, g *nativeReopenedGeneration, c *nativeCandidate, record generationRecord, raw []byte) (result stageSealWriter, resultErr error) {
	n := g.store
	n.beginMu.Lock()
	defer n.beginMu.Unlock()
	if ctx == nil {
		return nil, ErrGenerationStageSealUncertain
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := g.check(); err != nil {
		return nil, err
	}
	if err := executionReadback(g.record.nativeFile, raw); err != nil {
		return nil, err
	}
	if err := c.check(); err != nil {
		return nil, err
	}
	if err := stageSealNames(ctx, n, c.client, c.stageID, true); err != nil {
		return nil, err
	}
	// All source directories were recorded before materialization. Preserve their
	// actual creation identities; present descriptors alone cannot mint this seal.
	if err := stageSealSourceIDs(c, record.SourceDirectories); err != nil {
		return nil, err
	}
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		return nil, err
	}
	pair, err := stageSealNativePair(n)
	if err != nil {
		return nil, err
	}
	b := &nativeStageSeal{generation: g, source: c, pair: pair, client: c.client, stage: c.stageID}
	mutating := false
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, b.close())
			if mutating {
				resultErr = errors.Join(ErrGenerationStageSealUncertain, resultErr)
			}
		}
	}()
	component := stageSealComponent(c.client, c.stageID)
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	mutating = true
	pair.fence, err = createRegistrationFile(n.registrations, component+".once", sd)
	if err != nil {
		return nil, err
	}
	if err = pair.fence.Sync(); err != nil {
		return nil, err
	}
	if err = executionReadback(pair.fence, nil); err != nil {
		return nil, err
	}
	pair.ids.Fence, err = fileIdentity(pair.fence.pin.handle)
	if err != nil {
		return nil, err
	}
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	pair.journal, err = createRegistrationFile(n.registrations, component+".json", sd)
	if err != nil {
		return nil, err
	}
	pair.ids.Journal, err = fileIdentity(pair.journal.pin.handle)
	if err != nil || !pair.ids.valid() {
		return nil, errors.Join(ErrGenerationStageSealUncertain, err)
	}
	b.origin = stageSealOrigin{1, "original-generation-stage-seal", bytes.Clone(raw), pair.ids}
	origin, err := json.Marshal(b.origin)
	if err != nil || len(origin) > maxStageSealOriginBytes {
		return nil, errors.Join(ErrGenerationStageSealUncertain, err)
	}
	b.marker = append(origin, '\n')
	if err = executionWrite(ctx, pair.fence, nil, b.marker); err != nil {
		return nil, err
	}
	if err = pair.checkIdentities(); err != nil {
		return nil, err
	}
	return b, nil
}
func stageSealSourceIDs(c *nativeCandidate, ids map[string]diskIdentity) error {
	if len(c.directories) != len(ids) || len(ids) == 0 {
		return errors.New("original source directory set differs")
	}
	for name, pin := range c.directories {
		if err := pin.check(); err != nil {
			return err
		}
		id, err := fileIdentity(pin.handle)
		if err != nil || id != ids[name] {
			return errors.Join(errors.New("original source directory identity differs"), err)
		}
	}
	return nil
}
func (b *nativeStageSeal) complete(ctx context.Context, receipt fileRecord) error {
	if b.attached || len(b.raw) != 0 || ctx == nil {
		return ErrGenerationStageSealUncertain
	}
	var record generationRecord
	if json.Unmarshal(b.origin.OriginalCompletion, &record) != nil {
		return ErrGenerationStageSealUncertain
	}
	if err := errors.Join(ctx.Err(), b.generation.check(), b.source.check(), stageSealSourceIDs(b.source, record.SourceDirectories), b.pair.checkIdentities()); err != nil {
		return err
	}
	file, ok := receipt.file.(*nativeFile)
	if !ok || file == nil || file.parent != b.source.directories[""] || file.pin.name != file.parent.name+`\`+ReceiptFilename {
		return errors.New("actual original native receipt required")
	}
	if receipt.length <= 0 || receipt.length > MaxReceiptBytes || !validHex(receipt.digest, 64) {
		return ErrGenerationStageSealUncertain
	}
	id, err := fileIdentity(file.pin.handle)
	if err != nil {
		return err
	}
	b.receiptFile = file
	b.receiptRecord = baselineMember{Length: receipt.length, SHA256: receipt.digest, Identity: id}
	original := bytes.TrimSuffix(b.marker, []byte{'\n'})
	raw, err := json.Marshal(stageSealRecord{1, "completed-generation-stage-seal", registrationDigest(original), b.receiptRecord})
	if err != nil || len(raw) > maxStageSealRecordBytes {
		return errors.Join(ErrGenerationStageSealUncertain, err)
	}
	if err = b.checkReceipt(); err != nil {
		return err
	}
	if err = executionWrite(ctx, b.pair.journal, nil, raw); err != nil {
		return err
	}
	if err = errors.Join(ctx.Err(), b.checkReceipt(), b.generation.check(), b.pair.checkIdentities()); err != nil {
		return err
	}
	next := append(append(bytes.Clone(b.marker), []byte(registrationDigest(raw))...), '\n')
	// Final successful durable marker wins over late cancellation. A failed
	// publication/readback remains uncertain even when valid bytes reached disk.
	if err = executionWrite(context.WithoutCancel(ctx), b.pair.fence, b.marker, next); err != nil {
		return err
	}
	b.marker, b.raw = next, bytes.Clone(raw)
	return b.validate(context.WithoutCancel(ctx))
}

func (b *nativeStageSeal) checkReceipt() error {
	if b.receiptFile == nil {
		return ErrGenerationStageSealUncertain
	}
	if err := b.receiptFile.check(b.receiptRecord.Length); err != nil {
		return err
	}
	id, err := fileIdentity(b.receiptFile.pin.handle)
	if err != nil || id != b.receiptRecord.Identity {
		return errors.Join(ErrGenerationStageSealUncertain, err)
	}
	if _, err = b.receiptFile.Seek(0, io.SeekStart); err != nil {
		return err
	}
	raw, err := io.ReadAll(io.LimitReader(b.receiptFile, b.receiptRecord.Length+1))
	if err != nil || int64(len(raw)) != b.receiptRecord.Length || registrationDigest(raw) != b.receiptRecord.SHA256 {
		return errors.Join(ErrGenerationStageSealUncertain, err)
	}
	if b.receipt != nil && !bytes.Equal(raw, b.receipt) {
		return ErrGenerationStageSealUncertain
	}
	return b.receiptFile.check(b.receiptRecord.Length)
}
func (b *nativeStageSeal) validate(ctx context.Context) error {
	if ctx == nil {
		return ErrGenerationStageSealUncertain
	}
	if err := errors.Join(ctx.Err(), b.generation.check(), b.pair.checkIdentities()); err != nil {
		return err
	}
	if err := stageSealNames(ctx, b.generation.store, b.client, b.stage, false); err != nil {
		return err
	}
	if err := executionReadback(b.pair.fence, b.marker); err != nil {
		return err
	}
	if err := executionReadback(b.pair.journal, b.raw); err != nil {
		return err
	}
	if err := b.checkReceipt(); err != nil {
		return err
	}
	return ctx.Err()
}
func (b *nativeStageSeal) close() error { return b.pair.close() }
func (n *nativeStore) openStageSeal(ctx context.Context, backend generationBackend, record generationRecord, raw []byte) (stageSealReaderBackend, []byte, []byte, []byte, error) {
	g, ok := backend.(*nativeReopenedGeneration)
	if !ok || g == nil || g.store != n || g.client != record.ClientSID || g.stage != record.Generation {
		return nil, nil, nil, nil, ErrNativeRequired
	}
	if err := n.check(); err != nil {
		return nil, nil, nil, nil, err
	}
	sid, err := clientSID(record.ClientSID, n.broker)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	return openNativeStageSeal(ctx, g, sid, record, raw)
}
func readStageSealFile(file *nativeFile, bound int64) ([]byte, error) {
	length := int64(uint64(file.pin.info.FileSizeHigh)<<32 | uint64(file.pin.info.FileSizeLow))
	if length <= 0 || length > bound {
		return nil, ErrGenerationStageSealUncertain
	}
	raw, err := io.ReadAll(io.LimitReader(file, length+1))
	if err != nil || int64(len(raw)) != length {
		return nil, errors.Join(ErrGenerationStageSealUncertain, err)
	}
	return raw, executionReadback(file, raw)
}
func openNativeStageSeal(ctx context.Context, g *nativeReopenedGeneration, sid *windows.SID, record generationRecord, raw []byte) (result stageSealReaderBackend, marker, seal, receipt []byte, resultErr error) {
	n := g.store
	n.beginMu.Lock()
	defer n.beginMu.Unlock()
	if ctx == nil || len(g.held) > 2*maxProtectedGenerations+32 {
		return nil, nil, nil, nil, ErrGenerationStageSealUncertain
	}
	if err := stageSealNames(ctx, n, g.client, g.stage, false); err != nil {
		return nil, nil, nil, nil, err
	}
	component := stageSealComponent(g.client, g.stage)
	for _, held := range g.held {
		if held.file.pin.name == n.registrations.name+`\`+component+".once" {
			return nil, nil, nil, nil, errors.New("stage seal already retained by generation")
		}
	}
	pair, err := stageSealNativePair(n)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	source := &nativeCandidate{store: n, client: g.client, stageID: g.stage, directories: map[string]*nativePin{}}
	source.securityDir, err = descriptor(n.broker, sid, true)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	source.securityFile, err = descriptor(n.broker, sid, false)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	b := &nativeStageSeal{generation: g, source: source, pair: pair, client: g.client, stage: g.stage}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(ErrGenerationStageSealUncertain, resultErr, b.close())
			if b.receiptFile != nil {
				resultErr = errors.Join(resultErr, b.receiptFile.Close())
			}
			resultErr = errors.Join(resultErr, source.close())
		}
	}()
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	pair.fence, err = openAcquisitionFile(n.registrations, component+".once", sd, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, windows.OPEN_EXISTING)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	pair.ids.Fence, err = fileIdentity(pair.fence.pin.handle)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	marker, err = readStageSealFile(pair.fence, maxStageSealOriginBytes+66)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	pair.journal, err = openAcquisitionFile(n.registrations, component+".json", sd, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, windows.OPEN_EXISTING)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	pair.ids.Journal, err = fileIdentity(pair.journal.pin.handle)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	seal, err = readStageSealFile(pair.journal, maxStageSealRecordBytes)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	origin, completed, err := decodeStageSeal(marker, seal, raw)
	if err != nil || !sameExecutionJournalIDs(origin.Identities, pair.ids) {
		return nil, nil, nil, nil, errors.Join(ErrGenerationStageSealUncertain, err)
	}
	// Reopen only the fixed original stage, then each original plain directory.
	// The already held machine Store ancestors remain lifetime owners as well.
	names := make([]string, 0, len(record.SourceDirectories))
	for name := range record.SourceDirectories {
		names = append(names, name)
	}
	sort.Slice(names, func(i, j int) bool {
		if strings.Count(names[i], "/") != strings.Count(names[j], "/") {
			return strings.Count(names[i], "/") < strings.Count(names[j], "/")
		}
		return names[i] < names[j]
	})
	for _, name := range names {
		parent, leaf := n.candidates, g.stage
		if name != "" {
			if err = stageSealDirectoryName(name); err != nil {
				return nil, nil, nil, nil, err
			}
			parentName, base := splitReopenName(name)
			parent = source.directories[parentName]
			leaf = base
		}
		if parent == nil {
			return nil, nil, nil, nil, ErrGenerationStageSealUncertain
		}
		path := parent.name + `\` + leaf
		pin, e := openPin(physical(path), path, source.securityDir)
		if e != nil {
			return nil, nil, nil, nil, e
		}
		source.pins = append(source.pins, pin)
		source.directories[name] = pin
	}
	if err = stageSealSourceIDs(source, record.SourceDirectories); err != nil {
		return nil, nil, nil, nil, err
	}
	b.receiptFile, err = openAcquisitionFile(source.directories[""], ReceiptFilename, source.securityFile, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, windows.OPEN_EXISTING)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	b.origin, b.marker, b.raw, b.receiptRecord = origin, bytes.Clone(marker), bytes.Clone(seal), completed.Receipt
	receipt, err = readStageSealFile(b.receiptFile, MaxReceiptBytes)
	if err != nil {
		return nil, nil, nil, nil, err
	}
	b.receipt = bytes.Clone(receipt)
	if err = b.validate(ctx); err != nil {
		return nil, nil, nil, nil, err
	}
	// Transfer complete checked readonly resources only. No reader owns or closes
	// Store ancestors. Generation.close drains every attached file and source pin.
	attachedFiles := []generationHeldFile{}
	for _, f := range []*nativeFile{pair.fence, pair.journal, b.receiptFile} {
		id, e := fileIdentity(f.pin.handle)
		if e != nil {
			return nil, nil, nil, nil, e
		}
		content := marker
		if f == pair.journal {
			content = seal
		}
		if f == b.receiptFile {
			content = receipt
		}
		attachedFiles = append(attachedFiles, generationHeldFile{&nativeBaselineFile{nativeFile: f, identity: id}, int64(len(content)), registrationDigest(content)})
	}
	g.held = append(g.held, attachedFiles...)
	for _, pin := range source.pins {
		id := record.SourceDirectories[""]
		for name, p := range source.directories {
			if p == pin {
				id = record.SourceDirectories[name]
			}
		}
		g.pins[pin] = id
		g.ancestors = append(g.ancestors, pin)
	}
	source.pins = nil
	b.attached = true
	return b, marker, seal, receipt, nil
}
