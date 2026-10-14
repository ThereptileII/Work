//go:build windows

package brokerstore

import (
	"context"
	"encoding/json"
	"errors"
	"strings"
)

type nativeRepairSource struct {
	store             *nativeStore
	current, original *nativeReopenedGeneration
	record            generationRecord
	members           map[string]candidateFile
	owned             []candidateFile
	seal              stageSealReaderBackend
	candidate         *nativeCandidate
	closed            bool
	closeErr          error
}

func (n *nativeStore) openRepairSource(ctx context.Context, client, generation string) (repairSourceBackend, generationRecord, []byte, generationRecord, []byte, error) {
	sid, err := clientSID(client, n.broker)
	if err != nil {
		return nil, generationRecord{}, nil, generationRecord{}, nil, err
	}
	return openNativeRepairSource(ctx, n, sid, client, generation, n.openReadOnlyGeneration)
}
func (r *nativeRepairSource) files() map[string]candidateFile { return r.members }
func (r *nativeRepairSource) check() error {
	if r == nil || r.closed || r.current == nil || r.original == nil {
		return ErrClosed
	}
	if err := errors.Join(r.current.check(), r.current.checkHeldDigests(), r.original.check(), r.original.checkHeldDigests()); err != nil {
		return err
	}
	if r.candidate != nil {
		names := []string{ReceiptFilename}
		for name := range r.members {
			names = append(names, name)
		}
		if err := r.candidate.candidateInventory(names, true); err != nil {
			return err
		}
	}
	if r.seal != nil {
		if err := r.seal.validate(context.Background()); err != nil {
			return err
		}
	}
	for name, file := range r.members {
		m := r.record.Members[name]
		if err := file.check(m.Length); err != nil {
			return err
		}
	}
	return nil
}
func (r *nativeRepairSource) create(ctx context.Context, generation string) (generationBackend, error) {
	if err := r.check(); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	sid, err := clientSID(r.current.client, r.store.broker)
	if err != nil {
		return nil, err
	}
	g, err := prepareNativeInstalledGeneration(r.store, nil, sid, r.current.client, generation)
	if err != nil {
		return nil, err
	}
	if err = g.openApplications(); err != nil {
		return nil, errors.Join(err, g.close())
	}
	return g, nil
}
func (r *nativeRepairSource) close() error {
	if r == nil {
		return nil
	}
	if r.closed {
		return r.closeErr
	}
	r.closed = true
	for _, f := range r.owned {
		r.closeErr = errors.Join(r.closeErr, f.Close())
	}
	r.owned = nil
	// Stage seal readonly resources were transferred into the original metadata
	// generation. That same owner drains them; never close the stage pair twice.
	if r.original != nil && r.original != r.current {
		r.closeErr = errors.Join(r.closeErr, r.original.close())
	}
	if r.current != nil {
		r.closeErr = errors.Join(r.closeErr, r.current.close())
	}
	return r.closeErr
}

// Repair generation completion still binds the original registered baseline.
// Source closure is immutable retained metadata, not present permissions alone.
func repairOriginalSourceGeneration(record generationRecord) string {
	if record.Repair == nil {
		return record.Generation
	}
	var source generationRecord
	if json.Unmarshal(record.Repair.SourceCompletion, &source) != nil {
		return ""
	}
	return source.Generation
}

func (r *nativeRepairSource) prepareBaselineExecutor(ctx context.Context, store *Store, m baselineMember, dirs map[string]diskIdentity) error {
	if r.record.Purpose != approvedBaselineGenerationPurpose || r.current != r.original || m.Length <= 0 || !m.Identity.valid() {
		return ErrNativeRequired
	}
	g := r.original.nativeInstalledGeneration
	if g.members[generationExecutorMember] != nil {
		return verifyFile(ctx, store, g.members[generationExecutorMember], m.Length, m.SHA256)
	}
	app, err := g.openParent(g.tree.directories[""], "app", g.tree.securityDir)
	if err != nil {
		return err
	}
	if g.pins[app] != dirs["app"] {
		return ErrInstalledGenerationUncertain
	}
	file, err := baselineFile(app, "skager-start.exe", g.tree.securityFile, false)
	if err != nil {
		return err
	}
	r.owned = append(r.owned, file)
	g.members[generationExecutorMember] = file
	if file.identity != m.Identity {
		return ErrInstalledGenerationUncertain
	}
	return verifyFile(ctx, store, file, m.Length, m.SHA256)
}
func (r *nativeRepairSource) matchBaselineExecutor(ctx context.Context, store *Store, pid uint32, created uint64) error {
	m := r.record.Members[generationExecutorMember]
	file := r.original.members[generationExecutorMember]
	if file == nil {
		return ErrNativeRequired
	}
	if err := verifyFile(ctx, store, file, m.Length, m.SHA256); err != nil {
		return err
	}
	return r.original.matchExecutorProcess(ctx, pid, created)
}

func (r *nativeRepairSource) validateRepairHistory(ctx context.Context, completed map[string]string) error {
	if err := r.check(); err != nil {
		return err
	}
	return r.current.validateRepairHistory(ctx, completed)
}
func (g *nativeReopenedGeneration) validateRepairHistory(ctx context.Context, completed map[string]string) error {
	if ctx == nil {
		return ErrClosed
	}
	if err := errors.Join(ctx.Err(), g.check(), g.checkHeldDigests()); err != nil {
		return err
	}
	records := [][]byte{}
	for _, held := range g.held {
		if strings.HasPrefix(held.file.pin.name, g.registry.name+`\`) && strings.HasSuffix(held.file.pin.name, ".json") {
			raw, err := decommissionReadExact(held.file, held.length, nil)
			if err != nil {
				return err
			}
			records = append(records, raw)
		}
	}
	return validateRepairHistoryRecords(records, completed)
}

func (r *nativeRepairSource) retainMembers(files map[string]candidateFile) {
	for _, file := range files {
		if file != nil {
			r.owned = append(r.owned, file)
		}
	}
}
