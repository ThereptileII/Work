package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"os"
	"strings"

	"golang.org/x/sys/windows"
)

type nativeExecutionJournal struct {
	store          *nativeStore
	fence, journal *nativeFile
	ids            executionJournalIDs
	parents        map[string]*nativePin
	closed         bool
	closeErr       error
}

func executionJournalComponent(client, id string) string { return client + ".execution." + id }

func (n *nativeStore) prepareExecutionJournal(ctx context.Context, client, id string) (result executionJournalBackend, resultErr error) {
	if err := n.check(); err != nil {
		return nil, err
	}
	if _, err := clientSID(client, n.broker); err != nil {
		return nil, err
	}
	return createNativeExecutionJournal(ctx, n, client, id)
}

// The lower constructor still uses only the retained fixed native namespace.
// Inert native fixtures can exercise it with their current-user descriptor;
// production enters only through the actual broker/client checks above.
func createNativeExecutionJournal(ctx context.Context, n *nativeStore, client, id string) (result executionJournalBackend, resultErr error) {
	n.beginMu.Lock()
	defer n.beginMu.Unlock()
	if ctx == nil || !validHex(id, 32) {
		return nil, errors.New("fresh bounded execution identity required")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := n.check(); err != nil {
		return nil, err
	}
	if err := n.executionJournalCapacity(); err != nil {
		return nil, err
	}
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		return nil, err
	}
	j := &nativeExecutionJournal{store: n, ids: executionJournalIDs{Parents: map[string]diskIdentity{}}, parents: map[string]*nativePin{}}
	// All protected-store ancestors remain held by Store. Record their full
	// 128-bit file IDs now and compare them at every append/readback/close.
	for _, p := range n.pins {
		identity, err := fileIdentity(p.handle)
		if err != nil {
			return nil, err
		}
		j.ids.Parents[p.name] = identity
		j.parents[p.name] = p
	}
	component := executionJournalComponent(client, id)
	mutating := false
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, j.close())
			if mutating {
				resultErr = errors.Join(ErrExecutionJournalUncertain, resultErr)
			}
		}
	}()
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	// The permanent fence is always created first, even if later creation,
	// flush, readback or cancellation fails. No cleanup path removes either.
	mutating = true
	j.fence, err = createRegistrationFile(n.registrations, component+".once", sd)
	if err != nil {
		return nil, err
	}
	// Flush the empty reservation before creating its journal sibling. A crash
	// here must leave a permanent refusal, not a reusable execution ID.
	if err = j.fence.Sync(); err != nil {
		return nil, err
	}
	if err = executionReadback(j.fence, nil); err != nil {
		return nil, err
	}
	j.ids.Fence, err = fileIdentity(j.fence.pin.handle)
	if err != nil {
		return nil, err
	}
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	j.journal, err = createRegistrationFile(n.registrations, component+".journal", sd)
	if err != nil {
		return nil, err
	}
	j.ids.Journal, err = fileIdentity(j.journal.pin.handle)
	if err != nil || !j.ids.valid() {
		return nil, errors.Join(errors.New("original native execution journal identities unavailable"), err)
	}
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	return j, nil
}

// Bound the fixed native namespace before reservation, retaining all abandoned
// pairs and counting them towards capacity. No eviction or foreign-file adoption.
func (n *nativeStore) executionJournalCapacity() (resultErr error) {
	if err := n.registrations.check(); err != nil {
		return err
	}
	f, err := os.Open(physical(n.registrations.name))
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, f.Close()) }()
	want, err := fileIdentity(n.registrations.handle)
	if err != nil {
		return err
	}
	got, err := fileIdentity(windows.Handle(f.Fd()))
	if err != nil || want != got {
		return errors.Join(errors.New("execution namespace enumeration identity changed"), err)
	}
	entries, err := f.ReadDir(maxExecutionNamespaceFiles + 1)
	if err != nil && !errors.Is(err, io.EOF) {
		return err
	}
	if len(entries) > maxExecutionNamespaceFiles {
		return errors.New("protected execution namespace bound exhausted")
	}
	count := 0
	for _, e := range entries {
		if strings.Contains(strings.ToLower(e.Name()), ".execution.") {
			count++
		}
	}
	if count+2 > 2*maxExecutionJournals {
		return errors.New("permanent execution journal capacity exhausted; explicit recovery required")
	}
	return n.registrations.check()
}

func (j *nativeExecutionJournal) identities() (executionJournalIDs, error) {
	if err := j.checkIdentities(); err != nil {
		return executionJournalIDs{}, err
	}
	copy := j.ids
	copy.Parents = map[string]diskIdentity{}
	for name, id := range j.ids.Parents {
		copy.Parents[name] = id
	}
	return copy, nil
}

func (j *nativeExecutionJournal) checkIdentities() error {
	if j.closed || j.fence == nil || j.journal == nil {
		return ErrClosed
	}
	if err := j.store.check(); err != nil {
		return err
	}
	for name, p := range j.parents {
		if err := p.check(); err != nil {
			return err
		}
		got, err := fileIdentity(p.handle)
		if err != nil || got != j.ids.Parents[name] {
			return errors.Join(errors.New("execution journal original ancestor changed"), err)
		}
	}
	for f, want := range map[*nativeFile]diskIdentity{j.fence: j.ids.Fence, j.journal: j.ids.Journal} {
		got, err := fileIdentity(f.pin.handle)
		if err != nil || got != want {
			return errors.Join(errors.New("execution journal original file identity changed"), err)
		}
	}
	return nil
}

func executionReadback(file *nativeFile, expected []byte) error {
	if err := file.check(int64(len(expected))); err != nil {
		return err
	}
	if _, err := file.Seek(0, io.SeekStart); err != nil {
		return err
	}
	got, err := io.ReadAll(io.LimitReader(file, int64(len(expected))+1))
	if err != nil || !bytes.Equal(got, expected) {
		return errors.Join(errors.New("execution journal durable original bytes differ"), err)
	}
	return file.check(int64(len(expected)))
}

func executionWrite(ctx context.Context, file *nativeFile, previous, next []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if !bytes.HasPrefix(next, previous) {
		return errors.New("execution journal append would replace original bytes")
	}
	if err := executionReadback(file, previous); err != nil {
		return err
	}
	if _, err := file.Seek(int64(len(previous)), io.SeekStart); err != nil {
		return err
	}
	n, err := file.Write(next[len(previous):])
	if err == nil && n != len(next)-len(previous) {
		err = io.ErrShortWrite
	}
	if err != nil {
		return err
	}
	// Flush even if cancellation arrived during Write: durable bytes may exist
	// and cancellation must never imply that mutation was rolled back.
	if err = file.Sync(); err != nil {
		return err
	}
	if err = executionReadback(file, next); err != nil {
		return err
	}
	return ctx.Err()
}

func (j *nativeExecutionJournal) initialize(ctx context.Context, fence, raw []byte) error {
	if err := validateExecutionJournalBytes(fence, raw); err != nil {
		return err
	}
	var origin executionJournalOrigin
	if err := json.Unmarshal(fence, &origin); err != nil || !sameExecutionJournalIDs(origin.Identities, j.ids) {
		return errors.Join(errors.New("origin differs from original held journal identities"), err)
	}
	if err := j.checkIdentities(); err != nil {
		return err
	}
	if err := executionWrite(ctx, j.fence, nil, fence); err != nil {
		return err
	}
	if err := executionWrite(ctx, j.journal, nil, raw); err != nil {
		return err
	}
	return j.validate(fence, raw)
}

func (j *nativeExecutionJournal) append(ctx context.Context, fence, previous, next []byte) error {
	if err := j.validate(fence, previous); err != nil {
		return err
	}
	if err := validateExecutionJournalBytes(fence, next); err != nil {
		return err
	}
	if err := executionWrite(ctx, j.journal, previous, next); err != nil {
		return err
	}
	return j.validate(fence, next)
}

func (j *nativeExecutionJournal) validate(fence, raw []byte) error {
	if err := validateExecutionJournalBytes(fence, raw); err != nil {
		return err
	}
	var origin executionJournalOrigin
	if err := json.Unmarshal(fence, &origin); err != nil || !sameExecutionJournalIDs(origin.Identities, j.ids) {
		return errors.Join(errors.New("original fence does not bind held native journal identities"), err)
	}
	if err := j.checkIdentities(); err != nil {
		return err
	}
	if err := executionReadback(j.fence, fence); err != nil {
		return err
	}
	return executionReadback(j.journal, raw)
}

func (j *nativeExecutionJournal) close() error {
	if j.closed {
		return j.closeErr
	}
	j.closed = true
	if j.journal != nil {
		j.closeErr = errors.Join(j.closeErr, j.journal.Close())
	}
	if j.fence != nil {
		j.closeErr = errors.Join(j.closeErr, j.fence.Close())
	}
	return j.closeErr
}
