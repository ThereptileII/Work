package brokerstore

import (
	"context"
	"errors"
	"io"
	"os"

	"golang.org/x/sys/windows"
)

type nativeExecutionJournalReader struct {
	journal    *nativeExecutionJournal
	client, id string
}

func (r *nativeExecutionJournalReader) validate(fence, raw []byte) error {
	names, err := nativeExecutionJournalNames(context.Background(), r.journal.store)
	if err != nil {
		return err
	}
	if err := executionJournalPairNames(r.client, r.id, names); err != nil {
		return err
	}
	return r.journal.validate(fence, raw)
}
func (r *nativeExecutionJournalReader) close() error { return r.journal.close() }

func (n *nativeStore) openExecutionJournal(ctx context.Context, client, id string) (executionJournalReadBackend, []byte, []byte, error) {
	if err := n.check(); err != nil {
		return nil, nil, nil, err
	}
	if _, err := clientSID(client, n.broker); err != nil {
		return nil, nil, nil, err
	}
	return openNativeExecutionJournal(ctx, n, client, id)
}

// This lower fixed-namespace opener permits inert temporary fixtures. Production
// enters only through actual broker/client checks above; no caller path exists.
func openNativeExecutionJournal(ctx context.Context, n *nativeStore, client, id string) (result executionJournalReadBackend, fence, raw []byte, resultErr error) {
	n.beginMu.Lock()
	defer n.beginMu.Unlock()
	if ctx == nil || !validHex(id, 32) {
		return nil, nil, nil, errors.New("bounded original execution identity required")
	}
	if err := ctx.Err(); err != nil {
		return nil, nil, nil, err
	}
	if err := n.check(); err != nil {
		return nil, nil, nil, err
	}
	names, err := nativeExecutionJournalNames(ctx, n)
	if err != nil {
		return nil, nil, nil, err
	}
	if err = executionJournalPairNames(client, id, names); err != nil {
		return nil, nil, nil, err
	}
	sd, err := descriptor(n.broker, nil, false)
	if err != nil {
		return nil, nil, nil, err
	}
	j := &nativeExecutionJournal{store: n, ids: executionJournalIDs{Parents: map[string]diskIdentity{}}, parents: map[string]*nativePin{}}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(ErrExecutionJournalUncertain, resultErr, j.close())
		}
	}()
	for _, pin := range n.pins {
		identity, err := fileIdentity(pin.handle)
		if err != nil {
			return nil, nil, nil, err
		}
		j.ids.Parents[pin.name] = identity
		j.parents[pin.name] = pin
	}
	component := executionJournalComponent(client, id)
	open := func(suffix string, bound int64) (*nativeFile, []byte, error) {
		if err := ctx.Err(); err != nil {
			return nil, nil, err
		}
		file, err := openAcquisitionFile(n.registrations, component+suffix, sd, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, windows.OPEN_EXISTING)
		if err != nil {
			return nil, nil, err
		}
		length := int64(uint64(file.pin.info.FileSizeHigh)<<32 | uint64(file.pin.info.FileSizeLow))
		fail := func(err error) (*nativeFile, []byte, error) { return nil, nil, errors.Join(err, file.Close()) }
		if length <= 0 || length > bound {
			return fail(errors.New("original execution file missing/partial/outside bound"))
		}
		if err = file.check(length); err != nil {
			return fail(err)
		}
		kind, err := windows.GetFileType(file.pin.handle)
		if err != nil || kind != windows.FILE_TYPE_DISK {
			return fail(errors.Join(errors.New("ordinary original execution disk file required"), err))
		}
		original, err := io.ReadAll(io.LimitReader(file, length+1))
		if err != nil || int64(len(original)) != length {
			return fail(errors.Join(errors.New("original execution file changed during bounded read"), err))
		}
		if err = file.check(length); err != nil {
			return fail(err)
		}
		if err = ctx.Err(); err != nil {
			return fail(err)
		}
		return file, original, nil
	}
	j.fence, fence, err = open(".once", maxExecutionFenceBytes)
	if err != nil {
		return nil, nil, nil, err
	}
	j.ids.Fence, err = fileIdentity(j.fence.pin.handle)
	if err != nil {
		return nil, nil, nil, err
	}
	j.journal, raw, err = open(".journal", maxExecutionJournalBytes)
	if err != nil {
		return nil, nil, nil, err
	}
	j.ids.Journal, err = fileIdentity(j.journal.pin.handle)
	if err != nil {
		return nil, nil, nil, err
	}
	if _, _, err = decodeReadExecutionJournal(client, id, fence, raw); err != nil {
		return nil, nil, nil, err
	}
	if err = j.validate(fence, raw); err != nil {
		return nil, nil, nil, err
	}
	if err = ctx.Err(); err != nil {
		return nil, nil, nil, err
	}
	reader := &nativeExecutionJournalReader{journal: j, client: client, id: id}
	if err = reader.validate(fence, raw); err != nil {
		return nil, nil, nil, err
	}
	return reader, fence, raw, nil
}
func (n *nativeStore) executionJournalNames(ctx context.Context, client string) ([]string, error) {
	if err := n.check(); err != nil {
		return nil, err
	}
	if _, err := clientSID(client, n.broker); err != nil {
		return nil, err
	}
	return nativeExecutionJournalNames(ctx, n)
}
func nativeExecutionJournalNames(ctx context.Context, n *nativeStore) (names []string, resultErr error) {
	if ctx == nil {
		return nil, errors.New("bounded native enumeration context required")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := n.check(); err != nil {
		return nil, err
	}
	file, err := os.Open(physical(n.registrations.name))
	if err != nil {
		return nil, err
	}
	defer func() { resultErr = errors.Join(resultErr, file.Close()) }()
	want, err := fileIdentity(n.registrations.handle)
	if err != nil {
		return nil, err
	}
	got, err := fileIdentity(windows.Handle(file.Fd()))
	if err != nil || want != got {
		return nil, errors.Join(errors.New("original execution enumeration directory changed"), err)
	}
	entries, err := file.ReadDir(maxExecutionNamespaceFiles + 1)
	if err != nil && !errors.Is(err, io.EOF) {
		return nil, err
	}
	if len(entries) > maxExecutionNamespaceFiles {
		return nil, errors.New("fixed execution namespace exceeds enumeration bound")
	}
	for _, entry := range entries {
		names = append(names, entry.Name())
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	got, err = fileIdentity(windows.Handle(file.Fd()))
	if err != nil || want != got {
		return nil, errors.Join(errors.New("original execution enumeration directory changed"), err)
	}
	return names, n.check()
}
