//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"sync"

	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

type nativeInstalledCompletion struct {
	mu                sync.Mutex
	self              *nativeInstalledCompletion
	files             *nativeEnrollmentFiles
	publication       *nativeStartupPublication
	baseline          ApprovedClientBaseline
	record            installedCompletionRecord
	raw               []byte
	marker, completed *enrollmentPin
	writer            *os.File
	attempted, closed bool
	closeErr          error
}

func installedCompletionOriginalPin(f *nativeEnrollmentFiles, physical string) (*enrollmentPin, error) {
	var result *enrollmentPin
	for _, pin := range f.pins {
		if pin.file && pin.physical == physical {
			if result != nil {
				return nil, errors.New("one original installed completion member required")
			}
			result = pin
		}
	}
	if result == nil {
		return nil, errors.New("original installed completion member missing")
	}
	return result, result.check()
}

func installedCompletionHash(ctx context.Context, pin *enrollmentPin) (string, error) {
	if pin == nil || !pin.file {
		return "", errors.New("original installed file required")
	}
	return copyBaselineBytes(ctx, io.Discard, clientBaselinePinReader{pin}, pin.length)
}

func (s *nativeStartupPublication) prepareInstalledCompletion(ctx context.Context, original enrollmentFiles, client *brokerregistration.Request, source ApprovedClientBaseline) (result installedCompletion, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	f, ok := original.(*nativeEnrollmentFiles)
	nativeSource, nativeBaseline := source.(*nativeClientBaseline)
	if ctx == nil || ctx.Err() != nil || !ok || f == nil || s == nil || s.files == nil || s.files == f || client == nil || !nativeBaseline || nativeSource == nil || nativeSource.files == f || nativeSource.broker != nil || s.binding == nil || s.clientSID != client.ClientSID {
		return nil, errors.New("same original native installed publication owners required")
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if !f.clientAttempted || !f.clientBaselineReady || f.clientSID != client.ClientSID || f.clientSHA != client.RequestSHA256 || f.clientParent == nil || f.maintenanceHelper == nil || !validPin(f.maintenanceHelperSHA) {
		return nil, errors.New("original approved request baseline and installed helper required")
	}
	if err := f.check(); err != nil {
		return nil, err
	}
	if err := s.Check(ctx); err != nil {
		return nil, err
	}
	if err := source.Validate(); err != nil {
		return nil, err
	}
	request, err := installedCompletionOriginalPin(f, f.clientParent.physical+`\`+client.ClientSID+".json")
	if err != nil {
		return nil, err
	}
	rawRequest, err := readClientRequestPin(ctx, request)
	if err != nil || !bytes.Equal(rawRequest, client.OriginalBytes()) || digest(rawRequest) != client.RequestSHA256 {
		return nil, errors.Join(errors.New("original approved client request differs"), err)
	}
	baseline, err := installedCompletionOriginalPin(f, f.clientParent.physical+`\`+client.ClientSID+".baseline.zip")
	if err != nil {
		return nil, err
	}
	baselineSHA, err := installedCompletionHash(ctx, baseline)
	if err != nil {
		return nil, err
	}
	sourceSHA, err := copyClientBaseline(ctx, io.Discard, source)
	if err != nil || source.Size() != baseline.length || sourceSHA != baselineSHA {
		return nil, errors.Join(errors.New("original approved baseline publication differs"), err)
	}
	var brokerImage *enrollmentPin
	for _, pin := range f.pins {
		if pin.file && pin.dos == f.imagePath {
			if brokerImage != nil {
				return nil, errors.New("one original broker image required")
			}
			brokerImage = pin
		}
	}
	if brokerImage == nil || f.maintenanceHelper.dos != filepath.Join(filepath.Dir(f.imagePath), maintenanceImageName) || f.maintenanceHelper.physical != filepath.Dir(brokerImage.physical)+`\`+maintenanceImageName {
		return nil, errors.New("fixed original installed helper sibling required")
	}
	brokerSHA, err := installedCompletionHash(ctx, brokerImage)
	if err != nil {
		return nil, err
	}
	rawPublication, err := readClientRequestPin(ctx, s.binding)
	if err != nil {
		return nil, err
	}
	n := &nativeInstalledCompletion{files: f, publication: s, baseline: source}
	n.self = n
	cache, err := originalBootstrapCacheBindingNative(f)
	if err != nil {
		return nil, err
	}
	r := installedCompletionRecord{Schema: 2, BootstrapCache: cache, Purpose: installedCompletionPurpose, ClientSID: client.ClientSID, ClientRequest: rawRequest, ClientRequestSHA256: client.RequestSHA256, StartupPublication: rawPublication, StartupPublicationSHA256: s.bindingSHA}
	if r.Parent, err = startupDirectoryIdentity(f.clientParent); err != nil {
		return nil, err
	}
	for _, item := range []struct {
		pin    *enrollmentPin
		sha    string
		target *startupFileBinding
	}{
		{request, client.RequestSHA256, &r.Request}, {s.binding, s.bindingSHA, &r.Publication}, {baseline, baselineSHA, &r.Baseline}, {brokerImage, brokerSHA, &r.Broker}, {f.maintenanceHelper, f.maintenanceHelperSHA, &r.MaintenanceHelper},
	} {
		if *item.target, err = startupFileIdentity(ctx, item.pin, item.sha); err != nil {
			return nil, err
		}
	}
	n.record = r
	n.raw, err = marshalInstalledCompletion(r)
	if err != nil {
		return nil, err
	}
	if err = ctx.Err(); err != nil {
		return nil, err
	}
	// An old JSON without its reservation is also historical state, never adopted.
	if err = enrollmentAbsent(f.clientParent.physical + `\` + client.ClientSID + ".maintenance.json"); err != nil {
		return nil, err
	}
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return nil, err
	}
	sd, err := clientRequestDescriptor(broker.String(), false)
	if err != nil {
		return nil, err
	}
	n.writer, n.marker, err = f.newFile(f.clientParent, client.ClientSID+".maintenance.once", sd)
	if err != nil {
		return nil, err
	}
	result = n // Any reservation mutation retains its owner and permanent bytes.
	defer func() {
		if resultErr != nil {
			n.attempted = true
		}
	}()
	n.marker.dos = filepath.Join(f.clientParent.dos, client.ClientSID+".maintenance.once")
	if err = n.writeMarker(ctx, false); err != nil {
		n.attempted = true
		return result, errors.Join(ErrStartupUncertain, err)
	}
	return result, n.check(ctx)
}

func (n *nativeInstalledCompletion) check(ctx context.Context) error {
	if n == nil || n.self != n || n.closed || ctx == nil || ctx.Err() != nil || n.files == nil || n.publication == nil || n.baseline == nil {
		return errors.New("live original installed completion owner required")
	}
	if err := n.files.check(); err != nil {
		return err
	}
	if err := n.publication.Check(ctx); err != nil {
		return err
	}
	if err := n.baseline.Validate(); err != nil {
		return err
	}
	if n.marker != nil {
		if err := n.marker.check(); err != nil {
			return err
		}
	}
	if n.completed != nil {
		return startupHashPin(ctx, n.completed, digest(n.raw))
	}
	return ctx.Err()
}

func (n *nativeInstalledCompletion) writeMarker(ctx context.Context, completed bool) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	raw := installedCompletionMarker(digest(n.raw), completed)
	if _, err := n.writer.Seek(0, io.SeekStart); err != nil {
		return err
	}
	count, err := n.writer.Write(raw)
	if err != nil {
		return err
	}
	if count != len(raw) {
		return io.ErrShortWrite
	}
	n.marker.length = int64(len(raw))
	if err = n.writer.Sync(); err != nil {
		return err
	}
	got := make([]byte, len(raw)+1)
	count, err = n.writer.ReadAt(got, 0)
	if err != nil && !errors.Is(err, io.EOF) || count != len(raw) || !bytes.Equal(got[:count], raw) {
		return errors.Join(errors.New("original maintenance reservation readback differs"), err)
	}
	return n.marker.check()
}

func (n *nativeInstalledCompletion) Complete(ctx context.Context, guard func(context.Context) error) (resultErr error) {
	if n == nil {
		return errors.New("original installed completion owner required")
	}
	n.mu.Lock()
	defer n.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n.self != n || n.closed || n.attempted || guard == nil || ctx == nil {
		return errors.New("one original installed completion attempt required")
	}
	n.attempted = true
	if err := guard(ctx); err != nil {
		return err
	}
	n.files.mu.Lock()
	defer n.files.mu.Unlock()
	if err := n.check(ctx); err != nil {
		return err
	}
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return err
	}
	sd, err := clientRequestDescriptor(broker.String(), false)
	if err != nil {
		return err
	}
	file, pin, err := n.files.newFile(n.files.clientParent, n.record.ClientSID+".maintenance.json", sd)
	if err != nil {
		return err
	}
	n.completed = pin
	defer func() {
		if file != nil {
			resultErr = errors.Join(resultErr, enrollmentCleanup(file.Close()))
			n.completed = nil
		}
	}()
	count, err := file.Write(n.raw)
	if err != nil {
		return err
	}
	if count != len(n.raw) {
		return io.ErrShortWrite
	}
	pin.length = int64(len(n.raw))
	if err = file.Sync(); err != nil {
		return err
	}
	got := make([]byte, len(n.raw)+1)
	count, err = file.ReadAt(got, 0)
	if err != nil && !errors.Is(err, io.EOF) || count != len(n.raw) || !bytes.Equal(got[:count], n.raw) {
		return errors.Join(errors.New("original maintenance completion readback differs"), err)
	}
	if err = pin.check(); err != nil {
		return err
	}
	if err = enrollmentCleanup(file.Close()); err != nil {
		file = nil
		n.completed = nil
		return err
	}
	file = nil
	if err = reopenClientRequest(pin); err != nil {
		n.completed = nil
		return err
	}
	pin.dos = filepath.Join(n.files.clientParent.dos, n.record.ClientSID+".maintenance.json")
	if err = n.check(ctx); err != nil {
		return err
	}
	// The same-owner SCM guard also checks files.Check. Release its mutex only
	// for that private guard, then revalidate original custody before committing.
	err = func() error {
		n.files.mu.Unlock()
		defer n.files.mu.Lock()
		return guard(ctx)
	}()
	if err != nil {
		return err
	}
	if err = n.check(ctx); err != nil {
		return err
	}
	// Irrevocable bounded completion: after entry, late cancellation does not
	// relabel successfully synced/read-back completed evidence as uncommitted.
	if err = n.writeMarker(ctx, true); err != nil {
		return errors.Join(ErrStartupUncertain, err)
	}
	if err = enrollmentCleanup(n.writer.Close()); err != nil {
		n.writer = nil
		n.marker = nil
		return err
	}
	n.writer = nil
	if err = reopenClientRequest(n.marker); err != nil {
		n.marker = nil
		return err
	}
	return n.marker.check()
}

func (n *nativeInstalledCompletion) Close() error {
	if n == nil {
		return nil
	}
	if n.self != n {
		return errors.New("original installed completion owner required")
	}
	n.mu.Lock()
	defer n.mu.Unlock()
	if n.closed {
		return n.closeErr
	}
	n.closed = true
	if n.writer != nil {
		n.closeErr = errors.Join(n.closeErr, enrollmentCleanup(n.writer.Close()))
		n.writer = nil
	} else if n.marker != nil {
		n.closeErr = errors.Join(n.closeErr, enrollmentCleanup(windows.CloseHandle(n.marker.handle)))
	}
	if n.completed != nil {
		n.closeErr = errors.Join(n.closeErr, enrollmentCleanup(windows.CloseHandle(n.completed.handle)))
	}
	return n.closeErr
}
