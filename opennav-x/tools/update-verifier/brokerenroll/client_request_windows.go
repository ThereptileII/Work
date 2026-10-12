//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"io"
	"path/filepath"
	"runtime"
	"time"

	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

const clientRequestsDirectory = "ClientRegistrationRequests"

func validateClientSIDNative(value string) error {
	if !canonicalClientSID(value) {
		return errors.New("canonical account-domain client SID required")
	}
	sid, err := windows.StringToSid(value)
	if err != nil || !sid.IsValid() || sid.String() != value {
		return errors.New("canonical actual client SID required")
	}
	_, _, kind, err := sid.LookupAccount("")
	if err != nil || kind != windows.SidTypeUser {
		return errors.New("client SID must resolve to an actual user")
	}
	return nil
}

func clientRequestDescriptor(broker string, directory bool) (*windows.SECURITY_DESCRIPTOR, error) {
	flags, access := "", "FR"
	if directory {
		flags, access = "OICI", "FRFX"
	}
	// Approval remains external: administrators can edit this namespace when no
	// custody owner holds it. The broker has read/traverse only, clients no ACE.
	return windows.SecurityDescriptorFromString("O:BAD:P(A;" + flags + ";FA;;;SY)(A;" + flags + ";FA;;;BA)(A;" + flags + ";" + access + ";;;" + broker + ")")
}

func (f *nativeEnrollmentFiles) clientRegistration() (string, string) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.clientAttempted && !f.clientBaselineReady {
		// A partial client handoff must fail SCM's binding validation, rather
		// than looking like the supported no-client enrollment path.
		return "", "incomplete baseline publication"
	}
	return f.clientSID, f.clientSHA
}

func (f *nativeEnrollmentFiles) publishClientRequest(ctx context.Context, raw []byte, sid, approvedSHA string) (resultErr error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil {
		return errors.New("live client request publication context required")
	}
	parsed, err := brokerregistration.ParseApprovedRequest(raw, approvedSHA, time.Now().UTC())
	if err != nil || parsed.ClientSID != sid || validateClientSIDNative(sid) != nil {
		return errors.New("exact approved actual client request required")
	}
	if err := f.check(); err != nil {
		return err
	}
	if f.data == nil || f.clientAttempted {
		return errors.New("only one retained fresh enrollment may publish a request")
	}
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return err
	}
	directorySD, err := clientRequestDescriptor(broker.String(), true)
	if err != nil {
		return err
	}
	fileSD, err := clientRequestDescriptor(broker.String(), false)
	if err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	// From the first namespace mutation any partial result is permanent refusal.
	// Never adopt an existing directory/file, delete it, repair it or retry.
	f.clientAttempted = true
	parent, err := f.newDirectory(f.data, clientRequestsDirectory, directorySD, broker.String())
	if err != nil {
		return err
	}
	f.clientParent = parent
	file, pin, err := f.newFile(parent, sid+".json", fileSD)
	if err != nil {
		return err
	}
	writerOpen := true
	defer func() {
		if writerOpen {
			resultErr = errors.Join(resultErr, enrollmentCleanup(file.Close()))
		}
	}()
	owned := parsed.OriginalBytes()
	if n, err := file.Write(owned); err != nil {
		return err
	} else if n != len(owned) {
		return io.ErrShortWrite
	}
	pin.length = int64(len(owned))
	if err := file.Sync(); err != nil {
		return err
	}
	readback, err := readClientRequestPin(ctx, pin)
	if err != nil || !bytes.Equal(readback, owned) {
		return errors.Join(errors.New("new client request readback differs"), err)
	}
	// Reopen read-only with no write/delete sharing, matching the full original
	// native identity. Final at-creation ACLs protect the brief reopen interval.
	closeErr := enrollmentCleanup(file.Close())
	writerOpen = false
	if closeErr != nil {
		return closeErr
	}
	if err := reopenClientRequest(pin); err != nil {
		return err
	}
	pin.dos = filepath.Join(parent.dos, sid+".json")
	f.pins = append(f.pins, pin)
	readback, err = readClientRequestPin(ctx, pin)
	if err != nil || !bytes.Equal(readback, owned) {
		return errors.Join(errors.New("held approved client request differs"), err)
	}
	if err := f.check(); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	f.clientSID, f.clientSHA = sid, approvedSHA
	return nil
}

func reopenClientRequest(pin *enrollmentPin) error {
	pointer, err := windows.UTF16PtrFromString(enrollmentPhysical(pin.physical))
	if err != nil {
		return err
	}
	h, err := windows.CreateFile(pointer, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, nil, windows.OPEN_EXISTING, windows.FILE_FLAG_OPEN_REPARSE_POINT, 0)
	if err != nil {
		return err
	}
	pin.handle = h
	// Caller owns h even when subsequent identity validation fails.
	return nil
}

func readClientRequestPin(ctx context.Context, pin *enrollmentPin) ([]byte, error) {
	if ctx == nil || ctx.Err() != nil || !pin.file || pin.length < 1 || pin.length > brokerregistration.MaxRequestBytes {
		return nil, errors.New("bounded held client request required")
	}
	if err := pin.check(); err != nil {
		return nil, err
	}
	if _, err := windows.SetFilePointer(pin.handle, 0, nil, windows.FILE_BEGIN); err != nil {
		return nil, err
	}
	data := make([]byte, int(pin.length)+1)
	n := 0
	for n < len(data) {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		var done uint32
		err := windows.ReadFile(pin.handle, data[n:], &done, nil)
		if err != nil && !errors.Is(err, windows.ERROR_HANDLE_EOF) {
			return nil, err
		}
		n += int(done)
		if done == 0 || errors.Is(err, windows.ERROR_HANDLE_EOF) {
			break
		}
	}
	if int64(n) != pin.length {
		return nil, errors.New("held client request length changed")
	}
	if err := pin.check(); err != nil {
		return nil, err
	}
	return data[:n], ctx.Err()
}

func clientRequestBrokerPrincipal(broker *windows.SID) error {
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil || user.User.Sid == nil || !user.User.Sid.Equals(broker) {
		return errors.New("actual broker process principal required")
	}
	var token windows.Token
	err = windows.OpenThreadToken(windows.CurrentThread(), windows.TOKEN_QUERY, true, &token)
	if err == nil {
		return errors.Join(errors.New("impersonated client request read refused"), enrollmentCleanup(token.Close()))
	}
	if !errors.Is(err, windows.ERROR_NO_TOKEN) {
		return errors.New("non-impersonated broker request read required")
	}
	effective, err := windows.GetCurrentThreadEffectiveToken().GetTokenUser()
	if err != nil || effective.User.Sid == nil || !effective.User.Sid.Equals(broker) {
		return errors.New("actual effective broker principal required")
	}
	return nil
}

// ReadApprovedClientRequestNative reads only the fixed protected machine request
// as the actual non-impersonated broker. approvedSHA must arrive from independent
// operator approval through stopped SCM dispatch; neither an admin-editable file
// nor a self-hash is authority. Success returns owned original bytes, not proof
// of installed application custody, launch authorization or release acceptance.
// No path argument, admin-console fallback, repair or partial-file adoption exists.
func ReadApprovedClientRequestNative(ctx context.Context, sid, approvedSHA string) (result []byte, resultErr error) {
	if ctx == nil || ctx.Err() != nil || !canonicalClientSID(sid) || !validPin(approvedSHA) {
		return nil, errors.New("live exact independently approved request identity required")
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return nil, err
	}
	if err := clientRequestBrokerPrincipal(broker); err != nil {
		return nil, err
	}
	if err := validateClientSIDNative(sid); err != nil {
		return nil, err
	}
	f := &nativeEnrollmentFiles{}
	defer func() {
		if err := f.Close(); err != nil {
			result, resultErr = nil, errors.Join(resultErr, err)
		}
	}()
	programData, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, windows.KF_FLAG_DEFAULT)
	if err != nil {
		return nil, err
	}
	parent, err := f.holdAncestry(programData, true)
	if err != nil {
		return nil, err
	}
	dataSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FA;;;" + broker.String() + ")")
	if err != nil {
		return nil, err
	}
	directorySD, err := clientRequestDescriptor(broker.String(), true)
	if err != nil {
		return nil, err
	}
	fileSD, err := clientRequestDescriptor(broker.String(), false)
	if err != nil {
		return nil, err
	}
	for _, child := range []struct {
		name string
		sd   *windows.SECURITY_DESCRIPTOR
	}{{"SKAGER", dataSD}, {clientRequestsDirectory, directorySD}} {
		name := parent.physical + `\` + child.name
		pin, err := f.pinDirectory(enrollmentPhysical(name), name, filepath.Join(parent.dos, child.name), child.sd, false, broker.String())
		if err != nil {
			return nil, err
		}
		if pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
			return nil, errors.New("client request ancestry crossed volume")
		}
		parent = pin
	}
	pin := &enrollmentPin{physical: parent.physical + `\` + sid + ".json", dos: filepath.Join(parent.dos, sid+".json"), descriptor: fileSD, file: true}
	if err := reopenClientRequest(pin); err != nil {
		return nil, err
	}
	f.pins = append(f.pins, pin)
	pin.info, pin.id, err = enrollmentInfo(pin.handle)
	if err != nil {
		return nil, err
	}
	pin.length = int64(uint64(pin.info.FileSizeHigh)<<32 | uint64(pin.info.FileSizeLow))
	if pin.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.New("client request file crossed volume")
	}
	check := func() error {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := clientRequestBrokerPrincipal(broker); err != nil {
			return err
		}
		for _, held := range f.pins {
			if err := held.check(); err != nil {
				return err
			}
		}
		return nil
	}
	if err := check(); err != nil {
		return nil, err
	}
	raw, err := readClientRequestPin(ctx, pin)
	if err != nil {
		return nil, err
	}
	parsed, err := brokerregistration.ParseApprovedRequest(raw, approvedSHA, time.Now().UTC())
	if err != nil || parsed.ClientSID != sid {
		return nil, errors.New("protected request differs from independent approved identity")
	}
	if err := check(); err != nil {
		return nil, err
	}
	return parsed.OriginalBytes(), nil
}
