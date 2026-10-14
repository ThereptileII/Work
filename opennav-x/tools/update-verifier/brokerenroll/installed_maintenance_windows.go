//go:build windows

package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"io"
	"path/filepath"
	"runtime"
	"strings"
	"sync"

	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

type nativeInstalledMaintenanceAuthority struct {
	mu                                                  sync.Mutex
	files                                               *nativeEnrollmentFiles
	clientSID                                           string
	baseline                                            *enrollmentPin
	closed                                              bool
	closeErr                                            error
	record                                              installedCompletionRecord
	originalRaw                                         []byte
	codeParent, requestParent, brokerImage, helperImage *enrollmentPin
}

func (c *nativeInstalledMaintenanceAuthority) check(ctx context.Context) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.checkLocked(ctx)
}
func (c *nativeInstalledMaintenanceAuthority) checkLocked(ctx context.Context) error {
	if ctx == nil || ctx.Err() != nil || c.closed || c.files == nil {
		return errors.New("retained original installed completion required")
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	return errors.Join(maintenanceOperatorNative(c.clientSID), c.files.Check())
}
func (c *nativeInstalledMaintenanceAuthority) close() error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if !c.closed {
		c.closed = true
		if c.files != nil {
			c.closeErr = c.files.Close()
		}
	}
	return c.closeErr
}

// The completion owns these handles. The installed inventory consumes a
// borrowed read-only view; it cannot close/reopen or choose another archive.
type completedBaselineView struct {
	owner *nativeInstalledMaintenanceAuthority
	ctx   context.Context
}

func (b completedBaselineView) Size() int64     { return b.owner.baseline.length }
func (b completedBaselineView) Validate() error { return b.owner.check(b.ctx) }
func (b completedBaselineView) Close() error    { return nil }
func (b completedBaselineView) ReadAt(p []byte, off int64) (int, error) {
	b.owner.mu.Lock()
	defer b.owner.mu.Unlock()
	if err := b.owner.checkLocked(b.ctx); err != nil {
		return 0, err
	}
	n, err := readClientBaselinePinAt(b.owner.baseline, p, off)
	return n, errors.Join(err, b.owner.checkLocked(b.ctx))
}

func installedReadPin(ctx context.Context, p *enrollmentPin, bound int64) ([]byte, error) {
	if ctx == nil || ctx.Err() != nil || p == nil || !p.file || p.length < 1 || p.length > bound {
		return nil, errors.New("bounded original installed record required")
	}
	if err := p.check(); err != nil {
		return nil, err
	}
	raw := make([]byte, int(p.length))
	n, err := readClientBaselinePinAt(p, raw, 0)
	if err != nil || n != len(raw) {
		return nil, errors.Join(errors.New("original installed record length changed"), err)
	}
	return raw, errors.Join(ctx.Err(), p.check())
}

func installedMatchFile(ctx context.Context, pin *enrollmentPin, want startupFileBinding) error {
	got, err := startupFileIdentity(ctx, pin, want.SHA256)
	if err != nil || got != want {
		return errors.Join(errors.New("original installed file identity changed"), err)
	}
	return nil
}

// Only this fixed native entry can turn the ORIGINAL protected completion into
// a retained maintenance owner. The record's parser alone has no such power.
// Admin/SYSTEM are trusted, as in createFreshFiles; ordinary users cannot create
// or replace this namespace. Missing/partial legacy state refuses without repair.
func readInstalledCompletionNative(ctx context.Context) (installedMaintenanceInputs, error) {
	return readInstalledCompletionModeNative(ctx, true)
}

func readInstalledCompletionModeNative(ctx context.Context, fixedHelper bool, recoverMode ...bool) (result installedMaintenanceInputs, resultErr error) {
	if ctx == nil || ctx.Err() != nil {
		return result, errors.New("live installed maintenance required")
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := enrollmentOperator(); err != nil {
		return result, err
	}
	user, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil || user == nil || user.User.Sid == nil {
		return result, errors.New("actual installed user required")
	}
	sid := user.User.Sid.String()
	if err := maintenanceOperatorNative(sid); err != nil {
		return result, err
	}
	c := &nativeInstalledMaintenanceAuthority{files: &nativeEnrollmentFiles{}, clientSID: sid}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, c.close())
		}
	}()
	broker, err := enrollmentBrokerSID()
	if err != nil {
		return result, err
	}
	programData, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, 0)
	if err != nil {
		return result, err
	}
	parent, err := c.files.holdAncestry(programData, true)
	if err != nil {
		return result, err
	}
	dataSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FA;;;" + broker.String() + ")")
	if err != nil {
		return result, err
	}
	parent, err = maintenanceHoldChild(c.files, parent, "SKAGER", dataSD, broker.String())
	if err != nil {
		return result, err
	}
	directorySD, err := clientRequestDescriptor(broker.String(), true)
	if err != nil {
		return result, err
	}
	parent, err = maintenanceHoldChild(c.files, parent, clientRequestsDirectory, directorySD, broker.String())
	if err != nil {
		return result, err
	}
	fileSD, err := clientRequestDescriptor(broker.String(), false)
	if err != nil {
		return result, err
	}
	completion, err := maintenanceHoldFile(c.files, parent, sid+".maintenance.json", fileSD, maxInstalledCompletionBytes)
	if err != nil {
		return result, err
	}
	raw, err := installedReadPin(ctx, completion, maxInstalledCompletionBytes)
	if err != nil {
		return result, err
	}
	record, err := parseInstalledCompletion(raw)
	if err != nil || record.ClientSID != sid {
		return result, errors.Join(errors.New("completed installation belongs to another user"), err)
	}
	c.record, c.originalRaw, c.requestParent = record, bytes.Clone(raw), parent
	originalParent, err := startupDirectoryIdentity(parent)
	if err != nil || originalParent != record.Parent {
		return result, errors.Join(errors.New("original installed parent identity changed"), err)
	}
	marker, err := maintenanceHoldFile(c.files, parent, sid+".maintenance.once", fileSD, maxInstalledCompletionMarkerBytes)
	if err != nil {
		return result, err
	}
	markerBytes, err := installedReadPin(ctx, marker, maxInstalledCompletionMarkerBytes)
	if err != nil || validateInstalledCompletionMarker(markerBytes, digest(raw)) != nil {
		return result, errors.New("original installation completion is absent or partial")
	}
	request, err := maintenanceHoldFile(c.files, parent, sid+".json", fileSD, brokerregistration.MaxRequestBytes)
	if err != nil {
		return result, err
	}
	requestBytes, err := installedReadPin(ctx, request, brokerregistration.MaxRequestBytes)
	if err != nil || !bytes.Equal(requestBytes, record.ClientRequest) {
		return result, errors.New("original installed request changed")
	}
	if err := installedMatchFile(ctx, request, record.Request); err != nil {
		return result, err
	}
	client, err := brokerregistration.ParseRetainedApprovedRequest(record.ClientRequest, record.ClientRequestSHA256)
	if err != nil {
		return result, err
	}
	c.baseline, err = holdClientBaselineFile(c.files, parent, sid+".baseline.zip", fileSD)
	if err != nil {
		return result, err
	}
	if err := installedMatchFile(ctx, c.baseline, record.Baseline); err != nil {
		return result, err
	}
	programFiles, err := windows.KnownFolderPath(windows.FOLDERID_ProgramFilesX86, 0)
	if err != nil {
		return result, err
	}
	code, err := c.files.holdAncestry(programFiles, false)
	if err != nil {
		return result, err
	}
	codeSD, err := enrollmentCodeDescriptor(broker, true)
	if err != nil {
		return result, err
	}
	for _, component := range []string{"SKAGER", "UpdateBroker"} {
		code, err = maintenanceHoldChild(c.files, code, component, codeSD, broker.String())
		if err != nil {
			return result, err
		}
	}
	imageSD, err := enrollmentCodeDescriptor(broker, false)
	if err != nil {
		return result, err
	}
	c.codeParent = code
	codeRecord := record
	if fixedHelper {
		codeRecord, err = readCodeSuccessorNative(ctx, c, record, raw, fileSD, imageSD)
		if err != nil {
			return result, err
		}
	} else if len(recoverMode) == 0 || !recoverMode[0] {
		for _, name := range []string{sid + codeDrainSuffix, sid + codeIntentSuffix, sid + codeReservationSuffix, sid + codeSuccessorSuffix, sid + codeSuccessorPairSuffix, sid + codeSuccessorMarkerSuffix, sid + codeStartAttemptSuffix, sid + codeDemandAttemptSuffix, sid + codeBrokerStageSuffix, sid + codeHelperStageSuffix, sid + codeEarlyAbortSuffix} {
			if err := startupRestorationAbsent(parent, name); err != nil {
				return result, err
			}
		}
	}
	var helper *enrollmentPin
	if len(recoverMode) == 0 || !recoverMode[0] {
		for _, item := range []struct {
			name     string
			identity startupFileBinding
		}{{enrollmentImageName, codeRecord.Broker}, {maintenanceImageName, codeRecord.MaintenanceHelper}} {
			pin, err := maintenanceHoldFile(c.files, code, item.name, imageSD, maxBrokerBytes)
			if err != nil {
				return result, err
			}
			if err := installedMatchFile(ctx, pin, item.identity); err != nil {
				return result, err
			}
			if item.name == maintenanceImageName {
				helper = pin
				c.helperImage = pin
			} else {
				c.brokerImage = pin
			}
		}
	}
	// Query the running process, not argv[0] or an environment path. A copied
	// helper may not use the fixed installed authority. Initial execution still
	// relies on the authenticated protected helper and Windows loader boundary.
	image := make([]uint16, 32768)
	length := uint32(len(image))
	if fixedHelper && (windows.QueryFullProcessImageName(windows.CurrentProcess(), 0, &image[0], &length) != nil || length == 0 || length >= uint32(len(image)) ||
		!strings.EqualFold(windows.UTF16ToString(image[:length]), filepath.Join(programFiles, "SKAGER", "UpdateBroker", maintenanceImageName))) {
		return result, errors.New("run the fixed original installed maintenance executable")
	}
	// Corroborate the actual running native image against the retained full
	// file identity using the same metadata-only observer as initial publication.
	if fixedHelper {
		if err := (&nativeMaintenanceImage{ctx: ctx, files: c.files, pin: helper}).Check(ctx); err != nil {
			return result, err
		}
	}
	publication, err := ParseApprovedStartupPublication(record.StartupPublication, record.StartupPublicationSHA256, record.ClientRequest, record.ClientRequestSHA256)
	if err != nil {
		return result, err
	}
	local, err := windows.KnownFolderPath(windows.FOLDERID_LocalAppData, 0)
	if err != nil {
		return result, err
	}
	localParent, err := startupHoldAncestry(c.files, local, sid)
	if err != nil {
		return result, err
	}
	recovery, err := maintenanceHoldChild(c.files, localParent, startupRecoveryDirectory, nil, sid)
	if err != nil {
		return result, err
	}
	actualPublication, err := startupOpenFile(c.files, recovery, startupBindingName, false, maxStartupBindingBytes)
	if err != nil {
		return result, err
	}
	if err := installedMatchFile(ctx, actualPublication, record.Publication); err != nil {
		return result, err
	}
	// The later startup owner reopens and pins the actual record and links using
	// the original publication bytes. No LocalAppData path is an approval source.
	if err := c.check(ctx); err != nil {
		return result, err
	}
	result = installedMaintenanceInputs{completion: c, authority: maintenanceAuthority{brokerSHA: codeRecord.Broker.SHA256, client: client}, publication: publication, baseline: completedBaselineView{owner: c, ctx: ctx}}
	return result, nil
}

func prepareInstalledFromCompletionNative(ctx context.Context, m *MaintenanceOwner, baseline ApprovedClientBaseline) error {
	if m == nil || m.self != m || m.completion == nil {
		return errors.New("original completed installation owner required")
	}
	return prepareInstalledUninstallWith(m, ctx, "original-installed-completion", func(ctx context.Context, authority maintenanceAuthority, _ string) (installedUninstallBackend, error) {
		inventory, err := collectRetainedInstalledInventory(ctx, authority, baseline)
		if err != nil {
			return nil, err
		}
		return openInstalledUninstallInventoryNative(ctx, authority, baseline, inventory)
	})
}

var _ io.ReaderAt = completedBaselineView{}
