//go:build windows

package brokerenroll

import (
	"context"
	"errors"
	"golang.org/x/sys/windows"
)

type nativeCodeStartBackend struct {
	*nativeCodeFinalizeBackend
	publication *nativeCodeFinalizePublication
	attempted   bool
}

func (n *nativeCodeStartBackend) check(ctx context.Context) error {
	if n == nil || n.nativeCodeFinalizeBackend == nil || n.service.expectedStart != windows.SERVICE_AUTO_START {
		return errors.New("exact automatic start configuration required")
	}
	return n.nativeCodeFinalizeBackend.check(ctx)
}

func (n *nativeCodeStartBackend) fence(ctx context.Context) error {
	if n.attempted {
		return errors.New("native start fence consumed")
	}
	n.attempted = true
	c := n.original
	sd := n.publication.descriptor
	pin, err := maintenanceHoldFile(c.files, c.requestParent, c.clientSID+codeSuccessorSuffix, sd, maxCodeUpgradeRecordBytes)
	if err != nil {
		return err
	}
	raw, err := installedReadPin(ctx, pin, maxCodeUpgradeRecordBytes)
	if err != nil {
		return err
	}
	r, err := parseCodeUpgradeRecord(raw)
	if err != nil {
		return err
	}
	attempt := codeStartAttemptBytes(digest(c.originalRaw), r.PolicySHA256, digest(raw), digest(codeSuccessorMarker(digest(raw))))
	if len(attempt) == 0 {
		return errors.New("canonical start attempt required")
	}
	if err = startupRestorationAbsent(c.requestParent, c.clientSID+codeStartAttemptSuffix); err != nil {
		return err
	}
	_, err = writeCodeUpgradeFileNative(ctx, n.publication.files, c.requestParent, c.clientSID+codeStartAttemptSuffix, sd, attempt)
	return err
}
func (n *nativeCodeStartBackend) start() error {
	if !n.attempted {
		return errors.New("durable attempt required before start")
	}
	return windows.StartService(n.service.service, 0, nil)
}

// Drop the START-capable service owner after retaining the actual process. The
// finalizer receives a freshly validated query-only service observer, while its
// process/token/image/birth owner is transferred intact.
func (n *nativeCodeStartBackend) finalization(ctx context.Context) (codeFinalizeBackend, error) {
	observer, err := openCodeFinalizeObserverNative(ctx, n.original, n.brokerSID)
	if err != nil {
		return nil, err
	}
	if observer.service.expectedStart != windows.SERVICE_AUTO_START {
		return nil, errors.Join(errors.New("automatic service changed during observer handoff"), observer.close())
	}
	observer.image = n.image
	if err = n.nativeCodeFinalizeBackend.close(); err != nil {
		return nil, errors.Join(err, observer.close())
	}
	n.nativeCodeFinalizeBackend = observer
	return observer, nil
}
