//go:build windows

package brokerhost

import (
	"context"
	"errors"
	"path/filepath"
	"runtime"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/brokerstore"
	"golang.org/x/sys/windows"
)

type nativeClientRegistrationStore struct{ *brokerstore.Store }

func (s nativeClientRegistrationStore) beginRegistration(ctx context.Context, sid, channel, approvedSHA256 string) (clientRegistrationOwner, error) {
	owner, err := s.BeginRegistrationFresh(ctx, sid, channel, approvedSHA256)
	if owner == nil {
		return nil, err
	}
	return owner, err
}

// RegisterApprovedClientNative is an explicit service-side creation operation,
// never a normal pipe request or application-startup enrollment path. Before
// invoking it, the service caller MUST authenticate and retain the operator's
// exact protected original request and independently approved hash, with actual
// installed-byte/source and first-execution trust established independently.
// Present ACLs, hashes supplied by clients, mutable per-user installation records
// and executable self-reports cannot establish those prerequisites. This function
// does not authenticate that history, select/install an update or launch anything.
// It publishes the nonselected original baseline loading view only after private
// registration writers close. It requires the actual non-impersonated broker;
// the SCM caller separately retains the authenticated operator handoff.
func RegisterApprovedClientNative(ctx context.Context, originalRequest []byte, independentlyApprovedSHA256 string) error {
	// Keep token checks, native creation, verifier filesystem operations and
	// cleanup on the same actual non-impersonated broker thread.
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if err := registerApprovedClient(ctx, originalRequest, independentlyApprovedSHA256, time.Now().UTC(), clientRegistrationOperations{
		open: func() (clientRegistrationStore, error) {
			if err := brokerstore.ValidateInitialization(); err != nil {
				return nil, err
			}
			programData, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, windows.KF_FLAG_DEFAULT)
			if err != nil {
				return nil, errors.New("actual machine broker data unavailable")
			}
			store, err := brokerstore.Open(filepath.Join(programData, "SKAGER", "UpdateBroker"))
			if err != nil {
				return nil, err
			}
			return nativeClientRegistrationStore{store}, nil
		},
		observe:    observeStock,
		initialize: verifier.InitializeState,
		validate:   verifier.ValidateState,
		baseline:   importClientBaselineNative,
	}); err != nil {
		return err
	}
	// Registration's private writers must be closed before acquiring independent
	// read-only registration custody for the public baseline. A failure here does
	// not undo registration or make its permanent namespace safe to retry.
	if err := materializeRegisteredBaselineNative(ctx, originalRequest, independentlyApprovedSHA256); err != nil {
		return errors.Join(ErrClientRegistrationUncertain, err)
	}
	return nil
}

// The explicit approved SCM registration path produces the initial immutable
// public loading view as well as private baseline evidence. It remains unselected
// until an actual authenticated commissioning/startup transaction qualifies it.
func materializeRegisteredBaselineNative(ctx context.Context, raw []byte, approved string) (resultErr error) {
	if ctx == nil || ctx.Err() != nil {
		return errors.New("live registered-baseline publication context required")
	}
	request, err := brokerregistration.ParseRetainedApprovedRequest(raw, approved)
	if err != nil {
		return err
	}
	if err := brokerstore.ValidateInitialization(); err != nil {
		return err
	}
	programData, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, windows.KF_FLAG_DEFAULT)
	if err != nil {
		return err
	}
	store, err := brokerstore.Open(filepath.Join(programData, "SKAGER", "UpdateBroker"))
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, store.Close()) }()
	previous, err := store.OpenInstalledBaseline(ctx, request.ClientSID, request.Installed.Generation, approved, request.Installed)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, previous.Close()) }()
	view, err := store.MaterializeApprovedBaselineReadOnlyFresh(ctx, previous, raw, approved)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, view.Close()) }()
	// Releasing the private source must not revoke the independent public copy.
	if err := previous.Close(); err != nil {
		return err
	}
	return view.Validate()
}
