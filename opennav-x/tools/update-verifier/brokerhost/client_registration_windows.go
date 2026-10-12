//go:build windows

package brokerhost

import (
	"context"
	"errors"
	"path/filepath"
	"runtime"
	"time"

	verifier "example.com/opennav-update-verifier"
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
// does not authenticate that history, establish loader custody, install an update
// or launch anything. It requires the actual non-impersonated native broker.
// SCM dispatch and the authenticated operator handoff are separate integrations.
func RegisterApprovedClientNative(ctx context.Context, originalRequest []byte, independentlyApprovedSHA256 string) error {
	// Keep token checks, native creation, verifier filesystem operations and
	// cleanup on the same actual non-impersonated broker thread.
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	return registerApprovedClient(ctx, originalRequest, independentlyApprovedSHA256, time.Now().UTC(), clientRegistrationOperations{
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
	})
}
