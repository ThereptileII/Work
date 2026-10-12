//go:build windows

package brokerenroll

import (
	"context"
	"errors"
	"io"
	"runtime"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

const startupRecoveryDirectory = "SKAGER-Broker-Startup-Recovery"
const startupShortcutName = "Skager.lnk"
const maxStartupShortcutBytes = 1 << 20

func startupCurrentClientNative(ctx context.Context, raw []byte, sha string) error {
	if ctx == nil || ctx.Err() != nil {
		return errors.New("live explicit startup context required")
	}
	r, err := brokerregistration.ParseApprovedRequest(raw, sha, time.Now().UTC())
	if err != nil {
		return err
	}
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	return startupCurrentSID(r.ClientSID)
}
func startupCurrentSID(sid string) error {
	if err := enrollmentOperator(); err != nil {
		return err
	}
	actual, err := windows.GetCurrentProcessToken().GetTokenUser()
	if err != nil || actual == nil || actual.User.Sid == nil || actual.User.Sid.String() != sid || !canonicalClientSID(sid) {
		return errors.Join(errors.New("actual current operator user must equal approved registered client; no other profile guessing"), err)
	}
	return nil
}
func initializeStartupNative(ctx context.Context, files enrollmentFiles, client *brokerregistration.Request, baseline ApprovedClientBaseline) error {
	completionReached := false
	err := initializeServiceWithCompletion(ctx, files, newServiceBackend, servicePoll, func(ctx context.Context, files enrollmentFiles, service serviceBackend) error {
		completionReached = true
		return completeStartup(ctx, files, service, client, baseline, publishStartupNative, servicePoll)
	})
	if err != nil && completionReached {
		return errors.Join(ErrStartupUncertain, err)
	}
	return err
}

type startupBaselineSink struct {
	discardBaselineSink
	launcherSHA  string
	launcherSize int64
}

func (s *startupBaselineSink) WriteFile(ctx context.Context, name string, source io.Reader, size int64, sha string) error {
	if err := s.discardBaselineSink.WriteFile(ctx, name, source, size, sha); err != nil {
		return err
	}
	if name == "app/skager-start.exe" {
		s.launcherSHA, s.launcherSize = sha, size
	}
	return nil
}
func startupLauncherIdentity(ctx context.Context, client *brokerregistration.Request, baseline ApprovedClientBaseline) (string, int64, error) {
	if client == nil || baseline == nil {
		return "", 0, errors.New("original approved baseline required")
	}
	if err := baseline.Validate(); err != nil {
		return "", 0, err
	}
	sink := &startupBaselineSink{}
	if _, err := verifier.ImportApprovedBaseline(ctx, client.OriginalBytes(), client.RequestSHA256, baseline, baseline.Size(), sink); err != nil {
		return "", 0, err
	}
	if err := baseline.Validate(); err != nil {
		return "", 0, err
	}
	if !validPin(sink.launcherSHA) || sink.launcherSize < 1 || sink.launcherSize > maxBrokerBytes {
		return "", 0, errors.New("exact bounded inventoried original baseline launcher required")
	}
	return sink.launcherSHA, sink.launcherSize, nil
}
