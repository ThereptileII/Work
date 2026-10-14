//go:build windows

package brokerenroll

import (
	"context"
	"encoding/hex"
	"errors"
	"fmt"
	"os"
	"runtime"
	"strconv"
	"testing"
	"time"

	"golang.org/x/sys/windows"
)

// Build this qualification-only executable with go test -c ./brokerenroll.
// Normal test invocations retain their ordinary testing entry point. The private
// command accepts no product operation or caller-selected child filename.
func TestMain(m *testing.M) {
	// Approval transport is independently bounded to one hour; allow its wait
	// plus a minute for this fixed local writer. No product execution occurs here.
	ctx, cancel := context.WithTimeout(context.Background(), 61*time.Minute)
	handled, err := qualificationChallengeBridge(ctx, os.Args[1:], os.Stdin, os.Stdout, openQualificationChallengeNative)
	cancel()
	if !handled {
		os.Exit(m.Run())
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, "Qualification challenge consumption refused or uncertain; preserve state and do not reset or retry.")
		os.Exit(1)
	}
	os.Exit(0)
}

func qualificationChallengeNativeIdentity(pin *enrollmentPin) (qualificationChallengeIdentity, error) {
	id, err := startupDirectoryIdentity(pin)
	if err != nil {
		return qualificationChallengeIdentity{}, err
	}
	return qualificationChallengeIdentity{VolumeSerial: strconv.FormatUint(id.Volume, 10), FileID: hex.EncodeToString(id.FileID[:]), CreationTime: strconv.FormatUint(id.Creation, 10), DescriptorSHA256: id.DescriptorSHA256}, nil
}

type nativeQualificationChallengeOwner struct {
	files     *nativeEnrollmentFiles
	parent    *enrollmentPin
	original  qualificationChallengeIdentity
	attempted bool
	liveJob   bool
}

func openQualificationChallengeNative(ctx context.Context, path string) (qualificationChallengeOwner, error) {
	return openQualificationChallengeParentNative(ctx, path, true)
}

func openQualificationChallengeParentNative(ctx context.Context, parentPath string, liveJob bool) (result *nativeQualificationChallengeOwner, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if ctx == nil || ctx.Err() != nil || liveJob && os.Getenv("GITHUB_ACTIONS") != "true" {
		return nil, errors.New("live original native qualification owner required")
	}
	n := &nativeQualificationChallengeOwner{files: &nativeEnrollmentFiles{}, liveJob: liveJob}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, n.close())
		}
	}()
	var err error
	n.parent, err = n.files.holdAncestry(parentPath, true)
	if err != nil {
		return nil, err
	}
	n.parent.allowAdds = false
	if err := n.files.Check(); err != nil {
		return nil, err
	}
	n.original, err = qualificationChallengeNativeIdentity(n.parent)
	if err != nil {
		return nil, err
	}
	return n, nil
}

func (n *nativeQualificationChallengeOwner) identity(ctx context.Context) (qualificationChallengeIdentity, error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	if n == nil || n.files == nil || n.parent == nil || ctx == nil || ctx.Err() != nil {
		return qualificationChallengeIdentity{}, errors.New("live retained original qualification parent required")
	}
	if err := n.files.Check(); err != nil {
		return qualificationChallengeIdentity{}, err
	}
	actual, err := qualificationChallengeNativeIdentity(n.parent)
	if err != nil || actual != n.original {
		return qualificationChallengeIdentity{}, errors.Join(errors.New("original qualification parent changed"), err)
	}
	return n.original, nil
}

func (n *nativeQualificationChallengeOwner) close() error {
	if n == nil || n.files == nil {
		return nil
	}
	return n.files.Close()
}

func (n *nativeQualificationChallengeOwner) consume(ctx context.Context, raw []byte, record qualificationChallengeRecord) (receipt qualificationChallengeReceipt, resultErr error) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	parsed, err := parseQualificationChallenge(raw)
	if n == nil || n.attempted || err != nil || parsed != record {
		return receipt, errors.Join(errors.New("one exact qualification consumption required"), err)
	}
	parentID, err := n.identity(ctx)
	if err != nil {
		return receipt, err
	}
	// The coordinator independently authenticates job ID/name and approval. These
	// inherited native runner fields corroborate the live pending tuple as well.
	if n.liveJob && (os.Getenv("GITHUB_ACTIONS") != "true" || os.Getenv("GITHUB_REPOSITORY") != record.Repository || os.Getenv("GITHUB_SHA") != record.HeadSHA || os.Getenv("GITHUB_RUN_ID") != record.RunID || os.Getenv("GITHUB_RUN_ATTEMPT") != record.RunAttempt) {
		return receipt, errors.New("actual pending qualification runner differs")
	}
	sd, err := windows.SecurityDescriptorFromString("O:BAD:P(A;;FA;;;SY)(A;;FA;;;BA)")
	if err != nil {
		return receipt, err
	}
	name := ".installed-b0-consumed-" + record.Challenge + ".json"
	n.attempted = true
	pin, err := writeCodeUpgradeFileNative(ctx, n.files, n.parent, name, sd, raw)
	if err != nil {
		return receipt, err // Existing, zero-length and partial records all refuse.
	}
	recordID, err := qualificationChallengeNativeIdentity(pin)
	if err != nil {
		return receipt, err
	}
	if err := errors.Join(ctx.Err(), n.files.Check(), startupHashPin(ctx, pin, digest(raw))); err != nil {
		return receipt, err
	}
	receipt = qualificationChallengeReceipt{Schema: 1, State: "consumed", RecordSHA256: digest(raw), Parent: parentID, Record: recordID}
	return receipt, nil
}
