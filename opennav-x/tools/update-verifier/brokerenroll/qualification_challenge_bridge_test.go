package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"regexp"
	"strconv"
	"strings"
)

// Qualification transport only: none of these test sources is linked into a
// shipped helper. Approval authentication and original parent provenance belong
// to the pending-job coordinator; this owner durably consumes its exact bytes.
const qualificationChallengeFlag = "--consume-installed-b0-challenge"
const qualificationChallengePurpose = "installed-updater-b0-consumption"
const qualificationChallengeLimit = 4096

// Field order matches installed_updater_approval._encode's sorted ASCII keys.
type qualificationChallengeRecord struct {
	ApprovalSHA256 string `json:"approvalSha256"`
	Challenge      string `json:"challenge"`
	HeadSHA        string `json:"headSha"`
	JobID          string `json:"jobId"`
	JobName        string `json:"jobName"`
	Purpose        string `json:"purpose"`
	Repository     string `json:"repository"`
	RequestSHA256  string `json:"requestSha256"`
	RunAttempt     string `json:"runAttempt"`
	RunID          string `json:"runId"`
	Schema         int    `json:"schema"`
}

type qualificationChallengeIdentity struct {
	VolumeSerial     string `json:"volumeSerial"`
	FileID           string `json:"fileID"`
	CreationTime     string `json:"creationTime"`
	DescriptorSHA256 string `json:"descriptorSha256"`
}

type qualificationChallengeReceipt struct {
	Schema       int                            `json:"schema"`
	State        string                         `json:"state"`
	RecordSHA256 string                         `json:"recordSha256"`
	Parent       qualificationChallengeIdentity `json:"parent"`
	Record       qualificationChallengeIdentity `json:"record"`
}

type qualificationChallengeReady struct {
	Schema int                            `json:"schema"`
	State  string                         `json:"state"`
	Parent qualificationChallengeIdentity `json:"parent"`
}

type qualificationChallengeOwner interface {
	identity(context.Context) (qualificationChallengeIdentity, error)
	consume(context.Context, []byte, qualificationChallengeRecord) (qualificationChallengeReceipt, error)
	close() error
}

var qualificationDecimal = regexp.MustCompile(`^[1-9][0-9]{0,19}$`)
var qualificationJobName = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_.() -]{0,199}$`)
var qualificationSHA40 = regexp.MustCompile(`^[a-f0-9]{40}$`)
var qualificationFileID = regexp.MustCompile(`^[a-f0-9]{32}$`)

func qualificationChallengeArgs(args []string) (parent string, handled bool, err error) {
	for _, arg := range args {
		name, _, _ := strings.Cut(arg, "=")
		if name == qualificationChallengeFlag {
			handled = true
		}
	}
	if !handled {
		return "", false, nil
	}
	if len(args) != 1 || !strings.HasPrefix(args[0], qualificationChallengeFlag+"=") {
		return "", true, errors.New("one exact private qualification parent required")
	}
	parent = strings.TrimPrefix(args[0], qualificationChallengeFlag+"=")
	if parent == "" || strings.ContainsRune(parent, '\x00') {
		return "", true, errors.New("original protected qualification parent required")
	}
	return parent, true, nil
}

func parseQualificationChallenge(raw []byte) (qualificationChallengeRecord, error) {
	var r qualificationChallengeRecord
	decoder := json.NewDecoder(bytes.NewReader(raw))
	decoder.DisallowUnknownFields()
	if len(raw) == 0 || len(raw) > qualificationChallengeLimit || decoder.Decode(&r) != nil {
		return r, errors.New("bounded canonical qualification record required")
	}
	canonical, err := json.Marshal(r)
	if err != nil || !bytes.Equal(append(canonical, '\n'), raw) || r.Schema != 1 || r.Purpose != qualificationChallengePurpose || r.Repository != "ThereptileII/Work" || !qualificationSHA40.MatchString(r.HeadSHA) || !qualificationDecimal.MatchString(r.RunID) || !qualificationDecimal.MatchString(r.RunAttempt) || !qualificationDecimal.MatchString(r.JobID) || !qualificationJobName.MatchString(r.JobName) || strings.TrimSpace(r.JobName) != r.JobName || strings.HasSuffix(r.JobName, ".") || !validPin(r.Challenge) || !validPin(r.RequestSHA256) || !validPin(r.ApprovalSHA256) {
		return r, errors.New("exact approved qualification tuple and canonical bytes required")
	}
	return r, nil
}

func validQualificationIdentity(id qualificationChallengeIdentity) bool {
	_, volumeErr := strconv.ParseUint(id.VolumeSerial, 10, 64)
	_, creationErr := strconv.ParseUint(id.CreationTime, 10, 64)
	return volumeErr == nil && creationErr == nil && qualificationDecimal.MatchString(id.VolumeSerial) && qualificationDecimal.MatchString(id.CreationTime) && qualificationFileID.MatchString(id.FileID) && id.FileID != strings.Repeat("0", 32) && validPin(id.DescriptorSHA256)
}

func writeQualificationFrame(output io.Writer, value any) error {
	encoded, err := json.Marshal(value)
	if err != nil || len(encoded)+1 > qualificationChallengeLimit {
		return errors.New("bounded native qualification frame required")
	}
	encoded = append(encoded, '\n')
	n, err := output.Write(encoded)
	if err == nil && n != len(encoded) {
		err = io.ErrShortWrite
	}
	return err
}

func qualificationChallengeBridge(ctx context.Context, args []string, input io.Reader, output io.Writer, open func(context.Context, string) (qualificationChallengeOwner, error)) (handled bool, resultErr error) {
	parent, handled, err := qualificationChallengeArgs(args)
	if !handled || err != nil {
		return handled, err
	}
	if ctx == nil || ctx.Err() != nil || input == nil || output == nil || open == nil {
		return true, errors.New("live native qualification consumer required")
	}
	owner, err := open(ctx, parent)
	closed := false
	if owner != nil {
		defer func() {
			if !closed {
				resultErr = errors.Join(resultErr, owner.close())
			}
		}()
	}
	if err != nil || owner == nil {
		return true, errors.Join(errors.New("original native qualification parent required"), err)
	}
	parentID, err := owner.identity(ctx)
	if err != nil || !validQualificationIdentity(parentID) {
		return true, errors.Join(errors.New("retained original qualification parent identity required"), err)
	}
	// Ready is not consumption or approval. Keep THIS native ancestry alive while
	// the coordinator waits for independent approval; never reopen another parent.
	if err := writeQualificationFrame(output, qualificationChallengeReady{1, "ready", parentID}); err != nil {
		return true, err
	}
	type inputResult struct {
		raw []byte
		err error
	}
	read := make(chan inputResult, 1)
	go func() {
		raw, err := io.ReadAll(io.LimitReader(input, qualificationChallengeLimit+1))
		read <- inputResult{raw, err}
	}()
	var received inputResult
	select {
	case received = <-read:
	case <-ctx.Done():
		return true, ctx.Err() // No native effect was requested by the reader.
	}
	if received.err != nil {
		return true, errors.New("complete bounded qualification input required")
	}
	raw := received.raw
	r, err := parseQualificationChallenge(raw)
	if err != nil {
		return true, err
	}
	if err := ctx.Err(); err != nil {
		return true, err
	}
	observed, err := owner.identity(ctx)
	if err != nil || observed != parentID || ctx.Err() != nil {
		return true, errors.Join(errors.New("original ready parent custody changed"), err)
	}
	receipt, err := owner.consume(ctx, raw, r)
	if err != nil {
		return true, err // No success output after partial or uncertain publication.
	}
	if ctx.Err() != nil || receipt.Schema != 1 || receipt.State != "consumed" || receipt.RecordSHA256 != digest(raw) || receipt.Parent != parentID || !validQualificationIdentity(receipt.Record) || receipt.Record.VolumeSerial != parentID.VolumeSerial {
		return true, errors.New("original consumed record identity unverified")
	}
	if err := owner.close(); err != nil {
		closed = true
		return true, err
	}
	closed = true
	return true, writeQualificationFrame(output, receipt)
}
