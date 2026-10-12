package brokerhost

import (
	"context"
	"encoding/json"
	"errors"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/brokerstore"
	"example.com/opennav-update-verifier/releasepolicy"
)

const clientRegistrationLimit = 5 * time.Minute

// ErrClientRegistrationUncertain means durable local registration may already
// exist. Preserve its request/fence/tree for explicit operator review; this is
// never permission to retry, adopt, delete, reset trust or launch an update.
var ErrClientRegistrationUncertain = errors.New("client registration may be completed; explicit operator review required")

type clientRegistrationOwner interface {
	VerifierDirectories() (string, string, error)
	Complete(context.Context, []byte) error
	Close() error
}

type clientRegistrationStore interface {
	Validate() error
	beginRegistration(context.Context, string, string, string) (clientRegistrationOwner, error)
	Close() error
}

// Private seams test ordering and failure handling. Production always constructs
// the actual Windows broker store, retained stock observation and real verifier.
// These callbacks are not caller-controlled registration or installation proof.
type clientRegistrationOperations struct {
	open       func() (clientRegistrationStore, error)
	observe    func(context.Context, registration) (releasepolicy.OpenCpn, stockOwner, error)
	initialize func(string, verifier.TrustConfig) error
	validate   func(string, verifier.TrustConfig) error
}

func registerApprovedClient(ctx context.Context, raw []byte, independentlyApprovedSHA256 string, now time.Time, op clientRegistrationOperations) (resultErr error) {
	completed := false
	defer func() {
		if resultErr != nil && (completed || errors.Is(resultErr, brokerstore.ErrRegistrationUncertain)) {
			resultErr = errors.Join(ErrClientRegistrationUncertain, resultErr)
		}
	}()
	if ctx == nil || ctx.Err() != nil || op.open == nil || op.observe == nil || op.initialize == nil || op.validate == nil {
		return errors.New("live native approved client registration required")
	}
	ctx, cancel := context.WithTimeout(ctx, clientRegistrationLimit)
	defer cancel()
	// Reparse exact original bytes at this authority boundary before touching the
	// machine. Typed structs, self-reports, adjacent hashes and LocalAppData
	// records are never substituted for independently authenticated approval.
	request, err := brokerregistration.ParseApprovedRequest(raw, independentlyApprovedSHA256, now)
	if err != nil {
		return err
	}
	protected, err := json.Marshal(protectedRegistration{Schema: 2, ClientSID: request.ClientSID, ApprovedRequestSHA256: request.RequestSHA256, ApprovedRequest: request.OriginalBytes()})
	if err != nil || len(protected) > brokerstore.MaxRegistrationBytes {
		return errors.New("bounded protected client registration required")
	}
	r := registrationFromApproved(request)
	store, err := op.open()
	if store != nil {
		defer func() { resultErr = errors.Join(resultErr, store.Close()) }()
	}
	if err != nil {
		return err
	}
	if store == nil {
		return errors.New("actual native broker store required")
	}
	check := func() error {
		if err := ctx.Err(); err != nil {
			return err
		}
		return store.Validate()
	}
	if err = check(); err != nil {
		return err
	}
	observed, stock, err := op.observe(ctx, r)
	if stock != nil {
		defer func() {
			if stock != nil {
				resultErr = errors.Join(resultErr, stock.Close())
			}
		}()
	}
	if err != nil {
		return err
	}
	if stock == nil || observed != stockBaseline {
		return errors.New("retained approved stock observation required")
	}
	if err = check(); err != nil {
		return err
	}
	owner, err := store.beginRegistration(ctx, request.ClientSID, r.Trust.Channel, request.RequestSHA256)
	if owner != nil {
		defer func() {
			if owner != nil {
				resultErr = errors.Join(resultErr, owner.Close())
			}
		}()
	}
	if err != nil {
		return err
	}
	if owner == nil {
		return errors.New("fresh native client registration owner required")
	}
	if err = check(); err != nil {
		return err
	}
	state, artifacts, err := owner.VerifierDirectories()
	if err != nil {
		return err
	}
	if state == "" || artifacts == "" || state == artifacts {
		return errors.New("distinct fixed native verifier directories required")
	}
	config := r.configuration(state, artifacts)
	if err = check(); err != nil {
		return err
	}
	// The once fence already exists. This is the sole fresh attempt; failure
	// retains the fence/tree and never resets, adopts or imports per-user trust.
	if err = op.initialize(state, config.Trust); err != nil {
		return err
	}
	if err = check(); err != nil {
		return err
	}
	if err = op.validate(state, config.Trust); err != nil {
		return err
	}
	if err = check(); err != nil {
		return err
	}
	if err = stock.validate(); err != nil {
		return err
	}
	if err = check(); err != nil {
		return err
	}
	if err = owner.Complete(ctx, protected); err != nil {
		return err
	}
	completed = true
	// Release retained writers before reporting success. A late close/custody
	// failure may retain durable completion and is explicitly uncertain; it
	// never permits retry, deletion or automatic trust/registration repair.
	closeErr := owner.Close()
	owner = nil
	if closeErr != nil {
		return closeErr
	}
	closeErr = stock.Close()
	stock = nil
	if closeErr != nil {
		return closeErr
	}
	// Complete's durable publication is irrevocable. Cancellation after that
	// boundary does not undo it or invalidate an otherwise successful handoff.
	// Actual remaining custody/close failures still report uncertainty.
	return store.Validate()
}
