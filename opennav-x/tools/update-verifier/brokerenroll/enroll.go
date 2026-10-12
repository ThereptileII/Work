// Package brokerenroll provisions the fixed machine broker through an explicit
// independently authenticated operator invocation. Optional client registration
// requires a separate exact approved request and its authenticated baseline.
// It never launches an application, installs an update or authenticates itself.
package brokerenroll

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
	"example.com/opennav-update-verifier/repository"
)

const (
	serviceName                      = "SkagerUpdateBroker"
	serviceAccount                   = `NT SERVICE\SkagerUpdateBroker`
	maxTrustConfigurationBytes       = 576 << 10
	maxBrokerBytes             int64 = 64 << 20
	maxEnrollmentTime                = 30 * time.Minute
)

// ErrCleanup means retained native/input owners could not be released with a
// known result. Neither this result nor a partial attempt permits automatic
// adoption, deletion, rollback, trust reset or a second initialization attempt.
var ErrCleanup = errors.New("broker enrollment owner cleanup uncertain")

// Request contains public independently approved inputs, never private keys.
// Before invoking Run, an operator must authenticate the final provisioner and
// its loader chain/publisher, approve these exact public pins separately and
// establish the durable bootstrap cache's provenance. A self-hash, adjacent
// manifest, UAC prompt or present ACL cannot establish first-execution trust.
//
// Cache and artifact directories must already be independently provisioned for
// the operator. Native code checks and retains their current protected physical
// ancestry; it never creates, repairs, resets or adopts a bootstrap cache.
type Request struct {
	TrustConfiguration []byte
	BootstrapCacheDir  string
	ArtifactDirectory  string
	PolicySHA256       string
	ProvisionerSHA256  string
	TrustConfigSHA256  string
	// ClientRegistrationRequest is optional. Its exact original bytes and SHA
	// must be independently approved, rather than hashing a client declaration
	// or consulting mutable installed state to manufacture approval. Its original
	// trust bytes/channel must equal the approved bootstrap trust above.
	// It also requires approved-installed-baseline.zip in ArtifactDirectory,
	// authenticated by the installed identity pins in the exact approved request.
	ClientRegistrationRequest   []byte
	ApprovedClientRequestSHA256 string
}

type enrollmentFiles interface {
	ImagePath() string
	Check() error
	Close() error
}

type acquiredBroker interface {
	io.Reader
	io.Closer
}

type operations struct {
	holdDirectories  func(string, string) (io.Closer, error)
	verify           func(verifier.BrokerBootstrapRequest) (releasepolicy.BrokerBootstrapPolicy, []byte, error)
	acquire          func(context.Context, releasepolicy.Artifact, string, string) (acquiredBroker, error)
	create           func(context.Context, io.Reader, releasepolicy.Artifact) (enrollmentFiles, error)
	initialize       func(context.Context, enrollmentFiles) error
	validateClient   func(string) error
	openBaseline     func(context.Context, string) (ApprovedClientBaseline, error)
	validateBaseline func(context.Context, []byte, string, ApprovedClientBaseline) error
}

// Run verifies the distinct signed bootstrap and exact broker artifact, creates
// fresh protected fixed machine locations and a stopped virtual-account service,
// and invokes its bounded stopped-only initialization/validation. Success is
// local machine creation and, when separately requested, stopped-only client
// registration. It is not installed update authorization, application startup
// health or release qualification.
// Existing or partial machine objects are retained and refused, never adopted.
// Normal application startup never calls Run. Non-Windows platforms refuse.
func Run(ctx context.Context, req Request) error {
	return run(ctx, req, operations{
		holdDirectories: holdOperatorDirectories,
		verify:          verifier.VerifyBrokerBootstrap,
		acquire: func(ctx context.Context, a releasepolicy.Artifact, origin, directory string) (acquiredBroker, error) {
			return verifier.DownloadArtifact(ctx, a, origin, directory)
		},
		create:           createFreshFiles,
		initialize:       initializeService,
		validateClient:   validateClientSIDNative,
		openBaseline:     openClientBaselineInputNative,
		validateBaseline: validateApprovedClientBaseline,
	})
}

func run(ctx context.Context, req Request, op operations) (resultErr error) {
	if ctx == nil || ctx.Err() != nil {
		return errors.New("live enrollment context required")
	}
	ctx, cancel := context.WithTimeout(ctx, maxEnrollmentTime)
	defer cancel()
	if len(req.TrustConfiguration) == 0 || len(req.TrustConfiguration) > maxTrustConfigurationBytes ||
		!validPin(req.PolicySHA256) || !validPin(req.ProvisionerSHA256) || !validPin(req.TrustConfigSHA256) ||
		req.BootstrapCacheDir == "" || req.ArtifactDirectory == "" {
		return errors.New("independently approved enrollment inputs required")
	}
	// Copy once. Later caller mutations cannot change the selected trust bytes.
	trustBytes := bytes.Clone(req.TrustConfiguration)
	if digest(trustBytes) != req.TrustConfigSHA256 {
		return errors.New("trust configuration differs from approved SHA-256")
	}
	// Reuse the operator repository's exact private-beta public trust contract.
	// It validates the signed root and the pinned OpenCPN identity, not secrets.
	trust, err := repository.ValidateTrust(trustBytes, time.Now().UTC())
	if err != nil {
		return errors.New("approved private beta trust configuration required")
	}
	var client *brokerregistration.Request
	if len(req.ClientRegistrationRequest) != 0 || req.ApprovedClientRequestSHA256 != "" {
		parsed, err := brokerregistration.ParseApprovedRequest(req.ClientRegistrationRequest, req.ApprovedClientRequestSHA256, time.Now().UTC())
		if err != nil || parsed.TrustConfigSHA256 != req.TrustConfigSHA256 ||
			!bytes.Equal(parsed.TrustConfiguration, trustBytes) || parsed.Trust().Channel != trust.Channel {
			return errors.New("separately approved client request must share exact bootstrap trust")
		}
		if op.validateClient == nil || op.validateClient(parsed.ClientSID) != nil {
			return errors.New("canonical actual client account required")
		}
		client = &parsed
		if op.openBaseline == nil || op.validateBaseline == nil {
			return errors.New("approved installed baseline operations required")
		}
	}
	if op.holdDirectories == nil || op.verify == nil || op.acquire == nil || op.create == nil || op.initialize == nil {
		return errors.New("native enrollment operations required")
	}
	inputs, err := op.holdDirectories(req.BootstrapCacheDir, req.ArtifactDirectory)
	if err != nil || inputs == nil {
		if inputs != nil && inputs.Close() != nil {
			resultErr = ErrCleanup
		}
		if errors.Is(err, ErrCleanup) {
			resultErr = errors.Join(resultErr, ErrCleanup)
		}
		return errors.Join(errors.New("protected operator input directories required"), resultErr)
	}
	defer func() {
		if inputs.Close() != nil {
			resultErr = errors.Join(resultErr, ErrCleanup)
		}
	}()
	var baseline ApprovedClientBaseline
	if client != nil {
		baseline, err = op.openBaseline(ctx, req.ArtifactDirectory)
		if baseline != nil {
			defer func() {
				if baseline.Close() != nil {
					resultErr = errors.Join(resultErr, ErrCleanup)
				}
			}()
		}
		if err != nil || baseline == nil {
			return errors.Join(errors.New("retained protected approved baseline required"), enrollmentCleanupResult(err))
		}
		if err := baseline.Validate(); err != nil {
			return errors.Join(errors.New("approved baseline custody refused"), enrollmentCleanupResult(err))
		}
		if baseline.Size() < 1 || baseline.Size() > maxClientBaselineBytes {
			return errors.New("bounded approved baseline required")
		}
		if err := op.validateBaseline(ctx, client.OriginalBytes(), client.RequestSHA256, baseline); err != nil {
			return errors.Join(errors.New("baseline differs from independently approved installed identity"), enrollmentCleanupResult(err))
		}
		if err := baseline.Validate(); err != nil || ctx.Err() != nil {
			return errors.Join(errors.New("approved baseline custody changed or interrupted"), enrollmentCleanupResult(err))
		}
	}
	_, raw, err := op.verify(verifier.BrokerBootstrapRequest{
		MetadataURL: trust.MetadataURL, TrustedRoot: bytes.Clone(trust.BootstrapRoot),
		CacheDir: req.BootstrapCacheDir, Channel: trust.Channel, ArtifactOrigin: trust.ArtifactOrigin,
		ApprovedPolicySHA256: req.PolicySHA256, TrustedProvisionerSHA256: req.ProvisionerSHA256,
		TrustConfigSHA256: req.TrustConfigSHA256,
	})
	if err != nil || ctx.Err() != nil {
		return errors.New("signed broker bootstrap verification refused")
	}
	// Use only the exact verified target bytes, never a caller's parallel struct.
	policy, err := releasepolicy.ParseBrokerBootstrap(raw)
	if err != nil || digest(raw) != req.PolicySHA256 || policy.Channel != trust.Channel ||
		policy.Provisioner.SHA256 != req.ProvisionerSHA256 || policy.TrustConfigSHA256 != req.TrustConfigSHA256 ||
		policy.Broker.Bytes < 1 || policy.Broker.Bytes > maxBrokerBytes {
		return errors.New("approved broker bootstrap identity or size refused")
	}
	broker, err := op.acquire(ctx, policy.Broker, trust.ArtifactOrigin, req.ArtifactDirectory)
	if err != nil || broker == nil {
		if broker != nil && broker.Close() != nil {
			resultErr = ErrCleanup
		}
		if errors.Is(err, verifier.ErrArtifactCleanup) {
			resultErr = errors.Join(resultErr, ErrCleanup)
		}
		return errors.Join(errors.New("broker artifact acquisition refused"), resultErr)
	}
	defer func() {
		if broker.Close() != nil {
			resultErr = errors.Join(resultErr, ErrCleanup)
		}
	}()
	if ctx.Err() != nil {
		return errors.New("broker enrollment interrupted")
	}
	files, err := op.create(ctx, broker, policy.Broker)
	if err != nil || files == nil {
		if files != nil && files.Close() != nil {
			resultErr = ErrCleanup
		}
		if errors.Is(err, ErrCleanup) {
			resultErr = errors.Join(resultErr, ErrCleanup)
		}
		return errors.Join(errors.New("fresh protected machine files refused"), resultErr)
	}
	defer func() {
		if files.Close() != nil {
			resultErr = errors.Join(resultErr, ErrCleanup)
		}
	}()
	if ctx.Err() != nil {
		return errors.New("retained fresh machine files changed or interrupted")
	}
	if checkErr := files.Check(); checkErr != nil {
		if errors.Is(checkErr, ErrCleanup) {
			resultErr = ErrCleanup
		}
		return errors.Join(errors.New("retained fresh machine files changed or interrupted"), resultErr)
	}
	if client != nil {
		publisher, ok := files.(clientRequestFiles)
		if !ok {
			return errors.New("fresh client request custody required")
		}
		if err := publisher.publishClientRequest(ctx, client.OriginalBytes(), client.ClientSID, client.RequestSHA256); err != nil {
			if errors.Is(err, ErrCleanup) {
				resultErr = ErrCleanup
			}
			return errors.Join(errors.New("fresh protected client request publication refused"), resultErr)
		}
		if ctx.Err() != nil {
			return errors.New("client baseline publication interrupted")
		}
		if err := publisher.publishClientBaseline(ctx, baseline, client.ClientSID); err != nil {
			return errors.Join(errors.New("fresh protected client baseline publication refused"), enrollmentCleanupResult(err))
		}
		sid, approvedSHA := publisher.clientRegistration()
		if ctx.Err() != nil || sid != client.ClientSID || approvedSHA != client.RequestSHA256 {
			return errors.New("retained approved client request binding changed or interrupted")
		}
		if err := files.Check(); err != nil {
			return errors.Join(errors.New("retained client request custody refused"), enrollmentCleanupResult(err))
		}
		if err := baseline.Validate(); err != nil {
			return errors.Join(errors.New("retained approved baseline custody refused"), enrollmentCleanupResult(err))
		}
	}
	if err := op.initialize(ctx, files); err != nil {
		if errors.Is(err, ErrCleanup) {
			resultErr = ErrCleanup
		}
		return errors.Join(errors.New("stopped broker service initialization refused"), resultErr)
	}
	if ctx.Err() != nil {
		return errors.New("broker enrollment completion uncertain")
	}
	if checkErr := files.Check(); checkErr != nil {
		if errors.Is(checkErr, ErrCleanup) {
			resultErr = ErrCleanup
		}
		return errors.Join(errors.New("broker enrollment completion uncertain"), resultErr)
	}
	if baseline != nil {
		if err := baseline.Validate(); err != nil {
			return errors.Join(errors.New("approved baseline completion custody uncertain"), enrollmentCleanupResult(err))
		}
	}
	return nil
}

func validPin(value string) bool {
	if len(value) != 64 {
		return false
	}
	decoded, err := hex.DecodeString(value)
	return err == nil && hex.EncodeToString(decoded) == value
}
func digest(value []byte) string { sum := sha256.Sum256(value); return hex.EncodeToString(sum[:]) }

func enrollmentCleanupResult(err error) error {
	if errors.Is(err, ErrCleanup) {
		return ErrCleanup
	}
	return nil
}
