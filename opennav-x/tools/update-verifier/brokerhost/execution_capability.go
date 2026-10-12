package brokerhost

import (
	"context"
	"errors"

	"example.com/opennav-update-verifier/brokerstore"
)

// The producer's fixed original sidecar binds this exact executable's tested
// qualification protocol. It is provenance/capability evidence only, never
// health, commissioning, selected-chain, profile, peer or spawn authority.
type executionQualificationCapability struct {
	Schema                int    `json:"schema"`
	Purpose               string `json:"purpose"`
	Commit                string `json:"commit"`
	ExecutableSHA256      string `json:"executableSha256"`
	ConfigurationProtocol int    `json:"configurationProtocol"`
	CoreOutputSuppression int    `json:"coreOutputSuppression"`
	PluginPolicy          string `json:"pluginPolicy"`
}

// This private read seam permits fault tests only. The actual factory accepts
// solely sealed GenerationApplicationImage, not this interface, paths, supplied
// sidecars, callback authorities or serialized image identities.
type executionCapabilityImage interface {
	Identity(context.Context) (brokerstore.GenerationApplicationIdentity, error)
	QualificationBytes(context.Context) ([]byte, error)
}

func parseExecutionCapability(raw []byte, identity brokerstore.GenerationApplicationIdentity) error {
	var capability executionQualificationCapability
	if len(raw) == 0 || len(raw) > 4096 || !commitPattern.MatchString(identity.Commit) || !hashPattern.MatchString(identity.ExecutableSHA256) ||
		exactExecutionRecord(raw, &capability) != nil || capability.Schema != 1 || capability.Purpose != "skager-qualified-update-v1" ||
		capability.Commit != identity.Commit || capability.ExecutableSHA256 != identity.ExecutableSHA256 ||
		capability.ConfigurationProtocol != 1 || capability.CoreOutputSuppression != 1 || capability.PluginPolicy != "deferred-until-ordinary-start" {
		return ErrExecutionChildRefused
	}
	return nil
}

// This gate runs before the native factory can arm a pipe or return health
// environment fields. Original live custody is checked on both sides of the
// exact sidecar read; parser success alone cannot create a native observer.
func validateExecutionCapability(ctx context.Context, image executionCapabilityImage) (brokerstore.GenerationApplicationIdentity, error) {
	var empty brokerstore.GenerationApplicationIdentity
	if ctx == nil || image == nil {
		return empty, ErrExecutionChildRefused
	}
	if err := ctx.Err(); err != nil {
		return empty, errors.Join(ErrExecutionChildRefused, err)
	}
	before, err := image.Identity(ctx)
	if err != nil {
		return empty, errors.Join(ErrExecutionChildRefused, err)
	}
	raw, err := image.QualificationBytes(ctx)
	if err != nil {
		return empty, errors.Join(ErrExecutionChildRefused, err)
	}
	if err := parseExecutionCapability(raw, before); err != nil {
		return empty, err
	}
	after, err := image.Identity(ctx)
	if err != nil || after != before {
		return empty, errors.Join(ErrExecutionChildRefused, err)
	}
	if err := ctx.Err(); err != nil {
		return empty, errors.Join(ErrExecutionChildRefused, err)
	}
	return before, nil
}
