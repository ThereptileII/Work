package releasepolicy

import "errors"

type Installed struct {
	Version string
	Commit  string
}
type Decision string

const (
	Upgrade         Decision = "upgrade"
	Current         Decision = "current"
	Downgrade       Decision = "downgrade"
	Incompatible    Decision = "incompatible"
	ChannelMismatch Decision = "channelmismatch"
	VersionConflict Decision = "versionconflict"
)

// Evaluate is a pure classification, not authorization to install. observed
// must match both signed policy and an independently trusted local allowlist.
func Evaluate(p Policy, installed Installed, selectedChannel string, observed OpenCpn, localAllowlist []OpenCpn) (Decision, error) {
	if err := validate(p); err != nil {
		return "", err
	}
	current, err := ParseSemVer(installed.Version)
	if err != nil {
		return "", err
	}
	next, _ := ParseSemVer(p.Version)
	if !hex40.MatchString(installed.Commit) {
		return "", errors.New("invalid installed commit")
	}
	if selectedChannel != "beta" && selectedChannel != "stable" {
		return "", errors.New("invalid selected channel")
	}
	if selectedChannel != p.Channel {
		return ChannelMismatch, nil
	}
	if err := validateOpenCpn(observed); err != nil {
		return Incompatible, nil
	}
	allowedPolicy := false
	for _, c := range p.SupportedOpenCpn {
		if c == observed {
			allowedPolicy = true
			break
		}
	}
	allowedLocal := false
	for _, c := range localAllowlist {
		if c == observed {
			allowedLocal = true
			break
		}
	}
	if !allowedPolicy || !allowedLocal {
		return Incompatible, nil
	}
	switch next.Compare(current) {
	case -1:
		return Downgrade, nil
	case 1:
		return Upgrade, nil
	default:
		if installed.Commit != p.Commit {
			return VersionConflict, nil
		}
		return Current, nil
	}
}
