package repository

import (
	"bytes"
	"encoding/json"
	"errors"
	"os"
	"path"
	"path/filepath"
	"strings"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
	"github.com/theupdateframework/go-tuf/v2/metadata"
)

// Preserve exact independently approved bytes and their separate version floor.
// No machine identity belongs in this reusable signed declaration.
type brokerCodeUpgradeState struct {
	Policy []byte `json:"policy"`
	SHA256 string `json:"sha256"`
}

// PublishWithBrokerCodeUpgrade publishes a separately approved machine-code
// upgrade alongside the selected app. The approval pin is independent of the
// input file; the declaration grants no enrollment or local execution authority.
func PublishWithBrokerCodeUpgrade(directory, assets string, selection, policy []byte, selectionSHA string, upgrade []byte, upgradeSHA string, keys Keys, now time.Time) (int64, error) {
	return PublishWithBrokerBootstrapAndCodeUpgrade(directory, assets, selection, policy, selectionSHA, nil, "", upgrade, upgradeSHA, keys, now)
}

// PublishWithBrokerBootstrapAndCodeUpgrade can also update the reusable
// enrollment declaration. Both independently approved targets keep their
// distinct purposes and exact bytes; neither substitutes for the other.
func PublishWithBrokerBootstrapAndCodeUpgrade(directory, assets string, selection, policy []byte, selectionSHA string, bootstrap []byte, bootstrapSHA string, upgrade []byte, upgradeSHA string, keys Keys, now time.Time) (int64, error) {
	if len(upgrade) == 0 || upgradeSHA == "" || (len(bootstrap) == 0) != (bootstrapSHA == "") {
		return 0, errors.New("explicit broker code upgrade policy and approval pin required")
	}
	return publishBatchWithCodeUpgrade(directory, assets, selection, policy, selectionSHA, bootstrap, bootstrapSHA, upgrade, upgradeSHA, keys, now, nil)
}

func retainedCodeUpgrade(s *brokerCodeUpgradeState, trust Trust, configSHA string) (releasepolicy.BrokerCodeUpgradePolicy, error) {
	var empty releasepolicy.BrokerCodeUpgradePolicy
	if s == nil || !hashRE.MatchString(s.SHA256) || checksum(s.Policy) != s.SHA256 {
		return empty, errors.New("retained code upgrade approval identity refused")
	}
	p, e := releasepolicy.ParseBrokerCodeUpgrade(s.Policy)
	if e != nil || p.Bootstrap.Channel != trust.Channel || p.Bootstrap.TrustConfigSHA256 != configSHA {
		return empty, errors.New("code upgrade configuration identity refused")
	}
	for _, a := range bootstrapArtifacts(p.Bootstrap) {
		if a.URL != trust.ArtifactOrigin+"/"+a.Path {
			return empty, errors.New("code upgrade artifact origin refused")
		}
	}
	return p, nil
}

func codeUpgradeDestination(directory string, p releasepolicy.BrokerCodeUpgradePolicy, s *brokerCodeUpgradeState) (string, error) {
	target, e := releasepolicy.BrokerCodeUpgradeTargetPath(p.Bootstrap.Channel)
	if e != nil {
		return "", e
	}
	return filepath.Join(directory, "public", "targets", filepath.FromSlash(path.Dir(target)), s.SHA256+"."+path.Base(target)), nil
}

// Inspect existing ancestors even when the final policy directory is absent;
// a missing child must not hide a link or immutable Windows filename alias.
func preflightCodeUpgradeDestination(dest string, approved []byte) error {
	for parent := filepath.Dir(dest); ; parent = filepath.Dir(parent) {
		if _, e := os.Lstat(parent); e == nil {
			if e = plain(parent); e != nil {
				return e
			}
			break
		} else if !os.IsNotExist(e) {
			return e
		}
		if parent == filepath.Dir(parent) {
			return errors.New("policy parent unavailable")
		}
	}
	return preflightBootstrapDestination(dest, releasepolicy.Artifact{SHA256: checksum(approved), Bytes: int64(len(approved))})
}

func selectCodeUpgrade(directory, assets string, raw []byte, approvedSHA string, s state, trust Trust, app releasepolicy.Policy, bootstrap releasepolicy.BrokerBootstrapPolicy, selectedBootstrap *brokerBootstrapState) (releasepolicy.BrokerCodeUpgradePolicy, *brokerCodeUpgradeState, []bootstrapCopy, error) {
	var p releasepolicy.BrokerCodeUpgradePolicy
	fail := func() (releasepolicy.BrokerCodeUpgradePolicy, *brokerCodeUpgradeState, []bootstrapCopy, error) {
		return releasepolicy.BrokerCodeUpgradePolicy{}, nil, nil, errors.New("broker code upgrade selection refused; preserve operator state")
	}
	explicit := raw != nil || approvedSHA != ""
	retained := s.BrokerCodeUpgrade
	// A later declaration cannot hide corruption of retained authority or its
	// immutable bytes. Only the same explicit approval may resume missing files.
	if retained != nil {
		old, e := retainedCodeUpgrade(retained, trust, s.ConfigSHA256)
		if e != nil {
			return fail()
		}
		repair := explicit && approvedSHA == retained.SHA256 && bytes.Equal(raw, retained.Policy)
		dest, e := codeUpgradeDestination(directory, old, retained)
		if e != nil {
			return fail()
		}
		if e = preflightCodeUpgradeDestination(dest, retained.Policy); e != nil {
			return fail()
		}
		if _, e = os.Lstat(dest); e == nil {
			published, e := readPlain(dest, int64(len(retained.Policy)))
			if e != nil || !bytes.Equal(published, retained.Policy) {
				return fail()
			}
		} else if !os.IsNotExist(e) || !repair {
			return fail()
		}
		for _, a := range bootstrapArtifacts(old.Bootstrap) {
			dest := filepath.Join(directory, "public", filepath.FromSlash(a.Path))
			if e := preflightBootstrapDestination(dest, a); e != nil {
				return fail()
			}
			if _, e := os.Lstat(dest); e != nil && (!os.IsNotExist(e) || !repair) {
				return fail()
			}
		}
	}
	if explicit {
		if len(raw) == 0 || !hashRE.MatchString(approvedSHA) || checksum(raw) != approvedSHA {
			return fail()
		}
		next := &brokerCodeUpgradeState{Policy: append([]byte(nil), raw...), SHA256: approvedSHA}
		var e error
		p, e = retainedCodeUpgrade(next, trust, s.ConfigSHA256)
		if e != nil || p.Bootstrap.Version != app.Version || p.Bootstrap.Commit != app.Commit {
			return fail()
		}
		version, _ := releasepolicy.ParseSemVer(p.Bootstrap.Version)
		if retained != nil {
			old, e := retainedCodeUpgrade(retained, trust, s.ConfigSHA256)
			if e != nil {
				return fail()
			}
			previous, _ := releasepolicy.ParseSemVer(old.Bootstrap.Version)
			order := version.Compare(previous)
			if order < 0 || order == 0 && next.SHA256 != retained.SHA256 {
				return fail()
			}
			if order > 0 && (p.PreviousBrokerSHA256 != old.Bootstrap.Broker.SHA256 || p.PreviousHelperSHA256 != old.Bootstrap.Provisioner.SHA256) {
				return fail()
			}
		} else {
			if s.BrokerBootstrap == nil {
				return fail()
			}
			old, e := retainedBootstrap(s.BrokerBootstrap, trust, s.ConfigSHA256)
			if e != nil {
				return fail()
			}
			previous, _ := releasepolicy.ParseSemVer(old.Version)
			if version.Compare(previous) <= 0 || p.PreviousBrokerSHA256 != old.Broker.SHA256 || p.PreviousHelperSHA256 != old.Provisioner.SHA256 {
				return fail()
			}
		}
		retained = next
	} else if retained != nil {
		var e error
		p, e = retainedCodeUpgrade(retained, trust, s.ConfigSHA256)
		if e != nil {
			return fail()
		}
	} else {
		return p, nil, nil, nil
	}

	// Reject deterministic collisions across all retained and selected targets
	// before reserving authority. Shared source/notices require exact identity.
	union := append(artifacts(app), bootstrapArtifacts(p.Bootstrap)...)
	if selectedBootstrap != nil {
		union = append(union, bootstrapArtifacts(bootstrap)...)
	}
	seen := map[string]releasepolicy.Artifact{}
	for _, a := range union {
		folded := strings.ToLower(a.Path)
		if old, ok := seen[folded]; ok && old != a {
			return fail()
		}
		seen[folded] = a
		if e := preflightBootstrapDestination(filepath.Join(directory, "public", filepath.FromSlash(a.Path)), a); e != nil {
			return fail()
		}
	}
	// Existing policy bytes are immutable too. An omitted pending declaration
	// cannot manufacture missing bytes; an explicit exact approval can resume it.
	dest, e := codeUpgradeDestination(directory, p, retained)
	if e != nil {
		return fail()
	}
	if e = preflightCodeUpgradeDestination(dest, retained.Policy); e != nil {
		return fail()
	}
	if _, e = os.Lstat(dest); e == nil {
		old, e := readPlain(dest, int64(len(retained.Policy)))
		if e != nil || !bytes.Equal(old, retained.Policy) {
			return fail()
		}
	} else if !os.IsNotExist(e) || !explicit {
		return fail()
	}
	copies := []bootstrapCopy{}
	for _, a := range bootstrapArtifacts(p.Bootstrap) {
		dest := filepath.Join(directory, "public", filepath.FromSlash(a.Path))
		source := dest
		if _, e := os.Lstat(dest); os.IsNotExist(e) && explicit {
			source = filepath.Join(assets, path.Base(a.Path))
		}
		n, hash, e := hashFile(source)
		if e != nil || n != a.Bytes || hash != a.SHA256 {
			return fail()
		}
		copies = append(copies, bootstrapCopy{source: source, dest: dest, artifact: a})
	}
	return p, retained, copies, nil
}

func addCodeUpgradeTarget(targets *metadata.Metadata[metadata.TargetsType], p releasepolicy.BrokerCodeUpgradePolicy, s *brokerCodeUpgradeState) error {
	if s == nil {
		return nil
	}
	targetPath, e := releasepolicy.BrokerCodeUpgradeTargetPath(p.Bootstrap.Channel)
	if e != nil {
		return e
	}
	target, e := metadata.TargetFile().FromBytes(targetPath, s.Policy, "sha256")
	if e != nil {
		return e
	}
	identity := json.RawMessage(jsonBytes(releasepolicy.BrokerBootstrapIdentity{
		Purpose: p.Purpose, Channel: p.Bootstrap.Channel, Version: p.Bootstrap.Version,
		Commit: p.Bootstrap.Commit, TrustConfigSHA256: p.Bootstrap.TrustConfigSHA256,
	}))
	target.Custom = &identity
	targets.Signed.Targets[targetPath] = target
	return nil
}

func writeCodeUpgradeTarget(directory string, p releasepolicy.BrokerCodeUpgradePolicy, s *brokerCodeUpgradeState) error {
	if s == nil {
		return nil
	}
	dest, e := codeUpgradeDestination(directory, p, s)
	if e != nil {
		return e
	}
	if e = mkdir(filepath.Dir(dest)); e != nil {
		return e
	}
	return atomic(dest, s.Policy, false)
}
