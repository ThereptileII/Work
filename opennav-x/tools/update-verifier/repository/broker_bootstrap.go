package repository

import (
	"errors"
	"os"
	"path"
	"path/filepath"
	"strings"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
)

// Raw bytes are base64-encoded by JSON so state persistence cannot reformat the
// exact independently approved policy. Presence also retains its version floor.
type brokerBootstrapState struct {
	Policy []byte `json:"policy"`
	SHA256 string `json:"sha256"`
}

type bootstrapCopy struct {
	source, dest string
	artifact     releasepolicy.Artifact
}

// PublishWithBrokerBootstrap adds an explicitly operator-approved declaration to
// the selected application publication. Its hash must be obtained independently
// of the supplied policy. Neither application qualification nor a signed target
// authenticates the initial privileged verifier or grants execution authority.
// Ordinary Publish calls retain this target and its independent product floor.
func PublishWithBrokerBootstrap(directory, assets string, selection, policy []byte, selectionSHA string, bootstrap []byte, bootstrapSHA string, keys Keys, now time.Time) (int64, error) {
	if len(bootstrap) == 0 || bootstrapSHA == "" {
		return 0, errors.New("explicit bootstrap policy and approval pin required")
	}
	return publishBatch(directory, assets, selection, policy, selectionSHA, bootstrap, bootstrapSHA, keys, now, nil)
}

func bootstrapArtifacts(p releasepolicy.BrokerBootstrapPolicy) []releasepolicy.Artifact {
	return []releasepolicy.Artifact{p.Broker, p.Provisioner, p.CorrespondingSource, p.Notices}
}

func retainedBootstrap(s *brokerBootstrapState, trust Trust, configSHA string) (releasepolicy.BrokerBootstrapPolicy, error) {
	var empty releasepolicy.BrokerBootstrapPolicy
	if s == nil || !hashRE.MatchString(s.SHA256) || checksum(s.Policy) != s.SHA256 {
		return empty, errors.New("retained bootstrap approval identity refused")
	}
	p, e := releasepolicy.ParseBrokerBootstrap(s.Policy)
	if e != nil || p.Channel != trust.Channel || p.TrustConfigSHA256 != configSHA {
		return empty, errors.New("bootstrap configuration identity refused")
	}
	for _, a := range bootstrapArtifacts(p) {
		if a.URL != trust.ArtifactOrigin+"/"+a.Path {
			return empty, errors.New("bootstrap artifact origin refused")
		}
	}
	return p, nil
}

func selectBootstrap(directory, assets string, raw []byte, approvedSHA string, s state, trust Trust, app releasepolicy.Policy) (releasepolicy.BrokerBootstrapPolicy, *brokerBootstrapState, []bootstrapCopy, error) {
	var p releasepolicy.BrokerBootstrapPolicy
	fail := func() (releasepolicy.BrokerBootstrapPolicy, *brokerBootstrapState, []bootstrapCopy, error) {
		return releasepolicy.BrokerBootstrapPolicy{}, nil, nil, errors.New("broker bootstrap selection refused; preserve operator state")
	}
	explicit := raw != nil || approvedSHA != ""
	retained := s.BrokerBootstrap
	if explicit {
		if len(raw) == 0 || !hashRE.MatchString(approvedSHA) || checksum(raw) != approvedSHA {
			return fail()
		}
		next := &brokerBootstrapState{Policy: append([]byte(nil), raw...), SHA256: approvedSHA}
		var e error
		p, e = retainedBootstrap(next, trust, s.ConfigSHA256)
		if e != nil || p.Version != app.Version || p.Commit != app.Commit {
			return fail()
		}
		if retained != nil {
			old, e := retainedBootstrap(retained, trust, s.ConfigSHA256)
			if e != nil {
				return fail()
			}
			previous, _ := releasepolicy.ParseSemVer(old.Version)
			version, _ := releasepolicy.ParseSemVer(p.Version)
			order := version.Compare(previous)
			if order < 0 || (order == 0 && next.SHA256 != retained.SHA256) {
				return fail()
			}
		}
		retained = next
	} else if retained != nil {
		var e error
		p, e = retainedBootstrap(retained, trust, s.ConfigSHA256)
		if e != nil {
			return fail()
		}
	} else {
		return p, nil, nil, nil
	}
	// Check the union, including Windows case aliases, before reserving state.
	// Shared source/notices are allowed only when their entire identity agrees.
	seen := map[string]releasepolicy.Artifact{}
	for _, a := range append(artifacts(app), bootstrapArtifacts(p)...) {
		folded := strings.ToLower(a.Path)
		if old, ok := seen[folded]; ok && old != a {
			return fail()
		}
		seen[folded] = a
		if e := preflightBootstrapDestination(filepath.Join(directory, "public", filepath.FromSlash(a.Path)), a); e != nil {
			return fail()
		}
	}
	copies := []bootstrapCopy{}
	for _, a := range bootstrapArtifacts(p) {
		dest := filepath.Join(directory, "public", filepath.FromSlash(a.Path))
		source := dest
		if explicit {
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

// Deterministic immutable collisions are invalid selections, not recoverable
// publication faults. Reject them before consuming a new authority/version floor.
func preflightBootstrapDestination(dest string, a releasepolicy.Artifact) error {
	parent := filepath.Dir(dest)
	entries, e := os.ReadDir(parent)
	if os.IsNotExist(e) {
		return nil
	}
	if e != nil {
		return e
	}
	if e = plain(parent); e != nil {
		return e
	}
	for _, entry := range entries {
		if strings.EqualFold(entry.Name(), filepath.Base(dest)) && entry.Name() != filepath.Base(dest) {
			return errors.New("published artifact aliases a Windows destination")
		}
	}
	if _, e = os.Lstat(dest); os.IsNotExist(e) {
		return nil
	} else if e != nil {
		return e
	}
	n, hash, e := hashFile(dest)
	if e != nil || n != a.Bytes || hash != a.SHA256 {
		return errors.New("published artifact identity cannot be replaced")
	}
	return nil
}
