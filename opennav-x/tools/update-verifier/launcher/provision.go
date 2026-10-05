package launcher

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"os"

	verifier "example.com/opennav-update-verifier"
)

type provisioner interface {
	createMarker(string, []byte) error
	privateDirectory(string, bool) error
}

// Initialization is an explicit installer command, never called by ordinary
// startup. The once marker is outside the TUF state directory and precedes every
// first initialization attempt. Interrupted/removed state requires independent
// recovery; rerunning this command cannot establish a new rollback floor.
func initializeTrust(p platform, g *generation) error {
	data, err := p.read(g.layout.app+"/update-trust.json", maxTrustBytes)
	if errors.Is(err, os.ErrNotExist) {
		return nil
	}
	if err != nil {
		return errors.New("installed trust configuration unavailable")
	}
	if err := g.hold(p, "app/update-trust.json"); err != nil {
		return err
	}
	data, err = p.read(g.layout.app+"/update-trust.json", maxTrustBytes)
	if err != nil {
		return errors.New("installed trust configuration unavailable")
	}
	t, err := parseTrust(data)
	if err != nil {
		return err
	}
	config := preparationConfig(g.layout, g.owned, t)
	return provisionTrust(p, g.layout.root, config, data, verifier.InitializeState, verifier.ValidateState)
}
func provisionTrust(p platform, root string, c verifier.PrepareConfig, data []byte, initialize, validate func(string, verifier.TrustConfig) error) error {
	fs, ok := p.(provisioner)
	if !ok {
		return errors.New("native trust provisioning unavailable")
	}
	hash := sha256.Sum256(data)
	marker, _ := json.Marshal(struct {
		Schema       int    `json:"schema"`
		Channel      string `json:"channel"`
		ConfigSHA256 string `json:"configSha256"`
	}{1, c.Trust.Channel, hex.EncodeToString(hash[:])})
	name := root + "/update-trust-initialized-" + c.Trust.Channel
	previous, err := p.read(name, 4096)
	if err == nil {
		if !bytes.Equal(previous, marker) {
			return errors.New("installed trust changed; explicit migration required")
		}
		if err := fs.privateDirectory(root+"/update-trust", false); err != nil {
			return err
		}
		if err := fs.privateDirectory(c.ArtifactDirectory, false); err != nil {
			return err
		}
		if err := validate(c.StateDirectory, c.Trust); err != nil {
			return errors.New("retained update state unavailable; explicit recovery required")
		}
		return nil
	}
	if !errors.Is(err, os.ErrNotExist) {
		return errors.New("trust once marker unavailable; explicit recovery required")
	}
	if err := fs.createMarker(name, marker); err != nil {
		return errors.New("trust initialization already attempted or unavailable")
	}
	if err := fs.privateDirectory(root+"/update-trust", true); err != nil {
		return err
	}
	if err := fs.privateDirectory(c.ArtifactDirectory, true); err != nil {
		return err
	}
	if err := initialize(c.StateDirectory, c.Trust); err != nil {
		return errors.New("trust initialization incomplete; explicit recovery required")
	}
	return nil
}
