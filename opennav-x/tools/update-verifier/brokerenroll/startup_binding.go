package brokerenroll

import (
	"bytes"
	"encoding/json"
	"errors"
	"unicode/utf8"

	"example.com/opennav-update-verifier/brokerregistration"
)

const startupBindingPurpose = "original-shortcut-publication"
const maxStartupBindingBytes = 32 << 10

type startupDirectoryBinding struct {
	Volume           uint64   `json:"volume"`
	FileID           [16]byte `json:"fileId"`
	Creation         uint64   `json:"creation"`
	DescriptorSHA256 string   `json:"descriptorSha256"`
}

type startupFileBinding struct {
	startupDirectoryBinding
	Bytes  int64  `json:"bytes"`
	SHA256 string `json:"sha256"`
}

type startupLinkBinding struct {
	Name     string              `json:"name"`
	New      startupFileBinding  `json:"new"`
	Original *startupFileBinding `json:"original"`
}

type startupBindingRecord struct {
	Schema               int                     `json:"schema"`
	Purpose              string                  `json:"purpose"`
	ClientSID            string                  `json:"clientSid"`
	ClientRequestSHA256  string                  `json:"clientRequestSha256"`
	Generation           string                  `json:"generation"`
	Commit               string                  `json:"commit"`
	PackageSHA256        string                  `json:"packageSha256"`
	PrepublicationSHA256 string                  `json:"prepublicationSha256"`
	Menu                 startupDirectoryBinding `json:"menu"`
	Recovery             startupDirectoryBinding `json:"recovery"`
	Launcher             startupFileBinding      `json:"launcher"`
	Links                [3]startupLinkBinding   `json:"links"`
}

// ApprovedStartupPublication owns canonical original publication bytes matched
// to TWO separately approved inputs: this record and the original client request.
// It proves no current filesystem custody, cold application/profile state,
// broker witness, publisher identity, service completion or mutation authority.
// Zero values/copies/serialized records cannot grant restoration permission.
type ApprovedStartupPublication struct {
	raw    []byte
	sha256 string
}

func (p *ApprovedStartupPublication) OriginalBytes() []byte {
	if p == nil {
		return nil
	}
	return bytes.Clone(p.raw)
}

func (p *ApprovedStartupPublication) SHA256() string {
	if p == nil {
		return ""
	}
	return p.sha256
}

// ParseApprovedStartupPublication is an identity-only parser, not a native
// restoration entrypoint. approvedPublicationSHA256 must come from independent
// approval of the ORIGINAL completed publication, never an adjacent self-hash.
// The retained-request parser preserves identity after root rotation; this does
// not exempt fresh enrollment or current metadata from expiry/floor validation.
func ParseApprovedStartupPublication(raw []byte, approvedPublicationSHA256 string, originalRequest []byte, approvedRequestSHA256 string) (*ApprovedStartupPublication, error) {
	refuse := func() (*ApprovedStartupPublication, error) {
		return nil, errors.New("independently approved original startup publication required")
	}
	if len(raw) == 0 || len(raw) > maxStartupBindingBytes || !utf8.Valid(raw) || !validPin(approvedPublicationSHA256) || digest(raw) != approvedPublicationSHA256 {
		return refuse()
	}
	request, err := brokerregistration.ParseRetainedApprovedRequest(originalRequest, approvedRequestSHA256)
	if err != nil {
		return refuse()
	}
	var record startupBindingRecord
	d := json.NewDecoder(bytes.NewReader(raw))
	d.DisallowUnknownFields()
	if d.Decode(&record) != nil {
		return refuse()
	}
	// Exact canonical encoding rejects duplicate keys, extra array elements,
	// null/default substitutions, trailing documents and alternate spellings.
	canonical, err := json.Marshal(record)
	if err != nil || !bytes.Equal(canonical, raw) || validateStartupBinding(record, request) != nil {
		return refuse()
	}
	return &ApprovedStartupPublication{raw: bytes.Clone(raw), sha256: approvedPublicationSHA256}, nil
}

func validStartupDirectory(d startupDirectoryBinding) bool {
	return d.Volume != 0 && d.FileID != [16]byte{} && d.Creation != 0 && validPin(d.DescriptorSHA256)
}

func validStartupFile(f startupFileBinding, max int64) bool {
	return validStartupDirectory(f.startupDirectoryBinding) && f.Bytes > 0 && f.Bytes <= max && validPin(f.SHA256)
}

func validateStartupBinding(r startupBindingRecord, request brokerregistration.Request) error {
	if r.Schema != 1 || r.Purpose != startupBindingPurpose || r.ClientSID != request.ClientSID ||
		r.ClientRequestSHA256 != request.RequestSHA256 || r.Generation != request.Installed.Generation ||
		r.Commit != request.Installed.Commit || r.PackageSHA256 != request.Installed.PackageSHA256 ||
		!validPin(r.PrepublicationSHA256) || !validStartupDirectory(r.Menu) || !validStartupDirectory(r.Recovery) ||
		r.Menu.Volume != r.Recovery.Volume || r.Menu.FileID == r.Recovery.FileID || !validStartupFile(r.Launcher, maxBrokerBytes) {
		return errors.New("original startup publication identity differs")
	}
	// No two retained objects may claim the same native identity, even with the
	// same contents. Directory and file identities share the same NTFS namespace.
	type identity struct {
		volume uint64
		id     [16]byte
	}
	seen := map[identity]bool{}
	add := func(d startupDirectoryBinding) bool {
		key := identity{d.Volume, d.FileID}
		if seen[key] {
			return false
		}
		seen[key] = true
		return true
	}
	if !add(r.Menu) || !add(r.Recovery) || !add(r.Launcher.startupDirectoryBinding) {
		return errors.New("distinct original startup objects required")
	}
	for i, link := range r.Links {
		spec, err := startupShortcutMode(i).spec()
		if err != nil || link.Name != spec.name || !validStartupFile(link.New, 1<<20) || link.New.Volume != r.Menu.Volume || !add(link.New.startupDirectoryBinding) {
			return errors.New("all three distinct fixed published shortcuts required")
		}
		if link.Original != nil && (!validStartupFile(*link.Original, 1<<20) || link.Original.Volume != r.Recovery.Volume || !add(link.Original.startupDirectoryBinding)) {
			return errors.New("distinct bounded original shortcut backups required")
		}
	}
	return nil
}
