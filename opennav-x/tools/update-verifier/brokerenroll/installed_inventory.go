package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"strings"
	"unicode/utf8"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/baselineevidence"
	"example.com/opennav-update-verifier/brokerregistration"
)

const installedIntegrationOwner = "OpenNavX.Alpha1.SideBySide.1"
const installedIntegrationDirectory = "OpenNavXAlpha1"

type installedApprovedMember struct {
	sha  string
	size int64
}

type installedInventorySink struct {
	members map[string]installedApprovedMember
}

func (s *installedInventorySink) WriteFile(ctx context.Context, name string, source io.Reader, size int64, sha string) error {
	if !installedRelativePath(name) || !(strings.HasPrefix(name, "app/") || strings.HasPrefix(name, "docs/")) || name == "app/OPENNAV_PORTABLE_PREVIEW" || s.members[name].sha != "" {
		return errors.New("closed original installed payload required")
	}
	if err := (discardBaselineSink{}).WriteFile(ctx, name, source, size, sha); err != nil {
		return err
	}
	s.members[name] = installedApprovedMember{sha, size}
	return nil
}

func installedRelativePath(name string) bool {
	if len(name) == 0 || len(name) > 240 || strings.ContainsAny(name, "\\:\x00") {
		return false
	}
	for _, part := range strings.Split(name, "/") {
		if part == "" || part == "." || part == ".." || strings.HasSuffix(part, ".") || strings.HasSuffix(part, " ") {
			return false
		}
	}
	return true
}

func (s *installedInventorySink) WriteEvidence(ctx context.Context, name string, raw []byte, sha string) error {
	if ctx == nil || ctx.Err() != nil || digest(raw) != sha {
		return errors.New("original authenticated installed evidence required")
	}
	names, err := installedEvidenceTargets(name)
	if err != nil {
		return err
	}
	for _, target := range names {
		s.members[target] = installedApprovedMember{sha, int64(len(raw))}
	}
	return ctx.Err()
}

func installedEvidenceTargets(name string) ([]string, error) {
	switch {
	case name == "FILE_SHA256.json":
		return nil, nil // Never trust local copies of the original inventory.
	case name == "installer/package.json":
		return []string{"maintenance/package.json"}, nil
	case baselineevidence.IsEngineHelper(name):
		base := strings.TrimPrefix(name, "installer/engine/")
		targets := []string{base}
		if base != "Lifecycle.ps1" {
			targets = append(targets, "maintenance/"+base)
		}
		return targets, nil
	case baselineevidence.IsCommissioningHelper(name):
		// Commissioning originals authenticate broker evidence, not a Lifecycle
		// copy mapping. They authorize no additional installed deletion target.
		return nil, nil
	default:
		return nil, errors.New("unknown original installed evidence namespace")
	}
}

// Only the private completed-installation path calls this read-only inspection.
// It does not reset bootstrap expiry for fresh enrollment or authorize writes.
func collectRetainedInstalledInventory(ctx context.Context, authority maintenanceAuthority, source ApprovedClientBaseline) (map[string]installedApprovedMember, error) {
	if source == nil || source.Validate() != nil {
		return nil, errors.New("original completed baseline source required")
	}
	members, err := verifier.InspectRetainedApprovedBaseline(ctx, authority.client.OriginalBytes(), authority.client.RequestSHA256, source, source.Size())
	if err != nil {
		return nil, err
	}
	result := make(map[string]installedApprovedMember)
	for _, member := range members {
		var targets []string
		if strings.HasPrefix(member.Name, "app/") || strings.HasPrefix(member.Name, "docs/") {
			if !installedRelativePath(member.Name) || member.Name == "app/OPENNAV_PORTABLE_PREVIEW" {
				return nil, errors.New("closed retained payload required")
			}
			targets = []string{member.Name}
		} else {
			targets, err = installedEvidenceTargets(member.Name)
			if err != nil {
				return nil, err
			}
		}
		for _, name := range targets {
			if _, exists := result[name]; exists {
				return nil, errors.New("ambiguous retained installed target")
			}
			result[name] = installedApprovedMember{member.SHA256, member.Size}
		}
	}
	return result, source.Validate()
}

func collectInstalledInventory(ctx context.Context, authority maintenanceAuthority, source ApprovedClientBaseline) (map[string]installedApprovedMember, error) {
	if source == nil || source.Validate() != nil {
		return nil, errors.New("original native approved baseline source required")
	}
	sink := &installedInventorySink{members: map[string]installedApprovedMember{}}
	_, err := verifier.ImportApprovedBaseline(ctx, authority.client.OriginalBytes(), authority.client.RequestSHA256, source, source.Size(), sink)
	if err != nil {
		return nil, err
	}
	if err := source.Validate(); err != nil {
		return nil, err
	}
	return sink.members, nil
}

type installedState struct {
	Owner         string                                       `json:"owner"`
	Schema        int                                          `json:"schema"`
	Current       string                                       `json:"current"`
	Previous      string                                       `json:"previous"`
	Stock         struct{ Path, SHA256, Version, Arch string } `json:"stock"`
	ShortcutModes []string                                     `json:"shortcutModes,omitempty"`
}

// These mutable comparison records never supply removal hashes or paths. The
// only locator accepted is a fixed-length generation under the held fixed root.
func parseInstalledState(raw []byte, request brokerregistration.Request) (installedState, error) {
	var s installedState
	if installedExactObject(raw, []string{"owner", "schema", "current", "previous", "stock"}, []string{"shortcutModes"}) != nil {
		return s, errors.New("closed original installed state required")
	}
	var object map[string]json.RawMessage
	if json.Unmarshal(raw, &object) != nil || installedExactObject(object["stock"], []string{"path", "sha256", "version", "arch"}, nil) != nil || json.Unmarshal(raw, &s) != nil {
		return s, errors.New("exact installed stock/state comparison required")
	}
	if s.Owner != installedIntegrationOwner || s.Schema != 1 || !manualStartupRequestID(s.Current) || s.Previous != "" && (!manualStartupRequestID(s.Previous) || s.Previous == s.Current) ||
		!strings.EqualFold(s.Stock.Path, request.Stock.Path) || s.Stock.SHA256 != request.Stock.ExecutableSHA256 || s.Stock.Version != request.Stock.Version || s.Stock.Arch != request.Stock.Arch {
		return installedState{}, errors.New("installed state differs from original approved stock identity")
	}
	seen := map[string]bool{}
	for _, mode := range s.ShortcutModes {
		if mode != "xnav" && mode != "legacy" && mode != "safe" || seen[mode] {
			return installedState{}, errors.New("unknown installed shortcut modes")
		}
		seen[mode] = true
	}
	return s, nil
}

func installedExactObject(raw []byte, required, optional []string) error {
	if len(raw) == 0 || len(raw) > 32768 || !utf8.Valid(raw) {
		return errors.New("bounded UTF-8 installed comparison required")
	}
	d := json.NewDecoder(bytes.NewReader(raw))
	d.UseNumber()
	if err := installedJSONValue(d, 0); err != nil {
		return err
	}
	if _, err := d.Token(); err != io.EOF {
		return errors.New("trailing installed comparison data")
	}
	var fields map[string]json.RawMessage
	if json.Unmarshal(raw, &fields) != nil || fields == nil {
		return errors.New("installed comparison object required")
	}
	allowed := map[string]bool{}
	for _, name := range required {
		if _, ok := fields[name]; !ok || bytes.Equal(bytes.TrimSpace(fields[name]), []byte("null")) {
			return errors.New("missing installed comparison field")
		}
		allowed[name] = true
	}
	for _, name := range optional {
		allowed[name] = true
	}
	for name, value := range fields {
		if !allowed[name] || bytes.Equal(bytes.TrimSpace(value), []byte("null")) {
			return errors.New("unknown installed comparison field")
		}
	}
	return nil
}

func installedJSONValue(d *json.Decoder, depth int) error {
	if depth > 8 {
		return errors.New("installed comparison depth bound")
	}
	token, err := d.Token()
	if err != nil {
		return err
	}
	delim, ok := token.(json.Delim)
	if !ok {
		return nil
	}
	switch delim {
	case '{':
		seen := map[string]bool{}
		for d.More() {
			key, err := d.Token()
			if err != nil {
				return err
			}
			name, ok := key.(string)
			if !ok || seen[strings.ToLower(name)] {
				return errors.New("ambiguous installed comparison keys")
			}
			seen[strings.ToLower(name)] = true
			if err := installedJSONValue(d, depth+1); err != nil {
				return err
			}
		}
	case '[':
		for d.More() {
			if err := installedJSONValue(d, depth+1); err != nil {
				return err
			}
		}
	default:
		return errors.New("invalid installed comparison delimiter")
	}
	_, err = d.Token()
	return err
}
