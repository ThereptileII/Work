package verifier

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"unicode/utf8"
)

// The three bounded identity strings fit even with JSON character escapes.
// This limit is independent of the larger TUF targets-metadata transport limit.
const maxReleaseIdentityBytes = 2 << 10

// parseReleaseIdentity consumes the authenticated custom bytes without a struct
// decoder's case folding or last-key-wins behavior. Values must be strings, so
// nested objects/arrays are rejected at their first delimiter without recursion.
func parseReleaseIdentity(data []byte) (Release, error) {
	var release Release
	if len(data) == 0 || len(data) > maxReleaseIdentityBytes || !utf8.Valid(data) {
		return Release{}, errors.New("invalid custom identity byte length or UTF-8")
	}
	dec := json.NewDecoder(bytes.NewReader(data))
	dec.UseNumber()
	tok, err := dec.Token()
	if err != nil || tok != json.Delim('{') {
		return Release{}, errors.New("custom identity must be an object")
	}
	seen := make(map[string]bool, 3)
	for dec.More() {
		tok, err := dec.Token()
		if err != nil {
			return Release{}, err
		}
		key, ok := tok.(string)
		if !ok {
			return Release{}, errors.New("invalid custom identity key")
		}
		var field *string
		switch key {
		case "channel":
			field = &release.Channel
		case "version":
			field = &release.Version
		case "commit":
			field = &release.Commit
		default:
			return Release{}, fmt.Errorf("unknown custom identity field %q", key)
		}
		if seen[key] {
			return Release{}, fmt.Errorf("duplicate custom identity field %q", key)
		}
		seen[key] = true
		tok, err = dec.Token()
		if err != nil {
			return Release{}, err
		}
		value, ok := tok.(string)
		if !ok {
			return Release{}, fmt.Errorf("custom identity field %q must be a string", key)
		}
		*field = value
	}
	if tok, err = dec.Token(); err != nil || tok != json.Delim('}') {
		return Release{}, errors.New("unterminated custom identity object")
	}
	if _, err = dec.Token(); err != io.EOF {
		return Release{}, errors.New("trailing custom identity data")
	}
	if len(seen) != 3 {
		return Release{}, errors.New("custom identity requires channel, version and commit")
	}
	return release, nil
}
