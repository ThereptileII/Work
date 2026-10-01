package releasepolicy

import (
	"errors"
	"strings"
)

// SemVer is a strict SemVer 2.0 value. Core and numeric prerelease identifiers
// are compared as decimal strings, so they have no machine-integer ceiling.
type SemVer struct {
	raw  string
	core [3]string
	pre  []string
}

func ParseSemVer(s string) (SemVer, error) {
	var v SemVer
	if len(s) == 0 || len(s) > 128 {
		return v, errors.New("version length outside 1..128")
	}
	v.raw = s
	main, build, hasBuild := strings.Cut(s, "+")
	if hasBuild && !validIdentifiers(build, false) {
		return SemVer{}, errors.New("invalid build metadata")
	}
	core, pre, hasPre := strings.Cut(main, "-")
	if hasPre && !validIdentifiers(pre, true) {
		return SemVer{}, errors.New("invalid prerelease")
	}
	parts := strings.Split(core, ".")
	if len(parts) != 3 {
		return SemVer{}, errors.New("version must have three core numbers")
	}
	for i, p := range parts {
		if !numeric(p) || (len(p) > 1 && p[0] == '0') {
			return SemVer{}, errors.New("invalid core number")
		}
		v.core[i] = p
	}
	if hasPre {
		v.pre = strings.Split(pre, ".")
	}
	return v, nil
}
func numeric(s string) bool {
	if s == "" {
		return false
	}
	for i := 0; i < len(s); i++ {
		if s[i] < '0' || s[i] > '9' {
			return false
		}
	}
	return true
}
func validIdentifiers(s string, prerelease bool) bool {
	if s == "" {
		return false
	}
	for _, part := range strings.Split(s, ".") {
		if part == "" || (prerelease && numeric(part) && len(part) > 1 && part[0] == '0') {
			return false
		}
		for i := 0; i < len(part); i++ {
			c := part[i]
			if !(c >= '0' && c <= '9' || c >= 'A' && c <= 'Z' || c >= 'a' && c <= 'z' || c == '-') {
				return false
			}
		}
	}
	return true
}
func decimalCompare(a, b string) int {
	if len(a) < len(b) {
		return -1
	}
	if len(a) > len(b) {
		return 1
	}
	return strings.Compare(a, b)
}

// Compare returns -1, 0, or 1 by SemVer precedence. Build metadata is ignored.
func (v SemVer) Compare(other SemVer) int {
	for i := 0; i < 3; i++ {
		if n := decimalCompare(v.core[i], other.core[i]); n != 0 {
			return n
		}
	}
	if len(v.pre) == 0 && len(other.pre) > 0 {
		return 1
	}
	if len(v.pre) > 0 && len(other.pre) == 0 {
		return -1
	}
	for i := 0; i < len(v.pre) && i < len(other.pre); i++ {
		a, b := v.pre[i], other.pre[i]
		an, bn := numeric(a), numeric(b)
		if an && !bn {
			return -1
		}
		if !an && bn {
			return 1
		}
		n := strings.Compare(a, b)
		if an {
			n = decimalCompare(a, b)
		}
		if n != 0 {
			return n
		}
	}
	if len(v.pre) < len(other.pre) {
		return -1
	}
	if len(v.pre) > len(other.pre) {
		return 1
	}
	return 0
}
func (v SemVer) IsPrerelease() bool { return len(v.pre) > 0 }
