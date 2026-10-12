package brokerenroll

import (
	"context"
	"strconv"
	"strings"
)

// These capabilities belong only to the retained fresh file/service owners.
// They neither reopen an installed enrollment nor add caller-selected paths.
type clientRequestFiles interface {
	publishClientRequest(context.Context, []byte, string, string) error
	clientRegistration() (sid, approvedSHA256 string)
}

type clientRegistrationService interface {
	startClientRegistration(sid, approvedSHA256 string) error
}

func canonicalClientSID(value string) bool {
	parts := strings.Split(value, "-")
	if len(parts) != 8 || strings.Join(parts[:4], "-") != "S-1-5-21" {
		return false
	}
	for _, part := range parts[4:] {
		n, err := strconv.ParseUint(part, 10, 32)
		if err != nil || strconv.FormatUint(n, 10) != part {
			return false
		}
	}
	return true
}
