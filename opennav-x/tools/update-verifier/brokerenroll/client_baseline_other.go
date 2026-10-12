//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
)

func openClientBaselineInputNative(context.Context, string) (ApprovedClientBaseline, error) {
	return nil, errors.New("protected operator baseline custody requires native Windows")
}

// OpenApprovedClientBaselineNative refuses outside native Windows; no caller
// path, operator-console fallback or unprotected baseline import is supported.
func OpenApprovedClientBaselineNative(context.Context, string) (ApprovedClientBaseline, error) {
	return nil, errors.New("protected broker baseline custody requires native Windows")
}
