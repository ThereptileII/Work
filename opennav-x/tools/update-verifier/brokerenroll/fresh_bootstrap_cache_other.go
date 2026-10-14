//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
)

func freshBootstrapCacheNativeOperations() freshBootstrapCacheOperations {
	return freshBootstrapCacheOperations{source: func(context.Context, string) (maintenanceImageSource, error) {
		return nil, errors.New("native Windows fresh cache preparation required")
	}, open: func(context.Context, string) (freshBootstrapCacheOwner, error) {
		return nil, errors.New("native Windows fresh cache preparation required")
	}}
}
