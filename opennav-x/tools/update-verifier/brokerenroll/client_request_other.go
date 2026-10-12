//go:build !windows

package brokerenroll

import (
	"context"
	"errors"
)

func validateClientSIDNative(string) error {
	return errors.New("actual client account lookup requires native Windows")
}

// ReadApprovedClientRequestNative requires the real broker principal on Windows
// and a separately approved exact request hash; other platforms always refuse.
func ReadApprovedClientRequestNative(context.Context, string, string) ([]byte, error) {
	return nil, errors.New("protected broker client request read requires native Windows")
}
