//go:build !windows

package brokerstore

// Open never emulates distinct-principal protection on a portable host.
func Open(root string) (*Store, error) { return nil, ErrNativeRequired }
