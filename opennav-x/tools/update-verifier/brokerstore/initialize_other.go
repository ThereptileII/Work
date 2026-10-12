//go:build !windows

package brokerstore

// InitializeFresh never emulates the protected machine store on another OS.
func InitializeFresh() error { return ErrNativeRequired }

// ValidateInitialization requires native Windows creation-completion evidence.
func ValidateInitialization() error { return ErrNativeRequired }
