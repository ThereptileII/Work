//go:build !windows

package launcher

// The installed launcher exists only on Windows; elsewhere SKAGER is never linked.
func readCloudToken() string { return "" }
