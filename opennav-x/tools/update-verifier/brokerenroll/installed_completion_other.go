//go:build !windows

package brokerenroll

// There is deliberately no portable native publication capability. Ordinary
// startup already refuses off Windows; portable tests use private owner seams.
