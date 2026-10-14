package brokerenroll

import "strings"

// Bound only by the original native enrollment producer. This is directory
// custody, not permission to replace, initialize or migrate rollback floors.
type installedBootstrapCacheBinding struct {
	Path      string                  `json:"path"`
	Directory startupDirectoryBinding `json:"directory"`
}

func validInstalledBootstrapCache(b *installedBootstrapCacheBinding) bool {
	if b == nil || !validStartupDirectory(b.Directory) || len(b.Path) < 4 || b.Path[1:3] != `:\` ||
		!((b.Path[0] >= 'A' && b.Path[0] <= 'Z') || (b.Path[0] >= 'a' && b.Path[0] <= 'z')) || strings.ContainsAny(b.Path[3:], ":/\x00") {
		return false
	}
	for _, part := range strings.Split(b.Path[3:], `\`) {
		if part == "" || part == "." || part == ".." || strings.HasSuffix(part, ".") || strings.HasSuffix(part, " ") {
			return false
		}
	}
	return true
}
