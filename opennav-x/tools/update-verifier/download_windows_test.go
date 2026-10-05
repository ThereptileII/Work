package verifier

import (
	"os"
	"path/filepath"
	"testing"
)

// Native Windows gate: cross-compilation alone does not exercise sharing.
func TestDownloadedArtifactWindowsDeniesWriteAndDeleteSharing(t *testing.T) {
	path := filepath.Join(t.TempDir(), "artifact.exe")
	if err := os.WriteFile(path, []byte("verified bytes"), 0600); err != nil {
		t.Fatal(err)
	}
	handle, err := openDownloadedArtifact(path)
	if err != nil {
		t.Fatal(err)
	}
	defer handle.Close()
	if writer, err := os.OpenFile(path, os.O_WRONLY, 0); err == nil {
		writer.Close()
		t.Fatal("custody allowed a writer")
	}
	if err := os.Remove(path); err == nil {
		t.Fatal("custody allowed deletion")
	}
	if err := os.Rename(path, path+".replaced"); err == nil {
		t.Fatal("custody allowed rename")
	}
	reader, err := os.Open(path)
	if err != nil {
		t.Fatalf("custody prohibited another reader: %v", err)
	}
	reader.Close()
}
