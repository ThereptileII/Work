package launcher

import (
	"context"
	"io"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"golang.org/x/sys/windows"
)

// Runs only as an explicitly spawned inert test child. No application,
// installer, shell, network, navigation data or device is touched.
func TestNativeInertChild(t *testing.T) {
	if os.Getenv("SKAGER_LAUNCHER_INERT_CHILD") != "1" {
		return
	}
	typeOf, err := windows.GetFileType(windows.Handle(os.Stdin.Fd()))
	if err != nil || typeOf != windows.FILE_TYPE_PIPE {
		os.Exit(2)
	}
	data, err := io.ReadAll(io.LimitReader(os.Stdin, 257))
	if err != nil || string(data) != "digest\nversion\ncommit\n" || os.Getenv("SKAGER_UPDATE_CHALLENGE") != "" {
		os.Exit(3)
	}
	os.Exit(10)
}
func TestNativeInertProcessHandoffWithRetainedReadHandle(t *testing.T) {
	t.Setenv("SKAGER_LAUNCHER_INERT_CHILD", "1")
	t.Setenv("SKAGER_UPDATE_CHALLENGE", "must-not-be-inherited")
	executable, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	executable, err = localPath(executable)
	if err != nil {
		t.Fatal(err)
	}
	file, err := openNativeRead(executable)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	result, err := (nativePlatform{}).wait(context.Background(), processSpec{executable: executable, directory: strings.ReplaceAll(filepath.Dir(executable), "\\", "/"), arguments: []string{"-test.run=^TestNativeInertChild$"}, input: []byte("digest\nversion\ncommit\n"), limit: 10 * time.Second})
	if err != nil || result.timedOut || result.code != 10 {
		t.Fatalf("inert pipe/exit/image-sharing handoff: %+v %v", result, err)
	}
}
func TestNativeProvisionedDirectoryAndOwnedFileCustody(t *testing.T) {
	root := strings.ReplaceAll(t.TempDir(), "\\", "/")
	directory := root + "/private"
	p := nativePlatform{}
	if err := p.privateDirectory(directory, true); err != nil {
		t.Fatal(err)
	}
	if err := p.privateDirectory(directory, false); err != nil {
		t.Fatal(err)
	}
	name := directory + "/once"
	data := []byte("inert once marker")
	if err := p.createMarker(name, data); err != nil {
		t.Fatal(err)
	}
	if err := p.createMarker(name, data); err == nil {
		t.Fatal("once marker was replaced")
	}
	held, err := p.hold(name, digest(data))
	if err != nil {
		t.Fatal(err)
	}
	defer held.Close()
	if err := os.WriteFile(name, []byte("replace"), 0600); err == nil {
		t.Fatal("retained custody allowed modification")
	}
	if err := os.Remove(name); err == nil {
		t.Fatal("retained custody allowed deletion")
	}
}

func TestNativeVerifierPathsAreCanonical(t *testing.T) {
	l, err := deriveLayout("C:/Users/Owner/AppData/Local/OpenNavXAlpha1/generations/" + fixtureGeneration + "/app/skager-start.exe")
	if err != nil {
		t.Fatal(err)
	}
	c := preparationConfig(l, ownership{}, trustRecord{Channel: "beta"})
	for _, name := range []string{c.StateDirectory, c.ArtifactDirectory} {
		if name != filepath.Clean(name) || !filepath.IsAbs(name) || strings.Contains(name, "/") {
			t.Fatalf("verifier path is not canonical native absolute: %q", name)
		}
	}
}

func TestNativeStartupLockRefusesConcurrentMaintenance(t *testing.T) {
	p := nativePlatform{}
	name := strings.ReplaceAll(t.TempDir(), "\\", "/") + "/transaction.lock"
	first, err := p.lockStartup(name)
	if err != nil {
		t.Fatal(err)
	}
	defer first.Close()
	started := time.Now()
	second, err := p.lockStartup(name)
	if err == nil {
		second.Close()
		t.Fatal("concurrent installer/startup lock accepted")
	}
	if time.Since(started) > time.Second {
		t.Fatal("lock refusal was not bounded")
	}
	first.Close()
	third, err := p.lockStartup(name)
	if err != nil {
		t.Fatalf("released startup lock remained busy: %v", err)
	}
	third.Close()
}
