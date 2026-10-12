package brokerhost

import (
	"context"
	"io"
	"os"
	"os/exec"
	"strings"
	"testing"
	"time"

	"example.com/opennav-update-verifier/brokerstore"
)

func TestNativeExecutionApplicationOwnerInertHelper(t *testing.T) {
	if os.Getenv("SKAGER_INERT_APPLICATION_OWNER_HELPER") != "1" {
		return
	}
	if _, err := io.Copy(io.Discard, os.Stdin); err != nil {
		os.Exit(2)
	}
	os.Exit(0)
}

// This fixture establishes native query custody only. It cannot authenticate an
// application generation or construct a production selected-start conversation.
func TestNativeExecutionApplicationOwnerSameHandlesSurviveConversationAndExit(t *testing.T) {
	exe, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	cmd := exec.CommandContext(ctx, exe, "-test.run=^TestNativeExecutionApplicationOwnerInertHelper$")
	cmd.Env = append(os.Environ(), "SKAGER_INERT_APPLICATION_OWNER_HELPER=1")
	input, err := cmd.StdinPipe()
	if err != nil {
		t.Fatal(err)
	}
	if err = cmd.Start(); err != nil {
		t.Fatal(err)
	}
	waited := false
	defer func() {
		input.Close()
		if !waited {
			cmd.Process.Kill()
			cmd.Wait()
		}
	}()
	process, err := openExecutionProcess(uint32(cmd.Process.Pid))
	if err != nil {
		t.Fatal("native interactive fixture original process/token", err)
	}
	defer process.close()
	originalHandle, originalToken := process.handle, process.token
	identity := brokerstore.GenerationApplicationIdentity{ClientSID: process.identity.sid, Generation: strings.Repeat("a", 32)}
	child := &nativeExecutionCurrentChild{host: &nativeExecutionCurrentHost{identity: identity}, process: process}
	owner, err := child.Retain(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	defer owner.Close()
	native := owner.(*nativeExecutionApplicationOwner)
	if native.process != process || native.process.handle != originalHandle || native.process.token != originalToken || child.process != nil {
		t.Fatal("original process/token replaced during transfer")
	}
	if err = child.Close(); err != nil {
		t.Fatal(err)
	}
	state, err := owner.observe(context.Background())
	if state != executionApplicationLive || err != nil {
		t.Fatal("conversation cleanup closed or signalled ordinary fixture", state, err)
	}
	e := &engine{}
	start, err := e.applications.reserve(context.Background(), owner.clientSID(), owner.generation())
	if err != nil {
		t.Fatal(err)
	}
	if err = start.retain(context.Background(), owner); err != nil {
		t.Fatal(err)
	}
	start.finish(true)
	// Closing the inert fixture's input models its own normal shutdown. The
	// service never terminates/signals the ordinary application.
	if err = input.Close(); err != nil {
		t.Fatal(err)
	}
	err = cmd.Wait()
	waited = true
	if err != nil {
		t.Fatal("normal inert helper exit", err)
	}
	for i := 0; i < 2; i++ {
		actual, state, err := e.observeApplicationOwnerLocked(context.Background(), owner.clientSID())
		if actual != owner || state != executionApplicationExited || err != nil || process.handle != originalHandle || process.token != originalToken {
			t.Fatal("original signalled exit proof lost", state, err)
		}
	}
	if err = e.releaseApplicationOwnerLocked(context.Background(), owner.clientSID()); err != nil {
		t.Fatal(err)
	}
	if process.handle != 0 || process.token != 0 {
		t.Fatal("original query handles leaked")
	}
	// Explicit release drains custody; a later observation must refuse unknown.
	if _, state, err := e.observeApplicationOwnerLocked(context.Background(), owner.clientSID()); state != executionApplicationUnknown || err == nil {
		t.Fatal("released original exit became general absence authority")
	}
	if state, err := owner.observe(context.Background()); state != executionApplicationUnknown || err == nil {
		t.Fatal("released native owner reused")
	}
}
