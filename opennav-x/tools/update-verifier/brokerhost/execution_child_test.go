package brokerhost

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"io"
	"strings"
	"sync"
	"testing"
	"time"

	"example.com/opennav-update-verifier/brokerpipe"
	"example.com/opennav-update-verifier/brokerstore"
)

type memoryExecutionProfile struct {
	err, beforeErr, closeErr error
	closed                   bool
	beforeCalls              int
}

func (b *memoryExecutionProfile) validate(ctx context.Context) error {
	if b.closed {
		return ErrExecutionChildRefused
	}
	return errors.Join(ctx.Err(), b.err)
}
func (b *memoryExecutionProfile) beforeLaunch(ctx context.Context, _ executionProfileOriginal) error {
	b.beforeCalls++
	return errors.Join(ctx.Err(), b.beforeErr)
}
func (b *memoryExecutionProfile) close() error { b.closed = true; return b.closeErr }
func profileFixture() (*ExecutionColdProfile, *memoryExecutionProfile) {
	raw := []byte("[Settings]\nConnection=COM8\nUnknownPreserved=1\n")
	sum := sha256.Sum256(raw)
	b := &memoryExecutionProfile{}
	original := executionProfileOriginal{directory: executionProfileID{volume: 1, file: [16]byte{2}, created: 3}, file: executionProfileID{volume: 1, file: [16]byte{4}, created: 5}, digest: hex.EncodeToString(sum[:]), raw: raw}
	return &ExecutionColdProfile{state: &executionColdProfileState{original: original, backend: b}}, b
}

type memoryExecutionAuthority struct {
	peer  brokerpipe.PeerIdentity
	err   error
	calls int
}

func (a *memoryExecutionAuthority) withExecutor(ctx context.Context, use func(brokerpipe.PeerIdentity) error) error {
	a.calls++
	if err := errors.Join(ctx.Err(), a.err); err != nil {
		return err
	}
	return use(a.peer)
}

type memoryExecutionChild struct {
	zeroRead                            bool
	bytes                               []byte
	err, authErr, validateErr, closeErr error
	closed                              bool
	readChunk                           int
	block                               chan struct{}
	entered                             chan struct{}
	enteredOnce                         sync.Once
	readyAfter                          time.Duration
	onRead                              func()
}

func (b *memoryExecutionChild) check(ctx context.Context) error {
	if b.closed {
		return ErrExecutionChildRefused
	}
	return ctx.Err()
}
func (b *memoryExecutionChild) connect(ctx context.Context) error {
	return errors.Join(b.check(ctx), b.err)
}
func (b *memoryExecutionChild) authenticate(ctx context.Context) (time.Duration, error) {
	return b.readyAfter, errors.Join(b.check(ctx), b.authErr)
}
func (b *memoryExecutionChild) read(ctx context.Context, p []byte) (int, error) {
	if b.entered != nil {
		b.enteredOnce.Do(func() { close(b.entered) })
	}
	if b.block != nil {
		select {
		case <-ctx.Done():
			return 0, ctx.Err()
		case <-b.block:
		}
	}
	if b.onRead != nil {
		b.onRead()
	}
	if len(b.bytes) == 0 && b.zeroRead {
		return 0, nil
	}
	if len(b.bytes) == 0 {
		return 0, io.EOF
	}
	n := len(p)
	if b.readChunk > 0 && n > b.readChunk {
		n = b.readChunk
	}
	if n > len(b.bytes) {
		n = len(b.bytes)
	}
	copy(p, b.bytes[:n])
	b.bytes = b.bytes[n:]
	return n, nil
}
func (b *memoryExecutionChild) validate(ctx context.Context) error {
	return errors.Join(b.check(ctx), b.validateErr)
}
func (b *memoryExecutionChild) close() error { b.closed = true; return b.closeErr }
func executionFixture(t *testing.T) (*ExecutionChildObserver, *memoryExecutionChild, *memoryExecutionAuthority, *ExecutionColdProfile, string, string) {
	t.Helper()
	profile, _ := profileFixture()
	configuration, err := profile.configuration(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	identity := brokerstore.GenerationApplicationIdentity{ClientSID: "S-1-5-21-1-2-3-500", Generation: strings.Repeat("a", 32), Commit: strings.Repeat("b", 40), ExecutableSHA256: strings.Repeat("c", 64)}
	peer := brokerpipe.PeerIdentity{Peer: brokerpipe.Peer{SID: identity.ClientSID, SessionID: 1, ProcessID: 7}, ProcessCreation: 123, LogonIDHigh: 2, LogonIDLow: 3}
	authority := &memoryExecutionAuthority{peer: peer}
	backend := &memoryExecutionChild{readyAfter: 30 * time.Second}
	prepared := time.Now()
	env := map[string]string{"SKAGER_UPDATE_PIPE": "Skager.Update." + strings.Repeat("d", 32), "SKAGER_UPDATE_GENERATION": identity.Generation, "SKAGER_UPDATE_CHALLENGE": strings.Repeat("e", 64), "SKAGER_UPDATE_QUALIFY_PROFILE": "1"}
	o := newExecutionChildObserver(context.Background(), authority, peer, identity, backend, profile, prepared, env)
	o.state.now = func() time.Time { return prepared.Add(31 * time.Second) }
	t.Cleanup(func() { _ = o.Close(); _ = profile.Close() })
	tail := identity.Generation + " " + identity.Commit + " " + env["SKAGER_UPDATE_CHALLENGE"] + "\n"
	config := "SKAGER-UPDATE-CONFIG/1 " + strings.TrimSuffix(tail, "\n") + " " + configuration + "\n"
	return o, backend, authority, profile, tail, config
}
func phaseFrame(phase, tail string) string { return "SKAGER-UPDATE-" + phase + "/1 " + tail }
func TestExecutionChildQualifiedProtocol(t *testing.T) {
	for _, tc := range []struct {
		name, phases string
		cancel       bool
	}{{"direct", "READY", false}, {"warning", "WAIT,CONTINUE,READY", false}, {"cancel", "WAIT,CANCEL", true}} {
		t.Run(tc.name, func(t *testing.T) {
			o, b, _, _, tail, config := executionFixture(t)
			b.readChunk = 1
			b.bytes = []byte(config)
			for _, phase := range strings.Split(tc.phases, ",") {
				b.bytes = append(b.bytes, phaseFrame(phase, tail)...)
			}
			observed, err := o.Observe(context.Background())
			if tc.cancel {
				if !errors.Is(err, ErrExecutionChildCancelled) || observed != nil {
					t.Fatalf("cancel result %v %v", observed, err)
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			if err = observed.Validate(context.Background()); err != nil {
				t.Fatal(err)
			}
			if _, err = o.Observe(context.Background()); err == nil {
				t.Fatal("reused attempt")
			}
			if _, err = o.Environment(context.Background()); err == nil {
				t.Fatal("environment after consumption")
			}
		})
	}
}
func TestExecutionChildRejectsUnqualifiedOrMalformedEvidence(t *testing.T) {
	cases := []struct {
		name string
		wire func(string, string) string
	}{
		{"legacy-ready", func(t, c string) string { return phaseFrame("READY", t) }},
		{"legacy-wait", func(t, c string) string { return phaseFrame("WAIT", t) + c + phaseFrame("READY", t) }},
		{"missing-ready", func(t, c string) string { return c }},
		{"partial-eof", func(t, c string) string { return c + strings.TrimSuffix(phaseFrame("READY", t), "\n") }},
		{"trailing", func(t, c string) string { return c + phaseFrame("READY", t) + "x" }},
		{"duplicate-config", func(t, c string) string { return c + c + phaseFrame("READY", t) }},
		{"duplicate-wait", func(t, c string) string {
			return c + phaseFrame("WAIT", t) + phaseFrame("WAIT", t) + phaseFrame("READY", t)
		}},
		{"ready-before-continue", func(t, c string) string { return c + phaseFrame("WAIT", t) + phaseFrame("READY", t) }},
		{"continue-without-wait", func(t, c string) string { return c + phaseFrame("CONTINUE", t) + phaseFrame("READY", t) }},
		{"cancel-without-wait", func(t, c string) string { return c + phaseFrame("CANCEL", t) }},
		{"wrong-generation", func(t, c string) string { return c + phaseFrame("READY", strings.Replace(t, "a", "f", 1)) }},
		{"wrong-challenge", func(t, c string) string { return strings.Replace(c, "eeee", "ffff", 1) + phaseFrame("READY", t) }},
		{"wrong-profile", func(t, c string) string {
			return strings.Replace(c, "0000000000000003", "0000000000000006", 1) + phaseFrame("READY", t)
		}},
		{"uppercase-id", func(t, c string) string { return strings.ToUpper(c) + phaseFrame("READY", t) }},
		{"nonascii", func(t, c string) string { return c + "\xff" }},
		{"oversize", func(t, c string) string { return strings.Repeat("x", 513) + "\n" }},
		{"crlf", func(t, c string) string { return strings.ReplaceAll(c, "\n", "\r\n") + phaseFrame("READY", t) }},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			o, b, _, _, tail, config := executionFixture(t)
			b.bytes = []byte(tc.wire(tail, config))
			result, err := o.Observe(context.Background())
			if err == nil || result != nil {
				t.Fatalf("accepted %s", tc.name)
			}
			if !b.closed {
				t.Fatal("failed evidence retained backend")
			}
		})
	}
}
func TestExecutionChildLiveAuthorityProfileAndTimeRequired(t *testing.T) {
	cases := []struct {
		name   string
		change func(*ExecutionChildObserver, *memoryExecutionChild, *memoryExecutionAuthority, *ExecutionColdProfile)
	}{
		{"early-ready", func(o *ExecutionChildObserver, b *memoryExecutionChild, a *memoryExecutionAuthority, p *ExecutionColdProfile) {
			o.state.now = func() time.Time { return o.state.prepared.Add(29 * time.Second) }
		}},
		{"deadline", func(o *ExecutionChildObserver, b *memoryExecutionChild, a *memoryExecutionAuthority, p *ExecutionColdProfile) {
			o.state.now = func() time.Time { return o.state.prepared.Add(90 * time.Second) }
		}},
		{"wrong-native-child-age", func(o *ExecutionChildObserver, b *memoryExecutionChild, a *memoryExecutionAuthority, p *ExecutionColdProfile) {
			b.readyAfter = 29 * time.Second
		}},
		{"native-auth", func(o *ExecutionChildObserver, b *memoryExecutionChild, a *memoryExecutionAuthority, p *ExecutionColdProfile) {
			b.authErr = errors.New("native token changed")
		}},
		{"native-image", func(o *ExecutionChildObserver, b *memoryExecutionChild, a *memoryExecutionAuthority, p *ExecutionColdProfile) {
			b.validateErr = errors.New("original image changed")
		}},
		{"revoked-executor", func(o *ExecutionChildObserver, b *memoryExecutionChild, a *memoryExecutionAuthority, p *ExecutionColdProfile) {
			a.err = errors.New("disconnected")
		}},
		{"different-logon", func(o *ExecutionChildObserver, b *memoryExecutionChild, a *memoryExecutionAuthority, p *ExecutionColdProfile) {
			a.peer.LogonIDLow++
		}},
		{"different-client", func(o *ExecutionChildObserver, b *memoryExecutionChild, a *memoryExecutionAuthority, p *ExecutionColdProfile) {
			o.state.identity.ClientSID = "S-1-5-21-1-2-3-501"
		}},
		{"profile-closed", func(o *ExecutionChildObserver, b *memoryExecutionChild, a *memoryExecutionAuthority, p *ExecutionColdProfile) {
			_ = p.Close()
		}},
		{"profile-ancestry", func(o *ExecutionChildObserver, b *memoryExecutionChild, a *memoryExecutionAuthority, p *ExecutionColdProfile) {
			p.state.backend.(*memoryExecutionProfile).err = errors.New("physical directory changed")
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			o, b, a, p, tail, config := executionFixture(t)
			b.bytes = []byte(config + phaseFrame("READY", tail))
			tc.change(o, b, a, p)
			observed, err := o.Observe(context.Background())
			if err == nil || observed != nil {
				t.Fatal("accepted revoked or early evidence")
			}
		})
	}
}
func TestExecutionChildCancellationAndCloseJoinBlockedRead(t *testing.T) {
	for _, mode := range []string{"caller-cancel", "owner-close"} {
		t.Run(mode, func(t *testing.T) {
			o, b, _, _, _, _ := executionFixture(t)
			b.entered = make(chan struct{})
			b.block = make(chan struct{})
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			done := make(chan error, 1)
			go func() { _, err := o.Observe(ctx); done <- err }()
			<-b.entered
			if mode == "caller-cancel" {
				cancel()
			} else {
				if err := o.Close(); err != nil {
					t.Fatal(err)
				}
			}
			select {
			case err := <-done:
				if !errors.Is(err, context.Canceled) {
					t.Fatal(err)
				}
			case <-time.After(2 * time.Second):
				t.Fatal("blocked IO lifetime escaped")
			}
			if !b.closed {
				t.Fatal("native owner not closed")
			}
		})
	}
}
func TestExecutionChildHealthyReceiptRemainsLiveAndOpaque(t *testing.T) {
	o, b, a, p, tail, config := executionFixture(t)
	env, err := o.Environment(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if len(env) != 4 || env["SKAGER_UPDATE_QUALIFY_PROFILE"] != "1" {
		t.Fatal(env)
	}
	env["SKAGER_UPDATE_PIPE"] = "changed"
	if o.state.environment["SKAGER_UPDATE_PIPE"] == "changed" {
		t.Fatal("borrowed environment")
	}
	b.bytes = []byte(config + phaseFrame("READY", tail))
	healthy, err := o.Observe(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if _, err = json.Marshal(healthy); err == nil {
		t.Fatal("serialized live authority")
	}
	a.err = errors.New("executor died")
	if err = healthy.Validate(context.Background()); err == nil {
		t.Fatal("receipt survived executor death")
	}
	a.err = nil
	if err = healthy.Validate(context.Background()); err == nil {
		t.Fatal("revoked owner resurrected")
	}
	_ = p.Close()
}
func TestExecutionColdProfileCopiesLifetimeAndNoWriterExclusionClaim(t *testing.T) {
	p, b := profileFixture()
	defer p.Close()
	raw, err := p.OriginalBytes(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	raw[0] = 'X'
	again, err := p.OriginalBytes(context.Background())
	if err != nil || again[0] == 'X' {
		t.Fatal("borrowed original bytes")
	}
	if err = p.ValidateBeforeLaunch(context.Background()); err != nil {
		t.Fatal(err)
	}
	b.beforeErr = errors.New("legitimate later INI changed")
	if err = p.Validate(context.Background()); err != nil {
		t.Fatal("post-load original file exclusion incorrectly claimed")
	}
	if err = p.ValidateBeforeLaunch(context.Background()); err == nil {
		t.Fatal("changed prelaunch INI accepted")
	}
	b.beforeErr = nil
	if err = p.Validate(context.Background()); err == nil {
		t.Fatal("failed original resurrected")
	}
	if _, err = json.Marshal(p); err == nil {
		t.Fatal("serialized profile authority")
	}
	if err = p.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err = p.OriginalBytes(context.Background()); err == nil {
		t.Fatal("closed original available")
	}
}
func TestExecutionChildFailurePreservesCleanupUncertainty(t *testing.T) {
	o, b, _, _, _, _ := executionFixture(t)
	cause := errors.New("close handle uncertain")
	b.closeErr = cause
	b.err = errors.New("connection refused")
	_, err := o.Observe(context.Background())
	if !errors.Is(err, ErrExecutionChildCleanup) || !errors.Is(err, cause) {
		t.Fatal(err)
	}
	if err = o.Close(); !errors.Is(err, cause) {
		t.Fatal("lost cached cleanup", err)
	}
}
func TestExecutionChildZeroAndNonNativeCannotMintAuthority(t *testing.T) {
	ctx := context.Background()
	if _, err := PrepareExecutionChildNative(ctx, nil, nil, nil); err == nil {
		t.Fatal("zero factory")
	}
	if _, err := (&ExecutionChildObserver{}).Observe(ctx); err == nil {
		t.Fatal("zero observation")
	}
	if err := (&ObservedExecutionChild{}).Validate(ctx); err == nil {
		t.Fatal("zero receipt")
	}
	if _, err := (&ExecutionColdProfile{}).OriginalBytes(ctx); err == nil {
		t.Fatal("zero profile")
	}
	var protocol executionChildProtocol
	if _, err := protocol.feed('\n'); err == nil {
		t.Fatal("empty parser accepted")
	}
}

func TestExecutionChildRequiresActualEOFAfterTerminal(t *testing.T) {
	o, b, _, _, tail, config := executionFixture(t)
	b.bytes = []byte(config + phaseFrame("READY", tail))
	b.zeroRead = true
	if healthy, err := o.Observe(context.Background()); !errors.Is(err, io.ErrNoProgress) || healthy != nil {
		t.Fatalf("zero read became EOF: %v %v", healthy, err)
	}
}
