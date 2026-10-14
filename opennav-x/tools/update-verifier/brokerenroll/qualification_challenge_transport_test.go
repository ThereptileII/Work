package brokerenroll

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"strings"
	"testing"
)

func qualificationChallengeFixture() []byte {
	// Independent golden transport: Python's sorted ASCII keys and final newline.
	return []byte(`{"approvalSha256":"` + strings.Repeat("a", 64) + `","challenge":"` + strings.Repeat("b", 64) + `","headSha":"` + strings.Repeat("c", 40) + `","jobId":"123","jobName":"installed-qualification","purpose":"installed-updater-b0-consumption","repository":"ThereptileII/Work","requestSha256":"` + strings.Repeat("d", 64) + `","runAttempt":"1","runId":"456","schema":1}` + "\n")
}

func qualificationIdentityFixture() qualificationChallengeIdentity {
	return qualificationChallengeIdentity{"123456789", strings.Repeat("e", 32), "134000000000000000", strings.Repeat("f", 64)}
}

type qualificationOwnerFixture struct {
	id                                  qualificationChallengeIdentity
	identityErr, consumeErr, cleanupErr error
	consumeCalls, closeCalls, checks    int
	changedAfterReady                   bool
	foreignFinalParent                  bool
	wrongDigest                         bool
	events                              *[]string
}

func (n *qualificationOwnerFixture) identity(context.Context) (qualificationChallengeIdentity, error) {
	n.checks++
	id := n.id
	if n.changedAfterReady && n.checks > 1 {
		id.FileID = strings.Repeat("1", 32)
	}
	return id, n.identityErr
}
func (n *qualificationOwnerFixture) consume(_ context.Context, raw []byte, r qualificationChallengeRecord) (qualificationChallengeReceipt, error) {
	n.consumeCalls++
	*n.events = append(*n.events, "consume")
	parent := n.id
	if n.foreignFinalParent {
		parent.CreationTime = "1"
	}
	hash := digest(raw)
	if n.wrongDigest {
		hash = strings.Repeat("0", 64)
	}
	return qualificationChallengeReceipt{1, "consumed", hash, parent, n.id}, n.consumeErr
}
func (n *qualificationOwnerFixture) close() error {
	n.closeCalls++
	*n.events = append(*n.events, "close")
	return n.cleanupErr
}

type qualificationOutputFixture struct {
	bytes.Buffer
	events *[]string
}

func (o *qualificationOutputFixture) Write(raw []byte) (int, error) {
	var value struct{ State string }
	if err := json.Unmarshal(raw, &value); err != nil {
		return 0, err
	}
	*o.events = append(*o.events, value.State)
	return o.Buffer.Write(raw)
}

func TestQualificationChallengeTwoPhaseCustodyAndCheckedCleanup(t *testing.T) {
	events := []string{}
	owner := &qualificationOwnerFixture{id: qualificationIdentityFixture(), events: &events}
	output := &qualificationOutputFixture{events: &events}
	handled, err := qualificationChallengeBridge(context.Background(), []string{qualificationChallengeFlag + `=C:\Protected\Qualification`}, bytes.NewReader(qualificationChallengeFixture()), output, func(_ context.Context, path string) (qualificationChallengeOwner, error) {
		if path != `C:\Protected\Qualification` {
			t.Fatal("parent transport changed")
		}
		events = append(events, "open")
		return owner, nil
	})
	if !handled || err != nil || owner.consumeCalls != 1 || owner.closeCalls != 1 || strings.Join(events, ",") != "open,ready,consume,close,consumed" {
		t.Fatal("ready/original custody/cleanup order lost", handled, err, events)
	}
	frames := bytes.Split(bytes.TrimSpace(output.Bytes()), []byte("\n"))
	if len(frames) != 2 {
		t.Fatal("not exactly ready and consumed frames")
	}
	var ready qualificationChallengeReady
	var receipt qualificationChallengeReceipt
	if json.Unmarshal(frames[0], &ready) != nil || json.Unmarshal(frames[1], &receipt) != nil || ready.Parent != receipt.Parent || receipt.RecordSHA256 != digest(qualificationChallengeFixture()) {
		t.Fatal("original identity/digest missing")
	}
}

func TestQualificationChallengeInvalidInputNeverConsumes(t *testing.T) {
	golden := qualificationChallengeFixture()
	bad := map[string][]byte{
		"empty-eof":       nil,
		"partial-eof":     golden[:len(golden)/2],
		"oversized":       bytes.Repeat([]byte("x"), qualificationChallengeLimit+1),
		"duplicate":       bytes.Replace(golden, []byte(`"schema":1}`), []byte(`"schema":1,"schema":1}`), 1),
		"unknown":         bytes.Replace(golden, []byte(`"schema":1}`), []byte(`"schema":1,"unknown":1}`), 1),
		"noncanonical":    append([]byte(" "), golden...),
		"missing-newline": golden[:len(golden)-1],
		"foreign-repo":    bytes.Replace(golden, []byte("ThereptileII/Work"), []byte("foreign/Work"), 1),
		"runner-alias":    bytes.Replace(golden, []byte(`"runAttempt":"1"`), []byte(`"runAttempt":"01"`), 1),
		"challenge-case":  bytes.Replace(golden, []byte(strings.Repeat("b", 64)), []byte(strings.Repeat("B", 64)), 1),
	}
	for name, raw := range bad {
		t.Run(name, func(t *testing.T) {
			events := []string{}
			owner := &qualificationOwnerFixture{id: qualificationIdentityFixture(), events: &events}
			var output bytes.Buffer
			handled, err := qualificationChallengeBridge(context.Background(), []string{qualificationChallengeFlag + "=original-parent"}, bytes.NewReader(raw), &output, func(context.Context, string) (qualificationChallengeOwner, error) { return owner, nil })
			if !handled || err == nil || owner.consumeCalls != 0 || owner.closeCalls != 1 || strings.Contains(output.String(), `"consumed"`) {
				t.Fatal("invalid input consumed or leaked original owner", handled, err)
			}
		})
	}
}

func TestQualificationChallengeRefusalCannotEmitConsumed(t *testing.T) {
	for _, name := range []string{"partial-open", "identity", "parent-changed", "consume", "foreign-final-parent", "digest", "cleanup"} {
		t.Run(name, func(t *testing.T) {
			events := []string{}
			owner := &qualificationOwnerFixture{id: qualificationIdentityFixture(), events: &events}
			refused := errors.New("native original owner refused")
			var openErr error
			switch name {
			case "partial-open":
				openErr = refused
			case "identity":
				owner.identityErr = refused
			case "parent-changed":
				owner.changedAfterReady = true
			case "consume":
				owner.consumeErr = refused
			case "foreign-final-parent":
				owner.foreignFinalParent = true
			case "digest":
				owner.wrongDigest = true
			case "cleanup":
				owner.cleanupErr = refused
			}
			var output bytes.Buffer
			_, err := qualificationChallengeBridge(context.Background(), []string{qualificationChallengeFlag + "=original-parent"}, bytes.NewReader(qualificationChallengeFixture()), &output, func(context.Context, string) (qualificationChallengeOwner, error) { return owner, openErr })
			if err == nil || owner.closeCalls != 1 || strings.Contains(output.String(), `"consumed"`) {
				t.Fatal("uncertain native result became consumed", name, err)
			}
			if name == "parent-changed" && owner.consumeCalls != 0 {
				t.Fatal("a different parent consumed the record")
			}
			if name == "cleanup" && !errors.Is(err, refused) {
				t.Fatal("cleanup cause lost", err)
			}
		})
	}
}

type qualificationPanicReader struct{}

func (qualificationPanicReader) Read([]byte) (int, error) { panic("unexpected input read") }

type qualificationCancelOnReady struct {
	bytes.Buffer
	cancel context.CancelFunc
}

func (o *qualificationCancelOnReady) Write(raw []byte) (int, error) {
	o.cancel()
	return o.Buffer.Write(raw)
}

func TestQualificationChallengeCancellationAfterReadyNeverConsumes(t *testing.T) {
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	events := []string{}
	owner := &qualificationOwnerFixture{id: qualificationIdentityFixture(), events: &events}
	output := &qualificationCancelOnReady{cancel: cancel}
	_, err := qualificationChallengeBridge(ctx, []string{qualificationChallengeFlag + "=original-parent"}, bytes.NewReader(qualificationChallengeFixture()), output, func(context.Context, string) (qualificationChallengeOwner, error) { return owner, nil })
	if err == nil || owner.consumeCalls != 0 || owner.closeCalls != 1 || strings.Contains(output.String(), `"consumed"`) {
		t.Fatal("cancellation after readiness consumed a challenge", err)
	}
}

func TestQualificationChallengePrivateFlagNeverMixesWithTestsOrCommands(t *testing.T) {
	for _, args := range [][]string{{qualificationChallengeFlag}, {qualificationChallengeFlag, "parent"}, {qualificationChallengeFlag + "=parent", "-test.run=TestOther"}, {"unknown", qualificationChallengeFlag + "=parent"}, {qualificationChallengeFlag + "=parent", qualificationChallengeFlag + "=parent"}, {qualificationChallengeFlag + "="}} {
		handled, err := qualificationChallengeBridge(context.Background(), args, qualificationPanicReader{}, io.Discard, func(context.Context, string) (qualificationChallengeOwner, error) {
			t.Fatal("mixed arguments reached native owner")
			return nil, nil
		})
		if !handled || err == nil {
			t.Fatal("mixed or incomplete private command accepted", args)
		}
	}
	if handled, err := qualificationChallengeBridge(context.Background(), []string{"-test.run=TestNativeOther", "-test.v"}, qualificationPanicReader{}, io.Discard, nil); handled || err != nil {
		t.Fatal("ordinary native test routing changed", handled, err)
	}
}
