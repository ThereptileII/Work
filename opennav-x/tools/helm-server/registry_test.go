package helm

import (
	"testing"
	"time"
)

func TestPairingRevocationAndDisconnect(t *testing.T) {
	now := time.Unix(100, 0)
	r, err := NewSessionRegistry("pc-1", "SKAGER", "123456", now, 1)
	if err != nil {
		t.Fatal(err)
	}
	cred, err := r.Pair("123456", "helm-phone", now)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := r.Authorize(cred.Token, now); err != nil {
		t.Fatal(err)
	}
	r.Disconnect(cred.SessionID, now.Add(time.Second))
	if _, err := r.Authorize(cred.Token, now.Add(2*time.Second)); err == nil {
		t.Fatal("disconnected session remained authorized")
	}
	r.Revoke(cred.SessionID)
	if got := len(r.Sessions()); got != 0 {
		t.Fatalf("revoke retained %d sessions", got)
	}
}

func TestPairingExpiresAndDoesNotLeakToken(t *testing.T) {
	now := time.Unix(100, 0)
	r, err := NewSessionRegistry("pc-1", "SKAGER", "123456", now, 2)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := r.Pair("wrong", "phone", now); err == nil {
		t.Fatal("wrong code accepted")
	}
	if _, err := r.Pair("123456", "phone", now.Add(11*time.Minute)); err == nil {
		t.Fatal("expired code accepted")
	}
	if len(r.Sessions()) != 0 {
		t.Fatal("failed pairing created a session")
	}
}

func TestInputRejectsUnsafeEventsAndAcceptsNormalizedTouch(t *testing.T) {
	now := time.Unix(100, 0)
	r, _ := NewSessionRegistry("pc-1", "SKAGER", "123456", now, 2)
	cred, _ := r.Pair("123456", "phone", now)
	if _, err := r.AcceptInput(cred.Token, InputEvent{Type: "pointer_down", X: .5, Y: .5, Sequence: 1}, now); err != nil {
		t.Fatal(err)
	}
	if _, err := r.AcceptInput(cred.Token, InputEvent{Type: "pointer_down", X: 2, Y: .5, Sequence: 2}, now); err == nil {
		t.Fatal("out-of-range input accepted")
	}
	if _, err := r.AcceptInput(cred.Token, InputEvent{Type: "pointer_down", X: .5, Y: .5}, now); err == nil {
		t.Fatal("zero sequence accepted")
	}
}
