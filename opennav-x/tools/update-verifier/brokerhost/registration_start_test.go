package brokerhost

import (
	"bytes"
	"context"
	"errors"
	"strings"
	"testing"
)

func TestClientRegistrationServiceStart(t *testing.T) {
	raw := testApprovedClientRequest()
	pin := clientRequestDigest(raw)
	reads, writes := 0, 0
	read := func(_ context.Context, sid, approved string) ([]byte, error) {
		reads++
		if sid != testSID || approved != pin {
			t.Fatal("different SCM request")
		}
		return raw, nil
	}
	write := func(_ context.Context, owned []byte, approved string) error {
		writes++
		if !bytes.Equal(owned, raw) || approved != pin {
			t.Fatal("different original request")
		}
		return nil
	}
	handled, err := registerClientServiceStart(context.Background(), []string{ServiceName, "register-client", testSID, pin}, read, write)
	if !handled || err != nil || reads != 1 || writes != 1 {
		t.Fatalf("handled=%v error=%v reads=%d writes=%d", handled, err, reads, writes)
	}
}
func TestClientRegistrationServiceStartRefusesBeforeMutation(t *testing.T) {
	raw := testApprovedClientRequest()
	pin := clientRequestDigest(raw)
	for name, args := range map[string][]string{
		"missing":       {ServiceName, "register-client"},
		"extra":         {ServiceName, "register-client", testSID, pin, `C:\`},
		"wrong service": {"other", "register-client", testSID, pin},
		"service case":  {strings.ToLower(ServiceName), "register-client", testSID, pin},
		"path SID":      {ServiceName, "register-client", "../account", pin},
		"SID alias":     {ServiceName, "register-client", strings.Replace(testSID, "S-1-5-21", "S-1-5-021", 1), pin},
		"SID overflow":  {ServiceName, "register-client", "S-1-5-21-1-2-3-4294967296", pin},
		"short pin":     {ServiceName, "register-client", testSID, "a"},
		"upper pin":     {ServiceName, "register-client", testSID, strings.ToUpper(pin)},
	} {
		t.Run(name, func(t *testing.T) {
			calls := 0
			handled, err := registerClientServiceStart(context.Background(), args, func(context.Context, string, string) ([]byte, error) { calls++; return raw, nil }, func(context.Context, []byte, string) error { calls++; return nil })
			if !handled || err == nil || calls != 0 {
				t.Fatal("invalid SCM arguments reached callback")
			}
		})
	}
}
func TestClientRegistrationServiceStartKeepsExistingModes(t *testing.T) {
	for _, args := range [][]string{{ServiceName}, {ServiceName, "initialize-store"}, {ServiceName, "verify-store"}, {ServiceName, "REGISTER-CLIENT", testSID, strings.Repeat("a", 64)}} {
		handled, err := registerClientServiceStart(context.Background(), args, nil, nil)
		if handled || err != nil {
			t.Fatal("intercepted existing/refused mode")
		}
	}
}
func TestClientRegistrationServiceStartOriginalApprovalAndCommit(t *testing.T) {
	raw := testApprovedClientRequest()
	pin := clientRequestDigest(raw)
	args := []string{ServiceName, "register-client", testSID, pin}
	for _, name := range []string{"nil context", "cancelled", "read failure", "changed bytes", "wrong user", "cancel during read", "nil reader", "nil writer", "commit cancellation", "uncertain commit"} {
		t.Run(name, func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			var errSentinel = errors.New("inert reader refusal")
			reads, writes := 0, 0
			read := func(context.Context, string, string) ([]byte, error) {
				reads++
				switch name {
				case "read failure":
					return nil, errSentinel
				case "changed bytes":
					return append(bytes.Clone(raw), ' '), nil
				case "cancel during read":
					cancel()
				}
				return raw, nil
			}
			write := func(context.Context, []byte, string) error {
				writes++
				if name == "commit cancellation" {
					cancel()
				}
				if name == "uncertain commit" {
					return ErrClientRegistrationUncertain
				}
				return nil
			}
			selected := append([]string(nil), args...)
			switch name {
			case "nil context":
				ctx = nil
			case "cancelled":
				cancel()
			case "wrong user":
				selected[2] = "S-1-5-21-1-2-3-1001"
			case "nil reader":
				read = nil
			case "nil writer":
				write = nil
			}
			handled, err := registerClientServiceStart(ctx, selected, read, write)
			if !handled {
				t.Fatal("explicit registration was not handled")
			}
			if name == "commit cancellation" {
				if err != nil || writes != 1 {
					t.Fatal("durable success relabelled cancellation")
				}
				return
			}
			if name == "uncertain commit" {
				if !errors.Is(err, ErrClientRegistrationUncertain) || writes != 1 {
					t.Fatal("commit uncertainty lost")
				}
				return
			}
			if err == nil || writes != 0 {
				t.Fatal("invalid request reached native writer")
			}
			if name == "read failure" && !errors.Is(err, errSentinel) {
				t.Fatal("read failure lost")
			}
		})
	}
}
