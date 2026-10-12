package brokerhost

import (
	"context"
	"errors"
	"strings"
	"testing"
)

func TestSCMInitializationDispatch(t *testing.T) {
	for _, tc := range []struct {
		name    string
		args    []string
		handled bool
		wantErr bool
		calls   int
	}{
		{"normal", []string{ServiceName}, false, false, 0},
		{"explicit initialization", []string{ServiceName, "initialize-store"}, true, false, 1},
		{"missing", nil, false, true, 0},
		{"wrong service", []string{"other"}, false, true, 0},
		{"wrong initializer service", []string{"other", "initialize-store"}, false, true, 0},
		{"service case alias", []string{strings.ToLower(ServiceName), "initialize-store"}, false, true, 0},
		{"command case alias", []string{ServiceName, "INITIALIZE-STORE"}, false, true, 0},
		{"unknown action", []string{ServiceName, "install"}, false, true, 0},
		{"extra arguments", []string{ServiceName, "initialize-store", "root"}, false, true, 0},
		{"empty argument", []string{ServiceName, ""}, false, true, 0},
	} {
		t.Run(tc.name, func(t *testing.T) {
			calls := 0
			handled, err := initializeServiceStart(context.Background(), tc.args, func() error { calls++; return nil }, nil)
			if handled != tc.handled || (err != nil) != tc.wantErr || calls != tc.calls {
				t.Fatalf("handled=%v err=%v calls=%d; want handled=%v error=%v calls=%d", handled, err, calls, tc.handled, tc.wantErr, tc.calls)
			}
		})
	}
}

func TestSCMInitializationRefusal(t *testing.T) {
	args := []string{ServiceName, "initialize-store"}
	t.Run("native failure is redacted", func(t *testing.T) {
		calls := 0
		handled, err := initializeServiceStart(context.Background(), args, func() error { calls++; return errors.New("private fixture path") }, nil)
		if !handled || err == nil || calls != 1 || strings.Contains(err.Error(), "private fixture path") {
			t.Fatalf("initialization failure was not retained and redacted: handled=%v err=%v calls=%d", handled, err, calls)
		}
	})
	t.Run("missing native creator", func(t *testing.T) {
		handled, err := initializeServiceStart(context.Background(), args, nil, nil)
		if !handled || err == nil {
			t.Fatalf("missing creator accepted: handled=%v err=%v", handled, err)
		}
	})
	for _, normal := range []bool{false, true} {
		t.Run(map[bool]string{false: "cancelled initialization", true: "cancelled normal start"}[normal], func(t *testing.T) {
			ctx, cancel := context.WithCancel(context.Background())
			cancel()
			selected := args
			if normal {
				selected = []string{ServiceName}
			}
			calls := 0
			_, err := initializeServiceStart(ctx, selected, func() error { calls++; return nil }, nil)
			if err == nil || calls != 0 {
				t.Fatalf("cancelled start invoked creator: err=%v calls=%d", err, calls)
			}
		})
	}
	t.Run("nil context", func(t *testing.T) {
		calls := 0
		_, err := initializeServiceStart(nil, args, func() error { calls++; return nil }, nil)
		if err == nil || calls != 0 {
			t.Fatalf("nil context invoked creator: err=%v calls=%d", err, calls)
		}
	})
	t.Run("cancelled during initialization never reports success", func(t *testing.T) {
		ctx, cancel := context.WithCancel(context.Background())
		defer cancel()
		calls := 0
		handled, err := initializeServiceStart(ctx, args, func() error { calls++; cancel(); return nil }, nil)
		if !handled || err == nil || calls != 1 {
			t.Fatalf("interrupted initialization accepted: handled=%v err=%v calls=%d", handled, err, calls)
		}
	})
}
