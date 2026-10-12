package brokerpipe

import (
	"context"
	"testing"
)

func TestExecutionSessionStartupModeBoundBeforeCommand(t *testing.T) {
	for _, mode := range []StartupMode{StartupXNav, StartupLegacy, StartupSafe} {
		for _, mismatch := range []bool{false, true} {
			ctx, cancel := context.WithCancel(context.Background())
			q := executionTestRequest()
			q.Operation = "start-current"
			q.StartupMode = mode
			authority, connection := &fakeExecutionAuthority{}, &fakeExecutionConnection{}
			session, err := newExecutionSession(ctx, cancel, q, authority, connection)
			if err != nil {
				cancel()
				t.Fatal(err)
			}
			bindExecutionTestSession(t, session)
			directive := ExecutionDirective{Phase: SpawnCurrent, ApplicationGeneration: q.StageID, StartupMode: mode}
			if mismatch {
				directive.StartupMode = StartupLegacy
				if mode == StartupLegacy {
					directive.StartupMode = StartupSafe
				}
			}
			_, err = session.ExchangeCommand(ctx, directive)
			if mismatch {
				if err == nil || len(connection.commands) != 0 {
					t.Fatal("mismatched mode transmitted")
				}
			} else {
				if err != nil || len(connection.commands) != 1 || connection.commands[0].StartupMode != mode {
					t.Fatalf("mode not retained: %v", err)
				}
			}
			session.Close()
			cancel()
		}
	}
}
