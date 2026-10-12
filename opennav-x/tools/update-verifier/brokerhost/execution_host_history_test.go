package brokerhost

import (
	"errors"
	"strings"
	"testing"

	"example.com/opennav-update-verifier/brokerstore"
)

func TestExecutionHostHistoryAdmitsOnlyExactHeldNewJournal(t *testing.T) {
	sid := "S-1-5-21-1-2-3-500"
	id := strings.Repeat("1", 32)
	oldID := strings.Repeat("2", 32)
	entry := func(id string) brokerstore.ExecutionJournalEntry {
		return brokerstore.ExecutionJournalEntry{ExecutionID: id, EntryNames: []string{sid + ".execution." + id + ".journal", sid + ".execution." + id + ".once"}, Readable: true}
	}
	old := entry(oldID)
	original, err := executionSelectionInventoryDigest(sid, []brokerstore.ExecutionJournalEntry{old})
	if err != nil {
		t.Fatal(err)
	}
	own := brokerstore.ExecutionJournalSnapshot{ExecutionID: id, ClientSID: sid, JournalSHA256: strings.Repeat("3", 64)}
	for _, liveWriter := range []bool{true, false} {
		t.Run(map[bool]string{true: "exclusive-writer", false: "readable-original"}[liveWriter], func(t *testing.T) {
			next := entry(id)
			var enumerateErr error
			if liveWriter {
				next.Readable = false
				next.Problem = "original execution pair incomplete or custody uncertain"
				enumerateErr = &brokerstore.ExecutionJournalNamespaceError{ClientSID: sid, Entries: []brokerstore.ExecutionJournalEntry{old, next}, Cause: errors.New("exclusive writer")}
			}
			if err := executionHostInventory(sid, id, original, []brokerstore.ExecutionJournalEntry{old, next}, enumerateErr, &own); err != nil {
				t.Fatal(err)
			}
		})
	}
	cases := map[string]func([]brokerstore.ExecutionJournalEntry) []brokerstore.ExecutionJournalEntry{
		"unknown-attempt": func(e []brokerstore.ExecutionJournalEntry) []brokerstore.ExecutionJournalEntry {
			return append(e, entry(strings.Repeat("4", 32)))
		},
		"lost-history": func(e []brokerstore.ExecutionJournalEntry) []brokerstore.ExecutionJournalEntry { return e[1:] },
		"changed-old": func(e []brokerstore.ExecutionJournalEntry) []brokerstore.ExecutionJournalEntry {
			e[0].Readable = false
			return e
		},
		"own-alias": func(e []brokerstore.ExecutionJournalEntry) []brokerstore.ExecutionJournalEntry {
			e[1].EntryNames[0] = strings.ToUpper(e[1].EntryNames[0])
			return e
		},
		"own-extra": func(e []brokerstore.ExecutionJournalEntry) []brokerstore.ExecutionJournalEntry {
			e[1].EntryNames = append(e[1].EntryNames, "extra")
			return e
		},
		"own-missing": func(e []brokerstore.ExecutionJournalEntry) []brokerstore.ExecutionJournalEntry { return e[:1] },
		"own-partial": func(e []brokerstore.ExecutionJournalEntry) []brokerstore.ExecutionJournalEntry {
			e[1].Problem = "permanent reservation/journal pair missing or duplicate"
			e[1].Readable = false
			return e
		},
	}
	for name, mutate := range cases {
		t.Run(name, func(t *testing.T) {
			if executionHostInventory(sid, id, original, mutate([]brokerstore.ExecutionJournalEntry{entry(oldID), entry(id)}), nil, &own) == nil {
				t.Fatal("changed namespace admitted")
			}
		})
	}
	t.Run("unclassified-enumeration-failure", func(t *testing.T) {
		if executionHostInventory(sid, id, original, []brokerstore.ExecutionJournalEntry{old, entry(id)}, errors.New("unknown"), &own) == nil {
			t.Fatal("unknown failure ignored")
		}
	})
	t.Run("typed-error-not-own-failure", func(t *testing.T) {
		e := []brokerstore.ExecutionJournalEntry{old, entry(id)}
		err := &brokerstore.ExecutionJournalNamespaceError{ClientSID: sid, Entries: e, Cause: errors.New("other unknown")}
		if executionHostInventory(sid, id, original, e, err, &own) == nil {
			t.Fatal("readable own pair waived unrelated error")
		}
	})
}
