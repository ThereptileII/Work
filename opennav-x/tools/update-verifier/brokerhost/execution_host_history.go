package brokerhost

import (
	"encoding/json"
	"errors"
	"example.com/opennav-update-verifier/brokerstore"
	"sort"
)

// Admit exactly the live native writer's original pair. Exclusive writer sharing
// can prevent independent reopen, so its own Snapshot supplies custody, while ALL
// prior original readers and the complete remaining inventory must stay exact.
// No extra/alias/partial pair, new attempt or arbitrary enumeration error passes.
func executionHostInventory(sid, id, original string, entries []brokerstore.ExecutionJournalEntry, enumerationErr error, own *brokerstore.ExecutionJournalSnapshot) error {
	if len(entries) > 128 {
		return errExecutionHostRefused
	}
	var previous []brokerstore.ExecutionJournalEntry
	found := false
	ownUnreadable := false
	for _, entry := range entries {
		if own != nil && entry.ExecutionID == id {
			if found || own.ExecutionID != id || own.ClientSID != sid || !hashPattern.MatchString(own.JournalSHA256) || len(entry.EntryNames) != 2 {
				return errExecutionHostRefused
			}
			names := append([]string(nil), entry.EntryNames...)
			sort.Strings(names)
			prefix := sid + ".execution." + id
			if names[0] != prefix+".journal" || names[1] != prefix+".once" {
				return errExecutionHostRefused
			}
			if !entry.Readable && entry.Problem != "original execution pair incomplete or custody uncertain" {
				return errExecutionHostRefused
			}
			if entry.Readable && entry.Problem != "" {
				return errExecutionHostRefused
			}
			ownUnreadable = !entry.Readable
			found = true
			continue
		}
		previous = append(previous, entry)
	}
	if own != nil && !found {
		return errExecutionHostRefused
	}
	digest, err := executionSelectionInventoryDigest(sid, previous)
	if err != nil || digest != original {
		return errors.Join(errExecutionHostRefused, err)
	}
	if enumerationErr != nil {
		var uncertain *brokerstore.ExecutionJournalNamespaceError
		if own == nil || !found || !ownUnreadable || !errors.As(enumerationErr, &uncertain) || uncertain.ClientSID != sid {
			return errExecutionHostRefused
		}
		// Only the exact writer entry may fail; prior entries were all required
		// readable above. Cancellation/incomplete enumeration never becomes admission.
		if len(uncertain.Entries) != len(entries) {
			return errExecutionHostRefused
		}
		rawA, _ := json.Marshal(uncertain.Entries)
		rawB, _ := json.Marshal(entries)
		if string(rawA) != string(rawB) {
			return errExecutionHostRefused
		}
	}
	return nil
}
