package launcher

import (
	"bytes"
	"encoding/binary"
	"testing"
)

func TestMaintenanceFolderAccessChangesOnlyExactExistingQueryACE(t *testing.T) {
	user, system, broker := queryTestSID(21), queryTestSID(18), queryTestSID(80)
	for _, position := range []int{0, 1, 2} {
		aces := [][]byte{queryTestACE(user, 0x1fffff), queryTestACE(system, 0x1fffff)}
		aces = append(aces, nil)
		copy(aces[position+1:], aces[position:])
		aces[position] = queryTestACE(broker, brokerQueryTokenMask)
		raw := queryTestACL(aces...)
		raw = append(raw, 0xde, 0xad, 0xbe, 0xef)
		binary.LittleEndian.PutUint16(raw[2:4], uint16(len(raw)))
		before := bytes.Clone(raw)
		got, err := maintenanceFolderACL(raw, broker, [][]byte{user, system})
		if err != nil {
			t.Fatal(err)
		}
		offset := 8
		for index := 0; index < position; index++ {
			offset += len(aces[index])
		}
		expected := bytes.Clone(raw)
		binary.LittleEndian.PutUint32(expected[offset+4:offset+8], 0x0c)
		if !bytes.Equal(got, expected) || !bytes.Equal(raw, before) {
			t.Fatal("maintenance changed original ACE/trustee/header/padding bytes")
		}
		if _, err = appendBrokerQueryACE(got, broker, brokerQueryTokenMask, [][]byte{user, system}); err == nil {
			t.Fatal("ordinary query helper silently widened its policy")
		}
		if _, err = maintenanceFolderACL(got, broker, [][]byte{user, system}); err == nil {
			t.Fatal("temporary grant used as query-only baseline")
		}
		got[8] ^= 1
		if !bytes.Equal(raw, before) {
			t.Fatal("maintenance result aliases baseline")
		}
	}
}

func TestMaintenanceFolderAccessRejectsAmbiguousOrExcessiveQueryGrant(t *testing.T) {
	user, broker := queryTestSID(21), queryTestSID(80)
	base := queryTestACL(queryTestACE(user, 0x1fffff), queryTestACE(broker, 8))
	tests := map[string]func([]byte) []byte{
		"missing-broker": func([]byte) []byte { return queryTestACL(queryTestACE(user, 0x1fffff)) },
		"duplicate": func([]byte) []byte {
			return queryTestACL(queryTestACE(user, 0x1fffff), queryTestACE(broker, 8), queryTestACE(broker, 8))
		},
		"impersonate-existing":  func(b []byte) []byte { binary.LittleEndian.PutUint32(b[len(b)-len(broker)-4:], 0x0c); return b },
		"query-source-existing": func(b []byte) []byte { binary.LittleEndian.PutUint32(b[len(b)-len(broker)-4:], 0x18); return b },
		"duplicate-right":       func(b []byte) []byte { binary.LittleEndian.PutUint32(b[len(b)-len(broker)-4:], 0xa); return b },
		"adjust-right":          func(b []byte) []byte { binary.LittleEndian.PutUint32(b[len(b)-len(broker)-4:], 0x28); return b },
		"process-rights": func(b []byte) []byte {
			binary.LittleEndian.PutUint32(b[len(b)-len(broker)-4:], brokerQueryProcessMask)
			return b
		},
		"deny":             func(b []byte) []byte { b[8] = 1; return b },
		"callback":         func(b []byte) []byte { b[8] = 9; return b },
		"inherited-broker": func(b []byte) []byte { b[8+len(queryTestACE(user, 0x1fffff))+1] = 0x10; return b },
		"unknown-trustee":  func([]byte) []byte { return queryTestACL(queryTestACE(queryTestSID(999), 8), queryTestACE(broker, 8)) },
		"truncated":        func(b []byte) []byte { return b[:len(b)-1] },
		"bad-size":         func(b []byte) []byte { b[2]--; return b },
		"bad-sid":          func(b []byte) []byte { b[17] = 15; return b },
	}
	for name, mutate := range tests {
		t.Run(name, func(t *testing.T) {
			if out, err := maintenanceFolderACL(mutate(bytes.Clone(base)), broker, [][]byte{user}); err == nil || out != nil {
				t.Fatal("unsafe temporary folder grant accepted")
			}
		})
	}
	if out, err := maintenanceFolderACL(base, nil, [][]byte{user}); err == nil || out != nil {
		t.Fatal("missing broker identity accepted")
	}
}
