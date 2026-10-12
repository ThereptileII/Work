package launcher

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"testing"
)

func queryTestSID(id uint32) []byte {
	out := []byte{1, 1, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0}
	binary.LittleEndian.PutUint32(out[8:], id)
	return out
}
func queryTestACE(sid []byte, mask uint32) []byte {
	out := make([]byte, 8+len(sid))
	binary.LittleEndian.PutUint16(out[2:4], uint16(len(out)))
	binary.LittleEndian.PutUint32(out[4:8], mask)
	copy(out[8:], sid)
	return out
}
func queryTestACL(aces ...[]byte) []byte {
	out := make([]byte, 8)
	out[0] = 2
	for _, ace := range aces {
		out = append(out, ace...)
	}
	binary.LittleEndian.PutUint16(out[2:4], uint16(len(out)))
	binary.LittleEndian.PutUint16(out[4:6], uint16(len(aces)))
	return out
}
func TestBrokerQueryAccessPreservesOriginals(t *testing.T) {
	user, system, broker := queryTestSID(21), queryTestSID(18), queryTestSID(80)
	for _, mask := range []uint32{brokerQueryTokenMask, brokerQueryProcessMask} {
		t.Run(fmt.Sprintf("%08x", mask), func(t *testing.T) {
			first, second := queryTestACE(user, 0x1fffff), queryTestACE(system, 0x1fffff)
			second[1] = 0x10
			original := queryTestACL(first, second)
			original = append(original, 0xde, 0xad, 0xbe, 0xef)
			binary.LittleEndian.PutUint16(original[2:4], uint16(len(original)))
			before := bytes.Clone(original)
			got, err := appendBrokerQueryACE(original, broker, mask, [][]byte{user, system})
			if err != nil {
				t.Fatal(err)
			}
			pos := 8 + len(first) + len(second)
			if !bytes.Equal(got[8:pos], original[8:pos]) || !bytes.Equal(got[pos:pos+8+len(broker)], queryTestACE(broker, mask)) || !bytes.Equal(got[pos+8+len(broker):], original[pos:]) || !bytes.Equal(original, before) {
				t.Fatal("original ACE/padding or caller buffer changed")
			}
			again, err := appendBrokerQueryACE(got, broker, mask, [][]byte{user, system})
			if err != nil || !bytes.Equal(got, again) {
				t.Fatal("exact grant not idempotent", err)
			}
			again[8] ^= 1
			if bytes.Equal(got, again) {
				t.Fatal("result aliases caller buffer")
			}
		})
	}
}
func TestBrokerQueryAccessRejectsUnsafeACL(t *testing.T) {
	user, broker := queryTestSID(21), queryTestSID(80)
	good := queryTestACL(queryTestACE(user, 0x1fffff))
	cases := map[string]func([]byte) []byte{
		"null": func([]byte) []byte { return nil }, "empty": func([]byte) []byte { return queryTestACL() },
		"revision": func(b []byte) []byte { b[0] = 4; return b }, "reserved": func(b []byte) []byte { b[6] = 1; return b }, "size": func(b []byte) []byte { b[2]--; return b },
		"count": func(b []byte) []byte { b[4] = 2; return b }, "truncated": func(b []byte) []byte { return b[:len(b)-1] }, "badAceSize": func(b []byte) []byte { b[10] = 0xff; return b },
		"deny": func(b []byte) []byte { b[8] = 1; return b }, "callback": func(b []byte) []byte { b[8] = 9; return b }, "inheritedOnly": func(b []byte) []byte { b[9] = 8; return b }, "objectInheritance": func(b []byte) []byte { b[9] = 1; return b },
		"sidRevision": func(b []byte) []byte { b[16] = 2; return b }, "sidLength": func(b []byte) []byte { b[17] = 15; return b },
		"unknownTrustee": func([]byte) []byte { return queryTestACL(queryTestACE(queryTestSID(1), 8)) },
		"brokerExcess":   func([]byte) []byte { return queryTestACL(queryTestACE(user, 0x1fffff), queryTestACE(broker, 0xa)) },
		"brokerWrongObject": func([]byte) []byte {
			return queryTestACL(queryTestACE(user, 0x1fffff), queryTestACE(broker, brokerQueryProcessMask))
		},
		"brokerDuplicate": func([]byte) []byte {
			return queryTestACL(queryTestACE(user, 0x1fffff), queryTestACE(broker, 8), queryTestACE(broker, 8))
		},
		"brokerInherited": func([]byte) []byte {
			a := queryTestACE(broker, 8)
			a[1] = 0x10
			return queryTestACL(queryTestACE(user, 0x1fffff), a)
		},
	}
	for name, change := range cases {
		t.Run(name, func(t *testing.T) {
			raw := change(bytes.Clone(good))
			if _, err := appendBrokerQueryACE(raw, broker, 8, [][]byte{user}); err == nil {
				t.Fatal("unsafe ACL accepted")
			}
		})
	}
	for _, mask := range []uint32{0, 2, 0xa, 0x1fffff} {
		if _, err := appendBrokerQueryACE(good, broker, mask, [][]byte{user}); err == nil {
			t.Fatal("broader mask accepted", mask)
		}
	}
	for _, sid := range [][]byte{nil, {1, 15, 0, 0, 0, 0, 0, 5}, {2, 0, 0, 0, 0, 0, 0, 5}} {
		if _, err := appendBrokerQueryACE(good, sid, 8, [][]byte{user}); err == nil {
			t.Fatal("malformed broker SID accepted")
		}
	}
}
