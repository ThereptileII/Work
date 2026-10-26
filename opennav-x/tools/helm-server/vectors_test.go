package helm

import (
	"encoding/hex"
	"testing"
)

// Fixed vectors shared with the app (mobile/helm/tests/lib.test.ts): both
// implementations must derive the same keys and MACs.
func TestCrossImplementationVectors(t *testing.T) {
	secret, err := ParsePairingKey("MFRG-GZDF-MZTW-Q2LK")
	if err != nil {
		t.Fatal(err)
	}
	if hex.EncodeToString(secret) != "6162636465666768696a" {
		t.Fatalf("secret %x", secret)
	}
	cn := make([]byte, 16)
	sn := make([]byte, 16)
	for i := range cn {
		cn[i], sn[i] = byte(i), byte(0xf0+i)
	}
	key, err := DeviceKey(secret, cn, sn, "srv-id-123", "dev-id-456")
	if err != nil {
		t.Fatal(err)
	}
	vectors := map[string]string{
		"device key": hex.EncodeToString(key),
		"pair":       MAC(secret, pairMessage("srv-id-123", b64(cn), "Kristoffer's iPhone")),
		"paired":     MAC(key, pairedMessage("dev-id-456", b64(cn), b64(sn))),
		"offer":      MAC(key, OfferMessage("dev-id-456", "chal", b64(cn), "v=0\r\no=- 1 2 IN IP4 0.0.0.0\r\n", true)),
		"answer":     MAC(key, AnswerMessage("dev-id-456", "chal", b64(cn), "v=0\r\n")),
	}
	want := map[string]string{
		"device key": "f09c0dbb89b09fb48068dfb70e0b9ee7c50f61f7f8abd6f6419633d54e4273df",
		"pair":       "I3rNCSABtAlm1yCr1h-ZYq-i0Kh44ruXuNyzn8XTgaY",
		"paired":     "AisXu2TQiz_G5mFCPGey6yBWZuocJqaK94uZrsEsAB8",
		"offer":      "u-0UVbiCISaOHcH3s7Fm_0l2N9an9gNqBFFhB-9Q3ZE",
		"answer":     "0JUoQoVOUgZbNCxh13Wr2DbLjqgTe89magcsLpveGTw",
	}
	for k, v := range vectors {
		if want[k] != v {
			t.Errorf("%s = %s, want %s", k, v, want[k])
		}
	}
}
