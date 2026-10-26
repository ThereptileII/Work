package helm

import "testing"

func TestAnnexBNALUnits(t *testing.T) {
	data := []byte{0, 0, 0, 1, 0x67, 1, 2, 0, 0, 1, 0x68, 3}
	nals := AnnexBNALUnits(data)
	if len(nals) != 2 || len(nals[0]) != 3 || nals[0][0] != 0x67 || nals[1][0] != 0x68 {
		t.Fatalf("unexpected NALs: %#v", nals)
	}
}
