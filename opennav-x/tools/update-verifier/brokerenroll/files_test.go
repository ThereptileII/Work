package brokerenroll

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"io"
	"testing"

	"example.com/opennav-update-verifier/releasepolicy"
)

func imageArtifact(data []byte) releasepolicy.Artifact {
	hash := sha256.Sum256(data)
	return releasepolicy.Artifact{Path: "artifacts/0123456789012345678901234567890123456789/" + enrollmentImageName, Bytes: int64(len(data)), SHA256: hex.EncodeToString(hash[:])}
}

func TestEnrollmentImageExactBytes(t *testing.T) {
	data := bytes.Repeat([]byte("broker bytes"), 9000)
	artifact := imageArtifact(data)
	var output bytes.Buffer
	if err := copyEnrollmentImage(context.Background(), &output, bytes.NewReader(data), artifact); err != nil || !bytes.Equal(output.Bytes(), data) {
		t.Fatalf("exact stream: %v", err)
	}
	for _, input := range [][]byte{data[:len(data)-1], append(append([]byte{}, data...), 1), bytes.Repeat([]byte("x"), len(data))} {
		output.Reset()
		if err := copyEnrollmentImage(context.Background(), &output, bytes.NewReader(input), artifact); err == nil {
			t.Fatal("changed stream accepted")
		}
		if output.Len() > len(data) {
			t.Fatal("extra bytes written")
		}
	}
}

type enrollmentFailWriter struct{}

func (enrollmentFailWriter) Write(data []byte) (int, error) { return len(data) - 1, nil }

type enrollmentEmptyReader struct{}

func (enrollmentEmptyReader) Read([]byte) (int, error) { return 0, nil }

type enrollmentErrorReader struct{}

func (enrollmentErrorReader) Read([]byte) (int, error) { return 0, io.ErrUnexpectedEOF }

func TestEnrollmentImageRefusesUncertainIO(t *testing.T) {
	artifact := imageArtifact([]byte("broker"))
	if err := copyEnrollmentImage(context.Background(), enrollmentFailWriter{}, bytes.NewBufferString("broker"), artifact); !errors.Is(err, io.ErrShortWrite) {
		t.Fatalf("short write: %v", err)
	}
	if err := copyEnrollmentImage(context.Background(), io.Discard, enrollmentEmptyReader{}, artifact); !errors.Is(err, io.ErrNoProgress) {
		t.Fatalf("empty stream: %v", err)
	}
	if err := copyEnrollmentImage(context.Background(), io.Discard, enrollmentErrorReader{}, artifact); !errors.Is(err, io.ErrUnexpectedEOF) {
		t.Fatalf("read error: %v", err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if err := copyEnrollmentImage(ctx, io.Discard, bytes.NewBufferString("broker"), artifact); err == nil {
		t.Fatal("cancelled stream accepted")
	}
}

func TestEnrollmentImageInputsBeforeCreation(t *testing.T) {
	good := imageArtifact([]byte("broker"))
	for _, change := range []func(*releasepolicy.Artifact){
		func(a *releasepolicy.Artifact) { a.Bytes = 0 }, func(a *releasepolicy.Artifact) { a.Bytes = maxBrokerBytes + 1 },
		func(a *releasepolicy.Artifact) {
			a.SHA256 = "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
		},
		func(a *releasepolicy.Artifact) { a.Path = "artifacts/other.exe" },
	} {
		a := good
		change(&a)
		if err := validateEnrollmentImage(context.Background(), bytes.NewBufferString("broker"), a); err == nil {
			t.Fatal("invalid artifact accepted")
		}
	}
	if err := validateEnrollmentImage(nil, bytes.NewBufferString("broker"), good); err == nil {
		t.Fatal("missing context accepted")
	}
	if err := validateEnrollmentImage(context.Background(), nil, good); err == nil {
		t.Fatal("missing source accepted")
	}
}
