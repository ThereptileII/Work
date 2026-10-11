//go:build !windows

package brokerpipe

import (
	"context"
	"errors"
	"testing"
)

func TestPortableNeverSubstitutesAuthentication(t *testing.T) {
	if e := Serve(context.Background(), func(context.Context, Peer, Request) (Result, error) {
		t.Fatal("portable handler invoked")
		return Result{}, nil
	}); !errors.Is(e, ErrUnavailable) {
		t.Fatal(e)
	}
	if _, e := Call(context.Background(), requestFixture()); !errors.Is(e, ErrUnavailable) {
		t.Fatal(e)
	}
}
