package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"testing"

	"example.com/opennav-update-verifier/repository"
	"time"
)

type freshCacheFixture struct {
	root                        []byte
	seedErr, checkErr, closeErr error
	seeded                      bool
	seeds, closes               int
}

func (f *freshCacheFixture) check(ctx context.Context) error {
	return errors.Join(ctx.Err(), f.checkErr)
}
func (f *freshCacheFixture) seed(ctx context.Context, root []byte) error {
	f.seeds++
	f.seeded = true
	f.root = append([]byte(nil), root...)
	return errors.Join(ctx.Err(), f.seedErr)
}
func (f *freshCacheFixture) close() error { f.closes++; return f.closeErr }
func TestFreshBootstrapCacheExactApprovedRootAndBoundedFailures(t *testing.T) {
	for _, fault := range []string{"valid", "trust-pin", "provisioner-pin", "invalid-trust", "source", "parent", "seed", "readback", "cleanup", "source-cleanup", "canceled"} {
		t.Run(fault, func(t *testing.T) {
			f := newEnrollmentFixture(t)
			req := FreshBootstrapCacheRequest{TrustConfiguration: bytes.Clone(f.req.TrustConfiguration), TrustConfigSHA256: f.req.TrustConfigSHA256, ProvisionerSHA256: f.req.ProvisionerSHA256, BootstrapCacheDir: "independently-approved-absent-cache"}
			source := &maintenanceImageFixture{Reader: bytes.NewReader([]byte("inert original source")), testOwner: &testOwner{}}
			owner := &freshCacheFixture{}
			opened := false
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			op := freshBootstrapCacheOperations{source: func(ctx context.Context, pin string) (maintenanceImageSource, error) {
				if pin != f.req.ProvisionerSHA256 {
					t.Fatal("provisioner approval substituted")
				}
				return source, nil
			}, open: func(ctx context.Context, path string) (freshBootstrapCacheOwner, error) {
				opened = true
				if path != req.BootstrapCacheDir || source.closed {
					t.Fatal("original path/source custody lost")
				}
				if fault == "parent" {
					return nil, errors.New("existing or redirected native parent/cache")
				}
				return owner, nil
			}}
			switch fault {
			case "trust-pin":
				req.TrustConfigSHA256 = digest([]byte("other"))
			case "provisioner-pin":
				req.ProvisionerSHA256 = ""
			case "invalid-trust":
				req.TrustConfiguration = []byte("{}")
				req.TrustConfigSHA256 = digest(req.TrustConfiguration)
			case "source":
				source.checkErr = errors.New("actual source differs")
			case "seed":
				owner.seedErr = errors.New("partial root write")
			case "readback":
				op.open = func(context.Context, string) (freshBootstrapCacheOwner, error) {
					opened = true
					return &freshCacheReadbackFailure{freshCacheFixture: owner}, nil
				}
			case "cleanup":
				owner.closeErr = errors.New("cache handle close refused")
			case "source-cleanup":
				source.closeErr = errors.New("source handle close refused")
			case "canceled":
				cancel()
			}
			err := prepareFreshBootstrapCacheWith(ctx, req, op)
			if (err == nil) != (fault == "valid") {
				t.Fatal("incorrect fresh preparation result", fault, err)
			}
			effect := fault == "valid" || fault == "seed" || fault == "readback" || fault == "cleanup" || fault == "source-cleanup"
			if errors.Is(err, ErrFreshBootstrapCacheUncertain) != (effect && fault != "valid") {
				t.Fatal("uncertain effect classification lost", fault, err)
			}
			if effect {
				trust, e := repository.ValidateTrust(f.req.TrustConfiguration, time.Now().UTC())
				if e != nil {
					t.Fatal(e)
				}
				if owner.seeds != 1 || owner.closes != 1 || !bytes.Equal(owner.root, trust.BootstrapRoot) || !source.closed {
					t.Fatal("exact original root/lifetime lost")
				}
			} else if owner.seeds != 0 {
				t.Fatal("unapproved preparation changed cache")
			}
			if !opened && owner.closes != 0 {
				t.Fatal("foreign owner closed")
			}
		})
	}
}

type freshCacheReadbackFailure struct{ *freshCacheFixture }

func (f *freshCacheReadbackFailure) check(ctx context.Context) error {
	if f.seeded {
		return errors.New("root native readback differs")
	}
	return f.freshCacheFixture.check(ctx)
}
