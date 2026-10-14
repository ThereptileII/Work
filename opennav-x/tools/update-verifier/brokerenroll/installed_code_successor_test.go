package brokerenroll

import (
	"context"
	"errors"
	"strings"
	"testing"
)

type successorPreparationFixture struct {
	req                       CodeUpgradeRequest
	released                  bool
	closes                    int
	err, releaseErr, closeErr error
}

func (p *successorPreparationFixture) check(ctx context.Context) error {
	return errors.Join(ctx.Err(), p.err)
}
func (p *successorPreparationFixture) request() CodeUpgradeRequest { return p.req }
func (p *successorPreparationFixture) releaseCompletion(ctx context.Context) error {
	p.released = true
	return errors.Join(ctx.Err(), p.releaseErr)
}
func (p *successorPreparationFixture) close() error { p.closes++; return p.closeErr }
func TestInstalledCodeSuccessorReopensThenReleasesOriginalPinsBeforeEngine(t *testing.T) {
	for _, fault := range []string{"valid", "comparisons", "prepare", "check", "mismatch", "resume", "release", "engine", "cleanup"} {
		t.Run(fault, func(t *testing.T) {
			c := InstalledCodeUpgradeComparisons{PolicySHA256: strings.Repeat("a", 64), ProvisionerSHA256: strings.Repeat("b", 64), TrustConfigSHA256: strings.Repeat("c", 64), OriginalCompletionSHA256: strings.Repeat("d", 64)}
			p := &successorPreparationFixture{req: CodeUpgradeRequest{TrustConfiguration: []byte("original trust"), BootstrapCacheDir: "original-bound-cache", ArtifactDirectory: "original-bound-cache", PolicySHA256: c.PolicySHA256, ProvisionerSHA256: c.ProvisionerSHA256, TrustConfigSHA256: c.TrustConfigSHA256, OriginalCompletionSHA256: c.OriginalCompletionSHA256}}
			if fault == "comparisons" {
				c.PolicySHA256 = ""
			}
			if fault == "check" {
				p.err = errors.New("native source changed")
			}
			if fault == "mismatch" {
				p.req.ProvisionerSHA256 = strings.Repeat("e", 64)
			}
			if fault == "resume" {
				p.req.Resume = true
			}
			if fault == "release" {
				p.releaseErr = errors.New("original pin release failed")
			}
			if fault == "cleanup" {
				p.closeErr = errors.New("source close failed")
			}
			prepared, upgraded := false, false
			err := runInstalledCodeUpgradeSuccessorWith(context.Background(), c, func(ctx context.Context, got InstalledCodeUpgradeComparisons) (installedCodeSuccessorPreparation, error) {
				prepared = true
				if got != c {
					t.Fatal("comparison tuple substituted")
				}
				if fault == "prepare" {
					return p, errors.New("original completion absent")
				}
				return p, nil
			}, func(ctx context.Context, req CodeUpgradeRequest) error {
				upgraded = true
				if !p.released || p.closes != 0 || req.BootstrapCacheDir != "original-bound-cache" || req.ArtifactDirectory != req.BootstrapCacheDir {
					t.Fatal("engine called before original release or with substituted cache")
				}
				if fault == "engine" {
					return errors.New("existing upgrade engine refused")
				}
				return nil
			})
			if (err == nil) != (fault == "valid") {
				t.Fatal("incorrect successor result", fault, err)
			}
			wantUpgrade := fault == "valid" || fault == "engine" || fault == "cleanup"
			if upgraded != wantUpgrade || prepared != (fault != "comparisons") || p.closes != installedSuccessorFixtureBool(fault != "comparisons") {
				t.Fatal("unsafe sequence or partial preparation leak", fault)
			}
		})
	}
}
func installedSuccessorFixtureBool(v bool) int {
	if v {
		return 1
	}
	return 0
}
