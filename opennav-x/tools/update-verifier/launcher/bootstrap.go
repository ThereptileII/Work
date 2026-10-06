package launcher

import (
	"context"
	"errors"
	"os"
	"strings"
	"time"
)

// This request only restricts bootstrap startup; it grants no commissioning or
// update authority. The boat wrapper still owns its independent audit. Ordinary
// product startup never enters this route or receives these extra arguments.
func bootstrap(ctx context.Context, p platform, l layout, state installState, expectation string) error {
	generationID, receiptHash, ok := strings.Cut(expectation, ":")
	if !ok || !generationPattern.MatchString(generationID) ||
		(receiptHash != "absent" && !hashPattern.MatchString(receiptHash)) ||
		generationID != l.generation || state.Current != generationID {
		return errors.New("invalid private bootstrap expectation")
	}
	g, err := loadGeneration(p, l)
	if err != nil {
		return err
	}
	defer g.close()
	if g.owned.Health != 1 {
		return errors.New("private bootstrap requires health version 1")
	}
	lock, err := p.lockStartup(l.root + "/transaction.lock")
	if err != nil {
		return err
	}
	// Release before invoking a supervisor which takes this same lock. The
	// absent-receipt expectation is revalidated there, not inferred again.
	locked := true
	defer func() {
		if locked {
			_ = lock.Close()
		}
	}()
	if err := assertCurrent(p, g, state.Stock); err != nil {
		return err
	}
	for _, name := range []string{l.root + "/update-pending.json", l.app + "/update-trust.json"} {
		if _, err := p.read(name, 32768); !errors.Is(err, os.ErrNotExist) {
			return errors.New("private bootstrap requires no pending update or configured trust")
		}
	}
	receipt := l.root + "/known-good/" + generationID + ".receipt"
	if receiptHash != "absent" {
		held, err := p.hold(receipt, receiptHash)
		if err != nil {
			return errors.New("selected bootstrap receipt changed")
		}
		defer held.Close()
		return p.start(processSpec{executable: l.app + "/opencpn.exe", directory: l.app, arguments: []string{"--xnav"}})
	}
	if _, err := p.read(receipt, 32768); !errors.Is(err, os.ErrNotExist) {
		return errors.New("bootstrap receipt appeared before qualification")
	}
	engine, err := p.powershell()
	if err != nil {
		return err
	}
	if err := lock.Close(); err != nil {
		return err
	}
	locked = false
	result, err := p.wait(ctx, processSpec{executable: engine, directory: l.app,
		arguments: []string{"-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", l.directory + "/UpdateSupervisor.ps1", "-InstallationRoot", l.root, "-Action", "QualifyCurrent", "-BootstrapExpectation", expectation}, limit: 11 * time.Minute})
	if result.timedOut && result.done != nil {
		<-result.done
	}
	if err != nil || result.timedOut || result.code != 0 {
		return errors.New("private bootstrap qualification refused; no retry")
	}
	return nil
}
