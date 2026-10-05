package launcher

import (
	"bytes"
	"context"
	"errors"
	"io"
	"os"
	"strings"
	"time"

	verifier "example.com/opennav-update-verifier"
	"example.com/opennav-update-verifier/releasepolicy"
)

type processSpec struct {
	executable, directory string
	arguments             []string
	input                 []byte
	limit                 time.Duration
}
type processResult struct {
	code     int
	timedOut bool
	done     <-chan struct{}
}
type platform interface {
	executable() (string, error)
	read(string, int64) ([]byte, error)
	hold(string, string) (io.Closer, error)
	measure(string) (string, error)
	lockStartup(string) (io.Closer, error)
	powershell() (string, error)
	wait(context.Context, processSpec) (processResult, error)
	start(processSpec) error
}
type preparation interface {
	path() string
	policy() releasepolicy.Policy
	close() error
}
type acquired struct{ prepared *verifier.PreparedUpdate }

func (a acquired) path() string                 { return a.prepared.Artifact.Path() }
func (a acquired) policy() releasepolicy.Policy { return a.prepared.Policy }
func (a acquired) close() error                 { return a.prepared.Close() }

type services struct {
	platform platform
	check    func(context.Context, verifier.PrepareConfig, releasepolicy.OpenCpn) (verifier.UpdateCandidate, error)
	prepare  func(context.Context, verifier.PrepareConfig, releasepolicy.OpenCpn, verifier.ConsentIdentity) (preparation, error)
}

// Run starts only the current generation selected by the owned installation.
// Missing/unavailable updater configuration falls back to ordinary startup,
// without retry or implicit trust initialization. After installation begins,
// timeout/failure cannot silently start an unqualified generation.
//
// On installer timeout this function retains custody until that exact process
// exits, performs no further update/startup actions, then returns an error. The
// operational wait budget is bounded; guardian lifetime intentionally is not.
// Native Windows process, ACL, image-sharing and recovery tests remain required.
func Run(ctx context.Context, arguments []string) error {
	p, err := newPlatform()
	if err != nil {
		return err
	}
	return run(ctx, arguments, services{platform: p, check: verifier.CheckUpdate, prepare: func(ctx context.Context, c verifier.PrepareConfig, o releasepolicy.OpenCpn, i verifier.ConsentIdentity) (preparation, error) {
		v, err := verifier.PrepareUpdate(ctx, c, o, i)
		if err != nil {
			return nil, err
		}
		return acquired{v}, nil
	}})
}

type generation struct {
	layout  layout
	owned   ownership
	managed map[string]string
	held    []io.Closer
}

func (g *generation) close() {
	if g == nil {
		return
	}
	for _, h := range g.held {
		_ = h.Close()
	}
	g.held = nil
}
func (g *generation) hold(p platform, relative string) error {
	hash := g.managed[strings.ToLower(relative)]
	if hash == "" {
		return errors.New("required helper missing from managed inventory")
	}
	name, err := ownedPath(g.layout.directory, relative)
	if err != nil {
		return err
	}
	h, err := p.hold(name, hash)
	if err != nil {
		return errors.New("owned executable or helper integrity failed")
	}
	g.held = append(g.held, h)
	return nil
}
func loadGenerationRecord(p platform, l layout) (*generation, error) {
	data, err := p.read(l.directory+"/ownership.json", 4<<20)
	if err != nil {
		return nil, errors.New("generation ownership unavailable")
	}
	o, managed, err := parseOwnership(data, l.directory)
	if err != nil {
		return nil, err
	}
	return &generation{layout: l, owned: o, managed: managed}, nil
}
func loadGeneration(p platform, l layout) (*generation, error) {
	g, err := loadGenerationRecord(p, l)
	if err != nil {
		return nil, err
	}
	for _, relative := range []string{"app/opencpn.exe", "app/skager-start.exe"} {
		if err := g.hold(p, relative); err != nil {
			g.close()
			return nil, err
		}
	}
	if g.owned.Health == 1 {
		for _, relative := range []string{"app/skager-update-prompt.exe", "Lifecycle.ps1", "UpdateSupervisor.ps1", "UpdateTransaction.ps1"} {
			if err := g.hold(p, relative); err != nil {
				g.close()
				return nil, err
			}
		}
	}
	return g, nil
}
func readState(p platform, l layout) (installState, error) {
	data, err := p.read(l.root+"/owner.json", 4096)
	if err != nil {
		return installState{}, errors.New("installation owner unavailable")
	}
	var own struct {
		Owner string `json:"owner"`
	}
	if err := decodeRecord(data, &own, []string{"owner"}, true); err != nil || own.Owner != owner {
		return installState{}, errors.New("unrecognized installation owner")
	}
	data, err = p.read(l.root+"/state.json", 32768)
	if err != nil {
		return installState{}, errors.New("installation state unavailable")
	}
	return parseState(data)
}
func selectedLayout(l layout, id string) layout {
	return layout{root: l.root, generation: id, directory: l.root + "/generations/" + id, app: l.root + "/generations/" + id + "/app"}
}
func assertCurrent(p platform, g *generation, stock stockRecord) error {
	s, err := readState(p, g.layout)
	if err != nil || s.Current != g.layout.generation || s.Stock != stock {
		return errors.New("selected generation or stock identity changed")
	}
	return nil
}
func pendingExists(p platform, l layout) (bool, error) {
	_, err := p.read(l.root+"/update-pending.json", 4096)
	if errors.Is(err, os.ErrNotExist) {
		return false, nil
	}
	if err != nil {
		return false, errors.New("pending update record unavailable")
	}
	return true, nil
}

func run(ctx context.Context, args []string, s services) error {
	if ctx == nil {
		return errors.New("launcher context required")
	}
	mode := "--xnav"
	if len(args) > 1 {
		return errors.New("unsupported startup arguments")
	}
	if len(args) == 1 {
		mode = args[0]
	}
	if mode != "--xnav" && mode != "--legacy" && mode != "--safe-mode" && mode != "--initialize-trust" {
		return errors.New("unsupported startup mode")
	}
	exe, err := s.platform.executable()
	if err != nil {
		return errors.New("installed launcher location unavailable")
	}
	l, err := deriveLayout(exe)
	if err != nil {
		return err
	}
	state, err := readState(s.platform, l)
	if err != nil {
		return err
	}
	if mode != "--initialize-trust" {
		pending, err := pendingExists(s.platform, l)
		if err != nil {
			return err
		}
		if pending {
			return recoverStartup(ctx, s.platform, l, state, mode)
		}
	}
	if state.Current != l.generation {
		return errors.New("launcher is not the selected installed generation")
	}
	g, err := loadGeneration(s.platform, l)
	if err != nil {
		return err
	}
	defer func() { g.close() }()
	if mode == "--initialize-trust" {
		return initializeTrust(s.platform, g)
	}
	if mode != "--xnav" || g.owned.Health != 1 {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	// A first launch must establish a locally authenticated recovery generation
	// before an update can be offered. Lifecycle validates the receipt itself.
	if _, err := s.platform.read(l.root+"/known-good/"+l.generation+".receipt", 32768); err != nil {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	data, err := s.platform.read(l.app+"/update-trust.json", maxTrustBytes)
	if err != nil {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	// Provisioned trust itself must be covered by the installed immutable record.
	if err := g.hold(s.platform, "app/update-trust.json"); err != nil {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	// Reread while Windows denies write/delete, avoiding a pre-hold byte race.
	data, err = s.platform.read(l.app+"/update-trust.json", maxTrustBytes)
	if err != nil {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	trust, err := parseTrust(data)
	if err != nil {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	observed, err := observeStock(s.platform, state.Stock, trust.Allowlist)
	if err != nil {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	config := preparationConfig(l, g.owned, trust)
	checkCtx, cancel := context.WithTimeout(ctx, 5*time.Second)
	candidate, err := s.check(checkCtx, config, observed)
	cancel()
	if err != nil || candidate.Decision != releasepolicy.Upgrade {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	input, err := promptInput(candidate.Consent)
	if err != nil {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	if err := assertCurrent(s.platform, g, state.Stock); err != nil {
		return err
	}
	choice, err := s.platform.wait(ctx, processSpec{executable: l.app + "/skager-update-prompt.exe", directory: l.app, input: input, limit: 5 * time.Minute})
	if err != nil || choice.timedOut || choice.code != 10 {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	// Refresh measured local identity and current pointer after user interaction.
	if err := assertCurrent(s.platform, g, state.Stock); err != nil {
		return err
	}
	observed, err = observeStock(s.platform, state.Stock, trust.Allowlist)
	if err != nil {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	prepared, err := s.prepare(ctx, config, observed, candidate.Consent)
	if err != nil {
		return ordinary(ctx, s.platform, g, state.Stock, mode)
	}
	defer prepared.close()
	if err := assertCurrent(s.platform, g, state.Stock); err != nil {
		return err
	}
	result, err := s.platform.wait(ctx, processSpec{executable: prepared.path(), directory: l.app, arguments: []string{"/S", "/ACTION=Update", "/SUPERVISED=1"}, limit: 15 * time.Minute})
	if result.timedOut {
		// Do not kill NSIS or release its immutable artifact while it is alive.
		if result.done != nil {
			<-result.done
		}
		return errors.New("installer wait expired; acceptance and startup withheld; inspect recovery")
	}
	if err != nil || result.code != 0 {
		return errors.New("supervised installation did not complete; inspect recovery before startup")
	}
	return launchInstalled(ctx, s.platform, g, state.Stock, prepared.policy())
}
func promptInput(consent verifier.ConsentIdentity) ([]byte, error) {
	if _, err := releasepolicy.ParseSemVer(consent.Version); err != nil || !commitPattern.MatchString(consent.Commit) || !hashPattern.MatchString(consent.PolicySHA256) {
		return nil, errors.New("invalid confirmed offer identity")
	}
	input := []byte(consent.PolicySHA256 + "\n" + consent.Version + "\n" + consent.Commit + "\n")
	if len(input) > 256 {
		return nil, errors.New("offer exceeds prompt bound")
	}
	return input, nil
}
func observeStock(p platform, stock stockRecord, allowlist []releasepolicy.OpenCpn) (releasepolicy.OpenCpn, error) {
	hash, err := p.measure(stock.Path)
	if err != nil || hash != stock.SHA256 {
		return releasepolicy.OpenCpn{}, errors.New("installed OpenCPN identity changed")
	}
	var found releasepolicy.OpenCpn
	count := 0
	for _, entry := range allowlist {
		if entry.ExecutableSHA256 == hash && entry.Arch == "x86" && entry.Version == stock.Version && entry.UpstreamCommit == baselineCommit {
			found = entry
			count++
		}
	}
	if count != 1 {
		return found, errors.New("installed OpenCPN lacks unique independent compatibility approval")
	}
	return found, nil
}
func ordinary(ctx context.Context, p platform, g *generation, stock stockRecord, mode string) error {
	if err := assertCurrent(p, g, stock); err != nil {
		return err
	}
	if mode == "--xnav" && g.owned.Health == 1 {
		if _, err := p.read(g.layout.root+"/known-good/"+g.layout.generation+".receipt", 32768); err != nil {
			return supervise(ctx, p, g, stock, "QualifyCurrent", "")
		}
	}
	lock, err := p.lockStartup(g.layout.root + "/transaction.lock")
	if err != nil {
		return errors.New("installation maintenance is in progress; application startup withheld")
	}
	defer lock.Close()
	// Installer maintenance takes this same lock and rechecks closed processes.
	// Hold it across the final pointer read and actual process creation.
	if err := assertCurrent(p, g, stock); err != nil {
		return err
	}
	return p.start(processSpec{executable: g.layout.app + "/opencpn.exe", directory: g.layout.app, arguments: []string{mode}})
}
func supervise(ctx context.Context, p platform, g *generation, stock stockRecord, action, transaction string) error {
	if err := assertCurrent(p, g, stock); err != nil {
		return err
	}
	return invokeSupervisor(ctx, p, g, action, transaction)
}
func invokeSupervisor(ctx context.Context, p platform, g *generation, action, transaction string) error {
	engine, err := p.powershell()
	if err != nil {
		return errors.New("system Windows PowerShell unavailable")
	}
	args := []string{"-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", g.layout.directory + "/UpdateSupervisor.ps1", "-InstallationRoot", g.layout.root, "-Action", action}
	if transaction != "" {
		if !generationPattern.MatchString(transaction) {
			return errors.New("invalid pending transaction")
		}
		args = append(args, "-Transaction", transaction)
	}
	result, err := p.wait(ctx, processSpec{executable: engine, directory: g.layout.app, arguments: args, limit: 4 * time.Minute})
	if result.timedOut && result.done != nil {
		<-result.done
	}
	if err != nil || result.timedOut || result.code != 0 {
		return errors.New("supervised startup unavailable; no acceptance or retry")
	}
	return nil
}

// Interrupted candidate corruption cannot be allowed to veto recovery. Check
// the running launcher's own identity, bind the immutable pending identities,
// and execute only the fully verified previous generation's recovery engine.
func recoverStartup(ctx context.Context, p platform, l layout, state installState, mode string) error {
	running, err := loadGenerationRecord(p, l)
	if err != nil {
		return err
	}
	defer running.close()
	if err := running.hold(p, "app/skager-start.exe"); err != nil {
		return err
	}
	data, err := p.read(l.root+"/update-pending.json", 4096)
	if err != nil {
		return errors.New("pending recovery identity unavailable")
	}
	var pending pendingRecord
	if err := decodeRecord(data, &pending, []string{"schema", "owner", "transaction", "candidate", "previous", "attempts", "session"}, false); err != nil || pending.Schema != 1 || pending.Owner != "SKAGER.UpdateStartup.1" || !generationPattern.MatchString(pending.Transaction) || !validPendingIdentity(pending.Candidate) || !validPendingIdentity(pending.Previous) || pending.Candidate.Generation == pending.Previous.Generation || (pending.Attempts != 0 && pending.Attempts != 1) || (pending.Attempts == 0 && pending.Session != "") || (pending.Attempts == 1 && !generationPattern.MatchString(pending.Session)) {
		return errors.New("pending recovery identity rejected")
	}
	// State publication precedes shortcut publication. A crash between them
	// legitimately enters through the previous launcher. This exception grants
	// only exact pending recovery, never ordinary startup or network access.
	if l.generation == pending.Candidate.Generation {
		if state.Current != l.generation || identity(running) != pending.Candidate {
			return errors.New("running candidate identity changed")
		}
	} else if l.generation != pending.Previous.Generation || identity(running) != pending.Previous {
		return errors.New("running launcher is unrelated to pending recovery")
	}
	current, err := loadGenerationRecord(p, selectedLayout(l, state.Current))
	if err != nil {
		return err
	}
	if state.Current == pending.Candidate.Generation {
		if state.Previous != pending.Previous.Generation || identity(current) != pending.Candidate {
			return errors.New("candidate recovery identity changed")
		}
	} else if state.Current != pending.Previous.Generation || identity(current) != pending.Previous {
		return errors.New("pending update is unrelated to selected generation")
	}
	previous, err := loadGeneration(p, selectedLayout(l, pending.Previous.Generation))
	if err != nil {
		return err
	}
	defer previous.close()
	if previous.owned.Health != 1 || identity(previous) != pending.Previous {
		return errors.New("previous recovery generation changed")
	}
	if err := assertCurrent(p, current, state.Stock); err != nil {
		return err
	}
	again, err := p.read(l.root+"/update-pending.json", 4096)
	if err != nil || !bytes.Equal(data, again) {
		return errors.New("pending recovery changed before launch")
	}
	if err := invokeSupervisor(ctx, p, previous, "RecoverPending", pending.Transaction); err != nil {
		return err
	}
	if remains, e := pendingExists(p, l); e != nil || remains {
		return errors.New("pending update requires manual recovery")
	}
	after, err := readState(p, l)
	if err != nil || after.Current != previous.layout.generation || after.Stock != state.Stock {
		return errors.New("recovery did not retain the exact previous generation")
	}
	return ordinary(ctx, p, previous, state.Stock, mode)
}
func validPendingIdentity(identity pendingIdentity) bool {
	return generationPattern.MatchString(identity.Generation) && commitPattern.MatchString(identity.Commit) && hashPattern.MatchString(identity.PackageSHA256) && hashPattern.MatchString(identity.ExecutableSHA256)
}

type pendingIdentity struct {
	Generation       string `json:"generation"`
	Commit           string `json:"commit"`
	PackageSHA256    string `json:"packageSha256"`
	ExecutableSHA256 string `json:"executableSha256"`
}
type pendingRecord struct {
	Schema      int             `json:"schema"`
	Owner       string          `json:"owner"`
	Transaction string          `json:"transaction"`
	Candidate   pendingIdentity `json:"candidate"`
	Previous    pendingIdentity `json:"previous"`
	Attempts    int             `json:"attempts"`
	Session     string          `json:"session"`
}

func identity(g *generation) pendingIdentity {
	return pendingIdentity{Generation: g.layout.generation, Commit: g.owned.Commit, PackageSHA256: g.owned.PackageSHA256, ExecutableSHA256: g.managed["app/opencpn.exe"]}
}
func launchInstalled(ctx context.Context, p platform, previous *generation, stock stockRecord, policy releasepolicy.Policy) error {
	state, err := readState(p, previous.layout)
	if err != nil || state.Stock != stock || state.Current == previous.layout.generation || state.Previous != previous.layout.generation {
		return errors.New("installer did not publish the expected successor")
	}
	next, err := loadGeneration(p, selectedLayout(previous.layout, state.Current))
	if err != nil {
		return err
	}
	defer next.close()
	if next.owned.Health != 1 || next.owned.Version != policy.Version || next.owned.Commit != policy.Commit {
		return errors.New("installed successor differs from confirmed release")
	}
	data, err := p.read(previous.layout.root+"/update-pending.json", 4096)
	if err != nil {
		return errors.New("installer did not publish supervised pending identity")
	}
	var pending pendingRecord
	if err := decodeRecord(data, &pending, []string{"schema", "owner", "transaction", "candidate", "previous", "attempts", "session"}, false); err != nil || pending.Schema != 1 || pending.Owner != "SKAGER.UpdateStartup.1" || !generationPattern.MatchString(pending.Transaction) || pending.Attempts != 0 || pending.Session != "" || pending.Candidate != identity(next) || pending.Previous != identity(previous) {
		return errors.New("installed pending transaction differs from exact update identities")
	}
	if err := supervise(ctx, p, next, stock, "LaunchPending", pending.Transaction); err != nil {
		return err
	}
	if remains, e := pendingExists(p, next.layout); e != nil || remains {
		return errors.New("supervised update has unresolved recovery state")
	}
	state, err = readState(p, next.layout)
	if err != nil {
		return err
	}
	if state.Current == next.layout.generation {
		return nil
	} // Supervisor authenticated startup receipt.
	if state.Current == previous.layout.generation {
		return ordinary(ctx, p, previous, stock, "--xnav")
	}
	return errors.New("supervised update selected an unrelated generation")
}
