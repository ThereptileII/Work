package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"path/filepath"
	"strconv"
	"strings"

	"golang.org/x/sys/windows"
)

const maxProtectedGenerations = 128

type generationParents struct {
	Schema      int
	ClientSID   string
	Directories map[string]diskIdentity
}
type generationHeldFile struct {
	file   *nativeBaselineFile
	length int64
	digest string
}
type nativeInstalledGeneration struct {
	creationContext                                 context.Context
	store                                           *nativeStore
	source                                          *nativeCandidate
	client, stage, origin                           string
	applications, clientRoot, generations, registry *nativePin
	tree                                            *nativeCandidate
	privateSD                                       *windows.SECURITY_DESCRIPTOR
	ancestors                                       []*nativePin
	pins                                            map[*nativePin]diskIdentity
	parentIDs                                       map[string]diskIdentity
	held                                            []generationHeldFile
	marker, record                                  *nativeBaselineFile
	markerBytes                                     []byte
	markerLength                                    int64
	members                                         map[string]*nativeBaselineFile
	siblings                                        map[string]*nativePin
	completed, closed                               bool
	readOnlyAttempted                               bool
	closeErr                                        error
}

func (n *nativeStore) prepareInstalledGeneration(previous installedBaselineBackend, source candidateBackend, acquisition *Acquisition, client, stage string) (generationBackend, error) {
	p, ok := previous.(*nativeInstalledBaseline)
	c, okc := source.(*nativeCandidate)
	if !ok || p == nil || p.store != n || !okc || c == nil || c.store != n || c.stageID != stage || c.client != client || p.client != client || p.record == nil || acquisition == nil {
		return nil, errors.New("retained native same-store baseline and candidate required")
	}
	a, oka := acquisition.backend.(*nativeAcquisition)
	if !oka || a == nil || a.candidate != c || !a.closed || a.closeErr != nil || a.failure != nil || a.index == nil || !a.index.complete || a.index.plan == nil || acquisition.plan == nil || *a.index.plan != *acquisition.plan {
		return nil, errors.New("actual completed native acquisition required")
	}
	for _, artifact := range acquisition.artifacts {
		f, ok := artifact.backend.(*nativeArtifact)
		if !ok || f == nil || f.acquisition != a || f.slot != artifact.slot || !f.indexed || !f.closed || f.closeErr != nil || f.custodyErr != nil || !f.identity.valid() || a.index.states[f.slot] != "deleted" || a.index.identities[f.slot] != f.identity {
			return nil, errors.New("native authenticated artifact custody differs")
		}
	}
	if err := n.check(); err != nil {
		return nil, err
	}
	sid, err := clientSID(client, n.broker)
	if err != nil {
		return nil, err
	}
	g, err := prepareNativeInstalledGeneration(n, c, sid, client, stage)
	if err != nil {
		return nil, err
	}
	if err = g.openApplications(); err != nil {
		return nil, errors.Join(err, g.close())
	}
	return g, nil
}

// Only the production factory above may supply actual identities. Native inert
// fixtures use this lower helper with a fresh temporary Applications directory.
func prepareNativeInstalledGeneration(n *nativeStore, c *nativeCandidate, sid *windows.SID, client, stage string) (*nativeInstalledGeneration, error) {
	if n.registrations == nil || !validHex(stage, 32) {
		return nil, errors.New("fixed generation namespace unavailable")
	}
	dir, err := descriptor(n.broker, sid, true)
	if err != nil {
		return nil, err
	}
	file, err := descriptor(n.broker, sid, false)
	if err != nil {
		return nil, err
	}
	private, err := descriptor(n.broker, nil, false)
	if err != nil {
		return nil, err
	}
	return &nativeInstalledGeneration{store: n, source: c, client: client, stage: stage, privateSD: private, pins: map[*nativePin]diskIdentity{}, members: map[string]*nativeBaselineFile{}, siblings: map[string]*nativePin{}, tree: &nativeCandidate{store: n, client: client, stageID: stage, directories: map[string]*nativePin{}, securityDir: dir, securityFile: file}}, nil
}
func (g *nativeInstalledGeneration) pin(p *nativePin) error {
	id, err := fileIdentity(p.handle)
	if err != nil {
		return err
	}
	g.pins[p] = id
	return nil
}
func (g *nativeInstalledGeneration) openApplications() error {
	base, err := windows.KnownFolderPath(windows.FOLDERID_ProgramFilesX86, windows.KF_FLAG_DEFAULT)
	if err != nil {
		return err
	}
	wanted := filepath.Join(base, "SKAGER", "Applications")
	if filepath.Clean(wanted) != wanted || len(wanted) < 4 || wanted[1] != ':' || wanted[2] != '\\' || strings.Contains(wanted[3:], ":") || !((wanted[0] >= 'A' && wanted[0] <= 'Z') || (wanted[0] >= 'a' && wanted[0] <= 'z')) {
		return errors.New("fixed native Program Files path required")
	}
	installer, _, _, err := windows.LookupSID("", `NT SERVICE\TrustedInstaller`)
	if err != nil {
		return err
	}
	policy := &ancestorPolicy{trusted: map[string]bool{"S-1-5-18": true, "S-1-5-32-544": true, installer.String(): true, g.store.broker.String(): true}}
	parentSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FRFX;;;AU)(A;OICI;FRFX;;;" + g.store.broker.String() + ")")
	if err != nil {
		return err
	}
	appSD, err := windows.SecurityDescriptorFromString("O:BAD:P(A;OICI;FA;;;SY)(A;OICI;FA;;;BA)(A;OICI;FRFX;;;AU)(A;OICI;FA;;;" + g.store.broker.String() + ")")
	if err != nil {
		return err
	}
	drive, err := openPin(wanted[:3], "", nil)
	if err != nil {
		return err
	}
	g.ancestors = append(g.ancestors, drive)
	drive.ancestor = policy
	if err = drive.check(); err != nil {
		return err
	}
	if err = g.pin(drive); err != nil {
		return err
	}
	parts, err := physicalParts(drive.name)
	if err != nil {
		return err
	}
	walk := strings.Join(parts[:3], `\`)
	volume, err := openPin(physical(walk), walk, nil)
	if err != nil {
		return err
	}
	g.ancestors = append(g.ancestors, volume)
	volume.ancestor = policy
	if err = volume.check(); err != nil {
		return err
	}
	if err = g.pin(volume); err != nil {
		return err
	}
	var serial, component, flags uint32
	fs := make([]uint16, 32)
	if err = windows.GetVolumeInformationByHandle(volume.handle, nil, 0, &serial, &component, &flags, &fs[0], uint32(len(fs))); err != nil || windows.UTF16ToString(fs) != "NTFS" || serial != volume.info.VolumeSerialNumber {
		return errors.New("launch view requires physical NTFS")
	}
	all := append(append([]string{}, parts[3:]...), strings.Split(wanted[3:], `\`)...)
	for i, part := range all {
		if !safeComponent(part) {
			return errors.New("ambiguous launch ancestry")
		}
		walk += `\` + part
		var sd *windows.SECURITY_DESCRIPTOR
		if i == len(all)-2 {
			sd = parentSD
		}
		if i == len(all)-1 {
			sd = appSD
		}
		p, err := openPin(physical(walk), walk, sd)
		if err != nil {
			return err
		}
		g.ancestors = append(g.ancestors, p)
		if sd == nil {
			p.ancestor = policy
		}
		if err = p.check(); err != nil {
			return err
		}
		if p.info.VolumeSerialNumber != serial {
			return errors.New("launch ancestry crossed volumes")
		}
		if err = g.pin(p); err != nil {
			return err
		}
		g.applications = p
	}
	g.applications.dos = wanted
	return g.check()
}
func (g *nativeInstalledGeneration) check() error {
	if g.closed {
		return ErrClosed
	}
	if err := g.store.check(); err != nil {
		return err
	}
	for p, id := range g.pins {
		if err := p.check(); err != nil {
			return err
		}
		actual, err := fileIdentity(p.handle)
		if err != nil || actual != id {
			return errors.Join(errors.New("generation full directory identity changed"), err)
		}
	}
	for _, h := range g.held {
		if err := h.file.check(h.length); err != nil {
			return err
		}
	}
	if g.marker != nil {
		if err := g.marker.check(g.markerLength); err != nil {
			return err
		}
		if g.markerBytes != nil {
			raw, err := readBaselineFile(context.Background(), g.marker, g.markerLength)
			if err != nil || !bytes.Equal(raw, g.markerBytes) {
				return errors.Join(errors.New("generation reservation changed"), err)
			}
		}
	}
	return nil
}

// Content is corroborated once per exact inventory boundary. Per-file checks
// use pinned identities and denying writer/delete sharing, avoiding quadratic
// reads of bounded but potentially large private completion records.
func (g *nativeInstalledGeneration) checkHeldDigests() error {
	for _, h := range g.held {
		if !validHex(h.digest, 64) {
			return errors.New("private creation digest unavailable")
		}
		raw, err := readBaselineFile(context.Background(), h.file, h.length)
		if err != nil {
			return err
		}
		if registrationDigest(raw) != h.digest {
			return errors.New("private original creation bytes changed")
		}
	}
	return nil
}
func generationMarker(client, stage, origin, digest string, done bool) []byte {
	status := "0"
	if done {
		status = "1"
	}
	if digest == "" {
		digest = strings.Repeat("0", 64)
	}
	return []byte("SKAGER-signed-generation-v1\n" + status + "\n" + client + "\n" + stage + "\n" + origin + "\n" + digest + "\n")
}
func generationMarkerDigest(raw []byte, client, stage, origin string) (string, error) {
	p := strings.Split(string(raw), "\n")
	if len(p) != 7 || p[0] != "SKAGER-signed-generation-v1" || p[1] != "1" || p[2] != client || p[3] != stage || p[4] != origin || !validHex(p[5], 64) || p[6] != "" || !bytes.Equal(raw, generationMarker(client, stage, origin, p[5], true)) {
		return "", errors.New("generation reservation incomplete or different")
	}
	return p[5], nil
}
func parentMarker(client, digest string, done bool) []byte {
	status := "0"
	if done {
		status = "1"
	}
	if digest == "" {
		digest = strings.Repeat("0", 64)
	}
	return []byte("SKAGER-generation-parents-v1\n" + status + "\n" + client + "\n" + digest + "\n")
}
func (g *nativeInstalledGeneration) hold(parent *nativePin, name string, limit int64) ([]byte, error) {
	f, err := baselineFile(parent, name, g.privateSD, false)
	if err != nil {
		return nil, err
	}
	length := int64(uint64(f.pin.info.FileSizeHigh)<<32 | uint64(f.pin.info.FileSizeLow))
	g.held = append(g.held, generationHeldFile{file: f, length: length})
	if length < 1 || length > limit {
		return nil, errors.New("bounded original private creation record required")
	}
	raw, err := readBaselineFile(g.creationContext, f, length)
	if err != nil {
		return nil, err
	}
	g.held[len(g.held)-1].digest = registrationDigest(raw)
	return raw, nil
}
func (g *nativeInstalledGeneration) parentIdentities() map[string]diskIdentity {
	ids := map[string]diskIdentity{"@applications": g.pins[g.applications], "@client": g.pins[g.clientRoot], "@generations": g.pins[g.generations], "@registry": g.pins[g.registry]}
	for i, p := range g.ancestors {
		ids["@ancestor/"+strconv.Itoa(i)] = g.pins[p]
	}
	return ids
}
func (g *nativeInstalledGeneration) openParent(parent *nativePin, component string, sd *windows.SECURITY_DESCRIPTOR) (*nativePin, error) {
	name := parent.name + `\` + component
	p, err := openPin(physical(name), name, sd)
	if err != nil {
		return nil, err
	}
	g.tree.pins = append(g.tree.pins, p)
	if p.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return nil, errors.New("generation parent crossed volume")
	}
	if err = g.pin(p); err != nil {
		return nil, err
	}
	return p, nil
}
func (g *nativeInstalledGeneration) freshParent(parent *nativePin, component string, sd *windows.SECURITY_DESCRIPTOR) (*nativePin, error) {
	if len(g.tree.pins) >= MaxFiles+maxProtectedGenerations+3 {
		return nil, errors.New("bounded generation directory inventory exhausted")
	}
	if g.creationContext == nil {
		return nil, errors.New("creation context required")
	}
	if err := g.creationContext.Err(); err != nil {
		return nil, err
	}
	old := g.tree.securityDir
	g.tree.securityDir = sd
	p, err := g.tree.newDirectory(parent, component)
	g.tree.securityDir = old
	if err != nil {
		return nil, err
	}
	if err = g.pin(p); err != nil {
		return nil, err
	}
	return p, nil
}
func (g *nativeInstalledGeneration) parents(ctx context.Context) error {
	if g.applications == nil {
		return errors.New("provisioned Applications owner required")
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	markerName := g.client + ".launch-parents.once"
	recordName := g.client + ".launch-parents.json"
	registryName := g.client + ".generations"
	// CREATE_NEW distinguishes original protected birth from authenticated reopen.
	marker, err := baselineFile(g.store.registrations, markerName, g.privateSD, true)
	if err == nil {
		g.held = append(g.held, generationHeldFile{file: marker})
		started := parentMarker(g.client, "", false)
		if err = writeRegistrationBytes(ctx, marker, 0, started, g.checkWithoutHeld); err != nil {
			return err
		}
		g.held[len(g.held)-1].length = int64(len(started))
		privateDir, err := descriptor(g.store.broker, nil, true)
		if err != nil {
			return err
		}
		if g.registry, err = g.freshParent(g.store.registrations, registryName, privateDir); err != nil {
			return err
		}
		if g.clientRoot, err = g.freshParent(g.applications, g.client, g.tree.securityDir); err != nil {
			return err
		}
		if g.generations, err = g.freshParent(g.clientRoot, "generations", g.tree.securityDir); err != nil {
			return err
		}
		g.parentIDs = g.parentIdentities()
		raw, err := json.Marshal(generationParents{1, g.client, g.parentIDs})
		if err != nil {
			return err
		}
		if err := ctx.Err(); err != nil {
			return err
		}
		record, err := baselineFile(g.store.registrations, recordName, g.privateSD, true)
		if err != nil {
			return err
		}
		g.held = append(g.held, generationHeldFile{file: record})
		if err = writeRegistrationBytes(ctx, record, 0, raw, g.checkWithoutHeld); err != nil {
			return err
		}
		g.held[len(g.held)-1].length = int64(len(raw))
		g.held[len(g.held)-1].digest = registrationDigest(raw)
		completed := parentMarker(g.client, registrationDigest(raw), true)
		if err = publishRegistrationCompletion(ctx, marker, int64(len(started)), completed, g.checkWithoutHeld); err != nil {
			if errors.Is(err, ErrRegistrationUncertain) {
				return errors.Join(ErrInstalledGenerationUncertain, err)
			}
			return err
		}
		g.held[len(g.held)-2].digest = registrationDigest(completed)
	} else {
		if !errors.Is(err, windows.ERROR_FILE_EXISTS) && !errors.Is(err, windows.ERROR_ALREADY_EXISTS) {
			return err
		}
		rawMarker, err := g.hold(g.store.registrations, markerName, 1024)
		if err != nil {
			return err
		}
		raw, err := g.hold(g.store.registrations, recordName, 64<<10)
		if err != nil {
			return err
		}
		var record generationParents
		if err = json.Unmarshal(raw, &record); err != nil {
			return err
		}
		canonical, err := json.Marshal(record)
		if err != nil || !bytes.Equal(canonical, raw) || record.Schema != 1 || record.ClientSID != g.client || !bytes.Equal(rawMarker, parentMarker(g.client, registrationDigest(raw), true)) {
			return errors.New("original completed parent creation evidence required")
		}
		privateDir, err := descriptor(g.store.broker, nil, true)
		if err != nil {
			return err
		}
		if g.registry, err = g.openParent(g.store.registrations, registryName, privateDir); err != nil {
			return err
		}
		if g.clientRoot, err = g.openParent(g.applications, g.client, g.tree.securityDir); err != nil {
			return err
		}
		if g.generations, err = g.openParent(g.clientRoot, "generations", g.tree.securityDir); err != nil {
			return err
		}
		g.parentIDs = g.parentIdentities()
		if !sameGenerationIDs(record.Directories, g.parentIDs) {
			return errors.New("parent historical full identities differ")
		}
	}
	return g.loadSiblings()
}
func (g *nativeInstalledGeneration) checkWithoutHeld() error {
	held, marker := g.held, g.marker
	g.held = nil
	g.marker = nil
	err := g.check()
	g.held = held
	g.marker = marker
	return err
}
func (g *nativeInstalledGeneration) loadSiblings() error {
	entries, err := boundedEntries(g.registry, maxProtectedGenerations*2+1)
	if err != nil {
		return err
	}
	if len(entries)%2 != 0 || len(entries) > maxProtectedGenerations*2 {
		return errors.New("generation registry partial or exhausted")
	}
	names := map[string]bool{}
	for _, entry := range entries {
		names[entry.Name()] = true
	}
	for _, entry := range entries {
		if err := g.creationContext.Err(); err != nil {
			return err
		}
		name := entry.Name()
		if !strings.HasSuffix(name, ".once") {
			continue
		}
		stage := strings.TrimSuffix(name, ".once")
		if !validHex(stage, 32) || !names[stage+".json"] || stage == g.stage {
			return errors.New("generation registry alias, partial or reused ID")
		}
		marker, err := g.hold(g.registry, name, 1024)
		if err != nil {
			return err
		}
		raw, err := g.hold(g.registry, stage+".json", maxGenerationRecordBytes)
		if err != nil {
			return err
		}
		var record generationRecord
		if err = json.Unmarshal(raw, &record); err != nil {
			return err
		}
		canonical, err := json.Marshal(record)
		if err != nil || !bytes.Equal(raw, canonical) || record.Schema != 1 || record.Purpose != "signed-candidate-generation" || record.ClientSID != g.client || record.Generation != stage || !record.Acquisition.valid() || registrationDigest(record.PreviousCompletion) != record.PreviousCompletionSHA256 {
			return errors.New("generation original completion invalid")
		}
		originRecord := record
		originRecord.Directories = nil
		originRecord.Members = map[string]baselineMember{}
		originRaw, err := json.Marshal(originRecord)
		if err != nil {
			return err
		}
		digest, err := generationMarkerDigest(marker, g.client, stage, registrationDigest(originRaw))
		if err != nil || digest != registrationDigest(raw) {
			return errors.Join(errors.New("generation original completion unbound"), err)
		}
		for key, id := range g.parentIDs {
			if record.Directories[key] != id {
				return errors.New("generation parent creation lineage differs")
			}
		}
		p, err := g.openParent(g.generations, stage, g.tree.securityDir)
		if err != nil {
			return err
		}
		if g.pins[p] != record.Directories[""] {
			return errors.New("generation sibling full root identity differs")
		}
		g.siblings[stage] = p
	}
	if len(g.siblings)*2 != len(entries) {
		return errors.New("unknown generation private namespace")
	}
	return g.parentInventory(false)
}
func (g *nativeInstalledGeneration) reserve(ctx context.Context, origin []byte) error {
	g.creationContext = ctx
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := g.check(); err != nil {
		return err
	}
	if g.marker != nil {
		return errors.New("generation reservation already consumed")
	}
	if err := g.parents(ctx); err != nil {
		return err
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	if len(g.siblings) >= maxProtectedGenerations {
		return errors.New("generation registry exhausted")
	}
	g.origin = registrationDigest(origin)
	var err error
	g.marker, err = baselineFile(g.registry, g.stage+".once", g.privateSD, true)
	if err != nil {
		return err
	}
	raw := generationMarker(g.client, g.stage, g.origin, "", false)
	if err = writeRegistrationBytes(ctx, g.marker, 0, raw, g.checkWithoutHeld); err != nil {
		return err
	}
	g.markerLength = int64(len(raw))
	g.markerBytes = bytes.Clone(raw)
	return g.check()
}
func (g *nativeInstalledGeneration) createTree() error {
	if g.marker == nil || g.markerLength == 0 || g.tree.directories[""] != nil {
		return errors.New("fresh durable generation reservation required")
	}
	if err := g.check(); err != nil {
		return err
	}
	root, err := g.freshParent(g.generations, g.stage, g.tree.securityDir)
	if err != nil {
		return err
	}
	g.tree.directories[""] = root
	if g.applications.dos != "" {
		root.dos = filepath.Join(g.applications.dos, g.client, "generations", g.stage)
	}
	return g.check()
}
func (g *nativeInstalledGeneration) create(relative string) (candidateFile, error) {
	if err := g.check(); err != nil {
		return nil, err
	}
	if g.record != nil || g.tree.directories[""] == nil {
		return nil, errors.New("fresh uncompleted generation required")
	}
	if !baselineEvidenceName(relative) {
		if err := memberPath(relative); err != nil {
			return nil, err
		}
	}
	if _, exists := g.members[relative]; exists {
		return nil, errors.New("generation member already exists")
	}
	parts := strings.Split(relative, "/")
	parent := g.tree.directories[""]
	for i, part := range parts[:len(parts)-1] {
		key := strings.Join(parts[:i+1], "/")
		p := g.tree.directories[key]
		if p == nil {
			var err error
			p, err = g.freshParent(parent, part, g.tree.securityDir)
			if err != nil {
				return nil, err
			}
			g.tree.directories[key] = p
		}
		parent = p
	}
	if err := g.creationContext.Err(); err != nil {
		return nil, err
	}
	f, err := baselineFile(parent, parts[len(parts)-1], g.tree.securityFile, true)
	if err != nil {
		return nil, err
	}
	g.members[relative] = f
	return f, nil
}
func exactGenerationEntries(pin *nativePin, names map[string]bool) error {
	entries, err := boundedEntries(pin, len(names)+1)
	if err != nil {
		return err
	}
	if len(entries) != len(names) {
		return errors.New("generation namespace has missing or additional entries")
	}
	for _, entry := range entries {
		if !names[entry.Name()] {
			return errors.New("generation namespace alias or unknown entry")
		}
	}
	return nil
}
func (g *nativeInstalledGeneration) parentInventory(current bool) error {
	public := map[string]bool{}
	private := map[string]bool{}
	for stage := range g.siblings {
		public[stage] = true
		private[stage+".once"] = true
		private[stage+".json"] = true
	}
	if current {
		public[g.stage] = true
	}
	if g.marker != nil {
		private[g.stage+".once"] = true
	}
	if g.record != nil {
		private[g.stage+".json"] = true
	}
	if err := exactGenerationEntries(g.clientRoot, map[string]bool{"generations": true}); err != nil {
		return err
	}
	if err := exactGenerationEntries(g.generations, public); err != nil {
		return err
	}
	return exactGenerationEntries(g.registry, private)
}
func (g *nativeInstalledGeneration) inventory(names []string, complete bool) error {
	if err := g.checkHeldDigests(); err != nil {
		return err
	}
	if err := g.check(); err != nil {
		return err
	}
	if complete != g.completed {
		return errors.New("generation completion state differs")
	}
	if err := g.parentInventory(true); err != nil {
		return err
	}
	expected := map[string]map[string]bool{}
	for name := range g.tree.directories {
		expected[name] = map[string]bool{}
	}
	for name := range g.tree.directories {
		if name != "" {
			parent, child := splitBaselineName(name)
			if expected[parent] == nil {
				return errors.New("generation directory parent missing")
			}
			expected[parent][child] = true
		}
	}
	for _, name := range names {
		parent, child := splitBaselineName(name)
		if expected[parent] == nil || expected[parent][child] {
			return errors.New("generation member parent/collision")
		}
		expected[parent][child] = true
	}
	for name, wanted := range expected {
		if err := exactGenerationEntries(g.tree.directories[name], wanted); err != nil {
			return err
		}
	}
	return g.check()
}
func (g *nativeInstalledGeneration) sourceIdentities(members map[string]fileRecord) (map[string]diskIdentity, map[string]diskIdentity, error) {
	if err := g.source.check(); err != nil {
		return nil, nil, err
	}
	dirs := map[string]diskIdentity{}
	files := map[string]diskIdentity{}
	for name, p := range g.source.directories {
		id, err := fileIdentity(p.handle)
		if err != nil {
			return nil, nil, err
		}
		dirs[name] = id
	}
	for name, m := range members {
		f, ok := m.file.(*nativeFile)
		if !ok || f == nil {
			return nil, nil, errors.New("retained native recovery member required")
		}
		if err := f.check(m.length); err != nil {
			return nil, nil, err
		}
		id, err := fileIdentity(f.pin.handle)
		if err != nil {
			return nil, nil, err
		}
		files[name] = id
	}
	return dirs, files, nil
}
func (g *nativeInstalledGeneration) identities(members map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, error) {
	if err := g.check(); err != nil {
		return nil, nil, err
	}
	dirs := map[string]diskIdentity{}
	files := map[string]diskIdentity{}
	for name, id := range g.parentIDs {
		dirs[name] = id
	}
	for name, p := range g.tree.directories {
		dirs[name] = g.pins[p]
	}
	for name, m := range members {
		f := g.members[name]
		if f == nil {
			return nil, nil, errors.New("generation member identity absent")
		}
		if err := f.check(m.Length); err != nil {
			return nil, nil, err
		}
		files[name] = f.identity
	}
	return dirs, files, nil
}
func (g *nativeInstalledGeneration) publish(ctx context.Context, raw []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := g.check(); err != nil {
		return err
	}
	if g.record != nil {
		return errors.New("generation completion publication already attempted")
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	var err error
	g.record, err = baselineFile(g.registry, g.stage+".json", g.privateSD, true)
	if err != nil {
		return err
	}
	if err = writeRegistrationBytes(ctx, g.record, 0, raw, g.check); err != nil {
		return err
	}
	g.held = append(g.held, generationHeldFile{g.record, int64(len(raw)), registrationDigest(raw)})
	names := make([]string, 0, len(g.members))
	for name := range g.members {
		names = append(names, name)
	}
	if err = g.inventory(names, false); err != nil {
		return err
	}
	completed := generationMarker(g.client, g.stage, g.origin, registrationDigest(raw), true)
	started := g.markerBytes
	g.markerBytes = nil
	if err = publishRegistrationCompletion(ctx, g.marker, g.markerLength, completed, g.check); err != nil {
		g.markerBytes = started
		if errors.Is(err, ErrRegistrationUncertain) {
			return errors.Join(ErrInstalledGenerationUncertain, err)
		}
		return err
	}
	g.markerBytes = bytes.Clone(completed)
	g.completed = true
	return nil
}
func (g *nativeInstalledGeneration) close() error {
	if g.closed {
		return g.closeErr
	}
	g.closed = true
	seen := map[*nativeBaselineFile]bool{}
	for _, held := range g.held {
		if held.file != nil && !seen[held.file] {
			g.closeErr = errors.Join(g.closeErr, held.file.Close())
			seen[held.file] = true
		}
	}
	for _, f := range []*nativeBaselineFile{g.marker, g.record} {
		if f != nil && !seen[f] {
			g.closeErr = errors.Join(g.closeErr, f.Close())
			seen[f] = true
		}
	}
	g.closeErr = errors.Join(g.closeErr, g.tree.close())
	// Parent and sibling pins belong to tree.pins; ancestry belongs only here.
	for i := len(g.ancestors) - 1; i >= 0; i-- {
		g.closeErr = errors.Join(g.closeErr, windows.CloseHandle(g.ancestors[i].handle))
	}
	return g.closeErr
}
