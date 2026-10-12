package brokerstore

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"path/filepath"
	"sort"
	"strings"

	"golang.org/x/sys/windows"
)

type nativeReopenedGeneration struct {
	*nativeInstalledGeneration
	baseline    *nativeInstalledBaseline
	closeResult error
	closeDone   bool
}

func (n *nativeStore) openReadOnlyGeneration(ctx context.Context, client, stage string) (generationReopenBackend, generationReopenEvidence, error) {
	if err := n.check(); err != nil {
		return nil, generationReopenEvidence{}, err
	}
	sid, err := clientSID(client, n.broker)
	if err != nil {
		return nil, generationReopenEvidence{}, err
	}
	g, err := prepareNativeInstalledGeneration(n, nil, sid, client, stage)
	if err != nil {
		return nil, generationReopenEvidence{}, err
	}
	if err = g.openApplications(); err != nil {
		return nil, generationReopenEvidence{}, errors.Join(err, g.close())
	}
	return openNativeReadOnlyGeneration(ctx, g, sid)
}

// Inert temporary fixtures supply only the already pinned Applications root and
// test principal. Production first resolves the actual account and fixed root.
// Every open below is existing-only; absence never reserves or repairs anything.
func openNativeReadOnlyGeneration(ctx context.Context, g *nativeInstalledGeneration, sid *windows.SID) (result generationReopenBackend, e generationReopenEvidence, resultErr error) {
	b := &nativeReopenedGeneration{nativeInstalledGeneration: g}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, b.close())
		}
	}()
	if ctx == nil || g.applications == nil {
		return nil, e, errors.New("live native fixed generation reopen context required")
	}
	g.creationContext = ctx
	if err := ctx.Err(); err != nil {
		return nil, e, err
	}
	marker, err := g.hold(g.store.registrations, g.client+".once", maxRegistrationMarkerBytes)
	if err != nil {
		return nil, e, err
	}
	markerFile := g.held[len(g.held)-1].file
	raw, err := g.hold(g.store.registrations, g.client+".json", MaxRegistrationBytes)
	if err != nil {
		return nil, e, err
	}
	registrationFile := g.held[len(g.held)-1].file
	var envelope baselineRegistrationEnvelope
	if json.Unmarshal(raw, &envelope) != nil {
		return nil, e, errors.New("original protected registration missing")
	}
	request, err := validateBaselineRegistration(raw, marker, g.client, envelope.ApprovedRequestSHA256)
	if err != nil {
		return nil, e, err
	}
	p, err := prepareNativeInstalledBaseline(g.store, sid, g.client, request.Installed.Generation, request.RequestSHA256)
	if err != nil {
		return nil, e, err
	}
	b.baseline = p
	_, baselineRaw, err := openNativeInstalledBaseline(ctx, p)
	if err != nil {
		return nil, e, err
	}
	e.lineage = generationBaselineOrigin{ApprovedRequestSHA256: request.RequestSHA256, Registration: bytes.Clone(raw), RegistrationMarker: bytes.Clone(marker), RegistrationIdentity: registrationFile.identity, RegistrationMarkerIdentity: markerFile.identity, BaselineMarker: bytes.Clone(p.markerBytes), BaselineMarkerIdentity: p.marker.identity, BaselineCompletionIdentity: p.record.identity}
	e.baselineCompletion = bytes.Clone(baselineRaw)
	// Keep the original private directory identity pins, but transfer the two
	// readonly files to the generation's single digest/lifetime owner. Microchecks
	// remain metadata-only; full original bytes are rechecked per inventory.
	g.held = append(g.held, generationHeldFile{p.marker, p.markerLength, registrationDigest(p.markerBytes)}, generationHeldFile{p.record, int64(len(baselineRaw)), registrationDigest(baselineRaw)})
	p.marker = nil
	p.record = nil
	var baseline baselineRecord
	if json.Unmarshal(baselineRaw, &baseline) != nil || baseline.Directories["@user"] != p.pins[p.user] || baseline.Directories["@installed"] != p.pins[p.installed] || baseline.Directories[""] != p.pins[p.tree.directories[""]] {
		return nil, e, errors.New("original baseline physical parent/root lineage differs")
	}
	if err = g.openCompletedParents(); err != nil {
		return nil, e, err
	}
	ownMarker, err := g.hold(g.registry, g.stage+".once", 1024)
	if err != nil {
		return nil, e, err
	}
	g.marker = g.held[len(g.held)-1].file
	ownRaw, err := g.hold(g.registry, g.stage+".json", maxGenerationRecordBytes)
	if err != nil {
		return nil, e, err
	}
	g.record = g.held[len(g.held)-1].file
	e.completion = bytes.Clone(ownRaw)
	record, err := validateGenerationReopenEvidence(e, g.client, g.stage)
	if err != nil {
		return nil, e, err
	}
	origin := record
	origin.Directories = nil
	origin.Members = map[string]baselineMember{}
	originRaw, err := json.Marshal(origin)
	if err != nil {
		return nil, e, err
	}
	g.origin = registrationDigest(originRaw)
	digest, err := generationMarkerDigest(ownMarker, g.client, g.stage, g.origin)
	if err != nil || digest != registrationDigest(ownRaw) {
		return nil, e, errors.Join(errors.New("original generation completion not bound to completed marker"), err)
	}
	g.markerBytes = bytes.Clone(ownMarker)
	g.markerLength = int64(len(ownMarker))
	for name, id := range g.parentIDs {
		if record.Directories[name] != id {
			return nil, e, errors.New("original generation physical parent lineage differs")
		}
	}
	root, err := g.openParent(g.generations, g.stage, g.tree.securityDir)
	if err != nil {
		return nil, e, err
	}
	g.tree.directories[""] = root
	if g.pins[root] != record.Directories[""] {
		return nil, e, errors.New("original generation full root identity differs")
	}
	if g.applications.dos != "" {
		root.dos = filepath.Join(g.applications.dos, g.client, "generations", g.stage)
	}
	if err = g.openCompletedSiblings(e); err != nil {
		return nil, e, err
	}
	g.completed = true
	g.readOnlyAttempted = true
	if err = ctx.Err(); err != nil {
		return nil, e, err
	}
	return b, e, b.check()
}

// Separate from parents(): reopening must not CREATE_NEW even a reservation.
func (g *nativeInstalledGeneration) openCompletedParents() error {
	marker, err := g.hold(g.store.registrations, g.client+".launch-parents.once", 1024)
	if err != nil {
		return err
	}
	raw, err := g.hold(g.store.registrations, g.client+".launch-parents.json", 64<<10)
	if err != nil {
		return err
	}
	var record generationParents
	if json.Unmarshal(raw, &record) != nil {
		return errors.New("original completed launch parents required")
	}
	canonical, err := json.Marshal(record)
	if err != nil || !bytes.Equal(canonical, raw) || record.Schema != 1 || record.ClientSID != g.client || !bytes.Equal(marker, parentMarker(g.client, registrationDigest(raw), true)) {
		return errors.New("exact original completed launch parents required")
	}
	privateDir, err := descriptor(g.store.broker, nil, true)
	if err != nil {
		return err
	}
	if g.registry, err = g.openParent(g.store.registrations, g.client+".generations", privateDir); err != nil {
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
		return errors.New("original parent full physical identities differ")
	}
	return nil
}
func (g *nativeInstalledGeneration) openCompletedSiblings(e generationReopenEvidence) error {
	entries, err := boundedEntries(g.registry, maxProtectedGenerations*2+1)
	if err != nil {
		return err
	}
	if len(entries) < 2 || len(entries)%2 != 0 || len(entries) > maxProtectedGenerations*2 {
		return errors.New("bounded complete generation registry required")
	}
	names := map[string]bool{}
	for _, entry := range entries {
		names[entry.Name()] = true
	}
	if !names[g.stage+".once"] || !names[g.stage+".json"] {
		return errors.New("current original generation records missing")
	}
	for _, entry := range entries {
		if err := g.creationContext.Err(); err != nil {
			return err
		}
		if !strings.HasSuffix(entry.Name(), ".once") {
			continue
		}
		stage := strings.TrimSuffix(entry.Name(), ".once")
		if !validHex(stage, 32) || !names[stage+".json"] {
			return errors.New("generation history partial or aliased")
		}
		if stage == g.stage {
			continue
		}
		raw, err := g.loadSibling(stage)
		if err != nil {
			return err
		}
		sibling := e
		sibling.completion = raw
		if _, err = validateGenerationReopenEvidence(sibling, g.client, stage); err != nil {
			return err
		}
	}
	if (len(g.siblings)+1)*2 != len(entries) {
		return errors.New("unknown private generation namespace")
	}
	return g.parentInventory(true)
}
func (b *nativeReopenedGeneration) check() error {
	if err := b.nativeInstalledGeneration.check(); err != nil {
		return err
	}
	if b.baseline == nil {
		return errors.New("original baseline directory custody missing")
	}
	return b.baseline.check()
}
func (b *nativeReopenedGeneration) inventory(names []string, complete bool) error {
	if err := b.check(); err != nil {
		return err
	}
	if err := b.nativeInstalledGeneration.inventory(names, complete); err != nil {
		return err
	}
	return b.check()
}
func (b *nativeReopenedGeneration) close() error {
	if b.closeDone {
		return b.closeResult
	}
	b.closeDone = true
	b.closeResult = b.nativeInstalledGeneration.close()
	if b.baseline != nil {
		b.closeResult = errors.Join(b.closeResult, b.baseline.close())
	}
	return b.closeResult
}
func (b *nativeReopenedGeneration) openReadOnlyMembers(ctx context.Context, members map[string]baselineMember, dirs map[string]diskIdentity) (result map[string]candidateFile, resultErr error) {
	result = map[string]candidateFile{}
	for name, id := range b.parentIDs {
		if dirs[name] != id {
			return result, errors.New("original generation parent identities differ")
		}
	}
	names := []string{}
	for name := range dirs {
		if strings.HasPrefix(name, "@") {
			if _, ok := b.parentIDs[name]; !ok {
				return result, errors.New("unknown generation parent identity")
			}
			continue
		}
		if name != "" {
			names = append(names, name)
		}
	}
	sort.Strings(names)
	for _, name := range names {
		if err := ctx.Err(); err != nil {
			return result, err
		}
		parent, component := splitBaselineName(name)
		if b.tree.directories[parent] == nil {
			return result, errors.New("original directory parent absent")
		}
		p, err := b.openParent(b.tree.directories[parent], component, b.tree.securityDir)
		if err != nil {
			return result, err
		}
		b.tree.directories[name] = p
		if b.pins[p] != dirs[name] {
			return result, errors.New("original generation full directory identity differs")
		}
	}
	names = names[:0]
	for name := range members {
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		if err := ctx.Err(); err != nil {
			return result, err
		}
		parent, component := splitBaselineName(name)
		if b.tree.directories[parent] == nil {
			return result, errors.New("original member parent absent")
		}
		f, err := baselineFile(b.tree.directories[parent], component, b.tree.securityFile, false)
		if err != nil {
			return result, err
		}
		result[name] = &generationReadOnlyFile{candidateFile: f, reader: f.File}
		b.members[name] = f
		if f.identity != members[name].Identity {
			return result, errors.New("original generation full member identity differs")
		}
		if err = f.check(members[name].Length); err != nil {
			return result, err
		}
	}
	return result, b.inventory(names, true)
}
