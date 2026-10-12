package brokerstore

import (
	"bytes"
	"context"
	"errors"

	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

type nativeBaselineGeneration struct {
	*nativeInstalledGeneration
	previous                               *nativeInstalledBaseline
	approved                               brokerregistration.Request
	registration, registrationMarker       *nativeBaselineFile
	registrationRaw, registrationMarkerRaw []byte
}

func (n *nativeStore) prepareBaselineGeneration(ctx context.Context, previous installedBaselineBackend, approved brokerregistration.Request) (baselineGenerationBackend, error) {
	p, ok := previous.(*nativeInstalledBaseline)
	if !ok || p == nil || p.store != n || p.record == nil || p.marker == nil {
		return nil, errors.New("actual same-store completed native baseline required")
	}
	request, err := brokerregistration.ParseRetainedApprovedRequest(approved.OriginalBytes(), approved.RequestSHA256)
	if err != nil || request.ClientSID != p.client || request.Installed.Generation != p.generation || request.RequestSHA256 != p.approved {
		return nil, errors.Join(errors.New("original native approved baseline binding differs"), err)
	}
	if err = n.check(); err != nil {
		return nil, err
	}
	sid, err := clientSID(request.ClientSID, n.broker)
	if err != nil {
		return nil, err
	}
	b, err := prepareNativeBaselineGeneration(ctx, n, p, sid, request)
	if err != nil {
		return nil, err
	}
	if err = b.openApplications(); err != nil {
		return nil, errors.Join(err, b.close())
	}
	return b, nil
}

// This lower helper remains separate solely for inert temporary native fixtures;
// production always checks actual principal/SID and opens fixed Program Files.
func prepareNativeBaselineGeneration(ctx context.Context, n *nativeStore, p *nativeInstalledBaseline, sid *windows.SID, approved brokerregistration.Request) (result *nativeBaselineGeneration, resultErr error) {
	if ctx == nil || p == nil || p.store != n || p.client != approved.ClientSID || p.generation != approved.Installed.Generation || p.approved != approved.RequestSHA256 || p.record == nil || p.marker == nil {
		return nil, errors.New("live native baseline approval required")
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := p.check(); err != nil {
		return nil, err
	}
	g, err := prepareNativeInstalledGeneration(n, nil, sid, p.client, p.generation)
	if err != nil {
		return nil, err
	}
	g.creationContext = ctx
	b := &nativeBaselineGeneration{nativeInstalledGeneration: g, previous: p, approved: approved}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, b.close())
		}
	}()
	rawMarker, err := g.hold(n.registrations, p.client+".once", maxRegistrationMarkerBytes)
	if err != nil {
		return nil, err
	}
	b.registrationMarker = g.held[len(g.held)-1].file
	raw, err := g.hold(n.registrations, p.client+".json", MaxRegistrationBytes)
	if err != nil {
		return nil, err
	}
	b.registration = g.held[len(g.held)-1].file
	request, err := validateBaselineRegistration(raw, rawMarker, p.client, p.approved)
	if err != nil || request.Installed != approved.Installed || !bytes.Equal(request.OriginalBytes(), approved.OriginalBytes()) {
		return nil, errors.Join(errors.New("native completed registration differs from independent baseline approval"), err)
	}
	b.registrationRaw = bytes.Clone(raw)
	b.registrationMarkerRaw = bytes.Clone(rawMarker)
	if err = g.check(); err != nil {
		return nil, err
	}
	return b, nil
}
func (b *nativeBaselineGeneration) baselineSource(members map[string]baselineMember) (map[string]diskIdentity, map[string]diskIdentity, generationBaselineOrigin, error) {
	empty := generationBaselineOrigin{}
	if err := b.creationContext.Err(); err != nil {
		return nil, nil, empty, err
	}
	if err := b.previous.check(); err != nil {
		return nil, nil, empty, err
	}
	if err := b.check(); err != nil {
		return nil, nil, empty, err
	}
	if err := b.checkHeldDigests(); err != nil {
		return nil, nil, empty, err
	}
	dirs, files, err := b.previous.identities(members)
	if err != nil {
		return nil, nil, empty, err
	}
	length := int64(uint64(b.previous.record.pin.info.FileSizeHigh)<<32 | uint64(b.previous.record.pin.info.FileSizeLow))
	if length < 1 || length > maxBaselineRecordBytes {
		return nil, nil, empty, errors.New("bounded native original completion required")
	}
	raw, err := readBaselineFile(b.creationContext, b.previous.record, length)
	if err != nil {
		return nil, nil, empty, err
	}
	if registrationDigest(raw) != b.previous.recordDigest {
		return nil, nil, empty, errors.New("original baseline completion changed")
	}
	marker, err := readBaselineFile(b.creationContext, b.previous.marker, b.previous.markerLength)
	if err != nil {
		return nil, nil, empty, err
	}
	digest, err := completedBaselineDigest(marker, b.client, b.stage, b.approved.RequestSHA256)
	if err != nil || digest != registrationDigest(raw) {
		return nil, nil, empty, errors.Join(errors.New("original native baseline completion unbound"), err)
	}
	origin := generationBaselineOrigin{ApprovedRequestSHA256: b.approved.RequestSHA256, Registration: bytes.Clone(b.registrationRaw), RegistrationMarker: bytes.Clone(b.registrationMarkerRaw), RegistrationIdentity: b.registration.identity, RegistrationMarkerIdentity: b.registrationMarker.identity, BaselineMarker: bytes.Clone(marker), BaselineMarkerIdentity: b.previous.marker.identity, BaselineCompletionIdentity: b.previous.record.identity}
	if !origin.RegistrationIdentity.valid() || !origin.RegistrationMarkerIdentity.valid() || !origin.BaselineMarkerIdentity.valid() || !origin.BaselineCompletionIdentity.valid() {
		return nil, nil, empty, errors.New("complete native original evidence IDs required")
	}
	return dirs, files, origin, nil
}
