//go:build windows

package brokerenroll

import (
	"context"
	"encoding/json"
	"errors"
	"path/filepath"

	"example.com/opennav-update-verifier/brokerregistration"
	"golang.org/x/sys/windows"
)

const startupBindingName = "shortcut-publication-complete.json"

func startupDirectoryIdentity(pin *enrollmentPin) (startupDirectoryBinding, error) {
	if pin == nil {
		return startupDirectoryBinding{}, errors.New("original native startup owner required")
	}
	if err := pin.check(); err != nil {
		return startupDirectoryBinding{}, err
	}
	// Existing menu folders have policy-checked native handles but no expected
	// descriptor pointer. Measure the actual descriptor through that handle.
	descriptor, err := windows.GetSecurityInfo(pin.handle, windows.SE_FILE_OBJECT, windows.OWNER_SECURITY_INFORMATION|windows.DACL_SECURITY_INFORMATION)
	if err != nil {
		return startupDirectoryBinding{}, err
	}
	value := startupDirectoryBinding{Volume: pin.id.Volume, FileID: pin.id.ID,
		Creation:         uint64(pin.info.CreationTime.HighDateTime)<<32 | uint64(pin.info.CreationTime.LowDateTime),
		DescriptorSHA256: digest([]byte(descriptor.String()))}
	if !validStartupDirectory(value) {
		return startupDirectoryBinding{}, errors.New("complete native startup identity required")
	}
	if err := pin.check(); err != nil {
		return startupDirectoryBinding{}, err
	}
	return value, nil
}

func startupFileIdentity(ctx context.Context, pin *enrollmentPin, sha string) (startupFileBinding, error) {
	if pin == nil || !pin.file {
		return startupFileBinding{}, errors.New("original retained startup file required")
	}
	if err := startupHashPin(ctx, pin, sha); err != nil {
		return startupFileBinding{}, err
	}
	identity, err := startupDirectoryIdentity(pin)
	if err != nil {
		return startupFileBinding{}, err
	}
	return startupFileBinding{startupDirectoryBinding: identity, Bytes: pin.length, SHA256: sha}, nil
}

// This is called only by the original publisher AFTER all three moves. The
// pre-move publication.json remains unchanged. Failed/partial publication never
// calls this writer; a failed completion write remains uncertain and is retained.
// The explicit operator path still needs separate independent approval. The
// original installation transaction may instead preserve this exact publisher's
// completion in its machine-protected installed-maintenance record. Both paths
// still require retained native/cold-state authority before restoration.
func completeStartupPublicationBinding(ctx context.Context, owner *nativeStartupPublication, menu, recovery *enrollmentPin, client *brokerregistration.Request, sd *windows.SECURITY_DESCRIPTOR) error {
	if ctx == nil || ctx.Err() != nil || owner == nil || owner.files == nil || client == nil || menu == nil || recovery == nil || menu.file || recovery.file || sd == nil || owner.binding != nil {
		return errors.New("fresh original completed shortcut publication required")
	}
	request, err := brokerregistration.ParseRetainedApprovedRequest(client.OriginalBytes(), client.RequestSHA256)
	if err != nil || owner.clientSID != request.ClientSID || len(owner.additional) != 2 || owner.mode != startupNormal || owner.additional[0].mode != startupLegacy || owner.additional[1].mode != startupSafe {
		return errors.New("original approved startup client and three owners required")
	}
	if err := owner.files.Check(); err != nil {
		return err
	}
	if err := startupHashPin(ctx, owner.manifest, owner.manifestSHA); err != nil {
		return err
	}
	record := startupBindingRecord{Schema: 1, Purpose: startupBindingPurpose, ClientSID: request.ClientSID,
		ClientRequestSHA256: request.RequestSHA256, Generation: request.Installed.Generation,
		Commit: request.Installed.Commit, PackageSHA256: request.Installed.PackageSHA256,
		PrepublicationSHA256: owner.manifestSHA}
	if record.Menu, err = startupDirectoryIdentity(menu); err != nil {
		return err
	}
	if record.Recovery, err = startupDirectoryIdentity(recovery); err != nil {
		return err
	}
	if record.Launcher, err = startupFileIdentity(ctx, owner.image, owner.imageSHA); err != nil {
		return err
	}
	for i, child := range append([]*nativeStartupPublication{owner}, owner.additional...) {
		spec, _ := startupShortcutMode(i).spec()
		if child.files != owner.files || child.clientSID != owner.clientSID || child.link == nil ||
			child.link.physical != menu.physical+`\`+spec.name || child.link.dos != filepath.Join(menu.dos, spec.name) {
			return errors.New("all original shortcuts must be published before completion")
		}
		record.Links[i].Name = spec.name
		if record.Links[i].New, err = startupFileIdentity(ctx, child.link, child.linkSHA); err != nil {
			return err
		}
		if child.original != nil {
			if child.original.physical != recovery.physical+`\`+spec.name || child.original.dos != filepath.Join(recovery.dos, spec.name) {
				return errors.New("only the same original fixed-role backup may complete")
			}
			identity, err := startupFileIdentity(ctx, child.original, child.originalSHA)
			if err != nil {
				return err
			}
			record.Links[i].Original = &identity
		}
	}
	if err := validateStartupBinding(record, request); err != nil {
		return err
	}
	raw, err := json.Marshal(record)
	if err != nil || len(raw) > maxStartupBindingBytes {
		return errors.New("bounded original startup binding required")
	}
	// CREATE_NEW, Sync, original-ID readback and retained read-only custody.
	// No existing completion file can be adopted, overwritten or repaired.
	pin, err := startupWriteFile(ctx, owner.files, recovery, startupBindingName, raw, sd, false)
	if err != nil {
		return errors.Join(ErrStartupUncertain, err)
	}
	owner.binding, owner.bindingSHA = pin, digest(raw)
	if err := owner.files.Check(); err != nil {
		return errors.Join(ErrStartupUncertain, err)
	}
	return startupHashPin(ctx, owner.binding, owner.bindingSHA)
}
