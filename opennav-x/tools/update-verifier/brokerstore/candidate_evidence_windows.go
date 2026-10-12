package brokerstore

import (
	"errors"

	"golang.org/x/sys/windows"
)

// checkRecoveryEvidenceResidue recognizes only the two fixed inert original
// sidecars for acquisition reconciliation. This is not completion/adoption or
// semantic evidence validation; incomplete candidates keep their reservation.
func (c *nativeCandidate) checkRecoveryEvidenceResidue(name string) (resultErr error) {
	parent := c.directories[""]
	if parent == nil {
		return errors.New("candidate root unavailable")
	}
	if name == "FILE_SHA256.json" {
		return c.checkRecoveryEvidenceFile(parent, name)
	}
	if name != "installer" {
		return errors.New("unknown candidate evidence namespace")
	}
	physicalName := parent.name + `\installer`
	dir, err := openPin(physical(physicalName), physicalName, c.securityDir)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, windows.CloseHandle(dir.handle)) }()
	if dir.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return errors.New("candidate evidence directory crossed volumes")
	}
	entries, err := boundedEntries(dir, 1)
	if err != nil {
		return err
	}
	// An empty directory can remain after exclusive creation failed. It is never
	// completed or adopted here. Only package.json is a recognized file residue.
	for _, entry := range entries {
		if entry.Name() != "package.json" {
			return errors.New("unknown installer evidence residue")
		}
		if err := c.checkRecoveryEvidenceFile(dir, "package.json"); err != nil {
			return err
		}
	}
	return c.check()
}

func (c *nativeCandidate) checkRecoveryEvidenceFile(parent *nativePin, name string) (resultErr error) {
	file, err := openAcquisitionFile(parent, name, c.securityFile, windows.GENERIC_READ|windows.READ_CONTROL, windows.FILE_SHARE_READ, windows.OPEN_EXISTING)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, file.Close()) }()
	length := int64(uint64(file.pin.info.FileSizeHigh)<<32 | uint64(file.pin.info.FileSizeLow))
	if length < 0 || length > maxCandidateEvidenceBytes {
		return errors.New("candidate evidence residue exceeds bound")
	}
	if err := file.check(length); err != nil {
		return err
	}
	return c.check()
}
