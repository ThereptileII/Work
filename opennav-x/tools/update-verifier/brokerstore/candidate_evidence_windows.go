package brokerstore

import (
	"errors"

	"example.com/opennav-update-verifier/baselineevidence"
	"golang.org/x/sys/windows"
)

// checkRecoveryEvidenceResidue recognizes only the fixed original sidecars and
// fixed execution helper namespaces for acquisition reconciliation. This is not completion/adoption or
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
	entries, err := boundedEntries(dir, 3)
	if err != nil {
		return err
	}
	// An empty directory can remain after exclusive creation failed. It is never
	// completed or adopted here. Partial fixed helper sets also remain inert.
	for _, entry := range entries {
		switch entry.Name() {
		case "package.json":
			err = c.checkRecoveryEvidenceFile(dir, "package.json")
		case "engine":
			err = c.checkRecoveryHelpersResidue(dir, "engine", 3, baselineevidence.IsEngineHelper)
		case "commissioning-v1":
			err = c.checkRecoveryHelpersResidue(dir, "commissioning-v1", 12, baselineevidence.IsCommissioningHelper)
		default:
			return errors.New("unknown installer evidence residue")
		}
		if err != nil {
			return err
		}
	}
	return c.check()
}

func (c *nativeCandidate) checkRecoveryHelpersResidue(parent *nativePin, component string, limit int, recognizes func(string) bool) (resultErr error) {
	name := parent.name + `\` + component
	dir, err := openPin(physical(name), name, c.securityDir)
	if err != nil {
		return err
	}
	defer func() { resultErr = errors.Join(resultErr, windows.CloseHandle(dir.handle)) }()
	if dir.info.VolumeSerialNumber != parent.info.VolumeSerialNumber {
		return errors.New("candidate execution helper directory crossed volumes")
	}
	entries, err := boundedEntries(dir, limit)
	if err != nil {
		return err
	}
	for _, entry := range entries {
		if !recognizes("installer/" + component + "/" + entry.Name()) {
			return errors.New("unknown execution helper residue")
		}
		if err := c.checkRecoveryEvidenceFile(dir, entry.Name()); err != nil {
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
