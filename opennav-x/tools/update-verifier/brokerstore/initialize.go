package brokerstore

import "errors"

// freshCreation separates the irreversible reservation from directory creation.
// Implementations retain every created object on failure; close only drains
// owned handles and must report uncertain cleanup.
type freshCreation interface {
	check() error
	rootAbsent() error
	reserve() error
	createRoot() error
	createCandidates() error
	createRegistrations() error
	complete() error
	close() error
}

func initializeFresh(creation freshCreation) (resultErr error) {
	defer func() { resultErr = errors.Join(resultErr, creation.close()) }()
	for _, step := range []func() error{
		creation.check, creation.rootAbsent, creation.check,
		creation.reserve, creation.check,
		creation.createRoot, creation.check,
		creation.createCandidates, creation.check,
		creation.createRegistrations, creation.check,
		creation.complete, creation.check,
	} {
		if err := step(); err != nil {
			return err
		}
	}
	return nil
}
