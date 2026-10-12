package launcher

import (
	"os"

	"example.com/opennav-update-verifier/brokerpipe"
	"golang.org/x/sys/windows"
)

// Normal startup never supplies any qualification/health environment. It does
// not alter an existing child or clear its lifetime qualification output latch.
// The live broker must independently retain the selected generation and grant
// this one normal attempt; the directive's generation is only a fixed lookup.
func brokerCurrentEnvironment(values []string, c brokerpipe.ExecutionCommand) ([]uint16, error) {
	if c.Phase != brokerpipe.SpawnCurrent || brokerExecutionPayload(c, c.StageID) != nil {
		return nil, errBrokerExecutionRefused
	}
	return ordinaryEnvironment(values)
}
func brokerCurrentArgumentForMode(mode brokerpipe.StartupMode) (string, error) {
	switch mode {
	case brokerpipe.StartupXNav:
		return "--xnav", nil
	case brokerpipe.StartupLegacy:
		return "--legacy", nil
	case brokerpipe.StartupSafe:
		return "--safe-mode", nil
	}
	return "", errBrokerExecutionRefused
}
func (image *nativeOrdinaryImage) createBrokerCurrent(c brokerpipe.ExecutionCommand, attributes *windows.SecurityAttributes) (*nativeOrdinaryChild, error) {
	environment, err := brokerCurrentEnvironment(os.Environ(), c)
	if err != nil {
		return nil, err
	}
	argument, err := brokerCurrentArgumentForMode(c.StartupMode)
	if err != nil {
		return nil, err
	}
	return image.createBrokerFixedChild(attributes, environment, argument)
}
