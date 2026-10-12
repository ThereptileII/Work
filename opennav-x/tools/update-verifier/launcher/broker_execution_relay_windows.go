package launcher

import (
	"os"
	"path"
	"strings"

	"example.com/opennav-update-verifier/brokerpipe"
	"golang.org/x/sys/windows"
)

// The only production caller first derives the fixed ProgramFilesX86 current-SID
// generation path from an authenticated finite broker directive. This factory
// shares physical image/ancestor custody, never ordinary executable policy.
func acquireNativeRelayImage(spec processSpec) (ordinaryImage, error) {
	name, err := localPath(spec.executable)
	if err != nil {
		return nil, err
	}
	directory, err := localPath(spec.directory)
	if err != nil || !strings.EqualFold(path.Dir(name), directory) || !strings.EqualFold(path.Base(name), "skager-start.exe") {
		return nil, errBrokerExecutionRefused
	}
	return acquireNativePinnedBrokerImage(name, spec.expectedImageHash)
}
func brokerRelayEnvironment(values []string, c brokerpipe.ExecutionCommand) ([]uint16, error) {
	if (c.Phase != brokerpipe.RelaySelectedExecutor && c.Phase != brokerpipe.RelayStartupExecutor) || brokerExecutionPayload(c, c.StageID) != nil {
		return nil, errBrokerExecutionRefused
	}
	return ordinaryEnvironment(values)
}
func (image *nativeOrdinaryImage) createBrokerRelay(c brokerpipe.ExecutionCommand, attributes *windows.SecurityAttributes) (*nativeOrdinaryChild, error) {
	if !strings.EqualFold(path.Base(strings.ReplaceAll(image.name, `\`, "/")), "skager-start.exe") {
		return nil, errBrokerExecutionRefused
	}
	environment, err := brokerRelayEnvironment(os.Environ(), c)
	if err != nil {
		return nil, err
	}
	// The new launcher makes a fresh current or startup-offer request. No path, command,
	// qualification environment or inherited pipe handles cross this boundary.
	argument := "--broker-current=" + c.ApplicationGeneration
	if c.Phase == brokerpipe.RelayStartupExecutor {
		argument = "--broker-offer=" + c.ApplicationGeneration
	}
	return image.createBrokerFixedChild(attributes, environment, argument)
}
