//go:build windows

package brokerenroll

import (
	"context"
	"crypto/sha256"
	"debug/pe"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"syscall"
	"testing"
	"time"

	"example.com/opennav-update-verifier/releasepolicy"
	"golang.org/x/sys/windows"
)

const enrollmentSmokeOptIn = "SCRUM-311-disposable-hosted-enrollment"

var enrollmentSmokeCommit = regexp.MustCompile(`^[0-9a-f]{40}$`)
var enrollmentSmokeRun = regexp.MustCompile(`^[1-9][0-9]*$`)

// This is deliberately absent from the ordinary component test run. A separate
// explicitly opted-in job on a disposable GitHub-hosted Windows machine may
// consume the retained, verified same-commit broker producer. It creates the
// fixed real machine namespace and service once, never deletes or adopts them.
func TestNativeFreshEnrollmentServiceSmoke(t *testing.T) {
	if os.Getenv("SKAGER_ENROLLMENT_NATIVE_OPT_IN") != enrollmentSmokeOptIn {
		t.Skip("real machine enrollment requires explicit disposable CI fixture opt-in")
	}
	if os.Getenv("GITHUB_ACTIONS") != "true" || os.Getenv("RUNNER_OS") != "Windows" ||
		os.Getenv("SKAGER_ENROLLMENT_RUNNER_ENVIRONMENT") != "github-hosted" ||
		os.Getenv("GITHUB_REPOSITORY") != "ThereptileII/Work" ||
		!enrollmentSmokeRun.MatchString(os.Getenv("GITHUB_RUN_ID")) ||
		!enrollmentSmokeCommit.MatchString(os.Getenv("GITHUB_SHA")) ||
		!enrollmentSmokeProducerMatches() {
		t.Fatal("disposable same-commit GitHub-hosted Windows runner identity required")
	}
	if err := enrollmentOperator(); err != nil {
		t.Fatal("actual elevated non-impersonated enrollment operator required:", err)
	}
	if err := enrollmentSmokeAbsent(); err != nil {
		t.Fatal("fixed machine enrollment namespace is not fresh:", err)
	}

	path := os.Getenv("SKAGER_ENROLLMENT_BROKER_PATH")
	workspace := os.Getenv("GITHUB_WORKSPACE")
	expectedPath := filepath.Join(workspace, "opennav-x", "build", "updater-native", "broker-install", enrollmentImageName)
	if workspace == "" || !filepath.IsAbs(workspace) || filepath.Base(path) != enrollmentImageName ||
		!filepath.IsAbs(path) || filepath.Clean(path) != path || !strings.EqualFold(path, expectedPath) {
		t.Fatal("retained producer broker path must be absolute and exact")
	}
	file, err := os.Open(path)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := file.Close(); err != nil {
			t.Error("retained producer handle close:", err)
		}
	})
	var info windows.ByHandleFileInformation
	if err := windows.GetFileInformationByHandle(windows.Handle(file.Fd()), &info); err != nil ||
		info.FileAttributes&(windows.FILE_ATTRIBUTE_DIRECTORY|windows.FILE_ATTRIBUTE_REPARSE_POINT) != 0 || info.NumberOfLinks != 1 {
		t.Fatal("retained producer must be a single-link ordinary broker file:", err)
	}
	wantSHA := os.Getenv("SKAGER_ENROLLMENT_BROKER_SHA256")
	if len(wantSHA) != sha256.Size*2 || strings.ToLower(wantSHA) != wantSHA {
		t.Fatal("independent retained producer SHA-256 required")
	}
	if _, err := hex.DecodeString(wantSHA); err != nil {
		t.Fatal("invalid retained producer SHA-256:", err)
	}
	stat, err := file.Stat()
	if err != nil || stat.Size() <= 0 || stat.Size() > maxBrokerBytes {
		t.Fatal("bounded retained producer broker required:", err)
	}
	image, err := pe.NewFile(file)
	if err != nil {
		t.Fatal("retained producer is not PE:", err)
	}
	optional, ok := image.OptionalHeader.(*pe.OptionalHeader32)
	if image.Machine != pe.IMAGE_FILE_MACHINE_I386 || !ok || optional.Subsystem != 2 {
		t.Fatal("retained producer must be the Win32 GUI broker")
	}
	if _, err := file.Seek(0, io.SeekStart); err != nil {
		t.Fatal(err)
	}
	hash := sha256.New()
	if _, err := io.Copy(hash, file); err != nil || hex.EncodeToString(hash.Sum(nil)) != wantSHA {
		t.Fatal("retained producer bytes differ from verified producer record:", err)
	}
	if _, err := file.Seek(0, io.SeekStart); err != nil {
		t.Fatal(err)
	}
	// Recheck immediately before the first privileged create. The creator also
	// uses exclusive native creation and refuses any existing fixed child.
	if err := enrollmentSmokeAbsent(); err != nil {
		t.Fatal("fixed machine namespace changed before creation:", err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Minute)
	defer cancel()
	artifact := releasepolicy.Artifact{Path: "bootstrap/" + enrollmentImageName, SHA256: wantSHA, Bytes: stat.Size()}
	files, err := createFreshFiles(ctx, file, artifact)
	if err != nil {
		var mutation *enrollmentAncestorMutationError
		if errors.As(err, &mutation) {
			// Same held descriptor observation, not a pathname ACL reread.
			// These facts diagnose helper refusal; they qualify no product.
			t.Logf("held ancestor mutation rejection: role=%s component=%d allowAdds=%t ownerSID=%s aceSID=%s aceSIDValid=%t mask=0x%08x flags=0x%02x",
				mutation.ancestor.role, mutation.ancestor.component, mutation.allowAdds,
				mutation.ownerSID, mutation.aceSID, mutation.aceSIDValid, mutation.mask, mutation.flags)
		}
		t.Fatal("fresh native files refused; fixed state retained for review:", err)
	}
	t.Cleanup(func() {
		if err := files.Close(); err != nil {
			t.Error("enrollment file handles close:", err)
		}
	})
	if err := initializeService(ctx, files); err != nil {
		var initial *serviceInitialStateError
		if errors.As(err, &initial) {
			var queryCode syscall.Errno
			_ = errors.As(initial.cause, &queryCode)
			t.Logf("held initial service status rejection: phase=%s statusAvailable=%t queryErrorCode=%d state=%d processID=%d win32Exit=%d serviceExit=%d controls=0x%08x",
				initial.phase, initial.cause == nil, uint32(queryCode), initial.state.state, initial.state.processID,
				initial.state.win32Exit, initial.state.serviceExit, initial.state.controls)
		}
		var mutation *serviceMutationError
		if errors.As(err, &mutation) {
			// Observations from the same held descriptor that refused enrollment;
			// no ACL reread, path, username or additional authority is involved.
			t.Logf("held machine mutation rejection: role=%s ancestor=%d ownerSID=%s aceSID=%s mask=0x%08x flags=0x%02x mutation=0x%08x",
				mutation.location.role, mutation.location.ancestor, mutation.ownerSID, mutation.aceSID,
				mutation.mask, mutation.flags, mutation.mutation)
		}
		t.Fatal("real stopped-only initialize-store/verify-store refused; fixed state retained for review:", err)
	}
	// verify-store above rereads completion as the actual broker principal. An
	// elevated test process cannot substitute an administrator read for that gate.
	if err := files.Check(); err != nil {
		t.Fatal("retained created files changed:", err)
	}
	if err := enrollmentSmokeServiceReadback(files.ImagePath()); err != nil {
		t.Fatal("created stopped service readback:", err)
	}
	t.Log("actual fresh files, stopped zero-exit initialize-store and broker-principal verify-store, fixed service configuration/DACL/SID passed")
}

// A narrowly selected helper-only retest keeps the original compiled broker and
// current test revision distinct. The PowerShell entry reauthenticates the exact
// artifact digest and original successful producer steps through GitHub before
// this fixture. A caller-supplied commit or mere environment switch is not enough.
func enrollmentSmokeProducerMatches() bool {
	producer := os.Getenv("SKAGER_ENROLLMENT_PRODUCER_COMMIT")
	if !enrollmentSmokeCommit.MatchString(producer) {
		return false
	}
	if producer == os.Getenv("GITHUB_SHA") {
		return os.Getenv("SKAGER_ENROLLMENT_RETAINED_PRODUCER") == ""
	}
	if os.Getenv("SKAGER_ENROLLMENT_RETAINED_PRODUCER") != "SCRUM-311-retained-original-broker" || !enrollmentSmokeCommit.MatchString(producer) {
		return false
	}
	path := filepath.Join(os.Getenv("GITHUB_WORKSPACE"), "opennav-x", "tools", "retained-broker-enrollment-selection.json")
	raw, err := os.ReadFile(path)
	if err != nil || len(raw) > 4096 {
		return false
	}
	var selection struct {
		Schema         int    `json:"schema"`
		Repository     string `json:"repository"`
		RunID          string `json:"runId"`
		RunAttempt     string `json:"runAttempt"`
		HeadSHA        string `json:"headSha"`
		JobID          string `json:"jobId"`
		ArtifactID     string `json:"artifactId"`
		ArtifactName   string `json:"artifactName"`
		ArtifactDigest string `json:"artifactDigest"`
	}
	decoder := json.NewDecoder(strings.NewReader(string(raw)))
	decoder.DisallowUnknownFields()
	if decoder.Decode(&selection) != nil || decoder.Decode(&struct{}{}) != io.EOF {
		return false
	}
	return selection.Schema == 1 && selection.Repository == "ThereptileII/Work" && selection.HeadSHA == producer &&
		enrollmentSmokeRun.MatchString(selection.RunID) && enrollmentSmokeRun.MatchString(selection.RunAttempt) &&
		enrollmentSmokeRun.MatchString(selection.JobID) && enrollmentSmokeRun.MatchString(selection.ArtifactID) &&
		selection.ArtifactName == "update-verifier-windows-2022-386-"+producer &&
		regexp.MustCompile(`^sha256:[0-9a-f]{64}$`).MatchString(selection.ArtifactDigest)
}

func TestNativeEnrollmentRetestKeepsProducerAndHelperDistinct(t *testing.T) {
	workspace := t.TempDir()
	tools := filepath.Join(workspace, "opennav-x", "tools")
	if err := os.MkdirAll(tools, 0700); err != nil {
		t.Fatal(err)
	}
	t.Setenv("GITHUB_WORKSPACE", workspace)
	t.Setenv("GITHUB_SHA", strings.Repeat("a", 40))
	t.Setenv("SKAGER_ENROLLMENT_PRODUCER_COMMIT", strings.Repeat("a", 40))
	t.Setenv("SKAGER_ENROLLMENT_RETAINED_PRODUCER", "")
	if !enrollmentSmokeProducerMatches() {
		t.Fatal("ordinary exact-commit fixture refused")
	}
	t.Setenv("SKAGER_ENROLLMENT_PRODUCER_COMMIT", strings.Repeat("b", 40))
	if enrollmentSmokeProducerMatches() {
		t.Fatal("different producer accepted without separate retained opt-in")
	}
	t.Setenv("SKAGER_ENROLLMENT_RETAINED_PRODUCER", "SCRUM-311-retained-original-broker")
	if enrollmentSmokeProducerMatches() {
		t.Fatal("environment declaration replaced original committed selection")
	}
	selection := fmt.Sprintf(`{"schema":1,"repository":"ThereptileII/Work","runId":"1","runAttempt":"1","headSha":"%s","jobId":"2","artifactId":"3","artifactName":"update-verifier-windows-2022-386-%s","artifactDigest":"sha256:%s"}`, strings.Repeat("b", 40), strings.Repeat("b", 40), strings.Repeat("c", 64))
	path := filepath.Join(tools, "retained-broker-enrollment-selection.json")
	if err := os.WriteFile(path, []byte(selection), 0600); err != nil {
		t.Fatal(err)
	}
	if !enrollmentSmokeProducerMatches() {
		t.Fatal("separate helper refused its exact original committed selection")
	}
	t.Setenv("SKAGER_ENROLLMENT_PRODUCER_COMMIT", strings.Repeat("d", 40))
	if enrollmentSmokeProducerMatches() {
		t.Fatal("different original producer substituted")
	}
}

func enrollmentSmokeAbsent() error {
	code, err := windows.KnownFolderPath(windows.FOLDERID_ProgramFilesX86, 0)
	if err != nil {
		return err
	}
	data, err := windows.KnownFolderPath(windows.FOLDERID_ProgramData, 0)
	if err != nil {
		return err
	}
	for _, path := range []string{
		filepath.Join(code, "SKAGER"),
		filepath.Join(code, "SKAGER", "Applications"),
		filepath.Join(code, "SKAGER", "UpdateBroker"),
		filepath.Join(code, "SKAGER", "UpdateBroker", enrollmentImageName),
		filepath.Join(data, "SKAGER"),
		filepath.Join(data, "SKAGER", enrollmentFenceName),
		filepath.Join(data, "SKAGER", "UpdateBroker.initialized"),
		filepath.Join(data, "SKAGER", "UpdateBroker"),
	} {
		if _, err := os.Lstat(path); err == nil {
			return fmt.Errorf("fixed path already exists: %s", path)
		} else if !errors.Is(err, os.ErrNotExist) {
			return err
		}
	}
	scm, err := windows.OpenSCManager(nil, nil, windows.SC_MANAGER_CONNECT)
	if err != nil {
		return err
	}
	defer windows.CloseServiceHandle(scm)
	name, _ := windows.UTF16PtrFromString(serviceName)
	service, err := windows.OpenService(scm, name, windows.SERVICE_QUERY_STATUS)
	if err == nil {
		_ = windows.CloseServiceHandle(service)
		return errors.New("fixed broker service already exists")
	}
	if !errors.Is(err, windows.ERROR_SERVICE_DOES_NOT_EXIST) {
		return err
	}
	key, _ := windows.UTF16PtrFromString(`SYSTEM\CurrentControlSet\Services\` + serviceName)
	var registry windows.Handle
	err = windows.RegOpenKeyEx(windows.HKEY_LOCAL_MACHINE, key, 0, windows.KEY_READ, &registry)
	if err == nil {
		_ = windows.RegCloseKey(registry)
		return errors.New("fixed broker service registry key already exists")
	}
	if !errors.Is(err, windows.ERROR_FILE_NOT_FOUND) {
		return err
	}
	return nil
}

func enrollmentSmokeServiceReadback(image string) (result error) {
	native, err := newServiceBackend()
	if err != nil {
		return err
	}
	n := native.(*nativeServiceBackend)
	defer func() { result = errors.Join(result, n.close()) }()
	name, _ := windows.UTF16PtrFromString(serviceName)
	n.service, err = windows.OpenService(n.scm, name, windows.SERVICE_QUERY_CONFIG|windows.SERVICE_QUERY_STATUS|windows.READ_CONTROL)
	if err != nil {
		return err
	}
	if err := n.holdRegistry(`SYSTEM\CurrentControlSet\Services\` + serviceName); err != nil {
		return err
	}
	expectedSID, err := enrollmentBrokerSID()
	if err != nil {
		return err
	}
	actualSID, _, _, err := windows.LookupSID("", serviceAccount)
	if err != nil || actualSID == nil || !actualSID.Equals(expectedSID) {
		return errors.New("actual created broker service SID differs")
	}
	n.expected, err = windows.SecurityDescriptorFromString("D:P(A;;0xf01ff;;;SY)(A;;0xf01ff;;;BA)(A;;0x20005;;;AU)(A;;0x20005;;;" + actualSID.String() + ")")
	if err != nil {
		return err
	}
	if err := n.check(image); err != nil {
		return err
	}
	s, err := n.status()
	if err != nil || !serviceCompleted(s) {
		return errors.Join(errors.New("created broker is not stopped with zero exit"), err)
	}
	return nil
}
