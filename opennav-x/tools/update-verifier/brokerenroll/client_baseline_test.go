package brokerenroll

import (
	"bytes"
	"context"
	"errors"
	"io"
	"reflect"
	"runtime"
	"testing"

	"example.com/opennav-update-verifier/releasepolicy"
)

type baselineFixture struct {
	*bytes.Reader
	*testOwner
	body      []byte
	size      int64
	checks    int
	failCheck int
	checkErr  error
	readErr   error
	readHook  func()
}

func newBaselineFixture() *baselineFixture {
	body := []byte("INERT baseline for private orchestration seam; not an approved ZIP")
	return &baselineFixture{Reader: bytes.NewReader(body), testOwner: &testOwner{}, body: body, size: int64(len(body))}
}
func (b *baselineFixture) Bytes() []byte { return b.body }
func (b *baselineFixture) Size() int64   { return b.size }
func (b *baselineFixture) Validate() error {
	if b.closed {
		return errors.New("baseline closed")
	}
	b.checks++
	if b.checkErr != nil {
		return b.checkErr
	}
	if b.failCheck != 0 && b.checks >= b.failCheck {
		return errors.New("baseline custody changed")
	}
	return nil
}
func (b *baselineFixture) ReadAt(p []byte, off int64) (int, error) {
	if b.closed {
		panic("closed baseline read")
	}
	if b.readHook != nil {
		b.readHook()
	}
	if b.readErr != nil {
		return 0, b.readErr
	}
	return b.Reader.ReadAt(p, off)
}

// The private seam tests orchestration only. The production Run binding always
// invokes the real ImportApprovedBaseline validator; inert bytes grant no trust.
func clientBaselineOperations(t *testing.T, f *enrollmentFixture) *baselineFixture {
	t.Helper()
	b := newBaselineFixture()
	b.onClose = func() { f.events = append(f.events, "baseline.close") }
	f.op.openBaseline = func(ctx context.Context, directory string) (ApprovedClientBaseline, error) {
		f.events = append(f.events, "baseline.open")
		if directory != f.req.ArtifactDirectory || f.inputs.closed || ctx.Err() != nil {
			t.Fatal("baseline lost fixed protected operator parent")
		}
		return b, nil
	}
	f.op.validateBaseline = func(ctx context.Context, raw []byte, sha string, source ApprovedClientBaseline) error {
		f.events = append(f.events, "baseline.validate")
		if source != b || b.closed || ctx.Err() != nil || digest(raw) != sha || sha != f.req.ApprovedClientRequestSHA256 {
			t.Fatal("baseline lost independently approved original request")
		}
		return nil
	}
	return b
}

func approvedBaselineEnrollment(t *testing.T) (*enrollmentFixture, *clientFilesFixture, *baselineFixture) {
	f := newEnrollmentFixture(t)
	f.req.ClientRegistrationRequest = approvedClientFixture(t, f.req.TrustConfiguration)
	f.req.ApprovedClientRequestSHA256 = digest(f.req.ClientRegistrationRequest)
	f.op.validateClient = func(string) error { return nil }
	b := clientBaselineOperations(t, f)
	files := &clientFilesFixture{testFiles: f.files}
	create := f.op.create
	f.op.create = func(ctx context.Context, source io.Reader, artifact releasepolicy.Artifact) (enrollmentFiles, error) {
		if _, err := create(ctx, source, artifact); err != nil {
			return nil, err
		}
		return files, nil
	}
	f.op.initialize = func(ctx context.Context, retained enrollmentFiles) error {
		f.events = append(f.events, "initialize")
		if retained != files || files.closed || b.closed || f.inputs.closed || f.broker.closed ||
			files.publishes != 1 || files.baselinePublishes != 1 || !bytes.Equal(files.baseline, b.Bytes()) {
			t.Fatal("SCM lost original baseline or fresh publication custody")
		}
		return ctx.Err()
	}
	return f, files, b
}

func TestClientBaselineRetainedBeforeMutationThroughSCM(t *testing.T) {
	f, _, b := approvedBaselineEnrollment(t)
	if err := run(context.Background(), f.req, f.op); err != nil {
		t.Fatal(err)
	}
	want := []string{"hold", "baseline.open", "baseline.validate", "verify", "acquire", "create", "initialize", "files.close", "broker.close", "baseline.close", "inputs.close"}
	if !reflect.DeepEqual(f.events, want) || !b.closed {
		t.Fatalf("order/custody: %v", f.events)
	}
}

func TestClientBaselineFailureBeforeMutation(t *testing.T) {
	for _, fault := range []string{"missing-open", "missing-validator", "open", "open-cleanup", "missing-owner", "initial-custody", "zero-size", "oversize", "validation", "validation-cleanup", "after-validation", "cancel-validation", "invalid-real-zip"} {
		t.Run(fault, func(t *testing.T) {
			f, _, b := approvedBaselineEnrollment(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "missing-open":
				f.op.openBaseline = nil
			case "missing-validator":
				f.op.validateBaseline = nil
			case "open":
				f.op.openBaseline = func(context.Context, string) (ApprovedClientBaseline, error) { return b, errors.New("open") }
			case "open-cleanup":
				f.op.openBaseline = func(context.Context, string) (ApprovedClientBaseline, error) { return b, ErrCleanup }
			case "missing-owner":
				f.op.openBaseline = func(context.Context, string) (ApprovedClientBaseline, error) { return nil, nil }
			case "initial-custody":
				b.checkErr = errors.New("changed")
			case "zero-size":
				b.size = 0
			case "oversize":
				b.size = maxClientBaselineBytes + 1
			case "validation":
				f.op.validateBaseline = func(context.Context, []byte, string, ApprovedClientBaseline) error { return errors.New("unapproved") }
			case "validation-cleanup":
				f.op.validateBaseline = func(context.Context, []byte, string, ApprovedClientBaseline) error { return ErrCleanup }
			case "after-validation":
				b.failCheck = 2
			case "cancel-validation":
				f.op.validateBaseline = func(context.Context, []byte, string, ApprovedClientBaseline) error { cancel(); return nil }
			case "invalid-real-zip":
				f.op.validateBaseline = validateApprovedClientBaseline
			}
			err := run(ctx, f.req, f.op)
			if err == nil {
				t.Fatal("invalid baseline accepted")
			}
			for _, event := range f.events {
				if event == "verify" || event == "acquire" || event == "create" || event == "initialize" {
					t.Fatalf("effects before baseline approval: %v", f.events)
				}
			}
			if fault == "open-cleanup" || fault == "validation-cleanup" {
				if !errors.Is(err, ErrCleanup) {
					t.Fatal("cleanup uncertainty lost")
				}
			}
			if fault != "missing-open" && fault != "missing-validator" && fault != "missing-owner" && !b.closed {
				t.Fatal("source owner leaked")
			}
		})
	}
}

func TestClientBaselineFailuresPreserveCustodyAndCompletionUncertainty(t *testing.T) {
	for _, fault := range []string{"copy", "publication", "cleanup", "source-custody", "cancel", "source-close", "post-SCM-custody", "cancel-after-SCM"} {
		t.Run(fault, func(t *testing.T) {
			f, files, b := approvedBaselineEnrollment(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			starts := 0
			initialize := f.op.initialize
			f.op.initialize = func(ctx context.Context, owner enrollmentFiles) error {
				starts++
				if err := initialize(ctx, owner); err != nil {
					return err
				}
				if fault == "post-SCM-custody" {
					b.checkErr = errors.New("changed")
				}
				if fault == "cancel-after-SCM" {
					cancel()
				}
				return nil
			}
			switch fault {
			case "copy":
				b.readErr = errors.New("read")
			case "publication":
				files.baselineFault = errors.New("publication")
			case "cleanup":
				files.baselineFault = ErrCleanup
			case "source-custody":
				files.baselineHook = func() { b.checkErr = errors.New("changed") }
			case "cancel":
				files.baselineHook = cancel
			case "source-close":
				b.closeErr = errors.New("close")
			}
			err := run(ctx, f.req, f.op)
			postCommit := fault == "source-close" || fault == "post-SCM-custody" || fault == "cancel-after-SCM"
			if err == nil || starts != map[bool]int{false: 0, true: 1}[postCommit] || !b.closed || !files.closed || !f.broker.closed || !f.inputs.closed {
				t.Fatalf("fault=%s err=%v starts=%d", fault, err, starts)
			}
			if fault == "cleanup" || fault == "source-close" {
				if !errors.Is(err, ErrCleanup) {
					t.Fatal("cleanup uncertainty lost")
				}
			}
		})
	}
}

type baselineShortWriter struct{}

func (baselineShortWriter) Write(p []byte) (int, error) { return len(p) - 1, nil }

func TestClientBaselineCopyExactAndBounded(t *testing.T) {
	for _, fault := range []string{"none", "short-file", "long-file", "zero", "oversize", "read", "short-write", "custody", "late-custody", "cancel-read"} {
		t.Run(fault, func(t *testing.T) {
			b := newBaselineFixture()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			var output bytes.Buffer
			var destination io.Writer = &output
			switch fault {
			case "short-file":
				b.size++
			case "long-file":
				b.size--
			case "zero":
				b.size = 0
			case "oversize":
				b.size = maxClientBaselineBytes + 1
			case "read":
				b.readErr = errors.New("read")
			case "short-write":
				destination = baselineShortWriter{}
			case "custody":
				b.checkErr = errors.New("changed")
			case "late-custody":
				b.failCheck = 2
			case "cancel-read":
				b.readHook = cancel
			}
			sha, err := copyClientBaseline(ctx, destination, b)
			if fault == "none" {
				if err != nil || sha != digest(b.Bytes()) || !bytes.Equal(output.Bytes(), b.Bytes()) {
					t.Fatalf("copy differs: %v", err)
				}
			} else if err == nil || sha != "" {
				t.Fatalf("fault=%s accepted", fault)
			}
		})
	}
}

func TestClientBaselineOtherPlatformExplicitlyRefuses(t *testing.T) {
	if runtime.GOOS == "windows" {
		t.Skip("non-Windows refusal")
	}
	if owner, err := OpenApprovedClientBaselineNative(context.Background(), inertClientSID); owner != nil || err == nil {
		t.Fatal("non-native broker baseline accepted")
	}
	if owner, err := openClientBaselineInputNative(context.Background(), "unused"); owner != nil || err == nil {
		t.Fatal("non-native operator baseline accepted")
	}
}
