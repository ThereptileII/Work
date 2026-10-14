package brokerenroll

import (
	"bytes"
	"context"
	"debug/pe"
	"encoding/binary"
	"errors"
	"io"
	"reflect"
	"testing"

	"example.com/opennav-update-verifier/brokerregistration"
	"example.com/opennav-update-verifier/releasepolicy"
)

func maintenancePEFixture(t *testing.T) []byte {
	t.Helper()
	raw := make([]byte, 1024)
	copy(raw, "MZ")
	binary.LittleEndian.PutUint32(raw[0x3c:], 0x80)
	copy(raw[0x80:], "PE\x00\x00")
	var headers bytes.Buffer
	file := pe.FileHeader{Machine: pe.IMAGE_FILE_MACHINE_I386, NumberOfSections: 1, SizeOfOptionalHeader: 224, Characteristics: pe.IMAGE_FILE_EXECUTABLE_IMAGE}
	optional := pe.OptionalHeader32{Magic: 0x10b, Subsystem: 2, NumberOfRvaAndSizes: 16, SizeOfHeaders: 512}
	optional.DataDirectory[pe.IMAGE_DIRECTORY_ENTRY_IMPORT] = pe.DataDirectory{VirtualAddress: 0x1000, Size: 40}
	section := pe.SectionHeader32{VirtualSize: 512, VirtualAddress: 0x1000, SizeOfRawData: 512, PointerToRawData: 512}
	copy(section.Name[:], ".idata")
	for _, part := range []any{file, optional, section} {
		if err := binary.Write(&headers, binary.LittleEndian, part); err != nil {
			t.Fatal(err)
		}
	}
	copy(raw[0x84:], headers.Bytes())
	binary.LittleEndian.PutUint32(raw[512:], 0x1080)
	binary.LittleEndian.PutUint32(raw[524:], 0x1040)
	binary.LittleEndian.PutUint32(raw[528:], 0x1090)
	copy(raw[576:], "kernel32.dll\x00")
	return raw
}

func TestStartupMaintenanceImageLoaderRefusesMutableAndAlternateDLLContracts(t *testing.T) {
	for _, fault := range []string{"valid", "foreign DLL", "path DLL", "delay imports", "CLR", "wrong ABI", "DLL image", "missing imports", "unterminated imports", "outside RVA", "truncated"} {
		t.Run(fault, func(t *testing.T) {
			raw := maintenancePEFixture(t)
			switch fault {
			case "foreign DLL":
				copy(raw[576:], "evil-one.dll\x00")
			case "path DLL":
				copy(raw[576:], ".\\kernel32.dll\x00")
			case "delay imports":
				binary.LittleEndian.PutUint32(raw[0x98+96+13*8:], 0x1000)
			case "CLR":
				binary.LittleEndian.PutUint32(raw[0x98+96+14*8:], 0x1000)
			case "wrong ABI":
				binary.LittleEndian.PutUint16(raw[0x84:], pe.IMAGE_FILE_MACHINE_AMD64)
			case "DLL image":
				binary.LittleEndian.PutUint16(raw[0x84+18:], pe.IMAGE_FILE_EXECUTABLE_IMAGE|pe.IMAGE_FILE_DLL)
			case "missing imports":
				binary.LittleEndian.PutUint32(raw[0x98+96+8:], 0)
			case "unterminated imports":
				copy(raw[532:552], raw[512:532])
			case "outside RVA":
				binary.LittleEndian.PutUint32(raw[524:], 0xfffffff0)
			case "truncated":
				raw = raw[:530]
			}
			if err := validateMaintenanceLoader(bytes.NewReader(raw), int64(len(raw))); (err == nil) != (fault == "valid") {
				t.Fatal("loader contract result differs", err)
			}
		})
	}
}

func TestStartupMaintenanceImageCopyChecksOriginalApprovedLengthAndHash(t *testing.T) {
	for _, fault := range []string{"valid", "wrong SHA", "short", "extra", "wrong artifact", "oversized", "cancelled"} {
		t.Run(fault, func(t *testing.T) {
			body := []byte("exact authenticated original provisioner")
			a := releasepolicy.Artifact{Path: "bootstrap/" + maintenanceImageName, SHA256: digest(body), Bytes: int64(len(body))}
			input := bytes.Clone(body)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			switch fault {
			case "wrong SHA":
				a.SHA256 = digest([]byte("foreign"))
			case "short":
				input = input[:len(input)-1]
			case "extra":
				input = append(input, '!')
			case "wrong artifact":
				a.Path = "bootstrap/" + enrollmentImageName
			case "oversized":
				a.Bytes = maxBrokerBytes + 1
			case "cancelled":
				cancel()
			}
			var destination bytes.Buffer
			err := copyMaintenanceImage(ctx, &destination, bytes.NewReader(input), a)
			if (err == nil) != (fault == "valid") {
				t.Fatal("copy approval result differs", err)
			}
			if fault == "valid" && !bytes.Equal(body, destination.Bytes()) {
				t.Fatal("original helper bytes changed")
			}
		})
	}
}

type maintenanceImageFixture struct {
	*bytes.Reader
	*testOwner
	checkErr error
}

func (s *maintenanceImageFixture) Check(ctx context.Context) error {
	if s.closed {
		return errors.New("original source closed")
	}
	return errors.Join(ctx.Err(), s.checkErr)
}

// Shared only with private startup orchestration fixtures. No native capability
// or valid PE is provided by these inert bytes.
func maintenanceImageOperations(t *testing.T, f *enrollmentFixture) *maintenanceImageFixture {
	t.Helper()
	s := &maintenanceImageFixture{Reader: bytes.NewReader([]byte("INERT PROVISIONER")), testOwner: &testOwner{onClose: func() { f.events = append(f.events, "maintenance.close") }}}
	f.op.openMaintenanceImage = func(ctx context.Context, a releasepolicy.Artifact) (maintenanceImageSource, error) {
		f.events = append(f.events, "maintenance.open")
		if a != f.policy.Provisioner || f.inputs.closed || ctx.Err() != nil {
			t.Fatal("original signed maintenance selection lost")
		}
		return s, nil
	}
	f.op.publishMaintenanceImage = func(ctx context.Context, files enrollmentFiles, source maintenanceImageSource, a releasepolicy.Artifact) error {
		f.events = append(f.events, "maintenance.publish")
		if source != s || s.closed || f.files.closed || f.broker.closed || a != f.policy.Provisioner {
			t.Fatal("original helper owner changed before publication")
		}
		return copyMaintenanceImage(ctx, io.Discard, source, a)
	}
	return s
}

func TestStartupMaintenanceImageRetainsOriginalSourceThroughCompletion(t *testing.T) {
	f, _, _ := approvedBaselineEnrollment(t)
	s := maintenanceImageOperations(t, f)
	initialize := f.op.initialize
	f.op.initializeStartup = func(ctx context.Context, files enrollmentFiles, _ *brokerregistration.Request, _ ApprovedClientBaseline) error {
		if s.closed {
			t.Fatal("startup outlived original helper source")
		}
		return initialize(ctx, files)
	}
	if err := run(context.Background(), f.req, f.op); err != nil {
		t.Fatal(err)
	}
	want := []string{"hold", "baseline.open", "baseline.validate", "verify", "maintenance.open", "acquire", "create", "maintenance.publish", "initialize", "files.close", "broker.close", "maintenance.close", "baseline.close", "inputs.close"}
	if !reflect.DeepEqual(f.events, want) || !s.closed {
		t.Fatal("startup helper lifetime/order differs", f.events)
	}
}

func TestStartupMaintenanceImageFailuresRefusePublicationOrActivation(t *testing.T) {
	for _, fault := range []string{"missing opener", "missing publisher", "open", "initial custody", "publish", "publish cleanup", "late custody", "source close"} {
		t.Run(fault, func(t *testing.T) {
			f, _, _ := approvedBaselineEnrollment(t)
			s := maintenanceImageOperations(t, f)
			initialized := false
			f.op.initializeStartup = func(context.Context, enrollmentFiles, *brokerregistration.Request, ApprovedClientBaseline) error {
				initialized = true
				return nil
			}
			switch fault {
			case "missing opener":
				f.op.openMaintenanceImage = nil
			case "missing publisher":
				f.op.publishMaintenanceImage = nil
			case "open":
				f.op.openMaintenanceImage = func(context.Context, releasepolicy.Artifact) (maintenanceImageSource, error) {
					return s, errors.New("private source detail")
				}
			case "initial custody":
				s.checkErr = errors.New("private source detail")
			case "publish", "publish cleanup":
				f.op.publishMaintenanceImage = func(context.Context, enrollmentFiles, maintenanceImageSource, releasepolicy.Artifact) error {
					if fault == "publish cleanup" {
						return ErrCleanup
					}
					return errors.New("private destination detail")
				}
			case "late custody":
				f.op.publishMaintenanceImage = func(context.Context, enrollmentFiles, maintenanceImageSource, releasepolicy.Artifact) error {
					s.checkErr = errors.New("private source detail")
					return nil
				}
			case "source close":
				s.closeErr = errors.New("private source detail")
			}
			err := run(context.Background(), f.req, f.op)
			if err == nil || initialized != (fault == "source close") {
				t.Fatal("failed helper input activated startup", f.events, err)
			}
			if (fault == "publish cleanup" || fault == "source close") && !errors.Is(err, ErrCleanup) {
				t.Fatal("cleanup classification lost", err)
			}
			if fault == "missing opener" || fault == "missing publisher" || fault == "open" || fault == "initial custody" {
				for _, event := range f.events {
					if event == "create" {
						t.Fatal("bad original source mutated machine", f.events)
					}
				}
			}
		})
	}
}

func TestStartupOrdinaryEnrollmentNeverAcquiresMaintenanceImage(t *testing.T) {
	f := newEnrollmentFixture(t)
	f.op.openMaintenanceImage = func(context.Context, releasepolicy.Artifact) (maintenanceImageSource, error) {
		t.Fatal("ordinary enrollment acquired customer startup capability")
		return nil, nil
	}
	if err := run(context.Background(), f.req, f.op); err != nil {
		t.Fatal(err)
	}
}
