"""Execute the actual source bodies; never maintain a copy of the writer."""
import hashlib
from pathlib import Path
import sys

source, output = map(Path, sys.argv[1:])
data = source.read_text()

def section(start, end):
    if data.count(start) != 1 or (end and data.count(end) != 1):
        raise SystemExit("Pinned serial source boundaries changed")
    begin = data.index(start)
    return data[begin:data.index(end, begin) if end else len(data)]

# Include PayloadToName too: an unpatched writer retains its actual unsafe
# memcpy, so the same sanitizer probe can demonstrate the regression.
writer = section("static uint64_t PayloadToName(",
                 "void CommDriverN2KSerial::ProcessManagementPacket(")
serializer = section("#define MaxActisenseMsgBuf", "")
output.write_text("// SHA256 of complete source: " +
                  hashlib.sha256(source.read_bytes()).hexdigest() + "\n" +
                  serializer + "\n" + writer + "\n" +
                  section("void CommDriverN2KSerial::handle_N2K_SERIAL_RAW(",
                          "int CommDriverN2KSerial::GetMfgCode()") + "\n" +
                  section("int CommDriverN2KSerial::SendMgmtMsg(", "int CommDriverN2KSerial::SetTXPGN(") + "\n" +
                  section("size_t CommDriverN2KSerialThread::WriteComPortPhysical(\n    std::vector<unsigned char> msg)",
                          "bool CommDriverN2KSerialThread::SetOutMsg("))

gate = (Path(__file__).resolve().parents[2] / "src/integration/OpenCPNPilot.cpp").read_text()
start = "bool OpenCPNPilot::Send(const adapters::PilotRequest &r)"
if gate.count(start) != 1:
    raise SystemExit("Pilot gate source boundary changed")
state_start = "adapters::PilotFeedback OpenCPNPilot::GetState() const"
state_end = "std::string OpenCPNPilot::Description() const"
if gate.count(state_start) != 1 or gate.count(state_end) != 1:
    raise SystemExit("Pilot state source boundary changed")
(output.parent / "actual_gate.inc").write_text(
    gate[gate.index(state_start):gate.index(state_end)] + gate[gate.index(start):])

(output.parent / "actual_lifecycle.inc").write_text(
    section("bool CommDriverN2KSerial::Open() {", "static uint64_t PayloadToName("))
# Constructor selection and flag type are outside extracted method bodies.
header = source.parents[1] / "include/model/comm_drv_n2k_serial.h"
if ": wxThread(wxTHREAD_JOINABLE)" not in data:
    raise SystemExit("Serial worker must have explicit joinable ownership")
if "std::atomic_bool m_bsec_thread_active" not in header.read_text():
    raise SystemExit("Cross-thread active flag must be atomic")
