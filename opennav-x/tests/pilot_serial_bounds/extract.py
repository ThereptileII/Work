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
                  serializer + "\n" + writer)
