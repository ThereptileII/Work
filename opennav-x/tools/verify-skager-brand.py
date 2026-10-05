#!/usr/bin/env python3
"""Fail closed if checked-in SKAGER art differs from its approved provenance."""
import hashlib
import json
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parents[1]
record = json.loads((ROOT / 'resources/branding/provenance.json').read_text())
assert record['sourceSha256'] == 'dce92b90a8657d045403cf4508d8220cb0306e6a251d58fbf40e6084ea3ee4ee'
for path, expected in {record['source']: record['sourceSha256'], **record['outputs']}.items():
    assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == expected, path
icon = (ROOT / 'resources/branding/skager.ico').read_bytes()
reserved, kind, count = struct.unpack_from('<HHH', icon)
assert (reserved, kind, count) == (0, 1, len(record['iconSizes']))
sizes = []
for n in range(count):
    w, h, colors, reserved, planes, depth, size, offset = struct.unpack_from('<BBBBHHII', icon, 6 + n * 16)
    w, h = w or 256, h or 256
    assert w == h and depth == 32 and planes == 0 and size > 0 and offset + size <= len(icon)
    sizes.append(w)
assert sorted(sizes) == record['iconSizes']
print('Approved source, wordmark, embedded bytes and nine Windows icon sizes verified')
