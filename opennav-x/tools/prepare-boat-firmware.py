#!/usr/bin/env python3
"""Prepare reviewed boat-side sources and a host codec oracle. Never flash."""
import hashlib
from pathlib import Path
import shutil
import subprocess
import tempfile
import urllib.request

root=Path(__file__).resolve().parents[1]
revision='9baf01bca09522794a9678dfb3f3c0720d5c9943'
name='Nissan_ev_to_NMEA_and_wifi.ino'
expected='a7fa8d8ad1c26f5f72401f84e42996242d375822f301f2ad4a930b5e718fc53d'
request=urllib.request.Request(f'https://raw.githubusercontent.com/ThereptileII/Work/{revision}/{name}',headers={'User-Agent':'OpenNavX-firmware-validation'})
original=urllib.request.urlopen(request,timeout=60).read()
assert hashlib.sha256(original).hexdigest()==expected,'Unverified boat firmware'
target=root/'build/boat-firmware'/Path(name).stem
target.mkdir(parents=True,exist_ok=True)
with tempfile.TemporaryDirectory(prefix='OpenNav firmware patch ') as directory:
    working=Path(directory)
    subprocess.run(['git','init','-q',str(working)],check=True)
    subprocess.run(['git','-C',str(working),'config','core.autocrlf','false'],check=True)
    (working/name).write_bytes(original)
    # Git for Windows can check out patch files as CRLF. Normalize only the
    # reviewed patch copy, never the verified upstream source bytes.
    patch=working/'reviewed.patch'
    patch.write_bytes((root/'hardware/leaf-bridge/boat-bridge-v2-expiry.patch').read_bytes().replace(b'\r\n',b'\n'))
    subprocess.run(['git','-C',str(working),'apply','--check',str(patch)],check=True)
    subprocess.run(['git','-C',str(working),'apply',str(patch)],check=True)
    shutil.copy2(working/name,target/name)
shutil.copy2(root/'hardware/leaf-bridge/FreshTelemetry.h',target/'FreshTelemetry.h')
assert hashlib.sha256((target/name).read_bytes()).hexdigest()=='192841d02d9eacc7581a38d6816057f315491032393fcb188731c9d13cea6c0b','Patched producer bytes differ'
source=(target/name).read_text(encoding='utf-8')
# These exact generated producer functions, not rewritten test encoders, run
# against the OpenNav normalized decoder/adapter in the host suite.
codec=source[source.index('void n2kSend127488('):source.index('// ======== Switch PGNs ========')]
assert 'd[1]=0x02' in codec and 'expiryMask' in codec
(target/'FirmwareCodec.inc').write_text(codec,encoding='utf-8',newline='\n')
(target/'SOURCE_INFO.txt').write_text(f'Boat source {revision}\nOriginal SHA256 {expected}\nPatched SHA256 {hashlib.sha256((target/name).read_bytes()).hexdigest()}\nNo physical acceptance or automatic flashing.\n',encoding='utf-8',newline='\n')
print(target)
