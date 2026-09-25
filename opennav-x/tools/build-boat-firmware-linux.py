#!/usr/bin/env python3
"""Compile the reviewed C6 firmware in isolated tools. Never upload/flash."""
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import urllib.request

assert sys.platform=='linux','Firmware CI builder currently qualified on Linux'
root=Path(__file__).resolve().parents[1]
tools=root/'.local/arduino-cli';tools.mkdir(parents=True,exist_ok=True)
cli=tools/'arduino-cli'
archive=tools/'arduino-cli-1.5.1.tar.gz'
if not archive.exists():
    request=urllib.request.Request('https://github.com/arduino/arduino-cli/releases/download/v1.5.1/arduino-cli_1.5.1_Linux_64bit.tar.gz',headers={'User-Agent':'OpenNavX-build'})
    archive.write_bytes(urllib.request.urlopen(request,timeout=120).read())
data=archive.read_bytes()
assert hashlib.sha256(data).hexdigest()=='28a8e119c498a25607821c36cb2dc49e8463941b261a0d99091baa7bc692dd2b','Arduino CLI checksum mismatch'
with tarfile.open(fileobj=io.BytesIO(data),mode='r:gz') as compressed:
    cli.write_bytes(compressed.extractfile('arduino-cli').read());cli.chmod(0o755)
store=root/'.local/boat-arduino';store.mkdir(exist_ok=True)
config=tools/'config.json'
config.write_text(json.dumps({'board_manager':{'additional_urls':['https://espressif.github.io/arduino-esp32/package_esp32_index.json']},
  'directories':{'data':str(store/'data'),'downloads':str(store/'downloads'),'user':str(store/'user')}}))
def run(*args):subprocess.run([str(cli),'--config-file',str(config),*args],check=True,cwd=root)
subprocess.run([sys.executable,str(root/'tools/prepare-boat-firmware.py')],check=True)
run('core','update-index')
index=json.loads((store/'data/library_index.json').read_text())
library=next(p for p in index['libraries'] if p['name']=='WebSockets' and p['version']=='2.7.2')
assert library['checksum']=='SHA-256:cace4969401d5a015d3558fd501c7449a946beaa90b7313cc5b262014b54ae7e'
run('core','install','esp32:esp32@3.2.1')
run('lib','install','WebSockets@2.7.2')
run('compile','--fqbn','esp32:esp32:XIAO_ESP32C6','--jobs','2',
    '--build-path',str(root/'build/boat-firmware-compiled'),
    str(root/'build/boat-firmware/Nissan_ev_to_NMEA_and_wifi'))
evidence=root/'evidence/local';evidence.mkdir(parents=True,exist_ok=True)
info={'arduino_cli':'1.5.1','core':'esp32:esp32@3.2.1','board':'esp32:esp32:XIAO_ESP32C6',
      'websockets':'2.7.2','physical_validation':'NOT RUN; compile and host tests only',
      'firmware_source':(root/'build/boat-firmware/Nissan_ev_to_NMEA_and_wifi/SOURCE_INFO.txt').read_text(),
      'binary_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (root/'build/boat-firmware-compiled').glob('*.bin')}}
(evidence/'boat-firmware-build.json').write_text(json.dumps(info,indent=2)+'\n')
print(json.dumps(info,indent=2))
