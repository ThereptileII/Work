#!/usr/bin/env python3
"""Package the isolated native integration for the side-by-side Alpha installer."""
import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import zipfile
ROOT = Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser()
p.add_argument('--preview',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--candidate',action='store_true',help='Disposable lifecycle qualification only; not a release allowlist')
a=p.parse_args()
if os.name != 'nt': raise SystemExit('Build the installer with native NSIS on Windows')
if a.output.exists(): raise SystemExit('Use a fresh installer output directory')
manifest=json.loads((ROOT/'installer/windows/compatibility.json').read_text())
if a.candidate:
    if os.environ.get('GITHUB_ACTIONS') != 'true': raise SystemExit('Candidate allowlist only permitted in disposable CI')
    manifest['supportedOpenCpn']=[{
        'version':'5.12.4','arch':'x86',
        'executableSha256':'7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c',
        'upstreamCommit':'37fd0cddb7334fe489e9f18aa163977a9c5c84f7',
        'qualification':'candidate - not release acceptance'}]
if not manifest['supportedOpenCpn']: raise SystemExit('No accepted OpenCPN configuration; release installer refused')
a.output.mkdir(parents=True)
preview=a.preview.resolve()
with zipfile.ZipFile(a.output/'payload.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    records=[]
    for directory in ('app','docs'):
        for f in sorted((preview/directory).rglob('*')):
            if not f.is_file() or f.name=='OPENNAV_PORTABLE_PREVIEW': continue
            path=f.relative_to(preview).as_posix()
            data=f.read_bytes();z.writestr(path,data)
            records.append({'path':path,'sha256':hashlib.sha256(data).hexdigest()})
commit=os.environ['GITHUB_SHA']
version=re.search(r'Version\[\] = "([^"]+)"',(ROOT/'src/application/Version.h').read_text()).group(1)
package={'schema':1,'version':version,'commit':commit,
         'payloadSha256':hashlib.sha256((a.output/'payload.zip').read_bytes()).hexdigest(),
         'supportedOpenCpn':manifest['supportedOpenCpn'],'files':records,
         'qualification':'candidate' if a.candidate else 'accepted'}
record=a.output/'package.json';record.write_text(json.dumps(package,indent=2)+'\n',encoding='utf-8')
compiler=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'NSIS/makensis.exe'
if not compiler.exists(): raise SystemExit('Native NSIS compiler missing')
setup=a.output/'OpenNavX-Alpha1-Setup.exe'
subprocess.run([str(compiler),'/V3',f'/DOUTPUT={setup.resolve()}',
               f'/DPACKAGE={a.output.resolve()}',
               f'/DENGINE={ROOT / "installer/windows/Lifecycle.ps1"}',
               '/DMANIFEST_SHA256='+hashlib.sha256(record.read_bytes()).hexdigest(),
               str(ROOT/'installer/windows/AlphaSetup.nsi')],check=True)
(setup.with_suffix('.exe.sha256')).write_text(hashlib.sha256(setup.read_bytes()).hexdigest()+'  '+setup.name+'\n')
print(setup)
