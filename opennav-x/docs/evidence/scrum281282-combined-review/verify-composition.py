from pathlib import Path
import tempfile,os,subprocess,json,hashlib,importlib.util,shutil
root=Path.cwd();source=root/'build/integration-source';lock=json.loads((root/'upstream.lock.json').read_text());names=['xnav','regression-tests','ais-transport','chart-presentation','maintained-curl','download-trust','wxcurl-trust','peer-response-buffer','peer-unavailable'];report={}
with tempfile.TemporaryDirectory() as d:
 env={**os.environ,'GIT_INDEX_FILE':d+'/index'}
 subprocess.run(['git','-C',str(source),'read-tree',lock['commit']],env=env,check=True)
 for name in names:subprocess.run(['git','-C',str(source),'apply','--cached','-'],env=env,input=(root/f'patches/opencpn-5.12.4-{name}.patch').read_bytes(),check=True)
 subprocess.run(['git','-C',str(source),'diff','--quiet'],env=env,check=True)
 report['core9PatchTree']=subprocess.check_output(['git','-C',str(source),'write-tree'],env=env,text=True).strip()
spec=importlib.util.spec_from_file_location('prep',root/'tools/prepare-ocharts-adapter.py');prep=importlib.util.module_from_spec(spec);spec.loader.exec_module(prep)
cache=root.parent/'scrum259-adapter-preparation/.local/pinned-source';dest=root/'.local/private-reconstructed-three';shutil.copytree(cache,dest);prep.apply_patches(dest,root)
paths=set()
import re
for patch in prep.PATCHES:paths.update(re.findall(r'^\+\+\+ b/(.+)$',(root/patch).read_text(),re.M))
for path in paths:assert (dest/path).read_bytes()==(root/'.local/private'/path).read_bytes(),path
report['private3PatchFiles']={p:hashlib.sha256((dest/p).read_bytes()).hexdigest() for p in sorted(paths)}
(root/'.local/composition.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
