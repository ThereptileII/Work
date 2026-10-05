from pathlib import Path
import subprocess,shutil,json,hashlib,tarfile,io
root=Path.cwd();out=root/'.local/lights';core=out/'verified-core';private=out/'verified-private'
core.mkdir()
pin=json.loads((root/'upstream.lock.json').read_text())['commit']
raw=subprocess.check_output(['git','-C','../scrum259-linux-6dafd29/upstream/OpenCPN','archive',pin]);tarfile.open(fileobj=io.BytesIO(raw)).extractall(core,filter='data')
subprocess.run(['git','init','-q'],cwd=core,check=True)
subprocess.run(['git','add','.'],cwd=core,check=True)
patches=['xnav','regression-tests','ais-transport','chart-presentation','maintained-curl','download-trust','wxcurl-trust','peer-response-buffer','peer-unavailable']
receipt={}
for name in patches:
 p=root/'patches'/('opencpn-5.12.4-'+name+'.patch');data=p.read_bytes()
 subprocess.run(['git','apply','--check','--index','-'],input=data,cwd=core,check=True)
 subprocess.run(['git','apply','--index','-'],input=data,cwd=core,check=True)
 receipt[p.name]=hashlib.sha256(data).hexdigest()
shutil.copytree('/home/standard/Projects/X-nav-worktrees/scrum259-adapter-preparation/.local/pinned-source',private)
subprocess.run(['git','init','-q'],cwd=private,check=True)
subprocess.run(['git','add','.'],cwd=private,check=True)
for name in ['ocharts-skager-presentation.patch','ocharts-wxcurl-trust.patch']:
 p=root/'patches'/name;data=p.read_bytes()
 subprocess.run(['git','apply','--check','--index','-'],input=data,cwd=private,check=True)
 subprocess.run(['git','apply','--index','-'],input=data,cwd=private,check=True)
 receipt[name]=hashlib.sha256(data).hexdigest()
(out/'patch-proof.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('All nine core and both private patch series apply in isolated source copies')
