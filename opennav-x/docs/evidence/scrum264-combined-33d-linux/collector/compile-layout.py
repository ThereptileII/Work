from pathlib import Path
import re,shlex,subprocess,json,hashlib,os
build=Path('/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29/build/xnav-linux');p=Path(__file__).resolve().parent;text=(build/'build.ninja').read_text();start=text.index('build CMakeFiles/opencpn.dir/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29/src/integration/ChartPresentation.cpp.o:');block=text[start:text.index('\n\nbuild ',start)]
flags=[]
for field in ('DEFINES','INCLUDES'):flags+=shlex.split(re.search(r'^  '+field+r' = (.+)$',block,re.M)[1])
prefix='/home/standard/Projects/X-nav/.local/sysroot/usr'
libs=shlex.split(subprocess.check_output([prefix+'/bin/wx-config','--prefix='+prefix,'--libs','core,base'],text=True))
command=['/usr/bin/c++','-std=gnu++17','-O3','-Wno-invalid offsetof'.replace(' ', '-'),*flags,str(p/'layout.cpp'),*libs,'-o',str(p/'layout')]
r=subprocess.run(command,text=True,capture_output=True);(p/'output/layout-compile-final.log').write_text(r.stdout+r.stderr)
if r.returncode:raise SystemExit('Layout helper compile failed; see retained log')
out=subprocess.check_output([str(p/'layout')],env=dict(os.environ,LD_LIBRARY_PATH=prefix+'/lib'));json.loads(out);(p/'layout.json').write_bytes(out)
(p/'output/layout-receipt.json').write_text(json.dumps({'command':command,'productionFlagsSourceSha256':hashlib.sha256((build/'build.ninja').read_bytes()).hexdigest(),'sourceSha256':hashlib.sha256((p/'layout.cpp').read_bytes()).hexdigest(),'executableSha256':hashlib.sha256((p/'layout').read_bytes()).hexdigest()},indent=2)+'\n')
print(out.decode())
