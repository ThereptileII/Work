from pathlib import Path
import subprocess,shlex,json,hashlib,re
root=Path.cwd();cache=Path('/home/standard/Projects/X-nav-worktrees/skager-product-fidelity/.local/integrated-fidelity');out=root/'.local/stale-overlay-probe/production';out.mkdir(parents=True,exist_ok=True)
stock=subprocess.check_output(['git','--git-dir='+str(cache/'upstream.git'),'show','37fd0cddb7334fe489e9f18aa163977a9c5c84f7:gui/src/ocpn_frame.cpp'])
source=out/'gui/src/ocpn_frame.cpp';source.parent.mkdir(parents=True,exist_ok=True);source.write_bytes(stock)
p=(root/'patches/opencpn-5.12.4-xnav.patch').read_text();start=p.index('diff --git a/gui/src/ocpn_frame.cpp ');end=p.find('\ndiff --git ',start+1);section=p[start:end if end!=-1 else None];(out/'frame.patch').write_text(section+'\n')
a=subprocess.run(['patch','--batch','-p1','-i',str(out/'frame.patch')],cwd=out,capture_output=True,text=True);(out/'patch.log').write_text(a.stdout+a.stderr);a.check_returncode()
ninja=(cache/'build/build.ninja').read_text();records=[]
for filename in ['FloatingSurface.cpp','Shell.cpp','OpenCPNIntegration.cpp','ocpn_frame.cpp']:
 pattern=r'^build [^\n]*'+re.escape(filename)+r'\.o:[^\n]*\n(?:(?!^build ).*\n)*?'
 blocks=ninja.split('\nbuild ');block=next(b for b in blocks if b.split('\n')[0].split(':')[0].endswith('/'+filename+'.o'))
 vals={line.split(' = ',1)[0].strip():line.split(' = ',1)[1] for line in block.splitlines()[1:] if ' = ' in line}
 cmd=['/usr/bin/c++']+shlex.split(vals['DEFINES'])+shlex.split(vals['FLAGS'])+shlex.split(vals['INCLUDES'])
 replacements=[('/home/standard/Projects/X-nav-worktrees/skager-product-integration/build/integration-source',str(cache/'upstream')),('/home/standard/Projects/X-nav-worktrees/skager-product-integration/build/xnav-linux',str(cache/'build')),('/home/standard/Projects/X-nav-worktrees/skager-product-integration/.local/sysroot','/home/standard/Projects/X-nav/.local/sysroot'),('/home/standard/Projects/X-nav-worktrees/waypoint-touch-regression/src',str(root/'src')),('/home/standard/Projects/X-nav-worktrees/waypoint-touch-regression/tests',str(root/'tests'))]
 for old,new in replacements:cmd=[x.replace(old,new) for x in cmd]
 src=source if filename=='ocpn_frame.cpp' else root/('src/integration/' if filename=='OpenCPNIntegration.cpp' else 'src/ui/')/filename
 obj=out/(filename+'.o');cmd+=['-c',str(src),'-o',str(obj)]
 result=subprocess.run(cmd,capture_output=True,text=True);(out/(filename+'.log')).write_text(result.stdout+result.stderr)
 records.append({'source':str(src),'sourceSha256':hashlib.sha256(src.read_bytes()).hexdigest(),'command':cmd,'exitCode':result.returncode})
 (out/'result.json').write_text(json.dumps(records,indent=2)+'\n');print(filename,result.returncode,flush=True)
 if result.returncode:print(result.stderr);result.check_returncode()
