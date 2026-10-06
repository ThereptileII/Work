#!/usr/bin/env python3
"""Exercise the production recapture callback and owned GTK surface natively."""
import argparse,hashlib,json,os,shlex,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--wx-config',type=Path,required=True);p.add_argument('--wx-prefix',type=Path,required=True);p.add_argument('--negative-no-hook',action='store_true');p.add_argument('--skip-lifecycle',action='store_true',help='The existing lifecycle executable has already run in this job');a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
def method(text,start):
 begin=text.index(start);end=text.index('{',begin)+1;depth=1
 while depth:depth+=(text[end]=='{')-(text[end]=='}');end+=1
 return text[begin:end]
patch=(ROOT/'patches/opencpn-5.12.4-xnav.patch').read_text();added='\n'.join(line[1:] for line in patch.splitlines() if line.startswith('+') and not line.startswith('+++'))
frame=method(added,'void MyFrame::OnRecaptureTimer(')
assert frame.index('Raise();')<frame.index('opennav::AfterFrameRecapture();')
assert '#if defined(OPENNAV_X) && defined(__WXGTK__)' in frame
if a.negative_no_hook:frame=frame.replace('  opennav::AfterFrameRecapture();','  /* negative control: omit hook */')
inputs={'frame-recapture.inc':frame,'integration-recapture.inc':method((ROOT/'src/integration/OpenCPNIntegration.cpp').read_text(),'void AfterFrameRecapture()'),'shell-transient.inc':method((ROOT/'src/ui/Shell.cpp').read_text(),'bool Shell::HasTransientSurface() const'),'shell-restack.inc':method((ROOT/'src/ui/Shell.cpp').read_text(),'void Shell::RestackChartControls()')}
for name,text in inputs.items():(out/name).write_text(text+'\n')
config=[str(a.wx_config),'--prefix='+str(a.wx_prefix)];flags=shlex.split(subprocess.check_output(config+['--cxxflags'],text=True));libs=shlex.split(subprocess.check_output(config+['--libs','core,base'],text=True));gtk=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','gtk+-3.0','x11'],text=True))
commands=[]
names=['frame_recapture_test'] if a.skip_lifecycle else ['frame_recapture_test','floating_surface_test']
for name in names:
 cmd=['g++','-std=c++17','-DOPENNAV_X',*flags,'-I'+str(ROOT/'src'),'-I'+str(out),str(ROOT/'tests'/(name+'.cpp')),str(ROOT/'src/ui/FloatingSurface.cpp'),*gtk,*libs,'-o',str(out/name)];commands.append(cmd)
 r=subprocess.run(cmd,capture_output=True,text=True);(out/(name+'-compile.log')).write_text(r.stdout+r.stderr);r.check_returncode()
env=dict(os.environ,LD_LIBRARY_PATH=str(a.wx_prefix/'lib'),GDK_BACKEND='x11');env.pop('WAYLAND_DISPLAY',None)
display=198
while Path(f'/tmp/.X{display}-lock').exists():display+=1
env['DISPLAY']=':'+str(display);x=subprocess.Popen([str(a.wx_prefix/'bin/Xvfb'),env['DISPLAY'],'-screen','0','1280x800x24','-nolisten','tcp'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
try:
 time.sleep(.7);runs=[]
 for name in names:
  r=subprocess.run([str(out/name)],cwd=out,env=env,capture_output=True,text=True,timeout=20);runs.append({'name':name,'exit_code':r.returncode,'output':r.stdout+r.stderr});print(name,r.returncode,r.stdout+r.stderr)
finally:x.terminate();x.wait(timeout=5)
result={'scope':'Offline Linux Xvfb native GTK surfaces; not full application, Windows or boat acceptance','negative_no_hook':a.negative_no_hook,'commands':commands,'extracted_method_sha256':{name:hashlib.sha256(text.encode()).hexdigest() for name,text in inputs.items()},'runs':runs}
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');raise SystemExit(any(r['exit_code'] for r in runs))
