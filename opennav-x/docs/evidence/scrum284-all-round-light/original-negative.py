from pathlib import Path
import json,importlib.util,subprocess,os,shutil,time,hashlib
r=Path.cwd();out=r/'.local/original-negative';out.mkdir(exist_ok=True);src=r/'.local/proof2/core'
for p in src.glob('*.inc'):shutil.copyfile(p,out/p.name)
s=importlib.util.spec_from_file_location('extract',r/'tools/verify-anchor-loader.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
original=(r.parent/'skager-product-fidelity/upstream/OpenCPN/libs/s52plib/src/s52plib.cpp').read_text();bodies=''.join(m.function(original,n) for n in ('int s52plib::RenderCARC_GLSL(','int s52plib::RenderCARC_VBO('));(out/'fan-methods.inc').write_text(bodies+bodies.replace('s52plib::RenderCARC_','s52plib::StockCARC_'))
receipt=json.loads((r/'.local/proof2/receipt.json').read_text())[0];cmd=[s.replace(str(src),str(out)) for s in receipt['command']];c=subprocess.run(cmd,text=True,capture_output=True);(out/'compile.log').write_text(c.stdout+c.stderr);c.check_returncode()
wx=Path('/home/standard/Projects/X-nav/.local/sysroot/usr');env={**os.environ,'LD_LIBRARY_PATH':str(wx/'lib'),'GDK_BACKEND':'x11','GSETTINGS_BACKEND':'memory','NO_AT_BRIDGE':'1','LIBGL_ALWAYS_SOFTWARE':'1'};env.pop('WAYLAND_DISPLAY',None);display=next(n for n in range(340,400) if not Path(f'/tmp/.X{n}-lock').exists());env['DISPLAY']=f':{display}'
with (out/'xvfb.log').open('w') as log:
 server=subprocess.Popen([str(wx/'bin/Xvfb'),env['DISPLAY'],'-screen','0','1024x768x24','-nolisten','tcp'],env=env,stdout=log,stderr=log)
 try:
  time.sleep(.5);assert server.poll() is None
  result=subprocess.run([str(out/'fan-test'),str(out)],env=env,text=True,capture_output=True,timeout=60);(out/'run.log').write_text(result.stdout+result.stderr)
  assert result.returncode==1,result.stdout+result.stderr
  (out/'negative.json').write_text(json.dumps({'source':'Original pinned core RenderCARC GLSL/VBO bodies substituted in the same focused fixture','sourceMethodsSha256':hashlib.sha256(bodies.encode()).hexdigest(),'exit':result.returncode,'output':result.stdout+result.stderr,'command':cmd},indent=2)+'\n');print(result.stdout+result.stderr)
 finally:server.terminate();server.wait(timeout=5)
