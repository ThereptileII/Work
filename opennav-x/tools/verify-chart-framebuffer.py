#!/usr/bin/env python3
"""Exercise the actual GL cache repair against recorded failing dimensions."""
import argparse,hashlib,json,os,shlex,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--wx-config',type=Path,required=True);p.add_argument('--wx-prefix',type=Path,required=True);p.add_argument('--negative-no-rebuild',action='store_true');a=p.parse_args()
out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);source=a.source.resolve()/'gui/src/glChartCanvas.cpp';text=source.read_text()
start=text.index('bool glChartCanvas::PrepareChartFramebuffer(');body=text.index('{',start);end=body+1;depth=1
while depth:depth+=(text[end]=='{')-(text[end]=='}');end+=1
method=text[start:end]
render=text[text.index('void glChartCanvas::Render() {'):]
assert '#ifdef OPENNAV_X\n  if (opennav::IsXNav())\n    presentation_fbo_ready = PrepareChartFramebuffer(gl_width, gl_height);\n#endif' in render
assert 'if (m_b_BuiltFBO && !bpost_hilite && presentation_fbo_ready' in render
assert render.index('PrepareChartFramebuffer(')<render.index('if (m_b_BuiltFBO && !bpost_hilite')
assert 'GL_CLAMP' not in method
if a.negative_no_rebuild:method=method.replace('    BuildFBO();','    (void)0;')
(out/'chart-framebuffer-method.inc').write_text(method+'\n')
config=[str(a.wx_config),'--prefix='+str(a.wx_prefix)];flags=shlex.split(subprocess.check_output(config+['--cxxflags'],text=True));libs=shlex.split(subprocess.check_output(config+['--libs','base'],text=True))
cmd=['g++','-std=c++17','-Wall','-Wextra','-Werror',*flags,'-I'+str(out),str(ROOT/'tests/chart_framebuffer_size_test.cpp'),*libs,'-o',str(out/'check')]
build=subprocess.run(cmd,capture_output=True,text=True);(out/'compile.log').write_text(build.stdout+build.stderr)
if build.returncode:print(build.stderr);build.check_returncode()
env=dict(os.environ,LD_LIBRARY_PATH=str(a.wx_prefix/'lib'));run=subprocess.run([str(out/'check')],env=env,capture_output=True,text=True)
evidence={'sourceSha256':hashlib.sha256(source.read_bytes()).hexdigest(),'methodSha256':hashlib.sha256(method.encode()).hexdigest(),'compileCommand':cmd,'exitCode':run.returncode,'output':run.stdout+run.stderr,'negativeNoRebuild':a.negative_no_rebuild,'verifiedCallerGate':'SKAGER shell only, before FBO cache selection; Standard chart style included','limitations':['Resize/allocation outcomes modeled around verbatim actual method; no GL driver draw','Actual failing Mesa GL sizes independently recorded in baseline API trace','Final integrated GL/Windows/boat validation remains open']}
(out/'result.json').write_text(json.dumps(evidence,indent=2)+'\n');print(run.stdout+run.stderr,end='');run.check_returncode()
