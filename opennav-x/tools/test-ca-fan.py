#!/usr/bin/env python3
"""Focused actual CA methods and wx/Mesa alpha paint; no OpenCPN launch."""
import argparse,hashlib,importlib.util,json,os,re,shlex,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('extract',ROOT/'tools/verify-anchor-loader.py');extract=importlib.util.module_from_spec(spec);spec.loader.exec_module(extract)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--all-round',action='store_true');p.add_argument('--baseline-header',type=Path);p.add_argument('--source',type=Path,required=True);p.add_argument('--private-source',type=Path,required=True);p.add_argument('--stock-core-source',type=Path,required=True);p.add_argument('--stock-private-source',type=Path,required=True);p.add_argument('--wx-prefix',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
 if a.all_round:
  assert a.baseline_header and a.baseline_header.is_file()
  baseline=a.baseline_header.read_text();current=(ROOT/'src/integration/ChartCaFan.h').read_text()
  assert baseline[baseline.index('inline bool DrawCaFanGL'):]==current[current.index('inline bool DrawCaFanGL'):], 'Shared optional-matrix upload changed'
  html=(ROOT/'docs/design/prototype/index.html').read_text();assert '.sector-arc{fill:none;stroke:var(--sector-stroke);stroke-width:1.2;stroke-opacity:.8;vector-effect:non-scaling-stroke}' in html
 wx=[str(a.wx_prefix/'bin/wx-config'),'--prefix='+str(a.wx_prefix)]
 flags=shlex.split(subprocess.check_output(wx+['--cxxflags'],text=True));libs=shlex.split(subprocess.check_output(wx+['--libs','core,base,gl'],text=True));receipts=[]
 for name,source,guard in [('core',a.source,'OPENNAV_X'),('private',a.private_source,'SKAGER_OCHARTS_ADAPTER')]:
  folder=out/name;folder.mkdir(exist_ok=True);cpp=source/'libs/s52plib/src/s52plib.cpp';text=cpp.read_text()
  signatures=['int s52plib::RenderCARC_GLSL(','int s52plib::RenderCARC_VBO(']
  methods={s:extract.function(text,s) for s in signatures}
  stock_path=(a.stock_core_source if name=='core' else a.stock_private_source)/'libs/s52plib/src/s52plib.cpp'
  stock_methods={s:extract.function(stock_path.read_text(),s) for s in signatures}
  (folder/'fan-methods.inc').write_text(''.join(methods.values())+''.join(body.replace('s52plib::RenderCARC_', 's52plib::StockCARC_',1) for body in stock_methods.values()))
  matrix_start=text.index('  mat4x4 m;',text.index('void PrepareS52ShaderUniforms('));matrix_end=text.index('  mat4x4 I;',matrix_start)
  matrix=text[matrix_start:matrix_end]
  (folder/'fan-matrix.inc').write_text('void ActualMatrix(GLfloat* out,int width,int height,double rotation) { struct VP { int pix_width,pix_height; double rotation; } v{width,height,rotation}; auto* vp=&v;\n'+matrix+'std::copy_n(&Q[0][0],16,out); }\n')
  state=(ROOT/'tests/chart_ca_fan_state.inc').read_text()
  if a.all_round:
   original=extract.function(baseline,'class CaFanTile').replace('class CaFanTile','class OriginalFanTile',1)
   (folder/'original-fan-tile.inc').write_text('namespace opennav::integration {\n'+original+';\n}\n')
   state=state.replace('Check(DrawCaFanGL(tile,S52texture_2D_shader_program,400,400),"hostile state draw")','Check(([&]{Light light;return lib.RenderCARC_GLSL(&light.node,&light.rule)==1;})(),"hostile actual all-round method state draw")')
  (folder/'fan-state-check.inc').write_text(state)
  cond=(source/'libs/s52plib/src/s52cnsy.cpp').read_text();cond_signatures=('bool GetDoubleAttr(S57Obj *obj,','bool GetStringAttr(S57Obj *obj,','static int _parseList(','wxString _selSYcol(','static void *LIGHTS06(void *param)\n')
  (folder/'conditional.inc').write_text(extract.function((a.source/'gui/src/s57obj.cpp').read_text(),'int S57Obj::GetAttributeIndex(')+''.join(extract.function(cond,s) for s in cond_signatures))
  shader=(source/'libs/s52plib/src/s52shaders.cpp').read_text();shader_bodies={}
  for n in ['S52texture_2D_vertex_shader_source','S52texture_2D_fragment_shader_source','S52ring_vertex_shader_source','S52ring_fragment_shader_source']:
   start=shader.index('static const GLchar *'+n+' =');end=shader.index('\n";',start)+3 if '\n";' in shader[start:] else 0
   # The pinned C++ string ends at the line containing the closing quoted semicolon.
   match=re.search(r'^    "[^\n]*";\s*$',shader[start:],re.M);assert match,n
   shader_bodies[n]=shader[start:start+match.end()]+'\n'
  (folder/'shaders.inc').write_text(''.join(shader_bodies.values()))
  cmd=['c++','-std=c++17','-O2','-Wall','-Wextra','-Wno-unused-parameter','-Wno-deprecated-copy','-D'+guard,'-DocpnUSE_GL',*flags,'-I'+str(a.wx_prefix/'include'),'-I'+str(ROOT/'src'),'-I'+str(folder),*['-I'+str(source/x) for x in ('libs/s52plib/src','libs/geoprim/src','libs/pugixml','libs/glu')],str(ROOT/('tests/chart_ca_all_round_test.cpp' if a.all_round else 'tests/chart_ca_fan_test.cpp')),str(a.source/'libs/geoprim/src/bbox.cpp'),*libs,'-lGL','-lGLEW','-o',str(folder/'fan-test')]
  result=subprocess.run(cmd,text=True,capture_output=True);(folder/'compile.log').write_text(result.stdout+result.stderr);result.check_returncode()
  receipts.append({'name':name,'sourceSha256':sha(cpp),'stockSourceSha256':sha(stock_path),'stockMethods':{k:hashlib.sha256(v.encode()).hexdigest() for k,v in stock_methods.items()},'matrixPrefixSha256':hashlib.sha256(matrix.encode()).hexdigest(),'helperSha256':sha(ROOT/'src/integration/ChartCaFan.h'),'allRoundHelperSha256':sha(ROOT/'src/integration/ChartCaAllRound.h') if a.all_round else None,'baselineHelperSha256':sha(a.baseline_header) if a.all_round else None,'fixtureSha256':sha(ROOT/('tests/chart_ca_all_round_test.cpp' if a.all_round else 'tests/chart_ca_fan_test.cpp')),'methods':{k:hashlib.sha256(v.encode()).hexdigest() for k,v in methods.items()},'shaderMethods':{k:hashlib.sha256(v.encode()).hexdigest() for k,v in shader_bodies.items()},'command':cmd})
 env={**os.environ,'LD_LIBRARY_PATH':str(a.wx_prefix/'lib'),'GDK_BACKEND':'x11','GSETTINGS_BACKEND':'memory','NO_AT_BRIDGE':'1','LIBGL_ALWAYS_SOFTWARE':'1'};env.pop('WAYLAND_DISPLAY',None)
 display=next(n for n in range(340,400) if not Path(f'/tmp/.X{n}-lock').exists());env['DISPLAY']=f':{display}'
 log=(out/'xvfb.log').open('w');server=subprocess.Popen([str(a.wx_prefix/'bin/Xvfb'),env['DISPLAY'],'-screen','0','1024x768x24','-nolisten','tcp'],env=env,stdout=log,stderr=log)
 try:
  time.sleep(.5);assert server.poll() is None
  for r in receipts:
   folder=out/r['name'];result=subprocess.run([str(folder/'fan-test'),str(folder)],env=env,text=True,capture_output=True,timeout=60);(folder/'run.log').write_text(result.stdout+result.stderr);r.update(exit=result.returncode,stdout=result.stdout,executableSha256=sha(folder/'fan-test'));(out/'receipt.json').write_text(json.dumps(receipts,indent=2)+'\n');print(r['name'],result.stdout+result.stderr,flush=True);result.check_returncode()
 finally:server.terminate();server.wait(timeout=5);log.close()
if __name__=='__main__':main()
