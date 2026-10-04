#!/usr/bin/env python3
"""Focused actual LC traversal and wx/Mesa cable paint; no OpenCPN launch."""
import argparse,hashlib,importlib.util,json,os,re,shlex,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('extract',ROOT/'tools/verify-anchor-loader.py');extract=importlib.util.module_from_spec(spec);spec.loader.exec_module(extract)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--private-source',type=Path,required=True);p.add_argument('--stock-core-file',type=Path,required=True);p.add_argument('--stock-private-file',type=Path,required=True);p.add_argument('--wx-prefix',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
 wx=[str(a.wx_prefix/'bin/wx-config'),'--prefix='+str(a.wx_prefix)]
 flags=shlex.split(subprocess.check_output(wx+['--cxxflags'],text=True));libs=shlex.split(subprocess.check_output(wx+['--libs','core,base,gl'],text=True));receipts=[]
 for name,source,guard in [('core',a.source,'OPENNAV_X'),('private',a.private_source,'SKAGER_OCHARTS_ADAPTER')]:
  folder=out/name;folder.mkdir(exist_ok=True);cpp=source/'libs/s52plib/src/s52plib.cpp';text=cpp.read_text()
  if name=='private':
   config=(source/'config.h.in').read_text().replace('#cmakedefine OPENGL_FOUND','#define OPENGL_FOUND 1')
   for key,value in {'API_VERSION':17,'PROJECT_VERSION_MAJOR':2,'PROJECT_VERSION_MINOR':0,'PROJECT_VERSION_PATCH':29,'PROJECT_VERSION_TWEAK':0}.items():config=config.replace('@'+key+'@',str(value))
   (folder/'config.h').write_text(config)
  signatures=['void s52plib::draw_lc_poly(']
  methods={s:extract.function(text,s) for s in signatures}
  stock_path=a.stock_core_file if name=='core' else a.stock_private_file
  stock_methods={s:extract.function(stock_path.read_text(),s) for s in signatures}
  old_upload=subprocess.check_output(['git','-C',str(ROOT),'show','48c2f8b:src/integration/ChartCaFan.h'],text=True)
  (folder/'ca-upload-before.inc').write_text(extract.function(old_upload,'inline bool DrawCaFanGL(').replace('DrawCaFanGL(', 'DrawCaFanGLBefore(',1))
  (folder/'cable-methods.inc').write_text(''.join(methods.values())+''.join(body.replace('s52plib::draw_lc_poly', 's52plib::stock_lc_poly',1) for body in stock_methods.values()))
  matrix_start=text.index('  mat4x4 m;',text.index('void PrepareS52ShaderUniforms('));matrix_end=text.index('  mat4x4 I;',matrix_start)
  matrix=text[matrix_start:matrix_end]
  (folder/'matrix.inc').write_text('void ActualMatrix(GLfloat* out,int width,int height,double rotation) { struct VP { int pix_width,pix_height; double rotation; } v{width,height,rotation}; auto* vp=&v;\n'+matrix+'std::copy_n(&Q[0][0],16,out); }\n')
  state=(ROOT/'tests/chart_ca_fan_state.inc').read_text().replace('DrawCaFanGL(tile,S52texture_2D_shader_program,400,400)','DrawCableWaveGL(tile,S52texture_2D_shader_program,line.programId(),400,400)')
  (folder/'cable-state.inc').write_text(state)
  shader=(source/'libs/s52plib/src/s52shaders.cpp').read_text();shader_bodies={}
  for n in ['S52texture_2D_vertex_shader_source','S52texture_2D_fragment_shader_source','S52color_tri_vertex_shader_source','S52color_tri_fragment_shader_source']:
   start=shader.index('static const GLchar *'+n+' =');end=shader.index('\n";',start)+3 if '\n";' in shader[start:] else 0
   # The pinned C++ string ends at the line containing the closing quoted semicolon.
   match=re.search(r'^    "[^\n]*";\s*$',shader[start:],re.M);assert match,n
   shader_bodies[n]=shader[start:start+match.end()]+'\n'
  (folder/'shaders.inc').write_text(''.join(shader_bodies.values()))
  cmd=['c++','-std=c++17','-O3','-Wall','-Wextra','-Wno-unused-parameter','-Wno-deprecated-copy','-D'+guard,'-DocpnUSE_GL',*flags,'-I'+str(a.wx_prefix/'include'),'-I'+str(ROOT/'src'),'-I'+str(folder),*['-I'+str(source/x) for x in ('libs/s52plib/src','libs/geoprim/src','libs/pugixml','libs/glu')],str(ROOT/'tests/chart_cable_wave_test.cpp'),str(a.source/'libs/geoprim/src/line_clip.cpp'),*libs,'-lGL','-lGLEW','-o',str(folder/'cable-test')]
  prefix=(ROOT/'tests/chart_cable_wave_test.cpp').read_text().split('#include \"cable-methods.inc\"')[0]+'#include \"cable-methods.inc\"\n'
  (folder/'no-gl.cpp').write_text(prefix)
  no_gl=cmd[:cmd.index(str(ROOT/'tests/chart_cable_wave_test.cpp'))]+['-fsyntax-only',str(folder/'no-gl.cpp')]
  no_gl.remove('-DocpnUSE_GL')
  if name=='private':
   headers=folder/'no-gl-headers';headers.mkdir(exist_ok=True)
   (headers/'config.h').write_text(config.replace('#define OPENGL_FOUND 1','/* OPENGL_FOUND unavailable */'))
   no_gl.insert(1,'-I'+str(headers))
  check=subprocess.run(no_gl,text=True,capture_output=True);(folder/'no-gl.log').write_text(check.stdout+check.stderr);check.check_returncode()
  result=subprocess.run(cmd,text=True,capture_output=True);(folder/'compile.log').write_text(result.stdout+result.stderr);result.check_returncode()
  receipts.append({'name':name,'sourceSha256':sha(cpp),'stockSourceSha256':sha(stock_path),'stockMethods':{k:hashlib.sha256(v.encode()).hexdigest() for k,v in stock_methods.items()},'matrixPrefixSha256':hashlib.sha256(matrix.encode()).hexdigest(),'helperSha256':sha(ROOT/'src/integration/ChartCableWave.h'),'uploadHelperSha256':sha(ROOT/'src/integration/ChartCaFan.h'),'fixtureSha256':sha(ROOT/'tests/chart_cable_wave_test.cpp'),'methods':{k:hashlib.sha256(v.encode()).hexdigest() for k,v in methods.items()},'shaderMethods':{k:hashlib.sha256(v.encode()).hexdigest() for k,v in shader_bodies.items()},'command':cmd})
 env={**os.environ,'LD_LIBRARY_PATH':str(a.wx_prefix/'lib'),'GDK_BACKEND':'x11','GSETTINGS_BACKEND':'memory','NO_AT_BRIDGE':'1','LIBGL_ALWAYS_SOFTWARE':'1'};env.pop('WAYLAND_DISPLAY',None)
 display=next(n for n in range(340,400) if not Path(f'/tmp/.X{n}-lock').exists());env['DISPLAY']=f':{display}'
 log=(out/'xvfb.log').open('w');server=subprocess.Popen([str(a.wx_prefix/'bin/Xvfb'),env['DISPLAY'],'-screen','0','1024x768x24','-nolisten','tcp'],env=env,stdout=log,stderr=log)
 try:
  time.sleep(.5);assert server.poll() is None
  for r in receipts:
   folder=out/r['name'];result=subprocess.run([str(folder/'cable-test'),str(folder)],env=env,text=True,capture_output=True,timeout=60);(folder/'run.log').write_text(result.stdout+result.stderr);r.update(exit=result.returncode,stdout=result.stdout,executableSha256=sha(folder/'cable-test'));(out/'receipt.json').write_text(json.dumps(receipts,indent=2)+'\n');print(r['name'],result.stdout+result.stderr,flush=True);result.check_returncode()
 finally:server.terminate();server.wait(timeout=5);log.close()
if __name__=='__main__':main()
