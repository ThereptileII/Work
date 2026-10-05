#!/usr/bin/env python3
"""Focused original trigger and actual wx/ocpnDC Mesa warning methods; no app launch."""
import argparse,hashlib,importlib.util,json,os,re,shlex,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('extract',ROOT/'tools/verify-anchor-loader.py');ex=importlib.util.module_from_spec(spec);spec.loader.exec_module(ex)
def main():
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--upstream',type=Path,required=True);p.add_argument('--wx-prefix',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
 current=(a.source/'gui/src/chcanv.cpp').read_text();stock=(a.upstream/'gui/src/chcanv.cpp').read_text();identities={}
 for sig in ['emboss_data *ChartCanvas::EmbossOverzoomIndicator(', 'void ChartCanvas::SetOverzoomFont(', 'void ChartCanvas::CreateOZEmbossMapData(']:
  b=ex.function(stock,sig);assert b==ex.function(current,sig),sig;identities[sig]=hashlib.sha256(b.encode()).hexdigest()
 (out/'trigger.inc').write_text(ex.function(current,'emboss_data *ChartCanvas::EmbossOverzoomIndicator('))
 for rel in ['gui/src/chcanv.cpp','gui/src/glChartCanvas.cpp']:
  t=(a.source/rel).read_text();start=t.index('  auto *overzoom = ');end=t.index('\n#else',start);block=t[start:end];assert block.count('EmbossOverzoomIndicator(')==1 and block.count('DrawEmboss(')==1
  (out/(Path(rel).stem+'-caller.inc')).write_text(block)
  if rel.endswith('/chcanv.cpp'):(out/'caller-test.inc').write_text(block.replace('EmbossOverzoomIndicator(dc)','Invoke()').replace('*this','canvas'))
 own=(ROOT/'src/integration/ChartPresentation.cpp').read_text();(out/'warning.inc').write_text(ex.function(own,'bool DrawChartOverzoomWarning('))
 controls=(ROOT/'src/ui/Controls.cpp').read_text();(out/'font.inc').write_text(ex.function(controls,'wxFont UiFontWeight('))
 dc=(a.upstream/'gui/src/ocpndc.cpp').read_text();sigs=['void ocpnDC::SetGLStipple(','void ocpnDC::SetPen(','void ocpnDC::SetBrush(','void ocpnDC::SetFont(','void ocpnDC::SetTextForeground(','const wxPen &ocpnDC::GetPen(','const wxBrush &ocpnDC::GetBrush(','const wxFont &ocpnDC::GetFont(','bool ocpnDC::ConfigurePen(','bool ocpnDC::ConfigureBrush(','void ocpnDC::drawrrhelperGLES2(','void ocpnDC::DrawRoundedRectangle(','void ocpnDC::DrawLine(','void ocpnDC::DrawText(']
 (out/'dc.inc').write_text('\n'.join(ex.function(dc,s) for s in sigs))
 shader=(a.upstream/'gui/include/gui/shaders.h').read_text();(out/'shader-class.inc').write_text(ex.function(shader,'class GLShaderProgram {')+';\n')
 shaders=(a.upstream/'gui/src/shaders.cpp').read_text();parts=[]
 start=shaders.index('const GLchar *preamble =\n');end=shaders.index(';',start)+1;(out/'preamble.inc').write_text(shaders[start:end])
 for n in ['color_tri_vertex_shader_source','color_tri_fragment_shader_source','texture_2D_vertex_shader_source','texture_2D_fragment_shader_source']:
  start=shaders.index('static const GLchar *'+n);end=re.search(r'^    "[^\n]*";\s*$',shaders[start:],re.M);assert end;parts.append(shaders[start:start+end.end()])
 (out/'shaders.inc').write_text('\n'.join(parts))
 wx=[str(a.wx_prefix/'bin/wx-config'),'--prefix='+str(a.wx_prefix)];flags=shlex.split(subprocess.check_output(wx+['--cxxflags','--libs','core,base,gl'],text=True))
 cmd=['c++','-std=c++17','-O2','-DOPENNAV_X','-DocpnUSE_GL','-DocpnUSE_GLSL','-I'+str(a.wx_prefix/'include'),'-I'+str(ROOT/'src'),'-I'+str(out),'-I'+str(a.upstream/'libs/s52plib/src'),'-I'+str(a.upstream/'libs/geoprim/src'),str(ROOT/'tests/overzoom_warning_test.cpp'),*flags,'-lGL','-lGLEW','-o',str(out/'test')]
 env={**os.environ,'LD_LIBRARY_PATH':str(a.wx_prefix/'lib'),'GDK_BACKEND':'x11','GSETTINGS_BACKEND':'memory','NO_AT_BRIDGE':'1','LIBGL_ALWAYS_SOFTWARE':'1'};env.pop('WAYLAND_DISPLAY',None)
 r=subprocess.run(cmd,env=env,text=True,capture_output=True);(out/'compile.log').write_text(r.stdout+r.stderr);r.check_returncode()
 env['DISPLAY']=':'+str(next(n for n in range(340,400) if not Path(f'/tmp/.X{n}-lock').exists()));server=subprocess.Popen([str(a.wx_prefix/'bin/Xvfb'),env['DISPLAY'],'-screen','0','1024x768x24','-nolisten','tcp'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
 try:
  time.sleep(.4);assert server.poll() is None
  r=subprocess.run([str(out/'test'),str(out)],env=env,text=True,capture_output=True,timeout=30);(out/'run.log').write_text(r.stdout+r.stderr)
  report={'command':cmd,'exit':r.returncode,'unchangedSourceMethods':identities,'files':{str(q):hashlib.sha256(q.read_bytes()).hexdigest() for q in [ROOT/'src/integration/ChartPresentation.cpp',ROOT/'tests/overzoom_warning_test.cpp',*out.glob('*.inc'),out/'test']}}
  (out/'receipt.json').write_text(json.dumps(report,indent=2)+'\n');print(r.stdout+r.stderr);r.check_returncode()
 finally:server.terminate();server.wait(timeout=5)
if __name__=='__main__':main()
