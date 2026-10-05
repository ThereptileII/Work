#!/usr/bin/env python3
"""Compile focused wx tests around verbatim core/private font-establishment blocks."""
import argparse,hashlib,json,os,shlex,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def body(text,start):
    a=text.index(start);op=text.index('{',a);i=op+1;depth=1
    while depth:depth+=(text[i]=='{')-(text[i]=='}');i+=1
    return text[a:i]
def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--private-source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--wx-config',type=Path,required=True);p.add_argument('--wx-prefix',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    config=[str(a.wx_config),'--prefix='+str(a.wx_prefix)];cflags=shlex.split(subprocess.check_output(config+['--cxxflags'],text=True));libs=shlex.split(subprocess.check_output(config+['--libs','core,base'],text=True));results={}
    env={**os.environ,'LD_LIBRARY_PATH':str(a.wx_prefix/'lib')+':'+os.environ.get('LD_LIBRARY_PATH','')}
    for label,path,macro in [('core',a.source,'OPENNAV_X'),('private',a.private_source,'SKAGER_OCHARTS_ADAPTER')]:
        out=a.output/label;out.mkdir(exist_ok=True);cpp=(path/'libs/s52plib/src/s52plib.cpp').read_text();header=(path/'libs/s52plib/src/s52plib.h').read_text();render=body(cpp,'int s52plib::RenderT_All(');block=body(render,'if (!text->pFont)');setter=body(header,'void SetPresentationTextFace(')
        (out/'chart-text-font-block.inc').write_text(block);(out/'chart-text-face-setter.inc').write_text(setter)
        command=['c++','-std=c++17','-Wall','-Wextra','-Werror','-D'+macro,*cflags,'-I'+str(ROOT/'src'),'-I'+str(out),str(ROOT/'tests/chart_text_face_test.cpp'),*libs,'-o',str(out/'fixture')]
        subprocess.run(command,check=True,env=env);r=subprocess.run([str(out/'fixture')],capture_output=True,text=True,env=env);(out/'run.log').write_text(r.stdout+r.stderr);print(label,r.stdout+r.stderr,flush=True);r.check_returncode()
        # A bypass that ignores the verified instance's empty face must fail.
        mutants={'style-isolation':block.replace('!styled && !m_presentationTextFace.empty()','!styled'), 'specialized-font':block.replace('!styled && !m_presentationTextFace.empty()','!m_presentationTextFace.empty()'),
            'designated-geographic':block.replace('opennav::integration::GeographicChartName(\n              rzRules->obj->FeatureName, rules->INSTstr, bTX)', 'opennav::integration::ChartNameRole::Unchanged'),
            'generated-light':block.replace('!opennav::integration::IsGeneratedLightDescription(\n              rzRules->obj->FeatureName, rules->INSTstr, bTX)', 'true')}
        negatives={}
        try:
            for name,changed in mutants.items():
                assert changed!=block;(out/'chart-text-font-block.inc').write_text(changed);subprocess.run(command,check=True,env=env,capture_output=True);n=subprocess.run([str(out/'fixture')],capture_output=True,text=True,env=env);(out/(name+'.log')).write_text(n.stdout+n.stderr);assert n.returncode==1 and 'chart font check' in n.stderr;negatives[name]=n.returncode
        finally:
            (out/'chart-text-font-block.inc').write_text(block)
            # Retain a runnable binary matching the unmodified production block.
            subprocess.run(command,check=True,env=env,capture_output=True)
        results[label]={'sourceSha256':hashlib.sha256(cpp.encode()).hexdigest(),'renderTAllSha256':hashlib.sha256(render.encode()).hexdigest(),'fontBlockSha256':hashlib.sha256(block.encode()).hexdigest(),'setterSha256':hashlib.sha256(setter.encode()).hexdigest(),'compileCommand':command,'output':r.stdout,'negatives':negatives}
    (a.output/'receipt.json').write_text(json.dumps({'results':results,'limits':'Actual font-establishment blocks and wx fonts; template/cache ownership supplied by fixture. No chart canvas, native Windows HDC, GL draw or persisted profile acceptance.'},indent=2)+'\n')
if __name__=='__main__':main()
