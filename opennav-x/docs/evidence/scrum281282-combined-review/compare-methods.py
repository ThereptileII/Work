from pathlib import Path
import json,hashlib,importlib.util
root=Path.cwd();s=importlib.util.spec_from_file_location('e',root/'tools/verify-anchor-loader.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
refs={'core':root.parent/'scrum282-fishing-pattern/.local/fishing/core','private':root.parent/'scrum282-fishing-pattern/.local/fishing/followup/source'}
current={'core':root/'build/integration-source','private':root/'.local/private'};old={'core':root.parent/'scrum281-cable-waveform/build/integration-source','private':root.parent/'scrum281-cable-waveform/.local/private'};r={}
for name,dir in current.items():
 text=(dir/'libs/s52plib/src/s52plib.cpp').read_text();r[name]={}
 for sig,reference in [('void s52plib::draw_lc_poly(',old[name]),('render_canvas_parms *s52plib::CreatePatternBufferSpec(',refs[name])]:
  actual=m.function(text,sig);baseline=m.function((reference/'libs/s52plib/src/s52plib.cpp').read_text(),sig);assert actual==baseline,(name,sig)
  r[name][sig]={'sha256':hashlib.sha256(actual.encode()).hexdigest(),'bytes':len(actual.encode()),'identicalReviewedReference':str(reference/'libs/s52plib/src/s52plib.cpp')}
 for h in ['ChartFishingPattern.h','ChartCableWave.h']:assert text.count('#include "integration/'+h+'"')==1
r['headers']={}
for name,tree in [('ChartFishingPattern.h','scrum282-fishing-pattern'),('ChartCableWave.h','scrum281-cable-waveform'),('ChartCaFan.h','scrum281-cable-waveform')]:
 p=root/'src/integration'/name;ref=root.parent/tree/'src/integration'/name;assert p.read_bytes()==ref.read_bytes(),name;r['headers'][name]=hashlib.sha256(p.read_bytes()).hexdigest()
(root/'.local/method-equality.json').write_text(json.dumps(r,indent=2)+'\n');print('Two feature bodies in each complete unit and all three shared headers match reviewed isolated implementations exactly.')
