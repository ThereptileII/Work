from pathlib import Path
import hashlib,json,shutil,importlib.util
from PIL import Image,ImageDraw
r=Path.cwd();d=r/'docs/evidence/scrum284-all-round-light';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
spec=importlib.util.spec_from_file_location('prep',r/'tools/prepare-ocharts-adapter.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
owned={}
for p in m.LOCAL:
 t=r/'.local/private-local'/p;t.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(r/p,t);assert t.read_bytes()==(r/p).read_bytes();owned[p]=sha(t)
(d/'private-owned-inputs.json').write_text(json.dumps(owned,indent=2)+'\n')
for p in ['source-proof.json','source-proof.py','baseline-ChartCaFan.h','final-run.log','first-range-string-assumption.log','proof2-run.log','original-negative.py']:
 shutil.copyfile(r/'.local'/p,d/p)
for p in ['run.log','negative.json']:
 t=d/'original-negative'/p;t.parent.mkdir(exist_ok=True);shutil.copyfile(r/'.local/original-negative'/p,t)
shutil.copyfile(r/'.local/final/receipt.json',d/'receipt.json')
for k in ['core','private']:
 t=d/k;t.mkdir(exist_ok=True)
 for p in (r/'.local/final'/k).iterdir():
  if p.suffix in ('.png','.log','.inc'):shutil.copyfile(p,t/p.name)
canvas=Image.new('RGB',(1020,930),'#f5f5f5');draw=ImageDraw.Draw(canvas)
draw.text((15,10),'SCRUM-284 actual extracted SW / Mesa methods: core | private',fill='black')
draw.text((15,28),'Controlled fixed background. Full-circle paint extension; not supplied glyph or chart canvas.',fill='black')
for row,theme in enumerate(['DAY_BRIGHT','DUSK','NIGHT']):
 for ki,k in enumerate(['core','private']):
  for ci,color in enumerate([1,3,4]):
   x=15+ki*500+ci*160;y=65+row*280
   draw.text((x,y),f'{k} {theme} {color}',fill='black')
   for bi,b in enumerate(['software','gl']):
    im=Image.open(d/k/f'{theme}-{color}-{b}.png').convert('RGB').crop((139,139,261,261))
    im=im.resize((122,100)) if False else im
    canvas.paste(im,(x,y+18+bi*130));draw.text((x+125,y+35+bi*130),'SW' if bi==0 else 'GL',fill='black')
canvas.save(d/'comparison.png')
(d/'inputs.json').write_text(json.dumps({p:sha(r/p) for p in ['src/integration/ChartCaFan.h','src/integration/ChartCaAllRound.h','tests/chart_ca_all_round_test.cpp','tools/test-ca-fan.py','tools/prepare-ocharts-adapter.py','patches/opencpn-5.12.4-chart-presentation.patch','patches/ocharts-skager-presentation.patch']},indent=2)+'\n')
