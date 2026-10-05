from pathlib import Path
import hashlib,json,shutil,re
from PIL import Image,ImageDraw
r=Path.cwd();d=r/'docs/evidence/scrum286-overzoom-warning';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for p in (r/'.local/final2').iterdir():
 if p.suffix in ['.png','.log','.json','.inc']:shutil.copyfile(p,d/p.name)
for name in ['source-proof.json','verify-source.py','compile-units.py']:shutil.copyfile(r/'.local'/name,d/name)
(d/'objects').mkdir(exist_ok=True)
for p in (r/'.local/objects').iterdir():
 if p.suffix in ['.log','.json']:shutil.copyfile(p,d/'objects'/p.name)
(d/'fixture-setup-failures').mkdir(exist_ok=True)
for run in ['proof','proof2','proof3','proof4','final']:
 for name in ['compile.log','run.log','receipt.json']:
  p=r/'.local'/run/name
  if p.exists():shutil.copyfile(p,d/'fixture-setup-failures'/(run+'-'+name))
canvas=Image.new('RGB',(440,305),'#fafafa');draw=ImageDraw.Draw(canvas);draw.text((12,10),'Actual extracted warning / ocpnDC methods',fill='black');draw.text((12,27),'Controlled background; no application canvas',fill='black')
for row,(theme,title) in enumerate([(0,'Day'),(1,'Dusk'),(2,'Night')]):
 for col,backend in enumerate(['software','gl']):
  x=12+col*214;y=52+row*82;draw.text((x,y),title+' '+backend,fill='black');canvas.paste(Image.open(d/f'{backend}-{theme}.png').crop((0,0,180,55)),(x,y+18))
canvas.save(d/'comparison.png')
html=(r/'docs/design/prototype/index.html').read_text()
assert 'font-size:12px;line-height:1.65' in html and '.callout.warning b{color:var(--amber)}' in html and 'font-weight:550' in html
assert 'border-radius:0 8px 8px 0' in html and 'padding:13px 15px' in html
facts={}
def rgb(h):return tuple(bytes.fromhex(h.lstrip('#')))
def lum(c):
 a=[v/255 for v in c];return sum(w*(v/12.92 if v<=.04045 else ((v+.055)/1.055)**2.4) for w,v in zip([.2126,.7152,.0722],a))
for name,selector in [('Day',':root'),('Dusk','#app[data-theme=dusk]'),('Night','#app[data-theme=night]')]:
 body=html.split(selector+'{',1)[1].split('}',1)[0];tokens=dict(re.findall(r'--([\w-]+):([^;}]+)',body));bg=rgb(tokens['bg']);ink=rgb(tokens['amber']);back=tuple((v*9+b*246+127)//255 for v,b in zip(rgb('#ecc48c'),bg));ratio=(lum(ink)+.05)/(lum(back)+.05);facts[name]={'ink':ink,'backing':back,'contrast':ratio};assert ratio>4.5
(d/'prototype-roles.json').write_text(json.dumps({'prototypeSha256':sha(r/'docs/design/prototype/index.html'),'roles':facts,'fontPixels':12,'weight':550,'padding':[13,15],'note':'Composed warning heading extension; rounded8 plus left2 follows existing owned Callout, not a new literal prototype component.'},indent=2)+'\n')
(d/'inputs.json').write_text(json.dumps({p:sha(r/p) for p in ['src/integration/ChartPresentation.cpp','src/integration/ChartPresentation.h','patches/opencpn-5.12.4-chart-presentation.patch','tests/overzoom_warning_test.cpp','tools/test-overzoom-warning.py','src/ui/Controls.cpp','src/ui/Theme.h']},indent=2)+'\n')
