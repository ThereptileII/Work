from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import xml.etree.ElementTree as ET
import json
root=Path('.');out=root/'docs/evidence/scrum280-hazard-glyphs';loader=root/'.local/loader'
xml=ET.parse('.local/before/chartsymbols.xml').getroot();manifest=json.loads(Path('.local/generated/manifest.json').read_text())
font=ImageFont.truetype('/usr/share/fonts/liberation/LiberationSans-Regular.ttf',14)
small=ImageFont.truetype('/usr/share/fonts/liberation/LiberationSans-Regular.ttf',11)
themes=('DAY_BRIGHT','DUSK','NIGHT');files=('day','dusk','dark')
names=('UWTROC03','UWTROC04','WRECKS05','WRECKS01','WRECKS04','ISODGR51','QUAPOS01','QUAPOS02','QUAPOS03')
canvas=Image.new('RGB',(1120,1100),'#f1f1f1');d=ImageDraw.Draw(canvas)
d.text((16,10),'SCRUM-280 — actual pinned loader crops; 3× nearest-neighbor review (not chart canvas)',fill='black',font=font)
d.text((16,33),'Old / new use common geographic anchor. Retained neighbors and synthetic overlay composition are comparison only.',fill='black',font=small)
for t,theme in enumerate(themes):
 x=180+t*310;d.text((x,58),theme,fill='black',font=font)
 atlas=Image.open(f'.local/before/rastersymbols-{files[t]}.png').convert('RGBA')
 bg=tuple(manifest['palette'][theme]['DEPDW'])
 def symbol(name,old=False):
  n=xml.findall("symbols/symbol[name='"+name+"']")[-1];b=n.find('bitmap');p=b.find('pivot');g=b.find('graphics-location')
  pivot=(int(p.get('x')),int(p.get('y')))
  if old:
   a,b0=int(g.get('x')),int(g.get('y'));im=atlas.crop((a,b0,a+int(b.get('width')),b0+int(b.get('height'))))
  else:
   im=Image.open(loader/f'{name}-{theme}.png').convert('RGBA')
   if name in names[:3]:pivot=(12,12)
  return im,pivot
 for r,name in enumerate(names):
  y=90+r*90;d.text((12,y+20),name,fill='black',font=font)
  tile=Image.new('RGBA',(96,27),bg+(255,))
  positions=(22,68) if name in names[:3] else (45,)
  for k,ax in enumerate(positions):
   im,pivot=symbol(name,old=(name in names[:3] and k==0))
   tile.alpha_composite(im,(ax-pivot[0],(21 if name.startswith("QUAPOS") else 13)-pivot[1]))
  canvas.paste(tile.convert('RGB').resize((288,81),Image.Resampling.NEAREST),(x,y))
 # Synthetic composition of loaded images with true pivots; no runtime placement inference.
 tile=Image.new('RGBA',(96,36),bg+(255,))
 for ax,q in zip((22,49,76),('QUAPOS01','QUAPOS02','QUAPOS03')):
  for name in ('UWTROC03',q):
   im,p=symbol(name);tile.alpha_composite(im,(ax-p[0],22-p[1]))
 canvas.paste(tile.convert('RGB').resize((288,108),Image.Resampling.NEAREST),(x,930))
d.text((12,950),'Composition only:',fill='black',font=small);d.text((12,969),'rock + PA / PD / REP',fill='black',font=small)
d.text((12,1060),'Unpassed: recognition, quality-overlay actual placement, SW/GL canvas, Windows/private DLL and boat display.',fill='black',font=small)
canvas.save(out/'three-theme-loader-review.png')
