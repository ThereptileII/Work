#!/usr/bin/env python3
"""Exact prototype special-mark art split into a classified body and fitted X."""
import argparse,hashlib,json,re,subprocess,xml.etree.ElementTree as ET
from pathlib import Path
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'resources/chart-style/v1/yellow-buoy'
p=argparse.ArgumentParser();p.add_argument('--evidence',type=Path,required=True);a=p.parse_args()
ASSETS.mkdir(exist_ok=True);a.evidence.mkdir(parents=True,exist_ok=True)
prototype=ROOT/'docs/design/prototype'
script="const fs=require('fs');const seamarkGuide=JSON.parse(fs.readFileSync(process.argv[1]));"+(prototype/'src/chart-marker-art.js').read_text()+"\nconsole.log(chartSymbolGraphic({id:'point:BOYSPP11',code:'BOYSPP11',kind:'point'},27));"
art=ET.fromstring(subprocess.check_output(['node','-e',script,str(prototype/'src/seamarks.json')],text=True))
body=ET.fromstring(ET.tostring(art));head=ET.fromstring(ET.tostring(art));shape=body.find('g');top=shape.find('g')
assert len(top)==1 and top[0].get('d')=='M-3-12 3-6M3-12-3-6'
shape.remove(top)
for item in list(head.find('g')):
 if item.tag!='g':head.find('g').remove(item)
css=(prototype/'src/chart-symbols.css').read_text();rules=re.findall(r'\.chart-marker-art\{[^}]*\}',css);assert len(rules)==1
styles=rules[0];tokens=json.loads((ROOT/'docs/design/prototype-tokens.json').read_text())['themes']
proof={'issue':'SCRUM-264','nightBrightness':.78,'prototypeScale':27/32,'pivot':[12,14],'derivation':'Exact BOYSPP11 chartSymbolGraphic split: supplied X head removed from body and painted only from explicit yellow TOPMAR TOPSHP7. Geometry is not inferred physical dimensions.','sources':{},'symbols':{}}
for name,tree,tile,rcid in [('XNSPPY01',body,[692,1160,24,28],60013),('XNSPPT01',head,[756,1160,24,28],60015)]:
 proof['symbols'][name]={'tile':tile,'rcid':rcid,'themes':{}}
 for table,theme in [('DAY_BRIGHT','day'),('DUSK','dusk'),('NIGHT','night')]:
  def role(m):
   x=tokens[theme][m[1]];rgb=[int(x[i:i+2],16) for i in (1,3,5)]
   if theme=='night':rgb=[round(v*.78) for v in rgb]
   return '#'+''.join(f'{v:02x}' for v in rgb)
  svg='<svg xmlns="http://www.w3.org/2000/svg" width="24" height="28" viewBox="0 0 24 28"><style>'+styles+'</style><g transform="translate(12 14)">'+ET.tostring(tree,encoding='unicode')+'</g></svg>\n'
  svg=re.sub(r'var\((--[^)]+)\)',role,svg);path=ASSETS/(name+'-'+table+'.svg');path.write_text(svg)
  png=a.evidence/(name+'-'+table+'.png');subprocess.run(['rsvg-convert',str(path),'-o',str(png)],check=True)
  rgba=Image.open(png).convert('RGBA').tobytes();(ASSETS/(name+'-'+table+'-rgba.json')).write_text(json.dumps({'width':24,'height':28,'rows':[rgba[y*96:(y+1)*96].hex() for y in range(28)]},indent=2)+'\n')
  proof['symbols'][name]['themes'][table]={'rgbaSha256':hashlib.sha256(rgba).hexdigest(),'changedPixels':sum(bool(x) for x in rgba[3::4])}
for path in ['docs/design/prototype/src/chart-marker-art.js','docs/design/prototype/src/seamarks.json','docs/design/prototype/src/chart-symbols.css','docs/design/prototype-tokens.json','tools/derive-yellow-buoy-art.py']+[str(p.relative_to(ROOT)) for p in sorted(ASSETS.iterdir()) if p.name!='provenance.json']:
 proof['sources'][path]=hashlib.sha256((ROOT/path).read_bytes().replace(b'\r\n',b'\n')).hexdigest()
(ASSETS/'provenance.json').write_text(json.dumps(proof,indent=2)+'\n')
