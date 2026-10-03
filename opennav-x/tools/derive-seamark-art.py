#!/usr/bin/env python3
"""Offline provenance derivative. Normal builds use committed stdlib RGBA data.
Requires Node, rsvg-convert and Pillow; never alters the immutable prototype.
"""
import argparse, hashlib, json, re, subprocess
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'resources/chart-style/v1/seamarks'

def main():
    p=argparse.ArgumentParser();p.add_argument('--node',default='node');p.add_argument('--evidence',type=Path,required=True);a=p.parse_args()
    a.evidence.mkdir(parents=True,exist_ok=True)
    mapping=json.loads((ASSETS/'mapping.json').read_text())['symbols']
    prototype=ROOT/'docs/design/prototype'
    script="const fs=require('fs'); const seamarkGuide=JSON.parse(fs.readFileSync(process.argv[1]));"+(prototype/'src/chart-marker-art.js').read_text()+"\nconsole.log(JSON.stringify(Object.fromEntries(JSON.parse(process.argv[2]).map(code=>[code,chartSymbolGraphic({id:'point:'+code,code,kind:'point'},code==='LIGHTS13'?25:27)]))));"
    art=json.loads(subprocess.check_output([a.node,'-e',script,str(prototype/'src/seamarks.json'),json.dumps([s.get('prototypeSource',s['source']) for s in mapping.values()])],text=True))
    css_sources=(prototype/'src/chart-symbols.css').read_text()+(prototype/'src/style.css').read_text()
    rules=[]
    for selector in ('.chart-marker-art','.lighthouse-point','.lighthouse-rays'):
        found=re.findall(re.escape(selector)+r'\{[^}]*\}',css_sources)
        assert len(found)==1,(selector,found)
        rules.append(found[0])
    css=''.join(rules)
    tokens=json.loads((ROOT/'docs/design/prototype-tokens.json').read_text())['themes']
    provenance={'issue':'SCRUM-264','nightBrightness':.78,'rasterizer':subprocess.check_output(['rsvg-convert','--version'],text=True).strip(),'symbols':{},'sources':{},'colors':{}}
    for name,spec in mapping.items():provenance['symbols'][name]={**spec,'prototypeScale':25/32 if name.startswith('XNLIT') else 27/32,'themes':{}}
    for table,theme in [('DAY_BRIGHT','day'),('DUSK','dusk'),('NIGHT','night')]:
        resolved={}
        def replace(m):
            value=tokens[theme][m[1]]
            assert re.fullmatch('#[0-9a-fA-F]{6}',value),value
            rgb=tuple(int(value[i:i+2],16) for i in (1,3,5))
            if theme=='night':rgb=tuple(round(v*.78) for v in rgb)
            resolved[m[1]]=rgb
            return '#'+''.join(f'{v:02x}' for v in rgb)
        for name,spec in mapping.items():
            symbol_css=css.replace('var(--mark-yellow)', 'var('+spec['rayRole']+')') if 'rayRole' in spec else css
            svg='<svg xmlns="http://www.w3.org/2000/svg" width="24" height="28" viewBox="0 0 24 28"><style>'+symbol_css+'</style><g transform="translate(12 14)">'+art[spec.get('prototypeSource',spec['source'])]+'</g></svg>\n'
            svg=re.sub(r'var\((--[^)]+)\)',replace,svg)
            path=ASSETS/(name+'-'+table+'.svg');path.write_text(svg)
            png=a.evidence/(name+'-'+table+'.png')
            subprocess.run(['rsvg-convert',str(path),'-o',str(png)],check=True)
            rgba=Image.open(png).convert('RGBA').tobytes()
            (ASSETS/(name+'-'+table+'-rgba.json')).write_text(json.dumps({'width':24,'height':28,'rows':[rgba[y*96:(y+1)*96].hex() for y in range(28)]},indent=2)+'\n')
            provenance['symbols'][name]['themes'][table]={'rgbaSha256':hashlib.sha256(rgba).hexdigest(),'changedPixels':sum(bool(v) for v in rgba[3::4])}
        provenance['colors'][table]=resolved
        sheet=Image.new('RGB',(480,40+len(mapping)*36),'white');draw=ImageDraw.Draw(sheet)
        font=ImageFont.truetype('/usr/share/fonts/liberation/LiberationSans-Regular.ttf',12)
        draw.text((8,8),table+' prototype derivatives at native 1x size',font=font,fill='black')
        for i,(name,spec) in enumerate(mapping.items()):
            y=36+i*36;draw.text((8,y+7),name+' ← '+spec['source'],font=font,fill='black')
            water=tuple(resolved['--water']);draw.rectangle((235,y,475,y+30),fill=water)
            im=Image.open(a.evidence/(name+'-'+table+'.png')).convert('RGBA');sheet.paste(im,(250,y),im)
        sheet.save(a.evidence/(table+'-native-contact.png'))
    source_paths=['docs/design/prototype/src/chart-marker-art.js','docs/design/prototype/src/seamarks.json','docs/design/prototype/src/chart-symbols.css','docs/design/prototype/src/chart-symbols.js','docs/design/prototype/src/style.css','docs/design/prototype-tokens.json']
    source_paths += [x.relative_to(ROOT).as_posix() for x in sorted(ASSETS.iterdir()) if x.name!='provenance.json' and x.suffix in ('.json','.svg')]
    for path in source_paths:provenance['sources'][path]=hashlib.sha256((ROOT/path).read_bytes().replace(b'\r\n',b'\n')).hexdigest()
    (ASSETS/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
if __name__=='__main__':main()
