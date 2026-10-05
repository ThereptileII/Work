#!/usr/bin/env python3
"""Offline exact SCRUM-280 SVG alpha derivative; builds use stdlib-only masks."""
import argparse, hashlib, json, re, subprocess
from pathlib import Path
from PIL import Image
from chart_hazard_art import SYMBOLS, PIVOT
ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'resources/chart-style/v1/hazards'
def main():
    p=argparse.ArgumentParser();p.add_argument('--evidence',type=Path,required=True);a=p.parse_args()
    a.evidence.mkdir(parents=True,exist_ok=True)
    html=(ROOT/'docs/design/prototype/index.html').read_bytes().decode('utf-8')
    assert hashlib.sha256(html.encode()).hexdigest()=='b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447'
    css=''.join(re.findall(re.escape(s)+r'\{[^}]*\}',html)[-1] for s in ('.chart-marker-art','.chart-marker-art.marker-hazard','.marker-dot'))
    assert 'opacity:.8;stroke-width:1.2' in css and 'fill:var(--chart-text);stroke:none' in css
    assert 'chartSymbolGraphic(s,s.code===\'LIGHTS13\'?25:27)' in html
    assert '#app[data-theme=night] .chart-canvas{filter:brightness(.78)}' in html
    tokens=json.loads((ROOT/'docs/design/prototype-tokens.json').read_text())['themes']
    provenance={'issue':'SCRUM-280','prototypeScale':27/32,'nightBrightness':.78,'rasterizer':subprocess.check_output(['rsvg-convert','--version'],text=True).strip(),'symbols':{},'sources':{},'hazardRgb':{}}
    for table,theme in [('DAY_BRIGHT','day'),('DUSK','dusk'),('NIGHT','night')]:
        rgb=[int(tokens[theme]['--chart-text'][j:j+2],16) for j in (1,3,5)]
        provenance['hazardRgb'][table]=[round(c*.78) for c in rgb] if theme=='night' else rgb
    for name,(rcid,_,_,_,tile) in SYMBOLS.items():
        paths=re.search("'point:"+name+"':'([^']+)'",html)[1]
        svg='<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24"><style>'+css.replace('var(--chart-text)','#ffffff')+'</style><g transform="translate(12 12)"><g class="chart-marker-art marker-hazard" transform="scale(0.84375)">'+paths+'</g></g></svg>\n'
        target=ASSETS/(name+'.svg');target.write_text(svg)
        png=a.evidence/(name+'.png');subprocess.run(['rsvg-convert',str(target),'-o',str(png)],check=True)
        alpha=Image.open(png).convert('RGBA').getchannel('A').tobytes()
        assert not any(alpha[:24]+alpha[-24:]+alpha[::24]+alpha[23::24])
        (ASSETS/(name+'-alpha.json')).write_text(json.dumps({'width':24,'height':24,'rows':[alpha[i:i+24].hex() for i in range(0,576,24)]},indent=2)+'\n')
        provenance['symbols'][name]={'tile':list(tile),'pivot':list(PIVOT),'effectiveRcid':rcid,'alphaSha256':hashlib.sha256(alpha).hexdigest(),'changedPixels':sum(bool(c) for c in alpha)}
    paths=['docs/design/prototype/index.html','docs/design/prototype-tokens.json','tools/derive-hazard-art.py']+[p.relative_to(ROOT).as_posix() for p in sorted(ASSETS.iterdir()) if p.name!='provenance.json']
    provenance['sources']={p:hashlib.sha256((ROOT/p).read_bytes().replace(b'\r\n',b'\n')).hexdigest() for p in paths}
    (ASSETS/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
if __name__=='__main__':main()
