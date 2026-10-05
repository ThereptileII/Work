#!/usr/bin/env python3
"""Derive only the immutable prototype's actual generic-beacon paths/CSS."""
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'resources/chart-style/v1/generic-beacon'
parser = argparse.ArgumentParser()
parser.add_argument('--evidence', type=Path, required=True)
args = parser.parse_args()
args.evidence.mkdir(parents=True, exist_ok=True)
ASSETS.mkdir(parents=True, exist_ok=True)
prototype = ROOT / 'docs/design/prototype'
art_path = prototype / 'src/chart-marker-art.js'
script = "const fs=require('fs');const seamarkGuide=JSON.parse(fs.readFileSync(process.argv[1]));" + art_path.read_text() + "\nconsole.log(chartSymbolGraphic({id:'point:BCNGEN01',code:'BCNGEN01',kind:'point'},27));"
art = subprocess.check_output(['node', '-e', script, str(prototype/'src/seamarks.json')], text=True).strip()
assert 'marker-service' in art and 'marker-reference' not in art
css_text = (prototype/'src/chart-symbols.css').read_text() + (prototype/'src/style.css').read_text()
rules = []
for selector in ('.chart-marker-art', '.chart-marker-art.marker-service'):
    found = re.findall(re.escape(selector) + r'\{[^}]*\}', css_text)
    assert len(found) == 1
    rules.append(found[0])
tokens = json.loads((ROOT/'docs/design/prototype-tokens.json').read_text())['themes']
proof = {'issue': 'SCRUM-264', 'sourceSymbol': 'point:BCNGEN01', 'tile': [724,1160,24,28],
         'pivot': [12,14], 'prototypeScale': 27/32, 'nightBrightness': .78,
         'rasterizer': subprocess.check_output(['rsvg-convert','--version'], text=True).strip(),
         'selection': 'Only original Simplified generic selections1696/31748 and1708/31760; no classified/Paper replacement.',
         'themes': {}, 'sources': {}}
for table, theme in [('DAY_BRIGHT','day'), ('DUSK','dusk'), ('NIGHT','night')]:
    resolved = {}
    def replace(match):
        value = tokens[theme][match[1]]
        assert re.fullmatch('#[0-9a-fA-F]{6}', value)
        rgb = tuple(int(value[i:i+2],16) for i in (1,3,5))
        if theme == 'night':
            rgb = tuple(round(c*.78) for c in rgb)
        resolved[match[1]] = rgb
        return '#' + ''.join(f'{c:02x}' for c in rgb)
    svg = '<svg xmlns="http://www.w3.org/2000/svg" width="24" height="28" viewBox="0 0 24 28"><style>' + ''.join(rules) + '</style><g transform="translate(12 14)">' + art + '</g></svg>\n'
    svg = re.sub(r'var\((--[^)]+)\)', replace, svg)
    name = 'XNBCNG01-' + table
    path = ASSETS/(name+'.svg')
    path.write_text(svg)
    png = args.evidence/(name+'.png')
    subprocess.run(['rsvg-convert', str(path), '-o', str(png)], check=True)
    rgba = Image.open(png).convert('RGBA').tobytes()
    (ASSETS/(name+'-rgba.json')).write_text(json.dumps({'width':24, 'height':28, 'rows':[rgba[y*96:(y+1)*96].hex() for y in range(28)]},indent=2)+'\n')
    proof['themes'][table] = {'rgbaSha256':hashlib.sha256(rgba).hexdigest(), 'changedPixels':sum(bool(v) for v in rgba[3::4]), 'colors':resolved}
paths = ['docs/design/prototype/src/chart-marker-art.js', 'docs/design/prototype/src/chart-symbols.css',
         'docs/design/prototype/src/seamarks.json', 'docs/design/prototype/src/style.css',
         'docs/design/prototype-tokens.json', 'tools/derive-generic-beacon-art.py']
paths += [p.relative_to(ROOT).as_posix() for p in sorted(ASSETS.iterdir()) if p.name != 'provenance.json']
for path in paths:
    proof['sources'][path] = hashlib.sha256((ROOT/path).read_bytes().replace(b'\r\n',b'\n')).hexdigest()
(ASSETS/'provenance.json').write_text(json.dumps(proof,indent=2)+'\n')
