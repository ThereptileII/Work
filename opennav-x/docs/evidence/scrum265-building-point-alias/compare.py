#!/usr/bin/env python3
"""Resource review only: actual 9x9 atlas pixels, no chart-canvas simulation."""
import argparse
import hashlib
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--source', type=Path, required=True)
p.add_argument('--generated', type=Path, required=True)
p.add_argument('--baseline', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
manifest = json.loads((a.generated/'manifest.json').read_text())
canvas = Image.new('RGB', (960, 590), '#f1f3f2')
d = ImageDraw.Draw(canvas)
font = ImageFont.load_default(size=16)
small = ImageFont.load_default(size=13)
d.text((20, 12), 'SCRUM-265: actual atlas tiles on effective land (12x nearest + 1x)', font=font, fill='black')
columns = [('Original generic BUISGL01', (459,78,468,87)),
           ('Proposed generic XNBLDG01', (788,1160,797,1169)),
           ('Unchanged conspicuous BUISGL11', (478,78,487,87))]
for i, (label, _) in enumerate(columns):
    d.text((115+i*280, 43), label, font=small, fill='black')
records = []
for row, (table, suffix) in enumerate([('DAY_BRIGHT','day'), ('DUSK','dusk'), ('NIGHT','dark')]):
    filename = 'rastersymbols-'+suffix+'.png'
    source = Image.open(a.source/filename).convert('RGBA')
    generated = Image.open(a.generated/filename).convert('RGBA')
    baseline = Image.open(a.baseline/filename).convert('RGBA')
    for box in ((459,78,468,87),(478,78,487,87)):
        assert baseline.crop(box).tobytes() == generated.crop(box).tobytes()
    assert source.crop((459,78,468,87)).tobytes() == generated.crop((459,78,468,87)).tobytes()
    land = tuple(manifest['palette'][table]['LANDA'])
    y = 73+row*162
    d.text((12,y+49), table, font=small, fill='black')
    tiles = {}
    for i, (label, box) in enumerate(columns):
        atlas = generated if i == 1 else baseline
        tile = atlas.crop(box)
        patch = Image.new('RGBA', (9,9), (*land,255))
        patch.alpha_composite(tile)
        x = 115+i*280
        d.rectangle((x,y,x+240,y+139), fill=land)
        canvas.paste(patch.resize((108,108), Image.Resampling.NEAREST).convert('RGB'), (x+12,y+15))
        canvas.paste(patch.convert('RGB'), (x+184,y+64))
        name = ('XNBLDG01' if i==1 else 'BUISGL01' if i==0 else 'BUISGL11')
        tile.save(a.output/(name+'-'+table+'.png'))
        tiles[name] = {'rgbaSha256':hashlib.sha256(tile.tobytes()).hexdigest(),
                       'alphaSha256':hashlib.sha256(tile.getchannel('A').tobytes()).hexdigest()}
    records.append({'table':table, 'landRgb':land,
                    'fillRgb':manifest['palette'][table]['XNBLF'],
                    'outlineRgb':manifest['palette'][table]['XNBLO'], 'tiles':tiles})
d.text((20,568), 'Resource comparison only. Native/private/boat readability remains unqualified.', font=small, fill='black')
canvas.save(a.output/'theme-comparison.png')
(a.output/'comparison.json').write_text(json.dumps(records, indent=2)+'\n')
