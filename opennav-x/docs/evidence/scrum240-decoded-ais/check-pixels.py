#!/usr/bin/env python3
"""Check actual decoded-target capture pixels; never synthesizes AIS or images."""
from pathlib import Path
from collections import Counter
import hashlib,json
from PIL import Image
root=Path(__file__).resolve().parent
ink=(145,100,119);fill=(247,248,240)
rows=[]
for renderer in ('software','opengl'):
    for style in ('SKAGER','Standard'):
        path=root/renderer/(style+'-Day.png')
        im=Image.open(path).convert('RGB')
        assert im.size==(1280,800)
        for target,box in [('class_a',(470,180,505,224)),('class_b',(470,480,505,526))]:
            counts=Counter(im.crop(box).getdata())
            if style=='SKAGER':
                assert counts[ink]>=20 and counts[fill]>=30,(renderer,target,counts[ink],counts[fill])
            else:
                assert counts[ink]==0 and counts[fill]==0,(renderer,target)
                stock=(104,228,86) if renderer=='software' else (104,227,86)
                assert counts[stock]>=100,(renderer,target,counts[stock])
            rows.append({'renderer':renderer,'style':style,'target':target,'screen_roi':box,'exact_plum_pixels':counts[ink],'exact_floating_fill_pixels':counts[fill],'image_sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
        if style=='SKAGER':
            # These observed screen points distinguish the concave stern from a
            # filled triangle, away from the sampled antialiased boundary.
            water=(213,229,229) if renderer=='software' else (212,228,228)
            assert im.getpixel((486,492))==water and im.getpixel((487,492))==water
            assert im.getpixel((486,499))==fill and im.getpixel((487,499))==fill
            assert im.getpixel((486,207))==fill and im.getpixel((487,207))==fill
(root/'pixel-results.json').write_text(json.dumps({'status':'passed','scope':'8 actual target/style/renderer regions; notch and straight-stern interior probes','regions':rows},indent=2)+'\n')
print('Actual decoded AIS body pixels passed in 8 target/style/renderer regions')
