#!/usr/bin/env python3
"""Pin the effective S-52 tower raster shapes; never reads any chart cell."""
import argparse, hashlib, json, sys, xml.etree.ElementTree as ET
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from chart_raster_ink import decode
from chart_seamark_art import canonical
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--source', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
lock = json.loads((ROOT/'resources/chart-style/v1/source-lock.json').read_text())
data = {}
for name in ('chartsymbols.xml', 'rastersymbols-day.png', 'rastersymbols-dusk.png', 'rastersymbols-dark.png'):
    raw = (a.source/name).read_bytes()
    if name.endswith('.xml'): raw = raw.replace(b'\r\n', b'\n')
    assert hashlib.sha256(raw).hexdigest() == lock['files'][name]['sha256']
    data[name] = raw
xml = ET.fromstring(data['chartsymbols.xml'])
sheets = {t: decode(data['rastersymbols-'+s+'.png'])[1] for t,s in [('DAY_BRIGHT','day'),('DUSK','dusk'),('NIGHT','dark')]}
result = {'schema':1, 'issue':'SCRUM-308', 'prototypeSha256':hashlib.sha256((ROOT/'docs/design/prototype/index.html').read_bytes()).hexdigest(), 'sourceXmlSha256':hashlib.sha256(data['chartsymbols.xml']).hexdigest(), 'model':'Exact effective 14x26 raster, alpha and hotspot retained. Per-coordinate Day two-ink weights transfer ordinary outline/fill to XNGEO/DEPMD; conspicuous opaque/antialiased silhouette uses distinct XNBLO. No source lookup changes or fabricated light glyph.', 'symbols':{}}
for index,(source,name,x,lookup) in enumerate([('TOWERS01','XNLTWR01',343,'1147'),('TOWERS03','XNLTWR03',367,'1140')]):
    nodes = xml.findall("symbols/symbol[name='"+source+"']")
    assert len(nodes)==2 and nodes[-1].findtext('definition')=='R'
    node=nodes[-1]
    tiles={t:b''.join(v[((742+y)*1500+x)*4:((742+y)*1500+x+14)*4] for y in range(26)) for t,v in sheets.items()}
    assert len({v[3::4] for v in tiles.values()})==1
    day=tiles['DAY_BRIGHT']; outline=(34,31,31); fill=(230,236,245)
    delta=[f-o for f,o in zip(fill,outline)]; denominator=sum(d*d for d in delta)
    weights=[]
    for i in range(0,len(day),4):
        n=sum((day[i+c]-outline[c])*delta[c] for c in range(3)) if day[i+3] else 0
        if index: assert not day[i+3] or tuple(day[i:i+3])==outline; n=0
        if day[i+3]:
            assert 0<=n<=denominator
            assert max(abs((outline[c]*denominator+n*delta[c]+denominator//2)//denominator-day[i+c]) for c in range(3))<=1
        weights.append(n)
    result['symbols'][name]={'source':source,'sourceNodeCanonicalSha256':hashlib.sha256(canonical(node)).hexdigest(),'sourceDefinitionCount':2,'sourceRcid':342,'sourceTile':[x,742,14,26],'sourceTileRgbaSha256':{t:hashlib.sha256(v).hexdigest() for t,v in tiles.items()},'lookupId':lookup,'lookupCanonicalSha256':hashlib.sha256(canonical(xml.find("lookups/lookup[@id='"+lookup+"']"))).hexdigest(),'rcid':60018+index,'tile':[980+24*index,1160,14,26],'pivot':[6,22],'outlineRole':'XNGEO' if index==0 else 'XNBLO','fillRole':'DEPMD','denominator':denominator,'fillNumerators':[weights[y*14:(y+1)*14] for y in range(26)],'alphaRows':[day[3::4][y*14:(y+1)*14].hex() for y in range(26)]}
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text(json.dumps(result,indent=2)+'\n')
print(hashlib.sha256(a.output.read_bytes()).hexdigest())
