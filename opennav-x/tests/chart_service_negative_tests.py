"""Focused fail-closed controls for SCRUM-254, no application launch."""
import argparse
from pathlib import Path
import sys
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import chart_service_art as art
from chart_raster_ink import decode,encode

p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
a=p.parse_args();checks=0

def reject(action):
    global checks
    try:action()
    except AssertionError:checks+=1
    else:raise AssertionError('Invalid service artwork accepted')

xml=(a.source/'chartsymbols.xml').read_text()
for x,y in [(52,1160),(50,1158),(84,1160),(820,1160),(818,1158)]:
    tree=ET.fromstring(xml)
    tree.find('.//bitmap/graphics-location').attrib={'x':str(x),'y':str(y)}
    reject(lambda:art.relocate(ET.tostring(tree,encoding='unicode')))
# New tiles must also reject the already-authorized anchorage tile and each other.
original=art.SYMBOLS
for x in (20,52):
    changed=dict(original);row=list(changed['RTPBCN02']);row[-1]=(x,1160,24,24);changed['RTPBCN02']=tuple(row)
    import chart_anchor_art
    with patch.object(art,'SYMBOLS',changed):reject(lambda:art.relocate(chart_anchor_art.relocate(xml)))
# A source mask/SVG change is refused before painting, without mutating files.
read=Path.read_bytes
for target in ('PILBOP02.svg','RTPBCN02-alpha.json','SMCFAC02.svg','SMCFAC02-alpha.json'):
    with patch.object(Path,'read_bytes',lambda self:read(self)+(b'changed' if self.name==target else b'')):
        reject(art.coverage)
# An occupied pixel in either tile or its sampling moat must be rejected.
chunks,pixels=decode((a.source/'rastersymbols-day.png').read_bytes())
for x,y in ((52,1160),(82,1158),(820,1160),(818,1158)):
    damaged=bytearray(pixels);damaged[(y*1500+x)*4+3]=1
    content=encode(chunks,damaged)
    reject(lambda:art.paint(content,'DAY_BRIGHT'))
print(f'{checks} service overlap, occupied-pixel and source-provenance rejection controls passed')
