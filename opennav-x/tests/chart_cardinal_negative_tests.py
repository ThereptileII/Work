"""Focused fail-closed controls for SCRUM-256, no application launch."""
import argparse
from pathlib import Path
import sys
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
import chart_cardinal_art as art
from chart_raster_ink import decode,encode

p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
a=p.parse_args();checks=0

def reject(action):
    global checks
    try:action()
    except AssertionError:checks+=1
    else:raise AssertionError('Invalid cardinal artwork accepted')

xml=(a.source/'chartsymbols.xml').read_text()
for x,y in [(116,1160),(114,1158),(148,1160)]:
    tree=ET.fromstring(xml)
    tree.find('.//bitmap/graphics-location').attrib={'x':str(x),'y':str(y)}
    reject(lambda:art.relocate(ET.tostring(tree,encoding='unicode')))
# New tiles must also reject the already-authorized anchorage tile and each other.
original=art.SYMBOLS
for x in (20,52,116):
    changed=dict(original);row=list(changed['BOYCAR02']);row[-1]=(x,1160,24,28);changed['BOYCAR02']=tuple(row)
    import chart_anchor_art,chart_service_art
    with patch.object(art,'SYMBOLS',changed):reject(lambda:art.relocate(chart_service_art.relocate(chart_anchor_art.relocate(xml))))
# A source mask/SVG change is refused before painting, without mutating files.
read=Path.read_bytes
for target in ('BOYCAR01-NIGHT.svg','BOYCAR02-NIGHT-rgba.json'):
    with patch.object(Path,'read_bytes',lambda self:read(self)+(b'changed' if self.name==target else b'')):
        reject(lambda:art.coverage('NIGHT'))
# An occupied pixel in either tile or its sampling moat must be rejected.
chunks,pixels=decode((a.source/'rastersymbols-day.png').read_bytes())
for x,y in ((116,1160),(146,1158)):
    damaged=bytearray(pixels);damaged[(y*1500+x)*4+3]=1
    content=encode(chunks,damaged)
    reject(lambda:art.paint(content,'DAY_BRIGHT'))
for change in [('CATCAM1','CATCAM4'),('<table-name>Simplified</table-name>','<table-name>Paper</table-name>'),('SY(BOYCAR01)','SY(BOYCAR02)')]:
    # Touch the exact eligible lookup only; no production source is mutated.
    tree=ET.fromstring(xml);node=tree.find("lookups/lookup[@id='1022']")
    old=ET.tostring(node,encoding='unicode');bad=old.replace(*change)
    reject(lambda:art.require_selectors(ET.fromstring(ET.tostring(tree,encoding='unicode').replace(old,bad))))
tree=ET.fromstring(xml);tree.find("lookups/lookup[@id='1262']/instruction").text='SY(TOPMAR01)'
reject(lambda:art.require_selectors(tree))
print(f'{checks} cardinal overlap, occupied-pixel and source-provenance rejection controls passed')
