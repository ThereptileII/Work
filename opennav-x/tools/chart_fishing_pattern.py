"""SCRUM-282: one source-locked AP alias; same-name point and stock pattern stay exact."""
import copy
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from chart_construction_hatch import canonical
from chart_raster_ink import decode, encode

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT/'resources/chart-style/v1/fishing-pattern'
NAME, RCID, TILE = 'XNFISH03', '60017', (948,1160,24,24)
PATTERN_SHA = 'a839490017a9905e0e790a8e422a785eac43f67c99229ada32be1b265a0fed1a'
LOOKUP_SHA = 'd2b48e2099d8fddcb80d2006efd671694c031ce8f086918c9090417f50730cea'


def instruction(node):
    assert hashlib.sha256(canonical(node)).hexdigest() == LOOKUP_SHA
    return 'AP(XNFISH03);LS(DASH,1,CHGRD)'


def original(tree):
    nodes = tree.findall("patterns/pattern[name='FSHFAC03']")
    assert len(nodes) == 1 and hashlib.sha256(canonical(nodes[0])).hexdigest() == PATTERN_SHA
    return nodes[0]


def alias(stock):
    node = copy.deepcopy(stock)
    node.set('RCID', RCID)
    node.find('name').text = NAME
    # The real vector bounds/spacing/pivot survive loading, and the old HPGL
    # remains a stock fallback if the narrowly guarded compositor declines.
    ET.SubElement(node, 'prefer-bitmap').text = 'no'
    bitmap = ET.SubElement(node, 'bitmap', width='24', height='24')
    ET.SubElement(bitmap, 'distance', min='2000', max='10000')
    ET.SubElement(bitmap, 'pivot', x='12', y='12')
    ET.SubElement(bitmap, 'origin', x='0', y='0')
    ET.SubElement(bitmap, 'graphics-location', x='948', y='1160')
    return node


def relocate(xml):
    tree = ET.fromstring(xml)
    stock = original(tree)
    assert not tree.findall(".//*[@RCID='"+RCID+"']")
    assert not tree.findall(".//name[.='"+NAME+"']")
    x,y,w,h = TILE
    for bitmap in tree.findall('.//bitmap'):
        loc = bitmap.find('graphics-location')
        if loc is None: continue
        bx,by,bw,bh = map(int,(loc.get('x'),loc.get('y'),bitmap.get('width'),bitmap.get('height')))
        assert not (x-2 < bx+bw and x+w+2 > bx and y-2 < by+bh and y+h+2 > by), 'Fishing tile overlaps a declared resource'
    expr = r'(<lookup id="65" RCID="32101" name="FSHFAC">)(.*?)(</lookup>)'
    matches = list(re.finditer(expr,xml,re.S)); assert len(matches) == 1
    lookup = tree.find("lookups/lookup[@id='65']")
    wanted = instruction(lookup)
    xml = re.sub(expr,lambda m:m[1]+m[2].replace(lookup.findtext('instruction'),wanted)+m[3],xml,flags=re.S)
    return xml.replace('</patterns>',ET.tostring(alias(stock),encoding='unicode')+'\n    </patterns>',1)


def restore_for_validation(before, after):
    stock = original(before)
    assert canonical(original(after)) == canonical(stock)
    nodes = after.findall("patterns/pattern[name='"+NAME+"']")
    assert len(nodes) == 1 and canonical(nodes[0]) == canonical(alias(stock))
    after.find('patterns').remove(nodes[0])


def coverage():
    p = json.loads((ASSETS/'provenance.json').read_text())
    for name,digest in p['sources'].items():
        assert hashlib.sha256((ROOT/name).read_bytes().replace(b'\r\n',b'\n')).hexdigest() == digest, 'Fishing artwork provenance changed'
    data = json.loads((ASSETS/'XNFISH03-alpha.json').read_text())
    assert data['width'] == data['height'] == 24 and len(data['rows']) == 24
    assert all(len(row) == 48 for row in data['rows'])
    mask = bytes.fromhex(''.join(data['rows']))
    assert hashlib.sha256(mask).hexdigest() == p['alphaSha256']
    assert sum(bool(a) for a in mask) == p['pixels'] == 48
    return mask,p


def paint(content, table):
    mask,p = coverage()
    rgb = bytes(p['ink'][table])
    assert tuple(rgb) == {'DAY_BRIGHT':(156,134,150),'DUSK':(184,160,177),'NIGHT':(113,99,110)}[table]
    chunks,before = decode(content); after = bytearray(before)
    x,y,w,h = TILE
    for row in range(y-2,y+h+2):
        assert not any(before[(row*1500+x-2)*4+3:(row*1500+x+w+2)*4:4]), 'Occupied fishing tile/moat'
    for dy in range(h):
        for dx in range(w):
            a = mask[dy*w+dx]
            if a:
                i = ((y+dy)*1500+x+dx)*4
                after[i:i+4] = rgb+bytes([a])
    return encode(chunks,after), {'name':NAME,'RCID':RCID,'tile':TILE,'pixels':48,'rgb':list(rgb),
                                 'alphaSha256':p['alphaSha256'],'stockVectorCellPreserved':True}
