"""SCRUM-265: only pinned CROSSX01 construction-hatch ink; no pattern redesign."""
import copy
import hashlib
import re
import xml.etree.ElementTree as ET
from chart_raster_ink import decode, encode

COLOR = 'XNHAT'
NODE_SHA = 'ee3101a2fc4335fc4766bdbed224b18aee9c93c43ff38f82b3245be71379391d'
TILE_SHA = {'DAY_BRIGHT':'d160f03e2a7bfb503d8102c94765e7780043f774f17f71c6762e0324f21055c7',
            'DUSK':'4ab894760b4d2518a56d7f03e98f1c91ddb01b915a22ef9e09ab3071e1d5cb79',
            'NIGHT':'4ab894760b4d2518a56d7f03e98f1c91ddb01b915a22ef9e09ab3071e1d5cb79'}
INK = {'DAY_BRIGHT':(83,100,95), 'DUSK':(225,229,216), 'NIGHT':(117,133,121)}


def canonical(node):
    node = copy.deepcopy(node)
    for child in node.iter():
        if child.text is not None: child.text = child.text.strip() or None
        child.tail = None
    return ET.tostring(node)


def pattern(tree):
    nodes = tree.findall("patterns/pattern[name='CROSSX01']")
    assert len(nodes) == 1, 'CROSSX01 identity/uniqueness changed'
    return nodes[0]


def check_source(tree):
    node = pattern(tree)
    assert hashlib.sha256(canonical(node)).hexdigest() == NODE_SHA, 'Pinned construction pattern changed'
    owners = []
    for section in ('symbols','patterns','line-styles'):
        for candidate in tree.find(section):
            bitmap = candidate.find('bitmap')
            location = bitmap.find('graphics-location') if bitmap is not None else None
            if location is None: continue
            x,y,w,h = map(int,(location.get('x'),location.get('y'),bitmap.get('width'),bitmap.get('height')))
            if x < 416 and x+w > 400 and y < 1056 and y+h > 1040:
                owners.append((section,candidate.findtext('name'),candidate.get('RCID')))
    assert owners == [('patterns','CROSSX01','3')], 'Construction tile has another owner'
    return node


def recolor(xml):
    check_source(ET.fromstring(xml))
    expression = r'(<pattern RCID="3">)(.*?)(</pattern>)'
    matches = list(re.finditer(expression,xml,re.S))
    assert len(matches) == 1 and matches[0][2].count('<color-ref>ACHBRN</color-ref>') == 1
    return re.sub(expression,lambda m:m[1]+m[2].replace('<color-ref>ACHBRN</color-ref>','<color-ref>AXNHAT</color-ref>')+m[3],xml,flags=re.S)


def restore_for_validation(before, after):
    stock = check_source(before)
    styled = pattern(after)
    expected = copy.deepcopy(stock);expected.find('color-ref').text='AXNHAT'
    assert canonical(styled) == canonical(expected), 'Construction pattern changed beyond owned ink'
    styled.find('color-ref').text='ACHBRN'


def paint(content, table, color):
    assert tuple(color) == INK[table], 'Unexpected construction ink'
    chunks,pixels = decode(content)
    tile = b''.join(pixels[(y*1500+400)*4:(y*1500+416)*4] for y in range(1040,1056))
    assert hashlib.sha256(tile).hexdigest() == TILE_SHA[table], 'Construction tile source changed'
    changed = bytearray(pixels);count = 0
    for y in range(1040,1056):
        for x in range(400,416):
            offset=(y*1500+x)*4
            if pixels[offset+3]:
                assert table=='DAY_BRIGHT' and pixels[offset:offset+4]==bytes((177,145,57,255))
                changed[offset:offset+3]=bytes(color);count+=1
    assert count == (192 if table=='DAY_BRIGHT' else 0)
    assert changed[3::4] == pixels[3::4]
    return (encode(chunks,changed) if count else content), {
        'pattern':'CROSSX01','RCID':'3','color':COLOR,'targetRgb':list(color),
        'bounds':[400,1040,416,1056], 'changedPixels':count,
        'sourceTileSha256':TILE_SHA[table],
        'alphaSha256':hashlib.sha256(tile[3::4]).hexdigest(),
        'preexistingTransparentTheme':table!='DAY_BRIGHT',
        'geometrySpacingAlphaAndConditionsPreserved':True}
