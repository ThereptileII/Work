"""One pinned Simplified generic-building alias; stock/Paper/conspicuous stay intact."""
import copy
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from chart_raster_ink import decode, encode
from chart_seamark_art import canonical

ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT/'resources/chart-style/v1/building-point/recipe.json'
RECIPE_SHA256 = '0d5dfa911e9aec2c244249e6d38934852aa293302b9ac8ff6841cce9fa071145'
NAME, RCID, TILE = 'XNBLDG01', '60016', (788, 1160, 9, 9)
FILL, OUTLINE = 'XNBLF', 'XNBLO'
COLORS = {FILL, OUTLINE}


def recipe():
    raw = RECIPE.read_bytes().replace(b'\r\n', b'\n')
    assert hashlib.sha256(raw).hexdigest() == RECIPE_SHA256, 'Building point recipe changed'
    return json.loads(raw)


def instruction(lookup):
    original = lookup.findtext('instruction')
    if lookup.get('id') != '1091':
        return original
    assert hashlib.sha256(canonical(lookup)).hexdigest() == recipe()['lookupCanonicalSha256'], 'Generic building lookup changed'
    return 'SY('+NAME+')'


def source_node(tree):
    nodes = tree.findall("symbols/symbol[name='BUISGL01']")
    assert len(nodes) == 1
    assert hashlib.sha256(canonical(nodes[0])).hexdigest() == recipe()['sourceNodeCanonicalSha256'], 'Building symbol changed'
    return nodes[0]


def alias(stock):
    node = copy.deepcopy(stock)
    node.set('RCID', RCID)
    node.find('name').text = NAME
    node.find('bitmap/graphics-location').attrib = {'x':str(TILE[0]), 'y':str(TILE[1])}
    # Identical original HPGL pen letters/geometry; only these two ink identities.
    assert node.findtext('color-ref') == 'WLANDFKCHBRN'
    node.find('color-ref').text = 'W'+OUTLINE+'K'+FILL
    return node


def relocate(xml):
    tree = ET.fromstring(xml)
    nodes = tree.findall("lookups/lookup[@id='1091']")
    assert len(nodes) == 1
    replacement = instruction(nodes[0])
    stock = source_node(tree)
    assert not tree.findall("symbols/symbol[name='"+NAME+"']")
    assert not any(n.get('RCID') == RCID for n in tree.iter())
    x,y,w,h = TILE
    for bitmap in tree.findall('.//bitmap'):
        pos = bitmap.find('graphics-location')
        if pos is None:
            continue
        bx,by = int(pos.get('x')),int(pos.get('y'))
        bw,bh = int(bitmap.get('width')),int(bitmap.get('height'))
        assert not(x-2 < bx+bw and x+w+2 > bx and y-2 < by+bh and y+h+2 > by), 'Building alias overlaps declared artwork'
    pattern = r'(<lookup id="1091" RCID="31143" name="BUISGL">)(.*?)(</lookup>)'
    matches = list(re.finditer(pattern, xml, re.S))
    token = '<instruction>SY(BUISGL01)</instruction>'
    assert len(matches) == 1 and matches[0][2].count(token) == 1
    xml = re.sub(pattern, lambda m:m[1]+m[2].replace(token,'<instruction>'+replacement+'</instruction>')+m[3], xml, flags=re.S)
    assert xml.count('</symbols>') == 1
    return xml.replace('</symbols>', ET.tostring(alias(stock),encoding='unicode')+'</symbols>')


def restore_for_validation(before, after):
    nodes = after.findall("symbols/symbol[name='"+NAME+"']")
    assert len(nodes) == 1 and canonical(nodes[0]) == canonical(alias(source_node(before))), 'Building alias changed'
    after.find('symbols').remove(nodes[0])


def tile_rgba(source, table, colors):
    proof = recipe()
    assert hashlib.sha256(source).hexdigest() == proof['sourceTileRgbaSha256'][table], 'Building source tile changed'
    alpha = bytes.fromhex(''.join(proof['alphaRows']))
    assert source[3::4] == alpha and len(alpha) == 81
    # No theme-RGB inference: only the reviewed coordinate recipe from Day's
    # verified two-pen source. Signed weights retain filtered edge ringing.
    weights = sum(proof['fillNumerators'], [])
    den = proof['denominator']
    fill,outline = colors[FILL],colors[OUTLINE]
    rgba = bytearray()
    for n,a in zip(weights,alpha):
        pixel = [(o*den+n*(f-o)+den//2)//den for f,o in zip(fill,outline)]
        assert all(0 <= c <= 255 for c in pixel), 'Building ink outside byte range'
        rgba.extend((*pixel,a))
    assert len(rgba) == 324 and rgba[3::4] == source[3::4]
    return bytes(rgba)


def paint(content, table, colors):
    chunks,before = decode(content)
    source = b''.join(before[((78+y)*1500+459)*4:((78+y)*1500+468)*4] for y in range(9))
    rgba = tile_rgba(source,table,colors)
    x,y,w,h = TILE
    for row in range(y-2,y+h+2):
        assert not any(before[(row*1500+x-2)*4+3:(row*1500+x+w+2)*4:4]), 'Building tile or moat is not transparent'
    after = bytearray(before)
    for row in range(h):
        start = ((y+row)*1500+x)*4
        after[start:start+w*4] = rgba[row*w*4:(row+1)*w*4]
    return encode(chunks,after), {'symbol':NAME, 'RCID':RCID, 'lookupId':'1091',
        'tile':TILE, 'pivot':[4,4], 'fillRgb':list(colors[FILL]), 'outlineRgb':list(colors[OUTLINE]),
        'changedPixels':81, 'alphaSha256':hashlib.sha256(rgba[3::4]).hexdigest(),
        'recipeSha256':RECIPE_SHA256, 'sourceSymbolAndPaperUnchanged':True}
