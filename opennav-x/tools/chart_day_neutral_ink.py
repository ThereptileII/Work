"""SCRUM-261: exact Day CHBLK/CHGRD pixels with proven XML bitmap ownership.

Shared RGB values do not establish a role. Every rectangle referencing a pixel
must declare only CHBLK/CHGRD for that RGB; unknown/shared other roles and all
SOUND* rectangles are excluded. No tolerance, alpha or geometry substitution.
"""
import hashlib
import struct
import xml.etree.ElementTree as ET
from chart_raster_ink import decode, encode

XML_SHA = '84f93522576ed5872b24865cf6161872ed0b4b634f77c64ca697a04d3568e886'
PNG_SHA = 'ee1020a8b94b312faba8c146974e49147d280e5ef3cc13f3cd688bbfd54dde0c'
MASK_SHA = '5e4ecc6dbc228ed8b5874ba4094922c03b4a12b68220fc4553266460b228ce73'
TARGET = (83, 100, 95)  # Exact final prototype Day --mark-black, not --chart-text.
NEUTRAL = {(7, 7, 7), (125, 137, 140)}


def derive_day(xml, content, target):
    xml = xml.replace(b'\r\n', b'\n')
    assert hashlib.sha256(xml).hexdigest() == XML_SHA
    assert hashlib.sha256(content).hexdigest() == PNG_SHA
    assert tuple(target) == TARGET
    tree = ET.fromstring(xml)
    table = tree.find("color-tables/color-table[@name='DAY_BRIGHT']")
    palette = {c.get('name'): tuple(int(c.get(k)) for k in ('r', 'g', 'b'))
               for c in table.findall('color')}
    assert palette['CHBLK'] == (7, 7, 7) and palette['CHGRD'] == (125, 137, 140)
    chunks, pixels = decode(content)
    eligible, excluded = set(), set()
    for section in ('symbols', 'patterns', 'line-styles'):
        for node in tree.find(section):
            bitmap = node.find('bitmap')
            if bitmap is None or bitmap.find('graphics-location') is None:
                continue
            location = bitmap.find('graphics-location')
            x, y = int(location.get('x')), int(location.get('y'))
            w, h = int(bitmap.get('width')), int(bitmap.get('height'))
            assert 0 <= x < x+w <= 1500 and 0 <= y < y+h <= 1200
            refs = ''.join((node.findtext('color-ref') or '').split())
            assert len(refs) % 6 == 0
            roles = {refs[i+1:i+6] for i in range(0, len(refs), 6)}
            permitted = {rgb for rgb in NEUTRAL
                         if (same := {role for role in roles if palette[role] == rgb})
                         and same <= {'CHBLK', 'CHGRD'}
                         and not node.findtext('name', '').startswith('SOUND')}
            for row in range(y, y+h):
                for column in range(x, x+w):
                    index = row*1500+column
                    pixel = pixels[index*4:index*4+4]
                    rgb = tuple(pixel[:3])
                    if pixel[3] and rgb in NEUTRAL:
                        (eligible if rgb in permitted else excluded).add(index)
    mask = sorted(eligible - excluded)
    assert len(mask) == 40482
    assert hashlib.sha256(b''.join(struct.pack('<I', i) for i in mask)).hexdigest() == MASK_SHA
    changed = bytearray(pixels)
    for index in mask:
        changed[index*4:index*4+3] = bytes(target)
    assert changed[3::4] == pixels[3::4]
    return encode(chunks, changed), {
        'sourcePaletteRoles': ['CHBLK', 'CHGRD'], 'targetRgb': list(TARGET),
        'changedPixels': len(mask), 'blackPixels': 39511, 'grayPixels': 971,
        'maskIndexSha256': MASK_SHA, 'sourcePngSha256': PNG_SHA,
        'sourceXmlSha256': XML_SHA, 'excludedNeutralPixels': 4215,
        'soundingsAndOtherRolesPreserved': True,
        'alphaAndGeometryPreserved': True,
    }
