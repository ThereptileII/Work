"""SCRUM-261 independent Day pixel golden and exact-role safety exclusions."""
import hashlib
import re
import struct
import xml.etree.ElementTree as ET
from pathlib import Path
from chart_raster_ink import decode

SOURCE_RGBA = '9ec0625741a7d7780291232653aefd0dea929e76d670ab6ee821a406c2e63990'
EXPECTED_RGBA = '3d7f1b8210a701bada753367dbf9aa8a71a3dabe553b5f57a270c81446cf471f'
MASK_SHA = '5e4ecc6dbc228ed8b5874ba4094922c03b4a12b68220fc4553266460b228ce73'


def verify_day_neutral(source, output, metadata, check):
    root = Path(__file__).resolve().parents[1]
    html = (root/'docs/design/prototype/index.html').read_text()
    css = re.search(r'#app\{--mark-red:[^}]+', html)[0]
    check(re.search(r'--mark-black:([^;]+)', css)[1] == '#53645f')
    check(metadata['palette']['DAY_BRIGHT']['CHBLK'] == (83, 100, 95))
    check(metadata['palette']['DAY_BRIGHT']['CHGRD'] == (83, 100, 95))
    before_chunks, before = decode((source/'rastersymbols-day.png').read_bytes())
    after_chunks, after = decode((output/'rastersymbols-day.png').read_bytes())
    check(hashlib.sha256(before).hexdigest() == SOURCE_RGBA)
    check([(k,v) for k,v in before_chunks if k != b'IDAT'] ==
          [(k,v) for k,v in after_chunks if k != b'IDAT'])
    # Only existing separately verified transparent artwork slots are reversed.
    for x,w,h in [(20,20,20),(52,24,24),(84,24,24),(820,24,24),
                  (116,24,28),(148,24,28),(180,24,28),(212,24,28)]:
        for y in range(1160,1160+h):
            i=(y*1500+x)*4;after[i:i+w*4]=before[i:i+w*4]
    from chart_seamark_resources_tests import restore_tiles
    restore_tiles(before,after)
    # Independently decoded with Pillow, independently enumerated from original
    # XML rectangles/roles. See retained audit; no generator output is the oracle.
    check(hashlib.sha256(after).hexdigest() == EXPECTED_RGBA)
    check(before[3::4] == after[3::4])
    changed=[i for i in range(0,len(before),4) if before[i:i+4] != after[i:i+4]]
    check(len(changed) == metadata['dayNeutralRasterInk']['changedPixels'] == 40482)
    check(hashlib.sha256(b''.join(struct.pack('<I',i//4) for i in changed)).hexdigest() == MASK_SHA)
    check(sum(before[i:i+3] == b'\x07\x07\x07' for i in changed) == 39511)
    check(sum(before[i:i+3] == b'\x7d\x89\x8c' for i in changed) == 971)
    check(all(before[i+3] and after[i:i+3] == bytes((83,100,95)) for i in changed))
    # Sounding bitmaps can themselves declare CHGRD. Preserve every byte anyway.
    tree=ET.parse(source/'chartsymbols.xml').getroot()
    count=0
    for node in tree.findall('symbols/symbol'):
        if not node.findtext('name','').startswith('SOUND'):continue
        b=node.find('bitmap')
        if b is None:continue
        location=b.find('graphics-location')
        if location is None:continue
        x,y=int(location.get('x')),int(location.get('y'));w,h=int(b.get('width')),int(b.get('height'))
        for row in range(y,y+h):
            i=(row*1500+x)*4;check(before[i:i+w*4] == after[i:i+w*4])
        count+=1
    check(count > 100)
    # Reversing exactly changed RGB bytes returns the complete source atlas,
    # including every chromatic, off-palette, unreferenced and invisible byte.
    for i in changed:after[i:i+3]=before[i:i+3]
    check(after == before)
    for name,digest in [('rastersymbols-dusk.png','201f0663bd786d5e1d8ba09d8d43b106df164d5a282f60860258d226adadb6c9'),
                        ('rastersymbols-dark.png','9c91273e3dbe9563d79b2d2b0a757bb83207aa0db9183e8aecc1112070ead724')]:
        # Restore only the independently proved new seamark slots before the
        # existing complete Dusk/Night PNG golden. Other bytes stay exact.
        from chart_raster_ink import encode
        _,old_pixels=decode((source/name).read_bytes())
        chunks,current_pixels=decode((output/name).read_bytes())
        restore_tiles(old_pixels,current_pixels)
        # SCRUM-279 service tile is separately checked; preserve this older golden.
        for y in range(1160,1184):
            i=(y*1500+820)*4;current_pixels[i:i+96]=old_pixels[i:i+96]
        check(hashlib.sha256(encode(chunks,current_pixels)).hexdigest() == digest)
    from chart_day_neutral_ink import derive_day
    xml=(source/'chartsymbols.xml').read_bytes();png=(source/'rastersymbols-day.png').read_bytes()
    for bad_xml,bad_png,target in [(xml.replace(b'DCHGRD',b'DSNDG1',1),png,(83,100,95)),
                                   (xml,png+b'x',(83,100,95)),
                                   (xml,png,(104,123,122))]:
        try:derive_day(bad_xml,bad_png,target)
        except AssertionError:check(True)
        else:raise AssertionError('Changed source/role or wrong prototype token accepted')
