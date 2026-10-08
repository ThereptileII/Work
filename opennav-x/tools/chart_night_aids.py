"""SCRUM-323: keep navigation aids recognizable in the SKAGER Night palette.

The pinned S-52 NIGHT palette is designed for a black bridge display. On the
SKAGER Night water (#0e171c) its red/green/yellow/white aid inks
(e.g. CHGRN 22,34,7) become effectively invisible. This bounded derivation
replaces only those chromatic aid roles with the matching Day ink scaled by
one uniform factor. Scaling every channel equally preserves hue, so lateral
red/green, yellow and white meanings are unchanged; only luminance rises.

Raster symbols use the same rule: a Night sprite pixel changes only when the
same-coordinate Day pixel is exactly the pinned Day CHRED, CHGRN or CHYLW ink
(LITRD/LITGN/LITYW share those values) and both alphas are identical. Its new
RGB is that Day ink scaled by the same factor. Day white is excluded because it
equals the Day water fill used inside many symbols. Brown, magenta, blue,
neutral pixels, alpha, geometry and owned SKAGER artwork stay unchanged; the
lift runs after every owned-artwork step and only on untouched stock pixels.
Dusk and Day are untouched.
"""
import re
import xml.etree.ElementTree as ET
from chart_raster_ink import decode, encode

TABLE = 'NIGHT'
SHEET = 'rastersymbols-dark.png'
# Chromatic aid inks: lateral/special mark bodies and light flares.
ROLES = ('CHRED', 'CHGRN', 'CHYLW', 'CHWHT', 'LITRD', 'LITGN', 'LITYW')
FACTOR = 0.62
# Exact pinned Day sprite inks owned by this derivation (verified against the
# Day colour table in night_colors): CHRED/LITRD, CHGRN/LITGN, CHYLW/LITYW.
AID_DAY_RGB = {(241, 84, 105), (104, 228, 86), (244, 218, 72)}


def _day_colors(xml):
    body = re.search(r'<color-table name="DAY_BRIGHT">(.*?)</color-table>', xml, re.S)[1]
    found = {}
    for name in ROLES:
        matches = re.findall(r'<color name="' + name + r'" r="(\d+)" g="(\d+)" b="(\d+)"\s*/>', body)
        assert len(matches) == 1, 'Pinned Day aid colour missing or duplicated: ' + name
        found[name] = tuple(int(v) for v in matches[0])
    return found


def night_colors(original_xml):
    """Owned Night RGB for each aid role, derived from the pinned Day table."""
    day = _day_colors(original_xml)
    assert {day['CHRED'], day['CHGRN'], day['CHYLW']} == AID_DAY_RGB, 'Pinned Day aid inks changed'
    assert day['LITRD'] == day['CHRED'] and day['LITGN'] == day['CHGRN'] and day['LITYW'] == day['CHYLW']
    return {name: tuple(round(channel * FACTOR) for channel in rgb) for name, rgb in day.items()}


def recolor(xml, original_xml):
    colors = night_colors(original_xml)
    pattern = r'(<color-table name="' + TABLE + r'">)(.*?)(</color-table>)'
    assert len(re.findall(pattern, xml, re.S)) == 1

    def table_replace(match):
        body = match[2]
        for name, rgb in colors.items():
            color = r'(<color name="' + name + r'" r=")\d+(" g=")\d+(" b=")\d+("\s*/>)'
            assert len(re.findall(color, body)) == 1, 'Pinned Night aid colour missing: ' + name
            body = re.sub(color, lambda m: m[1] + str(rgb[0]) + m[2] + str(rgb[1]) + m[3] + str(rgb[2]) + m[4], body)
        return match[1] + body + match[3]

    return re.sub(pattern, table_replace, xml, flags=re.S)


def restore_for_validation(before, after):
    """Undo exactly the owned Night aid colours before whole-tree identity proof."""
    stock = next(t for t in before.find('color-tables') if t.get('name') == TABLE)
    styled = next(t for t in after.find('color-tables') if t.get('name') == TABLE)
    expected = night_colors(ET.tostring(before, encoding='unicode'))
    for name in ROLES:
        entries = styled.findall("color[@name='" + name + "']")
        assert len(entries) == 1
        entry = entries[0]
        rgb = expected[name]
        assert entry.attrib == {'name': name, 'r': str(rgb[0]), 'g': str(rgb[1]), 'b': str(rgb[2])}, \
            'Unexpected Night aid colour: ' + name
        assert len(entry) == 0 and not (entry.text or '').strip()
        entry.attrib = stock.find("color[@name='" + name + "']").attrib.copy()


def changed_pixel(day, themed, i):
    """True when Night pixel i is owned by this derivation."""
    alpha = day[i + 3]
    if not alpha or themed[i + 3] != alpha:
        return False
    return tuple(day[i:i + 3]) in AID_DAY_RGB


def target(day, i):
    return bytes(round(channel * FACTOR) for channel in day[i:i + 3])


def paint(day_pixels, original_png, themed_png):
    """Run after every owned-artwork step. Only pixels still equal to the
    pinned stock Night sheet change, so owned tiles stay exactly as painted."""
    _, original = decode(original_png)
    chunks, pixels = decode(themed_png)
    assert len(day_pixels) == len(pixels) == len(original)
    result = bytearray(pixels)
    count = 0
    for i in range(0, len(pixels), 4):
        if pixels[i:i + 4] == original[i:i + 4] and changed_pixel(day_pixels, original, i):
            new = target(day_pixels, i)
            if result[i:i + 3] != new:
                result[i:i + 3] = new
                count += 1
    return encode(chunks, bytes(result)), {
        'factor': FACTOR, 'dayInks': sorted(AID_DAY_RGB), 'changedPixels': count,
        'source': 'same-coordinate Day pixel scaled by factor; alpha, geometry '
                  'and owned artwork preserved'}


def restore_pixels(day_pixels, before, after):
    """Test helper: restore owned Night aid pixels, proving nothing else changed."""
    restored = 0
    for i in range(0, len(before), 4):
        if changed_pixel(day_pixels, before, i) and after[i:i + 3] == target(day_pixels, i):
            if after[i:i + 3] != before[i:i + 3]:
                restored += 1
            after[i:i + 3] = before[i:i + 3]
    return restored


_pinned_day = None


def restore_from_pinned_day(before, after):
    """Test helper for whole-sheet proofs: undo owned Night aid pixels using the
    pinned Day sheet. Day/Dusk sheets never equal the Night targets here."""
    global _pinned_day
    if _pinned_day is None:
        from pathlib import Path
        source = Path(__file__).resolve().parents[1] / 'upstream/OpenCPN/data/s57data/rastersymbols-day.png'
        _, _pinned_day = decode(source.read_bytes())
    if len(_pinned_day) != len(before):
        return 0
    return restore_pixels(_pinned_day, before, after)
