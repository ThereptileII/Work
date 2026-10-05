#!/usr/bin/env python3
"""Compare actual component screenshots, including the untruncated text oracle.

Run chart_chrome_presentation_test in a disposable 420x200 X11 display for both
baseline and correction, then pass the parent containing before/ and after/.
This reads screenshots only; it does not launch the application or create data.
"""
import argparse
import json
from pathlib import Path
from PIL import Image

parser = argparse.ArgumentParser()
parser.add_argument('evidence', type=Path)
args = parser.parse_args()
# Immutable HTML floating surfaces and literal CSS alpha, independent of C++.
surfaces = {'Day': (247, 248, 240), 'Dusk': (36, 58, 64), 'Night': (21, 33, 41)}
def composite(front, alpha, back):
    return tuple(round((f * alpha + b * (255-alpha))/255) for f, b in zip(front, back))
def same(a, b, box):
    return a.crop(box).tobytes() == b.crop(box).tobytes()

results = []
for theme, surface in surfaces.items():
    before = Image.open(args.evidence/'before'/f'{theme}.png').convert('RGB')
    after = Image.open(args.evidence/'after'/f'{theme}.png').convert('RGB')
    assert before.size == after.size == (420, 200), theme
    caption = (9, 121, 70, 143)
    oracle = (9, 49, 70, 71)
    assert before.crop(caption).tobytes() != before.crop(oracle).tobytes(), 'baseline must reproduce truncation'
    assert after.crop(caption).tobytes() == after.crop(oracle).tobytes(), 'complete natural caption'
    border = composite((107, 139, 128), 28, surface)
    divider = composite((104, 131, 119), 48, surface)
    assert before.getpixel((250, 70)) == (192, 192, 192), 'baseline must reproduce native gray outline'
    for x in range(192, 357):
        assert after.getpixel((x, 70)) == border, (theme, x, 'top border')
        assert after.getpixel((x, 71)) == surface, (theme, x, 'one pixel border')
    for y in range(74, 118):
        for x in range(272, 277):
            expected = divider if x == 274 and 87 <= y < 105 else surface
            assert after.getpixel((x, y)) == expected, (theme, x, y, '1x18 divider + 2px margins')
            assert before.getpixel((x, y)) == surface, 'baseline lacks divider'
    # Four existing hit rectangles and icon rendering remain byte-identical.
    for x in (184, 228, 277, 321):
        assert same(before, after, (x, 74, x+44, 118)), (theme, x, 'unchanged button')
    for y in range(200):
        for x in range(420):
            if 180 <= x < 369 and 70 <= y < 122: continue
            if 9 <= x < 70 and 121 <= y < 143: continue
            assert before.getpixel((x, y)) == after.getpixel((x, y)), (theme, x, y, 'outside correction')
    results.append({'theme': theme, 'caption_matches_untruncated_native_oracle': True,
                    'border_rgb': border, 'divider_rgb': divider,
                    'button_pixels_unchanged': True, 'outside_pixels_unchanged': True})
print(json.dumps({'scope': 'actual Linux wx component captures only', 'results': results}, indent=2))
