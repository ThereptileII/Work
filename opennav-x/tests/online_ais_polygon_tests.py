#!/usr/bin/env python3
"""AIS fill geometry against the immutable SVG and pinned ocpnDC GL strip.

Run with --upstream /path/to/pinned/OpenCPN. This checks geometry, not a GL driver.
--source permits replaying the pre-fix production file as a negative control.
"""
import argparse
import hashlib
import math
from pathlib import Path
import re
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
PIN = "37fd0cddb7334fe489e9f18aa163977a9c5c84f7"
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--upstream", type=Path, required=True)
parser.add_argument("--source", type=Path,
                    default=ROOT / "src/integration/OnlineAisOverlay.cpp")
args, remaining = parser.parse_known_args()


def cross(a, b, p):
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


def triangle(a, b, c, p):
    signs = [cross(a, b, p), cross(b, c, p), cross(c, a, p)]
    return not (min(signs) < 0 < max(signs))


def polygon(vertices, p):
    """Independent even/odd software polygon interior oracle."""
    inside = False
    for a, b in zip(vertices, vertices[1:] + vertices[:1]):
        if (a[1] > p[1]) != (b[1] > p[1]):
            if p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]):
                inside = not inside
    return inside


class AisPolygon(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        html = (ROOT / "docs/design/prototype/index.html").read_bytes()
        if hashlib.sha256(html).hexdigest() != "b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447":
            raise AssertionError("immutable prototype changed")
        svg = re.search(r'class="ais-ship".*?<path d="([^"]+)"', html.decode()).group(1)
        if not re.fullmatch(r'M[-\d .]+Z', svg):
            raise AssertionError("unsupported reference SVG path")
        numbers = list(map(int, re.findall(r'-?\d+', svg)))
        cls.reference = list(zip(numbers[::2], numbers[1::2]))
        source = args.source.read_text()
        arrays = re.search(r'const int x\[4\]=\{([^}]+)\},y\[4\]=\{([^}]+)\}', source)
        cls.actual = list(zip(*(list(map(int, v.split(','))) for v in arrays.groups())))
        # The actual pinned source is the contract; no invented generic
        # triangulation replaces its special four-vertex drawing path.
        cls.dc = subprocess.check_output(
            ["git", "-C", str(args.upstream), "show", PIN + ":gui/src/ocpndc.cpp"], text=True)
        block = re.search(r'if \(n == 4\) \{(.*?)\} else if \(n == 3\)', cls.dc, re.S).group(1)
        expected = """float x1 = workBuf[4]; float y1 = workBuf[5];
            workBuf[4] = workBuf[6]; workBuf[5] = workBuf[7];
            workBuf[6] = x1; workBuf[7] = y1;
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);"""
        if re.sub(r'\s+', '', block) != re.sub(r'\s+', '', expected):
            raise AssertionError("pinned four-point GL strip contract changed")
        cls.strip = [cls.actual[i] for i in (0, 1, 3, 2)]

    def test_software_polygon_is_same_cyclic_svg_path(self):
        self.assertEqual(len(self.reference), 4)
        self.assertIn(self.actual, [self.reference[i:] + self.reference[:i] for i in range(4)])
        # Cyclic equality preserves all software edges, winding, fill and
        # transformed/rounded vertices; only the starting vertex can change.

    def test_stern_notch_is_not_filled_by_pinned_strip(self):
        point = (0, 8)
        self.assertFalse(polygon(self.reference, point))
        a, b, c, d = self.strip
        self.assertFalse(triangle(a, b, c, point) or triangle(b, c, d, point),
                         "pinned GL strip fills the prototype's open stern notch")

    def test_strip_union_matches_svg_interior(self):
        # Independent SVG interior vs actual strip's two triangles, including
        # empty background, bow, shoulders and notch, under affine transforms.
        for degrees, scale in ((0, 1), (37, 1.5), (90, 2)):
            angle = math.radians(degrees)
            def transform(p):
                return (100 + scale * (p[0] * math.cos(angle) - p[1] * math.sin(angle)),
                        70 + scale * (p[0] * math.sin(angle) + p[1] * math.cos(angle)))
            a, b, c, d = map(transform, self.strip)
            for x in range(-8, 9):
                for y in range(-14, 12):
                    p = (x + .137, y + .283)  # Avoid polygon-edge ambiguity.
                    q = transform(p)
                    self.assertEqual(polygon(self.reference, p),
                                     triangle(a, b, c, q) or triangle(b, c, d, q),
                                     (degrees, scale, p))


if __name__ == "__main__":
    unittest.main(argv=[__file__, *remaining])
