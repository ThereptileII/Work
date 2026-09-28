#!/usr/bin/env python3
"""Design authority checks independent of any native rendering claim."""
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import tempfile
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
spec = importlib.util.spec_from_file_location("prototype_render", HERE / "render.py")
render = importlib.util.module_from_spec(spec)
spec.loader.exec_module(render)
spec = importlib.util.spec_from_file_location("prototype_extract", HERE / "extract-tokens.py")
extractor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extractor)


class PrototypeContract(unittest.TestCase):
    def test_original_bytes(self):
        manifest = render.verify_original()
        self.assertEqual(len(manifest["files"]), 113)
        self.assertEqual(manifest["htmlSha256"],
            "b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447")

    def test_reformatted_original_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)
            original = b"<html>\r\nunchanged</html>\r\n"
            item = {"path": "index.html", "bytes": len(original),
                    "sha256": hashlib.sha256(original).hexdigest()}
            (path / "manifest.json").write_text(json.dumps({"files": [item]}))
            (path / "index.html").write_bytes(original.replace(b"\r\n", b"\n"))
            with patch.object(render, "ORIGINAL", path), patch.object(render, "MANIFEST", path / "manifest.json"):
                with self.assertRaisesRegex(RuntimeError, "Immutable prototype differs"):
                    render.verify_original()

    def test_native_palette_matches_actual_css_cascade(self):
        # Measure the final HTML cascade, including appended style blocks.
        # The supplied source fragment alone omits later chart-symbol tokens.
        native = (ROOT / "src/ui/Theme.h").read_text(encoding="utf-8")
        native = native[native.index("constexpr Palette Theme"):]
        extracted = json.loads((ROOT / "docs/design/prototype-tokens.json").read_text())
        self.assertEqual(extracted, extractor.extract())
        roles = ["bg", "surface", "surface2", "surface2", "line", "text",
                 "secondary", "muted", "mint", "mint", "amber", "red", "magenta"]
        for mode in ["Day", "Dusk", "Night"]:
            values = extracted['themes'][mode.lower()]
            body = re.search(r"case LightMode::" + mode + r":\s*return \{([^}]+)\}", native)[1]
            actual = [int(x, 16) for x in re.findall(r"0x[0-9A-Fa-f]+", body)]
            self.assertEqual(actual, [int(values["--"+r][1:], 16) for r in roles])

    def test_appended_chart_symbol_tokens_are_not_lost(self):
        tokens = extractor.extract()['themes']
        self.assertEqual(tokens['day']['--mark-black'], '#53645f')
        self.assertEqual(tokens['dusk']['--mark-black'], '#c3cec2')
        self.assertEqual(tokens['night']['--mark-black'], '#89988c')
        for theme in tokens.values():
            self.assertTrue(all('--mark-'+role in theme for role in
                ('red','green','yellow','black','white','blue','service','area')))


if __name__ == "__main__":
    unittest.main()
