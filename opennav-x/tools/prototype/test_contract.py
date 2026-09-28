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
        # Read the authoritative source, not a second manually maintained oracle.
        css = (render.ORIGINAL / "src/style.css").read_text(encoding="utf-8")
        native = (ROOT / "src/ui/Theme.h").read_text(encoding="utf-8")
        extracted = json.loads((ROOT / "docs/design/prototype-tokens.json").read_text())
        root = dict(re.findall(r"(--[\w-]+):([^;}]+)", re.search(r":root\{([^}]+)\}", css)[1]))
        roles = ["bg", "surface", "surface2", "surface2", "line", "text",
                 "secondary", "muted", "mint", "mint", "amber", "red", "magenta"]
        for mode in ["Day", "Dusk", "Night"]:
            values = dict(root)
            if mode != "Day":
                selector = f"#app[data-theme={mode.lower()}]"
                values.update(dict(re.findall(r"(--[\w-]+):([^;}]+)",
                    re.search(re.escape(selector) + r"\{([^}]+)\}", css)[1])))
            self.assertEqual(extracted["themes"][mode.lower()], values)
            body = re.search(r"case LightMode::" + mode + r":\s*return \{([^}]+)\}", native)[1]
            actual = [int(x, 16) for x in re.findall(r"0x[0-9A-Fa-f]+", body)]
            self.assertEqual(actual, [int(values["--"+r][1:], 16) for r in roles])


if __name__ == "__main__":
    unittest.main()
