#!/usr/bin/env python3
"""Lossless reference/current/diff evidence, never an automatic visual PASS.

Real unavailable inputs and actual OpenCPN charts differ from the mock content.
Do not mask those differences, invent matching sensor values or use a broad
similarity tolerance to accept an unfinished view.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from PIL import Image, ImageChops, ImageStat

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--reference", type=Path, required=True)
parser.add_argument("--supplemental-reference", type=Path,
                    help="Same-platform render of additional unchanged prototype states")
parser.add_argument("--current", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
record = {"result": "PENDING HUMAN REVIEW", "accepted": False,
          "policy": "No masks, resizing, pixel thresholds or automatic similarity acceptance", "views": []}
reference_manifest = json.loads((args.reference / "capture.json").read_text())

# Full passage is an action from the native horizon, not a separate prototype
# view.  The real prototype action opens the existing Route drawer, whose
# supplied canonical state is passage-day.  Keep this as a closed mapping so a
# new or misspelled native capture still fails instead of receiving a guessed
# reference.
REFERENCE_STATE_ALIASES = {
    "horizon-full-passage.png": "passage-day.png",
}

def reference_for(name):
    mapped_name = REFERENCE_STATE_ALIASES.get(name, name)
    if mapped_name != name:
        if reference_manifest.get("viewport") != {"width": 1280, "height": 800}:
            raise RuntimeError("Mapped prototype reference has the wrong viewport")
        if reference_manifest.get("deviceScaleFactor") != 1:
            raise RuntimeError("Mapped prototype reference has the wrong device scale")
        state = reference_manifest.get("states", {}).get(mapped_name.removesuffix(".png"))
        if not state or state.get("theme") != "day":
            raise RuntimeError("Mapped prototype reference is not the supplied day Passage state")
    return args.reference / mapped_name, mapped_name

manifest = json.loads((args.current / "capture.json").read_text())
for item in manifest["captures"]:
    name = item["file"]
    ref, reference_name = reference_for(name)
    current = args.current / name
    if (name not in REFERENCE_STATE_ALIASES and args.supplemental_reference
            and (args.supplemental_reference / name).is_file()):
        original = json.loads((args.reference / "capture.json").read_text())
        extra = json.loads((args.supplemental_reference / "capture.json").read_text())
        for key in ("htmlSha256", "platform", "viewport", "deviceScaleFactor"):
            if original[key] != extra[key]:
                raise RuntimeError("Supplemental reference differs in " + key)
        ref = args.supplemental_reference / name
    if not ref.is_file():
        raise RuntimeError(f"No supplied prototype state for {name}")
    if hashlib.sha256(current.read_bytes()).hexdigest() != item["sha256"]:
        raise RuntimeError(f"Current capture changed: {name}")
    destination = args.output / current.stem
    destination.mkdir()
    shutil.copy2(ref, destination / "reference.png")
    shutil.copy2(current, destination / "current.png")
    with Image.open(ref) as a, Image.open(current) as b:
        if a.size != (1280, 800) or b.size != a.size:
            raise RuntimeError("Compare only exact 1280x800 native/reference pixels")
        diff = ImageChops.difference(a.convert("RGB"), b.convert("RGB"))
        diff.save(destination / "diff.png")
        record["views"].append({"name": current.stem, "meanAbsoluteChannelDifference": ImageStat.Stat(diff).mean,
                                "referenceName": reference_name,
                                "referenceSha256": hashlib.sha256(ref.read_bytes()).hexdigest(),
                                "currentSha256": item["sha256"], "accepted": False})
(args.output / "comparison.json").write_text(json.dumps(record, indent=2) + "\n")
