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
manifest = json.loads((args.current / "capture.json").read_text())
for item in manifest["captures"]:
    name = item["file"]
    ref, current = args.reference / name, args.current / name
    if args.supplemental_reference and (args.supplemental_reference / name).is_file():
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
                                "referenceSha256": hashlib.sha256(ref.read_bytes()).hexdigest(),
                                "currentSha256": item["sha256"], "accepted": False})
(args.output / "comparison.json").write_text(json.dumps(record, indent=2) + "\n")
