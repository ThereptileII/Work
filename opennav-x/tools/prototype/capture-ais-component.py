#!/usr/bin/env python3
"""Capture dedicated offline native AIS/Passage/Instruments tests, never the product.

Synthetic data is confined to a non-installed executable. These images compare
the drawer component only; they do not qualify chart composition or live data.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--client", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--component", choices=["ais", "passage", "instruments", "energy", "settings", "anchor", "autopilot", "alerts", "radar"], default="ais")
    args = parser.parse_args()
    args.client = args.client.resolve()
    args.output = args.output.resolve()
    if args.output.exists():
        raise SystemExit("Use a new evidence directory")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    windows = sys.platform == "win32"
    if windows and os.environ.get("GITHUB_ACTIONS") != "true":
        raise SystemExit("Native capture requires disposable CI; never run fixtures aboard")
    env = dict(os.environ)
    xserver = None
    if windows:
        spec = importlib.util.spec_from_file_location("windows_ui", ROOT / "tools/windows-ui.py")
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        ui.ensure_desktop(1440, 900)
    else:
        env["GDK_BACKEND"] = "x11"
        env.pop("WAYLAND_DISPLAY", None)
        number = 177
        while Path(f"/tmp/.X{number}-lock").exists():
            number += 1
        env["DISPLAY"] = f":{number}"
        xserver = subprocess.Popen(["Xvfb", env["DISPLAY"], "-screen", "0", "1280x800x24",
                                   "-nolisten", "tcp"], stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
        time.sleep(.3)
        if xserver.poll() is not None:
            raise RuntimeError("Isolated desktop failed")
    try:
        try:
            result = subprocess.run([str(args.client), str(args.output)], env=env,
                                    capture_output=True, timeout=45)
        except subprocess.TimeoutExpired as exc:
            args.output.mkdir(exist_ok=True)
            (args.output / "interaction.log").write_bytes((exc.stdout or b"") + (exc.stderr or b""))
            raise
        args.output.mkdir(exist_ok=True)
        (args.output / "interaction.log").write_bytes(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(f"Component interactions failed ({result.returncode}); inspect interaction.log")
        record = json.loads((args.output / "result.json").read_text())
        minimum, images = {"ais": (40, 9), "passage": (26, 5), "instruments": (41, 5), "energy": (44, 7), "settings": (90, 12), "anchor": (30, 5), "autopilot": (77, 7), "alerts": (50, 7), "radar": (64, 6)}[args.component]
        assert record["passed"] and record["checks"] >= minimum
        assert len(record["captures"]) == images
        record["source_commit"] = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
        record["executable_sha256"] = hashlib.sha256(args.client.read_bytes()).hexdigest()
        record["platform"] = sys.platform
        record["conformance"] = "PENDING: inspect reference/current/diff; no similarity waiver"
        record["screenshots"] = {}
        from PIL import Image, ImageChops
        reference = ROOT / "docs/design/prototype/reference" / ("windows" if windows else "linux")
        comparison = args.output / "comparison"
        comparison.mkdir()
        radar_scope = None
        for name in record["captures"]:
            image_path = args.output / f"{name}.png"
            record["screenshots"][name] = hashlib.sha256(image_path.read_bytes()).hexdigest()
            with Image.open(image_path) as current:
                assert current.size == (1280, 800)
                theme = name.rsplit("-", 1)[-1]
                background = {"day": (21, 35, 38), "dusk": (29, 40, 46),
                              "night": (12, 17, 21)}[theme]
                sample = (95, 230) if args.component in {"instruments", "energy", "radar"} else (695, 230)
                assert current.convert("RGB").getpixel(sample) == background, \
                    f"{name}: component pixels absent or wrong theme"
                if args.component in {"instruments", "energy"}:
                    surface = {"day": (29,45,49), "dusk": (37,52,59), "night": (20,28,33)}[theme]
                    assert current.convert("RGB").getpixel((125,300)) == surface, f"{name}: wind card missing/wrong theme"
                    assert current.convert("RGB").getpixel((626,250)) == surface, f"{name}: numeric tile missing/wrong theme"
                    if name in {"instruments-day", "instruments-dusk", "instruments-night"}:
                        primary = {"day": (243,245,238), "dusk": (226,229,219), "night": (184,181,167)}[theme]
                        heading = current.convert("RGB").crop((332,438,383,462))
                        ink = sum(max(abs(a-b) for a,b in zip(pixel,primary)) < 20 for pixel in heading.getdata())
                        assert ink > 20, f"{name}: heading labels displaced by wind-rose transform"
                if args.component == "autopilot":
                    primary={"day":(243,245,238),"dusk":(226,229,219),"night":(184,181,167)}[theme]
                    heading=current.convert("RGB").crop((705,130,904,154))
                    assert sum(max(abs(a-b) for a,b in zip(pixel,primary))<20 for pixel in heading.getdata())>30, f"{name}: modal left drawer title unpainted"
                    # All mode/course labels, including disabled ones, must
                    # survive a modal. Crop away borders and filled backgrounds.
                    for box in [(720,430,770,446),(811,430,856,446),(902,430,942,446),(988,430,1038,446),
                                (748,498,831,516),(946,498,1000,516),(770,568,815,584),(946,568,1000,584)]:
                        area=current.convert("RGB").crop(box)
                        assert len(area.getcolors(1000000))>8, f"{name}: modal left a pilot label unpainted at {box}"
                if args.component == "alerts" and name in {"alerts-day", "alerts-dusk", "alerts-night"}:
                    critical={"day":(236,143,135),"dusk":(236,143,135),"night":(183,117,105)}[theme]
                    backing=tuple((ink*9+base*246+127)//255 for ink,base in zip(critical,background))
                    # Top-left corners of rounded action buttons must reveal
                    # the actual callout, never native wxPanel grey.
                    assert current.convert("RGB").getpixel((722,308))==backing, f"{name}: native button backing"
                if args.component == "radar":
                    # Capability/status, stale input and replay must never
                    # paint echoes, a sweep, a heading or calibrated range.
                    # The prototype's interior uses fixed ink in all themes.
                    scope = current.convert("RGB").crop((140,240,740,690)).tobytes()
                    if radar_scope is None:
                        radar_scope = scope
                    assert scope == radar_scope, f"{name}: status changed unavailable radar image"
                if args.component == "passage" and name in {"passage-day", "passage-dusk", "passage-night"}:
                    assert current.convert("RGB").getpixel((1030, 404)) == background, \
                        f"{name}: disabled edit icon has a native grey backing"
                assert len(current.crop((705, 140, 1057, 540)).getcolors(1000000)) > 8, \
                    f"{name}: content missing"
                original = reference / f"{name}.png"
                if not original.exists():
                    continue  # Do not invent prototype online/failure-state references.
                with Image.open(original) as ref:
                    # Exact component viewport, not a layout-tolerance mask.
                    bounds = ((80, 68, 1094, 634) if args.component == "instruments"
                              else (80, 68, 1094, 766) if args.component in {"energy", "radar"}
                              else (648, 80, 1080, 754) if args.component == "settings"
                              else (682, 80, 1080, 754))
                    a, b = ref.convert("RGB").crop(bounds), current.convert("RGB").crop(bounds)
                    a.save(comparison / f"{name}-reference.png")
                    b.save(comparison / f"{name}-current.png")
                    ImageChops.difference(a, b).save(comparison / f"{name}-diff.png")
        (args.output / "capture.json").write_text(json.dumps(record, indent=2) + "\n")
        print(f"PASS: {record['checks']} native component checks; {images} captures; visual conformance pending")
    finally:
        if xserver is not None:
            xserver.terminate()
            xserver.wait(timeout=5)


if __name__ == "__main__":
    main()
