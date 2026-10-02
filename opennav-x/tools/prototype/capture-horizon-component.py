#!/usr/bin/env python3
"""Run the isolated native horizon and retain exact HTML comparisons.

Illustrative content is confined to the test executable. Geometry is compared
against computed immutable HTML, independently of the native implementation.
Raster differences remain visible evidence, never a similarity waiver.
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

from PIL import Image, ImageChops

ROOT = Path(__file__).resolve().parents[2]
CAPTURES = {
    "owned-unavailable-day", "owned-current-day", "owned-severity-day",
    "owned-identity-day", "owned-aging-day", "owned-stale-day", "owned-replay-day",
    "prototype-fixture-day", "prototype-fixture-dusk", "prototype-fixture-night",
    "prototype-responsive-125", "prototype-responsive-150",
    "prototype-hover-day", "prototype-focus-day", "prototype-large-desktop-1920",
}


# The borderless component starts at (0,0) and captures this full client size.
DESKTOP_SIZE = (1920, 1080)


def validate_capture_names(names):
    assert len(names) == len(set(names)), "Duplicate capture identity"
    assert set(names) == CAPTURES, ("Missing/unexpected component evidence", names)
    canonical = {f"prototype-fixture-{theme}" for theme in ("day", "dusk", "night")}
    assert canonical <= set(names), "All three exact HTML comparisons are mandatory"
    return canonical


def validate_capture_size(name, size):
    if name == "prototype-large-desktop-1920":
        assert size == (1920, 1080), ("Large desktop capture cropped or resized", size)


def measured_rect(item):
    return [item["rect"][key] for key in ("x", "y", "width", "height")]


def exact_geometry(native, expected, name):
    assert len(native) == len(expected) == 4, (name, native, expected)
    delta = [float(a) - float(b) for a, b in zip(native, expected)]
    # CSS fractional track allocation and native device pixels can differ by
    # at most one pixel. This bound does not excuse different composition.
    assert all(abs(value) <= 1 for value in delta), (name, native, expected, delta)
    return delta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--client", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.output.exists():
        raise SystemExit("Use a new evidence directory")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    windows = sys.platform == "win32"
    if windows and os.environ.get("GITHUB_ACTIONS") != "true":
        raise SystemExit("Native fixtures require disposable CI, never the boat")
    reference = json.loads((args.reference / "capture.json").read_text())
    original = hashlib.sha256((ROOT / "docs/design/prototype/index.html").read_bytes()).hexdigest()
    assert reference["htmlSha256"] == original
    assert reference["platform"] == ("Windows" if windows else "Linux")
    assert reference["viewport"] == dict(width=1280, height=800)
    assert reference["deviceScaleFactor"] == 1
    env = dict(os.environ)
    xserver = None
    if windows:
        spec = importlib.util.spec_from_file_location("windows_ui", ROOT / "tools/windows-ui.py")
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        ui.ensure_desktop(*DESKTOP_SIZE)
    else:
        env["GDK_BACKEND"] = "x11"
        env.pop("WAYLAND_DISPLAY", None)
        number = next(n for n in range(177, 240) if not Path(f"/tmp/.X{n}-lock").exists())
        env["DISPLAY"] = f":{number}"
        xserver = subprocess.Popen(["Xvfb", env["DISPLAY"], "-screen", "0", f"{DESKTOP_SIZE[0]}x{DESKTOP_SIZE[1]}x24", "-nolisten", "tcp"],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(.4)
        assert xserver.poll() is None
    try:
        result = subprocess.run([str(args.client.resolve()), str(args.output)], env=env,
                                capture_output=True, timeout=60)
        args.output.mkdir(exist_ok=True)
        (args.output / "interaction.log").write_bytes(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError("Horizon input/geometry failed; inspect interaction.log")
        record = json.loads((args.output / "result.json").read_text())
        assert record["passed"] is True and record["checks"] >= 208
        captures = record["captures"]
        names = [item["name"] for item in captures]
        canonical = validate_capture_names(names)
        record.update(
            source_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
            source_dirty=bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT)),
            executable_sha256=hashlib.sha256(args.client.read_bytes()).hexdigest(),
            html_sha256=original, platform=sys.platform,
            conformance="PENDING: exact images require native and boat review; no similarity waiver",
        )
        record["screenshots"] = {}
        record["comparison"] = {}
        comparison = args.output / "comparison"
        comparison.mkdir()
        for item in captures:
            name = item["name"]
            assert name and all(c.isalnum() or c in "-_" for c in name), "Unsafe capture name"
            path = args.output / (name + ".png")
            record["screenshots"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
            with Image.open(path) as src:
                current = src.convert("RGB")
            validate_capture_size(name, current.size)
            x, y, width, height = item["horizon"]
            assert width > 0 and height > 0 and x >= 0 and y >= 0
            assert x + width <= current.width and y + height <= current.height
            assert len(item["events"]) == 4
            for index, control in enumerate([item["full_passage"], *item["events"]]):
                if control == [0, 0, 0, 0]:
                    assert name in {"owned-unavailable-day", "owned-stale-day", "owned-replay-day"}
                    assert index in (3, 4), "Only the unfilled later event slots may be hidden"
                    continue
                cx, cy, cw, ch = control
                assert cw > 0 and ch > 0 and x <= cx and y <= cy
                assert cx + cw <= x + width and cy + ch <= y + height
            if name not in canonical:
                continue
            theme = name.rsplit("-", 1)[-1]
            state = reference["states"]["navigation-" + theme]
            components = state["components"]
            expected = measured_rect(components[".timeline"][0])
            deltas = {"horizon": exact_geometry(item["horizon"], expected, name)}
            deltas["heading"] = exact_geometry(item["heading"], measured_rect(components[".timeline-heading"][0]), name + " heading")
            deltas["full_passage"] = exact_geometry(item["full_passage"], measured_rect(components[".timeline-heading .text-button"][0]), name + " Full passage")
            deltas["events"] = [exact_geometry(actual, measured_rect(html), name + f" event {i}")
                                for i, (actual, html) in enumerate(zip(item["events"], components[".timeline-event"]))]
            reference_path = args.reference / ("navigation-" + theme + ".png")
            assert hashlib.sha256(reference_path.read_bytes()).hexdigest() == state["screenshotSha256"]
            box = (int(expected[0]), int(expected[1]), int(expected[0] + expected[2]), int(expected[1] + expected[3]))
            with Image.open(reference_path) as src:
                ref = src.convert("RGB").crop(box)
            crop = current.crop(box)
            diff = ImageChops.difference(ref, crop)
            ref.save(comparison / (theme + "-reference.png"))
            crop.save(comparison / (theme + "-current.png"))
            diff.save(comparison / (theme + "-diff.png"))
            record["comparison"][theme] = {
                "bounds": expected, "geometry_delta": deltas,
                "changed_pixels": sum(pixel != (0, 0, 0) for pixel in diff.getdata()),
            }
        (args.output / "capture.json").write_text(json.dumps(record, indent=2) + "\n")
        print(f"PASS {record['checks']} horizon checks; {len(captures)} actual captures; exact differences retained")
    finally:
        if xserver is not None:
            xserver.terminate()
            xserver.wait(timeout=5)


if __name__ == "__main__":
    main()
