#!/usr/bin/env python3
"""Isolated native composition review; never connects to the boat/profile.

Captures the actual installed OpenCPN canvas and owned XNav controls. Linux
font fallback is development evidence, not Windows or boat visual acceptance.
Run serially with upstream tests: their REST server uses the same fixed port.
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
import io
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from diagnostic_snapshot import read_json_snapshot


def public_enc():
    """Reuse the inspected, public NOAA fixture; never use a user's charts."""
    cache = ROOT / "build/chart-fixtures"
    cache.mkdir(parents=True, exist_ok=True)
    path = cache / "US5SEAFL.zip"
    url = "https://www.charts.noaa.gov/ENCs/US5SEAFL.zip"
    digest = "b027e029dc7b76595381d89e3718145eb5069e5a03917e78bada4f8ebe8c84e4"
    if not path.exists():
        with urllib.request.urlopen(url, timeout=45) as response:
            content = response.read(64 * 1024 * 1024 + 1)
        assert len(content) <= 64 * 1024 * 1024
        assert hashlib.sha256(content).hexdigest() == digest, "NOAA fixture changed; review before repinning"
        path.write_bytes(content)
    content = path.read_bytes()
    assert hashlib.sha256(content).hexdigest() == digest
    with zipfile.ZipFile(io.BytesIO(content)) as archive:
        assert sum(f.file_size for f in archive.infolist()) < 256 * 1024 * 1024
        for f in archive.infolist():
            name = Path(f.filename)
            assert not name.is_absolute() and ".." not in name.parts and "\\" not in f.filename and ":" not in f.filename
            assert (f.external_attr >> 16) & 0o170000 != 0o120000
        archive.extractall(cache)
    charts = cache / "ENC_ROOT"
    assert (charts / "US5SEAFL/US5SEAFL.000").is_file()
    return charts, {"url": url, "sha256": digest,
                    "scope": "Public rendering fixture, not redistributed as a navigation product"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    windows = sys.platform == "win32"
    if windows and os.environ.get("GITHUB_ACTIONS") != "true":
        raise SystemExit("Windows capture requires a disposable CI desktop; use guarded boat tools aboard.")
    ui = None
    if windows:
        spec = importlib.util.spec_from_file_location("windows_ui", ROOT / "tools/windows-ui.py")
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
    parser.add_argument("--build", type=Path, default=ROOT / ("build/production-windows" if windows else "build/production-linux"))
    parser.add_argument("--app", type=Path, default=ROOT / ("build/production-install/opencpn.exe" if windows else "build/production-install/bin/opencpn"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--chart-style", choices=["XNav", "Standard"], default="XNav")
    parser.add_argument("--renderer", choices=["software", "opengl"], default="software")
    parser.add_argument("--public-enc", action="store_true")
    parser.add_argument("--navigation-only", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    profile = args.output.resolve() / "profile"
    subprocess.run([sys.executable, str(ROOT / "tools/prepare-test-profile.py"),
                    "--build", str(args.build), "--profile", str(profile)], check=True)
    with (profile / "opencpn.conf").open("a") as stream:
        stream.write(f"\n[Settings]\nOpenGL={int(args.renderer == 'opengl')}\n"
                     f"[OpenNav]\nChartPresentationV1={args.chart_style}\n")
        if args.public_enc:
            charts, chart_provenance = public_enc()
            stream.write(f"\n[ChartDirectories]\nChartDir1={charts.as_posix()}\n"
                         "[Settings/GlobalState]\nVPLatLon=47.6000,-122.3600\nVPScale=0.15\n")
        else:
            stream.write("[Settings/GlobalState]\nVPLatLon=59.0800,18.5000\nVPScale=0.001\n")
    number = 171
    while Path(f"/tmp/.X{number}-lock").exists():
        number += 1
    env = dict(os.environ, DISPLAY=f":{number}")
    record = {"authority": "Native Windows development" if windows else "Linux development only", "size": [1280, 800],
              "renderer": args.renderer, "chart_style": args.chart_style,
              "chart": chart_provenance if args.public_enc else "OpenCPN coastline reference",
              "input": "none; isolated disposable profile",
              "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "executable_sha256": hashlib.sha256(args.app.read_bytes()).hexdigest(), "captures": []}
    xserver = None if windows else subprocess.Popen(["Xvfb", env["DISPLAY"], "-screen", "0", "1280x800x24", "-nolisten", "tcp"],
                               env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if windows:
        record["desktop"] = ui.ensure_desktop(1440, 900)
    app = None
    window = None
    log = (args.output / "launch.log").open("w")

    def xdo(*command):
        return subprocess.check_output(["xdotool", *map(str, command)], env=env, text=True).strip()

    def data():
        return read_json_snapshot(profile / "opennav-diagnostics.json")

    def click(label):
        controls = [c for c in data()["runtime"]["display"]["interaction_controls"]
                    if c["label"] == label and c["visible"] and c["enabled"]]
        if len(controls) != 1:
            raise RuntimeError(f"Expected one visible enabled {label!r}; got {len(controls)}")
        c = controls[0]
        x, y = c["x"] + c["width"] // 2, c["y"] + c["height"] // 2
        if windows:
            assert ui.IsWindowEnabled(window), "Unexpected modal dialog"
            target = ui.WindowFromPoint(ui.W.POINT(x, y))
            owner = ui.W.DWORD()
            ui.GetWindowThreadProcessId(target, ui.C.byref(owner))
            assert owner.value == app.pid, "Capture input would land outside owned application"
            ui.SetForegroundWindow(window)
            ui.SetCursorPos(x, y)
            ui.MouseEvent(2, 0, 0, 0, 0)
            ui.MouseEvent(4, 0, 0, 0, 0)
            ui.SetCursorPos(0, 0)
        else:
            xdo("mousemove", x, y, "click", "1")
            xdo("mousemove", 0, 0)
        time.sleep(.6)

    def capture(name):
        path = args.output / f"{name}.png"
        if windows:
            from PIL import Image
            assert ui.IsWindowEnabled(window), "Unexpected modal dialog"
            outer = path.with_name(name + "-outer.png")
            ui.capture(window, outer, resize=False, screen_pixels=True)
            origin = ui.W.POINT(0, 0)
            client_to_screen = ui.declare(ui.user, "ClientToScreen", ui.W.BOOL, ui.W.HWND, ui.C.POINTER(ui.W.POINT))
            assert client_to_screen(window, ui.C.byref(origin))
            rect = ui.W.RECT()
            assert ui.GetWindowRect(window, ui.C.byref(rect))
            x, y = origin.x-rect.left, origin.y-rect.top
            with Image.open(outer) as image:
                image.crop((x, y, x+1280, y+800)).save(path)
        else:
            subprocess.run(["import", "-window", "root", str(path)], env=env, check=True)
        snapshot = data()
        style = snapshot["runtime"]["chart_presentation"]
        assert style["requested"] == args.chart_style, "Requested chart style was not retained"
        assert style["status"].startswith(args.chart_style + " "), "Requested chart style was not active"
        assert snapshot["runtime"]["chart"]["opengl_enabled"] == (args.renderer == "opengl"), "Requested renderer was not active"
        if args.navigation_only and not args.public_enc and args.chart_style == "XNav":
            from collections import Counter
            from PIL import Image
            theme = snapshot["runtime"]["display"]["light"].lower()
            tokens = json.loads((ROOT / "docs/design/prototype-tokens.json").read_text())["themes"][theme]
            with Image.open(path) as img:
                pixels = img.convert("RGB")
                colors = Counter(pixels.getpixel((x, y)) for x in range(120, 1050, 4) for y in range(150, 630, 4))
            for key in ("--land", "--water"):
                color = tuple(bytes.fromhex(tokens[key].removeprefix("#")))
                assert colors[color] / sum(colors.values()) > .02, f"{name}: {key} is absent or not prototype-colored"
        if args.public_enc:
            from collections import Counter
            from PIL import Image
            assert any(c["file"] == "US5SEAFL.000" for c in snapshot["runtime"]["chart"].get("quilt_members", [])), "Expected actual ENC in upstream quilt"
            with Image.open(path) as img:
                pixels = img.convert("RGB")
                colors = Counter(pixels.getpixel((x, y)) for y in range(180, 600, 2) for x in range(150, 950, 2))
            detail = sum(v for _, v in colors.most_common()[3:]) / sum(colors.values())
            assert len(colors) > 20 and detail > .005, "ENC details are absent"
            record.setdefault("chart_checks", []).append({"file": path.name, "colors": len(colors), "detail_fraction": detail})
        record["executable_build_commit"] = snapshot["build_commit"]
        (args.output / f"{name}.json").write_text(json.dumps(snapshot, indent=2) + "\n")
        record["captures"].append({"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})

    try:
        time.sleep(.6)
        app = subprocess.Popen([str(args.app), "--configdir", str(profile), "--xnav"] +
                               (["--rebuild_chart_db"] if args.public_enc else []) +
                               (["--no_opengl"] if args.renderer == "software" else []),
                               env=env, stdout=log, stderr=log)
        deadline = time.monotonic() + 75
        while time.monotonic() < deadline:
            if app.poll() is not None:
                raise RuntimeError("Application exited during startup")
            path = profile / "opencpn.log"
            if path.exists() and "OnInitTimer...Finalize Canvases" in path.read_text(errors="replace"):
                break
            time.sleep(.2)
        else:
            raise RuntimeError("Application did not finish initialization")
        if windows:
            window, pid = ui.wait_window("OpenNav X / OpenCPN", app.pid)
            assert ui.GetDpiForWindow(window) == 96, "Primary reference requires 96 DPI"
            rect, client = ui.W.RECT(), ui.W.RECT()
            assert ui.GetWindowRect(window, ui.C.byref(rect))
            assert ui.GetClientRect(window, ui.C.byref(client))
            width = 1280 + (rect.right-rect.left) - client.right
            height = 800 + (rect.bottom-rect.top) - client.bottom
            assert ui.SetWindowPos(window, None, 0, 0, width, height, 0)
            time.sleep(.5)
            assert ui.GetClientRect(window, ui.C.byref(client))
            assert (client.right, client.bottom) == (1280, 800)
            record["captureScope"] = "1280x800 native client crop; full outer capture retained; boat fullscreen pending"
        else:
            window = xdo("search", "--onlyvisible", "--pid", app.pid, "--name", "^OpenNav X / OpenCPN$").splitlines()[0]
            xdo("windowsize", window, 1280, 800)
            xdo("windowmove", window, 0, 0)
        time.sleep(1)
        capture("navigation-day")
        click("Day")
        capture("navigation-dusk")
        click("Dusk")
        capture("navigation-night")
        click("Night")
        if args.navigation_only:
            capture("navigation-return-day")
            record["result"] = "chart cycle captured; semantic and visual review required"
            return
        for label, name in [("Passage", "passage"), ("Traffic", "traffic"), ("Energy", "energy"),
                            ("Instruments", "instruments"), ("Settings", "settings")]:
            click(label)
            capture(name + "-day")
        record["result"] = "captured; conformance not asserted"
    finally:
        shutdown_error = None
        if app and app.poll() is None:
            try:
                if windows:
                    if not window:
                        window, _ = ui.wait_window("OpenNav X / OpenCPN", app.pid, timeout=5)
                    ui.close(window)  # normal WM_CLOSE; same path as the window close button
                else:
                    subprocess.run([str(args.app), "--configdir", str(profile), "--remote", "--quit"],
                                   env=env, stdout=log, stderr=log, timeout=15, check=True)
                app.wait(timeout=30)
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError, AssertionError, RuntimeError) as error:
                # Only this tool's disconnected disposable child is eligible.
                app.kill()
                app.wait()
                shutdown_error = type(error).__name__
        if app:
            record["exit_code"] = app.returncode
        log.close()
        if xserver:
            xserver.terminate()
            xserver.wait(timeout=10)
        (args.output / "capture.json").write_text(json.dumps(record, indent=2) + "\n")
        if shutdown_error:
            raise RuntimeError(f"Disposable native review shutdown failed: {shutdown_error}")
        if app and app.returncode != 0:
            raise RuntimeError(f"Native application exit was not clean: {app.returncode}")


if __name__ == "__main__":
    main()
