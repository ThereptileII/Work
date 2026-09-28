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
    parser.add_argument("--ais-settings", action="store_true", help="Exercise fixture-free AIS settings without a key")
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
    if not windows:
        env["GDK_BACKEND"] = "x11"
        env.pop("WAYLAND_DISPLAY", None)
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

    def click(label, expected_light=None):
        before = data()
        ticks = int(before["runtime"]["ui_update"]["ticks"])
        controls = [c for c in before["runtime"]["display"]["interaction_controls"]
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
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            current = data()
            current_light = current["runtime"]["display"]["light"]
            if int(current["runtime"]["ui_update"]["ticks"]) >= ticks+3 and (not expected_light or current_light == expected_light):
                break
            time.sleep(.1)
        else:
            raise AssertionError(f"{label}: application did not settle in the requested state {expected_light or ''}")
        time.sleep(.3)  # allow the invalidated native surfaces to finish painting

    def capture(name):
        path = args.output / f"{name}.png"
        client_origin = (0, 0)
        if windows:
            from PIL import Image
            assert ui.IsWindowEnabled(window), "Unexpected modal dialog"
            outer = path.with_name(name + "-outer.png")
            ui.capture(window, outer, resize=False, screen_pixels=True)
            origin = ui.W.POINT(0, 0)
            client_to_screen = ui.declare(ui.user, "ClientToScreen", ui.W.BOOL, ui.W.HWND, ui.C.POINTER(ui.W.POINT))
            assert client_to_screen(window, ui.C.byref(origin))
            client_origin = (origin.x, origin.y)
            rect = ui.W.RECT()
            assert ui.GetWindowRect(window, ui.C.byref(rect))
            x, y = origin.x-rect.left, origin.y-rect.top
            with Image.open(outer) as image:
                image.crop((x, y, x+1280, y+800)).save(path)
        else:
            subprocess.run(["import", "-window", "root", str(path)], env=env, check=True)
        snapshot = data()
        if name.startswith("traffic-") or name.startswith("online-ais-"):
            drawer = snapshot["runtime"]["display"]["drawer"]
            expected = dict(x=client_origin[0]+682, y=client_origin[1]+80, width=398, height=674)
            assert all(abs(drawer[k]-v) <= 1 for k, v in expected.items()), "AIS drawer differs from prototype geometry"
            from PIL import Image
            theme = snapshot["runtime"]["display"]["light"].lower()
            tokens = json.loads((ROOT / "docs/design/prototype-tokens.json").read_text())["themes"][theme]
            background = tuple(bytes.fromhex(tokens["--bg"].removeprefix("#")))
            def sheet_painted():
                with Image.open(path) as image:
                    image = image.convert("RGB")
                    return all(max(abs(a-b) for a,b in zip(image.getpixel(point),background)) <= 3
                               for point in [(686, 417), (1076, 417), (881, 84)])
            paint_start = time.monotonic()
            # Diagnose asynchronous native stacking/paint without relaxing a
            # single pixel criterion. Record the latency; it is a UX finding.
            while not sheet_painted() and not windows and time.monotonic()-paint_start < 3:
                time.sleep(.1)
                subprocess.run(["import", "-window", "root", str(path)], env=env, check=True)
            record.setdefault("drawer_paint_wait_seconds", {})[name] = time.monotonic()-paint_start
            assert sheet_painted(), "AIS sheet reports visible but is not painted above the chart"
            record.setdefault("drawer_layout", []).append(dict(file=name, actual=drawer, expected=expected))
        if name.startswith("navigation-"):
            spec = importlib.util.spec_from_file_location("chart_layout", ROOT / "tools/chart-render-check.py")
            geometry = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(geometry)
            client = dict(x=client_origin[0], y=client_origin[1], width=1280, height=800)
            frame = dict(x=rect.left,y=rect.top,width=rect.right-rect.left,height=rect.bottom-rect.top) if windows else dict(client)
            record.setdefault("layout", []).append(geometry.navigation_layout(snapshot["runtime"]["display"], frame, client))
            # Compare independently measured HTML metric geometry, allowing
            # only integer raster rounding, not broad layout similarity.
            reference = json.loads((ROOT / "docs/design/prototype/reference" /
                                  ("windows" if windows else "linux") / "capture.json").read_text())
            components = reference["states"]["navigation-day"]["components"]
            actual_rows = snapshot["runtime"]["display"]["rail_regions"]
            for actual, expected in zip(actual_rows, components[".metric"]):
                content = dict(x=actual["x"]-client_origin[0]+19,
                               y=actual["y"]-client_origin[1],
                               width=actual["width"]-37,height=actual["height"])
                assert all(abs(content[k]-expected["rect"][k]) <= 1 for k in content), "Rail row differs from canonical HTML geometry"
            pilot = [c for c in snapshot["runtime"]["display"]["interaction_controls"]
                     if c["label"] == "Autopilot" and c["visible"]]
            assert len(pilot)==1
            actual_pilot={k:pilot[0][k] for k in ("x","y","width","height")}
            actual_pilot["x"]-=client_origin[0]; actual_pilot["y"]-=client_origin[1]
            assert all(abs(actual_pilot[k]-components[".autopilot-summary"][0]["rect"][k]) <= 1
                       for k in actual_pilot), "Pilot summary differs from canonical HTML geometry"
            record.setdefault("prototype_rail_geometry", []).append(dict(file=name,pilot=actual_pilot))
            from PIL import Image
            theme = snapshot["runtime"]["display"]["light"].lower()
            tokens = json.loads((ROOT / "docs/design/prototype-tokens.json").read_text())["themes"][theme]
            ink = tuple(bytes.fromhex(tokens["--float-text"].removeprefix("#")))
            surface = tuple(bytes.fromhex(tokens["--floating"].removeprefix("#")))
            with Image.open(path) as img:
                pixels = img.convert("RGB")
                for label in ("Measure", "Waypoint", "+", "−", "North", "Follow boat"):
                    found = [c for c in snapshot["runtime"]["display"]["interaction_controls"]
                             if c["label"] == label and c["visible"]]
                    assert len(found) == 1, f"Floating control {label} missing/ambiguous"
                    control = found[0]
                    left, top = control["x"]-client_origin[0]+6, control["y"]-client_origin[1]+6
                    right, bottom = left+control["width"]-12, top+control["height"]-12
                    assert 0 <= left < right <= 1280 and 0 <= top < bottom <= 800
                    sample = [pixels.getpixel((x, y)) for x in range(left, right) for y in range(top, bottom)]
                    close = lambda color, target: max(abs(a-b) for a, b in zip(color, target)) <= 3
                    assert sum(close(c, surface) for c in sample) > len(sample)*.5, f"Floating {label} surface occluded"
                    # Thin SVG strokes can be entirely antialiased. Require
                    # at least half-coverage ink, never merely a changed pixel.
                    ink_distance = sum((a-b)**2 for a, b in zip(surface, ink))
                    def ink_coverage(color):
                        projection = sum((a-b)*(a-c) for a,b,c in zip(surface,color,ink)) / ink_distance
                        return projection >= .5 and all(min(a,b)-3 <= c <= max(a,b)+3 for a,b,c in zip(surface,ink,color))
                    assert sum(ink_coverage(c) for c in sample) >= 5, f"Floating {label} glyph/text absent (visible flag alone is insufficient)"
            record.setdefault("painted_controls", []).append(name)
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
        # Settings are a required new service flow, not a fabricated HTML state.
        group = "additional_captures" if name.startswith("online-ais-") else "captures"
        record.setdefault(group, []).append({"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})

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
        resize_ticks = int(data()["runtime"]["ui_update"]["ticks"])
        resize_started = time.time_ns()
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
            matches = xdo("search", "--all", "--onlyvisible", "--pid", app.pid, "--name", "^OpenNav X / OpenCPN$").splitlines()
            record["matched_windows"] = [{"id": item, "name": xdo("getwindowname", item), "geometry": xdo("getwindowgeometry", item)} for item in matches]
            assert len(matches) == 1, "The capture must resize the main frame, not an owned floating surface"
            window = matches[0]
            xdo("windowsize", window, 1280, 800)
            xdo("windowmove", window, 0, 0)
        # Coordinates are copied by the application tick. A fixed delay can
        # still leave the pre-resize overlay coordinates in diagnostics while
        # a fresh ENC/SENC or GL canvas finishes work. Never click that snapshot.
        settle = time.monotonic()
        while time.monotonic() - settle < 10:
            observed = data()
            if ((profile / "opennav-diagnostics.json").stat().st_mtime_ns > resize_started
                    and int(observed["runtime"]["ui_update"]["ticks"]) >= resize_ticks + 3
                    and observed["runtime"]["chart"]["canvas_pixels"] == {"width":1014,"height":566}):
                break
            time.sleep(.1)
        else:
            raise AssertionError("Current target-size canvas did not settle after resize")
        record["post_resize_settle_seconds"] = time.monotonic() - settle
        # Real input must reach the floating window's action, not the canvas
        # beneath it. Check the existing OpenCPN scale, never an independent
        # navigation calculation or a UI-only selected flag.
        scale = data()["runtime"]["chart"]["scale_ppm"]
        click("+")
        zoomed = data()["runtime"]["chart"]["scale_ppm"]
        assert zoomed > scale, "Floating zoom-in action did not reach OpenCPN"
        click("−")
        assert data()["runtime"]["chart"]["scale_ppm"] < zoomed, "Floating zoom-out action did not reach OpenCPN"
        record["floating_zoom_input"] = "Actual pointer input changes the upstream viewport scale in both directions"
        capture("navigation-day")
        click("Day", "Dusk")
        capture("navigation-dusk")
        click("Dusk", "Night")
        capture("navigation-night")
        click("Night", "Day")
        if args.navigation_only:
            capture("navigation-return-day")
            record["result"] = "chart cycle captured; semantic and visual review required"
            return
        for label, name in [("Passage", "passage"), ("Traffic", "traffic"), ("Energy", "energy"),
                            ("Instruments", "instruments"), ("Settings", "settings")]:
            click(label)
            capture(name + "-day")
            if label == "Traffic" and args.ais_settings:
                click("Online AIS settings")
                capture("online-ais-settings-day")
                # The isolated runner must have no product credential. Never
                # enable a test against somebody's saved or environment key.
                online = data()["runtime"]["online_ais"]
                assert not online["credential_present"], "Settings test requires an empty product credential store"
                assert not online["enabled"], "New isolated profile must default OFF"
                click("Enabled")
                deadline = time.monotonic()+5
                while time.monotonic()<deadline:
                    online = data()["runtime"]["online_ais"]
                    if online["enabled"] and online["connection_state"] == 1:
                        break  # CredentialMissing: no network connection attempted
                    time.sleep(.1)
                else:
                    raise AssertionError("Missing key was not explicitly withheld")
                capture("online-ais-key-needed-day")
                click("Off")
                assert not data()["runtime"]["online_ais"]["enabled"]
                click("Day", "Dusk")
                capture("online-ais-settings-dusk")
                click("Dusk", "Night")
                capture("online-ais-settings-night")
                click("Night", "Day")
                click("Back")
                click("Close")
                assert "drawer" not in data()["runtime"]["display"], "Close did not dismiss sheet"
                record["online_ais_settings"] = "Default OFF; missing key withholds connection; OFF, themes and Close exercised"
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
