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
    parser.add_argument("--depth-unit", choices=["feet", "meters", "fathoms"],
                        help="Set the upstream ENC display unit in this disposable profile")
    parser.add_argument("--navigation-only", action="store_true")
    parser.add_argument("--view", choices=["passage", "traffic", "energy", "instruments", "autopilot", "alerts", "health", "anchor", "radar", "settings"],
                        help="Focused local correction; CI defaults to every view")
    parser.add_argument("--health-reference", type=Path,
                        help="Fresh same-platform Source Health render from the unchanged HTML")
    parser.add_argument("--ais-settings", action="store_true", help="Exercise fixture-free AIS settings without a key")
    parser.add_argument("--review-window-guard", action="store_true",
                        help="Windows CI: qualify guarded boat capture against actual native owned windows")
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
        if args.depth_unit:
            if not args.public_enc:
                raise SystemExit("Depth-unit checks require the actual public ENC")
            stream.write("[Settings/GlobalState]\nS52_DEPTH_UNIT_SHOW=" +
                         str(["feet", "meters", "fathoms"].index(args.depth_unit)) + "\n")
    number = 171
    while Path(f"/tmp/.X{number}-lock").exists():
        number += 1
    env = dict(os.environ, DISPLAY=f":{number}")
    if not windows:
        env["GDK_BACKEND"] = "x11"
        env.pop("WAYLAND_DISPLAY", None)
    record = {"authority": "Native Windows development" if windows else "Linux development only", "size": [1280, 800],
              "selected_view": args.view,
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

    def click(label, expected_light=None, in_drawer=False):
        before = data()
        ticks = int(before["runtime"]["ui_update"]["ticks"])
        controls = [c for c in before["runtime"]["display"]["interaction_controls"]
                    if c["label"] == label and c["visible"] and c["enabled"]]
        if in_drawer:
            bounds = before["runtime"]["display"]["drawer"]
            controls = [c for c in controls if bounds["x"] <= c["x"] and bounds["y"] <= c["y"]
                        and c["x"]+c["width"] <= bounds["x"]+bounds["width"]
                        and c["y"]+c["height"] <= bounds["y"]+bounds["height"]]
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
            assert ui.IsWindowEnabled(target) and ui.text(target) == label, "Native input target differs from requested action"
            # Activate the target's own top-level surface. Activating the main
            # frame after hit-testing an owned sheet can change its stacking.
            ancestor = ui.declare(ui.user, "GetAncestor", ui.W.HWND, ui.W.HWND, ui.W.UINT)
            foreground = ui.declare(ui.user, "GetForegroundWindow", ui.W.HWND)
            surface = ancestor(target, 2)  # GA_ROOT: parents, not owner chain.
            ui.SetForegroundWindow(surface)
            deadline = time.monotonic()+3
            while foreground() != surface and time.monotonic() < deadline:
                time.sleep(.05)
            assert foreground() == surface, "Input surface did not receive foreground activation"
            ui.SetCursorPos(x, y)
            assert ui.WindowFromPoint(ui.W.POINT(x,y)) == target, "Activation obscured input target"
            record.setdefault("pointer_hits", []).append(dict(label=label,x=x,y=y,window=int(target),surface=int(surface)))
            ui.MouseEvent(2, 0, 0, 0, 0)
            time.sleep(.05)
            ui.MouseEvent(4, 0, 0, 0, 0)
        else:
            xdo("mousemove", x, y)
            hit = xdo("getmouselocation", "--shell")
            record.setdefault("pointer_hits", []).append(dict(label=label, x=x, y=y, x11=hit))
            # Match the native Windows pointer press duration. An immediate
            # X11 down/up pair can precede GTK activation of an owned surface.
            # This is one held click, not a retry after an unobserved action.
            xdo("mousedown", "1", "sleep", "0.05", "mouseup", "1")
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            current = data()
            current_light = current["runtime"]["display"]["light"]
            # Diagnostics publishes once per second. A tick count alone can
            # advance from a stale pre-click file while still describing the
            # old drawer. Observe the semantic result; never retry the input.
            closed = label != "Close" or ("drawer" not in current["runtime"]["display"] and current["ui_page"] == "Navigation")
            expected_page = None if in_drawer else {"Chart":"Navigation", "Passage":"Route", "Full passage":"Route", "Traffic":"AIS targets", "Energy":"Energy", "Instruments":"Vessel instruments", "Autopilot":"Manual autopilot", "Anchor":"Anchor watch", "Settings":"Settings", "Alerts":"Alerts", "Radar":"Radar status"}.get(label)
            if c["accessible_name"] == "Inspect source health":
                expected_page = "Source health"
            if label == "Online AIS settings":
                expected_page = "Online AIS settings"
            if label == "Back" and before["ui_page"] == "Online AIS settings":
                expected_page = "AIS targets"
            page_ready = not expected_page or current["ui_page"] == expected_page
            online_ready = True
            if before["ui_page"] == "Online AIS settings" and label in ("Off", "Enabled"):
                online_ready = current["runtime"]["online_ais"]["enabled"] == (label == "Enabled")
                if label == "Off":
                    online_ready = online_ready and current["runtime"]["online_ais"]["connection_state"] == 0
            if int(current["runtime"]["ui_update"]["ticks"]) >= ticks+3 and (not expected_light or current_light == expected_light) and closed and page_ready and online_ready:
                break
            time.sleep(.1)
        else:
            raise AssertionError(f"{label}: application did not settle in the requested state {expected_light or ''}")
        # Keep the pointer on the target until its native input event has been
        # consumed. Moving it away immediately can cancel a GTK release before
        # dispatch. Do not retry a click or substitute a command event.
        if windows: ui.SetCursorPos(0, 0)
        else: xdo("mousemove", 0, 0)
        time.sleep(.3)  # allow the invalidated native surfaces to finish painting

    def capture(name):
        path = args.output / f"{name}.png"
        client_origin = (0, 0)
        if windows:
            from PIL import Image
            assert ui.IsWindowEnabled(window), "Unexpected modal dialog"
            outer = path.with_name(name + "-outer.png")
            if args.review_window_guard:
                reviewed = subprocess.run([
                    "powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
                    str(ROOT / "tools/prototype/capture-reviewed-native.ps1"),
                    "-ProcessId", str(app.pid), "-Handle", str(window), "-Output", str(outer.resolve())],
                    capture_output=True, text=True, errors="replace",
                    creationflags=subprocess.CREATE_NO_WINDOW, timeout=15)
                # CREATE_NO_WINDOW does not reliably inherit the CI console's
                # stderr. Retain the actual refusal instead of only Python's
                # exit-code traceback. This runner uses a disposable profile
                # without credentials or live sources, never the boat profile.
                path.with_suffix(".guard.log").write_text(reviewed.stdout + reviewed.stderr, encoding="utf-8")
                if reviewed.returncode:
                    path.with_suffix(".rejected.json").write_text(json.dumps(data(), indent=2), encoding="utf-8")
                    raise AssertionError("Native capture guard refused " + name + ": " + reviewed.stderr.strip())
                guarded = json.loads(Path(str(outer) + ".json").read_text(encoding="utf-8-sig"))
                record.setdefault("guarded_windows", {})[name] = guarded
            else:
                ui.capture(window, outer, resize=False, screen_pixels=True)
            origin = ui.W.POINT(0, 0)
            client_to_screen = ui.declare(ui.user, "ClientToScreen", ui.W.BOOL, ui.W.HWND, ui.C.POINTER(ui.W.POINT))
            assert client_to_screen(window, ui.C.byref(origin))
            client_origin = (origin.x, origin.y)
            rect = ui.W.RECT()
            assert ui.GetWindowRect(window, ui.C.byref(rect))
            x, y = origin.x-rect.left, origin.y-rect.top
            if args.review_window_guard:
                x, y = origin.x-guarded["Bounds"]["Left"], origin.y-guarded["Bounds"]["Top"]
            with Image.open(outer) as image:
                image.crop((x, y, x+1280, y+800)).save(path)
        else:
            subprocess.run(["import", "-window", "root", str(path)], env=env, check=True)
        snapshot = data()
        # Actual native controls, compared with independently rendered HTML.
        reference = json.loads((ROOT / "docs/design/prototype/reference" /
                                ("windows" if windows else "linux") / "capture.json").read_text())
        if name in ("health-day", "health-dusk", "health-night"):
            health_reference = reference
            if not windows:
                health_reference = json.loads((ROOT / "docs/design/prototype/reference/linux/health-disclosures.json").read_text())
            if args.health_reference:
                health_reference = json.loads((args.health_reference / "capture.json").read_text())
                for key in ("htmlSha256", "platform", "viewport", "deviceScaleFactor"):
                    assert reference[key] == health_reference[key], "Mismatched health reference " + key
            expected_rows = health_reference["states"][name]["components"][".sensor-details"]
            rows = []
            for index, source_id in enumerate(("gps", "heading", "depth", "wind", "motor", "battery")):
                actual = [c for c in snapshot["runtime"]["display"]["interaction_controls"]
                          if c["accessible_name"] == "Inspect source " + source_id and c["visible"]]
                assert len(actual) == 1, "Unique visible Source Health disclosure required: " + source_id
                rect = {k:actual[0][k] for k in ("x", "y", "width", "height")}
                rect["x"] -= client_origin[0]; rect["y"] -= client_origin[1]
                if windows:
                    assert all(abs(rect[k]-v) <= 1 for k,v in expected_rows[index]["rect"].items()), (source_id, "disclosure differs from HTML", rect, expected_rows[index]["rect"])
                rows.append(rect)
            record.setdefault("source_health_layout", {})[name] = rows
        if windows and name in ("settings-day","settings-dusk","settings-night","sensors-day","display-day","system-day"):
            measured=json.loads((ROOT / "docs/design/prototype/reference/windows/settings-tabs.json").read_text())
            assert measured["htmlSha256"]==reference["htmlSha256"] and measured["platform"]=="Windows"
            assert measured["states"][name]["screenshotSha256"]==reference["states"][name]["screenshotSha256"]
            expected_tabs=measured["states"][name]["tabs"]
            assert len(expected_tabs)==8, "All independent section measurements required"
            drawer=snapshot["runtime"]["display"]["drawer"]
            for label, expected in expected_tabs.items():
                actual=[c for c in snapshot["runtime"]["display"]["interaction_controls"]
                        if c["label"]==label and c["visible"] and drawer["x"]<=c["x"]<drawer["x"]+drawer["width"]
                        and drawer["y"]<=c["y"]<drawer["y"]+drawer["height"]]
                assert len(actual)==1, (label,"unique Preferences section")
                rect={k:actual[0][k] for k in ("x","y","width","height")}
                rect["x"]-=client_origin[0];rect["y"]-=client_origin[1]
                assert all(abs(rect[k]-expected[k])<=1 for k in rect), (label,"tab differs from HTML",rect,expected)
        rail_reference = reference["states"]["navigation-day"]["components"][".nav-btn"][:8]
        sidebar = reference["states"]["navigation-day"]["components"][".sidebar"][0]["rect"]
        rail_actual = []
        for label, expected in zip(("Chart", "Passage", "Traffic", "Energy", "Instruments", "Anchor", "Radar", "Settings"), rail_reference):
            # Preferences has its own Radar tab. Identify the navigation role
            # by the independently rendered sidebar, then require exactly one
            # item and the unchanged exact control rectangle within it.
            controls = [c for c in snapshot["runtime"]["display"]["interaction_controls"]
                        if c["label"] == label and c["visible"]
                        and sidebar["x"] <= c["x"]-client_origin[0]
                        and c["x"]-client_origin[0]+c["width"] <= sidebar["x"]+sidebar["width"]
                        and sidebar["y"] <= c["y"]-client_origin[1]
                        and c["y"]-client_origin[1]+c["height"] <= sidebar["y"]+sidebar["height"]]
            assert len(controls) == 1, f"Prototype rail {label} absent/ambiguous"
            c = controls[0]
            actual = dict(x=c["x"]-client_origin[0], y=c["y"]-client_origin[1], width=c["width"], height=c["height"])
            assert all(abs(actual[k]-v) <= 1 for k,v in expected["rect"].items()), f"{label} rail geometry differs: {actual}"
            rail_actual.append(actual)
        record.setdefault("left_rail_layout", {})[name] = rail_actual
        if name.startswith(("traffic-", "online-ais-", "passage-", "anchor-", "autopilot-", "alerts-", "health-", "settings-", "sensors-", "display-", "system-")):
            drawer = snapshot["runtime"]["display"]["drawer"]
            wide = name.startswith(("settings-", "sensors-", "display-", "system-"))
            expected = dict(x=client_origin[0]+(648 if wide else 682), y=client_origin[1]+80, width=432 if wide else 398, height=674)
            assert all(abs(drawer[k]-v) <= 1 for k, v in expected.items()), "Drawer differs from prototype geometry"
            from PIL import Image
            theme = snapshot["runtime"]["display"]["light"].lower()
            tokens = json.loads((ROOT / "docs/design/prototype-tokens.json").read_text())["themes"][theme]
            background = tuple(bytes.fromhex(tokens["--bg"].removeprefix("#")))
            pilot_controls=[]
            if name.startswith("autopilot-"):
                expected_buttons=reference["states"]["autopilot-day"]["components"][".btn"][:8]
                for label, expected_button in zip(("−10°","−1°","+1°","+10°","Standby","Auto","Track","Wind"),expected_buttons):
                    found=[c for c in snapshot["runtime"]["display"]["interaction_controls"] if c["label"]==label and c["visible"]]
                    assert len(found)==1,(label,"unique pilot control required")
                    actual={k:found[0][k] for k in ("x","y","width","height")}
                    actual["x"]-=client_origin[0];actual["y"]-=client_origin[1]
                    # Windows is typography/geometry authority. Linux uses
                    # these same Windows-derived integer positions even when
                    # its fallback font changes the HTML's natural line box.
                    if windows:
                        assert all(abs(actual[k]-v)<=1 for k,v in expected_button["rect"].items()),(label,"pilot differs from HTML",actual,expected_button["rect"])
                    pilot_controls.append(actual)
            def sheet_painted():
                with Image.open(path) as image:
                    image = image.convert("RGB")
                    background_painted=all(max(abs(a-b) for a,b in zip(image.getpixel(point),background)) <= 3
                               for point in [(652 if wide else 686, 417), (1076, 417), (864 if wide else 881, 84)])
                    if not pilot_controls:return background_painted
                    # A surface color alone can hide an incompletely painted
                    # first frame. Require actual heading and all eight label
                    # interiors; outlines cannot satisfy this criterion.
                    regions=[(705,120,950,158)]+[(c["x"]+12,c["y"]+15,c["x"]+c["width"]-12,c["y"]+33) for c in pilot_controls]
                    return background_painted and all(sum(max(abs(a-b) for a,b in zip(pixel,background))>12 for pixel in image.crop(region).getdata())>=10 for region in regions)
            paint_start = time.monotonic()
            # Diagnose asynchronous native stacking/paint without relaxing a
            # single pixel criterion. Record the latency; it is a UX finding.
            while not sheet_painted() and not windows and time.monotonic()-paint_start < 3:
                time.sleep(.1)
                subprocess.run(["import", "-window", "root", str(path)], env=env, check=True)
            record.setdefault("drawer_paint_wait_seconds", {})[name] = time.monotonic()-paint_start
            assert sheet_painted(), "Sheet reports visible but is not painted above the chart"
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
            if args.chart_style == "XNav":
                chart = snapshot["runtime"]["display"]["chart_region"]
                scale = dict(snapshot["runtime"]["chart"]["scale_bar"])
                scale["x"] += chart["x"]; scale["y"] += chart["y"]
                follow = [c for c in snapshot["runtime"]["display"]["interaction_controls"]
                          if c["label"] == "Follow boat" and c["visible"]][0]
                assert scale["width"] > 0 and scale["height"] > 0, "Actual chart scale has no painted bounds"
                # Four pixels of backing keep real ENC soundings from reading
                # as scale text. The content retains the HTML's 25px gap.
                assert abs(scale["x"] + 4 - (follow["x"] + follow["width"]) - 25) <= 1, "Chart scale overlaps Follow boat or loses the prototype content gap"
                assert chart["x"] <= scale["x"] and chart["y"] <= scale["y"]
                assert scale["x"] + scale["width"] <= chart["x"] + chart["width"]
                assert scale["y"] + scale["height"] <= chart["y"] + chart["height"]
                record.setdefault("scale_layout", []).append(dict(file=name,actual=scale,follow=follow))
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
            if args.depth_unit:
                unit = snapshot["runtime"]["chart"]
                assert unit["depth_units_visible"], "Actual chart depth-unit indication was disabled"
                assert unit["depth_unit_type"] == ["feet", "meters", "fathoms"].index(args.depth_unit) + 1, "Upstream quilt unit differs from the explicitly configured ENC display unit"
                record["resolved_depth_unit"] = args.depth_unit
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
            window, pid = ui.wait_window("SKAGER / OpenCPN", app.pid)
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
            matches = xdo("search", "--all", "--onlyvisible", "--pid", app.pid, "--name", "^SKAGER / OpenCPN$").splitlines()
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
        # The real installed product has no synthetic input here. The horizon
        # must keep unavailable navigation actions disabled while its explicit
        # passage-inspection entry remains usable with no active route.
        snapshot = data()
        horizon = snapshot["runtime"]["display"]["horizon_region"]
        controls = snapshot["runtime"]["display"]["interaction_controls"]
        for label, enabled in (("Full passage", True), ("Horizon now", False),
                               ("Horizon event 1", False)):
            found = [c for c in controls if c["label"] == label and c["visible"]]
            assert len(found) == 1 and found[0]["enabled"] == enabled, (label, found)
            c = found[0]
            assert horizon["x"] <= c["x"] and horizon["y"] <= c["y"]
            assert c["x"] + c["width"] <= horizon["x"] + horizon["width"]
            assert c["y"] + c["height"] <= horizon["y"] + horizon["height"]
        before_follow = snapshot["runtime"]["chart"]["follow"]
        click("Full passage")
        capture("horizon-full-passage")
        click("Close")
        assert data()["runtime"]["chart"]["follow"] == before_follow
        record["horizon_unavailable_flow"] = "No invented navigation action; actual Full passage pointer opens existing Route drawer and Close restores chart without changing follow"
        for label, name in [("Passage", "passage"), ("Traffic", "traffic"), ("Energy", "energy"),
                            ("Instruments", "instruments"), ("Autopilot", "autopilot"), ("Alerts", "alerts"), ("Inspect source health", "health"), ("Anchor", "anchor"), ("Radar", "radar"), ("Settings", "settings")]:
            if args.view and name != args.view:
                continue
            if name == "health":
                controls=[c for c in data()["runtime"]["display"]["interaction_controls"]
                          if c["accessible_name"]=="Inspect source health" and c["visible"]]
                assert len(controls)==1, "Unique source-health entry required"
                click(controls[0]["label"])
            else:
                click(label)
            capture(name + "-day")
            if label in {"Autopilot", "Alerts", "Inspect source health"}:
                click("Day", "Dusk");capture(name+"-dusk")
                click("Dusk", "Night");capture(name+"-night")
                click("Night", "Day");click("Close")
                assert "drawer" not in data()["runtime"]["display"], "Prototype sheet close did not restore chart"
                record[name+"_flow"]="Owned real-state drawer; theme cycle; Close; no equipment command"
            if name == "health":
                controls=[c for c in data()["runtime"]["display"]["interaction_controls"]
                          if c["accessible_name"]=="Inspect source health" and c["visible"]]
                click(controls[0]["label"]);click("GPS",in_drawer=True)
                capture("health-gps-day")
                click("GPS",in_drawer=True);click("Close")
            if label == "Radar":
                from PIL import Image
                radar_interior=(260,280,620,600)
                with Image.open(args.output/'radar-day.png') as first:
                    scope=first.convert('RGB').crop(radar_interior)
                    assert len(scope.getcolors(1000000))>20, 'Actual radar scope failed to paint'
                    radar_pixels=scope.tobytes()
                display=data()["runtime"]["display"]
                regions={r['label']:r for r in display['product_regions']}
                # Diagnostics reports physical screen positions. HTML and the
                # saved image use the 1280x800 client origin; Windows' ordinary
                # caption/frame offsets are not part of prototype geometry.
                origin=(0,0)
                if windows:
                    point=ui.W.POINT(0,0)
                    to_screen=ui.declare(ui.user,"ClientToScreen",ui.W.BOOL,ui.W.HWND,ui.C.POINTER(ui.W.POINT))
                    assert to_screen(window,ui.C.byref(point))
                    origin=(point.x,point.y)
                for title,rect in [('Radar display',(112,212,657,508)),('Radar controls',(797,212,265,508))]:
                    actual=dict(regions[title]);actual['x']-=origin[0];actual['y']-=origin[1]
                    assert actual['visible'] and all(abs(actual[k]-v)<=1 for k,v in zip(('x','y','width','height'),rect)), (title,actual,rect)
                for title in ['Radar active','Guard zone']:
                    found=[c for c in display['interaction_controls'] if c['label']==title]
                    assert len(found)==1 and found[0]['visible'] and not found[0]['enabled'],(title,'unverified radar command exposed')
                assert not any(c['visible'] and c['label'] in ('Up','Down') for c in display['interaction_controls']), 'Radar must scroll only its control column'
                click("Day", "Dusk");capture("radar-dusk")
                click("Dusk", "Night");capture("radar-night")
                for state in ['dusk','night']:
                    with Image.open(args.output/f'radar-{state}.png') as current:
                        assert current.convert('RGB').crop(radar_interior).tobytes()==radar_pixels, f'Actual radar {state} scope is incomplete or changed its fixed prototype palette'
                for state,accent in [('day',(182,239,206)),('dusk',(155,197,177)),('night',(133,169,149))]:
                    with Image.open(args.output/f'radar-{state}.png') as current:
                        selected=current.convert('RGB').crop((9,498,70,559))
                        assert sum(pixel==accent for pixel in selected.getdata())>5, f'Radar navigation selection missing in {state}'
                click("Night", "Day");click("Close")
                assert data()["ui_page"]=="Navigation", "Radar close did not restore chart"
                record["radar_flow"]="Unavailable owned-status presentation; no invented echoes or scanner controls; three themes and Close"
            if label == "Anchor":
                click("Day", "Dusk");capture("anchor-dusk")
                click("Dusk", "Night");capture("anchor-night")
                click("Night", "Day");click("Close")
                assert "drawer" not in data()["runtime"]["display"], "Anchor close did not return to chart"
                chart=data()["runtime"]["display"]["chart_region"]
                assert chart["width"]==1014 and chart["height"]==566,"Anchor changed chart viewport"
                record["anchor_flow"]="Owned watch sheet; theme cycle; Close restores unchanged chart viewport; no navigation commands"
            if label == "Passage":
                click("Day", "Dusk")
                capture("passage-dusk")
                click("Dusk", "Night")
                capture("passage-night")
                click("Night", "Day")
                click("Close")
                assert "drawer" not in data()["runtime"]["display"], "Passage close did not return to chart"
                chart = data()["runtime"]["display"]["chart_region"]
                assert chart["width"] == 1014 and chart["height"] == 566, "Passage changed upstream chart viewport"
                record["passage_flow"] = "Current-route sheet; full theme cycle; Close retains original chart geometry"
            if label == "Instruments":
                display = data()["runtime"]["display"]
                assert not any(c["visible"] and c["label"] in ("Up", "Down") for c in display["interaction_controls"]), "Old scroll toolbar remains"
                regions = display["product_regions"]
                wind = next(r for r in regions if r["label"] == "Wind and heading")
                sog = next(r for r in regions if r["label"] == "SPEED OVER GROUND")
                assert abs(wind["width"] - 488) <= 1 and wind["height"] == 540
                assert abs(sog["x"] - wind["x"] - 506) <= 1 and sog["y"] == wind["y"]
                assert sog["height"] == 126 and sog["visible"], "Primary instrument tile clipped"
                click("Day", "Dusk")
                capture("instruments-dusk")
                click("Dusk", "Night")
                capture("instruments-night")
                click("Night", "Day")
                def wheel(direction):
                    before = data()["runtime"]["display"]["page_scroll_px"]
                    if windows:
                        origin = ui.W.POINT(0, 0)
                        client_to_screen = ui.declare(ui.user, "ClientToScreen", ui.W.BOOL, ui.W.HWND, ui.C.POINTER(ui.W.POINT))
                        assert client_to_screen(window, ui.C.byref(origin))
                        ui.SetCursorPos(origin.x+800, origin.y+330)
                        ui.MouseEvent(0x0800, 0, 0, (-120*direction) & 0xffffffff, 0)
                    else:
                        xdo("mousemove", 800, 330)
                        xdo("click", 5 if direction > 0 else 4)
                    deadline = time.monotonic()+5
                    while time.monotonic() < deadline:
                        value = data()["runtime"]["display"]["page_scroll_px"]
                        if (value-before)*direction > 0:
                            return value
                        time.sleep(.1)
                    raise AssertionError("Actual pointer wheel did not scroll Instruments")
                for _ in range(10):
                    if any(r["label"] == "WATER TEMPERATURE" and r["visible"] for r in data()["runtime"]["display"]["product_regions"]):
                        break
                    wheel(1)
                else:
                    raise AssertionError("Lower Instruments readings remain inaccessible")
                for _ in range(10):
                    if data()["runtime"]["display"]["page_scroll_px"] == 0:
                        break
                    wheel(-1)
                else:
                    raise AssertionError("Instruments cannot return to its heading")
                if windows: ui.SetCursorPos(0, 0)
                else: xdo("mousemove", 0, 0)
                record["instrument_wheel"] = "Real pointer wheel reaches lower readings and returns to top without permanent toolbar"
                click("Close")
                chart = data()["runtime"]["display"]["chart_region"]
                assert chart["width"] == 1014 and chart["height"] == 566
                record["instrument_flow"] = "Prototype wind/tile geometry; three themes; Close restores chart and horizon"
            if label == "Settings":
                click("Day", "Dusk")
                capture("settings-dusk")
                click("Dusk", "Night")
                capture("settings-night")
                click("Night", "Day")
                for tab, state_name in [("Sensors", "sensors"), ("Display", "display"), ("System", "system")]:
                    click(tab, in_drawer=True)
                    capture(state_name + "-day")
                click("Close")
                assert "drawer" not in data()["runtime"]["display"], "Settings close did not restore chart"
                chart = data()["runtime"]["display"]["chart_region"]
                assert chart["width"] == 1014 and chart["height"] == 566
                record["settings_flow"] = "Wide preferences sheet; eight sections; theme cycle; Sensors/Display/System; Close retains chart"
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
    except Exception as error:
        record["failure"] = repr(error)
        if app and app.poll() is None:
            try:
                (args.output/"failure-state.json").write_text(json.dumps(data(),indent=2)+"\n")
                capture("failure-state")
            except Exception as evidence_error:
                record["failureCaptureError"] = repr(evidence_error)
        raise
    finally:
        shutdown_error = None
        if app and app.poll() is None:
            try:
                if windows:
                    if not window:
                        window, _ = ui.wait_window("SKAGER / OpenCPN", app.pid, timeout=5)
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
