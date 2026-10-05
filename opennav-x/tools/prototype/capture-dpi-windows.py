#!/usr/bin/env python3
"""Read-only prototype UI at real Windows DPI; disposable native CI only.

Additional development gate, not a replacement for release lifecycle/alarms/
physical touch tests. No sensor fixtures, connections or actuator commands.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from diagnostic_snapshot import read_json_snapshot


def main():
    if sys.platform != "win32" or os.environ.get("GITHUB_ACTIONS") != "true":
        raise SystemExit("Disposable native Windows CI desktop required")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location("windows_ui", ROOT / "tools/windows-ui.py")
    ui = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ui)
    from PIL import Image
    app_path = ROOT / "build/production-install/opencpn.exe"
    helper = ROOT / "build/production-windows/Release/opennav-test-dpi.exe"
    env = dict(os.environ, OPENNAV_DISPOSABLE_DESKTOP="1")
    report = dict(authority="Actual GetDpiForWindow and screen pixels; no rescaling",
                  desktop=ui.ensure_desktop(1920, 1080), scales=[],
                  executableSha256=hashlib.sha256(app_path.read_bytes()).hexdigest(),
                  sourceCommit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                  scope="Additional prototype development gate; no release or boat acceptance")

    def dpi(*args):
        r = subprocess.run([str(helper), *map(str, args)], env=env,
                           capture_output=True, text=True, timeout=15)
        if r.returncode:
            raise RuntimeError(r.stderr.strip())
        return json.loads(r.stdout)

    original = dpi()["percent"]
    assert original in (100, 125, 150), "Original DPI cannot be restored by this helper"
    app = None
    handle = None
    entry = None
    profile = None

    def data(predicate=lambda d: True, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert app.poll() is None, "Application exited during display review"
            try:
                value = read_json_snapshot(profile / "opennav-diagnostics.json")
                if predicate(value):
                    return value
            except (OSError, ValueError):
                pass
            time.sleep(.15)
        raise RuntimeError("No current diagnostic publication for display interaction")

    def origin():
        point = ui.W.POINT(0, 0)
        convert = ui.declare(ui.user, "ClientToScreen", ui.W.BOOL, ui.W.HWND, ui.C.POINTER(ui.W.POINT))
        assert convert(handle, ui.C.byref(point))
        return point

    def contains(rect, item):
        return (rect[0] <= item["x"] < item["x"]+item["width"] <= rect[2] and
                rect[1] <= item["y"] < item["y"]+item["height"] <= rect[3])

    def snapshot(name):
        # Canonical state excludes accidental hover left by a previous run.
        # Dedicated release hover tests still verify Day and dim-mode tooltips.
        ui.SetCursorPos(0,0)
        time.sleep(.15)
        path = output / (name + ".png")
        ui.capture(handle, path, resize=False, screen_pixels=True)
        entry["screenshots"].append(dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        d = data()
        (output / (name + ".json")).write_text(json.dumps(d, indent=2))
        return path, d

    def rail(d):
        p = origin()
        labels = ("Chart", "Passage", "Traffic", "Energy", "Instruments", "Anchor", "Radar", "Settings")
        items = []
        for label in labels:
            matches = [c for c in d["runtime"]["display"]["interaction_controls"]
                       if c["visible"] and c["label"] == label and c["x"] < p.x+100*scale/100]
            assert len(matches) == 1, (label, "unique sidebar action", matches)
            c = matches[0]
            assert contains((p.x, p.y, p.x+1280, p.y+800), c), (label, "clipped sidebar action", c)
            expected_height = {100:61,125:51,150:43}[scale]*scale/100
            assert abs(c["height"]-expected_height)<=1, (label, "height differs from measured prototype", c)
            items.append(c)
        fields = d["runtime"]["display"]["rail_regions"]
        assert len(fields) == 4 and all(f["visible"] for f in fields), "Four primary readings must remain visible"
        assert all(contains((p.x, p.y, p.x+1280, p.y+800), f) for f in fields), "Primary reading outside window"
        return dict(actions=items, readings=fields)

    def click(label, drawer=False, touch=False):
        d = data()
        display = d["runtime"]["display"]
        candidates = [c for c in display["interaction_controls"]
                      if c["label"] == label and c["visible"] and c["enabled"]]
        p = origin()
        if drawer:
            r = display["drawer"]
            candidates = [c for c in candidates if contains((r["x"], r["y"], r["x"]+r["width"], r["y"]+r["height"]), c)]
        elif label in ("Chart", "Settings", "Energy", "Instruments"):
            candidates = [c for c in candidates if c["x"] < p.x+100*scale/100]
        assert len(candidates) == 1, (label, "visible actionable control", candidates)
        c = candidates[0]
        assert contains((p.x, p.y, p.x+1280, p.y+800), c), (label, "clipped input target")
        ui.SetForegroundWindow(handle)
        point = ui.W.POINT(c["x"]+c["width"]//2, c["y"]+c["height"]//2)
        hit = ui.WindowFromPoint(point)
        owner = ui.W.DWORD()
        ui.GetWindowThreadProcessId(hit, ui.C.byref(owner))
        assert owner.value == app.pid and ui.IsWindowEnabled(hit), "Input would leave enabled application"
        assert ui.text(hit) == label, (label, "Another surface obscures the control", ui.text(hit))
        before = int(d["runtime"]["ui_update"]["ticks"])
        if touch:
            assert dpi("--tap", point.x, point.y)["touch_injected"]
        else:
            assert ui.SetCursorPos(point.x, point.y)
            ui.MouseEvent(2, 0, 0, 0, 0)
            ui.MouseEvent(4, 0, 0, 0, 0)
        data(lambda d: int(d["runtime"]["ui_update"]["ticks"]) > before)

    def vessel_form(record):
        controls=record["runtime"]["display"]["interaction_controls"]
        fields=["Field: Vessel name","Field: Draft · metres","Field: Safety depth · metres",
                "Field: Usable battery capacity · kWh","Field: Minimum reserve · %"]
        for label in fields:
            assert sum(c["label"]==label for c in controls)==1, ("Missing or duplicate vessel field",label)
        save=[c for c in controls if c["label"]=="Save vessel profile"]
        assert len(save)==1 and save[0]["enabled"], "Vessel profile Save identity must remain available"
        return dict(fields=fields,save=dict(label=save[0]["label"],enabled=save[0]["enabled"]))

    try:
        for scale in (100, 125, 150):
            assert dpi(scale)["percent"] == scale
            profile = output / f"profile-{scale}"
            subprocess.run([sys.executable, str(ROOT / "tools/prepare-test-profile.py"),
                            "--build", str(ROOT / "build/production-windows"), "--profile", str(profile)], check=True)
            with (profile / "opencpn.conf").open("a") as stream:
                stream.write("\n[Settings/GlobalState]\nVPLatLon=59.0800,18.5000\nVPScale=0.003\n"
                             "\n[OpenNav]\nChartPresentationV1=XNav\n")
            entry = dict(percent=scale, screenshots=[])
            report["scales"].append(entry)
            with (output / f"launch-{scale}.log").open("w") as log:
                app = subprocess.Popen([str(app_path), "--configdir", str(profile), "--xnav", "--no_opengl"],
                                       env=env, stdout=log, stderr=log)
            handle, _ = ui.wait_window("SKAGER / OpenCPN", app.pid)
            deadline = time.monotonic()+60
            while time.monotonic() < deadline:
                log = profile / "opencpn.log"
                if log.exists() and "OnInitTimer...Finalize Canvases" in log.read_text(errors="replace"):
                    break
                assert app.poll() is None
                time.sleep(.2)
            else:
                raise RuntimeError("Chart initialization timeout")
            assert ui.GetDpiForWindow(handle) == 96*scale//100, "Requested DPI differs from actual window DPI"
            outer, client = ui.W.RECT(), ui.W.RECT()
            assert ui.GetWindowRect(handle, ui.C.byref(outer)) and ui.GetClientRect(handle, ui.C.byref(client))
            before = int(data()["runtime"]["ui_update"]["ticks"])
            assert ui.SetWindowPos(handle, None, 0, 0, 1280+outer.right-outer.left-client.right,
                                   800+outer.bottom-outer.top-client.bottom, 0)
            time.sleep(.5)
            assert ui.GetClientRect(handle, ui.C.byref(client)) and (client.right, client.bottom) == (1280, 800)
            d = data(lambda d: int(d["runtime"]["ui_update"]["ticks"]) > before+2)
            assert d["data_mode"] == "OPENCPN selected navigation", "Fixture-free live mode required"
            path, d = snapshot(f"{scale}-navigation-day")
            entry["rail"] = rail(d)
            chart = d["runtime"]["display"]["chart_region"]
            assert chart["width"] > 400 and chart["height"] > 200, "Chart viewport lost"
            outer = ui.W.RECT(); assert ui.GetWindowRect(handle, ui.C.byref(outer))
            with Image.open(path) as img:
                colors = Counter(img.convert("RGB").crop((chart["x"]-outer.left, chart["y"]-outer.top,
                    chart["x"]-outer.left+chart["width"], chart["y"]-outer.top+chart["height"])).getdata())
            assert colors[(238, 238, 226)] > 1000 and colors[(213, 229, 229)] > 1000, "Chart land/water missing"
            click("Settings", touch=True)
            data(lambda d: d["ui_page"] == "Settings")
            _,d=snapshot(f"{scale}-settings-day")
            measured=json.loads((ROOT/"docs/design/prototype/reference/windows/settings-tabs.json").read_text())
            expected=(measured["states"]["settings-day"] if scale==100 else measured["responsive"][str(scale)])["drawer"]
            p=origin();actual=dict(d["runtime"]["display"]["drawer"])
            actual["x"]-=p.x;actual["y"]-=p.y
            assert all(abs(actual[k]-expected[k]*scale/100)<=1 for k in ("x","y","width","height")), ("Preferences differs from independent prototype",actual,expected)
            entry["prototypeDrawer"]=dict(actual=actual,expectedCss=expected)
            click("Sensors", drawer=True)
            snapshot(f"{scale}-sensors-day")
            click("Display", drawer=True)
            click("Night", drawer=True, touch=True)
            data(lambda d: d["runtime"]["display"]["light"] == "Night")
            snapshot(f"{scale}-display-night")
            click("Close", drawer=True, touch=True)
            d = data(lambda d: d["ui_page"] == "Navigation" and "drawer" not in d["runtime"]["display"])
            entry["railAfterClose"] = rail(d)
            assert entry["rail"] == entry["railAfterClose"], "Drawer changed permanent rail geometry"
            profiles=[c for c in d["runtime"]["display"]["interaction_controls"] if c["label"]=="Vessel profile" and c["visible"]]
            assert len(profiles)==(0 if scale==150 else 1), "Profile visibility differs from prototype"
            if profiles:
                click("Vessel profile",touch=True)
                d=data(lambda d:d["ui_page"]=="Settings")
                entry["vesselForm"]=vessel_form(d)
                click("Close",drawer=True,touch=True)
                data(lambda d:d["ui_page"]=="Navigation")
            for label in ("Energy", "Instruments"):
                click(label)
                snapshot(f"{scale}-{label.lower()}-night")
                click("Close")
                data(lambda d: d["ui_page"] == "Navigation")
            ui.close(handle)
            assert app.wait(timeout=25) == 0
            app = None
            handle = None
            entry["result"] = "passed; visual review and physical touch remain required"
        report["result"] = "passed; development DPI/interaction checks only"
    except Exception as error:
        report["result"] = "failed"
        report["error"] = repr(error)
        if handle and app and app.poll() is None:
            try:
                snapshot(f"{scale}-failure")
            except Exception as capture_error:
                report["captureError"] = repr(capture_error)
        raise
    finally:
        if app and app.poll() is None:
            if handle:
                ui.close(handle)
            try:
                app.wait(timeout=20)
            except subprocess.TimeoutExpired:
                app.kill()  # Owned disposable CI process only; never the boat.
                app.wait(timeout=10)
                report["cleanup"] = "failed normal close; terminated owned disposable process"
        try:
            report["restoredDpi"] = dpi(original)
        finally:
            (output / "result.json").write_text(json.dumps(report, indent=2)+"\n")


if __name__ == "__main__":
    main()
