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

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from diagnostic_snapshot import read_json_snapshot


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
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    profile = args.output.resolve() / "profile"
    subprocess.run([sys.executable, str(ROOT / "tools/prepare-test-profile.py"),
                    "--build", str(args.build), "--profile", str(profile)], check=True)
    with (profile / "opencpn.conf").open("a") as stream:
        stream.write("\n[Settings]\nOpenGL=0\n[Settings/GlobalState]\n"
                     "VPLatLon=59.0800,18.5000\nVPScale=0.001\n")
    number = 171
    while Path(f"/tmp/.X{number}-lock").exists():
        number += 1
    env = dict(os.environ, DISPLAY=f":{number}")
    record = {"authority": "Native Windows development" if windows else "Linux development only", "size": [1280, 800],
              "renderer": "software", "input": "none; isolated disposable profile",
              "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "executable_sha256": hashlib.sha256(args.app.read_bytes()).hexdigest(), "captures": []}
    xserver = None if windows else subprocess.Popen(["Xvfb", env["DISPLAY"], "-screen", "0", "1280x800x24", "-nolisten", "tcp"],
                               env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if windows:
        record["desktop"] = ui.ensure_desktop(1440, 900)
    app = None
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
        (args.output / f"{name}.json").write_text(json.dumps(snapshot, indent=2) + "\n")
        record["captures"].append({"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})

    try:
        time.sleep(.6)
        app = subprocess.Popen([str(args.app), "--configdir", str(profile), "--no_opengl", "--xnav"],
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
        for label, name in [("Passage", "passage"), ("Traffic", "traffic"), ("Energy", "energy"),
                            ("Instruments", "instruments"), ("Settings", "settings")]:
            click(label)
            capture(name + "-day")
        record["result"] = "captured; conformance not asserted"
    finally:
        if app and app.poll() is None:
            subprocess.run([str(args.app), "--configdir", str(profile), "--remote", "--quit"],
                           env=env, stdout=log, stderr=log, timeout=15, check=True)
            try:
                app.wait(timeout=30)
            except subprocess.TimeoutExpired:
                # Only this tool's disconnected disposable child is eligible.
                app.kill()
                app.wait()
                raise RuntimeError("Disposable native review application did not exit cleanly")
        log.close()
        if xserver:
            xserver.terminate()
            xserver.wait(timeout=10)
        (args.output / "capture.json").write_text(json.dumps(record, indent=2) + "\n")


if __name__ == "__main__":
    main()
