#!/usr/bin/env python3
"""Isolated logical-viewport regression; never substitutes for Windows DPI."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from diagnostic_snapshot import read_json_snapshot


def main():
    if sys.platform != "linux":
        raise SystemExit("Linux development evidence only; native Windows uses the DPI probe")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--references", type=Path, default=ROOT / "evidence/local/prototype")
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    temporary = tempfile.TemporaryDirectory(prefix="xnav layout ")
    profile = Path(temporary.name)/"profile"
    subprocess.run([sys.executable, str(ROOT/"tools/prepare-test-profile.py"), "--build",
                    str(ROOT/"build/production-linux"), "--profile", str(profile)], check=True)
    app_path = ROOT/"build/production-install/bin/opencpn"
    number = 196
    while Path(f"/tmp/.X{number}-lock").exists(): number += 1
    env = dict(os.environ, DISPLAY=f":{number}", GDK_BACKEND="x11"); env.pop("WAYLAND_DISPLAY", None)
    server = subprocess.Popen(["Xvfb", env["DISPLAY"], "-screen", "0", "1280x800x24", "-nolisten", "tcp"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(.3)
    assert server.poll() is None, "Private display failed"
    app = None
    report = dict(scope="Linux logical layout only; not Windows DPI or physical display acceptance",
                  executableSha256=hashlib.sha256(app_path.read_bytes()).hexdigest(), cases=[])

    def xd(*a):
        return subprocess.check_output(["xdotool", *map(str, a)], env=env, text=True).strip()

    def data(predicate=lambda d: True):
        deadline = time.monotonic()+12
        while time.monotonic()<deadline:
            assert app.poll() is None, "Application exited"
            try:
                d = read_json_snapshot(profile/"opennav-diagnostics.json")
                if predicate(d): return d
            except (OSError, ValueError): pass
            time.sleep(.15)
        raise RuntimeError("Current layout publication timed out")

    def capture(name):
        path = output/(name+".png")
        subprocess.run(["import", "-window", "root", str(path)], env=env, check=True)
        (output/(name+".json")).write_text(json.dumps(data(), indent=2))
        return dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest())

    try:
        with (output/"launch.log").open("w") as log:
            app = subprocess.Popen([str(app_path), "--configdir", str(profile), "--xnav", "--no_opengl"],
                                   env=env, stdout=log, stderr=log)
        deadline = time.monotonic()+70
        while time.monotonic()<deadline:
            assert app.poll() is None
            log = profile/"opencpn.log"
            if log.exists() and "OnInitTimer...Finalize Canvases" in log.read_text(errors="replace"): break
            time.sleep(.2)
        else: raise RuntimeError("Initialization did not complete")
        windows = xd("search", "--all", "--onlyvisible", "--pid", app.pid, "--name", "^OpenNav X / OpenCPN$").splitlines()
        assert len(windows)==1
        window = windows[0]
        for index, (width, height, reference) in enumerate([
                (1280,800,ROOT/"docs/design/prototype/reference/linux/capture.json"),
                (1024,640,args.references/"responsive-125/capture.json"),
                (853,533,args.references/"responsive-150/capture.json"),
                (1280,800,ROOT/"docs/design/prototype/reference/linux/capture.json")]):
            before = int(data()["runtime"]["ui_update"]["ticks"])
            xd("windowsize",window,width,height); xd("windowmove",window,0,0)
            d = data(lambda d:int(d["runtime"]["ui_update"]["ticks"])>before+3)
            stem = f"{index}-{width}x{height}"
            entry = dict(width=width,height=height,screenshots=[capture(stem+"-chart")]);report["cases"].append(entry)
            ref = json.loads(reference.read_text())["states"]["navigation-day"]["components"]
            display = d["runtime"]["display"]
            chart = display["chart_region"]
            assert all(abs(chart[k]-v)<=1 for k,v in ref["#chartView"][0]["rect"].items()), ("Chart differs from HTML",chart,ref["#chartView"])
            labels = ("Chart","Passage","Traffic","Energy","Instruments","Anchor","Radar","Settings")
            controls=[]
            for i,label in enumerate(labels):
                found=[c for c in display["interaction_controls"] if c["label"]==label and c["x"]<80]
                assert len(found)==1
                c=found[0];assert c["visible"] and 0<=c["y"]<c["y"]+c["height"]<=height, ("Clipped action",c)
                expected=ref[".nav-btn"][i]["rect"]
                assert all(abs(c[k]-v)<=1 for k,v in expected.items()), (label,c,expected)
                controls.append(c)
            fields=display["rail_regions"]
            assert len(fields)==4 and all(c["visible"] and c["y"]+c["height"]<=height for c in fields)
            entry.update(chart=chart,navigation=controls,readings=fields)
            c=controls[-1];xd("mousemove",c["x"]+c["width"]//2,c["y"]+c["height"]//2,"click",1)
            d=data(lambda d:d["ui_page"]=="Settings" and "drawer" in d["runtime"]["display"])
            entry["screenshots"].append(capture(stem+"-settings"))
            drawer=d["runtime"]["display"]["drawer"]
            assert 0<=drawer["x"]<drawer["x"]+drawer["width"]<=width and 0<=drawer["y"]<drawer["y"]+drawer["height"]<=height
            # The compact sheet intentionally scrolls; verify the lower real
            # action is reachable, rather than accepting clipped form content.
            before = int(d["runtime"]["ui_update"]["ticks"])
            xd("mousemove", drawer["x"]+drawer["width"]//2,
               drawer["y"]+drawer["height"]-35, "click", "--repeat", 12, "--delay", 50, 5)
            d=data(lambda d:int(d["runtime"]["ui_update"]["ticks"])>before+2)
            links=[c for c in d["runtime"]["display"]["interaction_controls"]
                   if c["label"]=="Chart safety depth" and c["visible"]]
            assert len(links)==1 and links[0]["y"]+links[0]["height"]<=drawer["y"]+drawer["height"], "Lower settings action is unreachable"
            entry["screenshots"].append(capture(stem+"-settings-scrolled"))
            close=[c for c in d["runtime"]["display"]["interaction_controls"] if c["label"]=="Close" and c["visible"]]
            assert len(close)==1;c=close[0];xd("mousemove",c["x"]+c["width"]//2,c["y"]+c["height"]//2,"click",1)
            data(lambda d:d["ui_page"]=="Navigation" and "drawer" not in d["runtime"]["display"])
            d=data()
            profiles=[c for c in d["runtime"]["display"]["interaction_controls"] if c["label"]=="Vessel profile" and c["visible"]]
            assert len(profiles)==(1 if height>600 else 0), "Profile visibility differs from prototype media rule"
            if profiles:
                c=profiles[0]
                assert c["width"]==32 and c["height"]==32 and c["y"]+32<=height
                xd("mousemove",c["x"]+16,c["y"]+16,"click",1)
                d=data(lambda d:d["ui_page"]=="Settings" and "drawer" in d["runtime"]["display"])
                assert any(c["label"]=="Vessel dimensions" for c in d["runtime"]["display"]["interaction_controls"]), "Profile opened wrong section"
                entry["profileFlow"]="Actual pointer opens vessel settings; no mutation or connection"
                c=next(c for c in d["runtime"]["display"]["interaction_controls"] if c["label"]=="Close" and c["visible"])
                xd("mousemove",c["x"]+c["width"]//2,c["y"]+c["height"]//2,"click",1)
                data(lambda d:d["ui_page"]=="Navigation" and "drawer" not in d["runtime"]["display"])
        report["result"]="passed; native Windows and boat review required"
    except Exception as error:
        report["result"]="failed";report["error"]=repr(error);raise
    finally:
        try:
            if app and app.poll() is None:
                subprocess.run([str(app_path),"--configdir",str(profile),"--remote","--quit"],env=env,capture_output=True,timeout=15,check=True)
                app.wait(timeout=25)
        except (subprocess.SubprocessError, OSError) as error:
            report["shutdownError"]=repr(error);report["result"]="failed"
            if app and app.poll() is None:
                app.kill();app.wait(timeout=10)  # Only this isolated child.
            raise
        finally:
            report["exitCode"]=app.returncode if app else None
            server.terminate();server.wait(timeout=10);temporary.cleanup()
            (output/"result.json").write_text(json.dumps(report,indent=2)+"\n")
        if app:assert app.returncode==0,"Application did not close normally"


if __name__=="__main__": main()
