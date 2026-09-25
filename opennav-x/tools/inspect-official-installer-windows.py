"""Capture the official setup's initial UI after a failed silent CI prerequisite."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

assert sys.platform == "win32" and os.environ.get("GITHUB_ACTIONS") == "true"
setup = Path(sys.argv[1])
output = Path(sys.argv[2])
assert hashlib.sha256(setup.read_bytes()).hexdigest() == "e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa"
spec = importlib.util.spec_from_file_location("ui", Path(__file__).with_name("windows-ui.py"))
ui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ui)
record = {"purpose": "Initial official installer UI only; no installation acceptance", "windows": []}
record["desktop"] = ui.ensure_desktop()
process = subprocess.Popen([str(setup)])
try:
    deadline = time.monotonic() + 20
    seen = set()
    while time.monotonic() < deadline:
        for handle, pid, title in ui.windows(process.pid):
            controls = [ui.control_text(child) for child, _ in ui.children(handle)]
            signature = (title, tuple(controls))
            if signature in seen:
                continue
            seen.add(signature)
            item = {"title": title, "controls": controls, "pid": pid}
            try:
                name = "official-initial-" + str(len(record["windows"])) + ".png"
                ui.capture(handle, output / name, resize=False, screen_pixels=True)
                item["screenshot"] = name
            except Exception as error:
                item["capture_error"] = repr(error)
            record["windows"].append(item)
        if process.poll() is not None:
            record["exit_code"] = process.returncode
            break
        time.sleep(.5)
finally:
    if process.poll() is None:
        subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], capture_output=True)
        process.wait(timeout=15)
    (output / "official-initial-ui.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    print(json.dumps(record, indent=2))
