"""Read atomically published diagnostic JSON across Windows rename windows."""
import json
import time


def read_json_snapshot(path, timeout=2.0):
    deadline = time.monotonic() + timeout
    while True:
        try:
            # Parse errors are not an access race: never hide corrupt evidence.
            return json.loads(path.read_text(encoding="utf-8"))
        except (FileNotFoundError, PermissionError):
            if time.monotonic() >= deadline:
                raise
            time.sleep(.02)
