"""Read atomically published diagnostic JSON across Windows rename windows."""
import json
import time


def _publication_epoch(snapshot):
    assert isinstance(snapshot, dict), "Invalid diagnostic publication epoch"
    value = snapshot.get("publication_monotonic_ms")
    assert (snapshot.get("publication_clock") == "live monotonic clock" and
            isinstance(value, str) and 1 <= len(value) <= 19 and
            value.isascii() and value.isdecimal()), "Invalid diagnostic publication epoch"
    epoch = int(value)
    assert epoch <= 9223372036854775807, "Invalid diagnostic publication epoch"
    return epoch


def read_json_snapshot(path, timeout=2.0, *, after=None):
    """Optionally require the first newer publication, without polling its contents."""
    baseline = _publication_epoch(after) if after is not None else None
    deadline = time.monotonic() + timeout
    while True:
        try:
            # Parse errors are not an access race: never hide corrupt evidence.
            snapshot = json.loads(path.read_text(encoding="utf-8"))
        except (FileNotFoundError, PermissionError):
            if time.monotonic() >= deadline:
                raise
            time.sleep(.02)
            continue
        if baseline is None or _publication_epoch(snapshot) > baseline:
            return snapshot
        if time.monotonic() >= deadline:
            raise AssertionError("Newer diagnostic publication did not arrive")
        time.sleep(.02)
