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


def _workspace_ready(snapshot, process_id):
    runtime = snapshot.get("runtime")
    assert isinstance(runtime, dict), "Missing workspace diagnostic readiness"
    readiness = runtime.get("test_workspace_readiness")
    assert isinstance(readiness, dict), "Missing workspace diagnostic readiness"
    observed_pid = readiness.get("process_id")
    ready = readiness.get("dashboard_ready")
    assert (type(observed_pid) is int and 0 < observed_pid <= 4294967295 and
            type(ready) is bool), "Invalid workspace diagnostic readiness"
    return observed_pid == process_id and ready


def read_json_snapshot(path, timeout=2.0, *, after=None, workspace_pid=None):
    """Wait for identity/phase only; never poll the asserted workspace contents."""
    if workspace_pid is not None:
        assert type(workspace_pid) is int and 0 < workspace_pid <= 4294967295, \
            "Invalid workspace process identity"
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
        fresh = baseline is None or _publication_epoch(snapshot) > baseline
        ready = True
        if workspace_pid is not None:
            _publication_epoch(snapshot)
            ready = _workspace_ready(snapshot, workspace_pid)
        if fresh and ready:
            return snapshot
        if time.monotonic() >= deadline:
            if workspace_pid is not None:
                raise AssertionError("Ready workspace diagnostic did not arrive for process " + str(workspace_pid))
            raise AssertionError("Newer diagnostic publication did not arrive")
        time.sleep(.02)
