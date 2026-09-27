"""Recognize a fresh upstream startup across append or startup log rotation."""

START = b"------- OpenCPN version "
READY = b"OnInitTimer...Finalize Canvases"


def initialized_since(before: bytes, current: bytes) -> bool:
    """Require the latest new startup and its finalization, never old markers.

    OpenCPN rotates/truncates its log at startup. A lifetime marker count in the
    current file therefore cannot prove readiness after several mode restarts.
    The caller captures ``before`` immediately before its own launch/action.
    """
    fresh = current[len(before):] if current.startswith(before) else current
    started = fresh.rfind(START)
    return started >= 0 and READY in fresh[started:]
