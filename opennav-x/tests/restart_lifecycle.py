"""The replacement must not start until its parent exits; arguments stay exact."""
import pathlib
import subprocess
import sys
import tempfile
import time

exe = pathlib.Path(sys.argv[1]).resolve()
with tempfile.TemporaryDirectory(prefix='opennav restart ') as directory:
    marker = pathlib.Path(directory) / 'child result'
    parent = subprocess.Popen([str(exe), 'parent', str(marker), 'profile path'])
    try:
        deadline = time.monotonic() + 10
        while not marker.with_suffix('.armed').exists():
            # C++ appends the suffix, preserving spaces in the marker filename.
            if pathlib.Path(str(marker) + '.armed').exists():
                break
            if parent.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Restart helper did not arm')
            time.sleep(0.02)
        time.sleep(0.5)
        assert parent.poll() is None, 'Parent did not stay alive for the test'
        assert not marker.exists(), 'Replacement started while old process was alive'
        assert parent.wait(timeout=10) == 0, 'Parent failed'
        deadline = time.monotonic() + 10
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        expected = b'12:profile path\n11:with spaces\n12:quote"slash\\\n0:\n'
        assert marker.read_bytes() == expected, 'Restart arguments were changed'
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()
print('Restart waits for parent exit and preserves spaces, quotes and empty arguments')
