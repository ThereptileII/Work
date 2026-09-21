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
        assert not pathlib.Path(str(marker) + '.started').exists(), 'Replacement started while old process was alive'
        deadline = time.monotonic() + 10
        while parent.poll() is None:
            if pathlib.Path(str(marker) + '.started').exists():
                # Recheck after observing the signal: the parent could have
                # exited between the first poll and reading the marker.
                assert parent.poll() is not None, 'Replacement overlapped its parent'
            if time.monotonic() > deadline:
                raise RuntimeError('Parent did not exit')
            time.sleep(.02)
        assert parent.returncode == 0, 'Parent failed'
        deadline = time.monotonic() + 10
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        expected = b'12:profile path\n11:with spaces\n12:quote"slash\\\n0:\n'
        actual = marker.read_bytes()
        assert actual == expected, f'Restart arguments were changed: {actual!r}'
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()
print('Restart waits for parent exit and preserves spaces, quotes and empty arguments')
