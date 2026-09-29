"""Execute the native summary assertion with deterministic Win32 observations.

This verifies test-tool selection/geometry, not native rendering acceptance.
"""
import ast
import ctypes as C
from ctypes import wintypes as W
from pathlib import Path
import re


source = Path(__file__).resolve().parents[1] / 'tools/windows-ui.py'
tree = ast.parse(source.read_text())
function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                and node.name == 'assert_route_summary_layout')
namespace = dict(C=C, W=W, re=re)
exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
check = namespace['assert_route_summary_layout']
passed = 0


def fixture(label='17.7 NM  /  Sheltered bay'):
    # The prototype moves the same destination summary into the status row.
    global labels, parents, bounds
    labels = [(3, ''), (4, 'Alerts 2'), (5, label), (6, 'No vessel input')]
    parents = {3: 2, 4: 2, 5: 2, 6: 2}
    bounds = {1: (0, 0, 1280, 800), 2: (0, 0, 1280, 68),
              3: (0, 0, 180, 68), 4: (1232, 12, 1276, 56),
              5: (196, 24, 926, 44), 6: (942, 24, 1062, 44)}


def get_rect(handle, output):
    if handle not in bounds:
        return False
    value = C.cast(output, C.POINTER(W.RECT)).contents
    value.left, value.top, value.right, value.bottom = bounds[handle]
    return True


namespace.update(children=lambda _: labels, GetParent=lambda h: parents.get(h),
                 GetWindowRect=get_rect)


def verify(reject=False):
    global passed
    try:
        check(1)
    except AssertionError:
        if not reject:
            raise
    else:
        assert not reject, 'Invalid native summary observation was accepted'
    passed += 1


for caption in ('17.7 NM  /  Sheltered bay', '0.0 NM  /  Destination',
                '1234.5 NM  /  Arkösund', 'Route unavailable', 'No active route'):
    fixture(caption)
    verify()

for caption in ('17.7 NM to destination', '17.7 NM  /  ',
                'NaN NM  /  Sheltered bay', '-1.0 NM  /  Sheltered bay'):
    fixture(caption)
    verify(reject=True)

fixture()
labels.append((7, 'Route unavailable'))
parents[7] = 99  # Unrelated page text is not the status summary.
verify()

for fault in ('missing', 'duplicate', 'wrong-pane', 'missing-alerts', 'duplicate-alerts',
              'invalid-alert-caption', 'left-overlap', 'right-overlap', 'zero-width',
              'vertical-overlap', 'pane-clipped', 'frame-clipped', 'failed-query'):
    fixture()
    if fault == 'missing': labels = [entry for entry in labels if entry[0] != 5]
    if fault == 'duplicate':
        labels.append((7, 'No active route'))
        parents[7] = 2
    if fault == 'wrong-pane': parents[5] = 99
    if fault == 'missing-alerts': labels = [entry for entry in labels if entry[0] != 4]
    if fault == 'duplicate-alerts': labels.append((7, 'Alerts'))
    if fault == 'invalid-alert-caption': labels[1] = (4,'Alerts NaN')
    if fault == 'left-overlap': bounds[5] = (179, 24, 926, 44)
    if fault == 'right-overlap': bounds[5] = (196, 24, 943, 44)
    if fault == 'zero-width': bounds[5] = (196, 24, 196, 44)
    if fault == 'vertical-overlap': bounds[5] = (196, -1, 926, 44)
    if fault == 'pane-clipped': bounds[2] = (0, 0, 900, 68)
    if fault == 'frame-clipped': bounds[2] = (-1, 0, 1280, 68)
    if fault == 'failed-query': del bounds[5]
    verify(reject=True)

print(f'{passed} native summary caption/geometry contracts passed')
