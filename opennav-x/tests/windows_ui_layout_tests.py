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
    # Captions follow the exact6160 Windows failure and current Shell format.
    global labels, parents, bounds
    labels = [(3, 'Navigation'), (4, 'Demo'), (5, label), (6, 'System')]
    parents = {3: 2, 4: 2, 5: 2, 6: 2}
    bounds = {1: (0, 0, 1280, 800), 2: (8, 735, 1272, 792),
              3: (12, 739, 124, 787), 4: (512, 739, 600, 787),
              5: (608, 752, 1148, 776), 6: (1156, 739, 1268, 787)}


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
parents[7] = 99  # Unrelated page text is not the bottom summary.
verify()

for fault in ('missing', 'duplicate', 'wrong-pane', 'missing-demo', 'duplicate-demo',
              'missing-system', 'left-overlap', 'right-overlap', 'zero-width',
              'vertical-overlap', 'pane-clipped', 'frame-clipped', 'failed-query'):
    fixture()
    if fault == 'missing': labels = [entry for entry in labels if entry[0] != 5]
    if fault == 'duplicate':
        labels.append((7, 'No active route'))
        parents[7] = 2
    if fault == 'wrong-pane': parents[5] = 99
    if fault == 'missing-demo': labels = [entry for entry in labels if entry[0] != 4]
    if fault == 'duplicate-demo': labels.append((7, 'Demo'))
    if fault == 'missing-system': labels = [entry for entry in labels if entry[0] != 6]
    if fault == 'left-overlap': bounds[5] = (599, 752, 1148, 776)
    if fault == 'right-overlap': bounds[5] = (608, 752, 1157, 776)
    if fault == 'zero-width': bounds[5] = (608, 752, 608, 776)
    if fault == 'vertical-overlap': bounds[5] = (608, 738, 1148, 776)
    if fault == 'pane-clipped': bounds[2] = (8, 735, 1000, 792)
    if fault == 'frame-clipped': bounds[2] = (8, 735, 1272, 805)
    if fault == 'failed-query': del bounds[5]
    verify(reject=True)

print(f'{passed} native summary caption/geometry contracts passed')
