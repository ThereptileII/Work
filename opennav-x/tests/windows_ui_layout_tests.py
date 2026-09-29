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

# Instruments retains the actual prototype horizon; full workflow pages do
# not. Require each exact boundary, rejecting unexplained blank regions.
function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                and node.name == 'assert_page_geometry')
exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
page_check = namespace['assert_page_geometry']
page_passed = 0
for scale in (1., 1.25, 1.5):
    for horizon in (False, True):
        for delta in (0, -8, 8):
            labels = [(3, 'Chart'), (4, 'Alerts 1'), (5, 'System'),
                      (6, 'Configure instruments')]
            parents = {3: 10, 4: 11, 5: 12, 6: 13, 13: 14}
            logical_height = 800/scale
            top = round((56 if logical_height<=600 else 60 if logical_height<=740 else 68)*scale)
            footer = round(34*scale)
            timeline = round((98 if logical_height<=600 else 112 if logical_height<=740 else 132)*scale)
            left = round((80 if scale==1 else 70)*scale)
            right = 1280-round((186 if scale==1 else 156)*scale)
            bottom = 800-footer-(timeline if horizon else 0)
            bounds = {1:(0,0,1280,800),2:(left,top,right,bottom+delta),
                      10:(0,top,left,800-footer),11:(0,0,1280,top),
                      12:(0,800-footer,1280,800),14:(right,top,1280,800-footer)}
            namespace.update(GetDpiForWindow=lambda h:96*scale,
                             GetClientRect=lambda h,out:get_rect(1,out))
            try: page_check(1,2,horizon=horizon)
            except AssertionError:
                assert delta!=0, 'Valid Instruments or full-view geometry rejected'
            else:
                assert delta==0, 'Unexplained page gap/overlap accepted'
            page_passed+=1
print(f'{page_passed} exact page/timeline/DPI geometry checks passed')

# Exact independent canonical/client cases: captioned preview windows have a
# smaller client, not a different drawer style or a broad height tolerance.
node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='prototype_drawer_bounds')
exec(compile(ast.Module(body=[node],type_ignores=[]),'drawer contract','exec'),namespace)
bounds_for=namespace['prototype_drawer_bounds']
assert bounds_for(1280,800)==dict(x=682,y=80,width=398,height=674)
assert bounds_for(1264,761,origin=(8,31))==dict(x=674,y=111,width=398,height=635)
assert bounds_for(1280,800,1.25)==dict(x=569,y=90,width=498,height=652)
assert bounds_for(1280,800,1.5)==dict(x=428,y=102,width=597,height=629)
print('4 independent canonical/client/DPI drawer geometry checks passed')
