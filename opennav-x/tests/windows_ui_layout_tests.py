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

def is_child(parent, window):
    # Win32 IsChild includes all descendant levels, not the parent itself.
    visited = set()
    while window in parents and window not in visited:
        visited.add(window)
        window = parents[window]
        if window == parent:
            return True
    return False

namespace['IsChild'] = is_child


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
            labels = [(3, 'Chart'), (4, 'Alerts 1'), (12, 'OpenNav status footer'),
                      (6, 'Configure instruments')]
            parents = {3: 10, 4: 11, 12: 1, 6: 13, 13: 14}
            # The real Display page contains its own Configure instruments
            # action. Also exercise repeated captions at different nesting
            # depths: none defines the surrounding application geometry.
            labels += [(20, 'Configure instruments'), (21, 'Chart'),
                       (22, 'OpenNav status footer'), (23, 'Alerts 1')]
            parents.update({20: 30, 30: 31, 31: 2, 21: 2, 22: 30, 23: 31})
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

# A second matching control outside the page still fails. The descendant
# filter must never hide a duplicated real shell control.
bounds[2]=(left,top,right,bottom)
page_check(1,2,horizon=True)
labels.append((24, 'Configure instruments'))
parents[24] = 13
try:
    page_check(1, 2, horizon=True)
except AssertionError:
    pass
else:
    raise AssertionError('Duplicated shell rail action was accepted')
print('Nested page actions excluded; duplicated shell control rejected')

labels = [entry for entry in labels if entry[0] != 24]
for fault in ('missing-footer', 'duplicate-footer', 'old-system-button'):
    original = list(labels)
    if fault == 'missing-footer': labels = [entry for entry in labels if entry[0] != 12]
    if fault == 'duplicate-footer': labels.append((25, 'OpenNav status footer'))
    if fault == 'old-system-button': labels = [(h, 'System' if h == 12 else caption) for h, caption in labels]
    try: page_check(1, 2, horizon=True)
    except AssertionError: pass
    else: raise AssertionError('Invalid footer boundary accepted: ' + fault)
    labels = original
print('Missing/duplicated status footer and obsolete System boundary rejected')

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

# Exercise real-pointer harness protections with fake native observations.
# A direct HWND message would let all these clipped/covered actions pass.
node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'pointer_text')
pointer_namespace = dict(C=C, W=W)
exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), pointer_namespace)
class Clock:
    def __init__(self): self.now = 0
    def monotonic(self): self.now += .1; return self.now
    def sleep(self, seconds): self.now += seconds

for fault in (None, 'clipped', 'covered', 'no-hit', 'disabled', 'duplicate', 'missing', 'static',
              'preferences-scroll', 'preferences-stuck', 'preferences-covered',
              'preferences-null'):
    clock = Clock(); wire = []; front = [1]
    bounds = {1:(0,0,1280,800), 2:(680,180,1060,500), 3:(700,220,1040,292),
              4:(700,220,1040,292)}
    preferences = bool(fault and fault.startswith('preferences-'))
    if fault == 'clipped' or preferences: bounds[3] = (700,170,1040,242)
    parents = {3:2, 4:2, 2:1}
    captions = [] if fault == 'missing' else [(3,'Interface & recovery')]
    if fault == 'duplicate': captions.append((4,'Interface & recovery'))
    def native_class(handle, buffer, capacity):
        buffer.value = 'Static' if fault == 'static' else 'wxWindowNR'
        return len(buffer.value)
    def native_hit(point):
        # Duplicate test returns the queried target for two visible duplicate
        # controls; neither is an acceptable uniquely identified action.
        if fault == 'no-hit': return None
        if fault in ('covered','preferences-covered'): return 99
        if fault == 'preferences-null': return None
        if preferences and point.y == 340: return 2
        return queried[0]
    queried = [3]
    def native_rect(handle, output):
        if handle in (3,4): queried[0] = handle
        return get_rect(handle, output)
    def native_mouse(event, *args):
        wire.append(('mouse', event))
        if event == 0x0800 and fault == 'preferences-scroll': bounds[3] = (700,220,1040,292)
    pointer_namespace.update(
        time=clock, windows=lambda pid:[(1,pid,'OpenNav X / OpenCPN')],
        children=lambda root:captions, IsWindowEnabled=lambda h:fault!='disabled',
        text=lambda h:'OpenNav preferences' if preferences else 'OpenNav X / OpenCPN',
        GetClassNameW=native_class, GetWindowRect=native_rect,
        GetParent=lambda h:parents.get(h), IsChild=is_child, WindowFromPoint=native_hit,
        declare=lambda dll,name,*args: (lambda:front[0]) if name=='GetForegroundWindow' else (lambda h,flag:1),
        user=None, SetForegroundWindow=lambda h:front.__setitem__(0,h),
        SetCursorPos=lambda x,y:wire.append(('cursor',x,y)) or True,
        MouseEvent=native_mouse)
    try: pointer_namespace['pointer_text'](101,'Interface & recovery')
    except AssertionError as error:
        assert fault not in (None, 'preferences-scroll'), 'Visible recovery control rejected'
        assert not any(event in wire for event in (('mouse',2),('mouse',4))), ('Rejected target still received click', fault, wire)
        if fault != 'preferences-stuck': assert not wire, (fault,wire)
        if fault in ('preferences-covered', 'preferences-null'):
            message, rejected_label, evidence = error.args[0]
            assert (message, rejected_label) == ('Preferences scrolling surface covered', 'Interface & recovery')
            assert evidence['selected_target']['hwnd'] == 3
            assert evidence['viewport']['hwnd'] == 2
            assert evidence['surface']['hwnd'] == 1
            assert evidence['selected_target']['rect'] == [700,170,1040,242]
            assert evidence['viewport']['rect'] == [680,180,1060,500]
            assert evidence['surface']['rect'] == [0,0,1280,800]
            assert evidence['current_foreground']['hwnd'] == 1
            assert evidence['current_foreground']['native_class'] == 'wxWindowNR'
            assert evidence['current_foreground']['caption'] == 'OpenNav preferences'
            assert evidence['wheel_point'] == [870,340]
            assert evidence['descendant_relationship']['target_is_viewport_descendant'] is True
            assert evidence['descendant_relationship']['viewport_is_surface_descendant'] is True
            if fault == 'preferences-covered':
                assert evidence['hit']['hwnd'] == 99
                assert evidence['hit']['native_class'] == 'wxWindowNR'
                assert evidence['hit']['caption'] == 'OpenNav preferences'
                assert evidence['hit']['rect'] is None
                assert evidence['descendant_relationship']['hit_is_viewport'] is False
                assert evidence['descendant_relationship']['hit_is_viewport_descendant'] is False
            else:
                assert evidence['hit'] == {'hwnd': 0, 'native_class': '', 'caption': '', 'rect': None}
    else:
        assert fault in (None,'preferences-scroll'), ('Invalid recovery pointer target accepted', fault)
        assert wire[-3:] == [('cursor',870,256),('mouse',2),('mouse',4)]
        assert wire.count(('mouse',0x0800)) == int(preferences)
print('12 pointer recovery visibility/occlusion/identity/scroll guards passed without HWND command injection')

# Foreground activation is asynchronous across Win32 input queues. Exercise the
# Preferences scroll branch with delayed activation and retain strict refusal
# when activation is denied, an overlay appears, or the viewport moves.
scroll_cases = ('delayed-activation', 'activation-denied', 'activation-overlay',
                'activation-moved-viewport')
for scroll_case in scroll_cases:
    clock = Clock()
    wire = []
    front = [99]
    activation_due = [None]
    moved = [False]
    bounds = {1:(0,0,1280,800), 2:(680,180,1060,500), 3:(700,170,1040,242)}
    parents = {3:2, 2:1}
    captions = [(3, 'Interface & recovery')]
    queried = [3]

    def native_class(handle, buffer, capacity):
        buffer.value = 'wxWindowNR'
        return len(buffer.value)

    def native_hit(point):
        if scroll_case == 'activation-overlay':
            return 99
        return 2 if point.y == 340 else 3

    def native_rect(handle, output):
        if handle == 2 and moved[0]:
            value = (700, 180, 1060, 500)
        else:
            value = bounds.get(handle)
        if value is None:
            return False
        output = C.cast(output, C.POINTER(W.RECT)).contents
        output.left, output.top, output.right, output.bottom = value
        return True

    def current_front():
        if activation_due[0] is not None and clock.now >= activation_due[0]:
            front[0] = 1
        return front[0]

    def activate(handle):
        if scroll_case == 'delayed-activation':
            activation_due[0] = clock.now + .15
        elif scroll_case == 'activation-moved-viewport':
            front[0] = 1
            moved[0] = True
        elif scroll_case == 'activation-overlay':
            front[0] = 1
        # activation-denied deliberately leaves the unrelated foreground HWND.

    def native_mouse(event, *args):
        wire.append(('mouse', event))
        if event == 0x0800:
            bounds[3] = (700,220,1040,292)

    pointer_namespace.update(
        time=clock, windows=lambda pid:[(1,pid,'OpenNav X / OpenCPN')],
        children=lambda root:captions, IsWindowEnabled=lambda h:True,
        text=lambda h:'OpenNav preferences' if h == 1 else '',
        GetClassNameW=native_class, GetWindowRect=native_rect,
        GetParent=lambda h:parents.get(h), IsChild=is_child,
        WindowFromPoint=native_hit,
        declare=lambda dll,name,*args: current_front if name == 'GetForegroundWindow'
        else (lambda h,flag: 1),
        user=None, SetForegroundWindow=lambda h:activate(h),
        SetCursorPos=lambda x,y:wire.append(('cursor',x,y)) or True,
        MouseEvent=native_mouse)
    try:
        pointer_namespace['pointer_text'](101, 'Interface & recovery')
    except AssertionError:
        assert scroll_case != 'delayed-activation', scroll_case
        assert not wire, (scroll_case, wire)
    else:
        assert scroll_case == 'delayed-activation', scroll_case
        assert ('mouse', 0x0800) in wire
        assert ('mouse', 2) in wire and ('mouse', 4) in wire
print('4 delayed-foreground Preferences scroll activation guards passed without HWND command injection')
