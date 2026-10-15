"""Focused source contract for the native 1280x800 navigation surface."""
import ast
import copy
import ctypes as C
from ctypes import wintypes as W
from pathlib import Path
from types import SimpleNamespace


root = Path(__file__).resolve().parents[1]
smoke_path = root / 'tools/smoke-navigation.py'
smoke_source = smoke_path.read_text()
tree = ast.parse(smoke_source)
function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                and node.name == 'exact_native_reference_desktop')
namespace = {}
exec(compile(ast.Module(body=[function], type_ignores=[]), str(smoke_path), 'exec'), namespace)
is_reference = namespace['exact_native_reference_desktop']

assert is_reference({'after': (1280, 800, 32)})
for display in ({}, {'after': ()}, {'after': (1280,)},
                {'after': (1280, 799, 32)}, {'after': (1279, 800, 32)},
                {'after': (1920, 1080, 32)}):
    assert not is_reference(display), display

print('7 exact native reference desktop selection checks passed')

# Exercise the actual cancellation call site and native helper against separate
# mouse-up, queued button command and owner-release events. A name already on
# the background page is not evidence that Cancel has finished.
ui_path = root / 'tools/windows-ui.py'
ui_tree = ast.parse(ui_path.read_text())
functions = [n for n in ui_tree.body if isinstance(n, ast.FunctionDef)
             and n.name in ('click_text', 'complete_modal_action')]
assert len(functions) == 2
cancel_call = next(n for n in ast.walk(tree) if isinstance(n, ast.Expr)
                  and isinstance(n.value, ast.Call)
                  and isinstance(n.value.func, ast.Attribute)
                  and n.value.func.attr == 'complete_modal_action')
block = next(n.body for n in ast.walk(tree) if isinstance(n, ast.If)
             and cancel_call in n.body)
start = block.index(cancel_call) - 1
sequence = block[start:start + 4]  # Delete, completed Cancel, unchanged name, Edit.
assert isinstance(sequence[2], ast.Assert)
assert sequence[3].value.args[1].value == 'Edit waypoint'


class ModalQueue:
    def __init__(self, fault=None):
        self.fault = fault
        self.now = 0
        self.phase = None
        self.modal = False
        self.enabled = True
        self.calls = []
        self.name = 'ALPHA TEST UI edited'

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        if self.phase is None or self.fault == 'cancel-stuck':
            return
        self.phase += 1
        # One turn consumes native mouse-up, the next the posted wxEVT_BUTTON,
        # the third releases the frame after EndModal. No desired-edit polling.
        if self.phase >= 2 and self.fault != 'modal-still-visible':
            self.modal = False
        if self.phase >= 3 and self.fault != 'owner-still-disabled':
            self.enabled = True
        if self.phase >= 3 and self.fault == 'cancel-changed-mark':
            self.name = 'incorrectly changed'

    def windows(self, pid):
        return [(1, pid, 'SKAGER / OpenCPN')] + (
            [(2, pid, 'Delete waypoint')] * (2 if self.fault == 'duplicate' else 1)
            if self.modal else [])

    def visible(self, handle):
        if handle == 2:
            return self.modal and self.fault != 'initial-modal-hidden'
        if self.fault == 'initial-owner-hidden':
            return False
        return not (self.fault == 'owner-disappeared' and self.phase is not None)

    def is_enabled(self, handle):
        if handle == 2:
            return self.fault != 'modal-disabled'
        return self.enabled or self.fault == 'not-modal'

    def pid(self, handle, output):
        changed = self.fault == 'wrong-pid' or (
            self.fault == 'owner-pid-changed' and self.phase is not None)
        C.cast(output, C.POINTER(W.DWORD)).contents.value = 99 if changed else 77

    def parent(self, handle):
        if handle == 2 and self.fault == 'wrong-owner':
            return 99
        return 1 if handle in (2, 3, 4) else 0

    def children(self, handle):
        return [(10, self.name), (3, 'Delete waypoint'), (4, 'Edit waypoint')] if handle == 1 else []

    def send(self, handle, message, *_):
        if message == 0x202:
            if handle == 3:
                self.modal = self.fault != 'missing-modal'
                self.enabled = False
            elif handle == 4:
                self.calls.append('edit-opened' if self.enabled else 'edit-dropped')
        return 1

    def cancel(self, label, accessible_name=None):
        assert (label, accessible_name) == ('Cancel', 'Cancel')
        self.calls.append('physical-cancel')
        if self.fault == 'action-error':
            raise ValueError('original pointer failure')
        self.phase = 0


def run_cancel(fault=None, without_completion=False):
    queue = ModalQueue(fault)
    def class_name(handle, output, capacity):
        output.value = 'wxWindowNR'
    def rect(handle, output):
        r = C.cast(output, C.POINTER(W.RECT)).contents
        r.left = r.top = 0
        r.right = r.bottom = 48
        return True
    native = dict(C=C, W=W, time=queue, windows=queue.windows,
                  wait_window=lambda title, pid: (2, pid),
                  GetParent=queue.parent, GetWindowThreadProcessId=queue.pid,
                  IsWindowVisible=queue.visible, IsWindowEnabled=queue.is_enabled,
                  children=queue.children, text=lambda h: 'SKAGER / OpenCPN',
                  GetClassNameW=class_name, GetClientRect=rect, SendMessageW=queue.send)
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(ui_path), 'exec'), native)
    code = copy.deepcopy(sequence)
    if without_completion:
        # This is the original call sequence: queue Cancel then immediately
        # inspect unchanged background text and synchronously inject Edit.
        code[1].value = copy.deepcopy(cancel_call.value.args[3].body)
    scope = dict(ui=SimpleNamespace(**native), app=SimpleNamespace(pid=77),
                 handle=1, click_object=queue.cancel)
    try:
        exec(compile(ast.fix_missing_locations(ast.Module(body=code, type_ignores=[])),
                     str(smoke_path), 'exec'), scope)
    except (AssertionError, RuntimeError, ValueError) as error:
        assert fault is not None, error
        assert 'edit-opened' not in queue.calls and 'edit-dropped' not in queue.calls
        assert queue.calls.count('physical-cancel') <= 1
        if fault == 'action-error':
            assert isinstance(error, ValueError) and str(error) == 'original pointer failure'
    else:
        assert fault is None, ('Invalid modal completion accepted', fault)
        expected = 'edit-dropped' if without_completion else 'edit-opened'
        assert queue.calls == ['physical-cancel', expected], queue.calls
    return queue


run_cancel(without_completion=True)
run_cancel()
for fault in ('cancel-stuck', 'modal-still-visible', 'owner-still-disabled',
              'cancel-changed-mark', 'duplicate', 'modal-disabled', 'not-modal',
              'wrong-pid', 'wrong-owner', 'owner-disappeared', 'owner-pid-changed',
              'action-error', 'missing-modal', 'initial-modal-hidden', 'initial-owner-hidden'):
    run_cancel(fault)
print('17 modal cancellation sequencing/ownership/failure checks passed; original sequence drops Edit')
