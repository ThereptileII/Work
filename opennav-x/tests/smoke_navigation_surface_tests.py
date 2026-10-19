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
             and n.name in ('click_text', 'complete_modal_action', 'control_text')]
assert len(functions) == 3
name_check = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                  and n.name == 'require_native_waypoint_name')
wait_object = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                   and n.name == 'wait_object')
cancel_call = next(n for n in ast.walk(tree) if isinstance(n, ast.Expr)
                  and isinstance(n.value, ast.Call)
                  and isinstance(n.value.func, ast.Attribute)
                  and n.value.func.attr == 'complete_modal_action'
                  and len(n.value.args) >= 2
                  and isinstance(n.value.args[1], ast.Constant)
                  and n.value.args[1].value == 'Delete waypoint')
block = next(n.body for n in ast.walk(tree) if isinstance(n, ast.If)
             and cancel_call in n.body)
start = block.index(cancel_call) - 1
stop = next(i for i in range(start + 2, len(block))
            if isinstance(block[i], ast.Expr) and isinstance(block[i].value, ast.Call)
            and isinstance(block[i].value.func, ast.Attribute)
            and block[i].value.func.attr == 'click_text'
            and len(block[i].value.args) == 2
            and isinstance(block[i].value.args[1], ast.Constant)
            and block[i].value.args[1].value == 'Edit waypoint')
# Execute the real span through the next Edit, including publication wait and
# exact native-name observation; its number of statements is not the contract.
sequence = block[start:stop + 1]


class ModalQueue:
    def __init__(self, fault=None):
        self.fault = fault
        self.now = 0
        self.phase = None
        self.modal = False
        self.enabled = True
        self.calls = []
        self.name = 'ALPHA TEST UI edited'
        self.tick = 0
        self.published_enabled = False

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
        # HWND owner release precedes the next diagnostic publication. A stale
        # record still describes disabled controls until this separate event.
        if self.phase >= 4 and self.fault != 'publication-stuck':
            self.tick += 1
            self.published_enabled = self.enabled

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
        return 10

    def parent(self, handle):
        if handle == 2 and self.fault == 'wrong-owner':
            return 99
        return {2: 1, 3: 1, 4: 1, 10: 1, 20: 1, 22: 20, 23: 22, 21: 23}.get(handle, 0)

    def is_child(self, parent, handle):
        while handle:
            handle = self.parent(handle)
            if handle == parent:
                return True
        return False

    def children(self, handle):
        page = [(20, 'SKAGER product page: Waypoint detail'), (22, ''), (23, ''), (21, '')]
        # The unchanged cached caption intentionally cannot witness the EDIT's
        # value. Even a changed mark retains this misleading background text.
        if handle == 1:
            return [(10, 'ALPHA TEST UI edited'), (3, 'Delete waypoint'), (4, 'Edit waypoint')] + page
        return page[1:] if handle == 20 else []

    def send(self, handle, message, wparam=0, lparam=0):
        if message == 0x000D:  # The actual control_text helper sends WM_GETTEXT.
            assert handle == 21
            self.calls.append('read-name')
            value = C.create_unicode_buffer(self.name)
            assert len(self.name) < wparam
            C.memmove(lparam, value, C.sizeof(value))
            return len(self.name)
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

    def snapshot(self):
        field = dict(label='Field: Waypoint name', accessible_name='Waypoint name',
                     x=100, y=200, width=500, height=48, visible=True,
                     enabled=self.published_enabled)
        return dict(ui_page='Waypoint detail', runtime=dict(
            ui_update=dict(ticks=self.tick), display=dict(interaction_controls=[field])))


def run_cancel(fault=None, *, legacy=False, without_completion=False,
               without_fresh_wait=False, expected=None):
    queue = ModalQueue(fault)
    def class_name(handle, output, capacity):
        output.value = 'Edit' if handle == 21 else 'Static' if handle == 10 else 'wxWindowNR'
        return len(output.value)
    def rect(handle, output):
        r = C.cast(output, C.POINTER(W.RECT)).contents
        bounds = {1: (0, 0, 1280, 800), 20: (80, 68, 1094, 766),
                  21: (100, 200, 600, 248)}
        r.left, r.top, r.right, r.bottom = bounds.get(handle, (0, 0, 48, 48))
        return True
    native = dict(C=C, W=W, time=queue, windows=queue.windows,
                  wait_window=lambda title, pid: (2, pid),
                  GetParent=queue.parent, IsChild=queue.is_child, GetWindowThreadProcessId=queue.pid,
                  IsWindowVisible=queue.visible, IsWindowEnabled=queue.is_enabled,
                  children=queue.children, text=lambda h: 'SKAGER / OpenCPN',
                  GetClassNameW=class_name, GetClientRect=rect, GetWindowRect=rect,
                  SendMessageW=queue.send)
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(ui_path), 'exec'), native)
    code = copy.deepcopy(sequence)
    if legacy:
        # Historical negative control: cached background text allowed Edit to
        # be injected while the queued Cancel still held the owner disabled.
        unchanged = ast.parse("assert any(c == 'ALPHA TEST UI edited' for _, c in ui.children(handle))").body[0]
        code = [code[0], code[1], unchanged, code[-1]]
    if without_completion or legacy:
        code[1].value = copy.deepcopy(cancel_call.value.args[3].body)
    if without_fresh_wait:
        assignment = next(n for n in code if isinstance(n, ast.Assign)
                          and any(isinstance(target, ast.Name) and target.id == 'cancelled'
                                  for target in n.targets))
        assignment.value = ast.parse('object_snapshot()', mode='eval').body
    scope = dict(ui=SimpleNamespace(**native), app=SimpleNamespace(pid=77, poll=lambda: None),
                 handle=1, click_object=queue.cancel, time=queue,
                 object_snapshot=queue.snapshot, report={'native_waypoint_name': []})
    exec(compile(ast.Module(body=[name_check, wait_object], type_ignores=[]), str(smoke_path), 'exec'), scope)
    if expected is None:
        expected = 'refused' if fault or without_fresh_wait else 'edit-dropped' if legacy else 'edit-opened'
    try:
        exec(compile(ast.fix_missing_locations(ast.Module(body=code, type_ignores=[])),
                     str(smoke_path), 'exec'), scope)
    except (AssertionError, RuntimeError, ValueError) as error:
        assert expected == 'refused', error
        assert 'edit-opened' not in queue.calls and 'edit-dropped' not in queue.calls
        assert queue.calls.count('physical-cancel') <= 1
        if fault == 'action-error':
            assert isinstance(error, ValueError) and str(error) == 'original pointer failure'
        if fault == 'cancel-changed-mark':
            assert queue.calls == ['physical-cancel', 'read-name'], queue.calls
        if fault == 'publication-stuck' or without_fresh_wait:
            assert queue.calls == ['physical-cancel'], queue.calls
    else:
        assert expected != 'refused', ('Invalid modal completion accepted', fault)
        assert queue.calls == (['physical-cancel', expected] if legacy else
                               ['physical-cancel', 'read-name', expected]), queue.calls
        if not legacy:
            assert queue.tick > 0 and queue.phase >= 4
            observation, = scope['report']['native_waypoint_name']
            assert observation['stage'] == 'after-delete-cancel'
            assert observation['value'] == 'ALPHA TEST UI edited'
            assert observation['reader'] == 'WM_GETTEXT' and observation['edit_handle'] == 21
    return queue


run_cancel(legacy=True)
run_cancel()
for fault in ('cancel-stuck', 'modal-still-visible', 'owner-still-disabled',
              'cancel-changed-mark', 'duplicate', 'modal-disabled', 'not-modal',
              'wrong-pid', 'wrong-owner', 'owner-disappeared', 'owner-pid-changed',
              'action-error', 'missing-modal', 'initial-modal-hidden', 'initial-owner-hidden'):
    run_cancel(fault)
print('17 modal cancellation sequencing/ownership/failure checks passed; original sequence drops Edit')
run_cancel('publication-stuck')
run_cancel(without_fresh_wait=True)
# Removing owner completion lets an unrelated modal pass once fresh diagnostics
# arrive. The unmodified call site above refuses this same wrong-owner fixture.
run_cancel('wrong-owner', without_completion=True, expected='edit-opened')
print('3 fresh-publication and missing-completion negative controls passed')
