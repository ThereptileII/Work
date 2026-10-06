"""Explicit first-start UI handling for isolated navigation smoke profiles."""
import time

_SETUP_FIELDS = {'Field: Vessel name', 'Field: Draft · metres',
                 'Field: Chart safety depth · metres'}


def native_setup_window(ui, pid):
    """Witness the exact visible process-owned wxDialog, without activating it."""
    dialogs = [(handle, process) for handle, process, title in ui.windows(pid)
               if title == 'Boat Setup & Sensor Check']
    if not dialogs:
        return None
    assert len(dialogs) == 1 and dialogs[0][1] == pid, ('Ambiguous setup owner', dialogs)
    handle = dialogs[0][0]
    name = ui.C.create_unicode_buffer(128)
    assert ui.GetClassNameW(handle, name, len(name)) and name.value == '#32770', (
        'Expected the owned native setup dialog', name.value)
    bounds = ui.W.RECT()
    assert ui.GetWindowRect(handle, ui.C.byref(bounds))
    assert bounds.right > bounds.left and bounds.bottom > bounds.top
    return dict(x=bounds.left, y=bounds.top, width=bounds.right-bounds.left,
                height=bounds.bottom-bounds.top)


def _inside(control, bounds):
    return (bounds['x'] <= control['x'] and bounds['y'] <= control['y'] and
            control['x']+control['width'] <= bounds['x']+bounds['width'] and
            control['y']+control['height'] <= bounds['y']+bounds['height'])


def defer_boat_setup(read, click, *, native_window=None, timeout=8):
    """Use Later on the initial setup sheet; never write a completion marker.

    Call after deferred initialization and a fresh UI publication. On Windows,
    standard wxTextCtrl fields can be absent from the product diagnostic walk.
    An exact native dialog witness plus its initial action signature identifies
    that sheet without accepting an unrelated Later dialog.
    """
    before = read()
    controls = before['runtime']['display']['interaction_controls']
    visible = [c for c in controls if c['visible']]
    labels = {c['label'] for c in visible}
    witness = native_window() if native_window else None
    if witness is not None:
        actions = {label: [c for c in visible if c['label'] == label and _inside(c, witness)]
                   for label in ('Later', 'Back', 'Continue')}
        assert all(len(items) == 1 for items in actions.values()), (
            'Owned initial setup actions are incomplete or ambiguous', actions)
        assert not actions['Back'][0]['enabled'] and actions['Continue'][0]['enabled'], (
            'Expected the initial setup step', actions)
        later = actions['Later']
    else:
        if not labels & _SETUP_FIELDS:
            assert not {'Later', 'Back', 'Continue'} <= labels, (
                'Setup-like actions lack a verified initial sheet', visible)
            return {'status': 'not-present'}
        assert native_window is None, 'Setup diagnostics lack the expected native window'
        assert _SETUP_FIELDS | {'Later', 'Back', 'Continue'} <= labels, (
            'First-start setup is incomplete; cannot safely defer it', visible)
        later = [c for c in visible if c['label'] == 'Later']
    assert len(later) == 1 and later[0]['enabled'], ('Unique enabled setup Later required', later)
    target = later[0]
    assert target['width'] >= 48 and target['height'] >= 48, target
    before_tick = int(before['runtime']['ui_update']['ticks'])
    click(target)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        after = read()
        tick = int(after['runtime']['ui_update']['ticks'])
        remaining = {c['label'] for c in after['runtime']['display']['interaction_controls'] if c['visible']}
        closed = native_window() is None if native_window else True
        if closed and tick > before_tick and not remaining & (_SETUP_FIELDS | {'Later', 'Back', 'Continue'}):
            return {'status': 'deferred-with-Later', 'before_ticks': before_tick,
                    'after_ticks': tick}
        time.sleep(.1)
    raise AssertionError('Boat setup did not close after explicit Later; navigation assertions withheld')
