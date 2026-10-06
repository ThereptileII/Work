"""Explicit first-start UI handling for isolated navigation smoke profiles."""
import time

_SETUP_FIELDS = {'Field: Vessel name', 'Field: Draft · metres',
                 'Field: Chart safety depth · metres'}


def defer_boat_setup(read, click, *, timeout=8):
    """Use Later on the initial setup sheet; never write a completion marker.

    Call after deferred initialization and a fresh UI publication. The complete
    initial field/action signature avoids dismissing unrelated Later dialogs.
    """
    before = read()
    controls = before['runtime']['display']['interaction_controls']
    visible = [c for c in controls if c['visible']]
    labels = {c['label'] for c in visible}
    if not labels & _SETUP_FIELDS:
        return {'status': 'not-present'}
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
        if tick > before_tick and not remaining & (_SETUP_FIELDS | {'Later', 'Back', 'Continue'}):
            return {'status': 'deferred-with-Later', 'before_ticks': before_tick,
                    'after_ticks': tick}
        time.sleep(.1)
    raise AssertionError('Boat setup did not close after explicit Later; navigation assertions withheld')
